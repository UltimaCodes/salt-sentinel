/*
 * Salt Sentinel - ESP32 drive controller (rev 2)
 *
 * Drives the 4 gearmotors through the BTS7960 drivers (both left motors
 * share one driver pair, both right share the other) and holds the
 * pan/tilt servos. Hardware e-stop and rain sensors removed in rev 2 -
 * the 500ms serial watchdog is now the only automatic stop, duty cap
 * scales against measured pack voltage so 12V motors don't see 16.8V,
 * and MOT_EN has an external pulldown so motors are off before boot.
 *
 * Protocol: line-based ASCII, 115200 8N1.
 *   D <left> <right>   drive, -1000..1000 each
 *   E <0|1>            enable / disable the driver outputs
 *   S <pan> <tilt>     servo pulse width in microseconds
 *   P                  ping -> OK
 *   ?                  force an immediate telemetry line
 * Replies: OK | ERR <reason> | ST <telemetry> | EV <event>
 */

#include <Arduino.h>

#define USE_ENCODERS 1   // set 0 if the motors are the 2-wire variant

// Is the 100k/22k pack-voltage divider fitted to GPIO 34?
// 0 = no divider, duty cap fixed at the full-pack assumption (safe, rover
// slows as pack drains). 1 = divider fitted, true 12V held down to 16.8V.
// Leave at 0 until the resistors are soldered - a floating ADC next to
// four 20kHz drivers reads noise, and noise in the plausible window gets
// treated as real, which overvolts the motors while believing it's safe.
#define HAVE_VPACK_SENSE 0

// ----------------------------------------------------------------- pins
static const int PIN_L_RPWM = 25;
static const int PIN_L_LPWM = 26;
static const int PIN_R_RPWM = 27;
static const int PIN_R_LPWM = 14;

static const int PIN_MOT_EN     = 23;   // -> all 4 BTS7960 R_EN/L_EN pins
                                        // 10k EXTERNAL PULLDOWN REQUIRED
static const int PIN_SERVO_PAN  = 19;
static const int PIN_SERVO_TILT = 21;
static const int PIN_VPACK      = 34;   // ADC1, 100k/22k divider

#if USE_ENCODERS
static const int PIN_ENC_L = 39;
static const int PIN_ENC_R = 32;
#endif

// ----------------------------------------------------------------- config
static const int      PWM_FREQ   = 20000;    // above audible
static const int      PWM_BITS   = 10;       // 0..1023
static const int      PWM_MAX    = (1 << PWM_BITS) - 1;
static const int      SERVO_FREQ = 50;
static const int      SERVO_BITS = 16;

static const uint32_t WATCHDOG_MS   = 500;
static const float    MOTOR_V_NOM   = 12.0f;
static const float    DIV_RATIO     = 22.0f / (100.0f + 22.0f);
static const float    RAMP_PER_TICK = 25.0f; // duty units per 10 ms -> ~0.4 s to full
static const uint32_t TICK_MS       = 10;

static const uint16_t SERVO_MIN_US = 600;
static const uint16_t SERVO_MAX_US = 2400;
static const uint16_t SERVO_BACKLASH_US = 60;

// ----------------------------------------------------------------- state
struct Chan { float cur = 0; float tgt = 0; };
static Chan chL, chR;

static bool     enabled    = false;
static uint32_t lastCmdMs  = 0;
static uint32_t lastTickMs = 0;
static uint32_t lastTeleMs = 0;
static uint16_t panUs = 1500, tiltUs = 1500;

#if USE_ENCODERS
static volatile uint32_t encL = 0, encR = 0;
static void IRAM_ATTR isrEncL() { encL++; }
static void IRAM_ATTR isrEncR() { encR++; }
#endif

// ----------------------------------------------------------------- LEDC shim
// Arduino-ESP32 core 3.x replaced ledcSetup/ledcAttachPin with ledcAttach.
static void pwmInit(int pin, int freq, int bits) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(pin, freq, bits);
#else
  static int nextCh = 0;
  int ch = nextCh++;
  ledcSetup(ch, freq, bits);
  ledcAttachPin(pin, ch);
#endif
}

static void pwmWrite(int pin, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  static const int pins[] = {PIN_L_RPWM, PIN_L_LPWM, PIN_R_RPWM, PIN_R_LPWM,
                             PIN_SERVO_PAN, PIN_SERVO_TILT};
  for (int i = 0; i < 6; i++) if (pins[i] == pin) { ledcWrite(i, duty); return; }
#endif
}

// ----------------------------------------------------------------- helpers
static float packVolts() {
  uint32_t s[5];
  for (int i = 0; i < 5; i++) s[i] = analogReadMilliVolts(PIN_VPACK);
  for (int i = 1; i < 5; i++)
    for (int j = i; j > 0 && s[j] < s[j - 1]; j--) {
      uint32_t t = s[j]; s[j] = s[j - 1]; s[j - 1] = t;
    }
  return (s[2] / 1000.0f) / DIV_RATIO;
}

// Plausible range for a 4S pack in service. Outside this, the divider is
// broken - not the battery.
static const float VPACK_MIN_VALID = 9.0f;
static const float VPACK_MAX_VALID = 21.0f;
static const float SAFE_CAP = MOTOR_V_NOM / 16.8f;   // assume a full pack

static bool vpackFault = false;

static float dutyCap() {
#if !HAVE_VPACK_SENSE
  // No divider fitted: never trust GPIO 34, assume a full pack, cap hard.
  return SAFE_CAP;
#else
  float v = packVolts();

  // fail safe: a disconnected divider reads ~0V, which would otherwise
  // hand the motors 100% duty from a 16.8V pack
  if (v < VPACK_MIN_VALID || v > VPACK_MAX_VALID) {
    if (!vpackFault) {
      vpackFault = true;
      Serial.println("EV FAULT vpack_sense_implausible - assuming full pack");
    }
    return SAFE_CAP;
  }
  vpackFault = false;

  if (v < MOTOR_V_NOM) return 1.0f;   // genuinely sagging: full duty is safe
  float c = MOTOR_V_NOM / v;
  return c < 0.35f ? 0.35f : c;
#endif
}

static void applyChannel(int pinFwd, int pinRev, float duty) {
  if (duty >= 0) { pwmWrite(pinRev, 0); pwmWrite(pinFwd, (uint32_t)duty); }
  else           { pwmWrite(pinFwd, 0); pwmWrite(pinRev, (uint32_t)(-duty)); }
}

static void allStop(const char *why) {
  chL.tgt = chL.cur = 0;
  chR.tgt = chR.cur = 0;
  pwmWrite(PIN_L_RPWM, 0); pwmWrite(PIN_L_LPWM, 0);
  pwmWrite(PIN_R_RPWM, 0); pwmWrite(PIN_R_LPWM, 0);
  enabled = false;
  digitalWrite(PIN_MOT_EN, LOW);
  Serial.print("EV STOP ");
  Serial.println(why);
}

static void writeServo(int pin, uint16_t us) {
  uint32_t duty = (uint32_t)((us / 20000.0f) * ((1UL << SERVO_BITS) - 1));
  pwmWrite(pin, duty);
}

// Always arrive at a target angle from the same side, so gear lash is taken
// up identically every time.
static void setServo(int pin, uint16_t &store, uint16_t us) {
  if (us < SERVO_MIN_US) us = SERVO_MIN_US;
  if (us > SERVO_MAX_US) us = SERVO_MAX_US;
  if (us > store) {
    uint16_t over = (us + SERVO_BACKLASH_US > SERVO_MAX_US)
                    ? SERVO_MAX_US : us + SERVO_BACKLASH_US;
    writeServo(pin, over);
    delay(120);
  }
  writeServo(pin, us);
  store = us;
}

// ----------------------------------------------------------------- commands
static void handle(char *line) {
  lastCmdMs = millis();
  switch (line[0]) {
    case 'D': {
      int l = 0, r = 0;
      if (sscanf(line + 1, "%d %d", &l, &r) != 2) { Serial.println("ERR parse"); return; }
      l = constrain(l, -1000, 1000);
      r = constrain(r, -1000, 1000);
      float cap = dutyCap() * PWM_MAX;
      chL.tgt = (l / 1000.0f) * cap;
      chR.tgt = (r / 1000.0f) * cap;
      Serial.println("OK");
      break;
    }
    case 'E': {
      int v = atoi(line + 1);
      enabled = v != 0;
      digitalWrite(PIN_MOT_EN, enabled ? HIGH : LOW);
      if (!enabled) allStop("cmd");
      Serial.println("OK");
      break;
    }
    case 'S': {
      int p = 0, t = 0;
      if (sscanf(line + 1, "%d %d", &p, &t) != 2) { Serial.println("ERR parse"); return; }
      setServo(PIN_SERVO_PAN,  panUs,  (uint16_t)p);
      setServo(PIN_SERVO_TILT, tiltUs, (uint16_t)t);
      Serial.println("OK");
      break;
    }
    case '?': lastTeleMs = 0; Serial.println("OK"); break;
    case 'P': Serial.println("OK"); break;
    default:  Serial.println("ERR unknown"); break;
  }
}

// ----------------------------------------------------------------- setup
void setup() {
  // MOT_EN low FIRST, before anything else gets a chance to float it high.
  pinMode(PIN_MOT_EN, OUTPUT);
  digitalWrite(PIN_MOT_EN, LOW);

  Serial.begin(115200);

  pwmInit(PIN_L_RPWM, PWM_FREQ, PWM_BITS);
  pwmInit(PIN_L_LPWM, PWM_FREQ, PWM_BITS);
  pwmInit(PIN_R_RPWM, PWM_FREQ, PWM_BITS);
  pwmInit(PIN_R_LPWM, PWM_FREQ, PWM_BITS);
  pwmInit(PIN_SERVO_PAN,  SERVO_FREQ, SERVO_BITS);
  pwmInit(PIN_SERVO_TILT, SERVO_FREQ, SERVO_BITS);
  allStop("boot");

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_VPACK, ADC_11db);

#if USE_ENCODERS
  pinMode(PIN_ENC_L, INPUT_PULLUP);
  pinMode(PIN_ENC_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L), isrEncL, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R), isrEncR, RISING);
#endif

  writeServo(PIN_SERVO_PAN, panUs);
  writeServo(PIN_SERVO_TILT, tiltUs);

  Serial.println("EV READY salt-sentinel-drive v2");
}

// ----------------------------------------------------------------- loop
void loop() {
  static char buf[64];
  static uint8_t n = 0;

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (n) { buf[n] = 0; handle(buf); n = 0; }
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = c;
    }
  }

  uint32_t now = millis();

  // With the e-stop removed this is the only automatic stop on the rover.
  if (enabled && (now - lastCmdMs) > WATCHDOG_MS) {
    allStop("watchdog");
  }

  if (now - lastTickMs >= TICK_MS) {
    lastTickMs = now;
    if (!enabled) { chL.tgt = chR.tgt = 0; }

    Chan *cs[2] = {&chL, &chR};
    for (int i = 0; i < 2; i++) {
      float d = cs[i]->tgt - cs[i]->cur;
      if (d >  RAMP_PER_TICK) d =  RAMP_PER_TICK;
      if (d < -RAMP_PER_TICK) d = -RAMP_PER_TICK;
      cs[i]->cur += d;
    }
    if (enabled) {
      applyChannel(PIN_L_RPWM, PIN_L_LPWM, chL.cur);
      applyChannel(PIN_R_RPWM, PIN_R_LPWM, chR.cur);
    } else {
      pwmWrite(PIN_L_RPWM, 0); pwmWrite(PIN_L_LPWM, 0);
      pwmWrite(PIN_R_RPWM, 0); pwmWrite(PIN_R_LPWM, 0);
    }
  }

  if (now - lastTeleMs >= 100) {
    lastTeleMs = now;
#if HAVE_VPACK_SENSE
    Serial.print("ST v=");  Serial.print(packVolts(), 2);
#else
    Serial.print("ST v=-1");   // no divider fitted
#endif
    Serial.print(" en=");   Serial.print(enabled ? 1 : 0);
    Serial.print(" L=");    Serial.print((int)chL.cur);
    Serial.print(" R=");    Serial.print((int)chR.cur);
    Serial.print(" vf="); Serial.print(vpackFault ? 1 : 0);
    Serial.print(" cap="); Serial.print(dutyCap(), 2);
    Serial.print(" pan=");  Serial.print(panUs);
    Serial.print(" tilt="); Serial.print(tiltUs);
#if USE_ENCODERS
    Serial.print(" encL="); Serial.print(encL);
    Serial.print(" encR="); Serial.print(encR);
#endif
    Serial.println();
  }
}
