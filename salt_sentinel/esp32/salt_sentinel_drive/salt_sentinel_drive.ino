/*
 * Salt Sentinel - ESP32 drive controller (rev 5)
 *
 * Drives the 4 gearmotors through the BTS7960 drivers (both left motors
 * share one driver pair, both right share the other). No servos - the
 * sensor arm is a static bracket now, not pan/tilt. No encoders - the
 * motors are the plain 2-wire variant. R_EN/L_EN on all 4 drivers are
 * hardwired straight to the 5V rail (drivers enabled whenever powered) -
 * not GPIO-controlled, so `enabled` below is a software-only gate that
 * holds RPWM/LPWM at 0, not a real driver-enable signal.
 *
 * No duty cap: the motor rail comes from a dedicated 7.2V/20A buck, not
 * raw pack voltage, so 100% PWM is always true rated voltage - there's no
 * pack-voltage-dependent overvoltage risk to cap against, and no divider
 * to sense it with even if there were.
 *
 * ONE command source: USB Serial from the Pi, line-based ASCII, 115200 8N1
 * (no Bluetooth override - cli.py's teleop already gives manual control
 * over this same link, so a second RC path was redundant):
 *
 *   D <left> <right>   drive, -1000..1000 each
 *   E <0|1>            enable / disable the driver outputs
 *   P                  ping -> OK
 *   ?                  force an immediate telemetry line
 * Replies: OK | ERR <reason> | ST <telemetry> | EV <event>
 *
 * Watchdog: silence from the Pi for WATCHDOG_MS stops the motors - with
 * the hardware e-stop removed, this is the only automatic stop the rover
 * has.
 */

#include <Arduino.h>

// ----------------------------------------------------------------- pins
static const int PIN_L_RPWM = 25;
static const int PIN_L_LPWM = 26;
static const int PIN_R_RPWM = 27;
static const int PIN_R_LPWM = 14;
// No PIN_MOT_EN - R_EN/L_EN are hardwired to the 5V rail, not to the ESP32.

// ----------------------------------------------------------------- config
static const int      PWM_FREQ   = 20000;    // above audible
static const int      PWM_BITS   = 10;       // 0..1023
static const int      PWM_MAX    = (1 << PWM_BITS) - 1;

static const uint32_t WATCHDOG_MS    = 500;
static const float    RAMP_PER_TICK  = 25.0f; // duty units per 10 ms -> ~0.4 s to full
static const uint32_t TICK_MS        = 10;

// ----------------------------------------------------------------- state
struct Chan { float cur = 0; float tgt = 0; };
static Chan chL, chR;

static bool     enabled    = false;
static uint32_t lastCmdMs  = 0;
static uint32_t lastTickMs = 0;
static uint32_t lastTeleMs = 0;

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
  static const int pins[] = {PIN_L_RPWM, PIN_L_LPWM, PIN_R_RPWM, PIN_R_LPWM};
  for (int i = 0; i < 4; i++) if (pins[i] == pin) { ledcWrite(i, duty); return; }
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
  Serial.print("EV STOP ");
  Serial.println(why);
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
      chL.tgt = (l / 1000.0f) * PWM_MAX;
      chR.tgt = (r / 1000.0f) * PWM_MAX;
      Serial.println("OK");
      break;
    }
    case 'E': {
      int v = atoi(line + 1);
      enabled = v != 0;
      if (!enabled) allStop("cmd");
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
  Serial.begin(115200);

  pwmInit(PIN_L_RPWM, PWM_FREQ, PWM_BITS);
  pwmInit(PIN_L_LPWM, PWM_FREQ, PWM_BITS);
  pwmInit(PIN_R_RPWM, PWM_FREQ, PWM_BITS);
  pwmInit(PIN_R_LPWM, PWM_FREQ, PWM_BITS);
  allStop("boot");

  Serial.println("EV READY salt-sentinel-drive v5");
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
    Serial.print("ST en=");   Serial.print(enabled ? 1 : 0);
    Serial.print(" L=");      Serial.print((int)chL.cur);
    Serial.print(" R=");      Serial.print((int)chR.cur);
    Serial.println();
  }
}
