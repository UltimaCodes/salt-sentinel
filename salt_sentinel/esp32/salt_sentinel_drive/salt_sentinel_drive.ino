/*
 * Salt Sentinel - ESP32 drive controller (rev 4)
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
 * TWO command sources, arbitrated automatically - no explicit
 * "give back control" command needed:
 *
 *   USB Serial (the Pi), line-based ASCII, 115200 8N1:
 *     D <left> <right>   drive, -1000..1000 each
 *     E <0|1>            enable / disable the driver outputs
 *     P                  ping -> OK
 *     ?                  force an immediate telemetry line
 *   Replies: OK | ERR <reason> | ST <telemetry> | EV <event>
 *
 *   Bluetooth Classic ("thepretender"), single ASCII char, for a
 *   gamepad-style BT RC app - manual override for testing/demo:
 *     F/B/L/R/X          forward / backward / pivot left / pivot right / stop
 *   PIN-locked pairing (BT_PIN below) - the radio is on and visible from
 *   boot regardless of whether anything's connected, but a stranger's phone
 *   can't actually pair without entering the right PIN first. Change it
 *   before a demo if you're worried it's been seen.
 *
 * Arbitration: a BT command always takes effect immediately. A Serial (Pi)
 * D command only takes effect if no BT command has arrived in the last
 * BT_OVERRIDE_MS - so the Pi automatically regains control BT_OVERRIDE_MS
 * after a human lets go, with nothing extra to send on either side. Pi's
 * E (enable/disable) always applies regardless of BT state - that's the
 * supervisory safety path, not a driving command, and should never be
 * blocked by someone mid-override. Both sources feed the same watchdog:
 * silence from EITHER one for WATCHDOG_MS doesn't matter as long as the
 * OTHER is still talking, but silence from BOTH stops the motors.
 */

#include <Arduino.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error "Bluetooth Classic not enabled in this build - Tools > Board must be an ESP32 (not ESP32-C3/S2, which lack Classic BT)"
#endif

BluetoothSerial SerialBT;

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

// How long a BT command keeps overriding the Pi after the last one arrives.
static const uint32_t BT_OVERRIDE_MS = 5000;
static const float    BT_SPEED_FRAC  = 0.5f;  // manual-drive speed, tune to taste

// Required to pair - forces legacy PIN pairing instead of ESP32's default
// "Just Works" mode, which lets anyone in range pair with no prompt at all.
static const char *BT_PIN = "727";

// ----------------------------------------------------------------- state
struct Chan { float cur = 0; float tgt = 0; };
static Chan chL, chR;

static bool     enabled    = false;
static uint32_t lastCmdMs  = 0;      // fed by EITHER source - shared watchdog
static uint32_t lastBtMs   = 0;
static uint32_t lastTickMs = 0;
static uint32_t lastTeleMs = 0;

static bool btOverrideActive() { return (millis() - lastBtMs) < BT_OVERRIDE_MS; }

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
      // A human on Bluetooth wins for BT_OVERRIDE_MS after their last
      // command - the Pi's own drive value is simply not applied. lastCmdMs
      // is still updated above, so this doesn't trip the shared watchdog.
      if (!btOverrideActive()) {
        chL.tgt = (l / 1000.0f) * PWM_MAX;
        chR.tgt = (r / 1000.0f) * PWM_MAX;
      }
      Serial.println("OK");
      break;
    }
    case 'E': {
      // Always applies, even mid BT-override - this is the supervisory
      // safety path, not a driving command.
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

// Bluetooth: single-char gamepad commands, always take effect immediately.
static void handleBt(char c) {
  lastCmdMs = millis();
  lastBtMs = millis();
  enabled = true;
  float spd = BT_SPEED_FRAC * PWM_MAX;
  switch (c) {
    case 'F': chL.tgt = spd;  chR.tgt = spd;  break;
    case 'B': chL.tgt = -spd; chR.tgt = -spd; break;
    case 'L': chL.tgt = -spd; chR.tgt = spd;  break;
    case 'R': chL.tgt = spd;  chR.tgt = -spd; break;
    case 'X': default: chL.tgt = 0; chR.tgt = 0; break;
  }
}

// ----------------------------------------------------------------- setup
void setup() {
  Serial.begin(115200);
  SerialBT.setPin(BT_PIN, strlen(BT_PIN));   // must be called before begin()
  SerialBT.begin("thepretender");

  pwmInit(PIN_L_RPWM, PWM_FREQ, PWM_BITS);
  pwmInit(PIN_L_LPWM, PWM_FREQ, PWM_BITS);
  pwmInit(PIN_R_RPWM, PWM_FREQ, PWM_BITS);
  pwmInit(PIN_R_LPWM, PWM_FREQ, PWM_BITS);
  allStop("boot");

  Serial.println("EV READY salt-sentinel-drive v4");
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

  while (SerialBT.available()) {
    char c = (char)SerialBT.read();
    if (c != '\n' && c != '\r') handleBt(c);
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
    Serial.print(" bt=");     Serial.print(btOverrideActive() ? 1 : 0);
    Serial.println();
  }
}
