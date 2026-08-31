/*
 * Salt Sentinel - DEMO-ONLY Bluetooth drive controller
 *
 * NOT the real drive firmware (that's salt_sentinel_drive.ino, USB serial
 * from the Pi). Flash this one only for filming, then reflash the real one.
 *
 * Pair to "SaltSentinelDemo" with a gamepad-style Bluetooth RC app and send:
 *   F  forward   B  backward   X  stop (also anything unrecognised)
 * No L/R - all 4 drivers share ONE RPWM and ONE LPWM signal now, so there's
 * no way to turn one side independently. F/B/X is all this rig does.
 * C/T/S/A/P are accepted but ignored.
 *
 * ONE RPWM + ONE LPWM bus to all 4 drivers' RPWM/LPWM pins in parallel -
 * on GPIO25/26 specifically, because those are the pins already confirmed
 * working (that's what was spinning the one good wheel). If the other
 * boards' problem was the GPIO27/14 signal path rather than the boards
 * themselves, this fixes it as a side effect.
 *
 * No software duty cap: the 7.2V/20A buck regulates the motor rail in
 * hardware now, so 100% PWM is true rated voltage, not overvoltage.
 *
 * R_EN/L_EN are NOT wired to the ESP32 on this build - hardwired straight
 * to the 5V logic rail instead (drivers enabled whenever powered).
 *
 * NO WATCHDOG: F/B persist until you send something else - most
 * gamepad-style BT apps send a button's character once per press, not as
 * a repeated stream while held, so a keepalive-style timeout stops the
 * motors mid-hold. Fine for a supervised demo with a human right there
 * and a physical switch as the real stop; send X when you want it to stop.
 */

#include <Arduino.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error "Bluetooth Classic not enabled in this build - Tools > Board must be an ESP32 (not ESP32-C3/S2, which lack Classic BT)"
#endif

BluetoothSerial SerialBT;

// ----------------------------------------------------------------- pins
static const int PIN_RPWM = 25;   // buses to all 4 drivers' RPWM
static const int PIN_LPWM = 26;   // buses to all 4 drivers' LPWM
// No PIN_MOT_EN - R_EN/L_EN are hardwired to VCC, not to the ESP32.
static const int PIN_SERVO_PAN  = 19;
static const int PIN_SERVO_TILT = 21;

// ----------------------------------------------------------------- config
static const int      PWM_FREQ   = 20000;
static const int      PWM_BITS   = 10;
static const int      PWM_MAX    = (1 << PWM_BITS) - 1;
static const int      SERVO_FREQ = 50;
static const int      SERVO_BITS = 16;

static const float    RAMP_PER_TICK = 25.0f;
static const uint32_t TICK_MS       = 10;

// Cruise speed for filming, not a safety limit - raise toward PWM_MAX for more speed.
static const float DEMO_SPEED_FRAC = 0.75f;
static const int   DEMO_SPEED = (int)(DEMO_SPEED_FRAC * PWM_MAX);

static const uint16_t SERVO_CENTER_US = 1500;   // parked here once at boot, never moved

// ----------------------------------------------------------------- state
static float    cur = 0, tgt = 0;
static bool     enabled = false;
static uint32_t lastTickMs = 0;

// ----------------------------------------------------------------- LEDC shim
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
  static const int pins[] = {PIN_RPWM, PIN_LPWM, PIN_SERVO_PAN, PIN_SERVO_TILT};
  for (int i = 0; i < 4; i++) if (pins[i] == pin) { ledcWrite(i, duty); return; }
#endif
}

static void applyDrive(float duty) {
  if (duty >= 0) { pwmWrite(PIN_LPWM, 0); pwmWrite(PIN_RPWM, (uint32_t)duty); }
  else           { pwmWrite(PIN_RPWM, 0); pwmWrite(PIN_LPWM, (uint32_t)(-duty)); }
}

static void allStop() {
  tgt = cur = 0;
  pwmWrite(PIN_RPWM, 0); pwmWrite(PIN_LPWM, 0);
  enabled = false;
}

static void writeServo(int pin, uint16_t us) {
  uint32_t duty = (uint32_t)((us / 20000.0f) * ((1UL << SERVO_BITS) - 1));
  pwmWrite(pin, duty);
}

// ----------------------------------------------------------------- commands
static void handle(char c) {
  enabled = true;
  switch (c) {
    case 'F':
      tgt = (float)DEMO_SPEED;
      break;
    case 'B':
      tgt = -(float)DEMO_SPEED;
      break;
    case 'X':
      tgt = 0;
      break;
    case 'C': case 'T': case 'S': case 'A': case 'P':
      // accepted, no-op - this rig doesn't need them for drive footage
      break;
    default:
      tgt = 0;
      break;
  }
}

// ----------------------------------------------------------------- setup
void setup() {
  Serial.begin(115200);
  SerialBT.begin("SaltSentinelDemo");
  Serial.println("Bluetooth demo controller ready - pair to SaltSentinelDemo");

  pwmInit(PIN_RPWM, PWM_FREQ, PWM_BITS);
  pwmInit(PIN_LPWM, PWM_FREQ, PWM_BITS);
  pwmInit(PIN_SERVO_PAN,  SERVO_FREQ, SERVO_BITS);
  pwmInit(PIN_SERVO_TILT, SERVO_FREQ, SERVO_BITS);
  allStop();
  writeServo(PIN_SERVO_PAN, SERVO_CENTER_US);
  writeServo(PIN_SERVO_TILT, SERVO_CENTER_US);
}

// ----------------------------------------------------------------- loop
void loop() {
  if (SerialBT.available()) {
    char c = (char)SerialBT.read();
    if (c != '\n' && c != '\r') handle(c);
  }

  uint32_t now = millis();
  if (now - lastTickMs >= TICK_MS) {
    lastTickMs = now;
    if (!enabled) { tgt = 0; }
    float d = tgt - cur;
    if (d >  RAMP_PER_TICK) d =  RAMP_PER_TICK;
    if (d < -RAMP_PER_TICK) d = -RAMP_PER_TICK;
    cur += d;
    if (enabled) applyDrive(cur);
  }
}
