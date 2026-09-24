// RoboCraft joint controller firmware v2 — one ATmega328 per arm (I2C slave).
//
// Drives J1 and J2 (12 V 50:1 DC gear motors through the MOSFET H-bridges of
// the thesis), reads the Bourns 3590 10-turn precision potentiometers, runs a
// PI position loop (Ziegler–Nichols tuned, thesis Section 4.3) and drives the
// rack-and-pinion gripper servo.
//
// Changes w.r.t. the 2019 appendix code (documented in the README):
//   * Potentiometers on A0/A1. A4/A5 are SDA/SCL on the ATmega328 and were
//     shared with the I2C bus in the original sketch.
//   * PWM on real PWM pins. Pins 7 and 8 used before have no PWM hardware.
//   * Framed protocol with CRC-8 + sequence number (thesis observed corrupted
//     I2C data); invalid frames are dropped, not executed.
//   * Watchdog: no valid command for 250 ms -> motors off (fail-safe).
//   * Soft joint limits, integrator anti-windup, dead-band.
//
// Frame formats: see robocraft_kinematics/protocol.py.
//
// Build: Arduino IDE / arduino-cli, board "Arduino Uno" (ATmega328P, 16 MHz).
//        Set I2C_ADDRESS per arm (arm1 0x08, arm2 0x09, arm3 0x0A).

#include <Servo.h>
#include <Wire.h>

// ------------------------------------------------------------------ config
#define I2C_ADDRESS 0x08

// H-bridge inputs (Timer0: 5/6, Timer2: 3/11). Timer1 (9/10) serves the servos.
const uint8_t M1_FWD = 5, M1_REV = 6;
const uint8_t M2_FWD = 3, M2_REV = 11;
const uint8_t POT1 = A0, POT2 = A1;
const uint8_t GRIPPER_SERVO = 9;
const uint8_t BRAKE_SERVO = 10;   // optional: platform worm-gear brake

// Potentiometer: 10 turns = 3600 deg over 0..1023 (thesis: 3600/1023 per count),
// linearised with the thesis calibration  angle = 1.0074*raw_deg - 2.6413.
// ZERO_* is the pot angle at joint zero (thesis used 1172 deg / 1158 deg).
const float POT_DEG_PER_COUNT = 3600.0 / 1023.0;
const float LIN_GAIN = 1.0074, LIN_OFFSET = -2.6413;
const float ZERO1_DEG = 1172.0, ZERO2_DEG = 1158.0;
const float DIR1 = 1.0, DIR2 = 1.0;             // flip if a joint counts backwards

const float LIMIT1_DEG = 150.0, LIMIT2_DEG = 145.0;

// PI gains from Ziegler–Nichols (P: Kp = 0.5 Ku; PI: Kp = 0.45 Ku, Ti = Pu / 1.2).
// Re-identify Ku, Pu on your hardware (see README "Commissioning").
const float KU1 = 22.0, PU1 = 0.40;   // [PWM/deg], [s]
const float KU2 = 1.8, PU2 = 0.35;
const float KP1 = 0.45 * KU1, KI1 = KP1 / (PU1 / 1.2);
const float KP2 = 0.45 * KU2, KI2 = KP2 / (PU2 / 1.2);
const float DEADBAND_DEG = 0.3;
const int PWM_MIN = 35, PWM_MAX = 255;  // overcome static friction / saturation

const unsigned long LOOP_US = 5000;      // 200 Hz position loop
const unsigned long WATCHDOG_MS = 250;

// ------------------------------------------------------------------ state
volatile int16_t target1_cdeg = 0, target2_cdeg = 0;
volatile uint8_t gripper_deg = 0, flags = 0, last_seq = 0;
volatile unsigned long last_cmd_ms = 0;
volatile bool fault = false;

int16_t meas1_cdeg = 0, meas2_cdeg = 0;
float integ1 = 0, integ2 = 0;
bool at_target = false;
Servo gripper, brake;

uint8_t crc8(const uint8_t *d, uint8_t n) {
  uint8_t c = 0;
  for (uint8_t i = 0; i < n; i++) {
    c ^= d[i];
    for (uint8_t b = 0; b < 8; b++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
  }
  return c;
}

float read_joint_deg(uint8_t pin, float zero, float dir) {
  long acc = 0;
  for (uint8_t i = 0; i < 4; i++) acc += analogRead(pin);   // light oversampling
  float raw = (acc / 4.0) * POT_DEG_PER_COUNT;
  return dir * ((LIN_GAIN * raw + LIN_OFFSET) - zero);
}

void drive(uint8_t fwd, uint8_t rev, float u) {
  int pwm = constrain((int)fabs(u), 0, PWM_MAX);
  if (pwm > 0 && pwm < PWM_MIN) pwm = PWM_MIN;
  if (u > 0) { analogWrite(rev, 0); analogWrite(fwd, pwm); }
  else if (u < 0) { analogWrite(fwd, 0); analogWrite(rev, pwm); }
  else { analogWrite(fwd, 0); analogWrite(rev, 0); }
}

void motors_off() {
  drive(M1_FWD, M1_REV, 0);
  drive(M2_FWD, M2_REV, 0);
  integ1 = integ2 = 0;
}

float pi_step(float target, float meas, float kp, float ki, float &integ, float dt) {
  float e = target - meas;
  if (fabs(e) < DEADBAND_DEG) { integ = 0; return 0; }
  float u = kp * e + ki * integ;
  if (fabs(u) < PWM_MAX) integ += e * dt;   // anti-windup: integrate only when not saturated
  return u;
}

// ------------------------------------------------------------------ I2C
void onReceive(int n) {
  if (n != 9) { while (Wire.available()) Wire.read(); return; }
  uint8_t f[9];
  for (uint8_t i = 0; i < 9; i++) f[i] = Wire.read();
  if (f[0] != 0xA5 || crc8(f, 8) != f[8]) return;          // corrupted -> ignore
  target1_cdeg = (int16_t)((f[2] << 8) | f[3]);
  target2_cdeg = (int16_t)((f[4] << 8) | f[5]);
  gripper_deg = f[6];
  flags = f[7];
  last_seq = f[1];
  if (flags & 0x04) fault = false;                         // clear fault
  last_cmd_ms = millis();
}

void onRequest() {
  uint8_t f[8];
  f[0] = 0x5A;
  f[1] = last_seq;
  f[2] = (uint8_t)(meas1_cdeg >> 8); f[3] = (uint8_t)meas1_cdeg;
  f[4] = (uint8_t)(meas2_cdeg >> 8); f[5] = (uint8_t)meas2_cdeg;
  bool wd = (millis() - last_cmd_ms) > WATCHDOG_MS;
  f[6] = ((flags & 1) ? 1 : 0) | (fault ? 2 : 0) | (wd ? 4 : 0) | (at_target ? 8 : 0);
  f[7] = crc8(f, 7);
  Wire.write(f, 8);
}

// ------------------------------------------------------------------ main
void setup() {
  pinMode(M1_FWD, OUTPUT); pinMode(M1_REV, OUTPUT);
  pinMode(M2_FWD, OUTPUT); pinMode(M2_REV, OUTPUT);
  motors_off();
  gripper.attach(GRIPPER_SERVO);
  brake.attach(BRAKE_SERVO);
  Serial.begin(115200);
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);
  Serial.println(F("RoboCraft joint controller v2 ready"));
}

void loop() {
  static unsigned long t_prev = micros();
  unsigned long now = micros();
  if (now - t_prev < LOOP_US) return;
  float dt = (now - t_prev) * 1e-6;
  t_prev = now;

  float q1 = read_joint_deg(POT1, ZERO1_DEG, DIR1);
  float q2 = read_joint_deg(POT2, ZERO2_DEG, DIR2);
  noInterrupts();
  meas1_cdeg = (int16_t)constrain(q1 * 100.0, -32768.0, 32767.0);
  meas2_cdeg = (int16_t)constrain(q2 * 100.0, -32768.0, 32767.0);
  float t1 = target1_cdeg / 100.0, t2 = target2_cdeg / 100.0;
  uint8_t fl = flags, gd = gripper_deg;
  unsigned long age = millis() - last_cmd_ms;
  interrupts();

  // hard limit supervision: leaving the soft range by >5 deg latches a fault
  if (fabs(q1) > LIMIT1_DEG + 5 || fabs(q2) > LIMIT2_DEG + 5) fault = true;

  bool enabled = (fl & 0x01) && !fault && age < WATCHDOG_MS;
  if (!enabled) {
    motors_off();
  } else {
    t1 = constrain(t1, -LIMIT1_DEG, LIMIT1_DEG);
    t2 = constrain(t2, -LIMIT2_DEG, LIMIT2_DEG);
    drive(M1_FWD, M1_REV, pi_step(t1, q1, KP1, KI1, integ1, dt));
    drive(M2_FWD, M2_REV, pi_step(t2, q2, KP2, KI2, integ2, dt));
  }
  at_target = fabs(t1 - q1) < 0.5 && fabs(t2 - q2) < 0.5;
  gripper.write(gd);
  brake.write((fl & 0x02) ? 90 : 0);
}
