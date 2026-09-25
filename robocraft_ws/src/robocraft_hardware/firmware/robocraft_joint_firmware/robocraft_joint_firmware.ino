// RoboCraft joint controller firmware v2 — one ATmega328 per arm (I2C slave).
//
// Drives J1 and J2 (12 V 50:1 DC gear motors through the MOSFET H-bridges of
// the thesis), reads the Bourns 3590 10-turn precision potentiometers, runs a
// position loop (PI + velocity feed-forward, gains from Ziegler–Nichols,
// thesis Section 4.3) and drives the rack-and-pinion gripper servo.
//
// Controller per joint (units: degrees, PWM counts 0..255):
//     u = Kp * e + Ki * integral(e) + Kff * w_target
//   e        = target - measured angle
//   w_target = speed of the target, estimated from consecutive I2C targets
//   Kff      = PWM needed per deg/s of speed (from the motor model)
// See analysis/07_motor_control_tuning.py: feed-forward reduces the tracking
// error on a smooth 30 deg move from ~5 deg to <1 deg.
//
// Changes w.r.t. the 2019 appendix code (documented in the README):
//   * Potentiometers on A0/A1. A4/A5 are SDA/SCL on the ATmega328 and were
//     shared with the I2C bus in the original sketch.
//   * PWM on real PWM pins. Pins 7 and 8 used before have no PWM hardware.
//   * Framed protocol with CRC-8 + sequence number (thesis observed corrupted
//     I2C data); invalid frames are dropped, not executed.
//   * Watchdog: no valid command for 250 ms -> motors off (fail-safe).
//   * Soft joint limits, integrator anti-windup, dead-band, velocity feed-forward.
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

// Ultimate gain Ku [PWM/deg] and period Pu [s] per joint. Starting values come
// from the motor model in analysis/07_motor_control_tuning.py; confirm them on
// the robot with the Ziegler–Nichols test (raise Kp until the joint oscillates
// steadily: that Kp is Ku, the oscillation period is Pu).
const float KU1 = 43.7, PU1 = 0.37;
const float KU2 = 38.8, PU2 = 0.27;
// Tyreus-Luyben PI (gentler than classic ZN, see the analysis) ...
const float KP1 = KU1 / 3.2, KI1 = KP1 / (2.2 * PU1);
const float KP2 = KU2 / 3.2, KI2 = KP2 / (2.2 * PU2);
// ... plus feed-forward: PWM per deg/s = (1 / K_motor) * (255 / 12 V), K from the model.
const float KFF1 = 0.86, KFF2 = 0.75;
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
float prev_t1 = 0, prev_t2 = 0, speed1 = 0, speed2 = 0;   // target speed estimate (feed-forward)
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

// One control step of one joint. Returns the signed PWM command.
float control_step(float target, float meas, float target_speed, float kp, float ki, float kff,
                   float &integ, float dt) {
  float e = target - meas;
  float u_ff = kff * target_speed;                 // voltage the motor needs to follow the motion
  if (fabs(e) < DEADBAND_DEG && fabs(target_speed) < 1.0) { integ = 0; return 0; }   // at rest
  float u = kp * e + ki * integ + u_ff;
  if (fabs(u) < PWM_MAX) integ += e * dt;          // anti-windup: integrate only when not saturated
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
    // target speed: low-pass filtered difference of consecutive targets [deg/s]
    speed1 = 0.8 * speed1 + 0.2 * (t1 - prev_t1) / dt;
    speed2 = 0.8 * speed2 + 0.2 * (t2 - prev_t2) / dt;
    drive(M1_FWD, M1_REV, control_step(t1, q1, speed1, KP1, KI1, KFF1, integ1, dt));
    drive(M2_FWD, M2_REV, control_step(t2, q2, speed2, KP2, KI2, KFF2, integ2, dt));
  }
  prev_t1 = t1;
  prev_t2 = t2;
  at_target = fabs(t1 - q1) < 0.5 && fabs(t2 - q2) < 0.5;
  gripper.write(gd);
  brake.write((fl & 0x02) ? 90 : 0);
}
