// ============================================================================
//  MOTION DEMO for documentation (temporary replacement for src/main.cpp)
//  Uses the real MotorControl class (same code as the application):
//  Uq = Kp*err - Kd*vel + integral(Ki*err), gains from config.h.
//
//  No IMU, no ToF, no button, no laser.
//  Serial 115200. Type a command character, then Enter if your terminal
//  needs it (pio device monitor sends characters immediately).
// ============================================================================
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdlib.h>

#include "config.h"
#include "drivers/I2CBus.h"
#include "control/MotorControl.h"

namespace {

constexpr float kPi       = 3.14159265358979f;
constexpr float kRadToDeg = 180.0f / kPi;
constexpr float kDegToRad = kPi / 180.0f;

// Motion limits (setpoint generator). Tracking was verified on the bare motor
// up to a peak target speed of 2.2 rad/s and 28 rad/s^2; higher speeds are untested.
constexpr float ACCEL_RAD_S2 = 15.0f;
constexpr float SPEED_DEFAULT = 2.0f;     // rad/s  (= 115 deg/s = 19 rpm)
constexpr float SPEED_MIN     = 0.5f;
constexpr float SPEED_MAX     = 5.0f;
constexpr float SPEED_STEP    = 0.5f;

constexpr uint32_t RUN_MAX_MS   = 120000; // continuous run stops by itself after 2 min
constexpr float    MOVE_MAX_DEG = 3600.0f;

constexpr uint32_t LOG_PERIOD_MS     = 20;     // CSV stream rate (50 Hz)
constexpr float    SETTLE_BAND_DEG   = 0.5f;
constexpr uint32_t SETTLE_HOLD_MS    = 200;
constexpr uint32_t SETTLE_TIMEOUT_MS = 3000;

enum class Mode { OFF, HOLD, MOVE, SETTLE, RUN };

I2CBus       bus;
MotorControl motor;

Mode  mode = Mode::OFF;
float home = 0.0f;          // shaft angle at the end of the alignment [rad]
float sp = 0.0f;            // setpoint [rad]
float spVel = 0.0f;         // setpoint velocity [rad/s]
float goal = 0.0f;          // MOVE target [rad]
float vMax = SPEED_DEFAULT; // speed limit [rad/s]
float runVel = 0.0f;        // RUN target speed [rad/s]

uint32_t lastUs = 0;
bool     logging = false;
uint32_t lastLogMs = 0;
uint32_t t0Log = 0;

// bookkeeping for the summary lines
float    startAngle = 0.0f;     // actual angle when a move/run starts [rad]
float    moveCmdDeg = 0.0f;
uint32_t startMs = 0;
float    maxTrackErr = 0.0f;    // max |setpoint - angle| during the motion [rad]
uint32_t settleStartMs = 0, inBandSinceMs = 0;
bool     inBand = false;

// numeric command "m<degrees>" + Enter
bool numMode = false;
char numBuf[16];
uint8_t numLen = 0;

float relDeg(float angleRad) { return (angleRad - home) * kRadToDeg; }

void printHelp() {
  Serial.println("==== MOTION DEMO ====");
  Serial.println(" a  align (4 s, rotates the shaft) - required first");
  Serial.println(" 1 / 2 / 3   move +90 / +180 / +360 deg      4 / 5 / 6   move -90 / -180 / -360 deg");
  Serial.println(" m<deg> + Enter   move by any angle, e.g. m45 or m-30");
  Serial.println(" 0  go back to the home position (position after alignment)");
  Serial.println(" r  run continuously forward   b  run continuously backward   s  stop (decelerate and hold)");
  Serial.println(" +/-  speed limit up/down by 0.5 rad/s (0.5 .. 5.0)");
  Serial.println(" l  CSV log on/off (t_ms,setpoint_deg,angle_deg,error_deg)   p  status");
  Serial.println(" !  emergency stop (motor disabled, needs 'a' again)");
}

void printStatus() {
  Serial.println("---- STATUS ----");
  Serial.print("mode: ");
  switch (mode) {
    case Mode::OFF:    Serial.println("OFF (not aligned)"); break;
    case Mode::HOLD:   Serial.println("HOLD");   break;
    case Mode::MOVE:   Serial.println("MOVE");   break;
    case Mode::SETTLE: Serial.println("SETTLE"); break;
    case Mode::RUN:    Serial.println("RUN");    break;
  }
  Serial.print("speed limit [rad/s]: "); Serial.print(vMax, 1);
  Serial.print("  ("); Serial.print(vMax * kRadToDeg, 0); Serial.print(" deg/s, ");
  Serial.print(vMax * 60.0f / (2.0f * kPi), 1); Serial.println(" rpm)");
  Serial.print("acceleration [rad/s^2]: "); Serial.println(ACCEL_RAD_S2, 1);
  if (mode != Mode::OFF) {
    Serial.print("angle relative to home [deg]: "); Serial.println(relDeg(motor.getAngle()), 2);
    Serial.print("loop rate [Hz]: "); Serial.println(motor.loopRateHz());
  }
}

void emergencyStop(const char *why) {
  motor.stop();
  mode = Mode::OFF;
  spVel = 0.0f;
  Serial.print("STOPPED: "); Serial.println(why);
  Serial.println("Motor disabled. Send 'a' to align again.");
}

void doAlign() {
  mode = Mode::OFF;
  Serial.println("[ALIGN] started: the shaft will turn about +/-50 deg");
  bool ok = motor.startAlignment();
  if (!ok) {
    Serial.println("[ALIGN] FAILED (see MOT: lines). Check ground, supply, magnet.");
    return;
  }
  home = motor.getAngle();
  sp = home; goal = home; spVel = 0.0f; runVel = 0.0f;
  lastUs = micros();
  mode = Mode::HOLD;
  Serial.println("[ALIGN] OK. Position 0.00 deg = home. Ready for commands.");
}

void beginMotionBookkeeping(float cmdDeg) {
  startAngle = motor.getAngle();
  moveCmdDeg = cmdDeg;
  startMs = millis();
  maxTrackErr = 0.0f;
}

void startMove(float deltaDeg) {
  if (mode != Mode::HOLD) { Serial.println("Busy or not aligned: wait for the motor to hold, or press 'a'."); return; }
  if (deltaDeg == 0.0f || fabsf(deltaDeg) > MOVE_MAX_DEG) { Serial.println("Invalid angle."); return; }
  goal = sp + deltaDeg * kDegToRad;
  spVel = 0.0f;
  beginMotionBookkeeping(deltaDeg);
  mode = Mode::MOVE;
  Serial.print("[MOVE] "); Serial.print(deltaDeg > 0 ? "+" : ""); Serial.print(deltaDeg, 1);
  Serial.print(" deg at up to "); Serial.print(vMax, 1); Serial.println(" rad/s ...");
}

void goHome() {
  startMove((home - sp) * kRadToDeg);
}

void startRun(float dir) {
  if (mode != Mode::HOLD) { Serial.println("Busy or not aligned."); return; }
  beginMotionBookkeeping(0.0f);
  runVel = dir * vMax;
  mode = Mode::RUN;
  Serial.print("[RUN] "); Serial.print(dir > 0 ? "forward" : "backward");
  Serial.print(" at "); Serial.print(vMax, 1); Serial.println(" rad/s. Send 's' to stop.");
}

void stopRun() {
  if (mode == Mode::RUN) { runVel = 0.0f; Serial.println("[RUN] stopping ..."); }
  else Serial.println("Not running.");
}

void changeSpeed(float delta) {
  vMax += delta;
  if (vMax < SPEED_MIN) vMax = SPEED_MIN;
  if (vMax > SPEED_MAX) vMax = SPEED_MAX;
  if (mode == Mode::RUN && runVel != 0.0f) runVel = (runVel > 0 ? 1.0f : -1.0f) * vMax;
  Serial.print("speed limit "); Serial.print(vMax, 1); Serial.println(" rad/s");
  if (vMax > 2.2f) Serial.println("(above 2.2 rad/s is untested: watch the max tracking error)");
}

void handleChar(char c) {
  if (numMode) {
    if (c == '\n' || c == '\r') {
      numMode = false;
      numBuf[numLen] = '\0';
      if (numLen > 0) startMove((float)atof(numBuf));
      return;
    }
    if (numLen < sizeof(numBuf) - 1) numBuf[numLen++] = c;
    return;
  }
  switch (c) {
    case 'h': printHelp(); break;
    case 'p': printStatus(); break;
    case 'a': doAlign(); break;
    case '1': startMove(+90.0f);  break;
    case '2': startMove(+180.0f); break;
    case '3': startMove(+360.0f); break;
    case '4': startMove(-90.0f);  break;
    case '5': startMove(-180.0f); break;
    case '6': startMove(-360.0f); break;
    case '0': goHome(); break;
    case 'm': numMode = true; numLen = 0; break;
    case 'r': startRun(+1.0f); break;
    case 'b': startRun(-1.0f); break;
    case 's': stopRun(); break;
    case '+': changeSpeed(+SPEED_STEP); break;
    case '-': changeSpeed(-SPEED_STEP); break;
    case 'l':
      logging = !logging;
      if (logging) { t0Log = millis(); Serial.println("t_ms,setpoint_deg,angle_deg,error_deg"); }
      else Serial.println("[LOG] off");
      break;
    case '!': emergencyStop("emergency stop command"); break;
    default: break;   // ignore CR/LF/space
  }
}

// Trapezoidal velocity profile: accelerate, cruise at vMax, decelerate so that
// the setpoint arrives at `goal` with zero velocity.
void updateTrajectory(float dt) {
  const float maxDv = ACCEL_RAD_S2 * dt;

  if (mode == Mode::MOVE) {
    float d = goal - sp;
    float dir = (d >= 0.0f) ? 1.0f : -1.0f;
    float vStop = sqrtf(2.0f * ACCEL_RAD_S2 * fabsf(d));
    float vCmd = dir * fminf(vMax, vStop);
    float dv = vCmd - spVel;
    if (dv >  maxDv) dv =  maxDv;
    if (dv < -maxDv) dv = -maxDv;
    spVel += dv;
    float next = sp + spVel * dt;
    if ((goal - next) * dir <= 0.0f || (fabsf(goal - next) < 1e-4f && fabsf(spVel) < 0.05f)) {
      sp = goal; spVel = 0.0f;
      mode = Mode::SETTLE;
      settleStartMs = millis();
      inBand = false;
    } else {
      sp = next;
    }
  } else if (mode == Mode::RUN) {
    if (runVel != 0.0f && (uint32_t)(millis() - startMs) > RUN_MAX_MS) {
      runVel = 0.0f;
      Serial.println("[RUN] time limit reached, stopping");
    }
    float dv = runVel - spVel;
    if (dv >  maxDv) dv =  maxDv;
    if (dv < -maxDv) dv = -maxDv;
    spVel += dv;
    sp += spVel * dt;
    if (runVel == 0.0f && fabsf(spVel) < 1e-3f) {
      spVel = 0.0f;
      goal = sp;
      float angleNow = motor.getAngle();
      float turned = (angleNow - startAngle) * kRadToDeg;
      float secs = (float)(millis() - startMs) * 0.001f;
      Serial.print("[RUN] stopped: turned "); Serial.print(turned, 1);
      Serial.print(" deg ("); Serial.print(turned / 360.0f, 2);
      Serial.print(" turns) in "); Serial.print(secs, 1);
      Serial.print(" s, max tracking error "); Serial.print(maxTrackErr * kRadToDeg, 2);
      Serial.println(" deg");
      mode = Mode::HOLD;
    }
  }
}

void finishMoveIfSettled() {
  if (mode != Mode::SETTLE) return;
  float err = (goal - motor.getAngle()) * kRadToDeg;
  uint32_t now = millis();

  if (fabsf(err) <= SETTLE_BAND_DEG) {
    if (!inBand) { inBand = true; inBandSinceMs = now; }
  } else {
    inBand = false;
  }
  bool settled  = inBand && (uint32_t)(now - inBandSinceMs) >= SETTLE_HOLD_MS;
  bool timedOut = (uint32_t)(now - settleStartMs) >= SETTLE_TIMEOUT_MS;
  if (!settled && !timedOut) return;

  float angleNow = motor.getAngle();
  float actual = (angleNow - startAngle) * kRadToDeg;
  Serial.print("[MOVE] done: commanded "); Serial.print(moveCmdDeg > 0 ? "+" : ""); Serial.print(moveCmdDeg, 1);
  Serial.print(" deg | actual "); Serial.print(actual > 0 ? "+" : ""); Serial.print(actual, 2);
  Serial.print(" deg | end error "); Serial.print(err > 0 ? "+" : ""); Serial.print(err, 2);
  Serial.print(" deg | position "); Serial.print(relDeg(angleNow), 2);
  Serial.print(" deg from home | max tracking error "); Serial.print(maxTrackErr * kRadToDeg, 2);
  Serial.print(" deg | time "); Serial.print(millis() - startMs); Serial.print(" ms");
  if (timedOut && !settled) Serial.print(" | NOT settled within 0.5 deg");
  Serial.println();
  mode = Mode::HOLD;
}

}  // namespace

// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);          // laser stays off

  Serial.println();
  Serial.println("MOTION DEMO (laser supply disconnected? shaft free to turn?)");

  bus.begin();
  bus.scan();

  if (!motor.begin(bus.getWire())) {
    Serial.println("FAIL: motor.begin() (magnet not detected, driver or motor init failed)");
    while (true) { delay(1000); }
  }
  Wire.setClock(I2C_FREQUENCY);          // every Wire.begin() resets the clock

  Serial.println("Motor initialised, driver disabled.");
  printHelp();
}

void loop() {
  while (Serial.available() > 0) handleChar((char)Serial.read());

  if (mode == Mode::OFF) return;

<<<<<<< Updated upstream
    application.update();
}
=======
  uint32_t nowUs = micros();
  float dt = (float)(nowUs - lastUs) * 1e-6f;
  lastUs = nowUs;
  if (dt > 0.005f) dt = 0.005f;

  updateTrajectory(dt);
  motor.setTarget(sp);
  motor.loopFOC();                       // control law + FOC, as often as possible

  if (motor.isUnstable()) { emergencyStop("position loop oscillating"); return; }

  float angle = motor.getAngle();
  float trackErr = fabsf(sp - angle);
  if (mode == Mode::MOVE || mode == Mode::SETTLE || mode == Mode::RUN) {
    if (trackErr > maxTrackErr) maxTrackErr = trackErr;
  }
  if (trackErr > 20.0f * kDegToRad && (mode == Mode::MOVE || mode == Mode::RUN)) {
    emergencyStop("tracking error above 20 deg (stalled or blocked?)");
    return;
  }

  finishMoveIfSettled();

  if (logging && (uint32_t)(millis() - lastLogMs) >= LOG_PERIOD_MS) {
    lastLogMs = millis();
    Serial.print(millis() - t0Log); Serial.print(',');
    Serial.print(relDeg(sp), 2);    Serial.print(',');
    Serial.print(relDeg(angle), 2); Serial.print(',');
    Serial.println((sp - angle) * kRadToDeg, 2);
  }
}
>>>>>>> Stashed changes
