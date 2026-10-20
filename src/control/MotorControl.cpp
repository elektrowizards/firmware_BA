#include "MotorControl.h"
#include "config.h"

namespace {
    float clampf(float v, float lo, float hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }
}

MotorControl::MotorControl()
    : encoder_(AS5600_I2C),
      motor_(MOTOR_POLE_PAIRS),
      driver_(
          MOTOR_IN1,
          MOTOR_IN2,
          MOTOR_IN3,
          MOTOR_ENABLE
      ) {
}

bool MotorControl::readRegister(uint8_t reg, uint8_t& value) {
    if (!wire_) return false;

    wire_->beginTransmission((uint8_t)AS5600_ADDRESS);
    wire_->write(reg);

    if (wire_->endTransmission(false) != 0)
        return false;

    if (wire_->requestFrom(
        (uint8_t)AS5600_ADDRESS,
        (uint8_t)1
    ) != 1) return false;

    value = (uint8_t)wire_->read();
    return true;
}

bool MotorControl::magnetOK() {
    uint8_t status = 0;
    if (!readRegister(0x0B, status)) return false;

    // AS5600 STATUS bit 5: magnet detected (MD)
    return (status & 0x20) != 0;
}

void MotorControl::reportMagnet() {
    uint8_t status = 0, agc = 0;
    if (!readRegister(0x0B, status) || !readRegister(0x1A, agc)) return;

    Serial.print("[AS5600] MD=");  Serial.print((status >> 5) & 1);
    Serial.print(" ML=");          Serial.print((status >> 4) & 1);
    Serial.print(" MH=");          Serial.print((status >> 3) & 1);
    Serial.print(" AGC=");         Serial.println(agc);

    // ML = field too weak, MH = field too strong. Warning only: ML=1 was
    // observed permanently on the bench while the encoder still worked.
    if (status & 0x18)
        Serial.println("[AS5600] WARNING: magnet field outside the ideal range");
}

bool MotorControl::begin(TwoWire& wire) {
    wire_ = &wire;

    pinMode(MOTOR_ENABLE, OUTPUT);
    digitalWrite(MOTOR_ENABLE, LOW);      // DRI0058: EN is active HIGH

    SimpleFOCDebug::enable(&Serial);      // makes the MOT: lines visible

    if (!magnetOK()) return false;
    reportMagnet();

    // NOTE: encoder_.init() calls Wire.begin(), which resets the I2C clock to
    // 100 kHz. The caller must call Wire.setClock() after all begin() calls.
    encoder_.init(&wire);
    encoder_.min_elapsed_time = MOTOR_ENCODER_VEL_WINDOW_S;
    motor_.linkSensor(&encoder_);

    // 3-PWM driver configuration
    driver_.voltage_power_supply = MOTOR_SUPPLY_VOLTAGE;
    driver_.pwm_frequency        = 20000;
    driver_.enable_active_high   = true;

    // driver_.voltage_limit is deliberately NOT lowered: SinePWM centres the
    // phase voltages at driver.voltage_limit/2 and clips each phase to
    // [0, driver.voltage_limit]. A low driver limit clips the output.
    if (driver_.init() != 1) return false;

    motor_.linkDriver(&driver_);

    // Torque (voltage) mode: the position law lives in loopFOC().
    motor_.foc_modulation    = FOCModulationType::SinePWM;
    motor_.controller        = MotionControlType::torque;
    motor_.torque_controller = TorqueControlType::voltage;

    // High limit for the alignment; lowered to MOTOR_VOLTAGE_LIMIT afterwards.
    motor_.voltage_limit        = MOTOR_ALIGN_VOLTAGE;
    motor_.voltage_sensor_align = MOTOR_ALIGN_VOLTAGE;
    motor_.LPF_velocity.Tf      = MOTOR_VELOCITY_FILTER_TF;

    if (motor_.init() != 1) return false;

    initialized_ = true;
    stop();

    return true;
}

bool MotorControl::startAlignment() {
    if (!initialized_) return false;
    if (!magnetOK()) return false;

    active_ = false;
    unstable_ = false;
    integral_ = 0.0f;

    // Alignment needs a high voltage: at 2 V the rotor settles in a cogging
    // detent and the electrical zero is biased (observed on the bench).
    motor_.voltage_limit        = MOTOR_ALIGN_VOLTAGE;
    motor_.voltage_sensor_align = MOTOR_ALIGN_VOLTAGE;
    motor_.sensor_direction     = Direction::UNKNOWN;
    motor_.zero_electric_angle  = NOT_SET;

    // May rotate the shaft by about +/-50 degrees
    motor_.enable();

    if (motor_.initFOC() != 1) {
        stop();
        return false;
    }

    // initFOC() only prints a failed pole-pair check, it still returns 1.
    if (!motor_.pp_check_result) {
        Serial.println("[MOTOR] pole-pair check failed");
        stop();
        return false;
    }

    motor_.voltage_limit = MOTOR_VOLTAGE_LIMIT;     // run limit

    targetRad_ = motor_.shaftAngle();
    motor_.shaft_angle = targetRad_;
    integral_ = 0.0f;

    lastUs_ = micros();
    flipWindowStartMs_ = millis();
    flips_ = 0;
    lastSaturationSign_ = 0;
    rateWindowStartMs_ = millis();
    rateCounter_ = 0;
    loopRateHz_ = 0;

    active_ = true;
    return true;
}

void MotorControl::loopFOC() {
    if (!active_) return;

    uint32_t nowUs = micros();
    float dt = (float)(nowUs - lastUs_) * 1e-6f;
    lastUs_ = nowUs;
    if (dt > 0.01f) dt = 0.01f;

    const float limit = MOTOR_VOLTAGE_LIMIT;

    // Position law (shaft_angle / shaft_velocity come from the previous move())
    float err = targetRad_ - motor_.shaft_angle;
    integral_ = clampf(integral_ + MOTOR_KI * err * dt,
                       -0.5f * limit, 0.5f * limit);

    float uq = MOTOR_KP * err
             - MOTOR_KD * motor_.shaft_velocity
             + integral_;
    uq = clampf(uq, -limit, limit);

    // Oscillation guard: Uq flipping between +/- limit
    uint32_t nowMs = millis();
    if (uq >= 0.9f * limit || uq <= -0.9f * limit) {
        int sign = (uq > 0.0f) ? 1 : -1;
        if (lastSaturationSign_ != 0 && sign != lastSaturationSign_) flips_++;
        lastSaturationSign_ = sign;
    }
    if ((uint32_t)(nowMs - flipWindowStartMs_) >= 500) {
        if (flips_ >= 6) unstable_ = true;
        flips_ = 0;
        flipWindowStartMs_ = nowMs;
    }
    if (unstable_) uq = 0.0f;

    motor_.loopFOC();       // sensor update + phase voltages
    motor_.move(uq);        // torque mode: sets the voltage command

    // Loop-rate measurement
    rateCounter_++;
    if ((uint32_t)(nowMs - rateWindowStartMs_) >= 1000) {
        loopRateHz_ = rateCounter_;
        rateCounter_ = 0;
        rateWindowStartMs_ = nowMs;
    }
}

void MotorControl::setTarget(float targetRad) {
    if (!active_) return;

    targetRad_ = targetRad;
}

void MotorControl::stop() {
    active_ = false;
    integral_ = 0.0f;

    if (initialized_)
        motor_.disable();

    digitalWrite(MOTOR_ENABLE, LOW);
}

float MotorControl::getAngle() {
    if (!active_) return NAN;
    return motor_.shaft_angle;
}

float MotorControl::getTarget() const {
    return targetRad_;
}

bool MotorControl::isActive() const {
    return active_;
}

bool MotorControl::isUnstable() const {
    return unstable_;
}

uint32_t MotorControl::loopRateHz() const {
    return loopRateHz_;
}