
#include "MotorControl.h"
#include "config.h"

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

bool MotorControl::magnetOK() {
    if (!wire_) return false;

    // AS5600 status register 0x0B, bit 5: magnet detected
    wire_->beginTransmission(AS5600_ADDRESS);
    wire_->write(0x0B);

    if (wire_->endTransmission(false) != 0)
        return false;

    if (wire_->requestFrom(
        (uint8_t)AS5600_ADDRESS,
        (uint8_t)1
    ) != 1) return false;

    return (wire_->read() & 0x20) != 0;
}

bool MotorControl::begin(TwoWire& wire) {
    wire_ = &wire;

    pinMode(MOTOR_ENABLE, OUTPUT);
    digitalWrite(MOTOR_ENABLE, LOW);

    if (!magnetOK()) return false;

    encoder_.init(&wire);
    motor_.linkSensor(&encoder_);

    // 3-PWM driver configuration
    driver_.voltage_power_supply =
        MOTOR_SUPPLY_VOLTAGE;

    driver_.voltage_limit =
        MOTOR_VOLTAGE_LIMIT;

    driver_.pwm_frequency = 20000;

    if (driver_.init() != 1) return false;

    motor_.linkDriver(&driver_);

    // Closed-loop angle control
    motor_.controller = MotionControlType::angle;
    motor_.torque_controller = TorqueControlType::voltage;

    motor_.voltage_limit =
        MOTOR_VOLTAGE_LIMIT;

    motor_.voltage_sensor_align =
        MOTOR_ALIGN_VOLTAGE;

    motor_.velocity_limit =
        MOTOR_VELOCITY_LIMIT;

    motor_.PID_velocity.P = MOTOR_VEL_P;
    motor_.PID_velocity.I = MOTOR_VEL_I;
    motor_.PID_velocity.D = 0.0f;

    motor_.P_angle.P = MOTOR_ANGLE_P;
    motor_.LPF_velocity.Tf = 0.02f;

    if (motor_.init() != 1) return false;

    initialized_ = true;
    stop();

    return true;
}

bool MotorControl::startAlignment() {
    if (!initialized_) return false;
    if (!magnetOK()) return false;

    // May rotate the shaft during electrical alignment
    motor_.enable();

    if (motor_.initFOC() != 1) {
        stop();
        return false;
    }

    encoder_.update();
    targetRad_ = motor_.shaftAngle();

    active_ = true;
    motor_.move(targetRad_);

    return true;
}

void MotorControl::loopFOC() {
    if (!active_) return;

    // Call as frequently as possible
    motor_.loopFOC();
}

void MotorControl::setTarget(float targetRad) {
    if (!active_) return;

    targetRad_ = targetRad;
    motor_.move(targetRad_);
}

void MotorControl::stop() {
    active_ = false;

    if (initialized_)
        motor_.disable();

    digitalWrite(MOTOR_ENABLE, LOW);
}

float MotorControl::getAngle() {
    if (!active_) return NAN;
    return motor_.shaftAngle();
}

float MotorControl::getTarget() const {
    return targetRad_;
}

bool MotorControl::isActive() const {
    return active_;
}
