#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>

// FIT1035 + DRI0058 + AS5600 position control.
//
// Control law (voltage mode, tuned on the bare motor):
//     Uq = Kp * (target - angle) - Kd * velocity + integral(Ki * error)
// It replaces the SimpleFOC angle -> velocity cascade, which hunted because of
// cogging/stiction. loopFOC() runs the law AND the FOC loop: call it as often
// as possible. setTarget() only stores the target.
class MotorControl {
public:
    MotorControl();

    bool begin(TwoWire& wire);
    bool startAlignment();          // blocking, ~4 s, rotates the shaft

    void loopFOC();                 // control law + SimpleFOC loopFOC()/move()
    void setTarget(float targetRad);// stores the target only
    void stop();

    float getAngle();
    float getTarget() const;

    bool isActive() const;
    bool isUnstable() const;        // Uq flipping between the limits (oscillation)
    uint32_t loopRateHz() const;    // loopFOC() calls per second, updated each second
    bool magnetOK();                // AS5600 MD bit (magnet detected)

private:
    TwoWire* wire_ = nullptr;

    MagneticSensorI2C encoder_;
    BLDCMotor motor_;
    BLDCDriver3PWM driver_;

    bool initialized_ = false;
    bool active_ = false;
    bool unstable_ = false;

    float targetRad_ = 0.0f;
    float integral_ = 0.0f;         // [V]

    uint32_t lastUs_ = 0;
    uint32_t flipWindowStartMs_ = 0;
    uint32_t flips_ = 0;
    int lastSaturationSign_ = 0;

    uint32_t rateWindowStartMs_ = 0;
    uint32_t rateCounter_ = 0;
    uint32_t loopRateHz_ = 0;

    bool readRegister(uint8_t reg, uint8_t& value);
    void reportMagnet();
};