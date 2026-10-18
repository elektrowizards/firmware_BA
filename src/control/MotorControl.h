
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>

class MotorControl {
public:
    MotorControl();

    bool begin(TwoWire& wire);
    bool startAlignment();

    void loopFOC();
    void setTarget(float targetRad);
    void stop();

    float getAngle();
    float getTarget() const;

    bool isActive() const;
    bool magnetOK();

private:
    TwoWire* wire_ = nullptr;

    MagneticSensorI2C encoder_;
    BLDCMotor motor_;
    BLDCDriver3PWM driver_;

    bool initialized_ = false;
    bool active_ = false;

    float targetRad_ = 0.0f;
};
