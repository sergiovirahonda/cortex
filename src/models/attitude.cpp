#include "attitude.h"

#include <Arduino.h>

// =================================================================
// ATTITUDE TRIM CLASS
// =================================================================

AttitudeTrim::AttitudeTrim() {
    rollTrim = 0;
    pitchTrim = 0;
    yawTrim = 0;
}

void AttitudeTrim::reset() {
    rollTrim = 0;
    pitchTrim = 0;
    yawTrim = 0;
}

void AttitudeTrim::setRollTrim(float rollTrim) {
    this->rollTrim = rollTrim;
}

void AttitudeTrim::setPitchTrim(float pitchTrim) {
    this->pitchTrim = pitchTrim;
}

void AttitudeTrim::setYawTrim(float yawTrim) {
    this->yawTrim = yawTrim;
}

float AttitudeTrim::getRollTrim() {
    return this->rollTrim;
}

float AttitudeTrim::getPitchTrim() {
    return this->pitchTrim;
}

float AttitudeTrim::getYawTrim() {
    return this->yawTrim;
}

// =================================================================
// ATTITUDE CLASS
// =================================================================

Attitude::Attitude(const DroneConfig& droneConfig) {
    this->droneConfig = droneConfig;
    rollAngle = 0;
    rollRate = 0;
    pitchAngle = 0;
    pitchRate = 0;
    yawRate = 0;
}

// Bulk setter
void Attitude::updateSensors(float rAngle, float rRate, float pAngle, float pRate, float yRate) {
    this->rollAngle = rAngle;
    this->rollRate  = rRate;
    this->pitchAngle = pAngle;
    this->pitchRate  = pRate;
    this->yawRate   = yRate;
}
// Raw getters
float Attitude::getRollAngle() { return this->rollAngle; }
float Attitude::getRollRate() { return this->rollRate; }
float Attitude::getPitchAngle() { return this->pitchAngle; }
float Attitude::getPitchRate() { return this->pitchRate; }
float Attitude::getYawRate() { return this->yawRate; }

// =================================================================
// STABILIZATION LOGIC (PID with dt for loop-rate independence)
// =================================================================
// dt in seconds; I-term uses error*dt so Ki is "per second". enableI: false
// before takeoff (avoids integrating ground-contact error), true when flying.

float Attitude::calculateRollPD(float desiredRollAngle, bool enableI) {
    unsigned long now = micros();
    float dt = (lastTime > 0) ? (now - lastTime) / 1e6f : (1.0f / 570.0f);
    if (dt <= 0.0f || dt > 0.02f) dt = 1.0f / 570.0f;  // clamp first run / glitches
    lastTime = now;
    lastDt = dt;

    float error = desiredRollAngle - this->rollAngle;
    float P = droneConfig.getRollKp() * error;
    float kI = droneConfig.getRollKi();

    float I = 0;
    if (enableI && kI > 0.0f) {
        this->rollErrorSum += error * dt;
        this->rollErrorSum = constrain(
            this->rollErrorSum,
            -droneConfig.getMaxIOutput() / droneConfig.getRollKi(),
            droneConfig.getMaxIOutput() / droneConfig.getRollKi()
        );
        I = droneConfig.getRollKi() * this->rollErrorSum;
    } else {
        this->rollErrorSum = 0;
        I = 0;
    }

    float D = droneConfig.getRollKd() * this->rollRate;
    float output = P + I - D;
    return constrain(output, -droneConfig.getMaxPDOutput(), droneConfig.getMaxPDOutput());
}

float Attitude::calculatePitchPD(float desiredPitchAngle, bool enableI) {
    float dt = lastDt;

    float error = desiredPitchAngle - this->pitchAngle;
    float P = droneConfig.getPitchKp() * error;
    float kI = droneConfig.getRollKi();

    float I = 0;
    if (enableI && kI > 0.0f) {
        this->pitchErrorSum += error * dt;
        this->pitchErrorSum = constrain(
            this->pitchErrorSum,
            -droneConfig.getMaxIOutput() / droneConfig.getPitchKi(),
            droneConfig.getMaxIOutput() / droneConfig.getPitchKi()
        );
        I = droneConfig.getPitchKi() * this->pitchErrorSum;
    } else {
        this->pitchErrorSum = 0;
        I = 0;
    }

    float D = droneConfig.getPitchKd() * this->pitchRate;
    float output = P + I - D;
    return constrain(output, -droneConfig.getMaxPDOutput(), droneConfig.getMaxPDOutput());
}

// YAW strategy: "P-Only"
// Yaw is naturally stable. D-term usually adds noise.
// Formula: Output = Kp * (DesiredRate - ActualRate)

float Attitude::calculateYawP(float desiredYawRate) {
    float error = desiredYawRate - this->yawRate;
    
    float output = droneConfig.getYawKp() * error;
    return constrain(
        output,
        -droneConfig.getMaxPDOutput(),
        droneConfig.getMaxPDOutput()
    );
}