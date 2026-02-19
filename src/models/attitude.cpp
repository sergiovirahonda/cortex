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
// gainBlend 0 = launch (high KP, PD only), 1 = flight (low KP, I when enableI).
// Linear blend of Kp/Kd in transition. I only when enableI (flight range).

float Attitude::calculateRollPD(float desiredRollAngle, bool enableI, float gainBlend) {
    unsigned long now = micros();
    float dt = (lastTime > 0) ? (now - lastTime) / 1e6f : (1.0f / 570.0f);
    if (dt <= 0.0f || dt > 0.02f) dt = 1.0f / 570.0f;  // clamp first run / glitches
    lastTime = now;
    lastDt = dt;

    float error = desiredRollAngle - this->rollAngle;
    float kP = (1.0f - gainBlend) * droneConfig.getRollLaunchKp() + gainBlend * droneConfig.getRollKp();
    float kD = (1.0f - gainBlend) * droneConfig.getRollLaunchKd() + gainBlend * droneConfig.getRollKd();
    float P = kP * error;
    float kI = droneConfig.getRollKi();

    float I = 0;
    if (enableI && kI > 0.0f) {
        this->rollErrorSum += error * dt;
        this->rollErrorSum = constrain(
            this->rollErrorSum,
            -droneConfig.getMaxIOutput() / kI,
            droneConfig.getMaxIOutput() / kI
        );
        I = kI * this->rollErrorSum;
    } else {
        this->rollErrorSum = 0;
        I = 0;
    }

    float D = kD * this->rollRate;
    float output = P + I - D;
    return constrain(output, -droneConfig.getMaxPDOutput(), droneConfig.getMaxPDOutput());
}

float Attitude::calculatePitchPD(float desiredPitchAngle, bool enableI, float gainBlend) {
    float dt = lastDt;

    float error = desiredPitchAngle - this->pitchAngle;
    float kP = (1.0f - gainBlend) * droneConfig.getPitchLaunchKp() + gainBlend * droneConfig.getPitchKp();
    float kD = (1.0f - gainBlend) * droneConfig.getPitchLaunchKd() + gainBlend * droneConfig.getPitchKd();
    float P = kP * error;
    float kI = droneConfig.getPitchKi();

    float I = 0;
    if (enableI && kI > 0.0f) {
        this->pitchErrorSum += error * dt;
        this->pitchErrorSum = constrain(
            this->pitchErrorSum,
            -droneConfig.getMaxIOutput() / kI,
            droneConfig.getMaxIOutput() / kI
        );
        I = kI * this->pitchErrorSum;
    } else {
        this->pitchErrorSum = 0;
        I = 0;
    }

    float D = kD * this->pitchRate;
    float output = P + I - D;
    return constrain(output, -droneConfig.getMaxPDOutput(), droneConfig.getMaxPDOutput());
}

float Attitude::calculateYawPI(float desiredYawRate, bool enableI, float gainBlend) {
    // 1. Calculate Error (Setpoint - Measurement)
    float error = desiredYawRate - this->yawRate;
    
    // 2. Proportional Term with Blending
    // Blends from LaunchKp (blend=0) to FlightKp (blend=1)
    float kP = (1.0f - gainBlend) * droneConfig.getYawLaunchKp() + gainBlend * droneConfig.getYawKp();
    float P = kP * error;

    // 3. Integral Term (The "Heading Lock")
    float I = 0;
    float kI = droneConfig.getYawKi();

    if (enableI && kI > 0.0f) {
        // Use the stored lastDt from the main loop
        this->yawErrorSum += error * lastDt;
        
        // Anti-Windup
        float maxI = droneConfig.getMaxIOutput();
        this->yawErrorSum = constrain(
            this->yawErrorSum,
            -maxI / kI,
            maxI / kI
        );
        I = kI * this->yawErrorSum;
    } else {
        this->yawErrorSum = 0;
        I = 0;
    }

    // Combined PI output
    float output = P + I;

    return constrain(
        output,
        -droneConfig.getMaxPDOutput(),
        droneConfig.getMaxPDOutput()
    );
}