#include "altitude.h"
#include <Arduino.h>
#include <math.h>

// =================================================================
// CONSTRUCTOR & STATE MANAGEMENT
// =================================================================

// AltitudeHold class - manages the target altitude and engagement state

AltitudeHold::AltitudeHold() {
    this->targetAltitudeCm = 0.0f;
    this->altitudeHoldEngaged = false;
}

void AltitudeHold::reset() {
    this->targetAltitudeCm = 0.0f;
    this->altitudeHoldEngaged = false;
}

void AltitudeHold::setTargetAltitudeCm(float targetAltitudeCm) {
    this->targetAltitudeCm = targetAltitudeCm;
}

void AltitudeHold::setAltitudeHoldEngaged(bool altitudeHoldEngaged) {
    this->altitudeHoldEngaged = altitudeHoldEngaged;
}

float AltitudeHold::getTargetAltitudeCm() {
    return this->targetAltitudeCm;
}

bool AltitudeHold::getAltitudeHoldEngaged() {
    return this->altitudeHoldEngaged;
}

// Altitude class - manages the altitude state and fusion

Altitude::Altitude(const DroneConfig& droneConfig) {
    this->droneConfig = droneConfig;
    this->currentAltitudeCm = 0.0f;
    this->currentVelocityCmS = 0.0f;
    this->altitudeErrorSum = 0.0f;
    this->lastTime = 0;
    this->lastDt = 0.0f;
}

void Altitude::resetPID() {
    this->altitudeErrorSum = 0.0f;
}

void Altitude::reset() {
    this->currentAltitudeCm = 0.0f;
    this->currentVelocityCmS = 0.0f;
    this->currentRawAccelerationGs = 0.0f;
    this->altitudeErrorSum = 0.0f;
    this->lastTime = 0;
    this->lastDt = 0.0f;
}

void Altitude::setCurrentRawAccelerationGs(float rawAccelZ_Gs) {
    this->currentRawAccelerationGs = rawAccelZ_Gs;
}

float Altitude::getCurrentAltitudeCm() { return this->currentAltitudeCm; }
float Altitude::getCurrentVelocityCmS() { return this->currentVelocityCmS; }
float Altitude::getCurrentRawAccelerationGs() { return this->currentRawAccelerationGs; }

// =================================================================
// SENSOR FUSION & KINEMATICS (The "Estimator")
// =================================================================

void Altitude::updateSensors(float pitchDeg, float rollDeg, float lidarAltCm) {
    // 1. Calculate Loop Delta Time (dt)
    unsigned long now = micros();
    float dt = (lastTime > 0) ? (now - lastTime) / 1e6f : (1.0f / 100.0f);
    if (dt <= 0.0f || dt > 0.05f) dt = 1.0f / 100.0f; // Clamp if dt gets weird
    this->lastTime = now;
    this->lastDt = dt; // Save for the PID calculation

    // 2. Convert degrees to radians for C++ trig functions
    float pitchRad = pitchDeg * (PI / 180.0f);
    float rollRad = rollDeg * (PI / 180.0f);
    
    float cosPitch = cos(pitchRad);
    float cosRoll = cos(rollRad);

    // 3. EARTH-FRAME TRIGONOMETRY
    // Fix the tilt bleed for both the accelerometer and the LiDAR
    float earthAccelZ_Gs = this->currentRawAccelerationGs * cosPitch * cosRoll;
    float trueLidarAltCm = lidarAltCm * cosPitch * cosRoll;

    // 4. ISOLATE MOVEMENT
    // Subtract Earth's gravity (1.0G) and convert to cm/s^2
    float dynamicAccelZ_Gs = earthAccelZ_Gs - 1.0f;
    float accel_cm_s2 = dynamicAccelZ_Gs * 980.665f;

    // 5. PREDICTION (Calculus)
    // Integrate acceleration -> velocity -> position
    this->currentVelocityCmS += (accel_cm_s2 * dt);
    this->currentAltitudeCm += (this->currentVelocityCmS * dt);

    // 6. CORRECTION (Complementary Filter) - only when LiDAR is in valid range
    // Reject out-of-range, zero, or invalid readings so bad LiDAR doesn't drag the estimate.
    // Safe: no null deref, no division; if min > max then lidarValid stays false and we skip correction.
    float minCm = this->droneConfig.getAltitudeLidarMinCm();
    float maxCm = this->droneConfig.getAltitudeLidarMaxCm();
    bool lidarValid = (lidarAltCm > 0.0f && lidarAltCm >= minCm && lidarAltCm <= maxCm);

    if (lidarValid) {
        // How far off is our pure math from the physical laser?
        float altitudeError = trueLidarAltCm - this->currentAltitudeCm;
        // Pull the math back to reality using the fusion gains
        this->currentAltitudeCm += (this->droneConfig.getAltitudeFusionKp() * altitudeError * dt);
        this->currentVelocityCmS += (this->droneConfig.getAltitudeFusionKi() * altitudeError * dt);
    }
}

// =================================================================
// STABILIZATION LOGIC (The "Controller")
// =================================================================

float Altitude::calculateAltitudePID(float desiredAltitudeCm) {
    // We use lastDt here to ensure the PID math is perfectly synchronized 
    // with the exact time slice used in updateSensors()
    float dt = this->lastDt;
    if (dt <= 0.0f) return 0.0f; // Safety catch

    // 1. Calculate Error
    float error = desiredAltitudeCm - this->currentAltitudeCm;

    // 2. Proportional (P) - The immediate rubber-band correction
    float P = this->droneConfig.getAltitudeKp() * error;

    // 3. Integral (I) - The steady-state error & battery sag corrector
    float I = 0.0f;
    if (this->droneConfig.getAltitudeKi() > 0.0f) {
        this->altitudeErrorSum += (error * dt);
        
        // Anti-Windup: Limit how much authority the I-term has (e.g., max 150 PWM)
        float maxI = this->droneConfig.getAltitudeMaxIOutput();
        this->altitudeErrorSum = constrain(
            this->altitudeErrorSum,
            -maxI / this->droneConfig.getAltitudeKi(),
            maxI / this->droneConfig.getAltitudeKi()
        );
        I = this->droneConfig.getAltitudeKi() * this->altitudeErrorSum;
    } else {
        this->altitudeErrorSum = 0.0f;
    }

    // 4. Derivative (D) - The aerodynamic shock absorber
    // Applied strictly to current velocity (Measurement) to prevent Derivative Kick
    // when the target altitude changes suddenly.
    float D = this->droneConfig.getAltitudeKd() * this->currentVelocityCmS;

    // 5. Final Motor Correction
    float output = P + I - D;

    // 6. Output Limit
    // Never allow the AltHold PID to command more than +/- 250 PWM to the motors
    return constrain(
        output,
        -this->droneConfig.getAltitudeMaxCorrection(),
        this->droneConfig.getAltitudeMaxCorrection()
    );
}