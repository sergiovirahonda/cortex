#include "drone_config.h"

DroneConfig::DroneConfig() {
    // =================================================================
    // ACCELEROMETER OFFSETS
    // =================================================================
    // These offsets ensure "Level" is always "Level", even if you boot on a hill
    // Leveled offset values
    accelOffsets.xOffset = 0.045; //  Towards negative -> Tilt to the right
    accelOffsets.yOffset = 0.001; // Towards negative -> Tilt forward
    accelOffsets.zOffset = -0.06; // MPU readings standard
    
    // =================================================================
    // PID GAINS (Mark4 7" wide X, 1300 KV, 8" props, ~570 Hz loop)
    // =================================================================
    // PID uses dt (seconds); I-term is error*dt so Ki is per-second. Wide X:
    // roll/pitch inertia similar; Kp ~10–20× Kd. Tune P first, then D, then I.
    //
    // Roll PID
    rollPID.kp = 6.50;
    rollPID.ki = 1.50;
    rollPID.kd = 1.20;

    // Pitch PID (slightly stiffer P, less D if frame is symmetric)
    pitchPID.kp = 6.50;
    pitchPID.ki = 1.50;
    pitchPID.kd = 1.20;
    
    // Yaw gain (P-only control)
    kpYaw = 4.0;
    
    // =================================================================
    // OUTPUT LIMITS
    // =================================================================
    // Safety limits to prevent overpowering throttle
    maxPDOutput = 350.0;  // Limit correction to avoid overpowering throttle
    maxIOutput = 75.0;    // Safety limit for integral term

    // =================================================================
    // TRIM SETTINGS
    // =================================================================
    trimDelay = 250;
    trimStep = 5.0;

    // =================================================================
    // FEATURE FLAGS
    // =================================================================
    featureFlagEnableAltitudeReading = false;
}

// Accelerometer offset getters
AccelerometerOffsetConfig DroneConfig::getAccelOffsets() const {
    return accelOffsets;
}

float DroneConfig::getAccelXOffset() const {
    return accelOffsets.xOffset;
}

float DroneConfig::getAccelYOffset() const {
    return accelOffsets.yOffset;
}

float DroneConfig::getAccelZOffset() const {
    return accelOffsets.zOffset;
}

// Roll PID getters
PIDGains DroneConfig::getRollPID() const {
    return rollPID;
}

float DroneConfig::getRollKp() const {
    return rollPID.kp;
}

float DroneConfig::getRollKi() const {
    return rollPID.ki;
}

float DroneConfig::getRollKd() const {
    return rollPID.kd;
}

// Pitch PID getters
PIDGains DroneConfig::getPitchPID() const {
    return pitchPID;
}

float DroneConfig::getPitchKp() const {
    return pitchPID.kp;
}

float DroneConfig::getPitchKi() const {
    return pitchPID.ki;
}

float DroneConfig::getPitchKd() const {
    return pitchPID.kd;
}

// Yaw gain getters
float DroneConfig::getYawKp() const {
    return kpYaw;
}

// Output limit getters
float DroneConfig::getMaxPDOutput() const {
    return maxPDOutput;
}

float DroneConfig::getMaxIOutput() const {
    return maxIOutput;
}

// Trim settings getters

int DroneConfig::getTrimDelay() const {
    return trimDelay;
}

float DroneConfig::getTrimStep() const {
    return trimStep;
}

// Feature flags getters
bool DroneConfig::getFeatureFlagEnableAltitudeReading() const {
    return featureFlagEnableAltitudeReading;
}