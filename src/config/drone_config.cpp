#include "drone_config.h"

DroneConfig::DroneConfig() {
    // =================================================================
    // ACCELEROMETER OFFSETS
    // =================================================================
    // These offsets ensure "Level" is always "Level", even if you boot on a hill
    // Leveled offset values
    accelOffsets.xOffset = 0.0215;    //  Towards negative -> Tilt to the right
    accelOffsets.yOffset = -0.0035;   // Towards negative -> Tilt forward
    accelOffsets.zOffset = -0.0600;   // MPU readings standard
    
    // =================================================================
    // PID GAINS (12" M2M, 8" props, 1300 KV, ~570 Hz loop)
    // =================================================================
    // Tuned for stable hand-test: lower Kp + higher Kd = less wobble. If sluggish, raise Kp.
    //
    // Roll PID
    rollPID.kp = 4.0;
    rollPID.ki = 1.2;
    rollPID.kd = 1.9;

    // Pitch PID (slightly softer P, more D to reduce pitch wobble)
    pitchPID.kp = 3.8;
    pitchPID.ki = 1.2;
    pitchPID.kd = 2.0;

    // Yaw gain (P-only; enough to fight twist, not so high it overshoots/wobbles)
    kpYaw = 3.45;

    // Launch gains (high KP, PD only; for spool-up / early climb)
    rollLaunchKp = 12.0f;
    rollLaunchKd = 3.0f;
    pitchLaunchKp = 12.0f;
    pitchLaunchKd = 3.0f;

    // Throttle ranges: 1 = high KP (launch), 2 = transition, 3 = low KP (flight)
    throttleIdle = 60;           // below: no corrections
    throttleLaunchEnd = 1100;    // below: 100% launch gains
    throttleFlightStart = 1300;  // above: 100% flight gains; between = blend

    // =================================================================
    // OUTPUT LIMITS (12" M2M: softer cap = smoother correction, less overshoot)
    // =================================================================
    maxPDOutput = 270.0;
    maxIOutput = 60.0;

    // =================================================================
    // TRIM SETTINGS (degrees / deg/s; trim is setpoint offset, not PWM)
    // =================================================================
    trimDelay = 250;              // ms between trim steps (debounce)
    trimStepPitchRollDeg = 0.2f;  // degrees per step (safe, fine adjustment)
    trimStepYawDegPerSec = 2.0f;  // deg/s per step (yaw rate trim)

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

// Launch gain getters (PD only)
float DroneConfig::getRollLaunchKp() const {
    return rollLaunchKp;
}
float DroneConfig::getRollLaunchKd() const {
    return rollLaunchKd;
}
float DroneConfig::getPitchLaunchKp() const {
    return pitchLaunchKp;
}
float DroneConfig::getPitchLaunchKd() const {
    return pitchLaunchKd;
}

// Throttle range getters
int DroneConfig::getThrottleIdle() const {
    return throttleIdle;
}
int DroneConfig::getThrottleLaunchEnd() const {
    return throttleLaunchEnd;
}
int DroneConfig::getThrottleFlightStart() const {
    return throttleFlightStart;
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

float DroneConfig::getTrimStepPitchRollDeg() const {
    return trimStepPitchRollDeg;
}

float DroneConfig::getTrimStepYawDegPerSec() const {
    return trimStepYawDegPerSec;
}

// Feature flags getters
bool DroneConfig::getFeatureFlagEnableAltitudeReading() const {
    return featureFlagEnableAltitudeReading;
}