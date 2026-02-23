#include "drone_config.h"
#include "drone_config_defaults.h"

DroneConfig::DroneConfig() {
    // =================================================================
    // ACCELEROMETER OFFSETS (from build_flags or drone_config_defaults.h)
    // =================================================================
    accelOffsets.xOffset = ACCEL_X_OFFSET;
    accelOffsets.yOffset = ACCEL_Y_OFFSET;
    accelOffsets.zOffset = ACCEL_Z_OFFSET;

    // =================================================================
    // PID GAINS (from build_flags or drone_config_defaults.h)
    // =================================================================
    rollPID.kp = ROLL_KP;
    rollPID.ki = ROLL_KI;
    rollPID.kd = ROLL_KD;
    pitchPID.kp = PITCH_KP;
    pitchPID.ki = PITCH_KI;
    pitchPID.kd = PITCH_KD;
    yawPID.kp = YAW_KP;
    yawPID.ki = YAW_KI;
    yawPID.kd = YAW_KD;

    rollLaunchKp = ROLL_LAUNCH_KP;
    rollLaunchKd = ROLL_LAUNCH_KD;
    pitchLaunchKp = PITCH_LAUNCH_KP;
    pitchLaunchKd = PITCH_LAUNCH_KD;
    yawLaunchKp = YAW_LAUNCH_KP;

    throttleIdle = THROTTLE_IDLE;
    throttleLaunchEnd = THROTTLE_LAUNCH_END;
    throttleFlightStart = THROTTLE_FLIGHT_START;

    yawFF = YAW_FF;

    maxPDOutput = MAX_PD_OUTPUT;
    maxIOutput = MAX_I_OUTPUT;

    trimDelay = TRIM_DELAY;
    trimStepPitchRollDeg = TRIM_STEP_PITCH_ROLL_DEG;
    trimStepYawDegPerSec = TRIM_STEP_YAW_DEG_PER_SEC;

    featureFlagEnableAltitudeReading = (FEATURE_FLAG_ALTITUDE != 0);
    featureFlagEnableDisplay = (FEATURE_FLAG_DISPLAY != 0);

    // Radio address: hardcoded; must match Synapse TX
    memcpy(radioAddress, "00001", 6);
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
PIDGains DroneConfig::getYawPID() const {
    return yawPID;
}

float DroneConfig::getYawKp() const {
    return yawPID.kp;
}

float DroneConfig::getYawKi() const {
    return yawPID.ki;
}

float DroneConfig::getYawKd() const {
    return yawPID.kd;
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
float DroneConfig::getYawLaunchKp() const {
    return yawLaunchKp;
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

float DroneConfig::getYawFF() const {
    return yawFF;
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

bool DroneConfig::getFeatureFlagEnableDisplay() const {
    return featureFlagEnableDisplay;
}

const byte* DroneConfig::getRadioAddress() const {
    return radioAddress;
}