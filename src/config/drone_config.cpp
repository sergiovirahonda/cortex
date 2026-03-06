#include "drone_config.h"
#include "drone_config_defaults.h"
#include <cstring>

DroneConfig::DroneConfig() {
    // =================================================================
    // ACCELEROMETER OFFSETS (from build_flags or drone_config_defaults.h)
    // =================================================================
    accelOffsets.xOffset = ACCEL_X_OFFSET;
    accelOffsets.yOffset = ACCEL_Y_OFFSET;
    accelOffsets.zOffset = ACCEL_Z_OFFSET;
    filterGyroCoef = MPU_FILTER_GYRO_COEF;

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
    yawHoldKp = YAW_HOLD_KP;
    yawDeadzoneRateDps = YAW_DEADZONE_RATE_DPS;
    yawFusionWeight = YAW_FUSION_WEIGHT;

    altitudeKp = ALT_KP;
    altitudeKi = ALT_KI;
    altitudeKd = ALT_KD;
    altitudeHoverThrottle = ALT_HOVER_THROTTLE;
    altitudeMaxCorrection = ALT_MAX_CORRECTION;
    altitudeFusionKp = ALT_FUSION_KP;
    altitudeFusionKi = ALT_FUSION_KI;
    altitudeMaxIOutput = ALT_MAX_I_OUTPUT;
    altitudeLidarMinCm = ALT_LIDAR_MIN_CM;
    altitudeLidarMaxCm = ALT_LIDAR_MAX_CM;
    altitudeLidarMinEngageCm = ALT_LIDAR_MIN_ENGAGE_CM;
    altitudeLidarMaxEngageCm = ALT_LIDAR_MAX_ENGAGE_CM;

    maxPDOutput = MAX_PD_OUTPUT;
    maxIOutput = MAX_I_OUTPUT;

    trimDelay = TRIM_DELAY;
    trimStepPitchRollDeg = TRIM_STEP_PITCH_ROLL_DEG;
    trimStepYawDegPerSec = TRIM_STEP_YAW_DEG_PER_SEC;

    featureFlagEnableAltitudeReading = (FEATURE_FLAG_ALTITUDE != 0);
    featureFlagEnableDisplay = (FEATURE_FLAG_DISPLAY != 0);
    featureFlagCompassHeading = (FEATURE_FLAG_COMPASS_HEADING != 0);

    // Radio address: hardcoded; must match Synapse TX
    memcpy(radioAddress, "00001", 6);

    // Blackbox: from build_flags or defaults
    blackboxQueueLen = BLACKBOX_QUEUE_LEN;
    blackboxProcessRateMs = BLACKBOX_PROCESS_RATE_MS;
    strncpy(blackboxIndexFilename, BLACKBOX_INDEX_FILENAME, BLACKBOX_INDEX_FILENAME_MAX - 1);
    blackboxIndexFilename[BLACKBOX_INDEX_FILENAME_MAX - 1] = '\0';
    strncpy(blackboxLogPrefix, BLACKBOX_LOG_PREFIX, BLACKBOX_LOG_PREFIX_MAX - 1);
    blackboxLogPrefix[BLACKBOX_LOG_PREFIX_MAX - 1] = '\0';
    strncpy(blackboxLogSuffix, BLACKBOX_LOG_SUFFIX, BLACKBOX_LOG_SUFFIX_MAX - 1);
    blackboxLogSuffix[BLACKBOX_LOG_SUFFIX_MAX - 1] = '\0';
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

float DroneConfig::getFilterGyroCoef() const {
    return filterGyroCoef;
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

float DroneConfig::getYawHoldKp() const {
    return yawHoldKp;
}

float DroneConfig::getYawDeadzoneRateDps() const {
    return yawDeadzoneRateDps;
}

float DroneConfig::getYawFusionWeight() const {
    return yawFusionWeight;
}

// Altitude hold getters
float DroneConfig::getAltitudeKp() const {
    return altitudeKp;
}
float DroneConfig::getAltitudeKi() const {
    return altitudeKi;
}
float DroneConfig::getAltitudeKd() const {
    return altitudeKd;
}
int DroneConfig::getAltitudeHoverThrottle() const {
    return altitudeHoverThrottle;
}
float DroneConfig::getAltitudeMaxCorrection() const {
    return altitudeMaxCorrection;
}
float DroneConfig::getAltitudeFusionKp() const {
    return altitudeFusionKp;
}
float DroneConfig::getAltitudeFusionKi() const {
    return altitudeFusionKi;
}
float DroneConfig::getAltitudeMaxIOutput() const {
    return altitudeMaxIOutput;
}
float DroneConfig::getAltitudeLidarMinCm() const {
    return altitudeLidarMinCm;
}
float DroneConfig::getAltitudeLidarMaxCm() const {
    return altitudeLidarMaxCm;
}
float DroneConfig::getAltitudeLidarMinEngageCm() const {
    return altitudeLidarMinEngageCm;
}
float DroneConfig::getAltitudeLidarMaxEngageCm() const {
    return altitudeLidarMaxEngageCm;
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

bool DroneConfig::getFeatureFlagCompassHeading() const {
    return featureFlagCompassHeading;
}

const byte* DroneConfig::getRadioAddress() const {
    return radioAddress;
}

uint32_t DroneConfig::getBlackboxQueueLen() const {
    return blackboxQueueLen;
}

uint32_t DroneConfig::getBlackboxProcessRateMs() const {
    return blackboxProcessRateMs;
}

const char* DroneConfig::getBlackboxIndexFilename() const {
    return blackboxIndexFilename;
}

const char* DroneConfig::getBlackboxLogPrefix() const {
    return blackboxLogPrefix;
}

const char* DroneConfig::getBlackboxLogSuffix() const {
    return blackboxLogSuffix;
}