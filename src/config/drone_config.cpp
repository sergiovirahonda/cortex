#include "drone_config.h"

DroneConfig::DroneConfig() {
    // =================================================================
    // ACCELEROMETER OFFSETS
    // =================================================================
    // These offsets ensure "Level" is always "Level", even if you boot on a hill
    // Commented out for now as it's not working
    // accelOffsets.xOffset = -0.03; //  Towards negative -> Tilt to the right
    // accelOffsets.yOffset = -0.07; // Towards negative -> Tilt to the front
    // accelOffsets.zOffset = -0.06; // MPU readings standard
    // Leveled offset values
    accelOffsets.xOffset = 0.04; //  Towards negative -> Tilt to the right
    accelOffsets.yOffset = -0.0098; // Towards negative -> Tilt to the front
    accelOffsets.zOffset = -0.06; // MPU readings standard
    
    // =================================================================
    // PID GAINS
    // =================================================================
    // Kp: The "Spring". Stiffens the drone.
    // Ki: The "Integral". Eliminates steady-state error.
    // Kd: The "Damper". Stops the bounce.
    // Rule of Thumb: Kp should be 10x to 20x larger than Kd in this setup.
    
    // Roll PID gains
    rollPID.kp = 2.0;
    rollPID.ki = 0.02;
    rollPID.kd = 0.20;
    
    // Pitch PID gains
    pitchPID.kp = 2.2;
    pitchPID.ki = 0.02;
    pitchPID.kd = 0.12;
    
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
    trimDelay = 500;
    trimStep = 0.10;
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

