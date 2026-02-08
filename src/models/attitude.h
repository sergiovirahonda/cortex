#ifndef ATTITUDE_H
#define ATTITUDE_H

#include "../config/drone_config.h"

class AttitudeTrim {
    private:
        float rollTrim;
        float pitchTrim;
        float yawTrim;
    public:
        AttitudeTrim();
        void reset();
        void setRollTrim(float rollTrim);
        void setPitchTrim(float pitchTrim);
        void setYawTrim(float yawTrim);
        float getRollTrim();
        float getPitchTrim();
        float getYawTrim();
};

class Attitude {
private:
    float rollAngle, rollRate;
    float pitchAngle, pitchRate;
    float yawRate;
    
    // Memory variables for the I-Term
    float rollErrorSum = 0;
    float pitchErrorSum = 0;
    
    // Time for loop-rate-independent integral (dt in seconds)
    unsigned long lastTime = 0;
    float lastDt = 0.0f;  // dt used for this loop (shared by roll/pitch)

    DroneConfig droneConfig;

public:
        Attitude(const DroneConfig& droneConfig);
        void updateSensors(float rAngle, float rRate, float pAngle, float pRate, float yRate);
        // Raw getters
        float getRollAngle();
        float getRollRate();
        float getPitchAngle();
        float getPitchRate();
        float getYawRate();
        // Calculate PID Outputs (gainBlend: 0 = launch high KP, 1 = flight low KP; I only when enableI)
        float calculateRollPD(float desiredRollAngle, bool enableI, float gainBlend = 1.0f);
        float calculatePitchPD(float desiredPitchAngle, bool enableI, float gainBlend = 1.0f);
        float calculateYawP(float desiredYawRate);
        // Add a reset method (Critical for safety!)
        void resetPID() {
            rollErrorSum = 0;
            pitchErrorSum = 0;
        }
};

#endif