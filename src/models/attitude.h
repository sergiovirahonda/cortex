#ifndef ATTITUDE_H
#define ATTITUDE_H

class Attitude {
private:
    float rollAngle, rollRate;
    float pitchAngle, pitchRate;
    float yawRate;
    
    // Memory variables for the I-Term
    float rollErrorSum = 0;
    float pitchErrorSum = 0;
    
    // Previous Time for accurate I-calculation (Optional but recommended)
    unsigned long lastTime = 0;

public:
        Attitude();
        void updateSensors(float rAngle, float rRate, float pAngle, float pRate, float yRate);
        // Raw getters
        float getRollAngle();
        float getRollRate();
        float getPitchAngle();
        float getPitchRate();
        float getYawRate();
        // Calculate PID Outputs
        float calculateRollPD(float desiredRollAngle);
        float calculatePitchPD(float desiredPitchAngle);
        float calculateYawP(float desiredYawRate);
        // Add a reset method (Critical for safety!)
        void resetPID() {
            rollErrorSum = 0;
            pitchErrorSum = 0;
        }
};

#endif