#ifndef ATTITUDE_H
#define ATTITUDE_H

class Attitude {
    private:
        float rollAngle, rollRate;
        float pitchAngle, pitchRate;
        float yawRate;
        float yawPrevError;
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
        float calculateYawP(float desiredYawRate); // Renamed to P (D is overkill)
};

#endif