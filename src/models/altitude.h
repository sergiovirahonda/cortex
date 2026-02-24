#ifndef ALTITUDE_H
#define ALTITUDE_H

#include "../config/drone_config.h"

class AltitudeHold {
    private:
        bool altitudeHoldEngaged;
        float targetAltitudeCm;
    public:
        AltitudeHold();
        void reset();
        void setAltitudeHoldEngaged(bool altitudeHoldEngaged);
        void setTargetAltitudeCm(float targetAltitudeCm);
        bool getAltitudeHoldEngaged();
        float getTargetAltitudeCm();
};

class Altitude {
private:
    // --- State Variables ---
    // The fused, mathematically corrected reality of where the drone is
    float currentAltitudeCm;
    float currentVelocityCmS;
    float currentRawAccelerationGs;
    
    // --- PID Memory ---
    // Accumulator for the I-Term
    float altitudeErrorSum;
    
    // --- Integration Timing ---
    // Required for tracking dt (Delta Time) across sensor updates and PID math
    unsigned long lastTime;
    float lastDt;

    // --- Configuration ---
    // Holds the Kp, Ki, Kd and fusion constants
    DroneConfig droneConfig;

public:
    Altitude(const DroneConfig& droneConfig);

    // =================================================================
    // SENSOR FUSION & STATE UPDATE
    // =================================================================
    // Takes raw Z-acceleration (in G's), current tilt angles, and raw LiDAR.
    // Calculates Earth-frame trig, integrates velocity, and applies the filter.
    void setCurrentRawAccelerationGs(float rawAccelZ_Gs);
    void updateSensors(float pitchDeg, float rollDeg, float lidarAltCm);
    
    // =================================================================
    // RAW GETTERS
    // =================================================================
    float getCurrentRawAccelerationGs();
    float getCurrentAltitudeCm();
    float getCurrentVelocityCmS();

    // =================================================================
    // STABILIZATION LOGIC (PID)
    // =================================================================
    // Compares current state to desired state and returns the raw PWM correction.
    // Note: The 'I' term is always active here. The Mixer handles windup prevention
    // by only calling this when actually in Stage 3 flight.
    float calculateAltitudePID(float desiredAltitudeCm);

    // =================================================================
    // SAFETY CONTROLS
    // =================================================================
    // Must be called by the mixer when AltHold is disengaged to kill the I-Term.
    void resetPID();
    void reset();
};

#endif