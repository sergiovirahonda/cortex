#ifndef DRONE_CONFIG_H
#define DRONE_CONFIG_H

// Structure for accelerometer offset configuration
struct AccelerometerOffsetConfig {
    float xOffset;
    float yOffset;
    float zOffset;
};

// Structure for PID gains
struct PIDGains {
    float kp;
    float ki;
    float kd;
};

class DroneConfig {
    private:
        // Accelerometer offsets
        AccelerometerOffsetConfig accelOffsets;
        
        // PID Gains
        PIDGains rollPID;
        PIDGains pitchPID;
        float kpYaw;
        
        // Output limits
        float maxPDOutput;
        float maxIOutput;
    
        // Trim configurations
        int trimDelay;
        float trimStep;
    
        // Feature flags
        bool featureFlagEnableAltitudeReading;
    
    public:
        DroneConfig();
        
        // Accelerometer offset getters
        AccelerometerOffsetConfig getAccelOffsets() const;
        float getAccelXOffset() const;
        float getAccelYOffset() const;
        float getAccelZOffset() const;
        
        // Roll PID getters
        PIDGains getRollPID() const;
        float getRollKp() const;
        float getRollKi() const;
        float getRollKd() const;
        
        // Pitch PID getters
        PIDGains getPitchPID() const;
        float getPitchKp() const;
        float getPitchKi() const;
        float getPitchKd() const;
        
        // Yaw gain getters
        float getYawKp() const;
        
        // Output limit getters
        float getMaxPDOutput() const;
        float getMaxIOutput() const;
    
        // Trim settings getters
        int getTrimDelay() const;
        float getTrimStep() const;
    
        // Feature flags getters
        bool getFeatureFlagEnableAltitudeReading() const;
};

#endif
