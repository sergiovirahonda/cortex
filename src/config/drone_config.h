#ifndef DRONE_CONFIG_H
#define DRONE_CONFIG_H

#include <Arduino.h>

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
        
        // PID Gains (flight: low KP, full PID)
        PIDGains rollPID;
        PIDGains pitchPID;
        PIDGains yawPID;
        // Launch gains (high KP, PD only; plain floats, no struct)
        float rollLaunchKp;
        float rollLaunchKd;
        float pitchLaunchKp;
        float pitchLaunchKd;
        float yawLaunchKp;

        // Throttle ranges: idle, launch end, flight start (3 ranges: high KP, transition, low KP)
        int throttleIdle;
        int throttleLaunchEnd;
        int throttleFlightStart;

        // Yaw rate feedforward (stage 3 only): output += YAW_FF * desiredYawRate for snappier stick response
        float yawFF;
        // Yaw hold (stick centered): lock heading; Kp on heading error, deadzone in deg/s
        float yawHoldKp;
        float yawDeadzoneRateDps;
        
        // Output limits
        float maxPDOutput;
        float maxIOutput;
    
        // Trim configurations (degrees / deg/s; applied as setpoint offset)
        int trimDelay;
        float trimStepPitchRollDeg;  // degrees per step (pitch, roll)
        float trimStepYawDegPerSec;  // deg/s per step (yaw)
    
        // Altitude hold (LIDAR): PID gains and hover throttle (gravity cancellation guess)
        float altitudeKp;
        float altitudeKi;
        float altitudeKd;
        float altitudeHoverThrottle;
        float altitudeMaxCorrection;
        float altitudeFusionKp;
        float altitudeFusionKi;
        float altitudeMaxIOutput;
        float altitudeLidarMinCm;
        float altitudeLidarMaxCm;
        float altitudeLidarMinEngageCm;  // min altitude (cm) to allow engaging hold (e.g. 1 m for stable flight)
        float altitudeLidarMaxEngageCm;  // max altitude (cm) at which we allow engaging hold
    
        // Feature flags
        bool featureFlagEnableAltitudeReading;
        bool featureFlagEnableDisplay;

        // Radio (nRF24L01 pipe address; must match TX)
        byte radioAddress[6];
    
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
        PIDGains getYawPID() const;
        float getYawKp() const;
        float getYawKi() const;
        float getYawKd() const;

        // Launch gain getters (PD only)
        float getRollLaunchKp() const;
        float getRollLaunchKd() const;
        float getPitchLaunchKp() const;
        float getPitchLaunchKd() const;
        float getYawLaunchKp() const;

        // Throttle range getters
        int getThrottleIdle() const;
        int getThrottleLaunchEnd() const;
        int getThrottleFlightStart() const;

        float getYawFF() const;
        float getYawHoldKp() const;
        float getYawDeadzoneRateDps() const;

        // Output limit getters
        float getMaxPDOutput() const;
        float getMaxIOutput() const;
    
        // Trim settings getters
        int getTrimDelay() const;
        float getTrimStepPitchRollDeg() const;
        float getTrimStepYawDegPerSec() const;
    
         // Altitude hold getters
         float getAltitudeKp() const;
         float getAltitudeKi() const;
         float getAltitudeKd() const;
         int getAltitudeHoverThrottle() const;
         float getAltitudeMaxCorrection() const;
         float getAltitudeFusionKp() const;
         float getAltitudeFusionKi() const;
         float getAltitudeMaxIOutput() const;
         float getAltitudeLidarMinCm() const;
         float getAltitudeLidarMaxCm() const;
         float getAltitudeLidarMinEngageCm() const;
         float getAltitudeLidarMaxEngageCm() const;
    
        // Feature flags getters
        bool getFeatureFlagEnableAltitudeReading() const;
        bool getFeatureFlagEnableDisplay() const;

        // Radio address getter (for RadioAdapter; must match TX address)
        const byte* getRadioAddress() const;
};

#endif
