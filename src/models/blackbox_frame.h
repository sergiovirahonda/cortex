#ifndef BLACKBOX_FRAME_H
#define BLACKBOX_FRAME_H

#include <Arduino.h>
#include <cstdint>

#include "attitude.h"
#include "drone_command.h"
#include "avionics_metrics.h"
#include "motor_output.h"

// Snapshot structs: copy of the data we log from each domain (by value, no pointers).

struct AttitudeSnapshot {
    float rollAngle;
    float rollRate;
    float pitchAngle;
    float pitchRate;
    float yawRate;
    float headingDeg;
    bool headingInitialized;
    bool onGround;
    float rollErrorSum;
    float pitchErrorSum;
    float yawErrorSum;
};

struct CommandSnapshot {
    int16_t pitch;
    int16_t roll;
    int16_t yaw;
    int16_t throttle;
    int16_t rollTrim;
    int16_t pitchTrim;
    int16_t yawTrim;
    uint8_t trimReset;
    int8_t altitudeHold;
};

struct AltitudeSnapshot {
    float relativeAltitudeCm;   // LiDAR (raw distance)
    float absoluteAltitudeM;    // Barometer (m over sea level)
};

struct GpsSnapshot {
    double latitude;
    double longitude;
    uint32_t satellites;
    bool valid;
    double altitudeMeters;
};

struct CompassSnapshot {
    float azimuthDeg;  // smoothed
    bool available;
};

struct MotorSnapshot {
    int m1;
    int m2;
    int m3;
    int m4;
};

struct BlackboxFrame {
    uint32_t timestampMs;
    AttitudeSnapshot attitude;
    CommandSnapshot command;
    AltitudeSnapshot altitude;
    GpsSnapshot gps;
    CompassSnapshot compass;
    MotorSnapshot motors;

    /** Fill this frame from the main loop models. Call from flight loop then enqueue. */
    void populate(
        const Attitude& attitude,
        const DroneCommand& command,
        const AvionicsMetrics& avionics,
        const MotorOutput& motors,
        uint32_t timestampMs);
};

#endif
