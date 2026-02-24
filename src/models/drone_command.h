#ifndef DRONE_COMMAND_H
#define DRONE_COMMAND_H

#include <Arduino.h>
#include <cstdint>

// --- THE WIRE FORMAT ---
// This struct is strictly for radio transmission.
// __attribute__((packed)) ensures no hidden gaps are added by the compiler.
struct DronePacket {
    // Drone control commands
    int16_t pitch;
    int16_t roll;
    int16_t yaw;
    int16_t throttle;
    // Drone trim commands
    int16_t rollTrim;
    int16_t pitchTrim;
    int16_t yawTrim;
    uint8_t trimReset;
    // Altitude hold: -1 = disable, 0 = normal, 1 = enable (int8_t so -1 is native on wire)
    int8_t altitudeHold;
} __attribute__((packed));

class DroneCommand {
    private:
        // Internal storage using fixed width types
        // Drone control commands
        int16_t pitch, roll, yaw, throttle;
        // Drone trim commands
        int16_t rollTrim, pitchTrim, yawTrim;
        uint8_t trimReset;
        // Altitude hold commands: -1 = disengage, 0 = no op, 1 = engage
        int8_t altitudeHold;

    public:
        DroneCommand(
            int16_t pitch,
            int16_t roll,
            int16_t yaw,
            int16_t throttle,
            int16_t rollTrim,
            int16_t pitchTrim,
            int16_t yawTrim,
            uint8_t trimReset,
            int8_t altitudeHold
        );

        // Getters - Control Commands
        int16_t getPitch();
        int16_t getRoll();
        int16_t getYaw();
        int16_t getThrottle();

        // Getters - Trim Commands
        int16_t getRollTrim();
        int16_t getPitchTrim();
        int16_t getYawTrim();
        uint8_t getTrimReset();

        // Getters - Altitude Hold Commands
        int8_t getAltitudeHold();

        // Setters - Control Commands
        void setPitch(int16_t pitch);
        void setRoll(int16_t roll);
        void setYaw(int16_t yaw);
        void setThrottle(int16_t throttle);

        // Setters - Trim Commands
        void setRollTrim(int16_t rollTrim);
        void setPitchTrim(int16_t pitchTrim);
        void setYawTrim(int16_t yawTrim);
        void setTrimReset(uint8_t trimReset);

        // Setters - Altitude Hold Commands
        void setAltitudeHold(int8_t altitudeHold);

        // Helpers
        void reset();
        void remap();
        
        // NEW: Export/Import for Radio
        DronePacket createPacket();
        void loadFromPacket(DronePacket packet);
};

// --- TELEMETRY WIRE FORMAT (7 bytes: pwm, roll, pitch, altitudeHold) ---
// Roll/pitch are sent as angle_degrees * TELEMETRY_ANGLE_SCALE; TX displays as value/100.0
const int TELEMETRY_ANGLE_SCALE = 100;

struct TelemetryPacket {
    int16_t pwm;   // Throttle (command)
    int16_t roll;  // Roll angle (degrees * TELEMETRY_ANGLE_SCALE)
    int16_t pitch; // Pitch angle (degrees * TELEMETRY_ANGLE_SCALE)
    uint8_t altitudeHold; // 1 = altitude hold engaged, 0 = not
} __attribute__((packed));

class TelemetryData {
private:
    int16_t pwm;
    int16_t roll;
    int16_t pitch;
    uint8_t altitudeHold;

public:
    TelemetryData(int16_t pwm, int16_t roll, int16_t pitch, uint8_t altitudeHold = 0);

    int16_t getPwm();
    int16_t getRoll();
    int16_t getPitch();
    uint8_t getAltitudeHold();

    void setPwm(int16_t pwm);
    void setRoll(int16_t roll);
    void setPitch(int16_t pitch);
    void setAltitudeHold(uint8_t altitudeHold);

    void reset();

    TelemetryPacket createPacket();
    void loadFromPacket(TelemetryPacket packet);
};

#endif