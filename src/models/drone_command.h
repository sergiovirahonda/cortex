#ifndef DRONE_COMMAND_H
#define DRONE_COMMAND_H

#include <Arduino.h>

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
} __attribute__((packed));

class DroneCommand {
    private:
        // Internal storage using fixed width types
        // Drone control commands
        int16_t pitch, roll, yaw, throttle;
        // Drone trim commands
        int16_t rollTrim, pitchTrim, yawTrim;
        uint8_t trimReset;

    public:
        DroneCommand(
            int16_t pitch,
            int16_t roll,
            int16_t yaw,
            int16_t throttle,
            int16_t rollTrim,
            int16_t pitchTrim,
            int16_t yawTrim,
            uint8_t trimReset
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
        // Helpers
        void reset();
        void remap();
        
        // NEW: Export/Import for Radio
        DronePacket createPacket();
        void loadFromPacket(DronePacket packet);
};

#endif