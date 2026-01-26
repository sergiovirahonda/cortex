#ifndef DRONE_COMMAND_H
#define DRONE_COMMAND_H

#include <Arduino.h>

// --- THE WIRE FORMAT ---
// This struct is strictly for radio transmission.
// __attribute__((packed)) ensures no hidden gaps are added by the compiler.
struct DronePacket {
    int16_t pitch;
    int16_t roll;
    int16_t yaw;
    int16_t throttle;
} __attribute__((packed));

class DroneCommand {
    private:
        // Internal storage using fixed width types
        int16_t pitch, roll, yaw, throttle;

    public:
        DroneCommand(int16_t pitch, int16_t roll, int16_t yaw, int16_t throttle);

        // Getters
        int16_t getPitch();
        int16_t getRoll();
        int16_t getYaw();
        int16_t getThrottle();

        // Setters
        void setPitch(int16_t pitch);
        void setRoll(int16_t roll);
        void setYaw(int16_t yaw);
        void setThrottle(int16_t throttle);
        
        // Helpers
        void reset();
        void remap();
        
        // NEW: Export/Import for Radio
        DronePacket createPacket();
        void loadFromPacket(DronePacket packet);
};

#endif