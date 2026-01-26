#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "driver/rmt.h" 
#include <Wire.h>

class NativeDShotMotorAdapter {
    private:
        rmt_channel_t _channel;
        rmt_item32_t _items[17];
        // Shared Logic (To avoid code duplication)
        void sendPacket(uint16_t value);
    public:
        void init(int pin, rmt_channel_t channel);
        void sendThrottle(uint16_t throttle);
        void sendCommand(uint16_t cmd);
};

#endif