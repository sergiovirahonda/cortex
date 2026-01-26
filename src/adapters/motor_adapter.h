#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "driver/rmt.h" 
#include <Wire.h>

class NativeDShotMotorAdapter {
    private:
        rmt_channel_t _channel;
        rmt_item32_t _items[17];
    public:
        void init(int pin, rmt_channel_t channel);
        void sendThrottle(int throttle);
        void armSequence();
};

#endif