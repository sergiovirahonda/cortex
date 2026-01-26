#include <Arduino.h>
#include "motor_adapter.h"

void NativeDShotMotorAdapter::init(int pin, rmt_channel_t channel) {
    _channel = channel;
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX((gpio_num_t)pin, channel);
    config.clk_div = 1;
    rmt_config(&config);
    rmt_driver_install(config.channel, 0, 0);
}

void NativeDShotMotorAdapter::sendThrottle(int throttle) {
    if (throttle > 2047) throttle = 2047;
    // Bump minimum to 48
    if (throttle < 50) throttle = 50;

    uint16_t safeThrottle = (uint16_t)throttle;
    
    uint16_t packet = (safeThrottle << 1) | 0; 
    int csum = 0;
    int csum_data = packet;
    for (int i = 0; i < 3; i++) {
        csum ^= (csum_data & 0x0F);
        csum_data >>= 4;
    }
    packet = (packet << 4) | (csum & 0x0F);
    
    for (int i = 0; i < 16; i++) {
        bool bit = (packet & 0x8000);
        packet <<= 1;
        if (bit) { _items[i] = {{{ 190, 1, 76, 0 }}}; } 
        else     { _items[i] = {{{ 95, 1, 171, 0 }}}; } 
    }
    _items[16] = {{{ 0, 0, 0, 0 }}}; 
    rmt_write_items(_channel, _items, 16, false);
}

void NativeDShotMotorAdapter::armSequence() {
    // 1. Safety Wait (Sends 0 to unlock ESCs) - 3 Seconds
    for(int i=0; i<300; i++) {
        sendThrottle(0);
        delay(10);
    }

    // 2. THE KICK (Sends 700 to break static friction) - 0.1 Seconds
    for(int i=0; i<20; i++) {
        sendThrottle(700); // The "Punch"
        delay(5);
    }

    // 3. Drop to Idle (Keeps them spinning)
    sendThrottle(200); 
}
