#include <Arduino.h>
#include "motor_adapter.h"

void NativeDShotMotorAdapter::init(int pin, rmt_channel_t channel) {
    _channel = channel;
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX((gpio_num_t)pin, channel);
    config.clk_div = 1;
    rmt_config(&config);
    rmt_driver_install(config.channel, 0, 0);
}

void NativeDShotMotorAdapter::sendThrottle(uint16_t throttle) {
    if (throttle > 2047) throttle = 2047;
    if (throttle > 0 && throttle < 48) throttle = 48; // Safety Lock
    sendPacket(throttle);
}

void NativeDShotMotorAdapter::sendCommand(uint16_t cmd) {
    if (cmd > 47) return; // Commands are only 0-47
    sendPacket(cmd);      // Send raw value (e.g. 21)
}

void NativeDShotMotorAdapter::sendPacket(uint16_t value) {
    uint16_t packet = (value << 1) | 0; 
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