#ifndef TF_LUNA_ADAPTER_H
#define TF_LUNA_ADAPTER_H

#include <Arduino.h>

/**
 * TF-Luna LiDAR over UART. Call update() each loop, then getDistanceCm(), getSignalStrength(), etc.
 * Default baud: 115200. Distance is filtered with a moving average to reduce noise.
 */
class TFLunaAdapter {
public:
    TFLunaAdapter(HardwareSerial* serial);
    void begin();   // call after serial->begin(115200, ...)
    void update();  // call every loop

    uint16_t getDistanceCm() const { return distanceCm_; }   // moving-average filtered
    uint16_t getSignalStrength() const { return signalStrength_; }
    float getTemperatureC() const { return temperatureC_; }

private:
    static const uint8_t FRAME_LEN = 9;
    static const uint8_t HEADER0 = 0x59;
    static const uint8_t HEADER1 = 0x59;
    static const uint8_t DIST_FILTER_SIZE = 8;

    HardwareSerial* serial_;
    uint16_t distanceCm_;      // filtered output
    uint16_t signalStrength_;
    float temperatureC_;
    uint16_t distBuf_[DIST_FILTER_SIZE];
    uint8_t distBufIdx_;
    uint8_t distBufCount_;
    uint32_t distSum_;
};

#endif
