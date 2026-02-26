#ifndef TF_LUNA_ADAPTER_H
#define TF_LUNA_ADAPTER_H

#include "lidar_adapter.h"

/** TF-Luna LiDAR over UART. Distance is filtered with a moving average. */
class TFLunaLidarAdapter : public LidarAdapter {
public:
    explicit TFLunaLidarAdapter(HardwareSerial* serial);

    void begin() override;
    void update() override;

    uint16_t getDistanceCm() const override { return distanceCm_; }
    uint16_t getSignalStrength() const override { return signalStrength_; }
    float getTemperatureC() const override { return temperatureC_; }

private:
    static const uint8_t FRAME_LEN = 9;
    static const uint8_t HEADER0 = 0x59;
    static const uint8_t HEADER1 = 0x59;
    static const uint8_t DIST_FILTER_SIZE = 8;

    HardwareSerial* serial_;
    uint16_t distanceCm_;
    uint16_t signalStrength_;
    float temperatureC_;
    uint16_t distBuf_[DIST_FILTER_SIZE];
    uint8_t distBufIdx_;
    uint8_t distBufCount_;
    uint32_t distSum_;
};

#endif
