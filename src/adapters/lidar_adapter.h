#ifndef LIDAR_ADAPTER_H
#define LIDAR_ADAPTER_H

#include <Arduino.h>

/**
 * LiDAR interface. Call update() each loop, then getDistanceCm(), getSignalStrength(), etc.
 * Implementations (e.g. TF-Luna) provide filtered distance and optional signal/temperature.
 */
class LidarAdapter {
public:
    virtual ~LidarAdapter() = default;

    virtual void begin() = 0;   // call after serial->begin(115200, ...) for UART-based LiDARs
    virtual void update() = 0;  // call every loop

    virtual uint16_t getDistanceCm() const = 0;
    virtual uint16_t getSignalStrength() const = 0;
    virtual float getTemperatureC() const = 0;

protected:
    LidarAdapter() = default;
};

#endif
