#ifndef BME280_ADAPTER_H
#define BME280_ADAPTER_H

#include "barometer_adapter.h"
#include <Wire.h>

class Adafruit_BME280;

/** BME280 implementation of BarometerAdapter (I2C). */
class BME280BarometerAdapter : public BarometerAdapter {
public:
    explicit BME280BarometerAdapter(TwoWire* wire);
    ~BME280BarometerAdapter() override;

    bool begin() override;
    bool isConnected() const override;
    float readPressureHpa() override;
    float readAltitudeMeters(float seaLevelHpa = 1013.25f) override;

private:
    Adafruit_BME280* bme_;
    TwoWire* wire_;
    bool connected_;
};

#endif
