#ifndef BMP280_ADAPTER_H
#define BMP280_ADAPTER_H

#include "barometer_adapter.h"
#include <Wire.h>

class Adafruit_BMP280;

/** BMP280 implementation of BarometerAdapter (I2C). Wire is set in constructor; begin(addr) only. */
class BMP280BarometerAdapter : public BarometerAdapter {
public:
    explicit BMP280BarometerAdapter(TwoWire* wire);
    ~BMP280BarometerAdapter() override;

    bool begin() override;
    bool isConnected() const override;
    float readPressureHpa() override;
    float readAltitudeMeters(float seaLevelHpa = 1013.25f) override;

private:
    Adafruit_BMP280* bmp_;
    TwoWire* wire_;
    bool connected_;
};

#endif
