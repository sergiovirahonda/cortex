#ifndef BMP280_ADAPTER_H
#define BMP280_ADAPTER_H

#include <Wire.h>

class Adafruit_BME280;

class BME280Adapter {
public:
    BME280Adapter(TwoWire* wire);
    bool begin();
    bool isConnected() const;
    float readPressureHpa();
    float readAltitudeMeters(float seaLevelHpa = 1013.25f);

private:
    Adafruit_BME280* bme;
    TwoWire* wire;
    bool connected;
};

#endif
