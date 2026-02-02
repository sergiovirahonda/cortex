#include "bmp280_adapter.h"
#include <Adafruit_BME280.h>
#include <Arduino.h>

BME280Adapter::BME280Adapter(TwoWire* wire) : wire(wire), connected(false) {
    bme = new Adafruit_BME280();
}

bool BME280Adapter::begin() {
    // BME280 I2C address: 0x77 (default) or 0x76
    connected = bme->begin(0x77, wire);
    if (!connected) {
        connected = bme->begin(0x76, wire);
    }
    if (connected) {
        bme->setSampling(
            Adafruit_BME280::MODE_NORMAL,
            Adafruit_BME280::SAMPLING_X2,
            Adafruit_BME280::SAMPLING_X2,
            Adafruit_BME280::SAMPLING_X2,
            Adafruit_BME280::FILTER_OFF,
            Adafruit_BME280::STANDBY_MS_250
        );
        delay(50);  // let first conversion complete before any read
    }
    return connected;
}

bool BME280Adapter::isConnected() const {
    return connected;
}

float BME280Adapter::readPressureHpa() {
    if (!connected) return 0.0f;
    float pPa = bme->readPressure();
    return pPa / 100.0f;  // Pa -> hPa
}

float BME280Adapter::readAltitudeMeters(float seaLevelHpa) {
    if (!connected) return 0.0f;
    return bme->readAltitude(seaLevelHpa);
}
