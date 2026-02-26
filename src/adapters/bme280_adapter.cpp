#include "bme280_adapter.h"
#include <Adafruit_BME280.h>
#include <Arduino.h>

BME280BarometerAdapter::BME280BarometerAdapter(TwoWire* wire)
    : wire_(wire), connected_(false) {
    bme_ = new Adafruit_BME280();
}

BME280BarometerAdapter::~BME280BarometerAdapter() {
    delete bme_;
    bme_ = nullptr;
}

bool BME280BarometerAdapter::begin() {
    // BME280 I2C address: 0x77 (default) or 0x76
    connected_ = bme_->begin(0x77, wire_);
    if (!connected_) {
        connected_ = bme_->begin(0x76, wire_);
    }
    if (connected_) {
        bme_->setSampling(
            Adafruit_BME280::MODE_NORMAL,
            Adafruit_BME280::SAMPLING_X2,
            Adafruit_BME280::SAMPLING_X2,
            Adafruit_BME280::SAMPLING_X2,
            Adafruit_BME280::FILTER_OFF,
            Adafruit_BME280::STANDBY_MS_250
        );
        delay(50);  // let first conversion complete before any read
    }
    return connected_;
}

bool BME280BarometerAdapter::isConnected() const {
    return connected_;
}

float BME280BarometerAdapter::readPressureHpa() {
    if (!connected_) return 0.0f;
    float pPa = bme_->readPressure();
    return pPa / 100.0f;  // Pa -> hPa
}

float BME280BarometerAdapter::readAltitudeMeters(float seaLevelHpa) {
    if (!connected_) return 0.0f;
    return bme_->readAltitude(seaLevelHpa);
}
