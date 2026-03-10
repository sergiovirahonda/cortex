#include "bmp280_adapter.h"
#include <Adafruit_BMP280.h>
#include <Arduino.h>

BMP280BarometerAdapter::BMP280BarometerAdapter(TwoWire* wire)
    : wire_(wire), connected_(false) {
    bmp_ = new Adafruit_BMP280(wire_);
}

BMP280BarometerAdapter::~BMP280BarometerAdapter() {
    delete bmp_;
    bmp_ = nullptr;
}

bool BMP280BarometerAdapter::begin() {
    // 1. DEFENSIVE CHECK: Never dereference a null pointer
    if (bmp_ == nullptr || wire_ == nullptr) {
        connected_ = false;
        return false;
    }

    // 2. The standard Address Fallback
    connected_ = bmp_->begin(0x77);
    if (!connected_) {
        connected_ = bmp_->begin(0x76);
    }
    
    // 3. Sensor Configuration
    if (connected_) {
        bmp_->setSampling(
            Adafruit_BMP280::MODE_NORMAL,
            Adafruit_BMP280::SAMPLING_X2,
            Adafruit_BMP280::SAMPLING_X2,
            Adafruit_BMP280::FILTER_OFF,
            Adafruit_BMP280::STANDBY_MS_250
        );
        delay(50);  // let first conversion complete before any read
    }
    
    return connected_;
}

bool BMP280BarometerAdapter::isConnected() const {
    return connected_;
}

float BMP280BarometerAdapter::readPressureHpa() {
    if (!connected_) return 0.0f;
    float pPa = bmp_->readPressure();
    return pPa / 100.0f;  // Pa -> hPa
}

float BMP280BarometerAdapter::readAltitudeMeters(float seaLevelHpa) {
    if (!connected_) return 0.0f;
    return bmp_->readAltitude(seaLevelHpa);
}
