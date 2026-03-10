#include "ina219_current_sensor_adapter.h"
#include <Adafruit_INA219.h>
#include <Arduino.h>

Ina219CurrentSensorAdapter::Ina219CurrentSensorAdapter(TwoWire* wire)
    : wire_(wire), connected_(false) {
    ina_ = new Adafruit_INA219(0x40);
}

Ina219CurrentSensorAdapter::~Ina219CurrentSensorAdapter() {
    delete ina_;
    ina_ = nullptr;
}

bool Ina219CurrentSensorAdapter::begin() {
    connected_ = ina_->begin(wire_);
    if (connected_) {
        ina_->setCalibration_32V_2A();
    }
    return connected_;
}

bool Ina219CurrentSensorAdapter::isConnected() const {
    return connected_;
}

float Ina219CurrentSensorAdapter::getBusVoltageV() {
    if (!connected_) return 0.0f;
    return ina_->getBusVoltage_V();
}

float Ina219CurrentSensorAdapter::getCurrentMa() {
    if (!connected_) return 0.0f;
    return ina_->getCurrent_mA();
}

float Ina219CurrentSensorAdapter::getPowerMw() {
    if (!connected_) return 0.0f;
    return ina_->getPower_mW();
}
