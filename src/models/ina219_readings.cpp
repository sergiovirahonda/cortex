#include "ina219_readings.h"

Ina219Readings::Ina219Readings()
    : busVoltageV_(0.0f), currentMa_(0.0f), powerMw_(0.0f), connected_(false) {}

float Ina219Readings::getBusVoltageV() const {
    return busVoltageV_;
}

float Ina219Readings::getCurrentMa() const {
    return currentMa_;
}

float Ina219Readings::getPowerMw() const {
    return powerMw_;
}

bool Ina219Readings::isConnected() const {
    return connected_;
}

void Ina219Readings::setBusVoltageV(float v) {
    busVoltageV_ = v;
}

void Ina219Readings::setCurrentMa(float ma) {
    currentMa_ = ma;
}

void Ina219Readings::setPowerMw(float mw) {
    powerMw_ = mw;
}

void Ina219Readings::setConnected(bool connected) {
    connected_ = connected;
}
