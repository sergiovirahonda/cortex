#include "barometer_readings.h"

BarometerReadings::BarometerReadings()
    : pressureHpa_(0.0f), altitudeMeters_(0.0f), connected_(false) {}

float BarometerReadings::getPressureHpa() const {
    return pressureHpa_;
}

float BarometerReadings::getAltitudeMeters() const {
    return altitudeMeters_;
}

bool BarometerReadings::isConnected() const {
    return connected_;
}

void BarometerReadings::setPressureHpa(float hpa) {
    pressureHpa_ = hpa;
}

void BarometerReadings::setAltitudeMeters(float meters) {
    altitudeMeters_ = meters;
}

void BarometerReadings::setConnected(bool connected) {
    connected_ = connected;
}
