#include "compass_readings.h"

CompassReadings::CompassReadings() {
    this->rawAzimuth = 0;
    this->smoothedAzimuth = 0;
    this->healthy = false;
}

// Setters

void CompassReadings::setRawAzimuth(int rawAzimuth) {
    this->rawAzimuth = rawAzimuth;
}

void CompassReadings::setSmoothedAzimuth(int smoothedAzimuth) {
    this->smoothedAzimuth = smoothedAzimuth;
}

void CompassReadings::setCompassHealth(bool healthy) {
    this->healthy = healthy;
}

// Getters

int CompassReadings::getRawAzimuth() {
    return this->rawAzimuth;
}

int CompassReadings::getSmoothedAzimuth() {
    return this->smoothedAzimuth;
}

bool CompassReadings::getCompassHealth() const {
    return this->healthy;
}