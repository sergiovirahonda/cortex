#include "compass_readings.h"

CompassReadings::CompassReadings() {
    this->rawAzimuth = 0;
    this->smoothedAzimuth = 0;
}

// Setters

void CompassReadings::setRawAzimuth(int rawAzimuth) {
    this->rawAzimuth = rawAzimuth;
}

void CompassReadings::setSmoothedAzimuth(int smoothedAzimuth) {
    this->smoothedAzimuth = smoothedAzimuth;
}

// Getters

int CompassReadings::getRawAzimuth() {
    return this->rawAzimuth;
}

int CompassReadings::getSmoothedAzimuth() {
    return this->smoothedAzimuth;
}