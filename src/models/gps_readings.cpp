#include "gps_readings.h"

GpsReadings::GpsReadings() {
    this->latitude = 0;
    this->longitude = 0;
    this->altitudeMeters = 0;
    this->satellites = 0;
    this->locationIsValid = false;
}

// Getters

double GpsReadings::getLatitude() {
    return this->latitude;
}

double GpsReadings::getLongitude() {
    return this->longitude;
}

double GpsReadings::getAltitudeMeters() {
    return this->altitudeMeters;
}

uint32_t GpsReadings::getSatellites() {
    return this->satellites;
}

bool GpsReadings::getLocationIsValid() {
    return this->locationIsValid;
}

// Setters

void GpsReadings::setLatitude(double latitude) {
    this->latitude = latitude;
}

void GpsReadings::setLongitude(double longitude) {
    this->longitude = longitude;
}

void GpsReadings::setAltitudeMeters(double altitudeMeters) {
    this->altitudeMeters = altitudeMeters;
}

void GpsReadings::setSatellites(uint32_t satellites) {
    this->satellites = satellites;
}

void GpsReadings::setLocationIsValid(bool locationIsValid) {
    this->locationIsValid = locationIsValid;
}

