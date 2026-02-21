#include "lidar_readings.h"

LidarReadings::LidarReadings() {
    this->distanceCm = 0;
    this->signalStrength = 0;
    this->temperatureC = 0;
}

uint16_t LidarReadings::getDistanceCm() {
    return this->distanceCm;
}

bool LidarReadings::isValid() {
    return this->signalStrength > 100 && this->distanceCm < 1200;
}

uint16_t LidarReadings::getSignalStrength() {
    return this->signalStrength;
}

float LidarReadings::getTemperatureC() {
    return this->temperatureC;
}

void LidarReadings::setDistanceCm(uint16_t distanceCm) {
    this->distanceCm = distanceCm;
}

void LidarReadings::setSignalStrength(uint16_t signalStrength) {
    this->signalStrength = signalStrength;
}

void LidarReadings::setTemperatureC(float temperatureC) {
    this->temperatureC = temperatureC;
}