#include "compass_adapter.h"
#include <Wire.h>
#include <QMC5883LCompass.h>

CompassAdapter::CompassAdapter() {
    this->alpha = 0.15; // 0.1 = Very smooth/slow, 0.9 = Fast/jittery
    this->smoothedAzimuth = 0;
}

void CompassAdapter::begin() {
    this->compass.init();
    // this->compass.setCalibration(-721, 1205, -1253, 876, -1046, 641);
}

void CompassAdapter::update() {
    this->compass.read();
}

int CompassAdapter::getRawAzimuth() {
    return this->compass.getAzimuth();
}

int CompassAdapter::getSmoothedAzimuth() {
    // Apply Low-Pass Filter to stop the 10-degree "jitter"
    // Formula: NewValue = (Alpha * Current) + ((1 - Alpha) * Previous)
    int rawAzimuth = this->compass.getAzimuth();
    this->smoothedAzimuth = (this->alpha * rawAzimuth) + ((1.0 - this->alpha) * this->smoothedAzimuth);
    return this->smoothedAzimuth;
}