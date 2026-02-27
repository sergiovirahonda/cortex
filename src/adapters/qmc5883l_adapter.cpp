#include "qmc5883l_adapter.h"
#include "../utils/angle_utils.h"
#include <QMC5883LCompass.h>
#include <math.h>

QMC5883LCompassAdapter::QMC5883LCompassAdapter()
    : alpha_(0.15f), smoothedAzimuth_(0) {
    compass_ = new QMC5883LCompass();
}

QMC5883LCompassAdapter::~QMC5883LCompassAdapter() {
    delete compass_;
    compass_ = nullptr;
}

void QMC5883LCompassAdapter::begin() {
    compass_->init();
}

bool QMC5883LCompassAdapter::isHealthy() const {
    return compass_ != nullptr;
}

void QMC5883LCompassAdapter::update() {
    compass_->read();
}

int QMC5883LCompassAdapter::getRawAzimuth() {
    return compass_->getAzimuth();
}

int QMC5883LCompassAdapter::getSmoothedAzimuth() {
    int rawAzimuth = compass_->getAzimuth();
    smoothedAzimuth_ = (int)((alpha_ * rawAzimuth) + ((1.0f - alpha_) * smoothedAzimuth_));
    return smoothedAzimuth_;
}
