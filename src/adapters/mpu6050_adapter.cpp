#include "mpu6050_adapter.h"
#include <Arduino.h>
#include <MPU6050_light.h>
#include "../config/drone_config.h"

Mpu6050Adapter::Mpu6050Adapter(MPU6050* mpu) : mpu_(mpu) {}

Mpu6050Adapter::~Mpu6050Adapter() {
    mpu_ = nullptr;
}

void Mpu6050Adapter::begin() {
    byte status = mpu_->begin();
    while (status != 0) { }
    delay(1000);
}

void Mpu6050Adapter::calibrate(const DroneConfig& droneConfig) {
    mpu_->setFilterGyroCoef(0.995);
    mpu_->setAccOffsets(
        droneConfig.getAccelXOffset(),
        droneConfig.getAccelYOffset(),
        droneConfig.getAccelZOffset()
    );
    mpu_->calcOffsets(true, false);
}

void Mpu6050Adapter::update() {
    mpu_->update();
}

void Mpu6050Adapter::getAttitude(Attitude& attitude) {
    mpu_->update();
    attitude.updateSensors(
        mpu_->getAngleY(),
        mpu_->getGyroY(),
        mpu_->getAngleX(),
        mpu_->getGyroX(),
        mpu_->getGyroZ()
    );
}

void Mpu6050Adapter::getRawAccelGs(Altitude& altitude) {
    altitude.setCurrentRawAccelerationGs(mpu_->getAccZ());
}
