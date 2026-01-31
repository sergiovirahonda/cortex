#include <Arduino.h>
#include <MPU6050_light.h>
#include <math.h>
#include "mpu_adapter.h"
#include "../config/drone_config.h"

MPUAdapter::MPUAdapter(MPU6050 *mpu) {
    this->mpu = mpu;
}

void MPUAdapter::begin() {
    byte status = this->mpu->begin();
    while (status != 0) { } // stop if can't connect
    delay(1000);
    
}

void MPUAdapter::calibrate(const DroneConfig& droneConfig) {
    // TUNE SOFTWARE FILTER
    // 0.98 is default. 
    // Increase to 0.99 if you see angles jumping when motors spin.
    this->mpu->setFilterGyroCoef(0.995);
    // 1. LOAD SAVED ACCEL DATA
    // This ensures "Level" is always "Level", even if you boot on a hill.
    // Offset -> X, Y, Z
    // before -.11
    this->mpu->setAccOffsets(
        droneConfig.getAccelXOffset(),
        droneConfig.getAccelYOffset(),
        droneConfig.getAccelZOffset()
    );
    // 2. CALIBRATE GYRO ONLY (Every Boot)
    // This fixes drift. The drone must be STILL, but angle doesn't matter.
    // false = Don't touch Accel
    // true  = Do calibrate Gyro
    this->mpu->calcOffsets(true, false);
}

void MPUAdapter::update() {
    this->mpu->update();
}

void MPUAdapter::getAttitude(Attitude& attitude) {
    this->mpu->update();
    attitude.updateSensors(
        this->mpu->getAngleY(),
        this->mpu->getGyroY(),
        this->mpu->getAngleX(),
        this->mpu->getGyroX(),
        this->mpu->getGyroZ()
    );
}