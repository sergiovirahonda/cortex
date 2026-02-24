#ifndef MPU_DRIVER_H
#define MPU_DRIVER_H

#include "../models/attitude.h"
#include "../models/altitude.h"
#include "../config/drone_config.h"
#include <MPU6050_light.h>
#include <Wire.h>

class MPUAdapter{
    public:
        MPUAdapter(MPU6050 *mpu);
        void begin();
        void calibrate(const DroneConfig& droneConfig);
        void update();

        void getAttitude(Attitude& attitude);
        void getRawAccelGs(Altitude& altitude);

    private:
        MPU6050 *mpu;
};

#endif