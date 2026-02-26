#ifndef MPU_DRIVER_H
#define MPU_DRIVER_H

#include "../models/attitude.h"
#include "../models/altitude.h"
#include "../config/drone_config.h"

/**
 * IMU / attitude interface. Implementations (e.g. MPU6050) provide calibration,
 * update, and attitude/accel data for Attitude and Altitude models.
 */
class MPUAdapter {
public:
    virtual ~MPUAdapter() = default;

    virtual void begin() = 0;
    virtual void calibrate(const DroneConfig& droneConfig) = 0;
    virtual void update() = 0;

    virtual void getAttitude(Attitude& attitude) = 0;
    virtual void getRawAccelGs(Altitude& altitude) = 0;

protected:
    MPUAdapter() = default;
};

#endif
