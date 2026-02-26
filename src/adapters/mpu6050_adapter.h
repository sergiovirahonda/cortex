#ifndef MPU6050_ADAPTER_H
#define MPU6050_ADAPTER_H

#include "mpu_adapter.h"

class MPU6050;

/** MPU6050 (MPU6050_light library) implementation of MPUAdapter. */
class Mpu6050Adapter : public MPUAdapter {
public:
    explicit Mpu6050Adapter(MPU6050* mpu);
    ~Mpu6050Adapter() override;

    void begin() override;
    void calibrate(const DroneConfig& droneConfig) override;
    void update() override;

    void getAttitude(Attitude& attitude) override;
    void getRawAccelGs(Altitude& altitude) override;

private:
    MPU6050* mpu_;
};

#endif
