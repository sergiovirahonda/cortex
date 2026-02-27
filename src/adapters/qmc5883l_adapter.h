#ifndef QMC5883L_ADAPTER_H
#define QMC5883L_ADAPTER_H

#include "compass_adapter.h"

class QMC5883LCompass;

/** QMC5883L magnetometer implementation of CompassAdapter (I2C). */
class QMC5883LCompassAdapter : public CompassAdapter {
public:
    QMC5883LCompassAdapter();
    ~QMC5883LCompassAdapter() override;

    void begin() override;
    void update() override;
    bool isHealthy() const override;
    int getRawAzimuth() override;
    int getSmoothedAzimuth() override;

private:
    QMC5883LCompass* compass_;
    float alpha_;
    int smoothedAzimuth_;
};

#endif
