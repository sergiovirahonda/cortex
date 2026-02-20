#ifndef COMPASS_ADAPTER_H
#define COMPASS_ADAPTER_H

#include <Wire.h>
#include <QMC5883LCompass.h>

class CompassAdapter {
  private:
    QMC5883LCompass compass;
    float alpha;
    int smoothedAzimuth;
    
  public:
    CompassAdapter();
    void begin();
    void update();
    int getRawAzimuth();
    int getSmoothedAzimuth();
};

#endif