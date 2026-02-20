#ifndef COMPASS_READINGS_H
#define COMPASS_READINGS_H

class CompassReadings {
  private:
    int rawAzimuth;
    int smoothedAzimuth;
  public:
    CompassReadings();
    void setRawAzimuth(int rawAzimuth);
    void setSmoothedAzimuth(int smoothedAzimuth);
    int getRawAzimuth();
    int getSmoothedAzimuth();
};

#endif