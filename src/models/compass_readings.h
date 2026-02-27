#ifndef COMPASS_READINGS_H
#define COMPASS_READINGS_H

class CompassReadings {
  private:
    int rawAzimuth;
    int smoothedAzimuth;
    bool healthy;
  public:
    CompassReadings();
    void setRawAzimuth(int rawAzimuth);
    void setSmoothedAzimuth(int smoothedAzimuth);
    void setCompassHealth(bool healthy);
    int getRawAzimuth();
    int getSmoothedAzimuth();
    bool getCompassHealth() const;
};

#endif