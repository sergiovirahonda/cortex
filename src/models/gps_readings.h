#ifndef GPS_READINGS_H
#define GPS_READINGS_H

#include <cstdint>

class GpsReadings {
    private:
      double latitude;
      double longitude;
      double altitudeMeters;
      uint32_t satellites;
      bool locationIsValid;
    public:
      GpsReadings();
      double getLatitude();
      double getLongitude();
      double getAltitudeMeters();
      uint32_t getSatellites();
      bool getLocationIsValid();
      void setLatitude(double latitude);
      void setLongitude(double longitude);
      void setAltitudeMeters(double altitudeMeters);
      void setSatellites(uint32_t satellites);
      void setLocationIsValid(bool locationIsValid);
};

#endif