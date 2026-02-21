#ifndef LIDAR_READINGS_H
#define LIDAR_READINGS_H

#include <cstdint>

class LidarReadings {
    private:
      uint16_t distanceCm;
      uint16_t signalStrength;
      float temperatureC;

    public:
      LidarReadings();
      uint16_t getDistanceCm();
      uint16_t getSignalStrength();
      bool isValid();
      float getTemperatureC();
      void setDistanceCm(uint16_t distanceCm);
      void setSignalStrength(uint16_t signalStrength);
      void setTemperatureC(float temperatureC);
};

#endif