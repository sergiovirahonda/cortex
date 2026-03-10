#ifndef AVIONICS_METRICS_H
#define AVIONICS_METRICS_H

#include "lidar_readings.h"
#include "gps_readings.h"
#include "compass_readings.h"
#include "barometer_readings.h"
#include "ina219_readings.h"

struct AvionicsMetrics {
    LidarReadings lidar;
    GpsReadings gps;
    CompassReadings compass;
    BarometerReadings barometer;
    Ina219Readings currentSensor;
};

#endif