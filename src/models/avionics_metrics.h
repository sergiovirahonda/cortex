#ifndef AVIONICS_METRICS_H
#define AVIONICS_METRICS_H

#include "lidar_readings.h"
#include "gps_readings.h"
#include "compass_readings.h"

struct AvionicsMetrics {
    LidarReadings lidar;
    GpsReadings gps;
    CompassReadings compass;
    
    // In the future, we just add new things here:
    // BatteryReadings battery;
    // OpticalFlowReadings opticalFlow;
};

#endif