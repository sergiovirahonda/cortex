#ifndef AVIONICS_PARAMS_H
#define AVIONICS_PARAMS_H

#include "../adapters/lidar_adapter.h"
#include "../adapters/gps_adapter.h"
#include "../adapters/compass_adapter.h"
#include "../adapters/barometer_adapter.h"
#include "../adapters/current_sensor_adapter.h"

struct AvionicsParams {
    LidarAdapter* lidar;
    GpsAdapter* gps;
    CompassAdapter* compass;
    BarometerAdapter* barometer;
    CurrentSensorAdapter* currentSensor;
};

#endif
