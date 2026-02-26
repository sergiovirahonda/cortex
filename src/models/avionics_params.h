#ifndef AVIONICS_PARAMS_H
#define AVIONICS_PARAMS_H

#include "../adapters/lidar_adapter.h"
#include "../adapters/gps_adapter.h"
#include "../adapters/compass_adapter.h"

// Struct to pack adapter pointers for FreeRTOS task parameter
struct AvionicsParams {
    LidarAdapter* lidar;
    GpsAdapter* gps;
    CompassAdapter* compass;
};

#endif
