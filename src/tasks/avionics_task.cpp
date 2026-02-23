#include "avionics_task.h"
#include "../models/avionics_params.h"
#include <freertos/semphr.h>

AvionicsMetrics currentAvionics;
static SemaphoreHandle_t avionicsMutex = NULL;

void initAvionicsMutex() {
    if (avionicsMutex == NULL) {
        avionicsMutex = xSemaphoreCreateMutex();
    }
}

void getAvionicsMetrics(AvionicsMetrics& out) {
    if (avionicsMutex != NULL) {
        xSemaphoreTake(avionicsMutex, portMAX_DELAY);
        out = currentAvionics;
        xSemaphoreGive(avionicsMutex);
    }
}

void AvionicsLoop(void* pvParameters) {
    AvionicsParams* params = (AvionicsParams*)pvParameters;
    if (params == NULL) return;

    for (;;) {
        // 1. Update hardware (slow – do outside the lock). Skip if adapter not present.
        if (params->lidar)  params->lidar->update();
        if (params->gps)    params->gps->update();
        if (params->compass) params->compass->update();

        // 2. Lock and update the master struct (fast, cross-core safe)
        if (avionicsMutex != NULL && xSemaphoreTake(avionicsMutex, portMAX_DELAY) == pdTRUE) {
            if (params->lidar) {
                currentAvionics.lidar.setDistanceCm(params->lidar->getDistanceCm());
                currentAvionics.lidar.setSignalStrength(params->lidar->getSignalStrength());
                currentAvionics.lidar.setTemperatureC(params->lidar->getTemperatureC());
            }
            if (params->gps) {
                currentAvionics.gps.setLatitude(params->gps->getLat());
                currentAvionics.gps.setLongitude(params->gps->getLng());
                currentAvionics.gps.setSatellites(params->gps->getSatellites());
                currentAvionics.gps.setLocationIsValid(params->gps->locationIsValid());
                currentAvionics.gps.setAltitudeMeters(params->gps->getAltitudeMeters());
            }
            if (params->compass) {
                currentAvionics.compass.setRawAzimuth(params->compass->getRawAzimuth());
                currentAvionics.compass.setSmoothedAzimuth(params->compass->getSmoothedAzimuth());
            }
            xSemaphoreGive(avionicsMutex);
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void startAvionicsTask(AvionicsParams* params, int coreID) {
    xTaskCreatePinnedToCore(
        AvionicsLoop,
        "AvionicsTask",
        4096,
        params,
        1,
        NULL,
        coreID
    );
}