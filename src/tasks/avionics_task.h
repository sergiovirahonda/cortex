#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "../models/avionics_params.h"
#include "../models/avionics_metrics.h"

// Single global avionics state (written by avionics task, read via getAvionicsMetrics).
extern AvionicsMetrics currentAvionics;

// Call once from setup() before startAvionicsTask() and before loop() runs.
void initAvionicsMutex();

// Thread-safe copy for the main loop (cross-core). Blocks until mutex is available.
void getAvionicsMetrics(AvionicsMetrics& out);

void startAvionicsTask(AvionicsParams* params, int coreID);