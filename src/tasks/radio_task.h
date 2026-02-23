#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "../models/drone_command.h"
#include "../adapters/radio_adapter.h"

// Call once from setup() before startRadioTask().
void initRadioMutexes();

// Core 1: get latest command from core 0. Returns true if a new packet was available.
bool getLatestRadioCommand(DronePacket& outPacket, unsigned long& outLastPacketTime);

// Core 1: submit telemetry to be sent by core 0 (mutex-protected).
void submitTelemetry(int16_t pwm, int16_t roll, int16_t pitch);

// Start the radio task on the given core (e.g. 0). Runs at 1000 Hz.
void startRadioTask(RadioAdapter* radio, int coreID);
