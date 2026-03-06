#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>

class BlackboxController;

void startBlackboxTask(BlackboxController* controller, int coreID);
