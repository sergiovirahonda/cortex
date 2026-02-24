#include "radio_task.h"
#include <freertos/semphr.h>

// --- Command snapshot (core 0 writes, core 1 reads) ---
static DronePacket latestPacket;
static unsigned long latestPacketTime = 0;
static bool commandHasNew = false;
static SemaphoreHandle_t commandMutex = NULL;

// --- Telemetry pending (core 1 writes, core 0 sends) ---
static TelemetryPacket pendingTelemetry;
static bool telemetryHasPending = false;
static SemaphoreHandle_t telemetryMutex = NULL;

static const int RADIO_LOOP_PERIOD_MS = 1;  // 1000 Hz

void initRadioMutexes() {
    if (commandMutex == NULL) {
        commandMutex = xSemaphoreCreateMutex();
    }
    if (telemetryMutex == NULL) {
        telemetryMutex = xSemaphoreCreateMutex();
    }
}

bool getLatestRadioCommand(DronePacket& outPacket, unsigned long& outLastPacketTime) {
    if (commandMutex == NULL) return false;
    if (xSemaphoreTake(commandMutex, portMAX_DELAY) != pdTRUE) return false;
    bool hadNew = commandHasNew;
    if (commandHasNew) {
        outPacket = latestPacket;
        outLastPacketTime = latestPacketTime;
        commandHasNew = false;
    }
    xSemaphoreGive(commandMutex);
    return hadNew;
}

void submitTelemetry(int16_t pwm, int16_t roll, int16_t pitch, bool altitudeHoldEngaged) {
    if (telemetryMutex == NULL) return;
    if (xSemaphoreTake(telemetryMutex, portMAX_DELAY) != pdTRUE) return;
    pendingTelemetry.pwm = pwm;
    pendingTelemetry.roll = roll;
    pendingTelemetry.pitch = pitch;
    pendingTelemetry.altitudeHold = altitudeHoldEngaged ? 1 : 0;
    telemetryHasPending = true;
    xSemaphoreGive(telemetryMutex);
}

static void RadioLoop(void* pvParameters) {
    RadioAdapter* radio = (RadioAdapter*)pvParameters;
    if (radio == NULL) return;

    for (;;) {
        // 1. Poll radio and update command snapshot
        DronePacket pkt;
        if (radio->receivePacket(pkt)) {
            if (commandMutex != NULL && xSemaphoreTake(commandMutex, portMAX_DELAY) == pdTRUE) {
                latestPacket = pkt;
                latestPacketTime = millis();
                commandHasNew = true;
                xSemaphoreGive(commandMutex);
            }
        }

        // 2. Send pending telemetry under mutex (core 0 only touches radio)
        if (telemetryMutex != NULL && xSemaphoreTake(telemetryMutex, portMAX_DELAY) == pdTRUE) {
            if (telemetryHasPending) {
                TelemetryData toSend(
                    pendingTelemetry.pwm,
                    pendingTelemetry.roll,
                    pendingTelemetry.pitch,
                    pendingTelemetry.altitudeHold
                );
                telemetryHasPending = false;
                xSemaphoreGive(telemetryMutex);
                radio->sendTelemetry(toSend);
            } else {
                xSemaphoreGive(telemetryMutex);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(RADIO_LOOP_PERIOD_MS));
    }
}

void startRadioTask(RadioAdapter* radio, int coreID) {
    xTaskCreatePinnedToCore(
        RadioLoop,
        "RadioTask",
        4096,
        radio,
        1,
        NULL,
        coreID
    );
}
