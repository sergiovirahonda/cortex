#include "blackbox_task.h"
#include "../controllers/blackbox_controller.h"
#include <freertos/task.h>

static void BlackboxLoop(void* pvParameters) {
    BlackboxController* controller = (BlackboxController*)pvParameters;
    if (controller == nullptr) return;

    const uint32_t rateMs = controller->getProcessRateMs();
    const TickType_t delayTicks = rateMs > 0 ? pdMS_TO_TICKS(rateMs) : 1;

    for (;;) {
        controller->processNextFrame();
        vTaskDelay(delayTicks);
    }
}

void startBlackboxTask(BlackboxController* controller, int coreID) {
    xTaskCreatePinnedToCore(
        BlackboxLoop,
        "BlackboxTask",
        4096,
        controller,
        0,  // Priority 0 (lowest; radio and avionics are 1)
        NULL,
        coreID
    );
}
