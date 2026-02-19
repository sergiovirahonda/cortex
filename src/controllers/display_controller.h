#ifndef DISPLAY_CONTROLLER_H
#define DISPLAY_CONTROLLER_H

#include "../config/drone_config.h"
#include "../adapters/display_adapter.h"
#include "../models/attitude.h"
#include "../models/drone_command.h"
#include "../models/motor_output.h"

/**
 * Keeps main.cpp clean: gates display by feature flag, centralizes what to draw.
 * When display disabled, all methods no-op. Simple status uses the adapter via clearDisplay/setCursor/println/display.
 * Flight telemetry is all in displayFlightMetrics() which uses the adapter directly.
 */
class DisplayController {
public:
    DisplayController(DisplayAdapter& display, const DroneConfig& config);

    /** Thin wrappers around adapter (no-op when display disabled). For setup/status messages. */
    void clearDisplay();
    void setCursor(int16_t x, int16_t y);
    void println(const char* s);
    /** Push buffer to screen. */
    void display();

    /**
     * Full flight telemetry screen; uses adapter directly. No-op when display disabled.
     */
    void displayFlightMetrics(
        float loopHz,
        Attitude& attitude,
        DroneCommand& command,
        MotorOutput& motorOutput,
        AttitudeTrim& attitudeTrim
    );

private:
    DisplayAdapter& display_;
    const DroneConfig& config_;
    bool enabled() const { return config_.getFeatureFlagEnableDisplay(); }
};

#endif
