#include "display_controller.h"

DisplayController::DisplayController(DisplayAdapter& display, const DroneConfig& config)
    : display_(display), config_(config) {}

void DisplayController::clearDisplay() {
    if (!enabled()) return;
    display_.clearDisplay();
}

void DisplayController::setCursor(int16_t x, int16_t y) {
    if (!enabled()) return;
    display_.setCursor(x, y);
}

void DisplayController::println(const char* s) {
    if (!enabled()) return;
    display_.println(s);
}

void DisplayController::display() {
    if (!enabled()) return;
    display_.display();
}

void DisplayController::displayFlightMetrics(
    float loopHz,
    Attitude& attitude,
    DroneCommand& command,
    MotorOutput& motorOutput,
    AttitudeTrim& attitudeTrim
) {
    if (!enabled()) return;

    display_.clearDisplay();
    display_.setCursor(0, 0);

    display_.print("- @ ");
    display_.print((int)loopHz);
    display_.println(" Hz -");

    display_.print("P: ");
    display_.print(attitude.getPitchAngle());
    display_.print(" | R: ");
    display_.println(attitude.getRollAngle());

    display_.print("Y: ");
    display_.print(attitude.getYawRate());
    display_.print(" | T: ");
    display_.println(command.getThrottle());

    display_.print("M1: ");
    display_.print(motorOutput.getMotor1Speed());
    display_.print(" | M2: ");
    display_.println(motorOutput.getMotor2Speed());
    display_.print("M3: ");
    display_.print(motorOutput.getMotor3Speed());
    display_.print(" | M4: ");
    display_.println(motorOutput.getMotor4Speed());

    display_.print("Trims: ");
    display_.print("P: ");
    display_.print(attitudeTrim.getPitchTrim());
    display_.print(" | R: ");
    display_.print(attitudeTrim.getRollTrim());
    display_.print(" | Y: ");
    display_.println(attitudeTrim.getYawTrim());

    display_.display();
}
