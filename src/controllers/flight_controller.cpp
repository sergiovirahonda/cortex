#include "flight_controller.h"
#include "models/motor_output.h"
#include "models/drone_command.h"

#include <Arduino.h>

// =================================================================
// CONTROL LIMITS (CONSTANTS)
// =================================================================
const int MIN_THROTTLE_PWM = 50;
const int MAX_THROTTLE_PWM = 2047;

FlightController::FlightController() {};

void FlightController::computeMotorOutput(
    MotorOutput& motorOutput,
    DroneCommand& command,
    float rollPD,
    float pitchPD,
    float yawPD
) {
    
    // =============================================================
    // MIXING MATRIX (Specific for REVERSED setup)
    // =============================================================
    // PITCH: Nose Down -> Front Motors (M4, M2) Increase
    // ROLL:  Left Side Down -> Left Motors (M4, M3) Increase
    // YAW:   Turn Right (CW) -> CW Motors (M2, M3) Increase

    int16_t throttle = command.getThrottle();

    float yaw_trim = 0.0; 
    yawPD = yawPD + yaw_trim;

    pitchPD = pitchPD - 30.0;

    // M4 (Front Left) - CCW
    int motor4Speed = throttle + pitchPD - rollPD + yawPD;
    // M2 (Front Right) - CW
    int motor2Speed = throttle + pitchPD + rollPD - yawPD;
    // M3 (Rear Left) - CW
    int motor3Speed = throttle - pitchPD - rollPD - yawPD;
    // M1 (Rear Right) - CCW
    int motor1Speed = throttle - pitchPD + rollPD + yawPD;

    // --- FINAL CLAMPING (Safety Check) ---
    // Ensures PWM values are within the acceptable range for the ESCs
    motor1Speed = constrain(motor1Speed, MIN_THROTTLE_PWM, MAX_THROTTLE_PWM);
    motor2Speed = constrain(motor2Speed, MIN_THROTTLE_PWM, MAX_THROTTLE_PWM);
    motor3Speed = constrain(motor3Speed, MIN_THROTTLE_PWM, MAX_THROTTLE_PWM);
    motor4Speed = constrain(motor4Speed, MIN_THROTTLE_PWM, MAX_THROTTLE_PWM);

    motorOutput.setMotor1Speed(motor1Speed);
    motorOutput.setMotor2Speed(motor2Speed);
    motorOutput.setMotor3Speed(motor3Speed);
    motorOutput.setMotor4Speed(motor4Speed);
}