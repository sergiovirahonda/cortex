#include "flight_controller.h"
#include "models/motor_output.h"
#include "models/drone_command.h"

#include <Arduino.h>

// =================================================================
// CONTROL LIMITS (CONSTANTS)
// =================================================================
const int MIN_THROTTLE_PWM = 50;
const int MAX_THROTTLE_PWM = 2047;

FlightController::FlightController() {
    lastPacketTime = 0;
    inFailsafe = false;
    failsafeThrottle = FAILSAFE_LANDING_THROTTLE;
}

void FlightController::updateFailsafe(bool packetReceived, int16_t currentThrottle) {
    if (packetReceived) {
        lastPacketTime = millis();
        inFailsafe = false;
    } else {
        // Check for failsafe timeout
        if (lastPacketTime > 0) {
            unsigned long timeSinceLastPacket = millis() - lastPacketTime;
            if (timeSinceLastPacket > FAILSAFE_TIMEOUT_MS) {
                // Only activate failsafe if drone is actually flying (throttle >= 1000)
                // If on ground (throttle < 1000), keep current throttle and don't try to land
                if (currentThrottle >= FAILSAFE_LANDING_THROTTLE) {
                    inFailsafe = true;
                    
                    // Gradually reduce throttle to landing speed
                    if (failsafeThrottle > FAILSAFE_LANDING_THROTTLE) {
                        failsafeThrottle -= FAILSAFE_THROTTLE_DECAY;
                        if (failsafeThrottle < FAILSAFE_LANDING_THROTTLE) {
                            failsafeThrottle = FAILSAFE_LANDING_THROTTLE;
                        }
                    }
                } else {
                    // On ground - exit failsafe and keep current throttle
                    inFailsafe = false;
                    failsafeThrottle = currentThrottle;
                }
            }
        }
    }
}

bool FlightController::isInFailsafe() const {
    return inFailsafe;
}

void FlightController::applyFailsafe(DroneCommand& command) {
    if (inFailsafe) {
        // Override command with failsafe values (level attitude, reduced throttle)
        command.setThrottle(failsafeThrottle);
        command.setPitch(0);   // Level pitch
        command.setRoll(0);    // Level roll
        command.setYaw(0);     // No yaw rotation
    } else {
        // Track current throttle when not in failsafe
        // This captures the throttle value before entering failsafe
        failsafeThrottle = command.getThrottle();
    }
}

void FlightController::computeAttitudeCorrections(
    DroneCommand& command,
    Attitude& attitude,
    float& pitchPD,
    float& rollPD,
    float& yawPD
) {
    int16_t throttle = command.getThrottle();
    
    if (throttle < 60) {
        // Range 1: Low throttle - Reset PID and disable corrections
        // Prevents motors from twitching while on the ground
        attitude.resetPID();
        pitchPD = 0;
        rollPD  = 0;
        yawPD   = 0;
    } else if (throttle < 1100) {
        // Range 2: Medium throttle - PD control only (no I term)
        // Avoids learning incorrect errors from ground contact
        pitchPD = attitude.calculatePitchPD(command.getPitch(), false);
        rollPD  = attitude.calculateRollPD(command.getRoll(), false);
        yawPD   = attitude.calculateYawP(command.getYaw());
    } else {
        // Range 3: High throttle - Full PID control (with I term)
        // Only calculate full PID when actually flying
        pitchPD = attitude.calculatePitchPD(command.getPitch(), true);
        rollPD  = attitude.calculateRollPD(command.getRoll(), true);
        yawPD   = attitude.calculateYawP(command.getYaw());
    }
}

void FlightController::computeMotorOutput(
    MotorOutput& motorOutput,
    DroneCommand& command,
    AttitudeTrim& trim,
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
    int16_t pitchTrim = trim.getPitchTrim();
    int16_t rollTrim = trim.getRollTrim();
    int16_t yawTrim = trim.getYawTrim();

    pitchPD = pitchPD - pitchTrim;
    rollPD = rollPD - rollTrim;
    yawPD = yawPD - yawTrim;

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