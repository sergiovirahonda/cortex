#include "flight_controller.h"
#include "models/motor_output.h"
#include "models/drone_command.h"
#include "config/drone_config.h"

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
    lastTrimTime = 0;
}

void FlightController::updateTrims(DroneCommand& command, AttitudeTrim& attitudeTrim, const DroneConfig& droneConfig) {
    if (millis() - lastTrimTime <= (unsigned long)droneConfig.getTrimDelay()) {
        return;
    }
    lastTrimTime = millis();

    float stepDeg = droneConfig.getTrimStepPitchRollDeg();   // degrees (pitch, roll)
    float stepYaw = droneConfig.getTrimStepYawDegPerSec();  // deg/s (yaw)

    // Pitch trim: -1 = FWD (Nose Down), 1 = BACK (Nose Up). +pitchTrim -> desiredPitch more positive -> nose down.
    if (command.getPitchTrim() == -1) {
        attitudeTrim.setPitchTrim(attitudeTrim.getPitchTrim() - stepDeg);
    } else if (command.getPitchTrim() == 1) {
        attitudeTrim.setPitchTrim(attitudeTrim.getPitchTrim() + stepDeg);
    }

    // Roll trim: -1 = LEFT, 1 = RIGHT. +rollTrim -> desiredRoll more positive -> roll right; so trim left = subtract.
    if (command.getRollTrim() == -1) {
        attitudeTrim.setRollTrim(attitudeTrim.getRollTrim() - stepDeg);
    } else if (command.getRollTrim() == 1) {
        attitudeTrim.setRollTrim(attitudeTrim.getRollTrim() + stepDeg);
    }

    // Yaw trim: 1 = CW, -1 = CCW. Trim in deg/s (rate).
    if (command.getYawTrim() == 1) {
        attitudeTrim.setYawTrim(attitudeTrim.getYawTrim() + stepYaw);
    } else if (command.getYawTrim() == -1) {
        attitudeTrim.setYawTrim(attitudeTrim.getYawTrim() - stepYaw);
    }

    if (command.getTrimReset() == 1) {
        attitudeTrim.reset();
    }
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

// Throttle bands for two-stage PID (see attitude.cpp for I-term rationale).
// I is disabled below FLYING_THROTTLE to avoid integrating ground-contact "error".
static const int16_t IDLE_THROTTLE = 60;       // below: no corrections, PID reset
static const int16_t FLYING_THROTTLE = 1100;   // above: full PID (I enabled)

// Two-stage attitude correction: low = off, medium = PD only, high = full PID.
// Trim is in degrees/deg/s and added to setpoint so I-term does not fight trim.
void FlightController::computeAttitudeCorrections(
    DroneCommand& command,
    Attitude& attitude,
    AttitudeTrim& trim,
    float& pitchPD,
    float& rollPD,
    float& yawPD
) {
    int16_t throttle = command.getThrottle();

    float desiredPitch = (float)command.getPitch() + trim.getPitchTrim();  // degrees
    float desiredRoll  = (float)command.getRoll()  + trim.getRollTrim();   // degrees
    float desiredYaw   = (float)command.getYaw()  + trim.getYawTrim();     // deg/s

    if (throttle < IDLE_THROTTLE) {
        attitude.resetPID();
        pitchPD = 0;
        rollPD  = 0;
        yawPD   = 0;
    } else if (throttle < FLYING_THROTTLE) {
        pitchPD = attitude.calculatePitchPD(desiredPitch, false);
        rollPD  = attitude.calculateRollPD(desiredRoll, false);
        yawPD   = attitude.calculateYawP(desiredYaw);
    } else {
        pitchPD = attitude.calculatePitchPD(desiredPitch, true);
        rollPD  = attitude.calculateRollPD(desiredRoll, true);
        yawPD   = attitude.calculateYawP(desiredYaw);
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
    // Trim is applied as setpoint offset in computeAttitudeCorrections, not here.

    int16_t throttle = command.getThrottle();

    // Yaw: +yawPD = CCW motors (M1,M4) faster; -yawPD = CW motors (M2,M3) faster = fight CCW twist.
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