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

// Float mapping (Arduino map() is integers only). Safe if in_max == in_min.
static float mapFloat(float x, float inMin, float inMax, float outMin, float outMax) {
    if (inMax <= inMin) return outMax;
    return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

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

    // Pitch: negative = nose down (subtract from trim), positive = nose up (add to trim).
    int16_t pt = command.getPitchTrim();
    if (pt < 0) {
        attitudeTrim.setPitchTrim(attitudeTrim.getPitchTrim() - stepDeg);
    } else if (pt > 0) {
        attitudeTrim.setPitchTrim(attitudeTrim.getPitchTrim() + stepDeg);
    }

    // Roll: negative = left (subtract), positive = right (add).
    int16_t rt = command.getRollTrim();
    if (rt < 0) {
        attitudeTrim.setRollTrim(attitudeTrim.getRollTrim() - stepDeg);
    } else if (rt > 0) {
        attitudeTrim.setRollTrim(attitudeTrim.getRollTrim() + stepDeg);
    }

    // Yaw: positive = CW (add), negative = CCW (subtract).
    int16_t yt = command.getYawTrim();
    if (yt > 0) {
        attitudeTrim.setYawTrim(attitudeTrim.getYawTrim() + stepYaw);
    } else if (yt < 0) {
        attitudeTrim.setYawTrim(attitudeTrim.getYawTrim() - stepYaw);
    }

    if (command.getTrimReset() != 0) {
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

// Three throttle ranges: 1 = high KP (launch), 2 = transition (blend), 3 = low KP (flight, I on).
// Trim is in degrees/deg/s and added to setpoint so I-term does not fight trim.
void FlightController::computeAttitudeCorrections(
    const DroneConfig& droneConfig,
    DroneCommand& command,
    Attitude& attitude,
    AttitudeTrim& trim,
    float& pitchPD,
    float& rollPD,
    float& yawPD
) {
    int16_t throttle = command.getThrottle();
    int16_t idle = droneConfig.getThrottleIdle();
    int16_t launchEnd = droneConfig.getThrottleLaunchEnd();
    int16_t flightStart = droneConfig.getThrottleFlightStart();

    float desiredPitch = (float)command.getPitch() + trim.getPitchTrim();  // degrees
    float desiredRoll  = (float)command.getRoll()  + trim.getRollTrim();   // degrees
    float desiredYaw   = (float)command.getYaw()  + trim.getYawTrim();     // deg/s

    if (throttle < idle) {
        attitude.resetPID();
        pitchPD = 0;
        rollPD  = 0;
        yawPD   = 0;
    } else {
        // Three stages: launch (high KP), transition (blend), flight (low KP, I on).
        float gainBlend;  // 0 = launch, 1 = flight; feed into attitude for Kp/Kd blend
        bool enableI;

        if (throttle < launchEnd) {
            // Stage 1: Below lift-off → MAX strength (launch gains, no I)
            gainBlend = 0.0f;
            enableI = false;
        } else if (throttle > flightStart) {
            // Stage 2: Flying → NORMAL strength (flight gains, I on)
            gainBlend = 1.0f;
            enableI = true;
        } else {
            // Stage 3: Transition zone → smoothly slide from launch to flight
            gainBlend = mapFloat((float)throttle, (float)launchEnd, (float)flightStart, 0.0f, 1.0f);
            // Enable I only in upper part of transition (0.8 = late, less wobble)
            const float I_IN_TRANSITION_THRESHOLD = 0.8f;
            enableI = (gainBlend >= I_IN_TRANSITION_THRESHOLD);
        }

        rollPD  = attitude.calculateRollPD(desiredRoll, enableI, gainBlend);
        pitchPD = attitude.calculatePitchPD(desiredPitch, enableI, gainBlend);
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