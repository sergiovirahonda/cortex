#include "flight_controller.h"
#include "models/altitude.h"
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

FlightController::FlightController(const DroneConfig& droneConfig) : config_(droneConfig) {
    lastPacketTime = 0;
    inFailsafe = false;
    failsafeThrottle = FAILSAFE_LANDING_THROTTLE;
    lastTrimTime = 0;
    lastAltitudeHoldTime = 0;
}

void FlightController::updateTrims(DroneCommand& command, AttitudeTrim& attitudeTrim) {
    if (millis() - lastTrimTime <= (unsigned long)config_.getTrimDelay()) {
        return;
    }
    lastTrimTime = millis();

    float stepDeg = config_.getTrimStepPitchRollDeg();   // degrees (pitch, roll)
    float stepYaw = config_.getTrimStepYawDegPerSec();  // deg/s (yaw)

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

void FlightController::updateAltitudeHolding(
    DroneCommand& command,
    Altitude& altitude,
    AltitudeHold& altitudeHold
) {
    // Failsafe disengage is handled in lockAltitude(); here we only handle pilot command (1 = engage, -1 = disengage).

    if (command.getAltitudeHold() == 0 ) {
        return;
    }

    if (command.getAltitudeHold() == 1 && !altitudeHold.getAltitudeHoldEngaged()) {
        // Min height to engage (e.g. 1 m) so we have stable flight before hold; separate from LiDAR range.
        if (altitude.getCurrentAltitudeCm() < config_.getAltitudeLidarMinEngageCm()) {
            return;
        }
        // Max height: don't engage too high (e.g. above 7 m)
        if (altitude.getCurrentAltitudeCm() > config_.getAltitudeLidarMaxEngageCm()) {
            return;
        }
        // Throttle: must be in flight range
        if (command.getThrottle() < config_.getThrottleFlightStart()) {
            return;
        }
        altitudeHold.setAltitudeHoldEngaged(true);
        altitudeHold.setTargetAltitudeCm(altitude.getCurrentAltitudeCm());
    }

    if (command.getAltitudeHold() == -1 && altitudeHold.getAltitudeHoldEngaged()) {
        altitudeHold.reset();
        altitude.resetPID();  // Clear I-term only; keep fused altitude/velocity so re-engage has a sane estimate
    }
}

void FlightController::lockAltitude(
    DroneCommand& command,
    Altitude& altitude,
    AltitudeHold& altitudeHold
) {
    if (!altitudeHold.getAltitudeHoldEngaged()) {
        return;
    }
    // If the radio is dead, immediately disengage AltHold so the failsafe landing can happen
    if (this->isInFailsafe()) {
        altitudeHold.reset();
        altitude.resetPID();
        return; 
    }
    // Notice: We completely ignore the user's physical throttle stick here.
    float targetAlt = altitudeHold.getTargetAltitudeCm();

    // Run the altitude PID to get the correction
    float correction = altitude.calculateAltitudePID(targetAlt);

    // Apply the correction to the throttle
    float throttle = config_.getAltitudeHoverThrottle() + correction;
    command.setThrottle(throttle); // Command override 1: Pure hover + PID

    // Command override 2: Zero pitch and roll (level attitude)
    command.setPitch(0);
    command.setRoll(0);
    command.setYaw(0);
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
    DroneCommand& command,
    Attitude& attitude,
    AttitudeTrim& trim,
    float& pitchPD,
    float& rollPD,
    float& yawPD
) {
    int16_t throttle = command.getThrottle();
    int16_t idle = config_.getThrottleIdle();
    int16_t launchEnd = config_.getThrottleLaunchEnd();
    int16_t flightStart = config_.getThrottleFlightStart();

    float desiredPitch = (float)command.getPitch() + trim.getPitchTrim();  // degrees
    float desiredRoll  = (float)command.getRoll()  + trim.getRollTrim();   // degrees
    float desiredYaw   = (float)command.getYaw()   + trim.getYawTrim();    // deg/s (remap() converts stick to rate)

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
        yawPD   = attitude.calculateYawPI(desiredYaw, enableI, gainBlend);
        // Stage 3 only: feedforward for snappier yaw stick response (PI is weak at low Kp)
        if (throttle > flightStart) {
            yawPD += config_.getYawFF() * desiredYaw;
            yawPD = constrain(yawPD, -config_.getMaxPDOutput(), config_.getMaxPDOutput());
        }
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