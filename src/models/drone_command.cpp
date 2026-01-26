#include "drone_command.h"
#include <Arduino.h>

// =================================================================
// CONTROL LIMITS CONSTANTS
// =================================================================
const int16_t MAX_ANGLE_DEGREES = 30;   
const int16_t MAX_YAW_RATE_DPS = 120;   
const int16_t MIN_THROTTLE_PWM = 50;
const int16_t MAX_THROTTLE_PWM = 2047;  

DroneCommand::DroneCommand(int16_t pitch, int16_t roll, int16_t yaw, int16_t throttle) {
    this->pitch = pitch;
    this->roll = roll;
    this->yaw = yaw;
    this->throttle = throttle;
};

// --- SETTERS ---
void DroneCommand::setPitch(int16_t pitch) { this->pitch = pitch; }
void DroneCommand::setRoll(int16_t roll)   { this->roll = roll; }
void DroneCommand::setYaw(int16_t yaw)     { this->yaw = yaw; }
void DroneCommand::setThrottle(int16_t throttle) { this->throttle = throttle; }

void DroneCommand::reset() {
    pitch = 0; roll = 0; yaw = 0; throttle = 0;
}

// --- GETTERS ---
int16_t DroneCommand::getPitch()    { return this->pitch; }
int16_t DroneCommand::getRoll()     { return this->roll; }
int16_t DroneCommand::getYaw()      { return this->yaw; }
int16_t DroneCommand::getThrottle() { return this->throttle; }

// --- RADIO HELPERS (The Bridge) ---
DronePacket DroneCommand::createPacket() {
    DronePacket pkt;
    pkt.pitch = this->pitch;
    pkt.roll = this->roll;
    pkt.yaw = this->yaw;
    pkt.throttle = this->throttle;
    return pkt;
}

void DroneCommand::loadFromPacket(DronePacket pkt) {
    this->pitch = pkt.pitch;
    this->roll = pkt.roll;
    this->yaw = pkt.yaw;
    this->throttle = pkt.throttle;
}

// --- LOGIC ---
void DroneCommand::remap() {
    // 1. Throttle: Map 0-100% input to DShot Range (e.g., 50 to 2047)
    // Note: We use long for the calculation to prevent overflow, then cast back
    int32_t baseThrottle = map(this->throttle, 0, 50, MIN_THROTTLE_PWM, MAX_THROTTLE_PWM);

    // 2. Pitch/Roll: Map -100 to 100 input to Angle degrees (e.g., -30 to 30)
    int32_t desiredPitchAngle = map(this->pitch, -100, 100, -MAX_ANGLE_DEGREES, MAX_ANGLE_DEGREES);
    int32_t desiredRollAngle = map(this->roll, -100, 100, -MAX_ANGLE_DEGREES, MAX_ANGLE_DEGREES);

    // 3. Yaw: Map -100 to 100 input to Rotation Rate
    int32_t desiredYawRate = map(this->yaw, -100, 100, MAX_YAW_RATE_DPS, -MAX_YAW_RATE_DPS);

    // Update state with the new calculated values
    this->throttle = (int16_t)baseThrottle;
    this->pitch = (int16_t)desiredPitchAngle;
    this->roll = (int16_t)desiredRollAngle;
    this->yaw = (int16_t)desiredYawRate;
}