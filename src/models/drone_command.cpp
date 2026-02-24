#include "drone_command.h"
#include "config/drone_config_defaults.h"
#include <Arduino.h>

// =================================================================
// CONTROL LIMITS CONSTANTS (MAX_YAW_RATE_DPS from drone_config_defaults.h / build_flags)
// =================================================================
const int16_t MAX_ANGLE_DEGREES = 30;   
const int16_t MIN_THROTTLE_PWM = 50;
const int16_t MAX_THROTTLE_PWM = 2047;  

DroneCommand::DroneCommand(
    int16_t pitch,
    int16_t roll,
    int16_t yaw,
    int16_t throttle,
    int16_t rollTrim,
    int16_t pitchTrim,
    int16_t yawTrim,
    uint8_t trimReset,
    int8_t altitudeHold
) {
    this->pitch = pitch;
    this->roll = roll;
    this->yaw = yaw;
    this->throttle = throttle;
    this->rollTrim = rollTrim;
    this->pitchTrim = pitchTrim;
    this->yawTrim = yawTrim;
    this->trimReset = trimReset;
    this->altitudeHold = altitudeHold;
};

// --- SETTERS ---
void DroneCommand::setPitch(int16_t pitch) { this->pitch = pitch; }
void DroneCommand::setRoll(int16_t roll)   { this->roll = roll; }
void DroneCommand::setYaw(int16_t yaw)     { this->yaw = yaw; }
void DroneCommand::setThrottle(int16_t throttle) { this->throttle = throttle; }
void DroneCommand::setRollTrim(int16_t rollTrim) { this->rollTrim = rollTrim; }
void DroneCommand::setPitchTrim(int16_t pitchTrim) { this->pitchTrim = pitchTrim; }
void DroneCommand::setYawTrim(int16_t yawTrim) { this->yawTrim = yawTrim; }
void DroneCommand::setTrimReset(uint8_t trimReset) { this->trimReset = trimReset; }
void DroneCommand::setAltitudeHold(int8_t altitudeHold) { this->altitudeHold = altitudeHold; }
void DroneCommand::reset() {
    pitch = 0; roll = 0; yaw = 0; throttle = 0;
    rollTrim = 0; pitchTrim = 0; yawTrim = 0;
}

// --- GETTERS ---
int16_t DroneCommand::getPitch()    { return this->pitch; }
int16_t DroneCommand::getRoll()     { return this->roll; }
int16_t DroneCommand::getYaw()      { return this->yaw; }
int16_t DroneCommand::getThrottle() { return this->throttle; }
int16_t DroneCommand::getRollTrim() { return this->rollTrim; }
int16_t DroneCommand::getPitchTrim() { return this->pitchTrim; }
int16_t DroneCommand::getYawTrim() { return this->yawTrim; }
uint8_t DroneCommand::getTrimReset() { return this->trimReset; }
int8_t DroneCommand::getAltitudeHold() { return this->altitudeHold; }

// --- RADIO HELPERS (The Bridge) ---
DronePacket DroneCommand::createPacket() {
    DronePacket pkt;
    pkt.pitch = this->pitch;
    pkt.roll = this->roll;
    pkt.yaw = this->yaw;
    pkt.throttle = this->throttle;
    pkt.rollTrim = this->rollTrim;
    pkt.pitchTrim = this->pitchTrim;
    pkt.yawTrim = this->yawTrim;
    pkt.trimReset = this->trimReset;
    pkt.altitudeHold = this->altitudeHold;
    return pkt;
}

void DroneCommand::loadFromPacket(DronePacket pkt) {
    this->pitch = pkt.pitch;
    this->roll = pkt.roll;
    this->yaw = pkt.yaw;
    this->throttle = pkt.throttle;
    this->rollTrim = pkt.rollTrim;
    this->pitchTrim = pkt.pitchTrim;
    this->yawTrim = pkt.yawTrim;
    this->trimReset = pkt.trimReset;
    this->altitudeHold = pkt.altitudeHold;
}

// --- LOGIC ---
void DroneCommand::remap() {
    // 1. Throttle: Map stick input to DShot range (50–2047).
    int32_t baseThrottle = map(this->throttle, 0, 100, MIN_THROTTLE_PWM, MAX_THROTTLE_PWM);

    // 2. Pitch/Roll: Map -100 to 100 input to Angle degrees (e.g., -30 to 30)
    int32_t desiredPitchAngle = map(this->pitch, -100, 100, -MAX_ANGLE_DEGREES, MAX_ANGLE_DEGREES);
    int32_t desiredRollAngle = map(this->roll, -100, 100, -MAX_ANGLE_DEGREES, MAX_ANGLE_DEGREES);

    // 3. Yaw: Map -100 to 100 input to rotation rate (deg/s)
    int32_t desiredYawRate = map(this->yaw, -100, 100, (int32_t)MAX_YAW_RATE_DPS, -(int32_t)MAX_YAW_RATE_DPS);

    // Update state with the new calculated values
    this->throttle = (int16_t)baseThrottle;
    this->pitch = (int16_t)desiredPitchAngle;
    this->roll = (int16_t)desiredRollAngle;
    this->yaw = (int16_t)desiredYawRate;
}

// =================================================================
// TelemetryData
// =================================================================

TelemetryData::TelemetryData(int16_t pwm, int16_t roll, int16_t pitch, uint8_t altitudeHold) {
    this->pwm = pwm;
    this->roll = roll;
    this->pitch = pitch;
    this->altitudeHold = altitudeHold;
}

int16_t TelemetryData::getPwm()   { return this->pwm; }
int16_t TelemetryData::getRoll()  { return this->roll; }
int16_t TelemetryData::getPitch() { return this->pitch; }
uint8_t TelemetryData::getAltitudeHold() { return this->altitudeHold; }

void TelemetryData::setPwm(int16_t pwm)     { this->pwm = pwm; }
void TelemetryData::setRoll(int16_t roll)   { this->roll = roll; }
void TelemetryData::setPitch(int16_t pitch) { this->pitch = pitch; }
void TelemetryData::setAltitudeHold(uint8_t altitudeHold) { this->altitudeHold = altitudeHold; }

void TelemetryData::reset() {
    pwm = 0;
    roll = 0;
    pitch = 0;
    altitudeHold = 0;
}

TelemetryPacket TelemetryData::createPacket() {
    TelemetryPacket pkt;
    pkt.pwm = this->pwm;
    pkt.roll = this->roll;
    pkt.pitch = this->pitch;
    pkt.altitudeHold = this->altitudeHold;
    return pkt;
}

void TelemetryData::loadFromPacket(TelemetryPacket pkt) {
    this->pwm = pkt.pwm;
    this->roll = pkt.roll;
    this->pitch = pkt.pitch;
    this->altitudeHold = pkt.altitudeHold;
}