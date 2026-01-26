#include "attitude.h"

#include <Arduino.h>

// =================================================================
// TUNING CONSTANTS (Single Loop Angle -> PWM)
// =================================================================

const float MAX_PD_OUTPUT = 350.0; // Limit correction to avoid overpowering throttle

// --- ROLL & PITCH GAINS ---
// Kp: The "Spring". Stiffens the drone.
// Kd: The "Damper". Stops the bounce.
// Rule of Thumb: Kp should be 10x to 20x larger than Kd in this setup.


const float KP_ROLL  = 2.0;   
const float KD_ROLL  = 0.20;

const float KP_PITCH = 2.2;
const float KD_PITCH = 0.12;

// --- YAW GAIN ---
// Yaw is purely P-Controlled for now.
const float KP_YAW   = 4.0;

// =================================================================

Attitude::Attitude() {
    rollAngle = 0;
    rollRate = 0;
    pitchAngle = 0;
    pitchRate = 0;
    yawRate = 0;
    yawPrevError = 0;
}

// Bulk setter
void Attitude::updateSensors(float rAngle, float rRate, float pAngle, float pRate, float yRate) {
    this->rollAngle = rAngle;
    this->rollRate  = rRate;
    this->pitchAngle = pAngle;
    this->pitchRate  = pRate;
    this->yawRate   = yRate;
}
// Raw getters
float Attitude::getRollAngle() { return this->rollAngle; }
float Attitude::getRollRate() { return this->rollRate; }
float Attitude::getPitchAngle() { return this->pitchAngle; }
float Attitude::getPitchRate() { return this->pitchRate; }
float Attitude::getYawRate() { return this->yawRate; }

// =================================================================
// STABILIZATION LOGIC
// =================================================================

// ROLL & PITCH strategy: "Standard PD"
// P-Term: Corrects the Error (Angle)
// D-Term: Resists the Motion (Gyro Rate) -> Prevents overshoot
// Formula: Output = (Kp * Error) - (Kd * Rate)

float Attitude::calculateRollPD(float desiredRollAngle) {
    float error = desiredRollAngle - this->rollAngle;
    
    float P = KP_ROLL * error;
    float D = KD_ROLL * this->rollRate; // D acts on Measurement (Rate)
    
    float output = P - D; 
    return constrain(output, -MAX_PD_OUTPUT, MAX_PD_OUTPUT);
}

float Attitude::calculatePitchPD(float desiredPitchAngle) {
    float error = desiredPitchAngle - this->pitchAngle;
    
    float P = KP_PITCH * error;
    float D = KD_PITCH * this->pitchRate;
    
    float output = P - D;
    return constrain(output, -MAX_PD_OUTPUT, MAX_PD_OUTPUT);
}

// YAW strategy: "P-Only"
// Yaw is naturally stable. D-term usually adds noise.
// Formula: Output = Kp * (DesiredRate - ActualRate)

float Attitude::calculateYawP(float desiredYawRate) {
    float error = desiredYawRate - this->yawRate;
    
    float output = KP_YAW * error;
    return constrain(output, -MAX_PD_OUTPUT, MAX_PD_OUTPUT);
}