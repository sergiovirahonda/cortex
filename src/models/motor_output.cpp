#include "motor_output.h"

MotorOutput::MotorOutput() {
    this->motor1Speed = 48;
    this->motor2Speed = 48;
    this->motor3Speed = 48;
    this->motor4Speed = 48;
}

// Setters

void MotorOutput::setMotor1Speed(int motor1Speed) {
    this->motor1Speed = motor1Speed;
}

void MotorOutput::setMotor2Speed(int motor2Speed) {
    this->motor2Speed = motor2Speed;
}

void MotorOutput::setMotor3Speed(int motor3Speed) {
    this->motor3Speed = motor3Speed;
}

void MotorOutput::setMotor4Speed(int motor4Speed) {
    this->motor4Speed = motor4Speed;
}

// Getters

int MotorOutput::getMotor1Speed() {
    return this->motor1Speed;
}

int MotorOutput::getMotor2Speed() {
    return this->motor2Speed;
}

int MotorOutput::getMotor3Speed() {
    return this->motor3Speed;
}

int MotorOutput::getMotor4Speed() {
    return this->motor4Speed;
}