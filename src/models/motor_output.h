#ifndef MOTOR_OUTPUT_H
#define MOTOR_OUTPUT_H

class MotorOutput {
  private:
    int motor1Speed;
    int motor2Speed;
    int motor3Speed;
    int motor4Speed;
  public:
    MotorOutput();
    void setMotor1Speed(int motor1Speed);
    void setMotor2Speed(int motor2Speed);
    void setMotor3Speed(int motor3Speed);
    void setMotor4Speed(int motor4Speed);
    int getMotor1Speed();
    int getMotor2Speed();
    int getMotor3Speed();
    int getMotor4Speed();
};

#endif