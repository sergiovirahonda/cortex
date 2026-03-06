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
    int getMotor1Speed() const;
    int getMotor2Speed() const;
    int getMotor3Speed() const;
    int getMotor4Speed() const;
};

#endif