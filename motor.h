#ifndef MOTOR_H
#define MOTOR_H

#include "Arduino.h"

class Motor
{
  private:
    uint8_t inputPin1, inputPin2;
    uint8_t pwmPin;

    uint8_t encoderPIn;

  public:
    Motor(uint8_t _inputPin1, uint8_t inputPin2, uint8_t _pwmPin, uint8_t _encoderPin);

    void setupMotor();

    void setMotor(uint8_t direction, uint8_t speed);
    void stopMotor();

    void readEncoder();
};

#endif