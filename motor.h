#ifndef MOTOR_H
#define MOTOR_H

#include "Arduino.h"
#include "constants.h"

class Motor
{
  private:
    uint8_t inputPin1, inputPin2;
    uint8_t pwmPin;

    uint8_t encoderPin;

    uint8_t ticksIndex;
    volatile uint32_t previousEncoderTicks;

  public:
    Motor();
    Motor(uint8_t _inputPin1, uint8_t _inputPin2, uint8_t _pwmPin, uint8_t _encoderPin, uint8_t _ticksIndex);

    void setupMotor();

    void setMotor(uint8_t direction, uint8_t speed);
    void stopMotor();

    volatile uint32_t getRPM();
};

#endif