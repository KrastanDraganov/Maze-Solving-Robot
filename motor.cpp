#include "motor.h"

Motor::Motor()
{
  inputPin1 = inputPin2 = pwmPin = encoderPin = ticksIndex = -1;
  previousEncoderTicks = 0;
}

Motor::Motor(uint8_t _inputPin1, uint8_t _inputPin2, uint8_t _pwmPin, uint8_t _encoderPin, uint8_t _ticksIndex)
{
  inputPin1 = _inputPin1;
  inputPin2 = _inputPin2;
  pwmPin = _pwmPin;
  encoderPin = _encoderPin;
  ticksIndex = _ticksIndex;
}

void Motor::setupMotor()
{
  pinMode(inputPin1, OUTPUT);
  pinMode(inputPin2, OUTPUT);

  pinMode(pwmPin, OUTPUT);

  pinMode(encoderPin, INPUT);
  
  if (ticksIndex == LEFT)
  { 
    attachInterrupt(digitalPinToInterrupt(encoderPin), readLeftEncoder, CHANGE);
  }
  else
  {
    attachInterrupt(digitalPinToInterrupt(encoderPin), readRightEncoder, CHANGE);
  }

  ticks[ticksIndex] = 0;
  previousEncoderTicks = 0;
}

void Motor::setMotor(uint8_t direction, uint8_t speed)
{
  analogWrite(pwmPin, speed);

  if (direction == FORWARD)
  {
    digitalWrite(inputPin1, HIGH);
    digitalWrite(inputPin2, LOW);
  }
  else if (direction == BACKWARDS)
  {
    digitalWrite(inputPin1, LOW);
    digitalWrite(inputPin2, HIGH);
  }
}

void Motor::stopMotor()
{
  analogWrite(pwmPin, 0);

  digitalWrite(inputPin1, LOW);
  digitalWrite(inputPin2, LOW);
}

volatile uint32_t Motor::getRPM()
{
  uint32_t currentEncoderTicks = ticks[ticksIndex];

  uint32_t rpm = currentEncoderTicks - previousEncoderTicks;
  previousEncoderTicks = currentEncoderTicks;
  
  return rpm;
}

void Motor::resetTicks()
{
  ticks[ticksIndex] = 0;
  previousEncoderTicks = 0;
}