#include "motor.h"

void Motor::Motor(uint8_t _inputPin1, uint8_t inputPin2, uint8_t _pwmPin, uint8_t _encoderPin)
{
  inputPin1 = _inputPin1;
  inputPin2 = _inputPin2;
  pwmPin = _pwmPin;
  encoderPin = _encoderPin;
}

void Motor::setupMotor()
{
  pinMode(inputPin1, OUTPUT);
  pinMode(inputPIn2, OUTPUT);

  pinMode(pwmPin, OUTPUT);

  pinMode(encoderPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPin), readEncoder, CHANGE);
  noInterrupts();
}

void Motor::setMotor(int direction, int speed)
{
  interrupts();

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
  noInterrupts();

  analogWrite(pwmPin, 0);

  digitalWrite(inputPin1, LOW);
  digitalWrite(inputPin2, LOW);
}

void Motor::readEncoder()
{
  // TO DO
}