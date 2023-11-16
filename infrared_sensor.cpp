#include "infrared_sensor.h"

InfraredSensor::InfraredSensor()
{
  digitalPin = analogPin = -1;
}

InfraredSensor::InfraredSensor(uint8_t _digitalPin, uint8_t _analogPin)
{
  digitalPin = _digitalPin;
  analogPin = _analogPin;
}

void InfraredSensor::setup()
{
  pinMode(digitalPin, INPUT);
  pinMode(analogPin, INPUT);
}

bool InfraredSensor::isNearObstacle()
{
  return digitalRead(digitalPin) == LOW;
}