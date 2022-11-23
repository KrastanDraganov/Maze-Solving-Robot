#include "ultrasonic_sensor.h"

UltrasonicSensor::UltrasonicSensor()
{
  triggerPin = echoPin = -1;
}

UltrasonicSensor::UltrasonicSensor(uint8_t _triggerPin, uint8_t _echoPin)
{
  triggerPin = _triggerPin;
  echoPin = _echoPin;
}

void UltrasonicSensor::setupSensor()
{
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

double UltrasonicSensor::measureDistance()
{
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
 
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
 
  digitalWrite(triggerPin, LOW);
 
  double duration = pulseIn(echoPin, HIGH);
 
  double distance = (duration * 0.034) / 2.0;
 
  return distance;
}