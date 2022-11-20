#include "ultrasonic_sensor.h"

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
  double distance = 0;
  
  uint8_t measuringAttempts = 10;

  for (int i = 0; i < measuringAttempts; ++i) 
  {
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
 
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
 
    digitalWrite(triggerPin, LOW);
 
    long duration = pulseIn(echoPin, HIGH);
 
    distance = distance + (duration * 0.034) / 2.0;
  }
 
  distance = distance / (1.0 * measuringAttempts);
 
  return distance;
}