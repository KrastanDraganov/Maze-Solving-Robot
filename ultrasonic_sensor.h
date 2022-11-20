#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include "constants.h"

class UltrasonicSensor
{
  private:
    uint8_t triggerPin;
    uint8_t echoPin;

  public:
    UltrasonicSensor(uint8_t _triggerPin, uint8_t _echoPin);

    void setupSensor();

    double measureDistance();
};

#endif