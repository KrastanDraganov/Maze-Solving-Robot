#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include "constants.h"

class InfraredSensor
{
  private:
    uint8_t digitalPin;
    uint8_t analogPin;

  public:
    InfraredSensor();
    InfraredSensor(uint8_t _digitalPin, uint8_t _analogPin);

    void setup();

    bool isNearObstacle();
};

#endif