#ifndef ROBOT_H
#define ROBOT_H

#include <NewPing.h>

#include "maze.h"
#include "motor.h"

class Robot
{
  private:
    uint8_t x, y;
    uint8_t orientation;
    Maze maze;

    uint8_t IRSensorLeft;
    uint8_t IRSensorRight;

    // TODO: put pins for ultrasonic sensor
    NewPing sonar = NewPing(1, 1, 20);

    Motor leftMotor = Motor(13, 12, 6, 2, LEFT);
    Motor rightMotor = Motor(7, 8, 5, 3, RIGHT);
  public:
    Robot();

    void initializeSensors();
    void initializeMotors();
    void initializeEncoders();

    bool checkForWallUsingSensors(uint8_t direction);

    uint8_t decideCrossroad(uint8_t wallsMask, bool isCrossroadAlreadyVisited);
    uint8_t getCorridorDirection(uint8_t wallsMask);

    void physicallyMoveRobot(uint8_t direction);
    void solveMaze();

    bool didFinish();
    void celebrate();
};

#endif