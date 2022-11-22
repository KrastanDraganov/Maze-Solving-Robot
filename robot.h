#ifndef ROBOT_H
#define ROBOT_H

#include "maze.h"
#include "motor.h"
#include "ultrasonic_sensor.h"
#include "stack.h"

class Robot
{
  private:
    uint8_t x, y;
    uint8_t orientation;

    Stack movementTraceBack;

    Maze maze;

    UltrasonicSensor leftSensor;
    UltrasonicSensor rightSensor;
    UltrasonicSensor frontSensor;

    Motor leftMotor;
    Motor rightMotor;
  public:
    Robot();

    void initializeSensors();
    void initializeMotors();

    bool checkForWallUsingSensors(uint8_t direction);

    uint8_t decideCrossroad(uint8_t wallsMask, bool isCrossroadAlreadyVisited);
    uint8_t getCorridorDirection(uint8_t wallsMask);

    void goForward();
    void goBackwards();
    void turnLeftBackwards();
    void turnLeftForward();
    void turnRightBackwards();
    void turnRightForward();
    
    void maneuverOverCrossroad(bool isMovingBackwards);
    void physicallyMoveRobot(uint8_t direction, uint8_t wallsMask, bool isCrossroad);
    void returnFromDeadEnd();
    
    void solveMaze();

    bool didFinish();
    void celebrate();
};

#endif