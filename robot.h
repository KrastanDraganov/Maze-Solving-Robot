#ifndef ROBOT_H
#define ROBOT_H

#include <AccelStepper.h>
#include <Adafruit_MotorShield.h>

#include "maze.h"
#include "infrared_sensor.h"
#include "stack.h"
#include "queue.h"
#include "movement.h"

class Robot
{
  private:
    int x, y;
    uint8_t orientation;

    Stack<uint8_t> movementTraceBack;

    bool hasToMovePhysically;
    Queue<MotorsMovement> motorsAssemblyLine;

    Maze maze;

    InfraredSensor leftSensor;
    InfraredSensor rightSensor;
    InfraredSensor frontSensor;

  public:
    Robot();

    void initializeSensors();
    void initializeMotors();

    bool checkForWallUsingSensors(uint8_t direction);

    uint8_t decideCrossroad(uint8_t wallsMask, bool isCrossroadAlreadyVisited);
    uint8_t getCorridorDirection(uint8_t wallsMask);

    void runMotors();
    void goForward();
    void turnLeft();
    void turnRight();

    void goBackwards();
    void turnLeftBackwards();
    void turnLeftForward();
    void turnRightBackwards();
    void turnRightForward();
    
    void maneuverOverCrossroadToDifferentPosition(uint8_t targetDirection, uint8_t wallsMask);
    void maneuverOverCrossroadToSamePosition(uint8_t wallsMask);
    void physicallyMoveRobot(uint8_t direction, uint8_t wallsMask, bool isCrossroad);
    void returnFromDeadEnd();
    
    void solveMaze();

    bool didFinish();
    void celebrate();

    void testDrive();
};

#endif