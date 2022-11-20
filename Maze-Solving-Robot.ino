#include "robot.h"

Robot robot;

void setup() 
{
  robot.initializeSensors();
  robot.initializeMotors();
}

void loop() 
{
  if (!robot.didFinish())
  {
    robot.solveMaze();
  }
  else
  {
    robot.celebrate();
  }
}
