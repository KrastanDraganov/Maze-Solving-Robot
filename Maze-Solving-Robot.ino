#include "robot.h"

Robot robot;

bool flag = false;

void setup() 
{
  Serial.begin(9600);
  // delay(1000);

  robot.initializeSensors();
  robot.initializeMotors();
}

void loop() 
{
  if (!flag)
  {
    robot.testDrive();
    flag = true;
  }

  // if (!robot.didFinish())
  // {
  //   robot.solveMaze();
  // }
  // else
  // {
  //   robot.celebrate();
  // }
}
