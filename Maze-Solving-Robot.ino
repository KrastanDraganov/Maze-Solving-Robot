#include "robot.h"

Robot robot;

bool flag = false;

void setup() 
{
  Serial.begin(9600);

  robot.initializeSensors();
  robot.initializeMotors();
}

void loop() 
{
  // if (!flag)
  // {
  //   robot.testDrive();
  //   flag = false;
  // }
  

  if (!robot.didFinish())
  {
    robot.solveMaze();
  }
  else
  {
    Serial.println("celebrate");
    robot.celebrate();
  }
}