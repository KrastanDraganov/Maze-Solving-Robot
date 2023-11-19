#include "robot.h"

Robot robot;

bool flag = true;

void setup() 
{
  Serial.begin(9600);

  robot.initializeSensors();
  robot.initializeMotors();

  delay(10000);
}

void loop() 
{

  robot.runMotors();

  // if (flag) {
  //   robot.testDrive();
  //   flag = false;
  // }

  if (!robot.didFinish())
  {
    robot.solveMaze();
  }
  else
  {
    robot.celebrate();
  }
}