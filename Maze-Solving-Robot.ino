#include "robot.h"

Robot robot;

bool flag = true;

void setup() 
{
  Serial.begin(9600);

  // Serial.println("=== MOJE BI SHREK E GAY 1.0");

  robot.initializeSensors();
  robot.initializeMotors();

  // Serial.println("=== MOJE BI SHREK E GAY 2.0");


}

void loop() 
{

  robot.testDrive();

  robot.runMotors();

  // Serial.println("=== MOJE BI SHREK E GAY");

  // leftMotorMotionController.run();
  // rightMotorMotionController.run();


  // if (!robot.didFinish())
  // {
  //   robot.solveMaze();
  // }
  // else
  // {
  //   robot.celebrate();
  // }
}