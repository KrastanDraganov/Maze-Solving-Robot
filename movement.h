#ifndef MOTORS_MOVEMENT_H
#define MOTORS_MOVEMENT_H

struct MotorsMovement {
  int leftMotorMaxSpeed;
  int leftMotorAcceleration;
  int leftMotorTargetPosition;

  int rightMotorMaxSpeed;
  int rightMotorAcceleration;
  int rightMotorTargetPosition;

  MotorsMovement()
        : leftMotorMaxSpeed(-1),
          leftMotorAcceleration(-1),
          leftMotorTargetPosition(-1),
          rightMotorMaxSpeed(-1),
          rightMotorAcceleration(-1),
          rightMotorTargetPosition(-1) {}

  MotorsMovement(int leftMaxSpeed, int leftAcceleration, int leftTargetPosition,
                   int rightMaxSpeed, int rightAcceleration, int rightTargetPosition)
        : leftMotorMaxSpeed(leftMaxSpeed),
          leftMotorAcceleration(leftAcceleration),
          leftMotorTargetPosition(leftTargetPosition),
          rightMotorMaxSpeed(rightMaxSpeed),
          rightMotorAcceleration(rightAcceleration),
          rightMotorTargetPosition(rightTargetPosition) {}
};

#endif