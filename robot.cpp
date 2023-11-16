#include "robot.h"

Adafruit_MotorShield motorShield(STEPPER_MOTOR_SHIELD_ADDRESS);

Adafruit_StepperMotor *leftMotorDriver = motorShield.getStepper(STEPPER_MOTOR_STEPS_PER_REVOLUTION, LEFT_STEPPER_MOTOR_INDEX);
Adafruit_StepperMotor *rightMotorDriver = motorShield.getStepper(STEPPER_MOTOR_STEPS_PER_REVOLUTION, RIGHT_STEPPER_MOTOR_INDEX);

void leftMotorStepForward()
{
  leftMotorDriver->onestep(FORWARD, SINGLE);
}

void leftMotorStepBackward()
{
  leftMotorDriver->onestep(BACKWARD, SINGLE);
}

void rightMotorStepForward()
{
  rightMotorDriver->onestep(FORWARD, SINGLE);
}

void rightMotorStepBackward()
{
  rightMotorDriver->onestep(BACKWARD, SINGLE);
}

AccelStepper leftMotorMotionController(leftMotorStepForward, leftMotorStepBackward);
AccelStepper rightMotorMotionController(rightMotorStepForward, rightMotorStepBackward);

int cells = 0;

void Robot::testDrive()
{

  Serial.print("FRONT --> ");
  Serial.println(frontSensor.isNearObstacle());
  Serial.println();

  Serial.print("RIGHT --> ");
  Serial.println(rightSensor.isNearObstacle());
  Serial.println();

  Serial.print("LEFT --> ");
  Serial.println(leftSensor.isNearObstacle());
  Serial.println();
  // if (cells == 0 || (cells < 5 && leftMotorMotionController.distanceToGo() == 0)) {
  //   delay(3000);

  //   goForward();

  //   ++cells;
  // }
}

Robot::Robot()
{
  x = y = 0;
  orientation = DOWN;

  leftSensor = InfraredSensor(4, A2);
  rightSensor = InfraredSensor(3, A1);
  frontSensor = InfraredSensor(2, A0);
}

void Robot::initializeSensors()
{
  leftSensor.setup();
  rightSensor.setup();
  frontSensor.setup();
}

void Robot::initializeMotors()
{
  motorShield.begin();
}

bool Robot::checkForWallUsingSensors(uint8_t direction)
{
  if (direction == RIGHT)
  {
    return rightSensor.isNearObstacle();
  }

  if (direction == LEFT)
  {
    return leftSensor.isNearObstacle();
  }

  if (direction == FORWARDS)
  {
    return frontSensor.isNearObstacle();
  }

  return false;
}

uint8_t Robot::decideCrossroad(uint8_t wallsMask, bool isCrossroadAlreadyVisited)
{
  uint8_t markersBackwards = maze.getCorridorMarkersCount(x, y, orientation, BACKWARDS) + 1;
  maze.updateCorridorMarkersCount(x, y, orientation, BACKWARDS, markersBackwards);

  if (isCrossroadAlreadyVisited)
  {
    if (markersBackwards == 1)
    {
      ++markersBackwards;
      maze.updateCorridorMarkersCount(x, y, orientation, BACKWARDS, markersBackwards);

      return BACKWARDS;
    }
  }

  uint8_t smallestMarkersCount = MARKERS_LIMIT + 1;
  uint8_t smallestDistanceToFinal = MAZE_SIZE + MAZE_SIZE + 1;
  uint8_t decisionDirection = -1;

  for (uint8_t direction = 0; direction < 4; ++direction)
  {
    if (direction == BACKWARDS)
    {
      continue;
    }

    bool isWallFlagSet = (wallsMask & (1 << direction));
    if (isWallFlagSet)
    {
      continue;
    }

    uint8_t currentMarkersCount = maze.getCorridorMarkersCount(x, y, orientation, direction);

    uint8_t currentDistanceToFinal = 0;
    currentDistanceToFinal += maze.getDistanceToFinal(x + MOVEMENT_CHANGES[orientation][direction][X]);
    currentDistanceToFinal += maze.getDistanceToFinal(y + MOVEMENT_CHANGES[orientation][direction][Y]);

    if (currentMarkersCount < smallestMarkersCount)
    {
      smallestMarkersCount = currentMarkersCount;
      smallestDistanceToFinal = currentDistanceToFinal;
      decisionDirection = direction;
    }
    else if (currentMarkersCount == smallestMarkersCount and currentDistanceToFinal <= smallestDistanceToFinal)
    {
      smallestDistanceToFinal = currentDistanceToFinal;
      decisionDirection = direction;
    }
  }

  maze.updateCorridorMarkersCount(x, y, orientation, decisionDirection, smallestMarkersCount + 1);

  return decisionDirection;
}

uint8_t Robot::getCorridorDirection(uint8_t wallsMask)
{

  for (uint8_t direction = 0; direction < 4; ++direction)
  {
    bool isWallFlagSet = (wallsMask & (1 << direction));
    if (direction != BACKWARDS and !isWallFlagSet)
    {
      return direction;
    }
  }

  return BACKWARDS;
}

void Robot::maneuverOverCrossroadToDifferentPosition(uint8_t targetDirection, uint8_t wallsMask)
{
  bool isFrontWallSet = (wallsMask & (1 << FORWARDS));
  bool isLeftWallSet = (wallsMask & (1 << LEFT));
  bool isRightWallSet = (wallsMask & (1 << RIGHT));

  if (targetDirection == LEFT)
  {
    if (!isFrontWallSet)
    {
      // goForward(DEFAULT_FORWARD_ROTATIONS);
      goForward();
      turnLeftBackwards();
    }
    else if (!isRightWallSet)
    {
      turnRightForward();
      goBackwards();
    }
  }
  else if (targetDirection == RIGHT)
  {
    if (!isFrontWallSet)
    {
      // goForward(DEFAULT_FORWARD_ROTATIONS);
      goForward();
      turnRightBackwards();
    }
    else if (!isLeftWallSet)
    {
      turnLeftForward();
      goBackwards();
    }
  }
  else if (targetDirection == FORWARDS)
  {
    if (!isLeftWallSet)
    {
      turnLeftForward();
      turnRightBackwards();
    }
    else if (!isRightWallSet)
    {
      turnRightForward();
      turnLeftBackwards();
    }
  }

  orientation = ORIENTATION_CHANGES[orientation][targetDirection];
  orientation = OPPOSITE_ORIENTATIONS[orientation];
}

void Robot::maneuverOverCrossroadToSamePosition(uint8_t wallsMask)
{
  bool isFrontWallSet = (wallsMask & (1 << FORWARDS));
  bool isLeftWallSet = (wallsMask & (1 << LEFT));
  bool isRightWallSet = (wallsMask & (1 << RIGHT));

  if (!isFrontWallSet)
  {
    // goForward(DEFAULT_FORWARD_ROTATIONS);
    goForward();

    if (!isLeftWallSet)
    {
      turnLeftBackwards();
      turnRightForward();
    }
    else if (!isRightWallSet)
    {
      turnRightBackwards();
      turnLeftForward();
    }
  }
  else // if there is front wall, there are definitely no left and right walls
  {
    turnLeftForward();
    goBackwards();
    turnLeftForward();
  }
}

void Robot::physicallyMoveRobot(uint8_t direction, uint8_t wallsMask, bool isCrossroad)
{
  if (isCrossroad)
  {
    movementTraceBack.clear();
    movementTraceBack.push(wallsMask);
  }

  movementTraceBack.push(direction);

  if (isCrossroad and direction == BACKWARDS)
  {
    maneuverOverCrossroadToSamePosition(wallsMask);
  }
  else if (direction == FORWARDS)
  {
    bool isStartingPosition = (x == 0 and y == 0);
    if (isStartingPosition)
    {
      // goForward(START_FORWARD_ROTATIONS);
      goForward();
    }
    else
    {
      // goForward(DEFAULT_FORWARD_ROTATIONS);
      goForward();
    }
  }
  else if (direction == LEFT)
  {
    turnLeftForward();
  }
  else if (direction == RIGHT)
  {
    turnRightForward();
  }
}

void Robot::returnFromDeadEnd()
{
  uint8_t lastDirection = -1;

  while (movementTraceBack.getSize() > 1)
  {
    uint8_t currentDirection = movementTraceBack.top();
    movementTraceBack.pop();

    if (currentDirection == LEFT)
    {
      turnLeftBackwards();
    }
    else if (currentDirection == RIGHT)
    {
      turnRightBackwards();
    }
    else if (currentDirection == FORWARDS)
    {
      goBackwards();
    }

    x = x + BACKWARDS_MOVEMENT_CHANGES[orientation][X];
    y = y + BACKWARDS_MOVEMENT_CHANGES[orientation][Y];

    if (currentDirection == FORWARDS)
    {
      orientation = BACKWARDS_ORIENTATION_CHANGES[orientation][BACKWARDS];
    }
    else
    {
      orientation = BACKWARDS_ORIENTATION_CHANGES[orientation][currentDirection];
    }

    lastDirection = currentDirection;
  }

  uint8_t wallsMask = movementTraceBack.top();
  movementTraceBack.pop();

  maneuverOverCrossroadToDifferentPosition(lastDirection, wallsMask);
}

void Robot::solveMaze()
{
  uint8_t wallsMask = 0;
  uint8_t corridorsCounter = 0;
  bool isMarkerPlacedSomewhere = false;

  for (int direction = 0; direction < 4; ++direction)
  {
    if (maze.isWall(x, y, orientation, direction))
    {
      wallsMask |= (1 << direction);
    }
    else if (checkForWallUsingSensors(direction))
    {
      wallsMask |= (1 << direction);
      maze.setWall(x, y, orientation, direction);
    }
    else
    {
      ++corridorsCounter;

      int corridorMarkersCount = maze.getCorridorMarkersCount(x, y, orientation, direction);
      if (corridorMarkersCount > 0)
      {
        isMarkerPlacedSomewhere = true;
      }
    }
  }

  bool isCrossroad = (corridorsCounter > 2);
  int directionDecision = -1;

  if (isCrossroad)
  {
    directionDecision = decideCrossroad(wallsMask, isMarkerPlacedSomewhere);
  }
  else
  {
    directionDecision = getCorridorDirection(wallsMask);
  }

  bool isDeadEnd = (directionDecision == BACKWARDS and !isCrossroad);
  if (isDeadEnd)
  {
    returnFromDeadEnd();
  }
  else
  {
    physicallyMoveRobot(directionDecision, wallsMask, isCrossroad);

    x = x + MOVEMENT_CHANGES[orientation][directionDecision][X];
    y = y + MOVEMENT_CHANGES[orientation][directionDecision][Y];

    if (y < 0)
    {
      y = MAZE_SIZE - 2;
      maze.resetValues(x);
    }

    orientation = ORIENTATION_CHANGES[orientation][directionDecision];
  }
}

bool Robot::didFinish()
{
  uint8_t distanceToFinal = maze.getDistanceToFinal(x) + maze.getDistanceToFinal(y);
  return (distanceToFinal == 0);
}

void Robot::runMotors()
{
  leftMotorMotionController.run();
  rightMotorMotionController.run();
}

void Robot::goForward()
{
  leftMotorMotionController.setMaxSpeed(200.0);
  leftMotorMotionController.setAcceleration(100.0);
  leftMotorMotionController.moveTo(leftMotorMotionController.currentPosition()-mmToSteps(DISTANCE_BETWEEN_CELLS));

  rightMotorMotionController.setMaxSpeed(200.0);
  rightMotorMotionController.setAcceleration(100.0);
  rightMotorMotionController.moveTo(rightMotorMotionController.currentPosition()-mmToSteps(DISTANCE_BETWEEN_CELLS));
}

void Robot::goBackwards()
{
  leftMotorMotionController.setMaxSpeed(200.0);
  leftMotorMotionController.setAcceleration(100.0);
  leftMotorMotionController.moveTo(leftMotorMotionController.currentPosition()+mmToSteps(DISTANCE_BETWEEN_CELLS));

  rightMotorMotionController.setMaxSpeed(200.0);
  rightMotorMotionController.setAcceleration(100.0);
  rightMotorMotionController.moveTo(rightMotorMotionController.currentPosition()+mmToSteps(DISTANCE_BETWEEN_CELLS));
}

void Robot::turnLeftBackwards()
{
  leftMotorMotionController.setMaxSpeed(-200.0);
  leftMotorMotionController.setAcceleration(100.0);
  leftMotorMotionController.moveTo(leftMotorMotionController.currentPosition()+mmToSteps(DISTANCE_BETWEEN_CELLS));

  rightMotorMotionController.setMaxSpeed(-200.0);
  rightMotorMotionController.setAcceleration(100.0);
  rightMotorMotionController.moveTo(rightMotorMotionController.currentPosition()-mmToSteps(DISTANCE_BETWEEN_CELLS));
}

void Robot::turnLeftForward()
{
  leftMotorMotionController.setMaxSpeed(-100.0);
  leftMotorMotionController.setAcceleration(100.0);
  leftMotorMotionController.moveTo(leftMotorMotionController.currentPosition()+mmToSteps(DISTANCE_BETWEEN_CELLS));

  rightMotorMotionController.setMaxSpeed(100.0);
  rightMotorMotionController.setAcceleration(100.0);
  rightMotorMotionController.moveTo(rightMotorMotionController.currentPosition()-mmToSteps(DISTANCE_BETWEEN_CELLS)); 
}

void Robot::turnRightBackwards()
{
}

void Robot::turnRightForward()
{
  leftMotorMotionController.setMaxSpeed(200.0);
  leftMotorMotionController.setAcceleration(100.0);
  leftMotorMotionController.moveTo(leftMotorMotionController.currentPosition()-mmToSteps(DISTANCE_BETWEEN_CELLS));

  rightMotorMotionController.setMaxSpeed(-200.0);
  rightMotorMotionController.setAcceleration(100.0);
  rightMotorMotionController.moveTo(rightMotorMotionController.currentPosition()+mmToSteps(DISTANCE_BETWEEN_CELLS));
}

void Robot::celebrate()
{
}
