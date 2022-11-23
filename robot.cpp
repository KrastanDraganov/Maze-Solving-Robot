#include "robot.h"

Robot::Robot()
{
  x = y = 0;
  orientation = DOWN;

  leftSensor = UltrasonicSensor(A5, A4);
  rightSensor = UltrasonicSensor(A3, A2);
  frontSensor = UltrasonicSensor(A1, A0);

  leftMotor = Motor(13, 12, 6, 2, LEFT);
  rightMotor = Motor(7, 8, 5, 3, RIGHT);
}

void Robot::initializeSensors()
{
  leftSensor.setupSensor();
  rightSensor.setupSensor();
  frontSensor.setupSensor();
}

void Robot::initializeMotors()
{
  leftMotor.setupMotor();
  rightMotor.setupMotor();
  noInterrupts();
}

bool Robot::checkForWallUsingSensors(uint8_t direction)
{
  if (direction == RIGHT)
  {
    return rightSensor.measureDistance() < CLOSE_TO_WALL_DISTANCE_CM;
  }
  
  if (direction == LEFT)
  {
    return leftSensor.measureDistance() < CLOSE_TO_WALL_DISTANCE_CM;
  }
  
  if (direction == FORWARD)
  {
    return frontSensor.measureDistance() < CLOSE_TO_WALL_DISTANCE_CM;
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
  bool isFrontWallSet = (wallsMask & (1 << FORWARD));
  bool isLeftWallSet = (wallsMask & (1 << LEFT));
  bool isRightWallSet = (wallsMask & (1 << RIGHT));

  if (targetDirection == LEFT)
  {
    if (!isFrontWallSet)
    {
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
      goForward();
      turnRightBackwards();
    }
    else if (!isLeftWallSet)
    {
      turnLeftForward();
      goBackwards();      
    }
  }
  else if (targetDirection == FORWARD)
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
    // maneuverOverCrossroad(false, wallsMask);
  }
  else if (direction == FORWARD)
  {
    goForward();
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
    else if (currentDirection == FORWARD)
    {
      goBackwards();        
    }

    x = x + BACKWARDS_MOVEMENT_CHANGES[orientation][X];
    y = y + BACKWARDS_MOVEMENT_CHANGES[orientation][Y];

    orientation = BACKWARDS_ORIENTATION_CHANGES[orientation][currentDirection];

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
    else if(checkForWallUsingSensors(direction))
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

    orientation = ORIENTATION_CHANGES[orientation][directionDecision];
  }
}

bool Robot::didFinish()
{
  uint8_t distanceToFinal = maze.getDistanceToFinal(x) + maze.getDistanceToFinal(y);
  return (distanceToFinal == 0);
}

void Robot::goForward()
{

}

void Robot::goBackwards()
{

}

void Robot::turnLeftBackwards()
{

}

void Robot::turnLeftForward()
{

}

void Robot::turnRightBackwards()
{

}

void Robot::turnRightForward()
{

}

void Robot::celebrate()
{
  // TODO - some celebration when robot reaches the final :)
}