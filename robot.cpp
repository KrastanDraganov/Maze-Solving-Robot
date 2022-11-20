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
    
  }
  else if (direction == LEFT)
  {
    
  }
  else if (direction == FORWARD)
  {

  }
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

void Robot::physicallyMoveRobot(uint8_t direction)
{
  // TODO
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

  physicallyMoveRobot(directionDecision);
  
  x = x + MOVEMENT_CHANGES[orientation][directionDecision][X];
  y = y + MOVEMENT_CHANGES[orientation][directionDecision][Y];

  orientation = ORIENTATION_CHANGES[orientation][directionDecision];
}

bool Robot::didFinish()
{
  uint8_t distanceToFinal = maze.getDistanceToFinal(x) + maze.getDistanceToFinal(y);
  return (distanceToFinal == 0);
}

void Robot::celebrate()
{
  // TODO - some celebration when robot reaches the final :)
}