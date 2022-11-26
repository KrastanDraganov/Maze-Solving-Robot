#include "robot.h"

void Robot::testDrive()
{
  Serial.print(frontSensor.measureDistance());
  Serial.print(" ");
  Serial.print(leftSensor.measureDistance());
  Serial.print(" ");
  Serial.println(rightSensor.measureDistance());

  for (int i = 0; i < 0; ++i)
  {
    goBackwards();
  }
}

Robot::Robot()
{
  x = y = 0;
  orientation = DOWN;

  leftSensor = UltrasonicSensor(A5, A4);
  rightSensor = UltrasonicSensor(A3, A2);
  frontSensor = UltrasonicSensor(A1, A0);

  leftMotor = Motor(12, 13, 6, 2, LEFT);
  rightMotor = Motor(8, 7, 5, 3, RIGHT);
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
}

bool Robot::checkForWallUsingSensors(uint8_t direction)
{
  if (direction == RIGHT)
  {
    return rightSensor.measureDistance() < CLOSE_TO_LEFT_RIGHT_WALL_DISTANCE_CM;
  }
  
  if (direction == LEFT)
  {
    return leftSensor.measureDistance() < CLOSE_TO_LEFT_RIGHT_WALL_DISTANCE_CM;
  }
  
  if (direction == FORWARD)
  {
    return frontSensor.measureDistance() < CLOSE_TO_FRONT_WALL_DISTANCE_CM;
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
      goForward(DEFAULT_FORWARD_ROTATIONS);
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
      goForward(DEFAULT_FORWARD_ROTATIONS);
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

void Robot::maneuverOverCrossroadToSamePosition(uint8_t wallsMask)
{
  bool isFrontWallSet = (wallsMask & (1 << FORWARD));
  bool isLeftWallSet = (wallsMask & (1 << LEFT));
  bool isRightWallSet = (wallsMask & (1 << RIGHT));

  if (!isFrontWallSet)
  {
    goForward(DEFAULT_FORWARD_ROTATIONS);
    
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
  else if (direction == FORWARD)
  {
    bool isStartingPosition = (x == 0 and y == 0);
    if (isStartingPosition)
    {
      goForward(START_FORWARD_ROTATIONS);
    }
    else
    {
      goForward(DEFAULT_FORWARD_ROTATIONS);
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
  Serial.print(x);
  Serial.print(" ");
  Serial.println(y);
  Serial.println(orientation);
  Serial.println();

  maze.printMaze();

  Serial.println();

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

void Robot::goForward(uint32_t rotations)
{
  uint8_t speedRight = 47;
  uint8_t speedLeft = 50;
  uint32_t counter = 0;

  rightMotor.resetTicks();
  leftMotor.resetTicks();
 
  while (counter < rotations)
  {
    rightMotor.setMotor(FORWARD, speedRight);
    leftMotor.setMotor(FORWARD, speedLeft);
 
    volatile uint32_t rightRPM = rightMotor.getRPM();
    volatile uint32_t leftRPM = leftMotor.getRPM();
    delay(10);

    counter += rightRPM;

    bool isLeftWall = (leftSensor.measureDistance() < CLOSE_TO_LEFT_RIGHT_WALL_DISTANCE_CM);
    bool isRightWall = (rightSensor.measureDistance() < CLOSE_TO_LEFT_RIGHT_WALL_DISTANCE_CM);

    if (isLeftWall and isRightWall)
    {
      speedLeft=50;
      if (leftSensor.measureDistance() < rightSensor.measureDistance())
      {
        speedLeft += 1;
      }
      else
      {
        speedLeft -= 1; 
      }
    }
    else if (isLeftWall)
    {
      speedLeft = 50;

      if (leftSensor.measureDistance() < 5)
      {
        speedLeft += 2;
      }
      else if (leftSensor.measureDistance() > 6 and leftSensor.measureDistance() < 8)
      {
        speedLeft -= 2;
      }
      else if (rightRPM > leftRPM)
      {
        speedLeft += 1;
      }
      else if (rightRPM < leftRPM)
      {
        speedLeft -= 1;
      }
    }
    else if (isRightWall)
    {
      speedLeft = 50;

      if (rightSensor.measureDistance() < 5)
      {
        speedLeft -= 2;
      }
      else if (rightSensor.measureDistance() > 6 and rightSensor.measureDistance() < 8)
      {
        speedLeft += 2;
      }
      else if (rightRPM > leftRPM)
      {
        speedLeft += 1;
      }
      else if (rightRPM < leftRPM)
      {
        speedLeft -= 1;
      }
    }
    else
    {
      if (rightRPM > leftRPM)
      {
        speedLeft += 1;
      }
      else if (rightRPM < leftRPM)
      {
        speedLeft -= 1;
      }
    }
  }

  rightMotor.stopMotor();
  leftMotor.stopMotor();

  delay(500);
}

void Robot::goBackwards()
{
  uint32_t rotations = 790;
  uint32_t counter = 0;
  
  uint8_t speedRight = 50;
  uint8_t speedLeft = 48;

  rightMotor.resetTicks();
  leftMotor.resetTicks();
 
  while (counter < rotations)
  {
 
    rightMotor.setMotor(BACKWARDS, speedRight);
    leftMotor.setMotor(BACKWARDS, speedLeft);
 
    volatile uint32_t rightRPM = rightMotor.getRPM();
    volatile uint32_t leftRPM = leftMotor.getRPM();
 
    counter += rightRPM;

    if (leftSensor.measureDistance() < 4)
    {
      speedLeft += 1;
    }
    else if (rightSensor.measureDistance() < 4)
    {
      speedLeft -= 1;  
    }
    else if (rightRPM > leftRPM)
    {
      speedLeft += 1;
    }
    else if(rightRPM < leftRPM)
    {
        speedLeft -= 1;
      }
    }
    
    /*if (speedRight > 130 or speedLeft > 130)
    {
      if (speedRight > speedLeft)
      {
        speedRight = 120 + (speedRight - speedLeft);
        speedLeft = 120;
      }
      else
      {
        speedRight = 120;
        speedLeft = 120 + (speedLeft - speedRight);
      }
    }*/
  
 
  rightMotor.stopMotor();
  leftMotor.stopMotor();

  delay(500);
}

void Robot::turnLeftBackwards() //done
{ 
  uint32_t rotations = 170;
  uint32_t counter = 0;

  uint32_t speedRight = 51;
  uint32_t speedLeft = 48;

  rightMotor.resetTicks();
  leftMotor.resetTicks();

  while (counter < rotations)
  {
    rightMotor.setMotor(BACKWARDS, speedRight);
    leftMotor.setMotor(BACKWARDS, speedLeft);

    volatile uint32_t rightRPM = rightMotor.getRPM();
    volatile uint32_t leftRPM = leftMotor.getRPM();

    counter += leftRPM;
  }
  
  rightMotor.stopMotor();
  leftMotor.stopMotor();

  delay(250);

  rotations = 720;
  counter = 0;
  
  speedRight = 70;

  rightMotor.resetTicks();
  leftMotor.resetTicks(); 

  while (counter < rotations)
  {
    rightMotor.setMotor(BACKWARDS, speedRight);
    //leftMotor.setMotor(FORWARD, speedLeft);

    volatile uint32_t rightRPM = rightMotor.getRPM();
    volatile uint32_t leftRPM = leftMotor.getRPM();

    counter += rightRPM;
  }  

  rightMotor.stopMotor();
  leftMotor.stopMotor();

  delay(250);
  
  rotations = 100;
  counter = 0;
  
  speedRight = 51;
  speedLeft = 48;
  
  rightMotor.resetTicks();
  leftMotor.resetTicks();

  while (counter < rotations)
  {
    rightMotor.setMotor(BACKWARDS, speedRight);
    leftMotor.setMotor(BACKWARDS, speedLeft);

    volatile uint32_t rightRPM = rightMotor.getRPM();
    volatile uint32_t leftRPM = leftMotor.getRPM();

    counter += leftRPM;
  }
  rightMotor.stopMotor();
  leftMotor.stopMotor();  

  delay(500);
}

void Robot::turnLeftForward()
{
  uint32_t rotations = 38;
  uint32_t counter = 0;

  uint32_t speedRight = 47;
  uint32_t speedLeft = 50;

  rightMotor.resetTicks();
  leftMotor.resetTicks();

  while (counter < rotations)
  {
    rightMotor.setMotor(FORWARD, speedRight);
    leftMotor.setMotor(FORWARD, speedLeft);

    volatile uint32_t rightRPM = rightMotor.getRPM();
    volatile uint32_t leftRPM = leftMotor.getRPM();

    counter += leftRPM;
  }
  

  rightMotor.stopMotor();
  leftMotor.stopMotor();

  delay(250);

  rotations = 670;
  counter = 0;
  
  speedRight = 70;

  rightMotor.resetTicks();
  leftMotor.resetTicks(); 

  while (counter < rotations)
  {
    rightMotor.setMotor(FORWARD, speedRight);
    //leftMotor.setMotor(FORWARD, speedLeft);

    volatile uint32_t rightRPM = rightMotor.getRPM();
    volatile uint32_t leftRPM = leftMotor.getRPM();

    counter += rightRPM;
  }  

  rightMotor.stopMotor();
  leftMotor.stopMotor();

  delay(250);
  
  rotations = 150;
  counter = 0;
  
  speedRight = 50;
  speedLeft = 50;
  
  rightMotor.resetTicks();
  leftMotor.resetTicks();

  while (counter < rotations)
  {
    rightMotor.setMotor(FORWARD, speedRight);
    leftMotor.setMotor(FORWARD, speedLeft);

    volatile uint32_t rightRPM = rightMotor.getRPM();
    volatile uint32_t leftRPM = leftMotor.getRPM();

    counter += leftRPM;
  }
  rightMotor.stopMotor();
  leftMotor.stopMotor();  

  delay(500);
}

void Robot::turnRightBackwards()
{
  uint32_t rotations = 200;
  uint32_t counter = 0;

  uint32_t speedRight = 50;
  uint32_t speedLeft = 48;

  rightMotor.resetTicks();
  leftMotor.resetTicks();

  while (counter < rotations)
  {
    rightMotor.setMotor(BACKWARDS, speedRight);
    leftMotor.setMotor(BACKWARDS, speedLeft);

    volatile uint32_t rightRPM = rightMotor.getRPM();
    volatile uint32_t leftRPM = leftMotor.getRPM();

    counter += leftRPM;
  }
  
  rightMotor.stopMotor();
  leftMotor.stopMotor();

  delay(250);

  rotations = 700;
  counter = 0;
  
  speedLeft = 70;

  rightMotor.resetTicks();
  leftMotor.resetTicks(); 

  while (counter < rotations)
  {
    leftMotor.setMotor(BACKWARDS, speedLeft);

    volatile uint32_t rightRPM = rightMotor.getRPM();
    volatile uint32_t leftRPM = leftMotor.getRPM();

    counter += leftRPM;
  }  

  rightMotor.stopMotor();
  leftMotor.stopMotor();

  delay(250);
  
  rotations = 120;
  counter = 0;
  
  speedRight = 50;
  speedLeft = 48;
  
  rightMotor.resetTicks();
  leftMotor.resetTicks();

  while (counter < rotations)
  {
    rightMotor.setMotor(BACKWARDS, speedRight);
    leftMotor.setMotor(BACKWARDS, speedLeft);

    volatile uint32_t rightRPM = rightMotor.getRPM();
    volatile uint32_t leftRPM = leftMotor.getRPM();

    counter += leftRPM;
  }
  rightMotor.stopMotor();
  leftMotor.stopMotor();  

  delay(500);
}

void Robot::turnRightForward() //done
{
  uint32_t rotations = 42; //38
  uint32_t counter = 0;

  uint32_t speedRight = 47;
  uint32_t speedLeft = 50;

  rightMotor.resetTicks();
  leftMotor.resetTicks();

  while (counter < rotations)
  {
    rightMotor.setMotor(FORWARD, speedRight);
    leftMotor.setMotor(FORWARD, speedLeft);

    volatile uint32_t rightRPM = rightMotor.getRPM();
    volatile uint32_t leftRPM = leftMotor.getRPM();

    counter += leftRPM;
  }
  

  rightMotor.stopMotor();
  leftMotor.stopMotor();

  delay(200);

  rotations = 750;
  counter = 0;
  
  speedLeft = 70;

  rightMotor.resetTicks();
  leftMotor.resetTicks(); 

  while (counter < rotations)
  {
    leftMotor.setMotor(FORWARD, speedLeft);

    volatile uint32_t rightRPM = rightMotor.getRPM();
    volatile uint32_t leftRPM = leftMotor.getRPM();

    counter += leftRPM;
  }  

  rightMotor.stopMotor();
  leftMotor.stopMotor();

  delay(200);
  
  rotations = 181;
  counter = 0;
  
  speedRight = 48;
  speedLeft = 49;
  
  rightMotor.resetTicks();
  leftMotor.resetTicks();

  while (counter < rotations)
  {
    rightMotor.setMotor(FORWARD, speedRight);
    leftMotor.setMotor(FORWARD, speedLeft);

    volatile uint32_t rightRPM = rightMotor.getRPM();
    volatile uint32_t leftRPM = leftMotor.getRPM();

    counter += leftRPM;
  }
  rightMotor.stopMotor();
  leftMotor.stopMotor();  

  delay(500);
}

void Robot::celebrate()
{
  uint32_t rotations = 642;
  uint32_t counter = 0;
  
  uint32_t speedRight = 70;

  rightMotor.resetTicks();
  leftMotor.resetTicks(); 

  while (counter < rotations)
  {
    rightMotor.setMotor(FORWARD, speedRight);
    //leftMotor.setMotor(FORWARD, speedLeft);

    volatile uint32_t rightRPM = rightMotor.getRPM();
    volatile uint32_t leftRPM = leftMotor.getRPM();

    counter += rightRPM;
  }  

  rightMotor.stopMotor();
  leftMotor.stopMotor();
}
