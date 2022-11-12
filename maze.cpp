#include "maze.h"

Maze::Maze()
{
  for (uint8_t x = 0; x < MAZE_SIZE; ++x)
  {
    for (uint8_t y = 0; y < MAZE_SIZE; ++y)
    {
      cellWalls[x][y] = 0;
      for (uint8_t i = 0; i < 4; ++i)
      {
        cellCorridorsMarkers[x][y][i] = 0;
      }
    }
  }
}

bool Maze::inBoundaries(uint8_t x, uint8_t y)
{
  return (x >= 0 and x < MAZE_SIZE and y >= 0 and y < MAZE_SIZE);
}

bool Maze::isWall(uint8_t x, uint8_t y, uint8_t orientation, uint8_t direction)
{
  uint8_t realOrientation = ORIENTATION_CHANGES[orientation][direction];

  return (cellWalls[x][y] & (1 << realOrientation));
}

void Maze::setWall(uint8_t x, uint8_t y, uint8_t orientation, uint8_t direction)
{
  uint8_t realOrientation = ORIENTATION_CHANGES[orientation][direction];
  cellWalls[x][y] |= (1 << realOrientation);

  // change the cell that is on the other side of the wall
  uint8_t nextX = x + MOVEMENT_CHANGES[orientation][direction][X];
  uint8_t nextY = y + MOVEMENT_CHANGES[orientation][direction][Y];

  if (inBoundaries(nextX, nextY))
  {
    cellWalls[nextX][nextY] |= (1 << OPPOSITE_ORIENTATIONS[realOrientation]);
  }
}

int Maze::getCorridorMarkersCount(uint8_t x, uint8_t y, uint8_t orientation, uint8_t direction)
{
  uint8_t realOrientation = ORIENTATION_CHANGES[orientation][direction];
  
  return cellCorridorsMarkers[x][y][realOrientation];
}

void Maze::updateCorridorMarkersCount(uint8_t x, uint8_t y, uint8_t orientation, uint8_t direction, uint8_t newMarkersCount)
{
  if (newMarkersCount > 2)
  {
    return;
  }

  uint8_t realOrientation = ORIENTATION_CHANGES[orientation][direction];
  cellCorridorsMarkers[x][y][realOrientation] = newMarkersCount;

  // change the cell that is on the other side of the wall
  uint8_t nextX = x + MOVEMENT_CHANGES[orientation][direction][X];
  uint8_t nextY = y + MOVEMENT_CHANGES[orientation][direction][Y];

  if (inBoundaries(nextX, nextY))
  {
    uint8_t oppositeOrientation = OPPOSITE_ORIENTATIONS[realOrientation];
    cellCorridorsMarkers[nextX][nextY][oppositeOrientation] = newMarkersCount;
  }
}

uint8_t Maze::getDistanceToFinal(uint8_t coordinate)
{
  if (coordinate <= MAZE_FINAL_LOW_BOUNDARY)
  {
    return MAZE_FINAL_LOW_BOUNDARY - coordinate;
  }

  return coordinate - MAZE_FINAL_HIGH_BOUNDARY;
}