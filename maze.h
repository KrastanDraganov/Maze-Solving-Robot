#ifndef MAZE_H
#define MAZE_H

#include "constants.h"

class Maze
{
  private:
    uint8_t cellWalls[MAZE_SIZE][MAZE_SIZE];
    uint8_t cellCorridorsMarkers[MAZE_SIZE][MAZE_SIZE];

  public:
    Maze();

    bool inBoundaries(uint8_t x, uint8_t y);

    bool isWall(uint8_t x, uint8_t y, uint8_t orientation, uint8_t direction);
    void setWall(uint8_t x, uint8_t y, uint8_t orientation, uint8_t direction);

    int getCorridorMarkersCount(uint8_t x, uint8_t y, uint8_t orientation, uint8_t direction);
    void updateCorridorMarkersCount(uint8_t x, uint8_t y, uint8_t orientation, uint8_t direction, uint8_t newMarkersCount);

    uint8_t getDistanceToFinal(uint8_t x);
};

#endif