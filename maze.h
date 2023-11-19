#ifndef MAZE_H
#define MAZE_H

#include "constants.h"

class Maze
{
  private:
    uint8_t cellWalls[MAZE_SIZE][MAZE_SIZE];
    uint8_t cellCorridorsMarkers[MAZE_SIZE][MAZE_SIZE][4];

  public:
    Maze();

    bool inBoundaries(int x, int y);

    bool isWall(int x, int y, uint8_t orientation, uint8_t direction);
    void setWall(int x, int y, uint8_t orientation, uint8_t direction);

    int getCorridorMarkersCount(int x, int y, uint8_t orientation, uint8_t direction);
    void updateCorridorMarkersCount(int x, int y, uint8_t orientation, uint8_t direction, uint8_t newMarkersCount);

    uint8_t getDistanceToFinal(int x);

    void resetValues(int reachedX);

    void printMaze();
};

#endif