#ifndef CONSTANTS_H
#define CONSTANTS_H

#include "Arduino.h"

const uint8_t MAZE_SIZE = 16;
const uint8_t MAZE_FINAL_LOW_BOUNDARY = 7;
const uint8_t MAZE_FINAL_HIGH_BOUNDARY = 8;

const uint8_t MARKERS_LIMIT = 2;

const uint8_t EMPTY_CORRIDOR = 0;
const uint8_t MARKED_ONCE_CORRIDOR = 1;
const uint8_t MARKED_TWICE_CORRIDOR = 2;
const uint8_t WALL_FLAG = 3;

const uint8_t RIGHT = 0;
const uint8_t LEFT = 1;

const uint8_t UP = 2;
const uint8_t DOWN = 3;

const uint8_t FORWARD = 2;
const uint8_t BACKWARDS = 3;

const uint8_t X = 0;
const uint8_t Y = 1;

const uint8_t OPPOSITE_ORIENTATIONS[4] = {LEFT, RIGHT, DOWN, UP};

const uint8_t ORIENTATION_CHANGES[4][4] =
{
  { DOWN,    UP,  RIGHT,  LEFT}, // RIGHT
  {   UP,  DOWN,   LEFT, RIGHT}, // LEFT
  {RIGHT,  LEFT,     UP,  DOWN}, // UP
  { LEFT, RIGHT,   DOWN,    UP}  // DOWN
};

const uint8_t MOVEMENT_CHANGES[4][4][2] =
{
  {{+1, 0}, {-1, 0}, {0, +1}, {0, -1}}, // RIGHT
  {{-1, 0}, {+1, 0}, {0, -1}, {0, +1}}, // LEFT
  {{0, +1}, {0, -1}, {-1, 0}, {+1, 0}}, // UP
  {{0, -1}, {0, +1}, {+1, 0}, {-1, 0}}  // DOWN
};

#endif