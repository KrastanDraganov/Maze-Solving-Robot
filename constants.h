#ifndef CONSTANTS_H
#define CONSTANTS_H

#include "Arduino.h"

const uint8_t MAZE_SIZE = 16;
const uint8_t MAZE_FINAL_LOW_BOUNDARY = 7;
const uint8_t MAZE_FINAL_HIGH_BOUNDARY = 8;

const double CLOSE_TO_LEFT_RIGHT_WALL_DISTANCE_CM = 4.0;
const double CLOSE_TO_FRONT_WALL_DISTANCE_CM = 8.0;

const uint32_t START_FORWARD_ROTATIONS = 50;
const uint32_t DEFAULT_FORWARD_ROTATIONS = 555;

const uint8_t MARKERS_LIMIT = 2;

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

const uint8_t BACKWARDS_ORIENTATION_CHANGES[4][4] =
{
  {   UP,  DOWN,   LEFT, RIGHT}, // RIGHT
  { DOWN,    UP,  RIGHT,  LEFT}, // LEFT
  { LEFT, RIGHT,   DOWN,    UP}, // UP
  {RIGHT,  LEFT,     UP,  DOWN}  // DOWN
};

const uint8_t BACKWARDS_MOVEMENT_CHANGES[4][2] =
{
  {0, -1}, // RIGHT
  {0, +1}, // LEFT
  {+1, 0}, // UP
  {-1, 0}  // DOWN
};

static volatile uint32_t ticks[2];

static void readRightEncoder()
{
  ++ticks[RIGHT];
}

static void readLeftEncoder()
{
  ++ticks[LEFT];
}

#endif