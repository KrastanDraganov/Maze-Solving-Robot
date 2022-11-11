#ifndef CONSTANTS_H
#define CONSTANTS_H

const uint8_t RIGHT = 0;
const uint8_t LEFT = 1;

const uint8_t UP = 2;
const uint8_t DOWN = 3;

const uint8_t FORWARD = 2;
const uint8_t BACKWARDS = 3;

const uint8_t ORIENTATION_CHANGES[4][4] =
{
  { DOWN,    UP, RIGHT,  LEFT}, // RIGHT
  {   UP,  DOWN,  LEFT, RIGHT}, // LEFT
  {RIGHT,  LEFT,    UP,  DOWN}, // UP
  { LEFT, RIGHT,  DOWN,    UP}  // DOWN
//  RIGHT   LEFT   DOWN    UP
};

const uint8_t MOVEMENT_CHANGES[4][4][2] =
{
  {{+1, 0}, {-1, 0}, {0, +1}, {0, -1}}, // RIGHT
  {{-1, 0}, {+1, 0}, {0, -1}, {0, +1}}, // LEFT
  {{0, +1}, {0, -1}, {-1, 0}, {+1, 0}}, // UP
  {{0, -1}, {0, +1}, {+1, 0}, {-1, 0}}  // DOWN
//   RIGHT     LEFT     DOWN       UP
};

#endif