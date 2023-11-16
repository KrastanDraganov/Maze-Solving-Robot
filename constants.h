#ifndef CONSTANTS_H
#define CONSTANTS_H

#include "Arduino.h"

const PROGMEM uint8_t STEPPER_MOTOR_SHIELD_ADDRESS = 0x60;

const PROGMEM uint8_t STEPPER_MOTOR_STEPS_PER_REVOLUTION = 200;

const PROGMEM uint8_t LEFT_STEPPER_MOTOR_INDEX = 1;
const PROGMEM uint8_t RIGHT_STEPPER_MOTOR_INDEX = 2;

const PROGMEM uint8_t WHEEL_DIAMETER = 53.5;
const PROGMEM uint8_t STEPPER_MOTOR_STEPS_COUNT = 200;

const PROGMEM uint8_t MAZE_SIZE = 16;
const PROGMEM uint8_t MAZE_FINAL_LOW_BOUNDARY = 7;
const PROGMEM uint8_t MAZE_FINAL_HIGH_BOUNDARY = 8;

const PROGMEM uint8_t DISTANCE_BETWEEN_CELLS = 180;

const PROGMEM uint8_t MARKERS_LIMIT = 2;

const PROGMEM uint8_t RIGHT = 0;
const PROGMEM uint8_t LEFT = 1;

const PROGMEM uint8_t UP = 2;
const PROGMEM uint8_t DOWN = 3;

const PROGMEM uint8_t FORWARDS = 2;
const PROGMEM uint8_t BACKWARDS = 3;

const PROGMEM uint8_t X = 0;
const PROGMEM uint8_t Y = 1;

const PROGMEM uint8_t OPPOSITE_ORIENTATIONS[4] = {LEFT, RIGHT, DOWN, UP};

const PROGMEM uint8_t ORIENTATION_CHANGES[4][4] =
    {
        {DOWN, UP, RIGHT, LEFT}, // RIGHT
        {UP, DOWN, LEFT, RIGHT}, // LEFT
        {RIGHT, LEFT, UP, DOWN}, // UP
        {LEFT, RIGHT, DOWN, UP}  // DOWN
};

const PROGMEM uint8_t MOVEMENT_CHANGES[4][4][2] =
    {
        {{+1, 0}, {-1, 0}, {0, +1}, {0, -1}}, // RIGHT
        {{-1, 0}, {+1, 0}, {0, -1}, {0, +1}}, // LEFT
        {{0, +1}, {0, -1}, {-1, 0}, {+1, 0}}, // UP
        {{0, -1}, {0, +1}, {+1, 0}, {-1, 0}}  // DOWN
};

const PROGMEM uint8_t BACKWARDS_ORIENTATION_CHANGES[4][4] =
    {
        {UP, DOWN, LEFT, RIGHT}, // RIGHT
        {DOWN, UP, RIGHT, LEFT}, // LEFT
        {LEFT, RIGHT, DOWN, UP}, // UP
        {RIGHT, LEFT, UP, DOWN}  // DOWN
};

const PROGMEM uint8_t BACKWARDS_MOVEMENT_CHANGES[4][2] =
    {
        {0, -1}, // RIGHT
        {0, +1}, // LEFT
        {+1, 0}, // UP
        {-1, 0}  // DOWN
};

static uint8_t mmToSteps(double distance)
{
  double wheelCircumference = PI * WHEEL_DIAMETER;
  double revolutions = 1.0 * distance / wheelCircumference;
  double steps = revolutions * STEPPER_MOTOR_STEPS_COUNT;

  return (uint8_t) round(steps);
}

#endif