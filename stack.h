#ifndef STACK_USING_ARRAY_H
#define STACK_USING_ARRAY_H

#include "constants.h"

class Stack
{
  private:
    uint8_t values[MAZE_SIZE + MAZE_SIZE];
    uint8_t stackSize;

  public:
    Stack();

    uint8_t getSize();
    bool empty();
    
    uint8_t top();
    void push(uint8_t newValue);
    void pop();
};

#endif