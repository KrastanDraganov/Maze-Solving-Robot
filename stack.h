#ifndef STACK_USING_ARRAY_H
#define STACK_USING_ARRAY_H

#include "constants.h"

template <typename T>
class Stack
{
  private:
    T values[MAZE_SIZE * MAZE_SIZE];
    uint8_t stackSize;

  public:
    Stack();

    uint8_t getSize();
    bool empty();
    void clear();
    
    T top();
    void push(T newValue);
    void pop();
};

#endif