#ifndef QUEUE_USING_ARRAY_H
#define QUEUE_USING_ARRAY_H

#include "constants.h"

template <typename T>
class Queue
{
  private:
    T values[MAZE_SIZE * MAZE_SIZE];

    uint8_t frontIndex;
    uint8_t rearIndex;

  public:
    Queue();

    uint8_t getSize();
    bool empty();
    void clear();
    
    T front();
    void push(T newValue);
    void pop();
};

#endif