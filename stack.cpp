#include "stack.h"

Stack::Stack()
{
  for (int i = 0; i < MAZE_SIZE + MAZE_SIZE; ++i)
  {
    values[i] = -1;
  }

  stackSize = 0;
}

uint8_t Stack::getSize()
{
  return stackSize;
}

bool Stack::empty()
{
  return (stackSize == 0);
}

uint8_t Stack::top()
{
  if (empty())
  {
    return -1;
  }

  return values[stackSize - 1];
}

void Stack::push(uint8_t newValue)
{
  values[stackSize++] = newValue;
}

void Stack::pop()
{
  if (empty())
  {
    return;
  }

  values[--stackSize] = -1;
}