#include "queue.h"
#include "movement.h"

MotorsMovement emptyObject;

template <typename T>
Queue<T>::Queue()
{
  frontIndex = rearIndex = 0;
}

template <typename T>
uint8_t Queue<T>::getSize()
{
  return rearIndex - frontIndex;
}

template <typename T>
bool Queue<T>::empty()
{
  return getSize() == 0;
}

template <typename T>
void Queue<T>::clear()
{
  while (!empty())
  {
    pop();
  }
}

template <typename T>
T Queue<T>::front()
{
  if (empty())
  {
    return emptyObject;
  }

  return values[frontIndex];
}

template <typename T>
void Queue<T>::push(T newValue)
{
  values[rearIndex++] = newValue;
}

template <typename T>
void Queue<T>::pop()
{
  if (empty())
  {
    return;
  }

  values[frontIndex++] = emptyObject;

  if (empty()) {
    frontIndex = rearIndex = 0;
  }
}

template class Queue<MotorsMovement>;