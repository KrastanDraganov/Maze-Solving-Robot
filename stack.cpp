#include "stack.h"

template <typename T>
Stack<T>::Stack()
{
  stackSize = 0;
}

template <typename T>
uint8_t Stack<T>::getSize()
{
  return stackSize;
}

template <typename T>
bool Stack<T>::empty()
{
  return (stackSize == 0);
}

template <typename T>
void Stack<T>::clear()
{
  while (!empty())
  {
    pop();
  }
}

template <typename T>
T Stack<T>::top()
{
  if (empty())
  {
    return -1;
  }

  return values[stackSize - 1];
}

template <typename T>
void Stack<T>::push(T newValue)
{
  values[stackSize++] = newValue;
}

template <typename T>
void Stack<T>::pop()
{
  if (empty())
  {
    return;
  }

  values[--stackSize] = -1;
}

template class Stack<uint8_t>;