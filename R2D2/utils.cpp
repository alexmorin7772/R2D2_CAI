/*
Utils.cpp, created by hmorin
C++ sourcefile for "Touch.ino"
Using an extra function for millis() time calculations
*/

#include "utils.h"

unsigned long getTicksDuration(unsigned long prevTicks, unsigned long currTicks) {
  if (prevTicks > currTicks) {                         // System time overflows (through 0)
    currTicks += (unsigned long)(-1) - prevTicks + 1;  // Shift current time forward relative to zero reference
    prevTicks = 0;
  }

  return (currTicks - prevTicks);  // Now just return the difference
}