#include <U8g2lib.h>

#ifndef KNOB_H
#define KNOB_H

class Knob {
  public:
    Knob(int knobIdx, int initialRotation = 8, int lowerLimit = 0, int upperLimit = 8) :
      knobIdx(knobIdx),
      rotation(initialRotation),
      lowerLimit(lowerLimit),
      upperLimit(upperLimit) {}

    void updateRotation(volatile uint8_t keyArray[]) volatile;
    void setLimits(int newLowerLimit, int newUpperLimit); 
    signed int getRotation() volatile;

  private:
    signed int rotation;
    int lowerLimit;
    int upperLimit;
    int rotationCurrState = 0;
    int rotationPrevState = 0;
    signed int rotationVariable = 0;
    int saveRotationVar = 0;
    uint8_t stateTransition = 0;
    int row;
    int knobIdx = 0;
};

#endif