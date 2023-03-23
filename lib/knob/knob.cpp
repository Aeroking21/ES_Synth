#include <Arduino.h>
#include <U8g2lib.h>
#include <math.h>
#include <STM32FreeRTOS.h>
#include <knob.h>

void Knob::updateRotation(volatile uint8_t keyArray[]) volatile {
    
  switch (knobIdx) {
    case 0: 
      row = 4;
      rotationCurrState = keyArray[row];
      stateTransition = (rotationPrevState & 0b1100) | (rotationCurrState >> 2);
      break;
    case 1: 
      row = 4;
      rotationCurrState = keyArray[row];
      stateTransition = ((rotationPrevState & 0b11) << 2) | rotationCurrState;
      break;
    case 2: 
      row = 3;
      rotationCurrState = keyArray[row];
      stateTransition = (rotationPrevState & 0b1100) | (rotationCurrState >> 2);
      break;
    case 3: 
      row = 3;
      rotationCurrState = keyArray[row];
      stateTransition = ((rotationPrevState & 0b11) << 2) | rotationCurrState;
      break;
  }

  switch (stateTransition) {
    case 0b0001:
      rotationVariable = 1;
      break;
    case 0b1110:
      rotationVariable = 1;
      break;
    case 0b1011:
      rotationVariable = -1;
      break;
    case 0b0100:
      rotationVariable = -1;
      break;
    case 0b0011:
    case 0b0110:
    case 0b1001:
    case 0b1100:
      rotationVariable = saveRotationVar;
      break;
    default: rotationVariable = 0;
  }
  
  rotationPrevState = rotationCurrState;
  saveRotationVar = rotationVariable;
  rotation += rotationVariable;

  rotation = std::max(std::min(rotation, upperLimit), lowerLimit);
}

void Knob::updateSwitch(volatile uint8_t keyArray[]) volatile 
{
  int onOff = 0;
  int state = 0;
  
  switch (knobIdx) {
    case 0: 
      row = 6;
      onOff = keyArray[row] & 1;
      stateTransition = prevOnOff << 1 | onOff;
      break;
    case 1: 
      row = 6;
      onOff = (keyArray[row] >> 1) & 1;
      stateTransition = prevOnOff << 1 | onOff;
      break;
    case 2: 
      row = 5;
      onOff = keyArray[row] & 1;
      stateTransition = prevOnOff << 1 | onOff;
      break;
    case 3: 
      row = 5;
      onOff = (keyArray[row] >> 1) & 1;
      stateTransition = prevOnOff << 1 | onOff;
      break;
  }

  switch (stateTransition) {
    case 0b00: state = 0; break;
    case 0b01: state = 1; break;
    case 0b11: state = 1; break;
    case 0b10: state = 2; break;
    default: state = 0; break;
  }
  if (state == 2) pressSwitch = !pressSwitch;

  prevOnOff = onOff;
}

void Knob::setLimits(int newLowerLimit, int newUpperLimit) {
  lowerLimit = newLowerLimit;
  upperLimit = newUpperLimit;
  // Clamp the current rotation value to the new limits
  rotation = std::max(std::min(rotation, upperLimit), lowerLimit);
}

signed int Knob::getRotation() volatile {
  return rotation;
}

signed int Knob::getSwitch() volatile {
  return pressSwitch;
}

