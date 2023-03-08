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
      stateTransition = (rotationPrevState << 2) | rotationCurrState;
      break;
    case 1: 
      row = 4;
      rotationCurrState = keyArray[row];
      stateTransition = (rotationPrevState & 0b1100) | (rotationCurrState >> 2);
      break;
    case 2: 
      row = 3;
      rotationCurrState = keyArray[row];
      stateTransition = (rotationPrevState << 2) | rotationCurrState;
      break;
    case 3: 
      row = 3;
      rotationCurrState = keyArray[row];
      stateTransition = (rotationPrevState & 0b1100) | (rotationCurrState >> 2);
      break;
  }

  // Serial.println(stateTransition, BIN);

  switch (stateTransition) {
    case 0b0001:
      rotationVariable = -1;
      Serial.println("Anti-clockwise!!");
      break;
    case 0b1110:
      rotationVariable = -1;
      Serial.println("Anti-clockwise!!");
      break;
    case 0b1011:
      rotationVariable = 1;
      Serial.println("Clockwise!!");
      break;
    case 0b0100:
      rotationVariable = 1;
      Serial.println("Clockwise!!");
      break;
    case 0b0011:
    case 0b0110:
    case 0b1001:
    case 0b1100:
      rotationVariable = saveRotationVar;
      Serial.println("Impossible!!");
      break;
    default: rotationVariable = 0;
  }
  
  rotationPrevState = rotationCurrState;
  saveRotationVar = rotationVariable;
  rotation += rotationVariable;

  rotation = std::max(std::min(rotation, upperLimit), lowerLimit);
  // Serial.println(rotation);
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

