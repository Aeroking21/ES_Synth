#include <U8g2lib.h>
#include <math.h>

#ifndef WAVESTYPE_H
#define WAVESTYPE_H

const int NUM_NOTES = 12;
double FREQ_RATIO =  std::pow(2.0, 1.0/12.0);
double BASE_FREQ = 440.0;
double SAMPLE_RATE = 22000.0; 

uint32_t calculateStepSizeforSawtooth(int note) {
  uint32_t freq = BASE_FREQ * std::pow(FREQ_RATIO, note);
  return (std::pow(2.0, 32.0) * freq) / SAMPLE_RATE;
}

const uint32_t stepSizes[NUM_NOTES] = {
  calculateStepSizeforSawtooth(-9),  // C
  calculateStepSizeforSawtooth(-8), // C#
  calculateStepSizeforSawtooth(-7), // D
  calculateStepSizeforSawtooth(-6), // D#
  calculateStepSizeforSawtooth(-5), // E
  calculateStepSizeforSawtooth(-4), // F
  calculateStepSizeforSawtooth(-3), // F#
  calculateStepSizeforSawtooth(-2), // G
  calculateStepSizeforSawtooth(-1), // G#
  calculateStepSizeforSawtooth(0), // A
  calculateStepSizeforSawtooth(1), // A#
  calculateStepSizeforSawtooth(2) // B
};

const uint32_t TABLE_SIZE = 10000;
const uint32_t SCALE_FACTOR = 0xffffffff;

uint32_t sineLookUpTable[TABLE_SIZE];

void generateSineLookUp () 
{
  for (int i = 0; i < TABLE_SIZE; i++) {
      double angle = 2 * PI * i / TABLE_SIZE;
      double sineValue = std::sin(angle);
      uint32_t fixedPointValue = (uint32_t)(sineValue * SCALE_FACTOR);
      sineLookUpTable[i] = fixedPointValue;
  }
}

uint32_t generateSineLookUpIdxAcc(int note)
{
  uint32_t frequency = BASE_FREQ * std::pow(FREQ_RATIO, note);
  int IdxAcc = frequency / SAMPLE_RATE * TABLE_SIZE;
  return IdxAcc;
}

const uint32_t sineLookUpAcc[NUM_NOTES] = {
  generateSineLookUpIdxAcc(-9),  // C
  generateSineLookUpIdxAcc(-8), // C#
  generateSineLookUpIdxAcc(-7), // D
  generateSineLookUpIdxAcc(-6), // D#
  generateSineLookUpIdxAcc(-5), // E
  generateSineLookUpIdxAcc(-4), // F
  generateSineLookUpIdxAcc(-3), // F#
  generateSineLookUpIdxAcc(-2), // G
  generateSineLookUpIdxAcc(-1), // G#
  generateSineLookUpIdxAcc(0), // A
  generateSineLookUpIdxAcc(1), // A#
  generateSineLookUpIdxAcc(2) // B
};

#endif