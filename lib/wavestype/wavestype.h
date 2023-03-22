#include <U8g2lib.h>
#include <math.h>

#ifndef WAVESTYPE_H
#define WAVESTYPE_H

constexpr int NUM_NOTES = 12;
constexpr double FREQ_RATIO = std::pow(2.0, 1.0/12.0);
constexpr double BASE_FREQ = 440.0;
constexpr double SAMPLE_RATE = 22000.0; 

constexpr uint32_t calculateStepSizeforSawtooth(int note) {
  uint32_t freq = BASE_FREQ * std::pow(FREQ_RATIO, note);
  return (std::pow(2.0, 32.0) * freq) / SAMPLE_RATE;
}

constexpr uint32_t stepSizes[NUM_NOTES] = {
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
  calculateStepSizeforSawtooth(2), // B
};

// Add sine wave look up table

#endif