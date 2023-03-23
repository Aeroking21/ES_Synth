'Echoes' is a high-level thread that generates indecies of decays of volume of sound.
'Echoes' uses atomic store and load of variable RE and keyArrayMutex from 'scanKeyTask'.
The variables determines the reset of the decay value and the amount of increment of decay value.
Whenever a key is pressed, the decay value is reset to 0 and increments over time. The increment is larger for releasing the key compared to holding the key. The decay value does not reset unless the key is released and presssed again.
The outputs are atomic and used by SampleISR for generating fading echoes of sounds when a key is pressed.
'Echoes' can be used for both sawtooth wave and sine wave sounds.

EchoSwitch switches the mode between normal sound and sound with echoes. When changing values of switch, the phase accumulator and indecies for lookup table are reset to ensure no undesired sounds are generated. Moreover, in echo mode, resets of 'currentStepSize' and 'sineIdxAcc' are disabled for 'phaseAcc' and 'sineAcc' to be decayed by outputs from 'echoes'.
