# ES-synth-starter

  Use this project as the starting point for your Embedded Systems labs and coursework.
  
  [Lab Part 1](doc/LabPart1.md)
  
  [Lab Part 2](doc/LabPart2.md)

## Additional Information

  [Handshaking and auto-detection](doc/handshaking.md)
  
  [Double buffering of audio samples](doc/doubleBuffer.md)

## Envelope

    4 parameters: attack, delay, sustain, release
    Envelope function is piecewise, based on time t, parameters, and state of key being pressed
    If key is released, enters release mode
    returns output in range 0-255 and is multiplied by Vout to change the current volume based on state of envelope
    Vout is then scaled appropriately, by dividing by 256
    
