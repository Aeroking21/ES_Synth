# Documentation (Team Tokyo)

## Table of content

- [Identification of Tasks](./README.md#identification-of-tasks)
- [Charatecrisation of Tasks](./README.md#characterisation-of-tasks)
- [Critical Instant Analysis](./README.md#real-time-critical-analysis)
- [Total CPU Utilisation](./README.md#total-cpu-utilisation)
- [Identification of shared data structures](./README.md#shared-resources)
- [Inter-task Blocking Dependencies](./README.md#dependencies)
- [Advanced Features](./README.md#advanced-features)
- [User Interface](./README.md#user-interface)
  </br>

## Identification of Tasks

### Threads:

- `scanKeysTask`, Priority: 3

  - Scans `keyArray[row]` to check the keys pressed for row 0 to 2, bits manipulation was used to concatenate them to form a single variable; other rows for Knobs (Volume, WaveType, Octave, Mode, Envelope, Echo) rotation and switch detection

  - If `keyboardMode == RECEIVER`, updates the `phaseAcc` (depending on wavetypes)

  - If `keyboardMode == SENDER`, transmits message in the format given in the lab with additional bit `TX_Message[3]` as the keyboard position index

- `displayUpdateTask`, Priority: 1

  - Displays key pressed, volume, wave type, octave, mode on main display -> `RECEIVER`, message transmitted when `!singleKeyboard`

- `decodeTask`: Priority: 2 (only applies to `RECEIVER`)

  - If messages received changes, decodes the message and updates `phaseAcc` (depending on wavetypes), keyboard position and octave

- `CAN_TX_Task`:, Priority: 2 (only applies to `SENDER`) - Transmits message

### Interrupts:

- `sampleISR`: With frequency of `22000Hz`, updates `Vout` and writes to the pin for the speaker output

- `CAN_RX_ISR`: Receives message from FIFO (`msgInQ`)
- `CAN_TX_ISR`: Transmit message from FIFO (`msgOutQ`)

</br>

## Characterisation of Tasks

The theoretical minimum initiation interval and measured worst execution time for each tasks are tabulated below.

| Task                | Priority (Low to High) | Minimum Initiation Interval (us) | Worst-case Execution Time (us) | Latency (ms) | CPU Utilisation (%) |
| :------------------ | :--------------------: | :------------------------------: | :----------------------------: | :----------: | :-----------------: |
| `displayUpdateTask` |           1            |              100000              |            15683.19            |      1       |                     |
| `decodeTask`        |      3 (RECEIVER)      |              25200               |              4.81              |              |                     |
| `CAN_TX_Task`       |       2 (SENDER)       |              60000               |             61.56              |              |                     |
| `scanKeyTask`       |           3            |              20000               |             86.44              |              |                     |
| `sampleISR`         |       Interrupt        |              45.45               |             11.91              |              |                     |
| `CAN_RX_ISR`        |       Interrupt        |                                  |             0.039              |              |                     |
| `CAN_TX_ISR`        |       Interrupt        |                                  |                                |              |                     |
|                     |                        |                                  |           **Total**            |              |                     |

</br>

## Total CPU Utilisation

### Memory Usage

RAM - 65.8%
Flash - 28.3%

### Stack Usage for each Thread

- displayUpdate - 128 words (256 allocated)
- scanKeyTask - 60 words (64 allocated)
- decodeTask - 61 words (64 allocated)
- CAN_TX_Task - 44 words (64 allocated)

## Real Time Critical Analysis

<!-- A critical time analysis is crucial to predict whether all the tasks will be executed within the deadlines of a system.
To do this, it is necessary to analyse the total latency of the system by computing the execution times for the worst case scenario possible and compare it to the latency of the lowest-priority task. -->

<!-- The total latency obtained is 39.7ms, which is clearly less than the latency of our lowest-priority task `displayUpdateTask`: 100ms. Therefore none of the deadlines will be missed and our schedule will work without failures as all the tasks will be executed in the correct time frame.

_Note: The execution times of `CAN_TX_Task` and of `CAN_TX_ISR` were measured together as the former depends on the latter when simulating its worst-case scenario (a full queue of outgoing messages)._

The total CPU utilisation of our program is around **40%**.  -->

</br>

## Shared Resources

All data and other resources that are accessed by multiple tasks is protected against errors caused by simultaneous access.

- **4 Mutexes** - Used to protect access by multiple tasks

  - `keyArrayMutex`: Copied locally using `memcpy` in threads to reduce locking time, shared by thread `scanKeysTask` & `displayUpdateTask`
  - `RX_MessageMutex`: Copied locally using `memcpy` in threads to reduce locking time, shared by thread `decodeTask` & `displayUpdateTask`
  - `keyboardPositionIdxMutex`: shared by thread `scanKeysTask` & `decodeTask`

- **2 Queues** - Uses FIFO buffer to reduce worst-case utilisation as one consider average initiation interval instead of peak

  - `msgInQ`
  - `msgOutQ`: Guarded by Counting Semaphores

- **Atomic operations** - Prevent threads from stalling and ensure that the operation completed in a single CPU operation

  - `EchoSwitch`
  - `EnvelopeSwitch`
  - `ModeSwitch`
  - `VolumeRotation`
  - `OctaveRotation`
  - `WavetypeRotation`
  - `currentStepSize`: Only accessed by the `sampleISR` interrupt hence cannot be protected by a Mutex

<!-- 1. Multiple objects of custom classes [CAN_Knob](./lib/Knob/can_knob.hpp), [Button](./lib/Button/button.hpp) and [Detect](./lib/Detect/detect.hpp) whose member variables are all written to using atomic operations, as they can be accessed in interrupts, such as `knob3.getRotation()` in the `sampleISR` interrupt. -->

## Inter-task Blocking Dependencies

All dependencies between the tasks of our program can be visualized in a dependency graph: red arrow represents blocking dependencies whereas green represents non-blocking dependencies.

Mutexes can only be accessed by one thread at a time hence would cause stalling issues. However, mutex is always c
Atomic operations is ignored, as they do not cause problems with dependencies.

<img src="./diagrams/dependencygraph1.pdf" alt="Single Keyboard" width="550">

<img src="./diagrams/dependencygraph2.pdf" alt="Multiply Keyboards" width="550">

<!--
All dependencies between the tasks of our program can be visualized in a dependency graph:

<img src="./dependency_graph.jpg" alt="Dependency Graph" width="550"/>

Where all red dependencies are external dependencies, interrupts are represented by an ellipsis, threads are represented by a rounded rectangle, and queues have been explicitely represented as green rectangles for a more detailed representation of dependencies.

We note that mutexes are ignored in this dependency graph as they contain non-blocking operations, and always unlock the ressource after a short period of time. Atomic operations are also ignored, as they do not cause problems with dependencies.

As shown in the graph, the RX and TX queues, respectively `msgInQ` and `msgOutQ` are dependencies for all tasks accessing them. This is because they are blocking whenever a task wants to read from them but they are empty (such as for the `decodeTask`), as well as when a task wants to write to them but they are full (such as for `CAN_TX_Task`).

The dependency between `CAN_TX_Task` and `CAN_TX_ISR` is because of the Counting Semaphore `CAN_TX_Semaphore` that regulates the flow of outgoing messages to the CAN Bus. This is because the STM32 can only load three messages at a time to be sent out to the bus. This semaphore therefore blocks the `CAN_TX_Task` thread until an output slot is free.

As this graph is acyclic (i.e. there are no cycles/loops), this means that there are **no risks of deadlocks** in our program. -->

</br>

## Advanced Features

- [Knob class](lib/knob)

  - For scalability, `knob.h` and `knob.cpp` are written to get the rotation and state of switch for knobs 0-3
  - Ensured thread safe by passing in local copy of `keyArray[i]`
  - Methods include `getRotation`, `updateRotation`, `updateSwitch`, `getSwitch` and `setLimits`

- Echoes

  - Integrated with multiple keyboards and can be used for both sawtooth wave and sine wave.
  - High-level thread that generates indecies of decays of volume of sound.
  - Uses atomic store and load of variable RE and `keyArrayMutex` from `scanKeyTask` to reset the decay value and the increment size of decay value.
  - Whenever a key is pressed, the decay value is reset to `0` and increments over time. The increment is larger when releasing the key compared to when holding the key. The decay value does not reset unless the key is released and presssed again.
  - When `keypressed` changes and `== 0`, the phase is reset to zero. In echo mode, resets of `currentStepSize` and `sineIdxAcc` are disabled for the volume to decay.

- [Sine Wave](lib/wavestype/wavestype.h)

  - Lookup table is generated upon setup based on `TABLE_SIZE` set
  - Different keys have different index accumulation to achieve the different frequency

- Envelope

  - 4 parameters: attack, delay, sustain, release
  - Piecewise function, based on time `t`, parameters, and state of key being pressed
  - If key is released, enters release mode
  - Returns output in range 0-255 and is multiplied by Vout to change the current volume based on state of envelope
  - Vout is then scaled appropriately, by dividing by `256`

- Polyphony
  - Detect all the simultaneous keys pressed and storing it in a global array. In the `sampleISR()` it takes the average of all the stepSizes of all keys pressed and adds to phase Accumulator.
  - Currently only working with single keyboard, uncomment `#define POLYPHONY` to use this feature.

<!-- As part of this project, we have implemented several advanced features:

- A board mode Button (Button of Knob 0) that, when being pressed, sets the current board as Receiver and all other connected boards to be Senders. (_Note: by default all boards are set to be receivers until a specific one is chosen_)

- A mute-switch has been implemented: by simply pressing on the Button of Knob 3 (the right-most knob), the output speaker of the current board will be disabled (and a `X` will be displayed instead of the volume level after `Vol:`).

- The octave of a Receiver board can be chosen using Knob 2 (displayed on the screen as `Oct:`).

- We have also implemented support for Sinusoidal Waveforms, which can be chosen with Knob 1 (displayed on the screen as `~:`). This was implemented using look-up tables that are generated on start-up of the board (the implementation for this feature is in [sample_library.hpp](./lib/SampleLibrary/sample_library.hpp)).

- We have implemented polyphony for the Sinusoidal Waveform, such that multiple notes from multiple boards with different octaves, can be played at once (without clipping).

- We have implemented Handshaking with up to 3 boards to determine their relative positions (displayed as `Idx` on their respective screens).

- This handshaking is then used to dynamically change the octaves of all contiguous boards, such that a board is always set to an octave higher than the one on its left.

- When multiple boards are connected, 'Knob 3' which controls the volume and 'Knob 1' which changes the type of waveform, are synchronized across all the boards. This is implemented using the [CAN_Knob](./lib/Knob/can_knob.hpp) custom object class which makes this code easily scalable to accomodate more advanced features. -->
