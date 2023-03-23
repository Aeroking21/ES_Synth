#include <U8g2lib.h>
#include <STM32FreeRTOS.h>
#include <knob.h>
#include <vector>

#ifndef VARIABLES_H
#define VARIABLES_H

#define SENDER 0
#define RECEIVER 1
#define STARTHANDSHAKE 1
#define ENDHANDSHAKE 0
#define COMPLETEHANDSHAKE 2

// #define POLYPHONY

// ---------------------- VAR ---------------------
// For keyboard modes
bool singleKeyboard = true;
bool keyboardMode = RECEIVER;
volatile int keyboardPositionIdx = 0;
int Idx = 0;
int octave = 4;
int activeNoteCount = 0;

// Polyphony setting
const uint8_t MAX_KEYS_PLAYED_TGT = 8;

Knob WaveType(1, 0, 0, 1);
Knob Volume(0, 4, 0, 8);
Knob Octave(2, 4, 3, 5);
Knob Mode(3, 0, 0, 0);
Knob Envelope(0,0,0,0);
Knob Echo(1,0,0,0);

// -------------------- VOLATILE --------------------
#ifdef POLYPHONY
volatile uint32_t currentStepSize[MAX_KEYS_PLAYED_TGT];
std::vector<uint16_t> activeNotes;
#else
volatile uint32_t currentStepSize;
#endif
volatile uint8_t keyArray[7];
volatile uint32_t sineAcc = 0;
volatile int sineIdxAcc = 0;
volatile signed int rotationVariable = 0;
volatile signed int WavetypeRotation = 8;
volatile signed int VolumeRotation = 4;
volatile signed int OctaveRotation = 4;
volatile bool ModeSwitch = 0;
volatile bool EnvelopeSwitch = 0;
volatile bool EchoSwitch = 0;
volatile uint8_t RX_Message[8] = {0};
uint32_t prevKeyPressed = 0;
uint8_t prevMessageIn[8] = {};


// ----------------- DO NOT CHANGE ------------------

const char* keyOrder[] = {"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};

// Pin definitions
// Row select and enable
const int RA0_PIN = D3; 
const int RA1_PIN = D6;
const int RA2_PIN = D12;
const int REN_PIN = A5;

// Matrix input and output
const int C0_PIN = A2;
const int C1_PIN = D9;
const int C2_PIN = A6;
const int C3_PIN = D1;
const int OUT_PIN = D11;

// Audio analogue out
const int OUTL_PIN = A4;
const int OUTR_PIN = A3;

// Joystick analogue in
const int JOYY_PIN = A0;
const int JOYX_PIN = A1;

// Output multiplexer bits
const int DEN_BIT = 3;
const int DRST_BIT = 4;
const int HKOW_BIT = 5;
const int HKOE_BIT = 6;

// Parameters
const int KNOB_MAX_ROTATION = 8;
const int KNOB_MIN_ROTATION = 0;

// Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

SemaphoreHandle_t keyArrayMutex;
SemaphoreHandle_t stepSizeMutex;
SemaphoreHandle_t RX_MessageMutex;
SemaphoreHandle_t CAN_TX_Semaphore;
SemaphoreHandle_t activeNotesMutex;
SemaphoreHandle_t keyboardPositionIdxMutex;
QueueHandle_t msgInQ;
QueueHandle_t msgOutQ;

#endif