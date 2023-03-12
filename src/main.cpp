#include <Arduino.h>
#include <U8g2lib.h>
#include <STM32FreeRTOS.h>
#include <string>
#include <knob.h>
#include <math.h>

//Constants
const uint32_t interval = 100; //Display update interval

const uint8_t MAX_KEYS_PLAYED_TGT = 8;

//volatile uint32_t currentStepSize;
volatile uint8_t keyArray[7];
volatile uint32_t currentStepSize[MAX_KEYS_PLAYED_TGT];

SemaphoreHandle_t keyArrayMutex;
SemaphoreHandle_t stepSizeMutex;

//Pin definitions
//Row select and enable
const int RA0_PIN = D3;
const int RA1_PIN = D6;
const int RA2_PIN = D12;
const int REN_PIN = A5;

//Matrix input and output
const int C0_PIN = A2;
const int C1_PIN = D9;
const int C2_PIN = A6;
const int C3_PIN = D1;
const int OUT_PIN = D11;

//Audio analogue out
const int OUTL_PIN = A4;
const int OUTR_PIN = A3;

//Joystick analogue in
const int JOYY_PIN = A0;
const int JOYX_PIN = A1;

//Output multiplexer bits
const int DEN_BIT = 3;
const int DRST_BIT = 4;
const int HKOW_BIT = 5;
const int HKOE_BIT = 6;

// Parameters
const int KNOB_MAX_ROTATION = 8;
const int KNOB_MIN_ROTATION = 0;

volatile signed int rotationVariable = 0;
volatile signed int knob3Rotation = 8;

//Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

// Expression to calculate step size 
constexpr int NUM_NOTES = 12;
constexpr double FREQ_RATIO = std::pow(2.0, 1.0/12.0);
constexpr double BASE_FREQ = 440.0;
constexpr double SAMPLE_RATE = 22000.0; 

constexpr uint32_t calculateStepSize(int note) {
  uint32_t freq = BASE_FREQ * std::pow(FREQ_RATIO, note);
  return (std::pow(2.0, 32.0) * freq) / SAMPLE_RATE;
}

constexpr uint32_t stepSizes[NUM_NOTES] = {
  calculateStepSize(-9),  // C
  calculateStepSize(-8), // C#
  calculateStepSize(-7), // D
  calculateStepSize(-6), // D#
  calculateStepSize(-5), // E
  calculateStepSize(-4), // F
  calculateStepSize(-3), // F#
  calculateStepSize(-2), // G
  calculateStepSize(-1), // G#
  calculateStepSize(0), // A
  calculateStepSize(1), // A#
  calculateStepSize(2) // B  
};

//Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
  digitalWrite(REN_PIN,LOW);
  digitalWrite(RA0_PIN, bitIdx & 0x01);
  digitalWrite(RA1_PIN, bitIdx & 0x02);
  digitalWrite(RA2_PIN, bitIdx & 0x04);
  digitalWrite(OUT_PIN,value);
  digitalWrite(REN_PIN,HIGH);
  delayMicroseconds(2);
  digitalWrite(REN_PIN,LOW);
}

void setRow(uint8_t rowIdx){

  digitalWrite(REN_PIN, LOW);
  digitalWrite(RA0_PIN, LOW);
  digitalWrite(RA1_PIN, LOW);
  digitalWrite(RA2_PIN, LOW);

  if (rowIdx == 0)
  {
    // digitalWrite(RA0_PIN, LOW);
  }
  if (rowIdx == 1)
  {
    digitalWrite(RA0_PIN, HIGH);
  }
  if (rowIdx == 2)
  {
    digitalWrite(RA1_PIN, HIGH);
  }
  if (rowIdx == 3)
  {
    digitalWrite(RA0_PIN, HIGH);
    digitalWrite(RA1_PIN, HIGH);
  }
  if (rowIdx == 4)
  {
    digitalWrite(RA2_PIN, HIGH);
  }
  
  digitalWrite(REN_PIN, HIGH);
}

uint8_t readCols(){
  uint8_t c0 = digitalRead(C0_PIN);
  uint8_t c1 = digitalRead(C1_PIN);
  uint8_t c2 = digitalRead(C2_PIN);
  uint8_t c3 = digitalRead(C3_PIN);

  uint8_t res = (c0 << 3) | (c1 << 2) | (c2 << 1) | c3;
  return res;
}

void sampleISR() {
  static int32_t phaseAcc[MAX_KEYS_PLAYED_TGT] = {};

  int32_t localCurrentStepSize[MAX_KEYS_PLAYED_TGT];
  int32_t keysPressed = 0;
  //maybe use a mutex and make a copy

  for (int i=0; i < MAX_KEYS_PLAYED_TGT; i++) {
    localCurrentStepSize[i] = currentStepSize[i]; 
    phaseAcc[i] += localCurrentStepSize[i];

    if(localCurrentStepSize[i] != 0){
      keysPressed++;
    } 
  }

  int32_t phaseAcc_final = 0;

  for (int i=0; i<MAX_KEYS_PLAYED_TGT; i++) {
   phaseAcc_final += phaseAcc[i] / keysPressed;
  }

  int32_t Vout = (phaseAcc_final >> 24);
  Vout = Vout >> (8 - knob3Rotation);
  analogWrite(OUTR_PIN, (Vout + 128));

}

void scanKeysTask(void * pvParameters) {

  const TickType_t xFrequency = 30/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  static signed int localknob3Rotation = 8;
  Knob knob3(3, 8, 0, 8);

  while(1){
    vTaskDelayUntil( &xLastWakeTime, xFrequency );

    uint8_t notesPressed[MAX_KEYS_PLAYED_TGT] = {128,128,128,128,128,128,128,128};

    // Serial.println("scan key loop");
    // xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    // uint32_t key_pressed = (keyArray[0] << 8) | (keyArray[1] << 4) | keyArray[2];

    // int noteIndex = ~key_pressed & 0xfff;
    // Serial.println(noteIndex);

    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);

    for (int row = 0; row < 4; row++) {
      setRow(row);
      delayMicroseconds(3);
      keyArray[row] = readCols(); 
    }
    
    int counter = 0;
    int index = 0;

    for(int i=0; i < 3; i++){

      uint8_t key = ~keyArray[i];

      for(int j=0; j < 4; j++){

        if(unsigned((key >> (3-j) & 1) == 1)){
          notesPressed[index] = counter;
          index++;
        }
        counter++;
      }
    }

    xSemaphoreGive(keyArrayMutex);
    
    int32_t localCurrentStepSize[MAX_KEYS_PLAYED_TGT] = {};
    
    for (int i = 0; i < MAX_KEYS_PLAYED_TGT; i++) {
    

      if (notesPressed[i] != 128) {
        Serial.println(notesPressed[i]);
        localCurrentStepSize[i] = stepSizes[notesPressed[i]];
      } 
      else {
        localCurrentStepSize[i] = 0;
      }
      
      xSemaphoreTake(stepSizeMutex, portMAX_DELAY);
      __atomic_store_n(&currentStepSize[i], localCurrentStepSize[i], __ATOMIC_RELAXED);
      xSemaphoreGive(stepSizeMutex);
    }

    knob3.updateRotation(keyArray);
    localknob3Rotation = knob3.getRotation();

    __atomic_store_n(&knob3Rotation, localknob3Rotation, __ATOMIC_RELAXED);
    
    // currentStepSize = lastStepSize;
    // __atomic_store_n(&currentStepSize, lastStepSize, __ATOMIC_RELAXED);
    // __atomic_store_n(&globalKeySymbol, keysymbol, __ATOMIC_RELAXED);
  }
}

void displayUpdateTask(void * pvParameters) {
  const TickType_t xFrequency2 = 100/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime2 = xTaskGetTickCount();
  
  // Update display
  u8g2.clearBuffer();         // clear the internal memory
  u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font

  while (1) {
    vTaskDelayUntil(&xLastWakeTime2, xFrequency2);

    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    uint32_t key_pressed = (keyArray[0] << 8) | (keyArray[1] << 4) | keyArray[2];
    xSemaphoreGive(keyArrayMutex);

    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setCursor(2,10);
    // Serial.println(key_pressed, BIN);

    int noteIndex = ~key_pressed & 0xfff;

    switch(noteIndex) {
      case 1:
        u8g2.print("B"); 
        break;
      case 2:
        u8g2.print("A#"); 
        break;
      case 4:
        u8g2.print("A"); 
        break;
      case 8:
        u8g2.print("G#"); 
        break;
      case 16:
        u8g2.print("G"); 
        break;
      case 32:
        u8g2.print("F#"); 
        break;
      case 64:
        u8g2.print("F"); 
        break;
      case 128:
        u8g2.print("E"); 
        break;
      case 256:
        u8g2.print("D#"); 
        break;
      case 512:
        u8g2.print("D"); 
        break;
      case 1024:
        u8g2.print("C#"); 
        break;
      case 2048:
        u8g2.print("C"); 
        break;
    }
    u8g2.setCursor(2,20);
    u8g2.print("Volume : "); 
    u8g2.print(knob3Rotation, DEC); 
    
    u8g2.sendBuffer();  
    
    digitalToggle(LED_BUILTIN);

    // UBaseType_t ux = uxTaskGetStackHighWaterMark( NULL );
    // Serial.print("stack: ");
    // Serial.println(ux);
  } 
}

void setup() {
  // put your setup code here, to run once:
  // uint32_t lastStepSize = globalLastStepSize;

  //Set pin directions
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(C0_PIN, INPUT);
  pinMode(C1_PIN, INPUT);
  pinMode(C2_PIN, INPUT);
  pinMode(C3_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);

  //Initialise display
  setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply

  //Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");

  //timer that executes interrupt that calls sampleISR()
  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();

  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
    scanKeysTask,		  /* Function that implements the task */
    "scanKeys",		    /* Text name for the task */
    64,      		      /* Stack size in words, not bytes */
    NULL,			        /* Parameter passed into the task */
    2,			          /* Task priority */
    &scanKeysHandle   /* Pointer to store the task handle */
  );	

  TaskHandle_t displayUpdateHandle = NULL;
  xTaskCreate(
    displayUpdateTask,		  /* Function that implements the task */
    "displayUpdate",		    /* Text name for the task */
    64,      		            /* Stack size in words, not bytes */
    NULL,			              /* Parameter passed into the task */
    1,			                /* Task priority */
    &displayUpdateHandle    /* Pointer to store the task handle */
  );	

  keyArrayMutex = xSemaphoreCreateMutex();
  stepSizeMutex = xSemaphoreCreateMutex();

  vTaskStartScheduler();
}

void loop() {}