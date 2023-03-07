#include <Arduino.h>
#include <U8g2lib.h>
#include <math.h>
#include <STM32FreeRTOS.h>
#include <knob.h>

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

volatile uint32_t currentStepSize = 0;
volatile uint8_t keyArray[7];
// volatile uint8_t* keyArray_ptr = keyArray;
volatile signed int rotationVariable = 0;
volatile signed int knob3Rotation = 8;

SemaphoreHandle_t keyArrayMutex;

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
  calculateStepSize(2), // B
  calculateStepSize(1), // A#
  calculateStepSize(0), // A
  calculateStepSize(-1), // G#
  calculateStepSize(-2), // G
  calculateStepSize(-3), // F#
  calculateStepSize(-4), // F
  calculateStepSize(-5), // E
  calculateStepSize(-6), // D#
  calculateStepSize(-7), // D
  calculateStepSize(-8), // C#
  calculateStepSize(-9)  // C
};

// const uint32_t bitMask[12] = {
//   0b000000000001, 0b000000000010, 0b000000000100,
//   0b000000001000, 0b000000010000, 0b000000100000,
//   0b000001000000, 0b000010000000, 0b000100000000,
//   0b001000000000, 0b010000000000, 0b100000000000};

void sampleISR() {
  static uint32_t phaseAcc = 0;
  phaseAcc += currentStepSize;

  int32_t Vout = (phaseAcc >> 24) - 128;
  
  Vout = Vout >> (8 - knob3Rotation);
  analogWrite(OUTR_PIN, Vout + 128);
}

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

uint8_t readCols(){
  // digitalWrite(REN_PIN, HIGH);
  uint8_t c0 = digitalRead(C0_PIN);
  uint8_t c1 = digitalRead(C1_PIN);
  uint8_t c2 = digitalRead(C2_PIN);
  uint8_t c3 = digitalRead(C3_PIN);

  uint8_t res = (c0 << 3) | (c1 << 2) | (c2 << 1) | c3 ;
  // Serial.println(res, BIN);
  return res;
}

void setRows(uint8_t rowIdx){

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

void scanKeysTask(void * pvParameters) {

  static uint32_t localCurrentStepSize = 0;
  static signed int localknob3Rotation = 8;
  const TickType_t xFrequency = 30/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  Knob knob3(3, 8, 0, 8);
  
  while (1) {
    // Serial.println("scan key task loop!");
    vTaskDelayUntil( &xLastWakeTime, xFrequency );

    for (int i = 0; i < 4; i++) {
      setRows(i);
      delayMicroseconds(3);
      keyArray[i] = readCols(); 
    }

    static uint8_t rotationPrevState = 0;

    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    uint32_t key_pressed = (keyArray[0] << 8) | (keyArray[1] << 4) | keyArray[2];
    xSemaphoreGive(keyArrayMutex);

    // Serial.println(key_pressed, BIN);
    int noteIndex = ~key_pressed & 0xfff;

    switch(noteIndex) {
      case 0:
        localCurrentStepSize = 0;
        break;
      case 1:
        localCurrentStepSize = stepSizes[0];
        break;
      case 2:
        localCurrentStepSize = stepSizes[1];
        break;
      case 4:
        localCurrentStepSize = stepSizes[2];
        break;
      case 8:
        localCurrentStepSize = stepSizes[3];
        break;
      case 16:
        localCurrentStepSize = stepSizes[4];
        break;
      case 32:
        localCurrentStepSize = stepSizes[5];
        break;
      case 64:
        localCurrentStepSize = stepSizes[6];
        break;
      case 128:
        localCurrentStepSize = stepSizes[7];
        break;
      case 256:
        localCurrentStepSize = stepSizes[8];
        break;
      case 512:
        localCurrentStepSize = stepSizes[9];
        break;
      case 1024:
        localCurrentStepSize = stepSizes[10];
        break;
      case 2048:
        localCurrentStepSize = stepSizes[11];
        break;
      default:
        localCurrentStepSize = 0;
        break;
    }
    // Serial.println(localCurrentStepSize);

    // uint8_t stateTransition = (rotationPrevState << 2) | rotationCurrState;
    // int saveRotationVar = rotationVariable;
    knob3.updateRotation(keyArray);
    localknob3Rotation = knob3.getRotation();
    // switch (stateTransition) {
    //   case 0b0001:
    //     rotationVariable = -1;
    //     Serial.println("Anti-clockwise!!");
    //     break;
    //   case 0b1110:
    //     rotationVariable = -1;
    //     Serial.println("Anti-clockwise!!");
    //     break;
    //   case 0b1011:
    //     rotationVariable = 1;
    //     Serial.println("Clockwise!!");
    //     break;
    //   case 0b0100:
    //     rotationVariable = 1;
    //     Serial.println("Clockwise!!");
    //     break;
    //   case 0b0011:
    //   case 0b0110:
    //   case 0b1001:
    //   case 0b1100:
    //     rotationVariable = saveRotationVar;
    //     Serial.println("Impossible!!");
    //     break;
    //   default: rotationVariable = 0;
    // }

    // saveRotationVar = rotationVariable;
    // localknob3Rotation += rotationVariable;
    // localknob3Rotation = std::max(std::min(localknob3Rotation, KNOB_MAX_ROTATION), KNOB_MIN_ROTATION);
    
    // rotationPrevState = rotationCurrState;

    // UBaseType_t ux = uxTaskGetStackHighWaterMark( NULL );
    // Serial.print("stack: ");
    // Serial.println(ux);
    __atomic_store_n(&knob3Rotation, localknob3Rotation, __ATOMIC_RELAXED);
    __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
  }
}

void displayUpdateTask(void * pvParameters) {
  const TickType_t xFrequency2 = 100/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime2 = xTaskGetTickCount();
  
  // Update display
  u8g2.clearBuffer();         // clear the internal memory
  u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font

  while (1) {
    // Serial.println("display update task loop!");
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

  // Initialise display
  setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply

  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();

  // Initialise UART
  Serial.begin(9600);

  TaskHandle_t localCurrentStepSize = NULL;
  xTaskCreate(
  scanKeysTask,		/* Function that implements the task */
  "scanKeys",		/* Text name for the task */
  64,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  2,			/* Task priority */
  &localCurrentStepSize );	/* Pointer to store the task handle */
 
  Serial.println("scan key task created!");

  TaskHandle_t displayTask = NULL;
  xTaskCreate(
  displayUpdateTask,		/* Function that implements the task */
  "displayTasks",		/* Text name for the task */
  128,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  1,			/* Task priority */
  &displayTask );	/* Pointer to store the task handle */
 
  Serial.println("display update task created!");

  keyArrayMutex = xSemaphoreCreateMutex();

  if (keyArrayMutex == NULL) {
    Serial.println("Error: Mutex was not created because the memory required to hold the mutex could not be allocated");
  }

  vTaskStartScheduler();
}

void loop() {
 
}