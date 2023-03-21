#include <Arduino.h>
#include <U8g2lib.h>
#include <math.h>
#include <STM32FreeRTOS.h>
#include <knob.h>
#include <ES_CAN.h>

const uint8_t MAX_KEYS_PLAYED_TGT = 8;

int octave = 4;

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

volatile uint32_t currentStepSize;
volatile uint8_t keyArray[7];
// volatile uint8_t* keyArray_ptr = keyArray;
volatile signed int rotationVariable = 0;
volatile signed int knob3Rotation = 8;

volatile uint8_t RX_Message[8] = {0};

SemaphoreHandle_t keyArrayMutex;
SemaphoreHandle_t stepSizeMutex;
SemaphoreHandle_t RX_MessageMutex;
SemaphoreHandle_t CAN_TX_Semaphore;
QueueHandle_t msgInQ;
QueueHandle_t msgOutQ;

const char* keyOrder[] = {"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};

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
  calculateStepSize(2), // B
};

const uint32_t bitMask[12] = {
  0b000000000001, 0b000000000010, 0b000000000100,
  0b000000001000, 0b000000010000, 0b000000100000,
  0b000001000000, 0b000010000000, 0b000100000000,
  0b001000000000, 0b010000000000, 0b100000000000};

void sampleISR() {

  static uint32_t phaseAcc = 0;
  phaseAcc += currentStepSize;

  // static int32_t phaseAcc[MAX_KEYS_PLAYED_TGT] = {};

  // int32_t localCurrentStepSize[MAX_KEYS_PLAYED_TGT];
  // int32_t localCurrentStepSize;
  // int32_t keysPressed = 0;
  //maybe use a mutex and make a copy

  // for (int i=0; i < MAX_KEYS_PLAYED_TGT; i++) {
  //   localCurrentStepSize[i] = currentStepSize[i]; 
  //   phaseAcc[i] += localCurrentStepSize[i];

  //   if(localCurrentStepSize[i] != 0){
  //     keysPressed++;
  //   } 
  // }

  // int32_t phaseAcc_final = 0;

  // for (int i=0; i<MAX_KEYS_PLAYED_TGT; i++) {
  //  phaseAcc_final += phaseAcc[i] / keysPressed;
  // }
  

  int32_t Vout = (phaseAcc >> 24) - 128;
  
  Vout = Vout >> (8 - knob3Rotation);
  analogWrite(OUTR_PIN, Vout + 128);


  // int32_t Vout = (phaseAcc_final >> 24);
  // Vout = Vout >> (8 - knob3Rotation);
  // Vout = Vout << (RX_Message[1] - 4);
  // analogWrite(OUTR_PIN, (Vout + 128));

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

  uint8_t res = (c3 << 3) | (c2 << 2) | (c1 << 1) | c0 ;
  // Serial.println(res, BIN);
  return res;
}

void setRows(uint8_t rowIdx){

  digitalWrite(REN_PIN, LOW);
  digitalWrite(RA0_PIN, LOW);
  digitalWrite(RA1_PIN, LOW);
  digitalWrite(RA2_PIN, LOW);

  switch (rowIdx) {
  case 1:
    digitalWrite(RA0_PIN, HIGH);
    break;
  case 2:
    digitalWrite(RA1_PIN, HIGH);
    break;
  case 3:
    digitalWrite(RA0_PIN, HIGH);
    digitalWrite(RA1_PIN, HIGH);
    break;
  case 4:
    digitalWrite(RA2_PIN, HIGH);
    break;
  case 5:
    digitalWrite(RA0_PIN, HIGH);  
    digitalWrite(RA2_PIN, HIGH);  
    break;
  case 6:
    digitalWrite(RA1_PIN, HIGH);  
    digitalWrite(RA2_PIN, HIGH);  
    break;
  case 7:
    digitalWrite(RA0_PIN, HIGH);  
    digitalWrite(RA1_PIN, HIGH);  
    digitalWrite(RA2_PIN, HIGH);  
    break;
  default: break;
  }

  digitalWrite(REN_PIN, HIGH);
}

void scanKeysTask(void * pvParameters) {
  
  const TickType_t xFrequency = 10/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  uint32_t ID = 0x123;
  uint8_t TX_Message[8] = {0};
  int boardIndex = 0;
  static int localOctave = 4;

  static uint32_t localCurrentStepSize = 0;
  static signed int localknob3Rotation = 8;

  bool outBits[] = {1,1,1,1,1,1,1};

  // uint32_t notesPressed[8] = {128, 128, 128, 128, 128, 128, 128, 128};

  Knob knob3(3, 8, 0, 8);
  
  while (1) {
    // Serial.println("scan key task loop!");
    vTaskDelayUntil( &xLastWakeTime, xFrequency );

    for (int i = 0; i < 7; i++) {
      setRows(i);
      digitalWrite(OUT_PIN, outBits[i]); //Set value to latch in DFF
      digitalWrite(REN_PIN, 1);          //Enable selected row
      delayMicroseconds(3);
      keyArray[i] = readCols(); 
      digitalWrite(REN_PIN,0);          //Disable selected row
    }

    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    uint32_t key_pressed = ((~keyArray[2] & 0xf) << 8) | ((~keyArray[1] & 0xf) << 4) | (~keyArray[0] & 0xf);
    knob3.updateRotation(keyArray);
    localknob3Rotation = knob3.getRotation();
    int eastDetect = ~keyArray[6] & 0x1;
    int westDetect = ~keyArray[5] & 0x1;
    xSemaphoreGive(keyArrayMutex);

    int detectKeyOneHot = key_pressed & 0xfff;

    int exponent = 0;
    int noteIndex = 0;
    while (detectKeyOneHot >= 1) {
      detectKeyOneHot >>= 1;
      exponent++;
    }

    // ------ Debugging purposes
    // Serial.println(detectKeyOneHot, BIN);

    // if (eastDetect) {
    //   boardIndex = 0;
    // }
    // else if (westDetect) {
    //   boardIndex = 1;
    // }
    // localOctave -= boardIndex;
    // __atomic_store_n(&octave, localOctave, __ATOMIC_RELAXED);

    // int counter = 0;
    // int index = 0;
    // for(int i=0; i < 3; i++){

    //   uint8_t key = ~keyArray[i];
    //   for(int j=0; j < 4; j++){

    //     if(unsigned((key >> j) & 1) == 1){
    //       notesPressed[index] = counter;
    //       index++;
    //     }
    //     counter++;
    //   }
    // }
    
    // int32_t localCurrentStepSize[MAX_KEYS_PLAYED_TGT] = {};
    
    // for (int i = 0; i < MAX_KEYS_PLAYED_TGT; i++) {
    

    //   if (notesPressed[i] != 128) {
    //     // Serial.println(notesPressed[i]);
    //     localCurrentStepSize[i] = stepSizes[notesPressed[i]];
    //   } 
    //   else {
    //     localCurrentStepSize[i] = 0;
    //   }
      
    //   xSemaphoreTake(stepSizeMutex, portMAX_DELAY);
    //   __atomic_store_n(&currentStepSize[i], localCurrentStepSize[i], __ATOMIC_RELAXED);
    //   xSemaphoreGive(stepSizeMutex);
    // }

    if (key_pressed != 0x0){
      
      TX_Message[0] = 'P';
      TX_Message[1] = 4;
      
      if (exponent == 0) {
        localCurrentStepSize = 0;
      }
      else {
        localCurrentStepSize = stepSizes[exponent-1];
        TX_Message[2] = exponent-1;
      }
      
      // switch(detectKeyOneHot) {
      //   case 0: localCurrentStepSize = 0; break;
      //   case 1: 
      //     localCurrentStepSize = stepSizes[0]; 
      //     TX_Message[2] = 0;
      //     break;
      //   case 2: 
      //     localCurrentStepSize = stepSizes[1]; 
      //     TX_Message[2] = 1;
      //     break;
      //   case 4: 
      //     localCurrentStepSize = stepSizes[2]; 
      //     TX_Message[2] = 2;
      //     break;
      //   case 8: 
      //     localCurrentStepSize = stepSizes[3]; 
      //     TX_Message[2] = 3;
      //     break;
      //   case 16: 
      //     localCurrentStepSize = stepSizes[4]; 
      //     TX_Message[2] = 4;
      //     break;
      //   case 32: 
      //     localCurrentStepSize = stepSizes[5]; 
      //     TX_Message[2] = 5;
      //     break;
      //   case 64: 
      //     localCurrentStepSize = stepSizes[6]; 
      //     TX_Message[2] = 6;
      //     break;
      //   case 128: 
      //     localCurrentStepSize = stepSizes[7]; 
      //     TX_Message[2] = 7;
      //     break;
      //   case 256: 
      //     localCurrentStepSize = stepSizes[8]; 
      //     TX_Message[2] = 8;
      //     break;
      //   case 512: 
      //     localCurrentStepSize = stepSizes[9]; 
      //     TX_Message[2] = 9;
      //     break;
      //   case 1024: 
      //     localCurrentStepSize = stepSizes[10]; 
      //     TX_Message[2] = 10;
      //     break;
      //   case 2048: 
      //     localCurrentStepSize = stepSizes[11]; 
      //     TX_Message[2] = 11;
      //     break;
      //   default:
      //     localCurrentStepSize = 0;
      //     break;
      // }
    }
    else {
      localCurrentStepSize = 0;
      TX_Message[0] = 'R';
    }

    uint32_t key_pressed_prev = key_pressed;

    __atomic_store_n(&knob3Rotation, localknob3Rotation, __ATOMIC_RELAXED);
    // __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);

    xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
  }
}

void displayUpdateTask(void * pvParameters) {
  uint32_t ID = 0x123;
  const TickType_t xFrequency2 = 100/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime2 = xTaskGetTickCount();
  
  // Update display
  u8g2.clearBuffer();         // clear the internal memory
  u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
  
  while (1) {
      
    vTaskDelayUntil(&xLastWakeTime2, xFrequency2);

    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    uint32_t key_pressed = ((~keyArray[2] & 0xf) << 8) | ((~keyArray[1] & 0xf) << 4) | (~keyArray[0] & 0xf);
    xSemaphoreGive(keyArrayMutex);

    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setCursor(2,10);
    // Serial.println(key_pressed, BIN);

    int detectKeyOneHot = key_pressed & 0xfff;

    int exponent = 0;
    int noteIndex = 0;
    while (detectKeyOneHot >= 1) {
      detectKeyOneHot >>= 1;
      exponent++;
    }

    if (exponent != 0) {
      u8g2.print(keyOrder[exponent-1]); 
    }

    // switch(detectKeyOneHot) {
    //   case 1:
    //     u8g2.print(keyOrder[0]); 
    //     break;
    //   case 2:
    //     u8g2.print(keyOrder[1]); 
    //     break;
    //   case 4:
    //     u8g2.print(keyOrder[2]); 
    //     break;
    //   case 8:
    //     u8g2.print(keyOrder[3]); 
    //     break;
    //   case 16:
    //     u8g2.print(keyOrder[4]); 
    //     break;
    //   case 32:
    //     u8g2.print(keyOrder[5]); 
    //     break;
    //   case 64:
    //     u8g2.print(keyOrder[6]); 
    //     break;
    //   case 128:
    //     u8g2.print(keyOrder[7]); 
    //     break;
    //   case 256:
    //     u8g2.print(keyOrder[8]); 
    //     break;
    //   case 512:
    //     u8g2.print(keyOrder[9]); 
    //     break;
    //   case 1024:
    //     u8g2.print(keyOrder[10]); 
    //     break;
    //   case 2048:
    //     u8g2.print(keyOrder[11]); 
    //     break;
    // }
    u8g2.setCursor(2,20);
    u8g2.print("Volume : "); 
    u8g2.print(knob3Rotation, DEC); 
    
    // xSemaphoreTake(RX_MessageMutex, portMAX_DELAY);
    // u8g2.print(RX_Message[1]);
    // u8g2.print(RX_Message[2]);
    // xSemaphoreGive(RX_MessageMutex);

    uint8_t localRX_Message[8];
    xSemaphoreTake(RX_MessageMutex, portMAX_DELAY);
      memcpy(&localRX_Message, (void*) &RX_Message, 8);
    xSemaphoreGive(RX_MessageMutex);
    u8g2.setCursor(66,30);
    u8g2.print((char) localRX_Message[0]);
    u8g2.print(localRX_Message[1]);
    u8g2.print(localRX_Message[2]);
    
    u8g2.sendBuffer();  
    
    digitalToggle(LED_BUILTIN);

    // UBaseType_t ux = uxTaskGetStackHighWaterMark( NULL );
    // Serial.print("stack: ");
    // Serial.println(ux);
  } 
}

void decodeTask(void * pvParameters) {
  uint32_t ID = 0x123;
  // static uint32_t localCurrentStepSize = 0;
  // int32_t localCurrentStepSize[MAX_KEYS_PLAYED_TGT] = {};
  int32_t localCurrentStepSize = 0;

  uint8_t message[8] = {0};
  // uint8_t notesPressed[MAX_KEYS_PLAYED_TGT] = {128,128,128,128,128,128,128,128};
  
  while (1) {

    if (xQueueReceive(msgInQ, message, portMAX_DELAY) == pdTRUE){
      xSemaphoreTake(RX_MessageMutex, portMAX_DELAY);
      memcpy((void*)RX_Message, &message, 8);
      xSemaphoreGive(RX_MessageMutex);

      if (message[0] == 'R'){
        localCurrentStepSize = 0;
      }
      else {
        // Serial.println("key pressed!");
        localCurrentStepSize = stepSizes[RX_Message[2]];
      }

      xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
      uint32_t key_pressed = ((~keyArray[2] & 0xf) << 8) | ((~keyArray[1] & 0xf) << 4) | (~keyArray[0] & 0xf);
      xSemaphoreGive(keyArrayMutex);

      // for (int i = 0; i < MAX_KEYS_PLAYED_TGT; i++) {
    
      //   if (notesPressed[i] != 128) {
      //     // Serial.println(notesPressed[i]);
      //     localCurrentStepSize[i] = stepSizes[notesPressed[i]];
      //   } 
      //   else {
      //     localCurrentStepSize[i] = 0;
      //   }
        
      //   xSemaphoreTake(stepSizeMutex, portMAX_DELAY);
      //   __atomic_store_n(&currentStepSize[i], localCurrentStepSize[i], __ATOMIC_RELAXED);
      //   xSemaphoreGive(stepSizeMutex);
      // }

      __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
    }    
  }
}

void CAN_RX_ISR (void) {
	uint8_t RX_Message_ISR[8];
	uint32_t ID = 0x123;
	CAN_RX(ID, RX_Message_ISR);
	xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
}

void CAN_TX_ISR (void) {
	xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);
  // Serial.println("TX ISR");
}

void CAN_TX_Task (void * pvParameters) {
	uint8_t msgOut[8];
	while (1) {
		xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
		xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
		CAN_TX(0x123, msgOut);
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

  msgInQ = xQueueCreate(36,8);
  msgOutQ = xQueueCreate(36,8);

  keyArrayMutex = xSemaphoreCreateMutex();
  stepSizeMutex = xSemaphoreCreateMutex();
  RX_MessageMutex = xSemaphoreCreateMutex();

  CAN_TX_Semaphore = xSemaphoreCreateCounting(3,3);

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

  TaskHandle_t decodeTaskHandle = NULL;
  xTaskCreate(
  decodeTask,		/* Function that implements the task */
  "decodeTask",		/* Text name for the task */
  256,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  1,			/* Task priority */
  &decodeTaskHandle );	/* Pointer to store the task handle */
 
  TaskHandle_t TXTask = NULL;
  xTaskCreate(
  CAN_TX_Task,		/* Function that implements the task */
  "CAN_TX_Task",		/* Text name for the task */
  256,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  1,			/* Task priority */
  &TXTask );	/* Pointer to store the task handle */



  if (keyArrayMutex == NULL) {
    Serial.println("Error: Mutex was not created because the memory required to hold the mutex could not be allocated");
  }
  if (RX_MessageMutex == NULL) {
    Serial.println("Error: Mutex was not created because the memory required to hold the mutex could not be allocated");
  }
  
  CAN_Init(true);
  setCANFilter(0x123, 0x7ff);
  CAN_RegisterRX_ISR(CAN_RX_ISR);
  CAN_RegisterTX_ISR(CAN_TX_ISR);
  CAN_Start();

  vTaskStartScheduler();
}

void loop() {
 
}