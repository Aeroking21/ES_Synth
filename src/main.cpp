#include <Arduino.h>
#include <U8g2lib.h>
#include <STM32FreeRTOS.h>
#include <string>


//Constants
const uint32_t interval = 100; //Display update interval
//volatile uint32_t currentStepSize;
volatile uint8_t keyArray[7];
const char * globalKeySymbol;

//Set key step sizes
const uint32_t stepSizes [] = {51149156, 54077543, 57396381, 60715220, 64424509, 68133799, 71647864, 76528508, 81018701, 85899346, 90975216, 96441538};
const std::string keyOrder[] = {"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};
const uint8_t NUM_KEYS = 8;
volatile uint32_t currentStepSize[NUM_KEYS];
volatile uint32_t NotesPressed[NUM_KEYS];

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



//Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

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
  digitalWrite(RA0_PIN, rowIdx & 0x01);
  digitalWrite(RA1_PIN, rowIdx & 0x02);
  digitalWrite(RA2_PIN, rowIdx & 0x04);
  digitalWrite(REN_PIN, HIGH);
}

uint8_t readCols(){
  uint8_t c0 = digitalRead(C0_PIN);
  uint8_t c1 = digitalRead(C1_PIN);
  uint8_t c2 = digitalRead(C2_PIN);
  uint8_t c3 = digitalRead(C3_PIN);

  uint8_t result = 0;
  result |= (c0 << 0);
  result |= (c1 << 1);
  result |= (c2 << 2);
  result |= (c3 << 3);

  return result;

}

void sampleISR() {
  static int32_t phaseAcc[NUM_KEYS] = {0,0,0,0,0,0,0,0};

  int32_t localCurrentStepSize[NUM_KEYS];
  int32_t keysPressed = 0;
  //maybe use a mutex and make a copy

  for (int i=0; i < NUM_KEYS; i++) {
    localCurrentStepSize[i] = currentStepSize[i]; 
    phaseAcc[i] += localCurrentStepSize[i];
    if(localCurrentStepSize[i] != 0){
      keysPressed++;
    } 
  }

  int32_t phaseAcc_final = 0;

  for (int i=0; i<NUM_KEYS; i++) {
   phaseAcc_final += phaseAcc[i] / keysPressed;
  }

  int32_t Vout = (phaseAcc_final >> 24);
  analogWrite(OUTR_PIN, (Vout + 128));

}

void scanKeysTask(void * pvParameters) {

  const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while(1){
    vTaskDelayUntil( &xLastWakeTime, xFrequency );

    uint32_t lastStepSize = 0;
    const uint32_t * localStepSizes = stepSizes;
    uint32_t notesPressed[8] = {128,128,128,128,128,128,128,128};

    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    for(uint8_t row = 0; row < 4; row++){
      setRow(row);
      delayMicroseconds(3);
 
      uint8_t keys = readCols();
      keyArray[row] = keys;
 
    }
    
    int counter = 0;
    int index = 0;
    for(int i=0; i < 3; i++){

      uint8_t key = ~keyArray[i];
      for(int j=0; j < 4; j++){

        if(unsigned((key >> j) & 1) == 1){
          notesPressed[index] = counter;
          index++;
        }
        counter++;
      }

    }

    xSemaphoreGive(keyArrayMutex);

    int32_t localCurrentStepSize = 0;

    xSemaphoreTake(stepSizeMutex, portMAX_DELAY);
    for (int i = 0; i < NUM_KEYS; i++) {

      if (notesPressed[i] != 128) {
        localCurrentStepSize = stepSizes[notesPressed[i]];
        currentStepSize[i] = localCurrentStepSize;
      } 
      else {
        currentStepSize[i] = 0;
      }
      
    }
    xSemaphoreGive(stepSizeMutex);
    
    // currentStepSize = lastStepSize;
    // __atomic_store_n(&currentStepSize, lastStepSize, __ATOMIC_RELAXED);
    // __atomic_store_n(&globalKeySymbol, keysymbol, __ATOMIC_RELAXED);
    

  }
}



void displayUpdateTask(void * pvParameters){

  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while(1){
    vTaskDelayUntil( &xLastWakeTime, xFrequency );

    //Update display
  
    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    u8g2.drawStr(2,10,"Hi");  // write something to the internal memory
    
    u8g2.setCursor(2,20);
    for(uint8_t i=0;i<3;i++){
      u8g2.print(keyArray[i], HEX); 
    }
    
    // Store the result in the global variable
    //u8g2.drawStr(2,30,globalKeySymbol);
    u8g2.setCursor(30,30);
    u8g2.sendBuffer();          // transfer internal memory to the display

    //Toggle LED
    digitalToggle(LED_BUILTIN);
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