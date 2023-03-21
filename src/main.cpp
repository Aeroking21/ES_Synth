#include <Arduino.h>
#include <U8g2lib.h>
#include <math.h>
#include <STM32FreeRTOS.h>
#include <ES_CAN.h>

// Self-written header files
#include <knob.h>
#include <wavestype.h>
#include <variables.h>

void sampleISR() {

  #ifdef POLYPHONY
    static int32_t phaseAcc[MAX_KEYS_PLAYED_TGT] = {};
    int32_t localCurrentStepSize[MAX_KEYS_PLAYED_TGT];
    int32_t keysPressed = 0;
    //maybe use a mutex and make a copy

    for (int i=0; i < MAX_KEYS_PLAYED_TGT; i++) 
    {
      localCurrentStepSize[i] = currentStepSize[i]; 
      phaseAcc[i] += localCurrentStepSize[i];

      if(localCurrentStepSize[i] != 0){
        keysPressed++;
      } 
    }

    int32_t phaseAcc_final = 0;
    for (int i=0; i<MAX_KEYS_PLAYED_TGT; i++) 
    {
      phaseAcc_final += phaseAcc[i] / keysPressed;
    }

    int32_t Vout = (phaseAcc_final >> 24);
    Vout = Vout >> (8 - VolumeRotation);
    analogWrite(OUTR_PIN, (Vout + 128));
  #else
    static uint32_t phaseAcc = 0;
    phaseAcc += currentStepSize;  

    int32_t Vout = (phaseAcc >> 24) - 128;
    Vout = Vout >> (8 - VolumeRotation);
    analogWrite(OUTR_PIN, Vout + 128);
  #endif
}

//Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value) 
{
  digitalWrite(REN_PIN,LOW);
  digitalWrite(RA0_PIN, bitIdx & 0x01);
  digitalWrite(RA1_PIN, bitIdx & 0x02);
  digitalWrite(RA2_PIN, bitIdx & 0x04);
  digitalWrite(OUT_PIN,value);
  digitalWrite(REN_PIN,HIGH);
  delayMicroseconds(2);
  digitalWrite(REN_PIN,LOW);
}

uint8_t readCols()
{
  uint8_t c0 = digitalRead(C0_PIN);
  uint8_t c1 = digitalRead(C1_PIN);
  uint8_t c2 = digitalRead(C2_PIN);
  uint8_t c3 = digitalRead(C3_PIN);

  uint8_t res = (c3 << 3) | (c2 << 2) | (c1 << 1) | c0 ;
  return res;
}

void setRow(uint8_t rowIdx){

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

void keyScanningRoutine() 
{  
  for (int i = 0; i < 8; i++) {
    setRow(i);
    delayMicroseconds(3);
    keyArray[i] = readCols(); 
  }
}

void scanKeysTask(void * pvParameters) 
{
  // Set initiation interval in millis
  const TickType_t xFrequency = 10/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  uint32_t ID = 0x123;
  uint8_t TX_Message[8] = {0};

  static uint32_t localCurrentStepSize = 0;
  static signed int localVolumeRotation = 8;

  Knob Volume(3, 8, 0, 8);

  #ifdef POLYPHONY
  uint32_t notesPressed[8] = {128, 128, 128, 128, 128, 128, 128, 128};
  #endif
  
  while (1) {
    // Serial.println("scan key task loop!");
    vTaskDelayUntil( &xLastWakeTime, xFrequency );

    keyScanningRoutine();

    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    uint32_t key_pressed = ((~keyArray[2] & 0xf) << 8) | ((~keyArray[1] & 0xf) << 4) | (~keyArray[0] & 0xf);
    Volume.updateRotation(keyArray);
    localVolumeRotation = Volume.getRotation();
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

    #ifdef POLYPHONY
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
    for (int i = 0; i < MAX_KEYS_PLAYED_TGT; i++) {

      if (notesPressed[i] != 128) 
      {
        TX_Message[0] = 'P';
        TX_Message[1] = 4;    
        currentStepSize[i] = stepSizes[notesPressed[i]];
      } 
      else {
        TX_Message[0] = 'R';
        TX_Message[1] = 4; 
        currentStepSize[i] = 0;
      }
    }
    xSemaphoreGive(stepSizeMutex);
    #else
    if (key_pressed != 0x0)
    {  
      TX_Message[0] = 'P';
      TX_Message[1] = 4;
      
      if (exponent == 0) {
        localCurrentStepSize = 0;
      }
      else {
        localCurrentStepSize = stepSizes[exponent-1];
        TX_Message[2] = exponent-1;
      }
    }
    else {
      localCurrentStepSize = 0;
      TX_Message[0] = 'R';
    }
    
    __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
    #endif

    __atomic_store_n(&VolumeRotation, localVolumeRotation, __ATOMIC_RELAXED);

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
    u8g2.setCursor(2,20);
    u8g2.print("Volume : "); 
    u8g2.print(VolumeRotation, DEC); 

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

  #ifdef POLYPHONY
  int32_t localCurrentStepSize[MAX_KEYS_PLAYED_TGT] = {};
  uint8_t notesPressed[MAX_KEYS_PLAYED_TGT] = {128,128,128,128,128,128,128,128};
  #else
  int32_t localCurrentStepSize = 0;
  #endif

  uint8_t messageIn[8] = {0};
  
  while (1) {

    if (xQueueReceive(msgInQ, messageIn, portMAX_DELAY) == pdTRUE){
      xSemaphoreTake(RX_MessageMutex, portMAX_DELAY);
      memcpy((void*)RX_Message, &messageIn, 8);
      xSemaphoreGive(RX_MessageMutex);

      #ifndef POLYPHONY
      if (messageIn[0] == 'R'){
        localCurrentStepSize = 0;
      }
      else {
        // Serial.println("key pressed!");
        localCurrentStepSize = stepSizes[RX_Message[2]];
      }

      __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
      #else
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
      xSemaphoreGive(keyArrayMutex);
      }

      xSemaphoreTake(stepSizeMutex, portMAX_DELAY);
      if (messageIn[0] == 'R')
      {
        // for (int i = 0; i < MAX_KEYS_PLAYED_TGT; i++) 
        // {
        //   if (notesPressed[i] == 128) currentStepSize[i] = 0;
        // }
      }
      else if (messageIn[0] == 'P')
      {
        // int32_t localCurrentStepSize = 0;
    
        // for (int i = 0; i < MAX_KEYS_PLAYED_TGT; i++) 
        // {
        //   if (notesPressed[i] != 128) 
        //   {
        //     localCurrentStepSize = stepSizes[notesPressed[i]];
        //     currentStepSize[i] = localCurrentStepSize;
        //   } 
        //   else {
        //     currentStepSize[i] = 0;
        //   }
        // }
      }
      xSemaphoreGive(stepSizeMutex);
      #endif
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
  setOutMuxBit(DRST_BIT, LOW);    //Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);   //Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);    //Enable display power supply

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
  scanKeysTask,		          /* Function that implements the task */
  "scanKeys",		            /* Text name for the task */
  64,      		              /* Stack size in words, not bytes */
  NULL,			                /* Parameter passed into the task */
  2,			                  /* Task priority */
  &localCurrentStepSize );	/* Pointer to store the task handle */
 
  TaskHandle_t displayTask = NULL;
  xTaskCreate(
  displayUpdateTask,		
  "displayTasks",		
  128,      		
  NULL,			
  1,			
  &displayTask );	

  TaskHandle_t decodeTaskHandle = NULL;
  xTaskCreate(
  decodeTask,		
  "decodeTask",	
  256,      		 
  NULL,			
  1,			
  &decodeTaskHandle );	
 
  TaskHandle_t TXTask = NULL;
  xTaskCreate(
  CAN_TX_Task,		
  "CAN_TX_Task",		
  256,      		
  NULL,		
  1,			
  &TXTask );	

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

  Serial.println("Done Setting Up");

  vTaskStartScheduler();
}

void loop() {
 
}