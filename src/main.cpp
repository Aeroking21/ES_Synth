#include <Arduino.h>
#include <U8g2lib.h>
#include <STM32FreeRTOS.h>
#include <ES_CAN.h>

// Constants
const uint32_t interval = 100; // Display update interval
constexpr uint32_t stepSizes[] = {0, 96418756, 91007187, 85899346, 81078186, 76527617, 72232452, 68178356, 64351799, 60740010, 57330935, 54113197, 51076057};

constexpr char *notes[] = {"-", "B", "A#", "A", "G#", "G", "F#", "F", "E", "D#", "D", "C#", "C"};

// Global Variables
volatile uint32_t currentStepSize;
volatile uint32_t newStepSize = 0;
volatile uint32_t phaseAcc = 0;
volatile uint8_t keyArray[7];
// volatile int noteInd;
//  volatile uint8_t TX_Message[8] = {0};
volatile uint8_t RX_Message[8] = {0};
volatile uint8_t RX_Message2[8] = {0};
volatile signed int knob3Rotation = 8;
volatile bool EN = 1;
volatile int echo = 0;
volatile int ech = 0;
volatile uint32_t VOUT = 0;
volatile uint32_t V = 0;
volatile int deg = 0;
// Global handle for mutex
SemaphoreHandle_t keyArrayMutex;
SemaphoreHandle_t RX_MessageMutex;
SemaphoreHandle_t RX_Message2Mutex;
SemaphoreHandle_t CAN_TX_Semaphore;
// Queue handler
QueueHandle_t msgInQ;
QueueHandle_t msgOutQ;

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

// Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

// Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value)
{
  digitalWrite(REN_PIN, LOW);
  digitalWrite(RA0_PIN, bitIdx & 0x01);
  digitalWrite(RA1_PIN, bitIdx & 0x02);
  digitalWrite(RA2_PIN, bitIdx & 0x04);
  digitalWrite(OUT_PIN, value);
  digitalWrite(REN_PIN, HIGH);
  delayMicroseconds(2);
  digitalWrite(REN_PIN, LOW);
}

uint8_t readCols()
{
  uint8_t C0 = digitalRead(C0_PIN);
  uint8_t C1 = digitalRead(C1_PIN);
  uint8_t C2 = digitalRead(C2_PIN);
  uint8_t C3 = digitalRead(C3_PIN);

  uint8_t key = (C0 << 3) | (C1 << 2) | (C2 << 1) | C3;
  return key;
}

void setRow(uint8_t rowIdx)
{
  digitalWrite(REN_PIN, LOW);
  digitalWrite(RA0_PIN, rowIdx & 0x01);
  digitalWrite(RA1_PIN, rowIdx & 0x02);
  digitalWrite(RA2_PIN, rowIdx & 0x04);
  digitalWrite(REN_PIN, HIGH);
}

void echoes(void *pvParameters)
{
  const TickType_t xFrequency5 = 50 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime5 = xTaskGetTickCount();
  while (1)
  {
    vTaskDelayUntil(&xLastWakeTime5, xFrequency5);

    uint8_t rx_message1[8];
    uint8_t rx_message2[8];
    xSemaphoreTake(RX_MessageMutex, portMAX_DELAY);
    std::copy(std::begin(RX_Message), std::end(RX_Message), std::begin(rx_message1));
    xSemaphoreGive(RX_MessageMutex);
    xSemaphoreTake(RX_Message2Mutex, portMAX_DELAY);
    std::copy(std::begin(RX_Message2), std::end(RX_Message2), std::begin(rx_message2));

    uint32_t _ech = __atomic_load_n(&ech, __ATOMIC_RELAXED);

    if ((char)rx_message1[0] == 'R')
    {
      if (_ech < 1000)
      {
        _ech += 4;
      }
    }
    else
    {
      if ((char)rx_message2[0] == 'R')
      {
        _ech = 1;
      }
      else
      {
        if (_ech < 1000)
        {
          _ech += 1;
        }
      }
    }
    __atomic_store_n(&ech, _ech, __ATOMIC_RELAXED);

    std::copy(std::begin(rx_message1), std::end(rx_message1), std::begin(RX_Message2));
    xSemaphoreGive(RX_Message2Mutex);

    //   uint32_t _newStepSize = __atomic_load_n(&currentStepSize, __ATOMIC_RELAXED);
    //   _newStepSize = _newStepSize*sin(DEG_TO_RAD*deg);
    //   __atomic_store_n(&newStepSize, _newStepSize, __ATOMIC_RELAXED);

    //   if(deg>=360){
    //     deg = 0;
    //   }
    //   else{
    //     deg++;
    //   }
  }
}

// void sampleSine(void *pvParameters)
// {
//   const TickType_t xFrequency6 = 5 / portTICK_PERIOD_MS;
//   TickType_t xLastWakeTime6 = xTaskGetTickCount();
//   while (1)
//   {
//     vTaskDelayUntil(&xLastWakeTime6, xFrequency6);
//   uint32_t _newStepSize = __atomic_load_n(&currentStepSize, __ATOMIC_RELAXED);
//   _newStepSize = _newStepSize * sin(DEG_TO_RAD * deg);
//   __atomic_store_n(&newStepSize, _newStepSize, __ATOMIC_RELAXED);

//   if (deg >= 360)
//   {
//     deg = 0;
//   }
//   else
//   {
//     deg++;
//   }
//   }
// }

void sampleISR()
{
  static uint32_t phaseAcc = 0;
  uint32_t _currentStepSize = __atomic_load_n(&currentStepSize, __ATOMIC_RELAXED);
  uint32_t _ech = __atomic_load_n(&ech, __ATOMIC_RELAXED);

  if (EN)
  {
    if (phaseAcc < (phaseAcc + _currentStepSize))
    {
      phaseAcc += _currentStepSize;
    }
    else
    {
      phaseAcc -= _currentStepSize;
      EN = 0;
    }
  }
  else
  {
    if (phaseAcc > (phaseAcc - _currentStepSize))
    {
      phaseAcc -= _currentStepSize;
    }
    else
    {
      phaseAcc += _currentStepSize;
      EN = 1;
    }
  }

    int32_t Vout = (phaseAcc >> 23) - 128;

    Vout = (Vout + 128)/*/_ech*10*/;

    Vout = Vout;

    analogWrite(OUTR_PIN, Vout);
  }

  void scanKeysTask(void *pvParameters)
  {
    const TickType_t xFrequency1 = 50 / portTICK_PERIOD_MS;
    TickType_t xLastWakeTime1 = xTaskGetTickCount();
    while (1)
    {
      vTaskDelayUntil(&xLastWakeTime1, xFrequency1);

      xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
      for (uint8_t i = 0; i < sizeof(keyArray); i++)
      {
        setRow(i);
        delayMicroseconds(3);
        keyArray[i] = readCols();
      }

      uint32_t noteIndex = (keyArray[0] << 8) | (keyArray[1] << 4) | keyArray[2];

      xSemaphoreGive(keyArrayMutex);
      noteIndex = ~noteIndex & 0xfff;
      //__atomic_store_n(&noteInd, 0, __ATOMIC_RELAXED);
      uint8_t TX_Message[8] = {0};
      TX_Message[0] = 'R';
      TX_Message[1] = 0;
      TX_Message[2] = 0;
      for (int i = sizeof(notes); i >= 1; i--)
      {
        if ((noteIndex >> (i - 1)) == 1)
        {
          // currentStepSize = stepSizes[i];
          //__atomic_store_n(&noteInd, i, __ATOMIC_RELAXED);
          // u8g2.print(notes[i]);
          TX_Message[0] = 'P';
          TX_Message[1] = 4;
          TX_Message[2] = i;
          break;
        }
        // else{
        //   currentStepSize=0;
        // }
      }
      // CAN_TX(0x123, TX_Message);
      xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
      //__atomic_store_n(&currentStepSize, stepSizes[noteInd], __ATOMIC_RELAXED);
    }
  }

  void displayUpdateTask(void *pvParameters)
  {
    const TickType_t xFrequency2 = 100 / portTICK_PERIOD_MS;
    TickType_t xLastWakeTime2 = xTaskGetTickCount();
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    while (1)
    {
      vTaskDelayUntil(&xLastWakeTime2, xFrequency2);
      // Update display
      u8g2.clearBuffer(); // clear the internal memory

      u8g2.drawStr(3, 10, "Helllo World!"); // write something to the internal memory
      u8g2.setCursor(3, 20);
      xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
      u8g2.print(keyArray[0], HEX);
      u8g2.print(keyArray[1], HEX);
      u8g2.print(keyArray[2], HEX);
      xSemaphoreGive(keyArrayMutex);

      uint8_t rx_message[8];
      xSemaphoreTake(RX_MessageMutex, portMAX_DELAY);
      std::copy(std::begin(RX_Message), std::end(RX_Message), std::begin(rx_message));
      xSemaphoreGive(RX_MessageMutex);

      u8g2.setCursor(3, 30);
      int index = rx_message[2]; //__atomic_load_n(&noteInd, __ATOMIC_RELAXED);
      u8g2.print(notes[index]);
      u8g2.setCursor(30, 30);
      u8g2.print(ech);

      // uint32_t ID;
      //  uint8_t RX_Message[8] = {0};

      // while (CAN_CheckRXLevel())
      // {
      //   CAN_RX(ID, RX_Message);
      // }
      u8g2.setCursor(66, 30);
      // xSemaphoreTake(RX_MessageMutex, portMAX_DELAY);
      u8g2.print((char)rx_message[0]);
      u8g2.print(rx_message[1]);
      u8g2.print(rx_message[2]);

      u8g2.sendBuffer(); // transfer internal memory to the display

      // Toggle LED
      digitalToggle(LED_BUILTIN);
    }
  }

  void CAN_RX_ISR(void)
  {
    uint8_t RX_Message_ISR[8];
    uint32_t ID;
    CAN_RX(ID, RX_Message_ISR);
    xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
  }

  void decodeTask(void *pvParameters)
  {
    while (1)
    {
      xQueueReceive(msgInQ, (void *)&RX_Message, portMAX_DELAY);

      uint8_t rx_message[8];
      xSemaphoreTake(RX_MessageMutex, portMAX_DELAY);
      std::copy(std::begin(RX_Message), std::end(RX_Message), std::begin(rx_message));
      xSemaphoreGive(RX_MessageMutex);
      if ((char)rx_message[0] == 'P')
      {
        uint32_t step = stepSizes[rx_message[2]];
        step = step << (rx_message[1] - 4);
        __atomic_store_n(&currentStepSize, step, __ATOMIC_RELAXED);
      }
      else
      {
        __atomic_store_n(&currentStepSize, stepSizes[0], __ATOMIC_RELAXED);
      }
    }
  }

  void CAN_TX_Task(void *pvParameters)
  {
    uint8_t msgOut[8];
    while (1)
    {
      xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
      xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
      CAN_TX(0x123, msgOut);
    }
  }

  void CAN_TX_ISR(void)
  {
    xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);
  }

  // #ifndef DISABLE_THREADS
  // 	xTaskCreate(scanKeysTask,"scanKeys",64,NULL,2,&scanKeysHandle);
  //   xTaskCreate(displayUpdateTask,"displayUpdate",256,NULL,1,&displayUpdateHandle);
  //   xTaskCreate(decodeTask,"decode",72,NULL,4,&decodeHandle);
  //   xTaskCreate(CAN_TX_Task,"CAN_TX",72,NULL,3,&CAN_TXHandle);
  // #endif

  // #ifdef TEST_SCANKEYS
  // 	uint32_t startTime = micros();
  // 	for (int iter = 0; iter < 32; iter++) {
  // 		scanKeysTask()
  // 	}
  // 	Serial.println(micros()-startTime);
  // 	while(1);
  // #endif

  void setup()
  {
    // put your setup code here, to run once:

    // Set pin directions
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
    setOutMuxBit(DRST_BIT, LOW); // Assert display logic reset
    delayMicroseconds(2);
    setOutMuxBit(DRST_BIT, HIGH); // Release display logic reset
    u8g2.begin();
    setOutMuxBit(DEN_BIT, HIGH); // Enable display power supply
    TIM_TypeDef *Instance = TIM1;
    HardwareTimer *sampleTimer = new HardwareTimer(Instance);
    sampleTimer->setOverflow(22000, HERTZ_FORMAT);
    sampleTimer->attachInterrupt(sampleISR);
    sampleTimer->resume();
    // sampleTimer->setOverflow(11000, HERTZ_FORMAT);
    // sampleTimer->attachInterrupt(sampleSine);
    // sampleTimer->resume();

    msgInQ = xQueueCreate(36, 8);
    msgOutQ = xQueueCreate(36, 8);

    CAN_Init(true);
    setCANFilter(0x123, 0x7ff);
    CAN_RegisterRX_ISR(CAN_RX_ISR);
    CAN_RegisterTX_ISR(CAN_TX_ISR);
    CAN_Start();

    TaskHandle_t scanKeysHandle = NULL;
    xTaskCreate(
        scanKeysTask,     /* Function that implements the task */
        "scanKeys",       /* Text name for the task */
        64,               /* Stack size in words, not bytes */
        NULL,             /* Parameter passed into the task */
        2,                /* Task priority */
        &scanKeysHandle); /* Pointer to store the task handle */

    TaskHandle_t displayUpdateHandle = NULL;
    xTaskCreate(
        displayUpdateTask,     /* Function that implements the task */
        "displayUpdate",       /* Text name for the task */
        256,                   /* Stack size in words, not bytes */
        NULL,                  /* Parameter passed into the task */
        1,                     /* Task priority */
        &displayUpdateHandle); /* Pointer to store the task handle */

    TaskHandle_t decodeHandle = NULL;
    xTaskCreate(
        decodeTask,     /* Function that implements the task */
        "decode",       /* Text name for the task */
        72,             /* Stack size in words, not bytes */
        NULL,           /* Parameter passed into the task */
        4,              /* Task priority */
        &decodeHandle); /* Pointer to store the task handle */

    TaskHandle_t CAN_TXHandle = NULL;
    xTaskCreate(
        CAN_TX_Task,    /* Function that implements the task */
        "CAN_TX",       /* Text name for the task */
        72,             /* Stack size in words, not bytes */
        NULL,           /* Parameter passed into the task */
        3,              /* Task priority */
        &CAN_TXHandle); /* Pointer to store the task handle */

    TaskHandle_t EchoHandle = NULL;
    xTaskCreate(
        echoes,           /* Function that implements the task */
        "echoes",         /* Text name for the task */
        64,               /* Stack size in words, not bytes */
        NULL,             /* Parameter passed into the task */
        5,                /* Task priority */
        &scanKeysHandle); /* Pointer to store the task handle */

    //   TaskHandle_t SineHandle = NULL;
    // xTaskCreate(
    //     sampleSine,           /* Function that implements the task */
    //     "sine",         /* Text name for the task */
    //     72,               /* Stack size in words, not bytes */
    //     NULL,             /* Parameter passed into the task */
    //     6,                /* Task priority */
    //     &SineHandle); /* Pointer to store the task handle */

    // Initialise UART
    Serial.begin(9600);
    Serial.println("Hello World");

    keyArrayMutex = xSemaphoreCreateMutex();
    RX_MessageMutex = xSemaphoreCreateMutex();
    RX_Message2Mutex = xSemaphoreCreateMutex();
    CAN_TX_Semaphore = xSemaphoreCreateCounting(3, 3);

    vTaskStartScheduler();
  }

  void loop()
  {

    // scanKeysTask(NULL);
  }