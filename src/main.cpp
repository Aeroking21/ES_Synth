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
volatile uint8_t keyArray[7];
volatile int noteInd;
// volatile uint8_t TX_Message[8] = {0};
volatile uint8_t RX_Message[8] = {0};

// Global handle for mutex
SemaphoreHandle_t keyArrayMutex;
// Queue handler
QueueHandle_t msgInQ;

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

void sampleISR()
{
  static uint32_t phaseAcc = 0;
  phaseAcc += currentStepSize;

  int32_t Vout = (phaseAcc >> 24) - 128;

  analogWrite(OUTR_PIN, Vout + 128);
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
    __atomic_store_n(&noteInd, 0, __ATOMIC_RELAXED);
    uint8_t TX_Message[8] = {0};
    TX_Message[0] = 'R';
    TX_Message[1] = 0;
    TX_Message[2] = 0;
    for (int i = sizeof(notes); i >= 1; i--)
    {
      if ((noteIndex >> (i - 1)) == 1)
      {
        // currentStepSize = stepSizes[i];
        __atomic_store_n(&noteInd, i, __ATOMIC_RELAXED);
        // u8g2.print(notes[i]);
        TX_Message[0] = 'P';
        TX_Message[1] = 4;
        TX_Message[2] = i - 1;
        break;
      }
      // else{
      //   currentStepSize=0;
      // }
    }
    CAN_TX(0x123, TX_Message);
    __atomic_store_n(&currentStepSize, stepSizes[noteInd], __ATOMIC_RELAXED);
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

    u8g2.setCursor(3, 30);
    int index = __atomic_load_n(&noteInd, __ATOMIC_RELAXED);
    u8g2.print(notes[index]);

    uint32_t ID;
    //uint8_t RX_Message[8] = {0};

    // while (CAN_CheckRXLevel())
    // {
    //   CAN_RX(ID, RX_Message);
    // }
    u8g2.setCursor(66, 30);
    u8g2.print((char)RX_Message[0]);
    u8g2.print(RX_Message[1]);
    u8g2.print(RX_Message[2]);

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
    xQueueReceive(msgInQ, (void*) &RX_Message, portMAX_DELAY);
  }
}

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

  msgInQ = xQueueCreate(36, 8);

  CAN_Init(true);
  setCANFilter(0x123, 0x7ff);
  CAN_RegisterRX_ISR(CAN_RX_ISR);
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
      72,                   /* Stack size in words, not bytes */
      NULL,                  /* Parameter passed into the task */
      3,                     /* Task priority */
      &decodeHandle); /* Pointer to store the task handle */

  // Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");

  keyArrayMutex = xSemaphoreCreateMutex();

  vTaskStartScheduler();
}

void loop()
{

  // scanKeysTask(NULL);
}