#include <Arduino.h>
#include <U8g2lib.h>
#include <math.h>
#include <STM32FreeRTOS.h>
#include <ES_CAN.h>

// Self-written header files
#include <knob.h>
#include <wavestype.h>
#include <variables.h>

int t = 0;
int i = 0;
bool keyReleased = false;
bool off = false;
double currentAmplitude;
uint32_t ech;
uint32_t sech;
bool RE = 0;

// envelope(10.0, 10.0, 255.0, 15.0, 100.0);
double envelope(double tA, double tD, double maxAmplitude, double tR, double ks)
{
  double kA = maxAmplitude / tA;
  double kD = (maxAmplitude - ks) / (tD);
  double kR = currentAmplitude / tR;
  double newtD = tD + tA;
  if (off == true)
  {
    if (t <= tR)
    {
      return currentAmplitude - kR * t;
    }
    else
    {
      keyReleased = false;
      currentStepSize = 0;
      currentAmplitude = 0;
      return 0.0;
    }
  }
  else if (t <= tA)
  {
    currentAmplitude = kA * t;
    keyReleased = true;
    return kA * t;
  }
  else if (t <= newtD)
  {
    currentAmplitude = maxAmplitude - kD * (t - tA);
    // Serial.println(8-kD*(t-tA));
    return maxAmplitude - kD * (t - tA);
    // over 8 seconds, output decreases from 8 to 6
    // 8-ks = 2
  }
  else
  {
    keyReleased = true;
    currentAmplitude = ks;
    return ks;
  }
}

uint32_t shiftIdxForOctave(int octave, int keyboardPositionIdx, uint32_t phase)
{
  int shift = octave + keyboardPositionIdx - 4;

  if (octave + keyboardPositionIdx >= 4)
  {
    phase = phase << std::abs(shift);
  }
  else if (4 - octave - keyboardPositionIdx < 4)
  {
    phase = phase >> std::abs(shift);
  }
  return phase;
}

void echoes(void *pvParameters)
{
  const TickType_t xFrequency5 = 75 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime5 = xTaskGetTickCount();
  while (1)
  {
    vTaskDelayUntil(&xLastWakeTime5, xFrequency5);

    uint8_t localKeyArray[7];
    uint8_t _messageIn[8] = {};
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    memcpy(&localKeyArray, (void *)&keyArray, 7);
    xSemaphoreGive(keyArrayMutex);
    //     xSemaphoreTake(RX_MessageMutex, portMAX_DELAY);
    // memcpy((void *)RX_Message, &_messageIn, 8);
    // xSemaphoreGive(RX_MessageMutex);

    uint32_t _keyPressed = ((~localKeyArray[2] & 0xf) << 8) | ((~localKeyArray[1] & 0xf) << 4) | (~localKeyArray[0] & 0xf);
    uint32_t _prevKeyPressed;
    static bool _RE;

    uint32_t _ech = __atomic_load_n(&ech, __ATOMIC_RELAXED);
    uint32_t _sech = __atomic_load_n(&sech, __ATOMIC_RELAXED);
    static double n = 0;

    if (_keyPressed == 0 && RE == 0)
    {
      if (_ech < 100)
      {
        _ech += 4;
      }
      if (n < 10)
      {
        n += 0.5;
      }
      _RE = 0;
    }
    else
    {
      // Serial.println("pressed");
      if (_keyPressed != _prevKeyPressed || (RE==1 && _RE==0))
      {
        _ech = 1;
        n = 0;
        _RE = 1;
      }
      else
      {
        if (_ech < 100)
        {
          _ech += 1;
        }
        if (n < 10)
        {
          n += 0.2;
        }
      }
    }
    _sech = (uint32_t)n;
    _prevKeyPressed = _keyPressed;

    __atomic_store_n(&ech, _ech, __ATOMIC_RELAXED);
    __atomic_store_n(&sech, _sech, __ATOMIC_RELAXED);
  }
}

void sampleISR()
{

  if (keyboardMode == RECEIVER)
  {
    i += 1;
    if (WavetypeRotation == 0) // Sawtooth Wave
    {
#ifdef POLYPHONY
      static int32_t phaseAcc[MAX_KEYS_PLAYED_TGT] = {};
      int32_t activeNotes = 0;
      // maybe use a mutex and make a copy

      for (int i = 0; i < MAX_KEYS_PLAYED_TGT; i++)
      {
        phaseAcc[i] += currentStepSize[i];

        if (currentStepSize[i] != 0)
        {
          activeNotes++;
        }
      }

      int32_t phaseAcc_final = 0;
      for (int i = 0; i < MAX_KEYS_PLAYED_TGT; i++)
      {
        phaseAcc_final += phaseAcc[i] / activeNotes;
      }

      int32_t Vout = (phaseAcc_final >> 24);
      Vout = Vout >> (8 - VolumeRotation);
      analogWrite(OUTR_PIN, (Vout + 128));
#else

      static uint32_t phaseAcc = 0;
      phaseAcc += shiftIdxForOctave(octave, keyboardPositionIdx, currentStepSize);

      int32_t Vout;
      if (EchoSwitch)
      {
        uint32_t _ech = __atomic_load_n(&ech, __ATOMIC_RELAXED);
        Vout = (phaseAcc >> 24) / _ech - 128;
      }
      else
      {
        Vout = (phaseAcc >> 24) - 128;
      }
      Vout = Vout >> (8 - VolumeRotation);

      if (EnvelopeSwitch)
      {
        Vout = Vout * envelope(10.0, 10.0, 255.0, 15.0, 100.0); // make output 0
        Vout = Vout / 256;

        if (currentStepSize != 0)
        {
          if (i > 2500)
          {
            t += 1;
            i = 0;
          }
        }
      }

      analogWrite(OUTR_PIN, Vout + 128);
#endif
    }
    else
    { // Sine Wave

      // Idx += sineIdxAcc;
      Idx += shiftIdxForOctave(octave, keyboardPositionIdx, sineIdxAcc);

      if (Idx > TABLE_SIZE)
        Idx = 0;
      sineAcc = sineLookUpTable[Idx];

      int32_t Vout;
      if (EchoSwitch)
      {
        uint32_t _sech = __atomic_load_n(&sech, __ATOMIC_RELAXED);
        int echo = (int)_sech;
        Vout = (sineAcc >> (24 + echo)) - 128;
      }
      else
      {
        Vout = (sineAcc >> 24) - 128;
      }
      Vout = Vout >> (8 - VolumeRotation);

      analogWrite(OUTR_PIN, Vout + 128);
    }
  }
}

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
  uint8_t c0 = digitalRead(C0_PIN);
  uint8_t c1 = digitalRead(C1_PIN);
  uint8_t c2 = digitalRead(C2_PIN);
  uint8_t c3 = digitalRead(C3_PIN);

  uint8_t res = (c3 << 3) | (c2 << 2) | (c1 << 1) | c0;
  return res;
}

void setRow(uint8_t rowIdx)
{

  digitalWrite(REN_PIN, LOW);
  digitalWrite(RA0_PIN, LOW);
  digitalWrite(RA1_PIN, LOW);
  digitalWrite(RA2_PIN, LOW);

  switch (rowIdx)
  {
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
  default:
    break;
  }

  digitalWrite(REN_PIN, HIGH);
}

void keyScanningRoutine()
{
  for (int i = 0; i < 7; i++)
  {
    setRow(i);
    delayMicroseconds(3);
    keyArray[i] = readCols();
  }
}

void scanKeysTask(void *pvParameters)
{
  // Set initiation interval in millis
  const TickType_t xFrequency = 20 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  uint32_t ID = 0x123;
  uint8_t TX_Message[8];
  uint8_t localKeyArray[7];

  static uint32_t localKeyboardPositionIdx;
  static uint32_t localCurrentStepSize = 0;
  static signed int localVolumeRotation = 4;
  static signed int localWavetypeRotation = 0;
  static signed int localOctaveRotation = 4;
  static bool localModeSwitch = 0;
  static bool localEchoSwitch = 0;
  static bool localEnvelopeSwitch = 0;

  while (1)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    keyScanningRoutine();

    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    memcpy(&localKeyArray, (void *)&keyArray, 7);
    xSemaphoreGive(keyArrayMutex);

    uint32_t keyPressed = ((~localKeyArray[2] & 0xf) << 8) | ((~localKeyArray[1] & 0xf) << 4) | (~localKeyArray[0] & 0xf);

    Volume.updateRotation(localKeyArray);
    WaveType.updateRotation(localKeyArray);
    Mode.updateSwitch(localKeyArray);

    localVolumeRotation = Volume.getRotation();
    localWavetypeRotation = WaveType.getRotation();
    localModeSwitch = Mode.getSwitch();

    if (localModeSwitch)
    {
      Envelope.updateSwitch(localKeyArray);
      Echo.updateSwitch(localKeyArray);
      localEnvelopeSwitch = Envelope.getSwitch();
      localEchoSwitch = Echo.getSwitch();
    }

    if (keyboardMode == RECEIVER)
    {
      Octave.updateRotation(localKeyArray);
      localOctaveRotation = Octave.getRotation();
    }

    int detectKeyOneHot = keyPressed & 0xfff;
    int exponent = 0;

    while (detectKeyOneHot >= 1)
    {
      detectKeyOneHot >>= 1;
      exponent++;
    }
#ifdef POLYPHONY

    uint32_t notesPressed[8] = {128, 128, 128, 128, 128, 128, 128, 128};
    uint32_t notesPressedOneHot = 0;
    int counter = 0;
    int index = 0;

    for (int i = 0; i < 3; i++)
    {

      uint8_t key = ~localKeyArray[i];
      for (int j = 0; j < 4; j++)
      {

        if (unsigned((key >> j) & 1) == 1)
        {
          notesPressed[index] = counter;
          index++;
        }
        counter++;
      }
    }

    if (keyboardMode == RECEIVER)
    {
      for (int i = 0; i < MAX_KEYS_PLAYED_TGT; i++)
      {

        if (notesPressed[i] != 128)
        {
          currentStepSize[i] = stepSizes[notesPressed[i]];
          notesPressedOneHot = notesPressedOneHot | (0x1 << notesPressed[i]);
        }
        else
        {
          currentStepSize[i] = 0;
          notesPressedOneHot = 0;
        }
      }
    }
    else if (keyboardMode == SENDER && !singleKeyboard)
    {
      if (notesPressedOneHot != prevNotesPressedOneHot)
      {
        for (int i = 0; i < MAX_KEYS_PLAYED_TGT; i++)
        {
          if (notesPressed[i] != 128)
          {
            TX_Message[0] = 'P';
            notesPressedOneHot = notesPressedOneHot | (0x1 << notesPressed[i]);
            TX_Message[2] = notesPressedOneHot;
            Serial.println("transmitted");
            Serial.println(notesPressedOneHot);
          }
          else
          {
            TX_Message[0] = 'R';
          }
        }
      }
      notesPressedOneHot = prevNotesPressedOneHot;

      xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
    }
#else
    if (keyboardMode == RECEIVER)
    {
      if (keyPressed != prevKeyPressed)
      {
        if (keyPressed == 0) {
            sineIdxAcc = 0;
          if (!off) t = 0;
          off = true;
          localCurrentStepSize = 0;
          t = 0;
          i = 0;
        }
        else
        {
          off = false;
          octave = localOctaveRotation;
          currentStepSize = stepSizes[exponent - 1];
          sineIdxAcc = sineLookUpAcc[exponent - 1];
        }
      }
      keyboardPositionIdx = 0;
    }
    else if (keyboardMode == SENDER && !singleKeyboard)
    {
      if (keyPressed != prevKeyPressed)
      {
        if (keyPressed != 0)
        {
          TX_Message[2] = exponent - 1;
          TX_Message[0] = 'P';
        }
        else
        {
          TX_Message[0] = 'R';
        }
      }
      xSemaphoreTake(keyboardPositionIdxMutex, portMAX_DELAY);
      TX_Message[1] = localOctaveRotation + keyboardPositionIdx;
      TX_Message[3] = keyboardPositionIdx;
      xSemaphoreGive(keyboardPositionIdxMutex);

      xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
    }
    prevKeyPressed = keyPressed;

#endif

static bool _localEchoSwitch;

if(localEchoSwitch!=_localEchoSwitch){
  currentStepSize = 0;
  Idx = 0;
  sineAcc = 0;
}
_localEchoSwitch = localEchoSwitch;

    __atomic_store_n(&EchoSwitch, localEchoSwitch, __ATOMIC_RELAXED);
    __atomic_store_n(&EnvelopeSwitch, localEnvelopeSwitch, __ATOMIC_RELAXED);
    __atomic_store_n(&ModeSwitch, localModeSwitch, __ATOMIC_RELAXED);
    __atomic_store_n(&OctaveRotation, localOctaveRotation, __ATOMIC_RELAXED);
    __atomic_store_n(&VolumeRotation, localVolumeRotation, __ATOMIC_RELAXED);
    __atomic_store_n(&WavetypeRotation, localWavetypeRotation, __ATOMIC_RELAXED);
    __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);

    // UBaseType_t ux = uxTaskGetStackHighWaterMark( NULL );
    // Serial.print("stack: ");
    // Serial.println(ux);
  }
}

void displayUpdateTask(void *pvParameters)
{
  uint32_t ID = 0x123;
  const TickType_t xFrequency2 = 100 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime2 = xTaskGetTickCount();
  uint8_t localKeyArray[7];

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);

  while (1)
  {

    vTaskDelayUntil(&xLastWakeTime2, xFrequency2);

    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    memcpy(&localKeyArray, (void *)&keyArray, 7);
    xSemaphoreGive(keyArrayMutex);

    uint32_t keyPressed = ((~localKeyArray[2] & 0xf) << 8) | ((~localKeyArray[1] & 0xf) << 4) | (~localKeyArray[0] & 0xf);

    u8g2.clearBuffer();

    int detectKeyOneHot = keyPressed & 0xfff;
    int exponent = 0;

    while (detectKeyOneHot >= 1)
    {
      detectKeyOneHot >>= 1;
      exponent++;
    }

    uint8_t localRX_Message[8];
    xSemaphoreTake(RX_MessageMutex, portMAX_DELAY);
    memcpy(&localRX_Message, (void*) &RX_Message, 8);
    xSemaphoreGive(RX_MessageMutex);

    if (keyboardMode == SENDER)
    {
      u8g2.setFont(u8g2_font_emoticons21_tr);
      u8g2.drawGlyph(55, 28, 50);
      u8g2.print("Sender");
    }
    else if (singleKeyboard)
    {
      u8g2.setFont(u8g2_font_ncenB08_tr);
      u8g2.setCursor(90, 10);
      u8g2.print("Single");
    }
    if (keyboardMode == RECEIVER)
    {
      if (exponent != 0)
      {
        u8g2.setFont(u8g2_font_ncenB08_tr);
        u8g2.setCursor(2,10);
        u8g2.print(keyOrder[exponent-1]); 
      }
      else if (localRX_Message[0] == 'P') {
        u8g2.setCursor(2,10);
        u8g2.print(keyOrder[localRX_Message[2]]);
      }
      if (!ModeSwitch){
        u8g2.setFont(u8g2_font_open_iconic_play_1x_t);
        u8g2.drawGlyph(2, 30, 79);
        u8g2.setFont(u8g2_font_ncenB08_tr);
        u8g2.setCursor(14, 30);
        u8g2.print(VolumeRotation);
        u8g2.setFont(u8g2_font_5x7_tr);
        u8g2.setCursor(5, 20);
        u8g2.print("Vol.");

        u8g2.setFont(u8g2_font_open_iconic_play_1x_t);
        u8g2.drawGlyph(40, 30, 64);
        u8g2.setFont(u8g2_font_ncenB08_tr);
        u8g2.setCursor(52, 30);
        u8g2.print(WavetypeRotation);
        u8g2.setFont(u8g2_font_5x7_tr);
        u8g2.setCursor(39, 20);
        u8g2.print("Wave");

        u8g2.setFont(u8g2_font_open_iconic_arrow_1x_t);
        u8g2.drawGlyph(78, 30, 88);
        u8g2.setFont(u8g2_font_ncenB08_tr);
        u8g2.setCursor(90, 30);
        u8g2.print(OctaveRotation);
        u8g2.setFont(u8g2_font_5x7_tr);
        u8g2.setCursor(79, 20);
        u8g2.print("Oct.");

        u8g2.setFont(u8g2_font_pressstart2p_8u);
        u8g2.drawGlyph(118, 28, 77);
      }
      else
      {
        u8g2.setFont(u8g2_font_iconquadpix_m_all);
        u8g2.drawGlyph(116, 30, 86);

        u8g2.setFont(u8g2_font_5x7_tr);
        u8g2.setCursor(5, 20);
        u8g2.print("Env");

        u8g2.setFont(u8g2_font_ncenB08_tr);

        if (EnvelopeSwitch)
        {
          u8g2.setCursor(4, 30);
          u8g2.print("ON");
        }
        else
        {
          u8g2.setCursor(2, 30);
          u8g2.print("OFF");
        }
        u8g2.setFont(u8g2_font_5x7_tr);
        u8g2.setCursor(38, 20);
        u8g2.print("Echo");

        u8g2.setFont(u8g2_font_ncenB08_tr);
        if (EchoSwitch)
        {
          u8g2.setCursor(40, 30);
          u8g2.print("ON");
        }
        else
        {
          u8g2.setCursor(38, 30);
          u8g2.print("OFF");
        }
      }

      if (!singleKeyboard)
      {
        u8g2.setFont(u8g2_font_ncenB08_tr);
        u8g2.setCursor(90, 10);
        u8g2.print((char)localRX_Message[0]);
        u8g2.print(localRX_Message[1]);
        u8g2.print(localRX_Message[2]);
        u8g2.print(localRX_Message[3]);
      }
    }

    u8g2.sendBuffer();

    digitalToggle(LED_BUILTIN);

    // UBaseType_t ux = uxTaskGetStackHighWaterMark( NULL );
    // Serial.print("stack: ");
    // Serial.println(ux);
  }
}

void decodeTask(void *pvParameters)
{
  uint32_t ID = 0x123;
  static uint32_t localCurrentStepSize = 0;

#ifdef POLYPHONY
  int32_t localCurrentStepSize[MAX_KEYS_PLAYED_TGT] = {};
  uint8_t notesPressed[MAX_KEYS_PLAYED_TGT] = {128, 128, 128, 128, 128, 128, 128, 128};
  uint8_t localKeyArray[7];
#else
  int32_t localKeyboardPositionIdx;
#endif

  uint8_t messageIn[8] = {};

  while (1)
  {

    if (keyboardMode == RECEIVER && !singleKeyboard)
    {
      if (xQueueReceive(msgInQ, messageIn, portMAX_DELAY) == pdTRUE)
      {
        xSemaphoreTake(RX_MessageMutex, portMAX_DELAY);
        memcpy((void *)RX_Message, &messageIn, 8);
        xSemaphoreGive(RX_MessageMutex);

        __atomic_load_n(&OctaveRotation, __ATOMIC_RELAXED);

#ifdef POLYPHONY

        xSemaphoreTake(stepSizeMutex, portMAX_DELAY);
        if (keyboardMode == RECEIVER)
        {
          // switch (messageIn[0]) {
          //   case 'P': {
          //     int value  = messageIn[2];
          //     xSemaphoreTake(activeNotesMutex, portMAX_DELAY);
          //     activeNotes.push_back(value);
          //     currentStepSize = stepSizes[value];
          //     xSemaphoreGive(activeNotesMutex);
          //     break;
          //   }
          //   case 'R': {
          //     uint16_t value  = messageIn[2];
          //     xSemaphoreTake(activeNotesMutex, portMAX_DELAY);
          //     activeNotes.erase(std::remove(activeNotes.begin(), activeNotes.end(), value), activeNotes.end());
          //     xSemaphoreGive(activeNotesMutex);
          //     break;
          //   }
          // }
          if (messageIn[0] == 'R')
          {
            for (int i = 0; i < MAX_KEYS_PLAYED_TGT; i++) 
            {
              if (notesPressed[i] == 128) currentStepSize[i] = 0;
            }
          }
          else if (messageIn[0] == 'P')
          {
            int32_t localCurrentStepSize = 0;
            int activeNotes;

            for (int i = 0; i < MAX_KEYS_PLAYED_TGT; i++) 
            {
              if (notesPressed[i] != 128) 
              {
                localCurrentStepSize = stepSizes[notesPressed[i]];
                currentStepSize[i] = localCurrentStepSize;
              } 
              else {
                currentStepSize[i] = 0;
              }
            }
          }
        }
        xSemaphoreGive(stepSizeMutex);
#else
        if ((messageIn[0] != prevMessageIn[0]) || (messageIn[2] != prevMessageIn[2]) || (messageIn[1] != prevMessageIn[1]))
        {
          if (messageIn[0] == 'R')
          {
            if (!EchoSwitch)
            {
              localurrentStepSize = 0;
              sineIdxAcc = 0;
            }
            RE = 0;
          }
          else
          {
            keyboardPositionIdx = messageIn[3];
            octave = OctaveRotation + keyboardPositionIdx;
            localCurrentStepSize = stepSizes[messageIn[2]];  
            sineIdxAcc = sineLookUpAcc[messageIn[2]];
            RE = 1;
          }
        }
        prevMessageIn[0] = messageIn[0];
        prevMessageIn[1] = messageIn[1];
        prevMessageIn[2] = messageIn[2];

#endif
      }
      __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
    }
    // UBaseType_t ux = uxTaskGetStackHighWaterMark( NULL );
    // Serial.print("stack: ");
    // Serial.println(ux);
  }
}

void CAN_RX_ISR(void)
{
  uint8_t RX_Message_ISR[8];
  uint32_t ID = 0x123;
  CAN_RX(ID, RX_Message_ISR);
  xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
}

void CAN_TX_ISR(void)
{
  xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);
}

void CAN_TX_Task(void *pvParameters)
{
  uint8_t msgOut[8];
  while (1)
  {
    xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
    xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
    CAN_TX(0x123, msgOut);
    // UBaseType_t ux = uxTaskGetStackHighWaterMark( NULL );
    // Serial.print("stack: ");
    // Serial.println(ux);
  }
}

void checkKeyboardandSetMode()
{
  uint8_t localKeyArray[7];
  delayMicroseconds(3000);
  keyScanningRoutine();
  xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
  memcpy(&localKeyArray, (void *)&keyArray, 7);
  xSemaphoreGive(keyArrayMutex);

  int eastDetect = (~localKeyArray[6] >> 3) & 1;
  int westDetect = (~localKeyArray[5] >> 3) & 1;
  // Serial.println(westDetect);
  // Serial.println(eastDetect);
  singleKeyboard = !westDetect & !eastDetect;
  if (singleKeyboard)
    keyboardMode = RECEIVER;
  if (!singleKeyboard && westDetect && !eastDetect)
  {
    keyboardMode = SENDER;
    keyboardPositionIdx = 1;
  }
  else if (!singleKeyboard && eastDetect && !westDetect)
  {
    keyboardMode = RECEIVER;
    keyboardPositionIdx = 0;
  }
}

uint32_t getModuleUID()
{
  uint32_t hash = 5381;
  uint32_t uid[3];

  uid[0] = HAL_GetUIDw0();
  uid[1] = HAL_GetUIDw1();
  uid[2] = HAL_GetUIDw2();

  // Perform a simple hash to reduce the 96-bit UID to a smaller data type
  for (int i = 0; i < 3; i++)
  {
    hash = ((hash << 5) + hash) + uid[i];
  }

  Serial.print("MODULE UID : ");
  Serial.println(hash);
  return hash;
}

void setup()
{
  generateSineLookUp();

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

  // Initialise UART
  Serial.begin(9600);

  msgInQ = xQueueCreate(36, 8);
  msgOutQ = xQueueCreate(36, 8);

  keyArrayMutex = xSemaphoreCreateMutex();
  stepSizeMutex = xSemaphoreCreateMutex();
  RX_MessageMutex = xSemaphoreCreateMutex();
  activeNotesMutex = xSemaphoreCreateMutex();
  keyboardPositionIdxMutex = xSemaphoreCreateMutex();

  CAN_TX_Semaphore = xSemaphoreCreateCounting(3, 3);

  HAL_Init();

  getModuleUID();

  checkKeyboardandSetMode();
  Serial.print("Keyboard Mode : ");
  Serial.println(keyboardMode ? "RECEIVER" : "SENDER");
  Serial.print("Single Keyboard : ");
  Serial.println(singleKeyboard ? "YES" : "NO");
  Serial.print("Keyboard Position : ");
  Serial.println(keyboardPositionIdx);

  TaskHandle_t scanKeysTaskHandle = NULL;
  xTaskCreate(
  scanKeysTask,		          /* Function that implements the task */
  "scanKeys",		            /* Text name for the task */
  64,      		              /* Stack size in words, not bytes */
  NULL,			                /* Parameter passed into the task */
  4,			                  /* Task priority */
  &scanKeysTaskHandle );	/* Pointer to store the task handle */

  TaskHandle_t displayTask = NULL;
  xTaskCreate(
      displayUpdateTask,
      "displayTasks",
      256,
      NULL,
      1,
      &displayTask);

  TaskHandle_t EchoHandle = NULL;
  xTaskCreate(
      echoes,
      "echoes",
      64,
      NULL,
      2,
      &EchoHandle);

  if (keyboardMode == RECEIVER && !singleKeyboard)
  {
    TaskHandle_t decodeTaskHandle = NULL;
    xTaskCreate(
        decodeTask,
        "decodeTask",
        64,
        NULL,
        3,
        &decodeTaskHandle);
  }

  if (keyboardMode == SENDER && !singleKeyboard)
  {
    TaskHandle_t TXTask = NULL;
    xTaskCreate(
        CAN_TX_Task,
        "CAN_TX_Task",
        64,
        NULL,
        3,
        &TXTask);
  }

  if (keyArrayMutex == NULL)
  {
    Serial.println("Error: Mutex was not created because the memory required to hold the mutex could not be allocated");
  }
  if (RX_MessageMutex == NULL)
  {
    Serial.println("Error: Mutex was not created because the memory required to hold the mutex could not be allocated");
  }

  if (!singleKeyboard)
  {
    CAN_Init(false);
    setCANFilter(0x123, 0x7ff);
    CAN_RegisterRX_ISR(CAN_RX_ISR);
    CAN_RegisterTX_ISR(CAN_TX_ISR);
    CAN_Start();
  }

  Serial.println("Done Setting Up");

  vTaskStartScheduler();
}

void loop()
{
  // checkKeyboardandSetMode();
}