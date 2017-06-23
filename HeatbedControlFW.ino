#include <PID_v1.h>
#include <SevSeg.h>
#include <TimerOne.h>
#include <ClickEncoder.h>
#include <EEPROM.h>
/* TinyFab.xyz heatbed controller

  Copyright 2017 tinyfab.xyz

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at
  http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
*/
//firmware Version
#define FIRMWARE_VERSION_MAJOR         1
#define FIRMWARE_VERSION_MINOR         0

//EEPROM ADDRESS
#define EEPROM_ADR_SETTEMP        0

//ENCODER SETTING
#define SETTING_ENCODER_STEP      (2)
#define SETTING_ENCODER_A_PIN     (A2)
#define SETTING_ENCODER_B_PIN     (A3)
#define SETTING_ENCODER_BTN_PIN   (A1)

//SEGMENT DISPLAY SETTING
#define SETTING_7SEG_NUM_DIG     (3)
#define SETTING_7SEG_DIG_1_PIN   (10)
#define SETTING_7SEG_DIG_2_PIN   (A4)
#define SETTING_7SEG_DIG_3_PIN   (A5)
#define SETTING_7SEG_A_PIN       (2)
#define SETTING_7SEG_B_PIN       (3)
#define SETTING_7SEG_C_PIN       (4)
#define SETTING_7SEG_D_PIN       (5)
#define SETTING_7SEG_E_PIN       (6)
#define SETTING_7SEG_F_PIN       (7)
#define SETTING_7SEG_G_PIN       (8)
#define SETTING_7SEG_H_PIN       (9)
#define SETTING_7SEG_BLINK_TIME  (800) //ms
#define SETTING_7SEG_BLINK_TIME2S  (2000) //ms
#define SETTING_7SEG_IDLE_TIME   (60000) //1 minute


//THERMISTER
#define SETTING_SENSOR_PIN       (A7)
#define SETTING_MAX_TEMP         (110)
#define SETTING_MIN_TEMP         (0)

//HEATER
#define SETTING_HEATER_PIN       (A0)
#define SETTING_AUX_PIN          (A6)
#define SETTING_PWM_PIN          (11)

//STATUS LED
#define SETTING_STATUS_PIN             (13)

//SYSTEM STATE
#define STATE_INIT                  (0)
#define STATE_OFF                   (1)
#define STATE_RUN_TEMP              (2)
#define STATE_RUN_TIMER             (3)
#define STATE_SET_TEMP              (4)
#define STATE_SET_TIMER             (5)
#define STATE_ERROR                 (6)


//Temperature STATE
#define TEMP_STATE_INIT                  (0)
#define TEMP_STATE_SAMPLE                (1)
#define TEMP_STATE_DATAREADY             (2)

//SYSTEM STATE
#define ENC_IDLE                    (0)
#define ENC_BTN_PRESS               (1)
#define ENC_BTN_HELD                (2)
#define ENC_BTN_RELEASED            (3)
#define ENC_BTN_CLICKED             (4)
#define ENC_BTN_DOUBLE_CLICKED      (5)
#define ENC_UP                      (6)
#define ENC_DOWN                    (7)

#define HEAT_STATE_UPPER_HYS        (1)
#define HEAT_STATE_LOWER_HYS        (0)
#define HEAT_MAX_FAULT_COUNT        (20) //number of fault allowed
#define HEAT_MAX_FAULT_COUNT_LOW    (50)
#define HEAT_MAX_FAULT_COUNT_MID    (100)
#define HEAT_MAX_FAULT_COUNT_HIGH   (200)
#define HEAT_FAULT_LOW_TEMP         (40) //degree c
#define HEAT_FAULT_MID_TEMP         (60)
#define HEAT_FAULT_HIGH_TEMP        (80)

//DEFAULT
#define HYSTERESIS    (2) //degree

//UNITS
#define UNIT_CELSIUS    (0)
#define UNIT_FARENHEIT  (1)

#define SECONDS (1000)
#define MINUTES (60000)
#define HOURS (360000)

//ERR
#define ERR_FLAG_UNKNOWN  (0)
#define ERR_FLAG_SENSOR   (1)
#define ERR_FLAG_TEMP     (2)

/////
char TXT_OFF[]  = "OFF";
char TXT_ON[]  = " ON";
char TXT_FAB[]  = "FAB";

////////////////////////////////////////////////
SevSeg sevseg; //Instantiate a seven segment controller object
ClickEncoder *encoder;

uint8_t sysState = STATE_INIT;
uint8_t heaterState = HEAT_STATE_LOWER_HYS;
uint8_t heaterEnable = 0;
float setTemp; //target Temperature
uint32_t setTime; //target duration
uint32_t heatStartTime;
float curTemp; //current Temperature
uint32_t curTime;
int8_t encoderValue; //-1, 0, +1
uint8_t setHysteresis;
uint8_t setMode;
uint32_t idleTimeout;
uint8_t blinkState;
uint32_t blinkTimeout;
uint8_t errFlag;
float heaterPWM;
double kp = 5, ki = 0.1, kd = 1;
//Specify the links and initial tuning parameters
PID heaterPID((double*)&curTemp, (double*)&heaterPWM, (double*)&setTemp, kp, ki, kd, P_ON_E, DIRECT); //P_ON_M specifies that Proportional on Measurement be used
//P_ON_E (Proportional on Error

///////////////////////////////////////////////

//isr for the encoder
void timerIsr(void) {
  encoder->service();
}

void reset_IdleTimeout(void)
{
  idleTimeout = millis() + SETTING_7SEG_IDLE_TIME;
}

uint8_t encoderHandler(void)
{
  uint8_t retVal;

  encoderValue = encoder->getValue();

  if (encoderValue == 1 )
  {
    retVal = ENC_UP;
  }
  else if (encoderValue == -1)
  {
    retVal = ENC_DOWN;
  }
  else
  {
    ClickEncoder::Button b = encoder->getButton();
    if (b != ClickEncoder::Open)
    {
      switch (b)
      {
        case ClickEncoder::Pressed:
          retVal = ENC_BTN_PRESS;
          break;

        case ClickEncoder::Held:
          retVal = ENC_BTN_HELD;
          break;

        case ClickEncoder::Released:
          retVal = ENC_BTN_RELEASED;
          break;

        case ClickEncoder::Clicked:
          retVal = ENC_BTN_CLICKED;
          break;

        case ClickEncoder::DoubleClicked:
          retVal = ENC_BTN_DOUBLE_CLICKED;
          break;
      }
    }
    else
    {
      retVal = ENC_IDLE;
    }
  }

  if (retVal != ENC_IDLE)
  {
    reset_IdleTimeout();
  }
  return retVal;
}


// which analog pin to connect
#define THERMISTORPIN       A7
// resistance at 25 degrees C
#define THERMISTORNOMINAL   100000
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL  25
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES          50
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT        3950
// the value of the 'other' resistor
#define SERIESRESISTOR      4700
// time between samples
#define TEMP_SAMPLETIME     20//millisecond   

void temperatureHandler(void)
{
  static uint8_t tempState = TEMP_STATE_INIT;
  static uint8_t sampleCount = 0;
  static uint32_t sampleTimeout = 0;
  static float average;
  static int samples[NUMSAMPLES];
  uint8_t i;

  switch (tempState)
  {
    case TEMP_STATE_INIT:
      {
        sampleCount = 0;
        sampleTimeout = 0;
        tempState = TEMP_STATE_SAMPLE;
      }
      break;

    case TEMP_STATE_SAMPLE:
      {
        if (sampleTimeout < millis())
        {
          samples[sampleCount] = analogRead(THERMISTORPIN);
          sampleCount++;
          if (sampleCount >= NUMSAMPLES)
          {
            tempState = TEMP_STATE_DATAREADY;
          }
          sampleTimeout = millis() + TEMP_SAMPLETIME;
        }
      }
      break;

    case TEMP_STATE_DATAREADY:
      {
        // average all the samples out
        average = 0;
        for (i = 0; i < NUMSAMPLES; i++) {
          average += samples[i];
        }
        average /= NUMSAMPLES;

        //      Serial.print("Average analog reading ");
        //      Serial.println(average);

        // convert the value to resistance
        average = 1023 / average - 1;
        average = SERIESRESISTOR / average;
        //      Serial.print("Thermistor resistance ");
        //      Serial.println(average);

        float steinhart;
        steinhart = average / THERMISTORNOMINAL;     // (R/Ro)
        steinhart = log(steinhart);                  // ln(R/Ro)
        steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
        steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
        steinhart = 1.0 / steinhart;                 // Invert
        steinhart -= 273.15;                         // convert to C

        curTemp = steinhart;
        tempState = TEMP_STATE_INIT;

        //        Serial.print("temp =  ");
        //        Serial.println(steinhart);
      }
      break;
  }
}

void heaterOff(void)
{
  digitalWrite(SETTING_HEATER_PIN, LOW);
  digitalWrite(SETTING_AUX_PIN, LOW);
  analogWrite(SETTING_PWM_PIN, 0);
  digitalWrite(SETTING_STATUS_PIN, LOW);
}

void heaterOn(void)
{
  digitalWrite(SETTING_HEATER_PIN, HIGH);
  digitalWrite(SETTING_AUX_PIN, HIGH);
  analogWrite(SETTING_PWM_PIN, heaterPWM);
  digitalWrite(SETTING_STATUS_PIN, HIGH);
}

void heaterHandler(void)
{
  float tempThreshold;
  static float lastTemp;
  static uint8_t faultCounter;
  uint16_t faultMax;

  if ((sysState == STATE_INIT) || (sysState == STATE_ERROR))
  {
    heaterOff();
    return;
  }

  //check sensor
  if ((curTemp <= SETTING_MIN_TEMP) || (curTemp > SETTING_MAX_TEMP))
  {
    Serial.print("exceeded temp\n");
    heaterOff();
    errFlag |= ERR_FLAG_SENSOR;
    sysState = STATE_ERROR;
  }
  else if (((setTime + heatStartTime) < millis()) && (setTime != 0))
  {
    Serial.print("timeout\n");
    heaterOff();
    reset_IdleTimeout();
    sysState = STATE_OFF;
    setTime = 0;
  }
  else
  {
    if (heaterEnable)
    {
      heaterPID.Compute();
      Serial.print(heaterPWM);
      Serial.print("\n");
      heaterOn();
      //      if (curTemp > setTemp)
      //      {
      //        heaterState = HEAT_STATE_UPPER_HYS;
      //      }
      //
      //      if (curTemp < setTemp - HYSTERESIS)
      //      {
      //        heaterState = HEAT_STATE_LOWER_HYS;
      //      }
      //
      //      if (heaterState == HEAT_STATE_LOWER_HYS)
      //      {
      //        tempThreshold =  setTemp;
      //      }
      //      else
      //      {
      //        tempThreshold = setTemp - HYSTERESIS;
      //      }
      //
      //      if (curTemp < tempThreshold)
      //      {
      //        heaterOn();
      //        //simple protection
      //        if (lastTemp > curTemp)
      //          //assume the temperature will be the same or increase with heater on
      //        {
      //          faultCounter++;
      //        }
      //
      //        if (curTemp >= HEAT_FAULT_HIGH_TEMP)
      //        {
      //          faultMax = HEAT_MAX_FAULT_COUNT_HIGH;
      //        }
      //        else if (curTemp >= HEAT_FAULT_MID_TEMP)
      //        {
      //          faultMax = HEAT_MAX_FAULT_COUNT_MID;
      //        }
      //        else if (curTemp >= HEAT_FAULT_LOW_TEMP)
      //        {
      //          faultMax = HEAT_MAX_FAULT_COUNT_LOW;
      //        }
      //        else
      //        {
      //          faultMax = HEAT_MAX_FAULT_COUNT;
      //        }
      //
      //        if (faultCounter > faultMax)
      //        {
      //          Serial.print("temperature fault\n");
      //          heaterOff();
      //          errFlag |= ERR_FLAG_TEMP;
      //          sysState = STATE_ERROR;
      //        }
      //        lastTemp = curTemp;
      //      }
      //      else
      //      {
      //        heaterOff();
      //        faultCounter = 0;
      //      }
    }
    else
    {
      heaterOff();
      faultCounter = 0;
    }
  }
}

void setup(void) {
  byte numDigits = SETTING_7SEG_NUM_DIG;
  byte digitPins[] = {SETTING_7SEG_DIG_1_PIN, SETTING_7SEG_DIG_2_PIN, SETTING_7SEG_DIG_3_PIN};
  byte segmentPins[] = {SETTING_7SEG_A_PIN, SETTING_7SEG_B_PIN, SETTING_7SEG_C_PIN, SETTING_7SEG_D_PIN, SETTING_7SEG_E_PIN, SETTING_7SEG_F_PIN, SETTING_7SEG_G_PIN, SETTING_7SEG_H_PIN};
  bool resistorsOnSegments = true; // 'false' means resistors are on digit pins
  byte hardwareConfig = COMMON_ANODE; // See README.md for options
  bool updateWithDelays = false; // Default. Recommended
  bool leadingZeros = false; // Use 'true' if you'd like to keep the leading zeros
  Serial.begin(19200);
  float f;

  heaterPID.SetMode(AUTOMATIC);

  //encoder
  encoder = new ClickEncoder(SETTING_ENCODER_A_PIN, SETTING_ENCODER_B_PIN, SETTING_ENCODER_BTN_PIN, SETTING_ENCODER_STEP);
  encoder->setAccelerationEnabled(true);
  Timer1.initialize(1000);
  Timer1.attachInterrupt(timerIsr);

  EEPROM.get(EEPROM_ADR_SETTEMP, f );
  if (isnan(f))
  {
    setTemp = 0; //init temp setting
  }
  else
  {
    setTemp = f;
  }

  setTime = 0; //init timer setting to infinity
  heatStartTime = 0;
  errFlag = 0;
  reset_IdleTimeout();


  //7seg
  sevseg.begin(hardwareConfig, numDigits, digitPins, segmentPins, resistorsOnSegments, updateWithDelays, leadingZeros);
  sevseg.setBrightness(90);

  //heat
  pinMode(SETTING_HEATER_PIN, OUTPUT);
  //  digitalWrite(SETTING_HEATER_PIN, LOW);

  pinMode(SETTING_AUX_PIN, OUTPUT);
  //  digitalWrite(SETTING_AUX_PIN, LOW);
  //status
  pinMode(SETTING_STATUS_PIN, OUTPUT);
  //  digitalWrite(SETTING_STATUS_PIN, LOW);

}

//main loop
void loop(void) {
  uint8_t encoderFlag;
  static uint32_t tempSetTemp = 0;
  static uint32_t tempSetTime = 0;

  char dispTXT[4];
  encoderFlag = encoderHandler();
  temperatureHandler();
  heaterHandler();

  switch (sysState) {
    case STATE_INIT:
      {
        heaterEnable = false;
        if (millis() < 1000)
        {
          sevseg.setNumber((FIRMWARE_VERSION_MAJOR * 100) + FIRMWARE_VERSION_MINOR, 2);
        }
        else
        {
          sevseg.setChars(TXT_FAB);
        }

        //check sensor
        if ((curTemp > SETTING_MIN_TEMP) && (curTemp < SETTING_MAX_TEMP))
        {
          if (millis() > 2000)
          {
            Serial.print("state: STATE_INIT -> STATE_OFF\n");
            sysState = STATE_OFF;
          }
        }

        //if this is still running in 2 second the sensor is dead
        if (millis() > 2000)
        {
          errFlag |= ERR_FLAG_SENSOR;
          sysState = STATE_ERROR;
        }

      }
      break;

    case STATE_ERROR:
      {
        heaterEnable = false;
        sprintf(dispTXT, "E%02x", errFlag);
        sevseg.setChars(dispTXT);
        //clear temperature error
        if ((curTemp > SETTING_MIN_TEMP) && (curTemp < SETTING_MAX_TEMP))
        {
          errFlag &= ~ERR_FLAG_SENSOR;
        }

        switch (encoderFlag)
        {
          case ENC_BTN_CLICKED:
            {
              errFlag &= ~ERR_FLAG_TEMP;
            }
            break;
          default:
            break;
        }

        if (errFlag == 0)
        {
          Serial.print("state: STATE_INIT -> STATE_OFF\n");
          sysState = STATE_OFF;
        }

      }
      break;

    case STATE_OFF:
      {
        static boolean sleep = false;
        heaterEnable = false;
        switch (encoderFlag)
        {
          case ENC_UP:
          case ENC_DOWN:
            {
              if (sleep == true)
              {
                sleep = false;
              }
              else
              {
                if (setTemp == 0)
                {
                  tempSetTemp = curTemp;
                }
                else
                {
                  tempSetTemp = setTemp;
                }
                sysState = STATE_SET_TEMP;
                Serial.print("state: STATE_OFF -> STATE_SET_TEMP\n");
              }
            }
            break;

          case ENC_BTN_CLICKED:
            {
              if (sleep == true)
              {
                sleep = false;
              }
              else
              {
                heaterState = HEAT_STATE_LOWER_HYS;
                sysState = STATE_RUN_TEMP;
                Serial.print("state: STATE_OFF -> STATE_RUN_TEMP\n");
              }
            }
            break;

          default:
            {
              if (idleTimeout < millis())
              {
                sevseg.blank();
                sleep = true;
              }
              else
              {
                if (blinkTimeout < millis())
                {
                  blinkTimeout = millis() + SETTING_7SEG_BLINK_TIME2S;
                  blinkState++;
                  if (blinkState & 1)
                  {
                    sevseg.setChars(TXT_OFF);
                  }
                  else
                  {
                    if (curTemp > 99.9)
                    {
                      sevseg.setNumber(curTemp, 0);
                    }
                    else
                    {
                      sevseg.setNumber(curTemp, 1);
                    }
                  }
                }
              }
            }
            break;
        }
      }
      break;

    case STATE_RUN_TEMP:
      {
        //check temp here and do stuff to it if it meets criterias
        heaterEnable = true;
        switch (encoderFlag)
        {
          case ENC_UP:
          case ENC_DOWN:
            //set temp
            {
              if (setTemp == 0)
              {
                tempSetTemp = curTemp;
              }
              else
              {
                tempSetTemp = setTemp;
              }
              sysState = STATE_SET_TEMP;
              Serial.print("state: STATE_RUN_TEMP -> STATE_SET_TEMP\n");
            }
            break;

          case ENC_BTN_CLICKED:
            {
              Serial.print("state: STATE_SET_TIMER -> STATE_RUN_TIMER\n");
              sysState = STATE_RUN_TIMER;
            }
            break;

          case ENC_BTN_HELD:
            {
              sysState = STATE_OFF;
              Serial.print("state: STATE_RUN_TEMP -> STATE_OFF\n");
            }
            break;

          default:
            if (curTemp > 99.9)
            {
              sevseg.setNumber(curTemp, 0);
            }
            else
            {
              sevseg.setNumber(curTemp, 1);
            }
            break;
        }
      }
      break;

    case STATE_RUN_TIMER:
      {
        //check temp here and do stuff to it if it meets criterias
        heaterEnable = true;
        switch (encoderFlag)
        {
          case ENC_BTN_CLICKED:
            {
              Serial.print("state: STATE_RUN_TIMER -> STATE_RUN_TEMP\n");
              sysState = STATE_RUN_TEMP;
            }
            break;

          case ENC_BTN_HELD:
            {
              sysState = STATE_OFF;
              Serial.print("state: STATE_RUN_TEMP -> STATE_OFF\n");
            }
            break;

          case ENC_UP:
          case ENC_DOWN:
            //timer mode
            {
              uint32_t tempTime = millis();
              if (heatStartTime != 0)
              {
                if ((tempSetTime + heatStartTime) > tempTime)
                {
                  tempSetTime = tempSetTime + heatStartTime - tempTime;
                }
                else
                {
                  tempSetTime = 0;
                }
              }
              sysState = STATE_SET_TIMER;
              Serial.print("state: STATE_RUN_TEMP -> STATE_SET_TIMER\n");
            }
            break;

          default:
            uint32_t tempTime = millis();
            if ((tempSetTime + heatStartTime) > tempTime)
            {
              curTime = tempSetTime + heatStartTime - tempTime;
            }
            else
            {
              curTime = 0;
            }

            if (curTime == 0)
            {
              sevseg.setChars(TXT_ON);
            }
            //            else if (curTime < MINUTES)
            //            {
            //              sevseg.setNumber((curTime / SECONDS), 2);
            //            }
            else if (curTime < MINUTES * 10) //9 min 60 sec
            {
              sevseg.setNumber(((curTime / SECONDS) % 60) + ((curTime / SECONDS) / 60) * 100, 2);
            }
            else if (curTime < HOURS * 10) //9 hr 60 min
            {
              sevseg.setNumber(((curTime / MINUTES) % 60) + ((curTime / MINUTES) / 60) * 100, 2);
            }
            else
            {
              sevseg.setNumber(curTime / MINUTES, 0);
            }
            break;
        }
      }
      break;

    case STATE_SET_TEMP:
      {
        //turn off heat first (safety)
        heaterEnable = false;

        switch (encoderFlag)
        {
          case ENC_UP:
          case ENC_DOWN:
            {
              tempSetTemp += encoderValue;
              if (tempSetTemp > SETTING_MAX_TEMP)
              {
                tempSetTemp = SETTING_MAX_TEMP;
              }
              if (tempSetTemp <= SETTING_MIN_TEMP)
              {
                tempSetTemp = SETTING_MIN_TEMP + 1;
              }
              if (tempSetTemp > 99)
              {
                sprintf(dispTXT, "%03d", tempSetTemp);
              }
              else
              {
                sprintf(dispTXT, "%02dc", tempSetTemp);
              }
              sevseg.setChars(dispTXT);
            }
            break;

          case ENC_BTN_CLICKED: //save and run
            {
              //set temp here

              Serial.print("state: STATE_SET_TEMP -> STATE_RUN_TEMP\n");
              heaterState = HEAT_STATE_LOWER_HYS;
              sysState = STATE_RUN_TEMP;
              setTemp = tempSetTemp;
              EEPROM.put(EEPROM_ADR_SETTEMP, setTemp);
            }
            break;

          case ENC_BTN_HELD:
            {
              sysState = STATE_OFF;
            }
            break;

          default:
            if (tempSetTemp > 99)
            {
              sprintf(dispTXT, "%03d", tempSetTemp);
            }
            else
            {
              sprintf(dispTXT, "%02dc", tempSetTemp);
            }
            break;
        }

        if (blinkTimeout < millis())
        {
          blinkTimeout = millis() + SETTING_7SEG_BLINK_TIME;
          blinkState++;
          if (blinkState & 1)
          {
            sevseg.setChars(dispTXT);
          }
          else
          {
            sevseg.blank();
          }
        }

        if (idleTimeout < millis())
        {
          sysState = STATE_RUN_TEMP;
          Serial.print("state: STATE_SET_TEMP(timeout) -> STATE_RUN_TEMP\n");
        }
      }
      break;

    case STATE_SET_TIMER:
      {
        //turn off heat first (safety)
        //        heaterEnable = false;
        switch (encoderFlag)
        {
          case ENC_DOWN:
            {
              if (tempSetTime < MINUTES)
              {
                tempSetTime = 0;
                sevseg.setChars(TXT_ON);
                break;
              }
            }//delibrate no break
          case ENC_UP:
            {
              //minute
              tempSetTime += (encoderValue * MINUTES);
              //setTime in millisecond
              sevseg.setNumber(tempSetTime / MINUTES, 0);
            }
            break;

          case ENC_BTN_CLICKED: //save and run
            {
              Serial.print("state: STATE_SET_TIMER -> STATE_RUN_TEMP\n");
              setTime = tempSetTime;
              heatStartTime = millis();
              sysState = STATE_RUN_TIMER;
            }
            break;

          default:
            break;
        }

        if (blinkTimeout < millis())
        {
          blinkTimeout = millis() + SETTING_7SEG_BLINK_TIME;
          blinkState++;
          if (blinkState & 1)
          {
            if (tempSetTime < MINUTES)
            {
              sevseg.setChars(TXT_ON);
            }
            else
            {
              sevseg.setNumber(tempSetTime / MINUTES, 0);
            }
          }
          else
          {
            sevseg.blank();
          }
        }

        if (idleTimeout < millis())
        {
          sysState = STATE_RUN_TEMP;
          heaterState = HEAT_STATE_LOWER_HYS;
          Serial.print("state: STATE_SET_TIMER(timeout) -> STATE_RUN_TEMP\n");
        }
      }
      break;
  }

  sevseg.refreshDisplay(); // Must run repeatedly
}

/// END ///
