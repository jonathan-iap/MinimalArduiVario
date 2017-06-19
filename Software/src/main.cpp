/*
Inspired by http://wildlab.org/index.php/2015/07/07/arduino-variometer/

Created on: 2 avr. 2016
Author: Jonathan Iapicco
*/

// Includes -------------------------------------------------------------------
#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include "PowerSaving.h"
#include "defines.h"
#include "CustomToneAC.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_BMP085_U.h" // Works for BMP180


// Global variables -----------------------------------------------------------
uint8_t buttons[]={BTN_UP, BTN_DOWN, BTN_SELECT};
uint8_t volume;
int8_t sensibility;
bool falling;
// DEBUG
// unsigned long time = 0;
// unsigned long timeTest = 0;


// Global object --------------------------------------------------------------
Adafruit_BMP085_Unified BMP180 = Adafruit_BMP085_Unified(10085);


// Functions declaration-------------------------------------------------------
// Init and checking
void killPocess(void);
bool isLimit(void);
bool isConfirm(uint8_t _volume, uint16_t _tone);
void initEeprom(void);
void readEeprom(void);
void CtrlSensor(void);
void displaySensorDetails(void);
// Engine function
bool vario(void);
bool bipSound(int16_t _toneFreq, int16_t _ddsAcc);
// settings
uint8_t getButtons(void);
void setVolume(uint8_t _button);
void setSensibility(uint8_t _button);
void setMode(void);
void menuSetting(uint8_t _button);


/******************************************************************************
SETUP
*****************************************************************************/
void setup()
{
  // DEBUG
  delay(2000); // Allow to be flash if watchdog fail
  // Power
  init_PowerSaving(); // Must be placed before any codes
  // Debug
  #ifdef DEBUG
  Serial.begin(BAUDRATE);
  #endif
  // Leds
  pinMode(LED_GOOD, OUTPUT);
  pinMode(LED_ERROR, OUTPUT);
  // Buttons
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_SELECT, INPUT_PULLUP);
  // Load settings
  initEeprom();
  // Initialization and check of the pressure sensor
  CtrlSensor();
  // i2C
  Wire.setClock(400000L); // change i2c speed to 400Khz (need to be placed after "wire.begin")
}


/******************************************************************************
LOOP
*****************************************************************************/
void loop()
{
  // Reading buttons and update settings
  menuSetting(getButtons());
  // Main function
  if(vario()) delay(20);
  else sleeping(WDT_30MS);
}


/******************************************************************************
Functions
******************************************************************************/

/*-----------------------------------------------------------------------------
Details : Just turn off tone and set leds to low
------------------------------------------------------------------------------*/
void killPocess(void)
{
  toneOff();
  digitalWrite(LED_GOOD, LOW);
  digitalWrite(LED_ERROR, LOW);
}

/*-----------------------------------------------------------------------------
Details : Alert if a limit is reached
------------------------------------------------------------------------------*/
bool isLimit(void)
{
  killPocess();

  digitalWrite(LED_ERROR, HIGH);
  toneOn(TONE_LIMIT, VOL_MAX, 500, LOW);
  digitalWrite(LED_ERROR, LOW);

  return true;
}

/*-----------------------------------------------------------------------------
Details : Confirm if a setting is changed
------------------------------------------------------------------------------*/
bool isConfirm(uint8_t _volume, uint16_t _tone)
{
  killPocess();

  digitalWrite(LED_GOOD, HIGH);
  toneOn(_tone, _volume, 200, LOW);
  digitalWrite(LED_GOOD, LOW);

  return true;
}

/*-----------------------------------------------------------------------------
Details : If you upload the code for the first time, memory can contain
invalid values.To correct this we set memory with default values after checking
the memory integrity.
------------------------------------------------------------------------------*/
void initEeprom(void)
{
  // Read old values
  readEeprom();

  // If values are corrupted we set default values
  if (volume > VOL_MAX || sensibility > MAX_SENS)
  {
    EEPROM.write(MEM_VOLUME, 5 );
    EEPROM.write(MEM_SENS, MIN_SENS);
    EEPROM.write(MEM_FALL, true);
    // Update values
    readEeprom();
  }
}

/*-----------------------------------------------------------------------------
Details : Read last settings
------------------------------------------------------------------------------*/
void readEeprom(void)
{
  volume = EEPROM.read(MEM_VOLUME);
  sensibility = EEPROM.read(MEM_SENS);
  falling = EEPROM.read(MEM_FALL);
}

/*-----------------------------------------------------------------------------
Details : Check if the communication with the sensor is correct.
If status is correct led GREEN blink, if not two long tone are played and
Led RED stay on.
------------------------------------------------------------------------------*/
void CtrlSensor(void)
{
  // First tone to say that the your device is running
  isConfirm(VOL_MAX, TONE_CONFIRM);
  #ifdef DEBUG
  Serial.print("Vario is on : ");
  #endif
  delay(50);

  // Initialization of pressure sensor
  bool state = BMP180.begin(BMP085_MODE_ULTRALOWPOWER);

  // Control routine
  // Error : Led error stays on, sensor is probably dead or power is not correct.
  if (!state)
  {
    isLimit();
    delay(100);
    isLimit();
    digitalWrite(LED_ERROR, HIGH);

    #ifdef DEBUG
    Serial.println("ERROR -> No detected sensor !");
    Serial.println("Please Check your I2C ADDR, your power and your wires");
    #endif

    while (1){} // Stop program
  }
  // OK : if sensor is correctly initialise led green and buzzer are plays
  else
  {
    // Second tone sensor is ok program run
    isConfirm(VOL_MAX, TONE_CONFIRM);

    #ifdef DEBUG
    displaySensorDetails();
    #endif
  }
}

/*-----------------------------------------------------------------------------
Details : Displays some basic information on this sensor from the unified
sensor API sensor_t type (see Adafruit_Sensor for more information)
------------------------------------------------------------------------------*/
void displaySensorDetails(void)
{
  Serial.println("Sensor is OK");

  sensor_t sensor;
  BMP180.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" hPa");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" hPa");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" hPa");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

/*-----------------------------------------------------------------------------
Details : Engine function, get pressure and filters values
------------------------------------------------------------------------------*/
bool vario(void)
{
  static int16_t ddsAcc;
  static float pressure, toneFreq, toneFreqLowpass;

  // DEBUG
  // timeTest = micros();

  // Value reading
  BMP180.getPressure(&pressure);

  // DEBUG
  // timeTest = (micros() - timeTest);
  //  Serial.print("Millis pressure : ");
  //  Serial.println(timeTest);
  //  Serial.println("");

  // For fast Initialization
  static float lowPassFast = pressure;
  static float lowPassSlow = pressure;

  // DEBUG
  // Serial.println("-----------------------------");
  // Serial.print("pressure: "); Serial.println(pressure,4);

  // Filtering on two levels
  lowPassFast = lowPassFast + (pressure - lowPassFast) * COEF_FAST;
  lowPassSlow = lowPassSlow + (pressure - lowPassSlow) * COEF_SLOW;

  // DEBUG
  // Serial.print("lowPassFast : "); Serial.println(lowPassFast,4);
  // Serial.print("lowPassSlow : "); Serial.println(lowPassSlow,4);
  // DEBUG PLOT
  // Serial.print(lowPassFast,4);
  // Serial.print(" ");
  // Serial.println(lowPassSlow,4);

  // Make difference
  toneFreq = (lowPassSlow - lowPassFast) * 50;

  // DEBUG
  // Serial.print("toneFreq: "); Serial.println(toneFreq,4);

  // New filtering
  toneFreqLowpass = toneFreqLowpass + (toneFreq - toneFreqLowpass) * COEF_LOWPASS;

  // DEBUG
  // Serial.print("toneFreqLowpass: "); Serial.println(toneFreqLowpass,4);

  // Value is constrain to stay in audible frequency
  toneFreq = constrain(toneFreqLowpass, -400, 500);

  // DEBUG
  // Serial.print("toneFreq: "); Serial.println(toneFreq,4);

  // "ddsAcc" give a "delay time" to produce a bip-bip-bip....
  ddsAcc += toneFreq * 100 + 500;

  // DEBUG
  // Serial.print("ddsAcc: "); Serial.println(ddsAcc);

  // Play tone depending variation.
  return bipSound(toneFreq, ddsAcc);
}

/*-----------------------------------------------------------------------------
Details : Manage tone
------------------------------------------------------------------------------*/
bool bipSound(int16_t _toneFreq, int16_t _ddsAcc)
{
  static bool ledErrState = LOW;

  // DEBUG
  //Serial.print("toneFreq: "); Serial.println(toneFreq);

  // Falling enable or staidy
  if (_toneFreq < sensibility || _ddsAcc > 0)
  {
    if (falling == true && _toneFreq < MIN_FALL) // Falling detection enable, falling tone
    {
      toneOn(_toneFreq + SOUND_FALL, volume);
      digitalWrite(LED_GOOD, LOW);
      if(ledErrState)
      {
        digitalWrite(LED_ERROR, LOW);
        ledErrState = LOW;
      }
      else
      {
        digitalWrite(LED_ERROR, HIGH);
        ledErrState = HIGH;
      }
    }
    else if (falling == true && _toneFreq > sensibility) // Falling detection enable, rising tone
    {
      toneOn(_toneFreq + SOUND_RISE, volume);
      digitalWrite(LED_GOOD, HIGH);
      digitalWrite(LED_ERROR, LOW);
      ledErrState = LOW;
    }
    else // Falling detection disable or steady give no tone
    {
      toneOff();
      digitalWrite(LED_GOOD, LOW);
      digitalWrite(LED_ERROR, LOW);
      ledErrState = LOW;
      return false;
    }
  }
  else // Rising
  {
    // Falling detection enable: rising become falling...
    if (falling == true)
    {
      toneOff();
      digitalWrite(LED_GOOD, LOW);
      return false;
    }
    // Falling detection disable
    else
    {
      toneOn(_toneFreq + SOUND_RISE, volume);
      digitalWrite(LED_GOOD, HIGH);
    }
  }
  return true;
}

/*-----------------------------------------------------------------------------
Details : Get buttons status.
------------------------------------------------------------------------------*/
uint8_t getButtons(void)
{
  uint8_t btnState = 0;

  // Read buttons
  for(int i=(Nb_Of_BTN-1); i>=0; i--)
  {
    if( !digitalRead(buttons[i]) )
    {
      btnState += buttons[i]; // Make the sum if many buttons are pressed
    }
  }

  // Return an unique value
  return btnState;
}

/*-----------------------------------------------------------------------------
Details : Set volume depending buttons status
------------------------------------------------------------------------------*/
void setVolume(uint8_t _button)
{
  uint8_t lastVolume = volume;

  if(_button == BTN_UP) // Increase volume
  {
    if(volume < VOL_MAX)
    {
      volumeUpdate(volume++);
      isConfirm(volume, TONE_CONFIRM);
    }
    else isLimit();
  }
  else if(_button == BTN_DOWN) // Decrease volume
  {
    if(volume > VOL_MIN)
    {
      volumeUpdate(volume--);
      isConfirm(volume, TONE_CONFIRM);
    }
    else isLimit();
  }

  // Save setting only if was changed
  if(sensibility != lastVolume) EEPROM.write(MEM_VOLUME, volume);
}

/*-----------------------------------------------------------------------------
Details : Set sensibility depending buttons status
------------------------------------------------------------------------------*/
void setSensibility(uint8_t _button)
{
  uint8_t lastSensibility = sensibility;
  bool isMax = false;

  if(_button == (BTN_SELECT+BTN_UP)) // Decrease sensibility
  {
    if(sensibility < MAX_SENS) sensibility += STEP_SENS;
    else isMax = true;
  }
  else if(_button == (BTN_SELECT+BTN_DOWN)) // Increase sensibility
  {
    if(sensibility > MIN_SENS) sensibility -= STEP_SENS;
    else isMax = true;
  }

  // Inform user what is current setting
  for(uint8_t i ; i<sensibility; i+=STEP_SENS)
  {
    isConfirm(VOL_MAX, TONE_CONFIRM+200);
    delay(50);
  }

  // If setting is at the maximum value
  if(isMax) isLimit();

  // Save setting only if was changed
  if(sensibility != lastSensibility) EEPROM.write(MEM_SENS, sensibility);
}

/*-----------------------------------------------------------------------------
Details : Enabling or disabling falling detection
------------------------------------------------------------------------------*/
void setMode(void)
{
  bool lastFalling = falling;

  // Enable/disable falling detection
  falling =! falling;
  // Save in EEprom
  EEPROM.write(MEM_FALL, falling);

  if(lastFalling) // falling was active and is now inactive
  {
    isConfirm(VOL_MAX, TONE_CONFIRM);
    delay(2);
    isConfirm(VOL_MAX, TONE_CONFIRM);
  }
  else // falling was inactive and is now active
  {
    isConfirm(VOL_MAX, TONE_CONFIRM);

    digitalWrite(LED_ERROR, HIGH);

    for(uint16_t i = SOUND_FALL ; i > (SOUND_FALL-200); i--)
    {
      toneOn(i, volume);
      delay(2);
    }
  }
}

/*-----------------------------------------------------------------------------
Details : Sorts buttons manipulation.
------------------------------------------------------------------------------*/
void menuSetting(uint8_t _button)
{
  static bool lastState = false;

  if(_button == BTN_SELECT || _button == (BTN_SELECT+BTN_UP) || _button == (BTN_SELECT+BTN_DOWN))
  {
    killPocess();

    // Do nothing while Buttons SELECT is pressed
    while((_button = getButtons()) == BTN_SELECT ){}

    if(_button == (BTN_SELECT+BTN_UP) || _button == (BTN_SELECT+BTN_DOWN))
    {
      lastState = true;

      setSensibility(_button);

      delay(DEBOUNCE);
    }
    else if(lastState) // Prevents to change again falling detection
    {
      lastState = false;
    }
    else setMode(); // Enable/disable falling detection
  }
  else
  {
    setVolume(_button);
  }
}
