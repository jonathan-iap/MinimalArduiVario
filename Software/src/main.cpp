// Includes -------------------------------------------------------------------
#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include "defines.h"
#include "CustomToneAC.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_BMP085_U.h" // Works for BMP180


// Defines --------------------------------------------------------------------
#define BAUDRATE 9600
#define Nb_Of_BTN 3

// Global variables -----------------------------------------------------------
uint8_t buttons[]={BTN_UP, BTN_DOWN, BTN_SELECT};
uint8_t volume;
int8_t sensibility;
bool falling;
unsigned long time = 0;
unsigned long timeTest = 0;
// Global object --------------------------------------------------------------
Adafruit_BMP085_Unified BMP180 = Adafruit_BMP085_Unified(10085);

// Functions declaration-------------------------------------------------------
// Init and checking
bool isLimit();
bool isConfirm(uint8_t _volume, uint16_t _tone);
void initEeprom();
void readEeprom();
void CtrlSensor();
void displaySensorDetails();
// Engine function
void vario();
void bipSound(int16_t toneFreq, int p_ddsAcc);
// settings
uint8_t getButtons();
void setVolume(uint8_t _button);
void setSensibility(uint8_t _button);
void setMode();
void menuSetting(uint8_t _button);

/******************************************************************************
SETUP
*****************************************************************************/
void setup()
{
  // Debug
  Serial.begin(BAUDRATE);
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
  // Fonction principal.
  vario();

  delay(20);

  menuSetting(getButtons());
}

/******************************************************************************
Functions
******************************************************************************/
// Alert if a limit is reach
bool isLimit()
{
  toneOff();
  digitalWrite(LED_GOOD, LOW);

  digitalWrite(LED_ERROR, HIGH);
  toneOn(TONE_LIMIT, VOL_MAX, 500, LOW);
  digitalWrite(LED_ERROR, LOW);

  return true;
}

// Confirm if a setting is changed
bool isConfirm(uint8_t _volume, uint16_t _tone)
{
  toneOff();
  digitalWrite(LED_ERROR, LOW);

  digitalWrite(LED_GOOD, HIGH);
  toneOn(_tone, _volume, 200, LOW);
  digitalWrite(LED_GOOD, LOW);

  return true;
}


void initEeprom()
{
  /*
  * If you upload the code for the first time, memory can contain invalid values.
  * To correct this we set memory with default values.
  */

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


void readEeprom()
{
  volume = EEPROM.read(MEM_VOLUME);
  sensibility = EEPROM.read(MEM_SENS);
  falling = EEPROM.read(MEM_FALL);
}


void CtrlSensor()
{
  // To say that the your device is running
  isConfirm(VOL_MAX, TONE_CONFIRM); /* rechange for VOL_MAX /!\*/
  Serial.print("Vario is on : ");
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

    Serial.println("ERROR -> No detected sensor !");
    Serial.println("Please Check your I2C ADDR, your power and your wires");

    while (1){} // Stop program
  }
  // OK : if sensor is correctly initialise led green and buzzer are plays
  else
  {
    isConfirm(VOL_MAX, TONE_CONFIRM); /* rechange for VOL_MAX /!\*/
    displaySensorDetails();
  }
}


// Displays some basic information on this sensor from the unified
// sensor API sensor_t type (see Adafruit_Sensor for more information)
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


/* Lecture et calcul des variations de pression/température */
void vario()
{
  static int ddsAcc;
  static float pressure, toneFreq, toneFreqLowpass;

  // DEBUG
  // timeTest = micros();

  // Lecture de la valeur de pression du capteur BMP180.
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

  // Filtrage de la valeur sur deux niveaux.
  lowPassFast = lowPassFast + (pressure - lowPassFast) * 0.1;
  lowPassSlow = lowPassSlow + (pressure - lowPassSlow) * 0.07;

  // DEBUG
  // Serial.print("lowPassFast : "); Serial.println(lowPassFast,4);
  // Serial.print("lowPassSlow : "); Serial.println(lowPassSlow,4);

  // On fait la différence des deux valeurs et on lui donne plus de poid.
  toneFreq = (lowPassSlow - lowPassFast) * 50;

  // DEBUG
  // Serial.print("toneFreq: "); Serial.println(toneFreq,4);

  // Filtrage de la nouvelle valeur.
  toneFreqLowpass = toneFreqLowpass + (toneFreq - toneFreqLowpass) * 0.12;

  // DEBUG
  // Serial.print("toneFreqLowpass: "); Serial.println(toneFreqLowpass,4);

  // La valeur est contrainte sur une plage en cas de forte variation.
  toneFreq = constrain(toneFreqLowpass, -400, 500);

  // DEBUG
  // Serial.print("toneFreq: "); Serial.println(toneFreq,4);

  // "ddsAcc" donne une valeur qui permetra d'interrompre le son quelque instant pour faire un bip-bip-bip....
  ddsAcc += toneFreq * 100 + 2000;

  // DEBUG
  // Serial.print("ddsAcc: "); Serial.println(ddsAcc);

  // Play tone depending variation.
  bipSound(toneFreq, ddsAcc);
}


/* Fonction qui gère le son de monté ou de descente. */
void bipSound(int16_t toneFreq, int ddsAcc)
{
  // DEBUG
  //Serial.print("toneFreq: "); Serial.println(toneFreq);

  // Si on descend ou si on reste à une même altitude.
  if (toneFreq < sensibility || ddsAcc > 0)
  {
    // Lorsque "falling" et activé Bip de descente si on dépasse le seuil de descente "MIN_FALL".
    if (falling == true && toneFreq < MIN_FALL)
    {
      toneOn(toneFreq + SOUND_FALL, volume);
      digitalWrite(LED_GOOD, LOW);
      digitalWrite(LED_ERROR, HIGH);
    }
    // Sinon Bip de monté
    else if (falling == true && toneFreq > sensibility)
    {
      toneOn(toneFreq + SOUND_RISE, volume);
      digitalWrite(LED_GOOD, HIGH);
      digitalWrite(LED_ERROR, LOW);
    }
    // et si "g_falling" est désactivée ou si on ne bouge pas, pas de son.
    else
    {
      toneOff();
      digitalWrite(LED_GOOD, LOW);
      digitalWrite(LED_ERROR, LOW);
    }
  }
  // Sinon si l'on monte.
  else
  {
    // Descente activé.
    if (falling == true)
    {
      toneOff();
      digitalWrite(LED_GOOD, LOW);
    }
    // Descente désactivé bip de monté.
    else
    {
      toneOn(toneFreq + SOUND_RISE, volume);
      digitalWrite(LED_GOOD, HIGH);
    }
  }
}


uint8_t getButtons()
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


void setVolume(uint8_t _button)
{
  uint8_t lastVolume = volume;

  if(_button == BTN_UP)
  {
    if(volume < VOL_MAX)
    {
      volumeUpdate(volume++);
      isConfirm(volume, TONE_CONFIRM);
    }
    else isLimit();
  }
  else if(_button == BTN_DOWN)
  {
    if(volume > VOL_MIN)
    {
      volumeUpdate(volume--);
      isConfirm(volume, TONE_CONFIRM);
    }
    else isLimit();
  }

  if(sensibility != lastVolume) EEPROM.write(MEM_VOLUME, volume);
}


void setSensibility(uint8_t _button)
{
  uint8_t lastSensibility = sensibility;

  if(_button == (BTN_SELECT+BTN_UP))
  {
    if(sensibility < MAX_SENS)
    {
      sensibility += STEP_SENS;
      for(uint8_t i ; i<sensibility; i+=STEP_SENS)
      {
        isConfirm(VOL_MAX, TONE_CONFIRM+200);
        delay(50);
      }
    }
    else isLimit();
  }
  else if(_button == (BTN_SELECT+BTN_DOWN))
  {
    if(sensibility > MIN_SENS)
    {
      sensibility -= STEP_SENS;
      for(uint8_t i ; i<sensibility; i+=STEP_SENS)
      {
        isConfirm(VOL_MAX, TONE_CONFIRM+200);
        delay(50);
      }
    }
    else isLimit();
  }

  if(sensibility != lastSensibility) EEPROM.write(MEM_SENS, sensibility);
}


void setMode()
{
  falling =! falling;

  EEPROM.write(MEM_FALL, falling);

  isConfirm(VOL_MAX, TONE_CONFIRM-200);
  delay(2);
  isConfirm(VOL_MAX, TONE_CONFIRM-200);
}


void menuSetting(uint8_t _button)
{
  if(_button == BTN_SELECT || _button == (BTN_SELECT+BTN_UP) || _button == (BTN_SELECT+BTN_DOWN))
  {
    while((_button = getButtons()) == BTN_SELECT ){}

    if(_button == (BTN_SELECT+BTN_UP) || _button == (BTN_SELECT+BTN_DOWN))
    {
      setSensibility(_button);
      delay(DEBOUNCE+400);
    }
    else setMode();
  }
  else
  {
    setVolume(_button);
  }
}
