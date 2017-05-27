// Includes -------------------------------------------------------------------
#include <Arduino.h>
#include "defines.h"
#include "NewToneAC.h"

// Defines --------------------------------------------------------------------
#define BAUDRATE 9600
#define Nb_Of_BTN 3

// Global variables -----------------------------------------------------------
uint8_t buttons[]={BTN_UP, BTN_DOWN, BTN_SELECT};
uint8_t volume;

// Functions declaration-------------------------------------------------------
uint8_t getButtons();
void setVolume(uint8_t _button);
bool isLimit();
bool isConfirm(uint8_t _volume);

/******************************************************************************
SETUP
*****************************************************************************/
void setup()
{
  // Leds
  pinMode(LED_GOOD, OUTPUT);
  pinMode(LED_ERROR, OUTPUT);
  // Buttons
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_SELECT, INPUT_PULLUP);
  // i2C
  //Wire.setClock(400000); // change i2c speed to 400Khz

  // Debug
  Serial.begin(BAUDRATE);
  volume = 1;

}

/******************************************************************************
LOOP
*****************************************************************************/
void loop()
{
   digitalWrite(LED_ERROR, HIGH);
   delay(100);
   digitalWrite(LED_ERROR, LOW);
   delay(100);
}

/******************************************************************************
Functions
******************************************************************************/
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
  if(_button == BTN_UP)
  {
    if(volume < VOL_MAX)
    {
      volumeUpdate(volume++);
      isConfirm(volume);
    }
    else isLimit();
  }
  else if(_button == BTN_DOWN)
  {
    if(volume > VOL_MIN)
    {
      volumeUpdate(volume--);
      isConfirm(volume);
    }
    else isLimit();
  }
}


bool isLimit()
{
  toneOff();
  digitalWrite(LED_GOOD, LOW);

  digitalWrite(LED_ERROR, HIGH);
  toneOn(TONE_LIMIT, VOL_MAX, 1000, LOW);
  digitalWrite(LED_ERROR, LOW);

  return true;
}


bool isConfirm(uint8_t _volume)
{
  toneOff();
  digitalWrite(LED_ERROR, LOW);

  digitalWrite(LED_GOOD, HIGH);
  toneOn(TONE_CONFIRM, _volume, 400, LOW);
  digitalWrite(LED_GOOD, LOW);

  return true;
}
