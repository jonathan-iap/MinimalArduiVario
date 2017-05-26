// Includes -------------------------------------------------------------------
#include <Arduino.h>
#include "defines.h"
#include "NewToneAC.h"

// Defines --------------------------------------------------------------------
#define BAUDRATE 9600
// Buttons State
#define Nb_Of_BTN 3
#define NONE   0
#define UP     1
#define DOWN   2
#define SELECT 3

// Global variables -----------------------------------------------------------
uint8_t buttons[]={BTN_UP, BTN_DOWN, BTN_SELECT};

// Functions declaration-------------------------------------------------------
uint8_t getButtons();

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
}

/******************************************************************************
LOOP
*****************************************************************************/
void loop()
{
  Serial.println(getButtons());
  delay(500);
}

/******************************************************************************
Functions
******************************************************************************/
uint8_t getButtons()
{
  uint8_t btnState = 0;

  for(int i=(Nb_Of_BTN-1); i>=0; i--)
  {
    if(digitalRead(buttons[i]))
    {
      btnState += buttons[i];
    }
  }

  return btnState;
}
