/*
* Frequency range : 1Hz to 30KHz
* Range depend to PWM freq
* Pins are fix, you need to use pin 10 and 9 on ATmega 168 an 328.
*
* Created on: 22 avr. 2017
*     Author: Jonathan Iapicco
*/

#include "CustomToneAC.h"
#include <avr/io.h>

// Convert the frequency into a value for setting timer1
#define FREQ_TO_ISR(freq) ((F_CPU / (T1_PRESCALER * (PWM_FREQ + 1))) / (2 * (freq)) -1)
// Give the time to make one periode at the pwm frequency
#define PWM_FREQ_TO_TIME (1000.0 / (F_CPU / (T1_PRESCALER * (PWM_FREQ + 1.0))))

volatile bool _toneFlipFlop = true; // Switch port output
volatile uint16_t _toneISR = 0; // Divide timer to create audible frequency
volatile uint16_t _counterISR = 0; // Counter for toneISR
bool _isFirstCall = true;


// User functions
/******************************************************************************
* Description : Plays tone as long as the function "toneOff()" is not called.
* Frequency is update when function is recall.
* ----------------------------------------------------------------------------
* frequency in Hertz : 1 -> 30000 Hz
*****************************************************************************/
void toneOn(uint16_t frequency)
{
  volumeUpdate(VOL_MAX);

  _toneISR = FREQ_TO_ISR(frequency);

  if(_isFirstCall)
  {
    setTimer();
    _isFirstCall = false;
  }
}


/******************************************************************************
* Description : Like toneOn(frequency) but with volume.
* Plays the note as long as the function "toneOff()" is not called.
* You can update volume, if you recall this function or you can be use directly
* the "volumeUpdate()" function.
* ----------------------------------------------------------------------------
* frequency in Hertz : 1 -> 30000 Hz
* volume : 0 -> 10
*****************************************************************************/
void toneOn(uint16_t frequency,  uint8_t volume)
{
  volumeUpdate(volume);

  if(volume == 0)
  {
    toneOff();
  }
  else
  {
    _toneISR = FREQ_TO_ISR(frequency);

    if(_isFirstCall)
    {
      setTimer();
      _isFirstCall = false;
    }
  }
}


/******************************************************************************
* Description : Plays the note depending length time.
* It's kill tone when time is reach.
* If "openLoop" is to "LOW" this function behaves like a blocking
* function "delay".
* If "openLoop" is to "HIGH" you can do several things at once. But you need
* to call this function as often as possible.
* Volume is automatically update or you can be use directly
* the "volumeUpdate()" function.
* Return "true" if time is reach.
*-----------------------------------------------------------------------------
* frequency in Hertz : 1 -> 30000 Hz
* volume : 0 -> 10
* length in Milliseconde : 0 -> 214748364 ms
*****************************************************************************/
bool toneOn(uint16_t frequency,  uint8_t volume, uint32_t length, bool openLoop)
{
  static uint32_t lengthMillis = 0;

  volumeUpdate(volume);

  if(volume == 0 || length == 0)
  {
    toneOff();
    _isFirstCall = true;
    return true;
  }
  else
  {
    if(_isFirstCall)
    {
      _isFirstCall = false;

      _toneISR = FREQ_TO_ISR(frequency); // Set tone frequency
      setTimer(); // Start timer routine

      lengthMillis = millis() + length; // set time

      if(!openLoop) // Blocking loop stop program
      {
        while(millis() < lengthMillis) // do nothing untils time isn't reached.
        {}

          toneOff();
          _isFirstCall = true;
          return true;
        }
      }
      else if(millis() > lengthMillis) // Open loop, program can continue
      {
        toneOff();
        _isFirstCall = true;
        return true;
      }
    }
    return false;
  }

  /******************************************************************************
  * Description : Update the volume
  *-----------------------------------------------------------------------------
  * volume : 0 -> 10
  *****************************************************************************/
  void volumeUpdate(uint8_t volume)
  {
    // Set PWM
    OCR1A = map(volume, 0, 10, 1, PWM_FREQ);
    OCR1B = OCR1A;
  }

  /******************************************************************************
  * Description : Stop tone.
  *****************************************************************************/
  void toneOff()
  {
    //For restart timer
    _isFirstCall = true;

    // Stop ISR routine
    noInterrupts();

    // Clear registers
    TCCR1A = 0;
    TCCR1B = 0;
    TCCR1C = 0;

    TIMSK1 = 0; // T1 disable

    interrupts();

    PORTB &= ~((1<<PORTB1) | (1<<PORTB2)); // Pin 9 & 10 to LOW
  }


  // Private Functions
  /******************************************************************************
  * Description : Engine of library. Set timer 1 and pins.
  *
  *   |<------Audible frequency---->|        {  }<-PWM at 200Kz give volume
  *   || || || || ||                || || || || ||
  * __||_||_||_||_||________________||_||_||_||_||____________ Pin 9 (portB1, oc1A)
  *
  *
  *                  || || || || ||                || || || || ||
  *__________________||_||_||_||_||________________||_||_||_||_||_______ Pin10 (portB2, oc1B)
  *
  *...|..|..|..|..|..|..|..|..|..|..|..|..|..|..|..|..|...-> ISR call at the PWM frequency
  *
  * Timer 1 is set to produce a fastPWM at 200Khz.
  * An ISR is attached and divide the main frequency to produce a lower frequency
  * that can be audible.
  * Pin 9 and 10 are 90° out off phase to produce voltage doubling.
  *
  * This library cannot produce high audible frequency is not the goal.
  * But it can be little increased by changing the value of #define PWM_FREQ.
  * 200Kz is a stable value without side effect.
  *****************************************************************************/
  void setTimer()
  {
    // Set pin 9 & 10 as OUTPUT (need for PWM)
    DDRB |= ((1 << DDB2) | (1 << DDB1));
    // Set pin 9 & 10 to LOW
    PORTB &= ~((1 << PORTB2) | (1 << PORTB1));

    //------------------------------------------
    // TIMER1 setting :
    //------------------------------------------
    noInterrupts(); // Disable interrupts

    // Clear register
    TCCR1A = 0;
    TCCR1B = 0;
    TCCR1C = 0;
    // Set registers : Fast PWM , ICR1 as TOP
    TCCR1A |= (1 << WGM11);
    TCCR1B |= (1 << WGM13) | (1 << WGM12);
    // Prescaler : x1
    TCCR1B |= (1 << CS10);
    // TOP frequency
    ICR1 = PWM_FREQ;
    // First use :
    TCCR1A |= (1 << COM1A1);  // Turn ON PWM on OC1B pin 9
    TCCR1A &= ~(1 << COM1B1); // Turn OFF PWM on OC1B pin 10
    _toneFlipFlop = true;     // Set counter
    _counterISR = _toneISR;          //
    // Enable ISR
    TIMSK1 |= (1 << TOIE1);

    interrupts(); // Enable interrupts
  }


  /******************************************************************************
  * Interrupt routine : Turn on and off the PWM on pin 9 and 10 to produce
  * a 90° out off phase and give an audible frequency.
  * If you turn PWM frequency to high this interrupt is not performed correctly.
  *****************************************************************************/
  ISR(TIMER1_OVF_vect)
  {
    // noInterrupts();

    if(_counterISR == 0)
    {
      if(_toneFlipFlop)
      {
        TCCR1A |= (1 << COM1B1);  // Turn ON PWM on OC1B pin 10
        TCCR1A &= ~(1 << COM1A1); // Turn OFF PWM on OC1A pin 9
        _toneFlipFlop = false;
      }
      else
      {
        TCCR1A |= (1 << COM1A1);  // Turn ON PWM on OC1B pin 9
        TCCR1A &= ~(1 << COM1B1); // Turn OFF PWM on OC1B pin 10
        _toneFlipFlop = true;
      }
      _counterISR = _toneISR;
    }
    else _counterISR--;

    // interrupts();
  }
