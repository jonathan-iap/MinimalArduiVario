/*
* Configuration file.
*
* Created on: 2 avr. 2016
*     Author: Jonathan Iapicco
*/

#ifndef Define_h
#define Define_h

// EEprom memory.
#define MEM_VOLUME  10
#define MEM_SENS    11
#define MEM_FALL    12

// sensibility setting
#define COEF_FAST     0.09   // Coefficient for filter low past fast
#define COEF_SLOW     0.06  // Coefficient for filter
#define COEF_LOWPASS  0.12  // Coefficient for low pass tone
#define MAX_SENS      40    // Max trigger threshold for clim
#define MIN_SENS      10    // Min trigger threshold for clim
#define MIN_FALL      -30   // Min trigger threshold for fall
#define STEP_SENS     10    // Step to set threshold

// Tone
#define SOUND_RISE 500  // Clim tone
#define SOUND_FALL 400  // fall tone

// Tone frequency
#define TONE_LIMIT    1400
#define TONE_CONFIRM  1100

// Debouncing time
#define DEBOUNCE  200

// Buttons - pins
#define BTN_UP        A1
#define BTN_DOWN      A2
#define BTN_SELECT    A3
#define Nb_Of_BTN 3

// Leds - pins
#define LED_GOOD  A0 // Green
#define LED_ERROR 13 // Red

#endif
