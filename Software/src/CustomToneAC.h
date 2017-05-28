/*
 * CustomToneAC.h
 *
 *  Created on: 22 avr. 2017
 *      Author: Jonathan Iapicco
 */

#ifndef LIB_CUSTOMTONEAC_NEWTONEAC_H_
#define LIB_CUSTOMTONEAC_NEWTONEAC_H_

#include "Arduino.h"

#define T1_PRESCALER 1

#if F_CPU == 16000000
#define PWM_FREQ 79 // Around 200KHz
#elif F_CPU == 8000000
#define PWM_FREQ 56 // Around 140KHz
#endif

#define VOL_MAX 10
#define VOL_MIN 0

// Private functions
void setTimer();
// User functions
void toneOn(uint16_t frequency);
void toneOn(uint16_t frequency,  uint8_t volume);
bool toneOn(uint16_t frequency,  uint8_t volume, uint32_t length, bool openLoop);
void volumeUpdate(uint8_t volume);
void toneOff();


#endif /* LIB_NEWTONEAC_NEWTONEAC_H_ */
