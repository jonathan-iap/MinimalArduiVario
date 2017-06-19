#ifndef LIB_POWER_H_
#define LIB_POWER_H_

#include <avr/power.h>
#include <avr/sleep.h>
#include <Arduino.h>
#include "defines.h"

#define PINS_ON_UC 19 // ATmega328P

#define	WDT_30MS   0
#define	WDT_60MS   1
#define	WDT_120MS  2
#define	WDT_250MS  3
#define	WDT_500MS  4
#define	WDT_1S     5
#define	WDT_2S     6
#define	WDT_4S     7
#define	WDT_8S     8

void WDT_Off(void);
void WDT_On(uint8_t timeOut);
void disableDevices(void);
void enableDevices(void);
void init_PowerSaving(void);
void sleeping(uint8_t duration);



#endif
