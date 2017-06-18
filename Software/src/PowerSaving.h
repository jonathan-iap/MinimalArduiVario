#ifndef LIB_POWER_H_
#define LIB_POWER_H_

#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <Arduino.h>
#include "defines.h"

#define PINS_ON_UC 19 // ATmega328P

void WDT_Off(void);
void WDT_On(void);
void disableDevices(void);
void enableDevices(void);
void init_PowerSaving(void);
void sleeping(void);



#endif
