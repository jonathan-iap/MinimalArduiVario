#include "PowerSaving.h"

// User functions
/******************************************************************************
Description : Turn "Off" Watchdog
******************************************************************************/
void WDT_Off(void)
{
  noInterrupts(); // Disable interrupts

  // Clear reset flag
  MCUSR &= ~(1 << WDRF);
  // Keep old prescaler setting to prevent unintentional time-out
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  // Turn off WDT
  WDTCSR = 0x00;

  interrupts(); // Enable interrupts
}


/******************************************************************************
Description : Turn "On" Watchdog
******************************************************************************/
void WDT_On(uint8_t timeOut)
{
  noInterrupts(); // Disable interrupts

  // Clear reset flag
  MCUSR &= ~(1 << WDRF);

  // Keep old prescaler setting to prevent unintentional time-out
  WDTCSR |= (1 << WDCE) | (1 << WDE);

  // set new watchdog timeout prescaler value
  switch (timeOut)
  {
    case WDT_30MS :
    {
      WDTCSR = (1<<WDE) | (1<<WDP0); // 32 Millisecondes
      break;
    }
    case WDT_60MS :
    {
      WDTCSR = (1<<WDE) | (1<<WDP1); // 64 Millisecondes
      break;
    }
    case WDT_120MS :
    {
      WDTCSR = (1<<WDE) | (1<<WDP1) | (1<<WDP0); // 125 Millisecondes
      break;
    }
    case WDT_250MS :
    {
      WDTCSR = (1<<WDE) | (1<<WDP2); // 250 Millisecondes
      break;
    }
    case WDT_500MS :
    {
      WDTCSR = (1<<WDE) | (1<<WDP2) | (1<<WDP0); // 500 Millisecondes
      break;
    }
    case WDT_1S :
    {
      WDTCSR = (1<<WDE) | (1<<WDP2) | (1<<WDP1); // 1 second
      break;
    }
    case WDT_2S :
    {
      WDTCSR = (1<<WDE) | (1<<WDP2) | (1<<WDP1) | (1<<WDP0); // 2 seconds
      break;
    }
    case WDT_4S :
    {
      WDTCSR = (1<<WDE) | (1<<WDP3); // 4 seconds
      break;
    }
    case WDT_8S :
    {
      WDTCSR = (1<<WDE) | (1<<WDP1) | (1<<WDP0); // 8 seconds
      break;
    }
  }

  // Enable the WD interrupt
  WDTCSR |= (1<<WDIE);

  interrupts();   // Enable interrupts

}


/******************************************************************************
Description : Disable devices that are not used to reduce power consumption.
Power Reduction Register (PRR) functions from avr/power.h library.
For every disable function there is an enable function
power_adc_disable() or power_adc_enable()
power_spi_disable() or power_spi_enable()
power_timer0_disable() or power_timer0_enable()
power_timer1_disable() or power_timer1_enable()
power_timer2_disable() or power_timer2_enable()
power_twi_disable() or power_twi_enable()
power_all_disable() or power_all_enable()
power_usart0_disable() or power_usart0_enable()

Note that for max power efficiency you should also disable the rest of the module for ADC and SPI
SPCR = 0; //disable SPI
ADCSRA = 0;  // disable ADC
******************************************************************************/
void disableDevices(void)
{
  noInterrupts(); // Disable interrupts

  // Disable the clock to the ADC module
  ADCSRA = 0;
  power_adc_disable();
  // Disable the clock to the SPI module
  SPCR = 0;
  power_spi_disable();
  // Disable Timer2, enable if we want to use tone()
  power_timer2_disable();
  #ifndef DEBUG
  // Disable serial communication
  power_usart0_disable();
  #endif

  interrupts(); // Disable interrupts
}

/******************************************************************************
Description : Same thing that "disableDevices" but in inverse
******************************************************************************/
void enableDevices(void)
{
  noInterrupts();

  power_timer0_enable();
  power_timer1_enable();
  power_twi_enable();
  #ifdef DEBUG
  power_usart0_enable();
  #endif

  interrupts();
}

/******************************************************************************
Description : To be used first before any code !
******************************************************************************/
void init_PowerSaving(void)
{
  // Set all pins as input and active pull-up
  for(uint8_t i = PINS_ON_UC ; i>0 ; i--)
  {
    pinMode(i, INPUT_PULLUP);
  }

  // Turn off all devices that are not used
  disableDevices();
}

/******************************************************************************
Description: Enters the arduino into sleep mode.
The 5 different modes are:
SLEEP_MODE_IDLE
SLEEP_MODE_ADC
SLEEP_MODE_PWR_SAVE
SLEEP_MODE_STANDBY
SLEEP_MODE_PWR_DOWN
******************************************************************************/
void sleeping(uint8_t duration)
{
  //DEBUG
  // Serial.println("Save Mode...");
  // delay(10);

  noInterrupts();

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();

  WDT_On(duration); // Enable watchdog

  interrupts();

  // Now enter sleep mode.
  sleep_cpu();

  // sleep and wait watchdog interrupt ........................................
  // The program will continue from here after the WDT timeout.................

  sleep_disable(); // First thing to do is disable sleep.
  WDT_Off(); // Disable watchdog

  // Re-enable the peripherals.
  enableDevices();

  //DEBUG
  // Serial.println("Wake-up !");
}


// Private functions
/******************************************************************************
Description: Watchdog Interrupt Service. This is executed when
watchdog timed out.
In this case do nothing. Watchdog is used to Wake-up the mcu.
******************************************************************************/
ISR(WDT_vect)
{
}
