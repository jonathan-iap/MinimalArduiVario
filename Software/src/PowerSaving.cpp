#include "PowerSaving.h"

/* Power Reduction Register (PRR) functions from avr/power.h library.
For every disable function there is an enable function
power_adc_disable() or power_adc_enable()
power_spi_disable() or power_spi_enable()
power_timer0_disable() or power_timer0_enable()
power_timer1_disable() or power_timer1_enable()
power_timer2_disable() or power_timer2_enable()
power_twi_disable() or power_twi_enable()
power_all_enable() or power_all_disable()

Note that for max power efficiency you should also disable the rest of the module for ADC and SPI
SPCR = 0; //disable SPI
ADCSRA = 0;  // disable ADC
*/

void setPowerSaving(void)
{
  for(uint8_t i = MAX_PINS_UC ; i>0 ; i--)
  {
    pinMode(i, INPUT_PULLUP);
  }
  ADCSRA = 0;  // disable ADC by setting ADCSRA register to 0
  power_adc_disable(); //disable the clock to the ADC module

  SPCR = 0; //disable SPI by setting SPCR register to 0
  power_spi_disable(); //disable the clock to the SPI module

  power_timer2_disable(); // disable Timer2, enable if we want to use tone()
   delay(5000); //Delay to see normal power level first
}
