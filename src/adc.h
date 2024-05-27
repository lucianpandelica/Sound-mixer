#ifndef ADC_H_
#define ADC_H_

#include <stdio.h>

void adc_init(void);
uint16_t myAnalogRead(uint8_t channel);

#endif // ADC_H_
