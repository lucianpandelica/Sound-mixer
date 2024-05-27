#include <stdio.h>
#include <avr/io.h>

#include "adc.h"

void adc_init(void)
{
    //(pg. 260) Set ADC prescaler to 128 (don't need higher precision for this project)
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

    //(pg. 258) Set ADC reference voltage to AVCC
    ADMUX |= (1 << REFS0);

    //(pg. 260) Enable ADC
    ADCSRA |= (1 << ADEN);
}

uint16_t myAnalogRead(uint8_t channel)
{
    // Force input channel to be between 0 and 7 (as ADC pins are PA0-7)
    //channel &= 0b00000111;
    channel &= 0b00000000;
    // TODO [task0]:
    // 1. Clear the old channel value (if any, last 5 bits in ADMUX)
    ADMUX &= ~(0b00011111);
    // 2. Select the new channel in ADMUX
    ADMUX |= channel;
    // 3. Start single conversion
    ADCSRA |= (1 << ADSC);
    // 4. Busy wait for conversion to complete
    while (bit_is_clear(ADCSRA, ADIF))
    ;
    // Return ADC value
    return (ADC);
}