
//#include <Arduino.h>
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <usart.h>
#include <avr/interrupt.h>


////////////////////////////////// Serial ports (USART and I2C) ///////////////////////////////////////////////////////

volatile float timer_flag = 0;
int voltage_reference = 648; // this would be when it has to stop.
float resistor = 10000;
float capacitance = 0;
int test = 0;

//arduinotech
int main(void)
{

    uart_init();
    io_redirect();

    TCCR0A |= (1<<WGM01);
    OCR0A = 9; //wait for 10 ticks... 5us
    TCCR0B |= (1<<CS01); //prescaler 8

    ADMUX = ADMUX | 0x40; // read from PINC0 otherwise known as A0:
    ADCSRB = ADCSRB & (0xF8); //free running mode 
    ADCSRA = ADCSRA | 0xE7; 

    TIMSK0 |= (1<<OCIE0A);
    sei(); // enable interrupts
    PINB = 0x10;

    while(1)
    {
        printf("C %f\n", capacitance);
        printf("delay %d\n", test);
    }

}

ISR(TIMER0_COMPA_vect)
{
    timer_flag++;

    float adc_low = ADCL;
    float adc_high = (adc_low + ((ADCH & 0x03) << 8));

    if(adc_high > voltage_reference)
    {
        test = timer_flag;
        capacitance =1000000*((timer_flag*5)/resistor);
        timer_flag = 0;
    }
}



