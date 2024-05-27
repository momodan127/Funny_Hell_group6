
//#include <Arduino.h>
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <usart.h>
#include <avr/interrupt.h>

#define ADC_PIN 7

////////////////////////////////// Capacitance code ///////////////////////////////////////////////////////

volatile float timer = 0;
volatile float start_timer = 0;
float real_time = 0;

int button = 0;

float resistor = 10000;
float capacitance = 0;


// function 
uint16_t adc_read(uint8_t adc_channel);
void init_ADC(void);
void init_timer(void);

//arduinotech
int main(void)
{
    

    uart_init();
    io_redirect();

    volatile uint16_t adc_result;
    

    init_ADC();
    init_timer();
    
    sei(); // enable interrupts


    while(1)
    {
        //Turn on power, and start reading the adc.
        DDRB |= (1<<PB4); //enable as output everthing else is input
        PORTB &= 0b11111001; // turn PB4 and PB3 as low
        PORTB |= (1<<PB4); //turn HIGH.

        adc_result = adc_read(ADC_PIN);

          if(adc_result > 647) // thats the volt when above 63.2%
          {
          capacitance = ((timer*11.5)/resistor);
          printf("capacitance %f, \n", capacitance);

          PORTB ^= (1<<PB4); //toggle to LOW
          DDRB |= (1<<PB3); // set as OUTPUT

         while(adc_result > 0)
           {
            adc_result = adc_read(ADC_PIN);
           }
          timer = 0;

          }
        }
        
        }


void init_ADC(void)
{
    // Select Vref = AVcc
    ADMUX |= (1<<REFS0);
    //set prescaler to 128 (125kHz) and turn on the ADC module
    ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN);

}

void init_timer(void)
{
// enable CTC mode timer/0
TCCR0A |= (1<<WGM01);

// set desired target value
OCR0A = 22;  // wait for 23 ticks, for 11,5us setting flag

//start timer, and prescaler 8
TCCR0B |= (0<<CS00) | (1<<CS01);

//enables interrupt for compare match value A for timer/0
TIMSK0 |= (1<<OCIE0A);
}

uint16_t adc_read(uint8_t adc_channel)
  {
  
  ADMUX &= 0xf0; // clear any previously used channel, but keep internal reference

  ADMUX |= adc_channel; // set the desired channel

  ADCSRA |= (1<<ADSC);  //start a conversion
  
  while ( (ADCSRA & (1<<ADSC)) ); // now wait for the conversion to complete
  

  return ADC; // now we have the result, so we return it to the calling function as a 16 bit unsigned int
  }


ISR(TIMER0_COMPA_vect)
{
 timer++;
}
