
#define F_CPU 16000000UL

#include <stdint.h>
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h> //here the delay functions are found
#include "usart.h"
#include "i2cmaster.h"
#include "lcd.h"
#include "lm75.h"
#include "stdlib.h"
#include <avr/interrupt.h>
#define ADC_PIN 0

void stop_motor_measurement(void);
int ticks;
float measurement;
int main (void) {
    
    uart_init();
	io_redirect();
   	uint16_t adc_result;

	uint16_t adc_result;
         // Select Vref = AVcc
         ADMUX = (1<<REFS0);
         //set prescaler to 128 and turn on the ADC module
		ADCSRA = (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN); 


	DDRB = 0x00; //I/O board:4...7 as outputs, for LEDs
	PORTB=0xFF; //enabling pullups
	DDRC = 0xF0; //I/O board PC0...3 as inputs, for buttons		
	PORTC = 0x3F; //Enable internal pull at PC 0...3 inputs
	DDRD&=~(1<<DDD4);//clear PD4 Pin PD4 is now an input
	TCCR0B|=(1<<CS02)|(1<<CS01)|(1<<CS00);//Turn on the counter clock on rise
	TCNT0=0;
	OCR0A=0;

	while(1) {
		if (PINC==0b00110111){ //which button should I use?
			OCR0A=50; //make motor go - which speed?
		}
		if (TCNT0==200){ //when optocoupler has counted 200 falling edges? so when it has gone 20cm 
			OCR0A=0; //make motor stop
			TCNT0=0; //reset the counter value
			OCR0A=-50; //make motor go other direction
		}
		adc_result = adc_read(ADC_PIN); //read input of sensor
		if (adc_result>3){ //if sensor input has had a big enough change
			stop_motor_measurement(); //motor gets stopped and measurement is taken
		}



		
	}

	return 0;
}

uint16_t adc_read(uint8_t adc_channel){
       ADMUX &= 0xf0; // clear any previously used channel, but keep internal reference
       ADMUX |= adc_channel; // set the desired channel
       //start a conversion
       ADCSRA |= (1<<ADSC);
       // now wait for the conversion to complete
       while ( (ADCSRA & (1<<ADSC)) );
       // now we have the result, so we return it to the calling function as a 16 bit unsigned int
}

void stop_motor_measurement(){
	OCR0A=0; //stop motor
	ticks=TCNT0;
	measurement=200.00-(TCNT0/10.00); //calculate measurement in cm
	printf("Measurement:%dcm\n",measurement); //print the measurement
	_delay_ms(5000); //wait a bit so that measurement can be read from ruler as well
	TCNT0=0; //reset counter value
	OCR0A=50; //make motor go out again so that object can be taken out
	if(TCNT0==25){ //when it has gone 2.5cm 
		OCR0A=0; //make motor stop
		TCNT0=0;//reset counter value
		OCR0A=-50;//make motor go back in
	}
	if((measurement+TCNT0-25)==200){ //if it is ffully retracted again which is measured by distance
		OCR0A=0; //make mnotor stop
	}
}


	




