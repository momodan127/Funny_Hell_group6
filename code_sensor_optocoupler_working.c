#define F_CPU 16000000UL

#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h> //here the delay functions are found
#include "usart.h"
#include "i2cmaster.h"
#include "lcd.h"
#include "lm75.h"
#include "stdlib.h"
#include <avr/interrupt.h>
#define ADC_PIN 2 //define Analog Pin for Pressure sensor 

volatile int mm; //volatile int so it can be changed in interrupt, for counting the millimeters
void motor_stop(void); //function to make motor stop and reverse directions
uint16_t adc_read(uint8_t adc_channel); //function prototype for analog read

int main (void){
    uart_init();
	io_redirect();
   
//setup motor:
    DDRB|=(1<<PB1); //make PB1 output (PWMA)
    DDRD|=(1<<PD6); //make PD6 output (AIN2)
    DDRD|=(1<<PD5); //make PD5 output (AIN1)
    DDRD|=(1<<PD4); //make PD4 output (STBY)*///
    
//setup PD2/INT0 for Opotcoupler
    DDRD &= ~(1 << DDD2);     // Clear the PD2 pin// PD2 (INT0 pin) is now an input
    PORTD |= (1 << PORTD2);    // turn On the Pull-up// PD2 is now an input with pull-up enabled (MIGHT NOT BE NEEDED)
    EICRA|=(1<<ISC00); // set INT0 to trigger on ANY logic change
    EIMSK|=(1<<INT0); // Turns on interrupt for INT0
    sei();//enables interrupt in Status_Register
    
//setup sensor/analog read
    uint16_t adc_result; 
    ADMUX = (1<<REFS0);// Select Vref = AVcc
    ADCSRA = (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN);//set prescaler to 128 and turn on the ADC module
          

while (1) {
    PORTD|=(1<<PD4); //set STBY high so that motor goes
    PORTD|=(1<<PD5) |(0<<PD6);//sets AIN1 high and AIN2 low so that motor spins CW
    PORTB|=(1<<PB1);
    
    //printf("%d\n",mm); //print the mm (testing purposes)
    if(mm==200){ //if ruler is extracted 20cm make motor stop
         motor_stop();
         mm=0;    
    }
    adc_result = adc_read(ADC_PIN);
    if(adc_result>200){ //if sensor detects more than 200 -make motor stop
        motor_stop();
     }
}
return 0;
}
           
 ISR(INT0_vect)//external interrupt 0
{
    mm++; //increase mm with every logic change(ruler)
}

void motor_stop(void){
    printf("stop\n");
    PORTD|=(0<<PD5) |(0<<PD6); //sets AIN1 and AIN2 to low so that motor stops
}
 
uint16_t adc_read(uint8_t adc_channel){ //function to read Analog input channel
       ADMUX &= 0xf0; // clear any previously used channel, but keep internal reference
       ADMUX |= adc_channel; // set the desired channel
       //start a conversion
       ADCSRA |= (1<<ADSC);
       // now wait for the conversion to complete
       while ( (ADCSRA & (1<<ADSC)) );
       // now we have the result, so we return it to the callingfunction as a 16 bit unsigned int

return ADC;
}
   

        
       
    