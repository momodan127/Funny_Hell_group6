
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
#define ADC_PIN 2 //define Analog Pin for Pressure sensor 
int x=1;
volatile int mm; //volatile int so it can be changed in interrupt, for counting the millimeters
void motor_stop(void); //function to make motor stop and reverse directions
void motor_go(void);
uint16_t adc_read(uint8_t adc_channel); //function prototype analog read

int main (void){
    uart_init();
	io_redirect();

    //setup motor:
    DDRB|=(1<<PB1); //make PB1 output (PWMA)
    DDRD|=(1<<PD6); //make PD6 output (AIN2)
    DDRD|=(1<<PD5); //make PD5 output (AIN1)
    DDRD|=(1<<PD4); //make PD4 output (STBY)

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

    //setup timer for PWM control
    /*TCCR1A |= (1 << WGM10) | (1 << COM1A1);// Configure Timer/Counter1 for phase correct PWM, Clear OC1A on Compare Match when up-counting, set OC1A on Compare Match when down-counting
    TCCR1B |= (1 << CS11) | (0 << CS10); // Set the prescaler to 8
    OCR1A = 100; //value range from 0 to 255*/
   


while (1) {
    motor_go();
    
   
    while(mm<20){ //if ruler is extracted 2cm make motor stop
        //wait until ruler is extended
        printf("%d\n",mm); //print the mm (testing purposes)
    }
    
    motor_stop();
    motor_go();
   
    do{
        adc_result = adc_read(ADC_PIN);//read input on A2
        printf("Sensor:%d\n",adc_result);
    }while(adc_result<20);//if sensor detects more than 20 -make motor stop
      
        
    
    OCR1A=0;
    int measurement=200-mm;
    printf("measurement:%d\n",measurement);
   _delay_ms(20000000);//wait till it starts again so measurements can be read
   
}
return(0);
}
           
ISR(INT0_vect)//external interrupt 0
{
    mm++; //increase mm with every logic change(ruler)
    
}

void motor_stop(void){
    mm=0;
    OCR1A=0;
    printf("stop\n"); //testing pruposes
   _delay_ms(5000);
}
void motor_go(void){
    
    
    if(x==1){
        PORTD|=(1<<PD4); //set STBY high so that motor goes
        PORTD|=(1<<PD5); //sets AIN1 high
        PORTD|=(0<<PD6);//set AIN2 low so that motor spins CCW
        x--;
        TCCR1A |= (1 << WGM10) | (1 << COM1A1);// Configure Timer/Counter1 for phase correct PWM, Clear OC1A on Compare Match when up-counting, set OC1A on Compare Match when down-counting
        TCCR1B |= (1 << CS11) | (0 << CS10); // Set the prescaler to 8
        OCR1A = 70; //value range from 0 to 255
        } 

       else {
            if(TCNT1==1023){
            PORTD|=(1<<PD4); //set STBY high so that motor goes
            PORTD|=(0<<PD5); //sets AIN1 low
            PORTD|=(1<<PD6);//set AIN2 high so that motor spins CW
             //setup timer for PWM control
            TCCR1A |= (1 << WGM10) | (1 << COM1A1);// Configure Timer/Counter1 for phase correct PWM, Clear OC1A on Compare Match when up-counting, set OC1A on Compare Match when down-counting
            TCCR1B |= (1 << CS11) | (0 << CS10); // Set the prescaler to 8
            OCR1A = 70; //value range from 0 to 255
        }}
    
    
    printf("go\n");
   
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

//test PWM signal
/*#define ADC_PIN 2
uint16_t adc_read(uint8_t adc_channel);

int main (void){
    uart_init();
	io_redirect();
    //setup motor:
    //DDRB|=(1<<PB1); //make PB1 output (PWMA)
  
    //setup sensor/analog read
    uint16_t adc_result; 
    ADMUX = (1<<REFS0);// Select Vref = AVcc
    ADCSRA = (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN);//set prescaler to 128 and turn on the ADC module
    //setup PWM signal
    //TCCR1A |= (1 << WGM10) | (1 << COM1A1);// Configure Timer/Counter1 for phase correct PWM, Clear OC1A on Compare Match when up-counting, set OC1A on Compare Match when down-counting
    //TCCR1B |= (1 << CS12) | (1 << CS10); // Set the prescaler to 1024
    //OCR1A = 255; //value range from 0 to 255
 
while (1) {
    adc_result = adc_read(ADC_PIN);
    printf("%d\n",adc_result);
     
}
return 0;
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
}*/
/*#define ADC_PIN 0

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
}*/


	




