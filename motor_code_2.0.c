#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h> //here the delay functions are found
#include "usart.h"
#include "i2cmaster.h"
#include "lcd.h"
#include "lm75.h"
#include <stdlib.h>
#include <avr/interrupt.h>
#define ADC_PIN 2 //define Analog Pin for Pressure sensor 
int x=1; //variable to manage motor direction
volatile int mm; //volatile int so it can be changed in interrupt, for counting the millimeters
void motor_stop(void); //function to make motor stop 
void motor_go(void); //function to make motor go
uint16_t adc_read(uint8_t adc_channel); //function prototype analog read

int main (void){
    uart_init();
	io_redirect();

    //setup motor:
    DDRD|=(1<<PD5); //make PD5 output (AIN1)
    DDRD|=(1<<PD4); //make PD4 output (AIN2)
    DDRD|=(1<<PD6); //make PD6 output (PWMA)
    DDRD|=(1<<PD7); //make PD4 output (STBY)

    //setup PD2/INT0 for Opotcoupler
    DDRD &= ~(1 << DDD2);     // Clear the PD2 pin// PD2 (INT0 pin) is now an input
    PORTD |= (1 << PORTD2);    // turn On the Pull-up// PD2 is now an input with pull-up enabled (MIGHT NOT BE NEEDED)
    EICRA|=(1<<ISC01)|(1<<ISC00); // set INT0 to trigger on ANY logic change
    EIMSK|=(1<<INT0); // Turns on interrupt for INT0
    sei();//enables interrupt in Status_Register
    
    //setup sensor/analog read
    uint16_t adc_result;
    ADMUX = (1<<REFS0);// Select Vref = AVcc
    ADCSRA = (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN);//set prescaler to 128 and turn on the ADC module
   
while (1) {   
    motor_go();
    while(mm<256){ //if ruler is extracted 2cm make motor stop
        //wait until ruler is extended
        printf("%d\n",mm); //print the mm (testing purposes)
    }
    motor_stop();
    motor_go();
    do{
        adc_result = adc_read(ADC_PIN);//read input on A2
        printf("Sensor::%d\n",adc_result); //print sensor result
    }while(adc_result<20);//if sensor detects more than 20 -make motor stop
    motor_stop();
    int measurement=256-mm; //calculate measurement by subztracting distance travelled from total distance
    printf("Measurement:%d\n",measurement); //print measurement
    _delay_ms(20000000);//wait till it starts again so measurements can be read*/
}
return(0);
}
           
ISR(INT0_vect)//external interrupt 0
{
    mm++; //increase mm with every logic change(ruler)
}

void motor_stop(void){
    mm=0; //reset mm counter
    OCR0A=0;//set OCR0A to 0 to stop PWM Signal
    _delay_ms(2000);
}

void motor_go(void){
    
    if(x==1){
        PORTD|=(1<<PD7); //set STBY high so that motor goes
        PORTD|=(1<<PD5); //sets AIN1 high
        PORTD|=(0<<PD4); //set AIN2 low so that motor spins CCW
        TCCR0A|=(1<<WGM00)|(1<<COM0A1)|(0<<COM0A0); // Configure Timer/Counter0 for phase correct PWM, Clear OC0A on Compare Match when up-counting, set OC0A on Compare Match when down-counting
        TCCR0B|=(1<<CS01); //presacler to 8
        OCR0A=0; //value range from 0 to 255
         x--; //decrease x so next time it jumps into else
        } 

       else {
        PORTD^=(1<<PD5); //sets AIN1 low
        PORTD^=(1<<PD4); //sets AIN2 high so that motor spins CW
        TCCR0A|=(1<<WGM00)|(1<<COM0A1)|(0<<COM0A0); // Configure Timer/Counter0 for phase correct PWM, Clear OC0A on Compare Match when up-counting, set OC0A on Compare Match when down-counting
        TCCR0B|=(1<<CS01); //presacler to 8
        OCR0A=0; //value range from 0 to 255
       }
}
 
uint16_t adc_read(uint8_t adc_channel){ //function to read Analog input channel
       ADMUX &= 0xf0; // clear any previously used channel, but keep internal reference
       ADMUX |= adc_channel; // set the desired channel
       ADCSRA |= (1<<ADSC);//start a conversion
       while ( (ADCSRA & (1<<ADSC)) );// now wait for the conversion to complete
return ADC; //return result
}