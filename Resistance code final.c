
//#include <Arduino.h>
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <usart.h>
#include <avr/interrupt.h>
#define V_ref 5.0
#define ADC_PIN 0

////////////////////////////////// Serial ports (USART and I2C) ///////////////////////////////////////////////////////

volatile float timer_flag = 0;
int voltage_reference = 648; // this would be when it has to stop.
float resistor = 10000;
float capacitance = 0;
int test = 0;

void Resistance(void);
uint16_t adc_read(uint8_t adc_channel);

//arduinotech
int main(void)
{

    uart_init();
    io_redirect();

    TCCR0A |= (1<<WGM01);
    OCR0A = 9; //wait for 10 ticks... 5us
    TCCR0B |= (1<<CS01); //prescaler 8


    ADMUX = (1<<REFS0);
    ADCSRA = (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN);



    TIMSK0 |= (1<<OCIE0A);
    sei(); // enable interrupts
    PINB = 0x10;
    
    while(1)
    {
        printf("C %f\n", capacitance);
        printf("delay %d\n", test);
        Resistance();
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

void Resistance(void){
DDRC = 0xF0;//0b11110000, four outputs and four inputs (buttons)
PORTC = 0x3F;
DDRD = 0x04;//resistors (channels)
PORTD = 0x04;
    uint16_t V_measured;
	float Vout = 0;
	float R2 = 0;
	int buffer = 0;
//When we change the scale all we need to do is change the desired scale pin as OUTPUT with LOW state and set the others as INPUTS.
	if(PINC == 0b00111110){//first button pressed (scale 0-2kOhms)
		DDRD = 0x24; // D5
		PORTD = 0xDF;

		V_measured = adc_read(ADC_PIN);
		float R1 = 2;//in kilo ohms
		buffer = V_measured * V_ref;
		Vout = (buffer)/1024.0;//in volts
		buffer = (V_ref/Vout)-1;
		R2 = R1*buffer*1000;//*1000 to express it in just Ohms
		if (R2 > 2000)
		{
			printf("Increase the scale");
		}
		else
		{
			printf("Resistance: %.2f",R2);
		}
		
	}

	if(PINC == 0b00111101){//second button pressed (scale 2k-20kOhms)
		DDRD = 0x14;// D4
		PORTD = 0xEF;

		V_measured = adc_read(ADC_PIN);
		float R1 = 20;// kilo ohms
		buffer = V_measured * V_ref;
		Vout = (buffer)/1024.0;//in volts
		buffer = (V_ref/Vout)-1;
		R2 = R1*buffer;
		if (R2 > 20)//still kilo ohm it wasn't *1000
		{
			printf("Increase the scale");
		}
		if (R2 < 2)
		{
			printf("Decrease the scale");
		}
		if (R2 >= 2 && R2 <= 20)
		{
			printf("Resistance: %.2f",R2);
		}
		
	}

	if(PINC == 0b00111011){//third button pressed (scale 20k-200kOhms)
		DDRD = 0x0C;// D3
		PORTD = 0xF7;

		V_measured = adc_read(ADC_PIN);
		float R1 = 200;
		buffer = V_measured * V_ref;
		Vout = (buffer)/1024.0;//in volts
		buffer = (V_ref/Vout)-1;
		R2 = R1*buffer;
		if (R2 > 200)
		{
			printf("Increase the scale");
		}
		if (R2 < 20)
		{
			printf("Decrease the scale");
		}
		if (R2 >= 20 && R2 <= 200)
		{
			printf("Resistance: %.2f",R2);
		}
	}

	if(PINC == 0b00110111){//fourth button pressed (scale 200k-1MOhm)
		DDRD = 0x44;// D6
		PORTD = 0xB7;

		V_measured = adc_read(ADC_PIN);
		float R1 = 1000;
		buffer = V_measured * V_ref;
		Vout = (buffer)/1024.0;//in volts
		buffer = (V_ref/Vout)-1;
		R2 = R1*buffer;
		if (R2 > 1000)
		{
			printf("Increase the scale");
		}
		if (R2 < 200)
		{
			printf("Decrease the scale");
		}
		if (R2 >= 200 && R2 <= 1000)
		{
			printf("Resistance: %.2f",R2);
		}
	}
	
}

uint16_t adc_read(uint8_t adc_channel){
    ADMUX &= 0xf0;
    ADMUX |= adc_channel;
    ADCSRA |= (1<<ADSC);
    while( (ADCSRA & (1<<ADSC)));
    return ADC;
}
// float V_measure(void){
// ADMUX = ADMUX | 0x40;
// ADCSRB = ADCSRB & (0xF8);
// ADCSRA = ADCSRA | 0xE7;
// unsigned int V_measure = ADCL;
// return(((V_measure + ((ADCH & 0x03)<<8))*5/1023)*5/3);
// }