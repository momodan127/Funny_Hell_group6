
//#include <Arduino.h>
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <usart.h>
#include <avr/interrupt.h>
#define V_ref 5.0
#define ADC_PIN_RES 1

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
    // PINB = 0x10;
    
    while(1)
    {
        // printf("C %f\n", capacitance);
        // printf("delay %d\n", test);

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
DDRC = 0b11110010; //, four outputs and four inputs (buttons)
PORTC = 0b00111101;
//DDRC=0b00111100;
//PORTC = 0b00111100;
    uint16_t V_measured = 0;
	float Vout = 0;
	float R2 = 0;
	float buffer = 0;
//When we change the scale all we need to do is change the desired scale pin as OUTPUT with LOW state and set the others as INPUTS AND LOW STATE!!!!
// while(1){
	if(PINC == 0b00111000){// (scale 0-2kOhms)
		// DDRB = 0b00100100;
		// PORTB = 0b00100000;
		DDRB |= (1<<PB2) | (1<<PB5);  //D10
		PORTB |= (1<<PB5); 
	
		float R1 = 2;//in kilo ohms
		V_measured = adc_read(ADC_PIN_RES);
		
		buffer = V_measured * V_ref;
		Vout = (buffer)/1024.0;//in volts
		buffer = (V_ref/Vout)-1;
		R2 = R1*buffer*1000;//*1000 to express it in just Ohms
			printf("Buffer = %.1f\n\n\n\n", buffer);
	printf("Vout = %.1f\n", Vout);
	printf("R2 = %.1f\n", R2);
	printf("Vmeasured = %d\n", V_measured);
		if (R2 > 2000)
		{
		
			printf("Increase the scale");
		}
		else
		{
	
			printf("Resistance: %.1f\n",R2);
		}
		
	}
	if(PINC == 0b00110100){//(scale 2k-20kOhms)
		DDRB |= (1<<PB3) | (1<<PB5); // D11
		PORTB |= (1<<PB5);

		float R1 = 20;// kilo ohms
		V_measured = adc_read(ADC_PIN_RES);

		buffer = V_measured * V_ref;
		Vout = (buffer)/1024.0;//in volts
		buffer = (V_ref/Vout)-1;
		R2 = R1*buffer;
	printf("Buffer = %.1f\n\n\n\n", buffer);
	printf("Vout = %.1f\n", Vout);
	printf("R2 = %.1f\n", R2);
	printf("Vmeasured = %d\n", V_measured);
		if (R2 > 20)
		{
			printf("Increase the scale");
		}
		if (R2 < 2)
		{
			printf("Decrease the scale");
		}
		if (R2 >= 2 && R2 <= 20)
		{
			printf("Resistance: %.1f",R2);
		}
		
	}

	// if(PINC == 0b00111011){//third button pressed (scale 20k-200kOhms)
	// 	DDRB |= (1<<PB4) | (1<<PB5); // D12
	// 	PORTB |= (1<<PB5);
	// 	V_measured = adc_read(ADC_PIN_RES);
	// 	float R1 = 200;
	// 	buffer = V_measured * V_ref;
	// 	Vout = (buffer)/1024.0;//in volts
	// 	buffer = (V_ref/Vout)-1;
	// 	R2 = R1*buffer;
	// printf("Buffer = %.1f\n\n\n\n", buffer);
	// printf("Vout = %.1f\n", Vout);
	// printf("R2 = %.1f\n", R2);
	// printf("Vmeasured = %d\n", V_measured);
	// 	if (R2 > 200)
	// 	{
	// 		printf("Increase the scale");
	// 	}
	// 	if (R2 < 20)
	// 	{
	// 		printf("Decrease the scale");
	// 	}
	// 	if (R2 >= 20 && R2 <= 200)
	// 	{
	// 		printf("Resistance: %.1f",R2);
	// 	}
// }
}

uint16_t adc_read(uint8_t adc_channel){
    ADMUX &= 0xf0;
    ADMUX |= adc_channel;
    ADCSRA |= (1<<ADSC);
    while( (ADCSRA & (1<<ADSC)));
    return ADC;
}
