/*
 * gokart.c
 * Program for uC-based electric go-kart.
 * Created: 2019-07-11 12:45:12
 * Author : Łukasz Lesiecki
 */ 
#define F_CPU 1000000UL
#define _NOP() do { __asm__ __volatile__ ("nop"); } while (0)//call _NOP() for really short delay
#define DELAY_MULTIPLIER 11

#define BRAKING 0
#define ACCELERATION 1

#define DRIVER_1 PA7
#define DRIVER_2 PA5
#define ENABLE PA6
#define FWD PA3
#define BWD PA4
#define ACC PA0
#define BRK PA1

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

/*
volatile int adc_result;

ISR(ADC_vect)
{
	//here ADSC is 0 again
	adc_result = ADCL | (ADCH<<8);
}
*/

int get_adc()
{
	unsigned int i = 0;
	ADCSRA |= (1<<ADSC);//start conversion
	while(!(ADCSRA & (1<<ADIF)) && i++ < 100);//wait for conversion to complete
	if(!(ADCSRA & (1<<ADIF)))//if conversion fails
		return -1;
	ADCSRA &= ~(1<<ADIF);//clear conversion complete flag
	return (ADCH<<8) | ADCL;
}

int main(void)
{
	//Timer/Counter0 settings:
	DDRA |= (1<<DRIVER_1);//OC0B(PA7) pin as output
	TCCR0A |= (1<<COM0B1);//
	TCCR0A &= ~(1<<COM0B0);//non-inverting mode, output = OC0B pin
	TCCR0A |= (1<<WGM01) | (1<<WGM00);//
	TCCR0B |= (1<<WGM02);//fast PWM, TOP = OCR0A
	TCCR0B &= ~((1<<CS02) | (1<<CS01));//
	TCCR0B |= (1<<CS00);//clock source - clkI/O, no prescalling
	OCR0A = F_CPU/20000;//PWM frequency ~20kHz
	//OCR0B = 20*F_CPU/20000/100;//PWM duty cycle ~20%
	//Timer/Counter1 settings:
	DDRA |= (1<<DRIVER_2);//OC1B(PA5) pin as output
	TCCR1A |= (1<<COM1B1);//
	TCCR1A &= ~(1<<COM1B0);//non-inverting mode, output = OC1B pin
	TCCR1A |= (1<<WGM11) | (1<<WGM10);//
	TCCR1B |= (1<<WGM13) | (1<<WGM12);//fast PWM, TOP = OCR1A
	TCCR1B &= ~((1<<CS12) | (1<<CS11));//
	TCCR1B |= (1<<CS10);//clock source - clkI/O, no prescalling
	OCR1A = F_CPU/20000;//PWM frequency ~20kHz
	//OCR1B = 10*F_CPU/20000/100;//PWM duty cycle ~10%
	//ADC settings:
	PRR &= ~(1<<PRADC);//power the ADC
	ADCSRA |= (1<<ADEN);//enable the ADC
	ADMUX &= ~((1<<REFS1) | (1<<REFS0));//Vref = VCC
	//ADCSRA |= (1<<ADIE);//interrupt when AD conversion is complete
	ADCSRA |= (1<<ADPS2);//ADC clock precaller /16
	//sei();//enable interrupts
	DDRA |= (1<<ENABLE);//PA6 pin as output
	unsigned int
		input = BRAKING,
		acceleration_value_x_10 = 0,
		braking = 0,
		fwd = 0,
		bwd = 0;
	int adc_result = 0;
	while(1)
	{
		(input = !input) ? (ADMUX &= 0xC0) : (ADMUX |= 0x01);//input selection for ADC
		adc_result = get_adc();
		
		if(adc_result < 0)//if ADC error
		{
			input = BRAKING;//max braking
			adc_result = 1000;
		}
		
		if(input == BRAKING)
			braking = adc_result;
		
		while(braking > 200)//braking mode
		{
			TCCR0B &= ~(1<<CS00);//no clock source (Timer/Counter0 stopped)
			PORTA &= ~(1<<DRIVER_1);//output low (0% duty cycle)
			TCCR1B &= ~(1<<CS00);//no clock source (Timer/Counter1 stopped)
			PORTA &= ~(1<<DRIVER_2);//output low (0% duty cycle)
			if(braking > 800)
				PORTA |= (1<<ENABLE);//enable the motor driver
			else
			{
				int i;
				for(i = 0; i < 1000; i++)//50ms of braking, ~20kHz PWM signal generation
				{
					unsigned char delay = braking/(1023/DELAY_MULTIPLIER) + 1;
					while(--delay)
						_NOP();
					PORTA &= ~(1<<ENABLE);//disable the motor driver
					delay = DELAY_MULTIPLIER - braking/(1023/DELAY_MULTIPLIER) + 1;
					while(--delay)
						_NOP();
					PORTA |= (1<<ENABLE);//enable the motor driver
				}
			}
			
			adc_result = get_adc();
			
			if(adc_result < 0)
				braking = 1000;//max braking if an error occurs
			else
				braking = adc_result;
			continue;
		}
		
		if(input == ACCELERATION && braking < 200)//normal drive mode
		{
			if(acceleration_value_x_10 < adc_result*10)
				acceleration_value_x_10 = adc_result*10;//accelerate immediately
			else if(acceleration_value_x_10 > 100)
				acceleration_value_x_10 -= 2;//decelerate slowly
			
			if(adc_result < 200)//no acceleration
			{
				PORTA &= ~(1<<ENABLE);//shut down the motor driver
				
				fwd = PINA & (1<<FWD);//check for direction change
				bwd = PINA & (1<<BWD);//check for direction change
			}
			
			if(!bwd != !fwd)
			{
				fwd = bwd = 0;
				TCCR0B &= ~(1<<CS00);//no clock source (Timer/Counter0 stopped)
				TCCR1B &= ~(1<<CS00);//no clock source (Timer/Counter1 stopped)
				PORTA &= ~(1<<DRIVER_2);//output low (0% duty cycle)
				PORTA &= ~(1<<DRIVER_1);//output low (0% duty cycle)
			}
			
			if(fwd)
			{
				//generate PWM signal for forward half-bridge:
				TCCR0B |= (1<<CS00);//clock source - clkI/O, no prescalling
				OCR0B = acceleration_value_x_10*F_CPU/10/1023/20000;//PWM duty cycle = adc/adc_max
				//continous LOW for backward half-bridge:
				TCCR1B &= ~(1<<CS00);//no clock source (Timer/Counter0 stopped)
				PORTA &= ~(1<<DRIVER_2);//output low (0% duty cycle)
			}
			
			if(bwd)
			{
				//generate PWM signal for backward half-bridge:
				TCCR1B |= (1<<CS00);//clock source - clkI/O, no prescalling
				OCR1B = acceleration_value_x_10*F_CPU/10/1023/20000;//PWM duty cycle = adc/adc_max
				//continous LOW for forward half-bridge:
				TCCR0B &= ~(1<<CS00);//no clock source (Timer/Counter0 stopped)
				PORTA &= ~(1<<DRIVER_1);//output low (0% duty cycle)
			}
		}
	}
}
