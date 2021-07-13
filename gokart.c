/*
 * gokart.c
 *
 * Created: 2020-11-19 01:44:31
 * Author : Łukasz Lesiecki
 */ 
#define F_CPU 1000000UL

// analog inputs
#define aiACC PORTC5
#define aiBRK PORTC4

// digital inputs
#define diBWD PORTD0
#define diLight PORTD1
#define diNeon PORTD2
#define diFWD PORTC1
#define diL PORTC3
#define diR PORTC2

// digital outputs
#define doNeon PORTB1
#define doLight PORTB0
#define doR PORTB3
#define doL PORTD7
#define doStop PORTD6
#define doBWD_indicator PORTC0
#define doEnable PORTB2
#define doFWD PORTD5
#define doBWD PORTD3
#define doErr PORTB4

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

struct Input
{
	volatile uint8_t * pin;
	uint8_t mask;
	int8_t delay;
	uint8_t state;
};

struct Drive
{
	// limits of readings from the analog acceleration and brake inputs
	const uint8_t pedal_min;
	const uint8_t pedal_max;
	const uint8_t pedal_error_limit;
};

const uint8_t TOP = F_CPU / 20000 - 1;// value of TOP for 20KHz frequency

void fwd_set_duty(uint8_t duty)//duty cycle in % as parameter
{
	// set desired duty cycle; multiply first to improve accuracy
	OCR0B = duty * TOP / 100;
	// connect OC0B pin (PD5) to timer if duty cycle > 0, disconnect otherwise
	TCCR0A = (TCCR0A & (~(1<<COM0B1))) | (!!duty<<COM0B1);
}

void enable_set_duty(uint8_t duty)//duty cycle in % as parameter
{
	// set desired duty cycle; multiply first to improve accuracy
	OCR1B = duty * TOP / 100;
	// connect OC1B pin (PB2) to timer if duty cycle > 0, disconnect otherwise
	TCCR1A = (TCCR1A & (~(1<<COM1B1))) | (!!duty<<COM1B1);
}

void bwd_set_duty(uint8_t duty)//duty cycle in % as parameter
{
	// set desired duty cycle; multiply first to improve accuracy
	OCR2B = duty * TOP / 100;
	// connect OC2B pin (PD3) to timer if duty cycle > 0, disconnect otherwise
	TCCR2A = (TCCR2A & (~(1<<COM2B1))) | (!!duty<<COM2B1);
}

void configure_registers()
{
	// outputs configuration:
	DDRB |= (1<<doNeon) | (1<<doEnable) | (1<<doLight) | (1<<doR) | (1<<doErr);
	DDRC |= (1<<doBWD_indicator);
	DDRD |= (1<<doStop) | (1<<doL) | (1<<doFWD) | (1<<doBWD);
	
	// pull-up configuration:
	PORTC |= (1<<diL) | (1<<diR) | (1<<diFWD);
	PORTD |= (1<<diBWD) | (1<<diLight) | (1<<diNeon);
	
	// FWD PWM settings:
	TCCR0A |= (1<<WGM00) | (1<<WGM01);// Fast PWM mode, non inverting
	// Fast PWM mode, source = CLK-IO, no prescaller
	TCCR0B |= (1<<WGM02) | (1<<CS00);
	OCR0A = TOP;// PWM frequency ~20kHz
	
	// enable PWM settings:
	// Fast PWM mode, non inverting, output at OC1B (PB2)
	TCCR1A |= (1<<WGM10) | (1<<WGM11);
	// Fast PWM mode, source = CLK-IO, no prescaller
	TCCR1B |= (1<<WGM12) | (1<<WGM13) | (1<<CS10);
	OCR1A = TOP;// PWM frequency ~20kHz
	
	// BWD PWM settings:
	// Fast PWM mode, non inverting, output at OC2B (PD3)
	TCCR2A |= (1<<WGM20) | (1<<WGM21);
	// Fast PWM mode, source = CLK-IO, no prescaller
	TCCR2B |= (1<<WGM22) | (1<<CS20);
	OCR2A = TOP;//PWM frequency ~20kHz
	
	// Analog/Digital Converter settings:
	// V ref = AVcc, ADC4 as input, conversion result left adjusted
	ADMUX |= (1<<REFS0) | (1<<MUX2) |(1<<ADLAR);
	// enable ADC, start conversion, 16x prescaller
	ADCSRA |= (1<<ADEN) | (1<<ADPS2);
}

uint8_t get_ADC_result(uint8_t * result)
{
	ADCSRA |= (1<<ADSC);// start conversion
	uint8_t iter_limit = 50;// create an iterations limit
	// wait until ADC Interrupt Flag is set or the iterations limit is reached
	while(!(ADCSRA & (1<<ADIF)) && --iter_limit > 0);
	ADCSRA &= ~(1<<ADIF);// clear ADC Interrupt Flag
	*result = ADCH;// get the conversion result
	return iter_limit;// later we can check if the iterations limit was reached
}

void set_error_indicator()
{
	PORTB |= (1<<doErr);
}

uint8_t set_driving_state(struct Drive * drive, struct Input * inputs, uint8_t accelerate, uint8_t brake)
{
	if(brake > drive->pedal_min)// regenerative braking mode
	{
		fwd_set_duty(0);
		bwd_set_duty(0);
		enable_set_duty((brake - drive->pedal_min) * 100
		/ (drive->pedal_max - drive->pedal_min)
		);
	}
	else if(accelerate > drive->pedal_error_limit)// input error, emergency brake
	{
		fwd_set_duty(0);
		bwd_set_duty(0);
		enable_set_duty(100);
		return 1;
	}
	else if(accelerate > drive->pedal_min)// normal drive mode
	{
		// calculating the desired duty cycle,
		// based on the acceleration pedal readings
		uint8_t duty_cycle =
		(accelerate - drive->pedal_min) * 100
		/ (drive->pedal_max - drive->pedal_min);
		
		if(inputs[0].state)// drive forwards
		{
			fwd_set_duty(duty_cycle);
			bwd_set_duty(0);
			enable_set_duty(100);
		}
		else if(inputs[1].state)// reverse
		{
			fwd_set_duty(0);
			// yes, it can reverse as fast as driving forwards :D
			bwd_set_duty(duty_cycle);
			enable_set_duty(100);
		}
		else// neutral
		{
			fwd_set_duty(0);
			bwd_set_duty(0);
			enable_set_duty(0);
		}
	}
	else// neutral
	{
		fwd_set_duty(0);
		bwd_set_duty(0);
		enable_set_duty(0);
	}
	return 0;
}

void handle_inputs(struct Input * inputs, uint8_t * acceleration_lock, const uint8_t debounce_delay)
{	
	for (uint8_t input_number = 0;
	input_number < 6;
	++input_number
	)
	{
		// high state = pulled up = button released
		if(*(inputs[input_number].pin) & inputs[input_number].mask)
		{
			if(inputs[input_number].delay > 0)
			--inputs[input_number].delay;
		}
		else// low state = pulled down = button pressed
		{
			if(!inputs[input_number].delay)
			{
				inputs[input_number].delay = debounce_delay;
				switch(input_number)
				{
					case 0:
					inputs[1].state = 0;
					PORTC &= ~(1<<doBWD_indicator);
					inputs[0].state ^= 1;
					*acceleration_lock = 1;
					break;
					case 1:
					inputs[0].state = 0;
					PORTC ^= (1<<doBWD_indicator);
					inputs[1].state ^= 1;
					*acceleration_lock = 1;
					break;
					case 2:
					inputs[2].state ^= 1;
					break;
					case 3:
					inputs[3].state ^= 1;
					break;
					case 4:
					PORTB ^= (1<<doLight);
					break;
					case 5:
					PORTB ^= (1<<doNeon);
					break;
				}
			}
		}
	}
}

void handle_turn_signals(struct Input * inputs, uint8_t * turn_signal_state)
{
	// period of the turn signal, the target frequency is 1.5Hz
	const uint8_t turn_signal_period = 199;
	//turn signals handling
	if(++(*turn_signal_state) > turn_signal_period)
	turn_signal_state = 0;
			
	if(inputs[2].state)
	{
		if(*turn_signal_state < turn_signal_period / 2)
			PORTD |= (1<<doL);
		else
			PORTD &= ~(1<<doL);
	}
	else
		PORTD &= ~(1<<doL);
			
	if(inputs[3].state)
	{
		if(*turn_signal_state < turn_signal_period / 2)
			PORTB |= (1<<doR);
		else
			PORTB &= ~(1<<doR);
	}
	else
		PORTB &= ~(1<<doR);
}

int main(void)
{
	configure_registers();
	
	// variable for storing the state of turn signal
	uint8_t turn_signal_state = 0;
	
	// anti-idiot protection, who switch into drive mode,
	// while holding acceleration pedal pushed :D
	uint8_t acceleration_lock = 1;
	
	const uint8_t debounce_delay = 60;//delay for debouncing
	
	struct Drive drive = {
		// treat the first one third of a pedal as 0%
		255 / 3,
		// treat the last one third of a pedal as 100%
		255 * 2 / 3,
		// define a value that will be treated as an input error
		220
		};
	
	struct Input inputs[] = {
		//PIN pointer, PORT mask, debounce, state
		{ &PINC, (1<<diFWD),	0, 0},// FWD
		{ &PIND, (1<<diBWD),	0, 0},// BWD
		{ &PINC, (1<<diL),		0, 0},// turn signal left
		{ &PINC, (1<<diR),		0, 0},// turn signal right
		{ &PIND, (1<<diLight),	0, 0},// lights
		{ &PIND, (1<<diNeon),	0, 0},// neon
	};
	
    while (1) 
    {
		uint8_t brake;
		uint8_t accelerate;
		
		ADMUX &= ~(1<<MUX0);// ADC4 as input
		if(!get_ADC_result(&brake))
			// signalize an error, if the iterations limit was reached
			set_error_indicator();
		ADMUX |= (1<<MUX0);// ADC5 as input
		if(!get_ADC_result(&accelerate))
			// signalize an error, if the iterations limit was reached
			set_error_indicator();                                                                                                                                                                                                                   
		
		// unlock acceleration when pedal is released
		if(accelerate < drive.pedal_min)
			acceleration_lock = 0;
		
		if(acceleration_lock)// don't accelerate, when the protection is active
			accelerate = 0;
		
		// signalize an error, if the pedal signal limit was exceeded
		if(set_driving_state(&drive, inputs, accelerate, brake))
			set_error_indicator();
		
		handle_inputs(inputs, &acceleration_lock, debounce_delay);
		
		handle_turn_signals(inputs, &turn_signal_state);
			
		_delay_ms(2);
    }
}
