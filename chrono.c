/*
 * chrono.c
 * Program for uC-based chronograph.
 * Created: 2019-05-06 14:33:47
 * Author : Luki
 */ 
#define F_CPU 1000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

/*
2313 uC leads:
PA2     VCC
PD0     PB7
PD1     PB6
PA1     PB5
PA0     PB4
PD2     PB3
PD3     PB2
PD4     PB1
PD5     PB0
GND     PD6
*/

unsigned char digitPB[10] = {//array for 7-segment display, port B
	//---abcde
	0b00011111,//0
	0b00001100,//1
	0b00011011,//2
	0b00011110,//3
	0b00001100,//4
	0b00010110,//5
	0b00010111,//6
	0b00011100,//7
	0b00011111,//8
	0b00011110,//9
};
unsigned char digitPD[10] = {//array for 7-segment display, port D
	//-fg-----
	0b01000000,//0
	0b00000000,//1
	0b00100000,//2
	0b00100000,//3
	0b01100000,//4
	0b01100000,//5
	0b01100000,//6
	0b00000000,//7
	0b01100000,//8
	0b01100000,//9
};

volatile unsigned int result;

ISR(INT0_vect)  //INT0 PD2
{
	TCNT1 = 0;
}

ISR(INT1_vect)  //INT1 PD3
{
	result = 300000/TCNT1;
}

int main(void)
{
	cli();//interrupts off
	DDRB |= (1<<PB0) | (1<<PB1) | (1<<PB2) | (1<<PB3) | (1<<PB4);//I/O settings
	DDRD |= (1<<PD0) | (1<<PD1) | (1<<PD4) | (1<<PD5) | (1<<PD6);//I/O settings
	GIMSK |= (1<<INT0);//INT0 on
	GIMSK |= (1<<INT1);//INT1 on
	PORTD |= (1<<PD2);//pull-up
	PORTD |= (1<<PD3);//pull-up
	MCUCR = (MCUCR & ~0b11110000) | 0b00001111;//INT when 0->1 (rising edge)
	TCCR1B = (TCCR1B & ~((1<<CS12) | (1<<CS11) | (1<<CS10))) | (1<<CS10);//clk src for counter without prescalling
	unsigned char digit;
	sei();//interrupts on
	while(1)
	{
		digit = result % 10;
		PORTB = (PORTB & 0b11100000) | digitPB[digit];
		PORTD = (PORTD & 0b10001100) | digitPD[digit] | (1<<PD0) | (1<<PD1);
		//_delay_ms(5);
		digit = result/10 % 10;
		PORTB = (PORTB & 0b11100000) | digitPB[digit];
		PORTD = (PORTD & 0b10001100) | digitPD[digit] | (1<<PD0) | (1<<PD4);
		//_delay_ms(5);
		digit = result/100 % 10;
		PORTB = (PORTB & 0b11100000) | digitPB[digit];
		PORTD = (PORTD & 0b10001100) | digitPD[digit] | (1<<PD1) | (1<<PD4);
		//_delay_ms(5);
	}
}
