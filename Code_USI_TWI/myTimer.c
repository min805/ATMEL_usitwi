/*
 * myTimer.c
 *
 * Created: 4/10/2017 1:45:07 PM
 *  Author: min
 */ 

#include "myConfig.h"
#include "myTimer.h"

/*************************************************
***************************************************/
void timer_init(uint16_t tic_time)
{
	/*  
	Initial Timer1
	1sec = 1hz
	f = 8Mhz/(2x(prescaler)x(1+OCR1A))
	0.9998 = 8MHz/(2 x 1024 x (1 + 3906))
	OCR1A = 3906; 	0.9998Hz,	1.00019sec
	OCR1A = 7813; 	0.4999Hz,	2.00040sec
	OCR1A = 39060; 	0.1000Hz, 	9.99936sec   
	*/ 
	//but real value here...                                           
	OCR1A = tic_time;	
	//OCR1A = 1953			//1953=0.25sec
	//OCR1A = 3906;			//3906=0.50sec
	//OCR1A = 7812;			//7812=1.00sec	
	TCNT1  = 0;
		///Timer1 prescaler = F_clk/1024
		///Timer1 mode = CTC with OCR1A(WGM = 0b0100)
		///No port out	
	TCCR1A = 0;
	TCCR1B = 0x0D;
		///0x0D->Clock On
		///(1<<WGM12)|(1<<CS12)|(1<<CS10)
		///0x08->Clock STOP!
		///(1<<WGM12)
	TCCR1C = 0;	
	TIMSK1 = (1<<OCIE1A);	// enable timer ISR
		///TIMSK1 &= ~(1<<OCIE1A);// disable timer ISR		
} 