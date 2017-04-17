/*
 * myTimer.c
 *
 * Created: 4/10/2017 1:45:07 PM
 *  Author: min
 */ 

#include "myConfig.h"
#include "myTimer.h"
#include "myADC.h"


/*************************************************
***************************************************/

ISR(TIM0_OVF_vect)
{
	TIMSK0 = 0;			//disable interrupt
	adc_start(_InCurrent);
	
}


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
	TCCR1B = (1<<WGM12)|(1<<CS12)|(1<<CS10);
		///0x0D->Clock On, divide by 1024		
		///0x08->Clock STOP!
		///(1<<WGM12)
	TCCR1C = 0;	
	TIMSK1 = (1<<OCIE1A);	// enable timer ISR
		///TIMSK1 &= ~(1<<OCIE1A);// disable timer ISR		
} 





/*************************************************
***************************************************/
void pwm_init(uint8_t channel)
{
	// Use Timer0
	BIT_SET(DDR_PWM,channel);	//PB2,OC0A
	TCCR0A = (1<<COM0B1)|(1<<WGM01)|(1<<WGM00); 
	//0x23 = 0010.0011;
	//COM0B[1:0] = 2 :2= non-Inverted PWM / 3= Inverted PWM
	//WGM[02:00] = 3 :FastPWM (0 to 0xFF)
	//WGM[02:00] = 7 :FastPWM (0 to OCR0A)(use this)
	
	TCCR0B = (1<<WGM02)|(1<<CS01)|(1<<CS00); 
	//	0x0B = 0000.1011;		//WGM02 = 1
	//CS[02:00] =  3 :clk/64 Prescaler
	TCNT0 = 0;
	OCR0A = MY_PWM_FREQ;		//240 if WGM=3,//0xF0 if WGM=7,TOP==OCR0A
	// It's 500.0Hz
	
	OCR0B = INIT_PWM_DUTY;		//initial set,almost 0% duty = 3.19A
	TIMSK0 = 0;			//start from disable interrupt
}

/*************************************************
***************************************************/
void pwm_write(bool inc, uint8_t step)
{
	
	uint8_t nowDuty = OCR0B;
	uint8_t nowFreq = OCR0A;
	
	//while(BIT_CHECK(TIFR0,TOV0)); //wait for TOV0
	//TIFR0 |= (1<<TOV0); //clear TOV0.(sure!)
	//if(TIFR0 & (1<<TOV0)){
	//	TIFR0 |= (1<<TOV0); //clear TOV0.(sure!)
	//}

	//----------------------------------------------------option #1
	//uint8_t i = 0;
	//for(i=0;i < step;i++){
	//	if(inc){	// increase duty
	//		//power_on();
	//		if( nowDuty < nowFreq ){OCR0B = (++nowDuty);
	//		}else{OCR0B = nowFreq;}
	//		//_delay_us(1);
	//	}else {		// decrease duty
	//		if( nowDuty != 0 ){		OCR0B = (--nowDuty);}
	//		/*if(nowDuty == 0){power-off(); */
	//	}
	//}// end for
	//----------------------------------------------------option #2
	if(inc){	//---increase Duty
		//power_on();
		if((nowDuty+step) < nowFreq ){ 
			OCR0B = nowDuty+step;
		}else{
			OCR0B = nowFreq;
		}
		
	}else{		//---decrease Duty
		if((nowDuty-step) > 0){
			OCR0B = nowDuty-step;
		}else{
			OCR0B = 0;
		}
		/*if(nowDuty == 0){power-off(); */
	}
	TIMSK0 = (1<<TOIE0);//Enable TOV0 interrupt
	//--------------------------------------------------------------
	//-----------------------------------------------
	//convert nowPWM to displayValue(%)
	//displayValue =(unsigned char)((_duty*100)/255);
	//-----------------------------------------------
}