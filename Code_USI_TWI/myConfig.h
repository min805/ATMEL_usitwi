/*
 * myConfig.h
 *
 * Created: 4/10/2017 1:28:40 PM
 *  Author: min
 
 - Request spec-------------------------------------------------------
 -	2.35A going to 0.700A
 -	2.00A going to 0.600A
 -	1.65A going to 0.500A
 -	6 sec +/-2 sec rise time
 -	20 sec +/- 5 sec falling time (2.35A only)  others on same slope
 -	Delay 120 sec +/-10sec
 
 
 */ 
/********************************************************************************
Change Activity:
Version    Date       Description
-------   ------      -------------
0.0.0    04.05.2017   Start the project.
********************************************************************************/

#ifndef MYCONFIG_H_
#define MYCONFIG_H_


#include <avr/io.h>
#include <avr/interrupt.h>
#ifndef F_CPU
#define F_CPU 8000000
#endif
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>


#define ENABLE_DEBUG	1

//BIT OPERATOR==================================
#define BIT_CHECK(var,pos)	((var)&(1<<(pos)))
#define BIT_SET(var,pos)	((var)|=(1<<(pos)))
#define BIT_CLEAR(var,pos)	((var)&=~(1<<(pos)))
//----flagWakeup----------
#define _BIT_TIC			0
#define _BIT_SEC			1
#define _BIT_I2C			2
#define _BIT_ADC			3

#define _BIT_COUNT_ON		0
#define _BIT_COUNT_DONE		1
#define _BIT_SLOPE_ON		2

//PORTB=================================
#define PORT_SW		PORTB
#define PIN_SW		PINB
#define DDR_SW		DDRB

#define _SW0		0	//pin2,  PORTB0
#define _SW1		1	//pin3,  PORTB1
#define _LCD		2	//pin5,  PORTB2
#define _TP			2	//pin5,  PORTB2 ->external 10k pull-up

//PORTA=================================
#define PORT_ADC	PORTA
#define PIN_ADC		PINA
#define DDR_ADC		DDRA
//#define PORT_USI	PORTA	//For LCD
//#define PIN_USI		PINA
//#define DDR_USI		DDRA



#define _InVoltgae	1	//pin12, PORTA1
#define _InCurrent	2	//pin11, PORTA2
#define _InSensor	3	//pin10, PORTA3
#define _SCL		4	//pin9,	 PORTA4
#define _MISO		5	//pin8,  PORTA5
#define _MOSI		6	//pin7,	 PORTA6
#define _SDA		6 	//pin7,  PORTA6
#define _OutPWM		7	//pin6,  PORTA7
#define _Temperature 34	//b0100010
//=======================================
								// ----- // Test	
//#define MY_TIC_TIME		781		//0.10sec
//#define TIC_FOR_1SEC	10							
//#define MY_TIC_TIME	1953	//0.25sec// 0.24  
//#define TIC_FOR_1SEC	4
#define MY_TIC_TIME	3906	//0.50sec
#define TIC_FOR_1SEC	2
//#define MY_TIC_TIME	7812	//1.00sec
//#define TIC_FOR_1SEC	1


#define MY_PWM_FREQ		240		//500Hz
#define INIT_PWM_DUTY	(MY_PWM_FREQ-1)
#define MY_MINIMUM		1

#define MY_ADDRESS		0x24



#define BRT_H		0x1df	//2.35A (2.346A=0x1df)
#define BRT_M		0x198	//2.00A (2.002A=0x198) 
#define BRT_L		0x150	//1.65A (1.650A=0x150)
#define DIM_H		0x8c	//0.70A (0.695A=0x8c)
#define DIM_M		0x7a	//0.60A (0.607A=0x7a)
#define DIM_L		0x70	//0.50A (0.557A=0x70)
//---------Slope.up time = 6sec
#define TIMEUP_H	2//0xa
#define TIMEUP_M	2//0xb
#define TIMEUP_L	2//0xc
//---------Slope.down time = 20sec At Hi state
#define TIMEDOWN	2//0x14
//---------Delay time = 120sec
#define TIME_DELAY	0x03

//COMMAND==========================
#define GET_CONFIG	0//0x00
#define GET_NOWAMP	1//0x01
#define GET_NOWPIR	2//0x02
#define GET_NOWMOD	3//0x03

#define SET_BRIGHT	17//0x11
#define SET_DIMM	18//0x12
#define SET_TIMEUP	19//0x13
#define SET_TIMEDN	20//0x14
#define SET_DELAY	21//0x15

#define STATE_COMMAND	0x00
#define STATE_DATA_1	0x01
#define STATE_DATA_2	0x02

void call_set_TxBuffer(uint8_t amount);
void call_set_nowAMP(uint16_t adcValue);
void set_gWakeUpFlag_i2c(void);
//void set_gWakeUpFlag_adc(void);

#endif /* MYCONFIG_H_ */