/*
 * myConfig.h
 *
 * Created: 4/10/2017 1:28:40 PM
 *  Author: min
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

//BIT OPERATOR==================================
#define BIT_CHECK(var,pos)	((var)&(1<<(pos)))
#define BIT_SET(var,pos)	((var)|=(1<<(pos)))
#define BIT_CLEAR(var,pos)	((var)&=~(1<<(pos)))
//----flagWakeup----------
#define _BIT_TIC			0
#define _BIT_I2C			1
#define _BIT_ADC			2
//#define _BIT_REPORT		3

#define _BIT_COUNT_ON		1
#define _BIT_COUNT_DONE		2

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
#define PORT_PWM	PORTA
#define PIN_PWM		PINA
#define DDR_PWM		DDRA


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
#define MY_TIC_TIME	1953		//0.25sec// 0.24  
//#define MY_TIC_TIME	3906	//0.50sec
//#define MY_TIC_TIME	7812	//1.00sec
#define TIC_FOR_1SEC	4

#define MY_PWM_FREQ	240		//500Hz
#define INIT_PWM_DUTY	220
#define MINIMUM_STEP	1

#define MY_ADDRESS		0x24


#define BRT_H	235	//2.35A
#define BRT_M	200	//2.00A
#define BRT_L	165	//1.65A
#define DIM_H	70	//0.70A
#define DIM_M	60	//0.60A
#define DIM_L	50	//0.50A
//---------Slope.up time = 6sec
#define TIMEUP_H	8
#define TIMEUP_M	7
#define TIMEUP_L	6
//---------Slope.down time = 20sec At Hi state
#define TIMEDOWN	20
//---------Delay time = 120sec
#define TIME_DELAY		120

//COMMAND==========================
#define MCMD_GET_CONFIG 0x00
#define MCMD_GET_NOWAMP 0x01
#define MCMD_GET_NOWPIR 0x02
#define MCMD_GET_NOWMOD 0x03

#define MCMD_SET_BRIGHT 0x14
#define MCMD_SET_DIMM 	0x15
#define MCMD_SET_TIMEUP 0x16
#define MCMD_SET_TIMEDN 0x17
#define MCMD_SET_DELAY	0x18

#define STATE_COMMAND	0x00
#define STATE_DATA_1	0x01
#define STATE_DATA_2	0x02

void call_set_TxBuffer(uint8_t amount);
void set_gWakeUpFlag_i2c(void);


#endif /* MYCONFIG_H_ */