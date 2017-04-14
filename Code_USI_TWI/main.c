/*
 * Code_USI_TWI.c
 *
 * Created: 4/10/2017 1:26:02 PM
 *  Author: min
 */ 


#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include "myConfig.h"
#include "myUsiTwiSlave.h"
#include "myTimer.h"

uint8_t gWakeUpFlag;
uint8_t gCountFlag;
uint8_t ticCount;
uint8_t secCount;

//...................... 
uint16_t cfg_Bright[4]={BRT_H,BRT_M,BRT_L,BRT_H};	//ADC value
uint16_t cfg_Dimm[4]  ={DIM_H,DIM_M,DIM_L,DIM_H};	//ADC value
uint8_t cfg_TimeUp[4] ={TIMEUP_H,TIMEUP_M,TIMEUP_L,TIMEUP_H};	//PWM value
uint8_t cfg_TimeDn	  = TIMEDOWN;				//PWM value
uint8_t cfg_TimeDelay = TIME_DELAY;				//SEC value

bool nowPIR = true;
uint8_t nowMODE = 0;
uint16_t nowAMP = 0x1234;

/*************************************************
***************************************************/
ISR(TIM1_COMPA_vect)
{
	BIT_SET(gWakeUpFlag,_BIT_TIC);
	wdt_reset();
	if(++ticCount >= TIC_FOR_1SEC){
		ticCount = 0;
		secCount++;
	}
	//------------------------------
	//PORT_SW = PIN_SW^(1<<_TP);
	//------------------------------
}

/*************************************************
callback function for gWakeUpFlag I2C
***************************************************/
void set_gWakeUpFlag_i2c(void)
{
	BIT_SET(gWakeUpFlag,_BIT_I2C);
}


/*************************************************
callback function for build txBufffer
***************************************************/
void call_set_TxBuffer(uint8_t amount)
{
	uint8_t data,i;
	static uint8_t state = STATE_COMMAND;
	static uint8_t command;
	static uint16_t data_h;


	for(i=0; i < amount; i++)
	{
		data = usiTwi_ByteFromRxBuffer();
		if(state == STATE_COMMAND){
			//usiTwi_flushTxBuffers();
			command = data;
			switch(command)
			{
			case GET_CONFIG:	
				usiTwi_ByteToTxBuffer((uint8_t)((cfg_Bright[nowMODE]>>8)&0x00FF) );	
				usiTwi_ByteToTxBuffer((uint8_t)(cfg_Bright[nowMODE]&0x00FF) );
						
				usiTwi_ByteToTxBuffer((uint8_t)((cfg_Dimm[nowMODE]>>8)&0x00FF) );
				usiTwi_ByteToTxBuffer((uint8_t)(cfg_Dimm[nowMODE]&0x00FF) );
												
				usiTwi_ByteToTxBuffer(cfg_TimeUp[nowMODE]);
				usiTwi_ByteToTxBuffer(cfg_TimeDn);
				
				usiTwi_ByteToTxBuffer(cfg_TimeDelay);

				state = STATE_COMMAND;
			break;
			case GET_NOWAMP:
				usiTwi_ByteToTxBuffer((uint8_t)((nowAMP>>8)&0x00FF));
				usiTwi_ByteToTxBuffer((uint8_t)(nowAMP&0x00FF));
				
				
				state = STATE_COMMAND;
			break;
			case GET_NOWPIR:				
				usiTwi_ByteToTxBuffer(0);
				if(nowPIR != false){
					usiTwi_ByteToTxBuffer(0x01);
				}else{
					usiTwi_ByteToTxBuffer(0x00);
				}
				
				state = STATE_COMMAND;
			break;
			case GET_NOWMOD:				
				usiTwi_ByteToTxBuffer(0);
				usiTwi_ByteToTxBuffer(nowMODE);
				
				state = STATE_COMMAND;
			break;
			case SET_BRIGHT:
			case SET_DIMM:
			case SET_TIMEUP:
			case SET_TIMEDN:
			case SET_DELAY:
				state = STATE_DATA_1;
			break;
			default:
				state = STATE_COMMAND;
			}
		}else if(state == STATE_DATA_1){
			data_h = (uint16_t)data;
			state = STATE_DATA_2;
			
		}else if(state == STATE_DATA_2){
			switch(command)
			{
			case SET_BRIGHT:
				cfg_Bright[nowMODE] = (data_h<<8)&0xff00;
				cfg_Bright[nowMODE] |= data;
			break;
			case SET_DIMM:				
				cfg_Dimm[nowMODE] = (data_h<<8)&0xff00;
				cfg_Dimm[nowMODE] |= data;
			break;
			case SET_TIMEUP:
				cfg_TimeUp[nowMODE] = data;
			break;
			case SET_TIMEDN:
				cfg_TimeDn = data;
			break;
			case SET_DELAY:
				cfg_TimeDelay = data;
			break;
			} //end switch
			state = STATE_COMMAND;
		} //end if
	} //end for
} //

/*************************************************
***************************************************/
void debug_init(void)
{
	PORT_SW &= ~(1<<_TP);	//Pull-up
	DDR_SW |= (1<<_TP);		//0=input, 1=output(Save energy)
}

uint8_t getSwitch(void)
{
	uint8_t retValue = 0;
	//pull up, input
	//MCUCR &= ~(1<<PUD);
	//Add external full up!!
	PORT_SW |= ~((1<<_SW0)|(1<<_SW1));// tri-state(hi-z)
	DDR_SW  &= ~((1<<_SW0)|(1<<_SW1));
	
	if(PINB & (1<<_SW0)){ retValue |= 0x01;}
	if(PINB & (1<<_SW1)){ retValue |= 0x02;}
	//if(retValue > 0x03) retValue = 0;	
	if (retValue == 0x03){
		cfg_TimeDelay = MY_MINIMUM;	//1.0sec 
	}
	
	return retValue;
}
int main(void)
{   
	debug_init(); 
	timer_init(MY_TIC_TIME);
	usiTwi_Slave_init(MY_ADDRESS);
	
	set_sleep_mode(SLEEP_MODE_IDLE);
	wdt_enable(WDTO_8S);
	
	nowMODE = getSwitch();

	uint16_t targetAMP = cfg_Bright[nowMODE];
	while(1)
    {
		if(BIT_CHECK(gWakeUpFlag,_BIT_I2C))	{
			BIT_CLEAR(gWakeUpFlag,_BIT_I2C);
			usiTwi_is_Stop();
		}
		if(BIT_CHECK(gWakeUpFlag,_BIT_TIC))	{
			BIT_CLEAR(gWakeUpFlag,_BIT_TIC);
			//main work here!
		}
		/* */
		while( !gWakeUpFlag ) {
			cli();
			sleep_enable();
			sei();
			sleep_cpu();
			sleep_disable();
		}
		/* */
		//------------------------------
		//PORT_SW = PIN_SW^(1<<_TP);
		//------------------------------		
    }
}