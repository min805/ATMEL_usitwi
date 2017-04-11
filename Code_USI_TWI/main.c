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

uint16_t cfg_Bright[3]={BRT_H,BRT_M,BRT_L};	//ADC value
uint16_t cfg_Dimm[3]  ={DIM_H,DIM_M,DIM_L};	//ADC value
uint8_t cfg_TimeUp[3] ={TIMEUP_H,TIMEUP_M,TIMEUP_L};	//PWM value
uint8_t cfg_TimeDn	  = TIMEDOWN;				//PWM value
uint8_t cfg_TimeDelay = TIME_DELAY;				//SEC value

bool nowPIR;
uint8_t nowMODE;
uint16_t nowAMP;

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
	static uint8_t command,data_h;


	for(i=0; i < amount; i++)
	{
		data = usiTwi_ByteFromRxBuffer();
		if(state == STATE_COMMAND){
			command = data;
			switch(command)
			{
				case MCMD_GET_CONFIG:
				usiTwi_ByteToTxBuffer((uint8_t)command);
				usiTwi_ByteToTxBuffer((uint8_t)(cfg_Bright[nowMODE]&0x00FF) );
				usiTwi_ByteToTxBuffer((uint8_t)((cfg_Bright[nowMODE]>>8)&0x00FF) );
				usiTwi_ByteToTxBuffer((uint8_t)(cfg_Dimm[nowMODE]&0x00FF) );
				usiTwi_ByteToTxBuffer((uint8_t)((cfg_Dimm[nowMODE]>>8)&0x00FF) );
				usiTwi_ByteToTxBuffer(cfg_TimeUp[nowMODE]);
				usiTwi_ByteToTxBuffer(cfg_TimeDn);
				usiTwi_ByteToTxBuffer(cfg_TimeDelay);
				state = STATE_COMMAND;
				break;
				case MCMD_GET_NOWAMP:
				usiTwi_ByteToTxBuffer((uint8_t)command);
				usiTwi_ByteToTxBuffer((uint8_t)(nowAMP&0x00FF));
				usiTwi_ByteToTxBuffer((uint8_t)((nowAMP>>8)&0x00FF));
				state = STATE_COMMAND;
				break;
				case MCMD_GET_NOWPIR:
				usiTwi_ByteToTxBuffer((uint8_t)command);
				usiTwi_ByteToTxBuffer(nowPIR);
				state = STATE_COMMAND;
				break;
				case MCMD_GET_NOWMOD:
				usiTwi_ByteToTxBuffer((uint8_t)command);
				usiTwi_ByteToTxBuffer(nowMODE);
				state = STATE_COMMAND;
				break;
				case MCMD_SET_BRIGHT:
				case MCMD_SET_DIMM:
				case MCMD_SET_TIMEUP:
				case MCMD_SET_TIMEDN:
				case MCMD_SET_DELAY:
				state = STATE_DATA_1;
				break;
				default:
				state = STATE_COMMAND;
			}
		}else if(state == STATE_DATA_1){
			data_h = data;
			state = STATE_DATA_2;
		}else if(state == STATE_DATA_2){
			switch(command)
			{
				case MCMD_SET_BRIGHT:
				cfg_Bright[nowMODE] = data_h;
				cfg_Bright[nowMODE] |= (((uint16_t)data)<<8)&0xff00;
				break;
				case MCMD_SET_DIMM:
				cfg_Dimm[nowMODE] = data_h;
				cfg_Dimm[nowMODE] |= (((uint16_t)data)<<8)&0xff00;
				break;
				case MCMD_SET_TIMEUP:
				cfg_TimeUp[nowMODE] = data;
				break;
				case MCMD_SET_TIMEDN:
				cfg_TimeDn = data;
				break;
				case MCMD_SET_DELAY:
				cfg_TimeDelay = data;
				break;
			} //end switch
			state = STATE_COMMAND;
		} //end if
	} //end for
} //

/*************************************************
***************************************************/
uint8_t getSwitch(void)
{
	uint8_t retValue = 0;
	if(PINB & (1<<_SW0)){ retValue |= 0x01;}
	if(PINB & (1<<_SW1)){ retValue |= 0x02;}
	//if(retValue > 0x03) retValue = 0;	
	return retValue;
}
int main(void)
{
    
	timer_init(MY_TIC_TIME);
	usiTwiSlave_init(MY_ADDRESS);
	set_sleep_mode(SLEEP_MODE_IDLE);
	wdt_enable(WDTO_8S);
	
	nowMODE = getSwitch();
	uint16_t targetAMP = cfg_Bright[nowMODE];
	while(1)
    {
		if(BIT_CHECK(gWakeUpFlag,_BIT_I2C))	{
			BIT_CLEAR(gWakeUpFlag,_BIT_I2C);
		}
		if(BIT_CHECK(gWakeUpFlag,_BIT_TIC))	{
			BIT_CLEAR(gWakeUpFlag,_BIT_TIC);
			
		}
		while( !gWakeUpFlag ) {
			cli();
			sleep_enable();
			sei();
			sleep_cpu();
			sleep_disable();
		}
		//------------------------------
		PORT_SW = PIN_SW^(1<<_TP);
		//------------------------------		
    }
}