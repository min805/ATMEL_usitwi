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
#include "myADC.h"


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
uint8_t nowMODE = 0; //For Debugging
uint16_t nowAMP = 0;

void debug_init(void);
uint8_t mode_init(void);
void counter_reset(void);
bool counter_is_done(uint8_t times);
uint16_t set_targetAMP(uint16_t targetI, uint8_t mode);
void set_PWM(uint16_t targetI, uint8_t mode);


/*************************************************
***************************************************/
ISR(TIM1_COMPA_vect)
{
	wdt_reset();
	if(BIT_CHECK(gCountFlag,_BIT_SLOPE_ON)){
		BIT_CLEAR(gCountFlag,_BIT_SLOPE_ON);
		BIT_SET(gWakeUpFlag,_BIT_TIC);
	}		
	if(++ticCount >= TIC_FOR_1SEC){
		BIT_SET(gWakeUpFlag,_BIT_SEC);
		ticCount = 0;
		if(++secCount == 255) secCount = 0;	//secCount = 0~254
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

void set_gWakeUpFlag_adc(void)
{
	BIT_SET(gWakeUpFlag,_BIT_ADC);
}

/*************************************************
callback function for update nowAMP
***************************************************/
void call_set_nowAMP(uint16_t adcValue)
{
	nowAMP = adcValue;
	//set_gWakeUpFlag_adc();	
	BIT_SET(gWakeUpFlag,_BIT_ADC);
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
	
	cli();
	for(i=0; i < amount; i++)
	{
		data = usiTwi_ByteFromRxBuffer();
		if(state == STATE_COMMAND){
			//-------------------------------
			usiTwi_flushTxBuffers();
			//-------------------------------
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
	sei();	
} //

/*************************************************
	Local method
***************************************************/
void debug_init(void)
{
	PORT_SW &= ~(1<<_TP);	//Pull-up
	DDR_SW |= (1<<_TP);		//0=input, 1=output(Save energy)
}

/*************************************************/
uint8_t mode_init(void)
{
	uint8_t retValue = 0;
	//pull up, input
	//MCUCR &= ~(1<<PUD);
	//Add external full up!!
	PORT_SW |= ~((1<<_SW0)|(1<<_SW1));// tri-state(hi-z)
	DDR_SW  &= ~((1<<_SW0)|(1<<_SW1));
	
	if(PINB & (1<<_SW0)){ retValue |= 0x01;}
	if(PINB & (1<<_SW1)){ retValue |= 0x02;}
	
	if (retValue == 0x03){
		cfg_TimeDelay = MY_MINIMUM;	//1.0sec 
	}else if(retValue > 0x03){
		 retValue = 0;
	}
	return retValue;
}

/*************************************************/
//void counter_start(void)
//{
//	secCount = 0;
//	BIT_SET(gCountFlag,_BIT_COUNT_ON);
//	BIT_CLEAR(gCountFlag,_BIT_COUNT_DONE);
//}
void counter_reset(void)
{
	secCount = 0;
	BIT_CLEAR(gCountFlag,_BIT_COUNT_ON);
	BIT_CLEAR(gCountFlag,_BIT_COUNT_DONE);
}
bool counter_is_done(uint8_t times)
{
	bool retValue = false;
	if(BIT_CHECK(gCountFlag,_BIT_COUNT_ON)){
		if(BIT_CHECK(gCountFlag,_BIT_COUNT_DONE)){
			return true;
		}
		if(secCount >= times){
			BIT_SET(gCountFlag,_BIT_COUNT_DONE);
			retValue = true;
		}
	}else{
		//Start the counter
		secCount = 0;
		BIT_SET(gCountFlag,_BIT_COUNT_ON);
		BIT_CLEAR(gCountFlag,_BIT_COUNT_DONE);
	}
	return retValue;
}

/*************************************************/
uint16_t set_targetAMP(uint16_t targetI, uint8_t mode)
{
	uint16_t retAMP = targetI;
	nowPIR = adc_io_read(_InSensor);
	if(nowPIR){	//Got Object
		retAMP = cfg_Bright[mode];
		counter_reset();
	}else{
		if(counter_is_done(cfg_TimeDelay)){
			retAMP = cfg_Dimm[mode];
		}
	}	
	return retAMP;
} 

/*************************************************/
void set_PWM(uint16_t targetI, uint8_t mode)
{
	uint8_t step;
	
	if(nowAMP > targetI){
	//-Current going down -> PWM up!
		if((nowAMP-targetI)>cfg_TimeDn){step = cfg_TimeDn;
		}else{			step = MY_MINIMUM;		}
		BIT_SET(gCountFlag,_BIT_SLOPE_ON);	
		pwm_write(true,step);//pwm_write(false,step);	
	}else if(nowAMP < targetI){
	//-Current going up -> PWM down!
		if((targetI-nowAMP)>cfg_TimeUp[mode]){step = cfg_TimeUp[mode];
		}else{			step = MY_MINIMUM;		}
		BIT_SET(gCountFlag,_BIT_SLOPE_ON);
		pwm_write(false,step);//pwm_write(true,step);		
	}
	//else{
	//	BIT_CLEAR(gCountFlag,_BIT_SLOPE_ON);
	//}
}

/*************************************************
***************************************************/
int main(void)
{   
	uint16_t targetAMP;
	
	debug_init(); 
	timer_init(MY_TIC_TIME);
	usiTwi_Slave_init(MY_ADDRESS);	
	pwm_init(_OutPWM);
	adc_init();
	adc_io_init(_InSensor);

	set_sleep_mode(SLEEP_MODE_IDLE);
	wdt_enable(WDTO_8S);

	adc_start(_InCurrent);	
	nowMODE = mode_init();
	targetAMP = cfg_Bright[nowMODE];
	sei();
	while(1)
    {
		if(BIT_CHECK(gWakeUpFlag,_BIT_I2C))	{
			BIT_CLEAR(gWakeUpFlag,_BIT_I2C);
			usiTwi_is_Stop();
		}
		if(BIT_CHECK(gWakeUpFlag,_BIT_ADC))	{
			BIT_CLEAR(gWakeUpFlag,_BIT_ADC);
			targetAMP = set_targetAMP(targetAMP,nowMODE);
			set_PWM(targetAMP,nowMODE);
		}
		if(BIT_CHECK(gWakeUpFlag,_BIT_SEC))	{
			BIT_CLEAR(gWakeUpFlag,_BIT_SEC);
			BIT_CLEAR(gWakeUpFlag,_BIT_TIC);
		//------------------------------
		PORT_SW = PIN_SW^(1<<_TP);
		//------------------------------			
			adc_start(_InCurrent);					
		}
		if(BIT_CHECK(gWakeUpFlag,_BIT_TIC))	{
			BIT_CLEAR(gWakeUpFlag,_BIT_TIC);
			adc_start(_InCurrent);			
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
		
    }
}