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


uint8_t gWakeUpFlag = 0;
uint8_t gCountFlag  = 0;
uint8_t ticCount = 0;
uint8_t secCount = 0;
//......................TEST...low,..mod...hi.. 
//uint16_t cfg_Bright[4]={BRT_H,BRT_L,BRT_M,BRT_H};	//ADC value
//uint16_t cfg_Dimm[4]  ={DIM_H,DIM_L,DIM_M,DIM_H};	//ADC value
//uint16_t cfg_Dimm2[4]  ={DIM2_H,DIM2_L,DIM2_M,DIM2_H};	//ADC value	
//uint8_t cfg_TimeUp[4] ={TIMEUP_H,TIMEUP_L,TIMEUP_M,TIMEUP_H};	//PWM value
uint16_t table_Bright[4]={BRT_H,BRT_L,BRT_M,BRT_H};	//ADC value
uint16_t table_Dimm0[4] ={DIM_H,DIM_L,DIM_M,DIM_H};	//ADC value
uint16_t table_Dimm1[4] ={DIM2_H,DIM2_L,DIM2_M,DIM2_H};	//ADC value
uint8_t  table_TimeUp[4]={TIMEUP_H,TIMEUP_L,TIMEUP_M,TIMEUP_H};	//PWM value

uint16_t cfg_Bright  = 0;
uint16_t cfg_Dimm    = 0;
uint16_t cfg_TimeUp  = 0;
uint8_t cfg_TimeDn	  = TIMEDOWN;				//PWM value
uint8_t cfg_TimeDelay = TIME_DELAY;				//SEC value

bool nowPIR = true;
uint8_t nowDIMLevel = 0;
uint8_t nowMODE = 0; //For Debugging
uint16_t nowAMP = 0;
uint16_t targetAMP= 0;

void debug_init(void);
void mode_init(void);
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

//void set_gWakeUpFlag_adc(void)
//{
//	BIT_SET(gWakeUpFlag,_BIT_ADC);
//}

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
				usiTwi_ByteToTxBuffer((uint8_t)((cfg_Bright>>8)&0x00FF) );	
				usiTwi_ByteToTxBuffer((uint8_t)(cfg_Bright&0x00FF) );						
				usiTwi_ByteToTxBuffer((uint8_t)((cfg_Dimm>>8)&0x00FF) );
				usiTwi_ByteToTxBuffer((uint8_t)(cfg_Dimm&0x00FF) );												
				usiTwi_ByteToTxBuffer(cfg_TimeUp);
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
			case GET_NOWDIM:
			usiTwi_ByteToTxBuffer(0);
			usiTwi_ByteToTxBuffer(nowDIMLevel);
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
				cfg_Bright = (data_h<<8)&0xff00;
				cfg_Bright |= data;
			break;
			case SET_DIMM:				
				cfg_Dimm = (data_h<<8)&0xff00;
				cfg_Dimm |= data;
			break;
			case SET_TIMEUP:
				cfg_TimeUp = data;
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
//	PORT_SW &= ~(1<<_TP);	//Pull-up
//	DDR_SW |= (1<<_TP);		//0=input, 1=output(Save energy)
}

/*************************************************/
void mode_init(void)
{
	uint8_t retMode = 0;
	//pull up, input
	//MCUCR &= ~(1<<PUD);
	//Add external full up!!
	PORT_SW |= ~((1<<_SW0)|(1<<_SW1)|(1<<_DIM));// tri-state(hi-z)
	DDR_SW  &= ~((1<<_SW0)|(1<<_SW1)|(1<<_DIM));

	if(PINB & (1<<_DIM)){	nowDIMLevel=1;		//50%
	}else{					nowDIMLevel=0;	}	//30%	
	
	if(PINB & (1<<_SW0)){ retMode |= 0x01;}
	if(PINB & (1<<_SW1)){ retMode |= 0x02;}
	nowMODE = retMode & 0x03;
	
	cfg_Bright = table_Bright[nowMODE];
	if(nowDIMLevel == 0){	cfg_Dimm = table_Dimm0[nowMODE];
	}else{					cfg_Dimm = table_Dimm1[nowMODE];}
	cfg_TimeUp = table_TimeUp[nowMODE];
	
	if (nowMODE == 0x0){			//TEST MODE
		cfg_TimeDelay = MY_MINIMUM;	//1.0sec 
	}
	//back to pin out - save energy
	//DDR_SW |= (1<<_SW0)|(1<<_SW1)|(1<<_DIM);

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
	if(!nowPIR){	//Got Object
		retAMP = cfg_Bright;
		counter_reset();
	}else{
		if(counter_is_done(cfg_TimeDelay)){
			retAMP = cfg_Dimm;
		}
	}	
	return retAMP;
} 

/*************************************************/
void set_PWM(uint16_t targetI, uint8_t mode)
{
	uint8_t step;
	uint16_t nowI = nowAMP;
	if(nowI > (targetI) ){
	//-Current going down -> increase PWM duty!
		step = cfg_TimeDn;
		if((nowI-targetI)<= step){
			step = MY_MINIMUM;
		}
		BIT_SET(gCountFlag,_BIT_SLOPE_ON);	
		pwm_write(true,step);	
	}else if(nowI < (targetI)){
	//-Current going up -> decrease PWM duty!
		step = cfg_TimeUp;
		if ((targetI-nowI)<= step){
			step = MY_MINIMUM;
		}
		BIT_SET(gCountFlag,_BIT_SLOPE_ON);
		pwm_write(false,step);		
	}
	//else{
	//	BIT_CLEAR(gCountFlag,_BIT_SLOPE_ON);
	//}
}

/*************************************************
***************************************************/
int main(void)
{   	
	debug_init(); 
	timer_init(MY_TIC_TIME);
	usiTwi_Slave_init(MY_ADDRESS);	
	pwm_init(_OutPWM);
	adc_init();
	adc_io_init(_InSensor);
	mode_init();
	targetAMP = cfg_Bright;	//Set initial value, after mode_init()
	
	set_sleep_mode(SLEEP_MODE_IDLE);
	wdt_enable(WDTO_8S);

	adc_start(_InCurrent);		
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
			/////////////////////////////
			//adc_start(_InCurrent);
			/////////////////////////////
		}
		if(BIT_CHECK(gWakeUpFlag,_BIT_SEC))	{
			BIT_CLEAR(gWakeUpFlag,_BIT_SEC);
			BIT_CLEAR(gWakeUpFlag,_BIT_TIC);
		//------------------------------
		//PORT_SW = PIN_SW^(1<<_TP);
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