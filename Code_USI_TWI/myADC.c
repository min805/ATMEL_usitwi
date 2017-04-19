/*
 * myADC.c
 *
 * Created: 4/5/2017 10:47:44 AM
 *  Author: min
 */ 
#include "myConfig.h"
#include "myADC.h"



ISR(ADC_vect)
{
	static uint8_t adcCount = 0;
	static uint16_t adcValue = 0;
	adcValue += ADCW;				//ADCW = ADCL+ADCH
	if(++adcCount >= ADCDIVIDE){
		adcCount = 0;		
		ADCSRA &= ~( (1<<ADATE)|(1<<ADEN)|(1<<ADSC) );//ADC Disable	
		call_set_nowAMP(adcValue/ADCDIVIDE);	
		adcValue = 0;	
	}
}


/*************************************************
***************************************************/
void adc_init(void)
{
	ADMUX = 0;	//Voltage Reference = AVCC
	
	//ADCSRA = (1<<ADEN)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
	ADCSRA = (1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
	//ADEN: 0
	//ADC interrupt Enable
	//Prescale111: 128 (8MHZ/128)= 62.5KHZ
	//Prescale101: 32 (8MHZ/32)= 250KHZ
	ADCSRB = 0;
	//ADLAR=0,
}
/*************************************************
***************************************************/
void adc_start(uint8_t channel)
{
	ADMUX = ((ADMUX & 0xC0)| channel);	//Channel select C0=1100.0000
	ADCSRA |= ( (1<<ADATE)|(1<<ADEN)|(1<<ADSC) );	//AutoTrigger + Enable + start
	ADCSRB = 0;	// Free Running Mode
}


/*************************************************
   manual adc convert
***************************************************/
uint16_t adc_convert(uint8_t channel)
{
	uint8_t i = 0;
	uint16_t adcValue = 0;
	cli();

	ADCSRA |= (1<<ADEN);				//ADC Enable

	ADMUX = ((ADMUX & 0xC0)| channel);	//Channel select
	for(i=0;i < ADCDIVIDE;i++){
		ADCSRA |= (1<<ADSC);			//start conversion!!
		while(!(ADCSRA &(1<<ADIF)));	//wait until set ADIF
		ADCSRA |= (1<<ADIF);			//clear ADIF
		adcValue += ADCW;				//ADCW = ADCL+ADCH
		_delay_us(1);
	}
	ADCSRA &= ~(1<<ADEN);				//ADC Disable
	sei();
	return (adcValue/ADCDIVIDE);
}

/*************************************************
***************************************************/
void adc_io_init(uint8_t channel)
{
	BIT_CLEAR(PORT_ADC,channel);	//Tri-state
	BIT_CLEAR(DDR_SW,channel);	//0=input, 1=output(Save energy)
}


bool adc_io_read(uint8_t channel)
{
	bool retValue=true;
	if(BIT_CHECK(PINA,_InCurrent)){ 
		retValue = false;	//Invert !!
	}
	return retValue;
}