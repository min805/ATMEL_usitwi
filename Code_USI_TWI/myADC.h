/* 
* MyADC.h
*
* Created: 4/3/2017 4:58:16 PM
* Author: min
*/
#ifndef __MYADC_H__
#define __MYADC_H__

#define ADCDIVIDE 16
// 10bit adc = 1023
// 1023 * 16 = 0x3FF0 (in 16bit)
// 1023 * 24 = 0x5FF8
// 1023 * 32 = 0x7FE0
// 1023 * 64 = 0xFFC0

void adc_init(void);
void adc_start(uint8_t channel);

uint16_t adc_convert(uint8_t channel);


void adc_io_init(uint8_t channel);
bool adc_io_read(uint8_t channel);

#endif //__MYADC_H__
