/*
 * myTimer.h
 *
 * Created: 4/10/2017 1:45:41 PM
 *  Author: min
 */ 


#ifndef MYTIMER_H_
#define MYTIMER_H_

#define PORT_PWM	PORTA
#define PIN_PWM		PINA
#define DDR_PWM		DDRA


void timer_init(uint16_t tic_time);

void pwm_init(uint8_t channel);
void pwm_write(bool inc, uint8_t step);

#endif /* MYTIMER_H_ */