/*
 * myUsiTwiSlave.h
 *
 * Created: 4/10/2017 1:28:00 PM
 *  Author: min
 */ 


#ifndef MYUSITWISLAVE_H_
#define MYUSITWISLAVE_H_

#include "myConfig.h"

// permitted RX buffer sizes: 1, 2, 4, 8, 16, 32, 64, 128 or 256

#define DDR_USI             DDRA
#define PORT_USI            PORTA
#define PIN_USI             PINA
#define PORT_USI_SDA        PORTA6
#define PORT_USI_SCL        PORTA4
#define PIN_USI_SDA         PINA6
#define PIN_USI_SCL         PINA4
//#define USI_START_COND_INT  USISIF
#define USI_START_VECTOR    USI_STR_vect
#define USI_OVERFLOW_VECTOR USI_OVF_vect


#ifndef TWI_RX_BUFFER_SIZE
#define TWI_RX_BUFFER_SIZE  ( 16 )
#endif
#define TWI_RX_BUFFER_MASK  ( TWI_RX_BUFFER_SIZE - 1 )

#if ( TWI_RX_BUFFER_SIZE & TWI_RX_BUFFER_MASK )
#  error TWI RX buffer size is not a power of 2
#endif

// permitted TX buffer sizes: 1, 2, 4, 8, 16, 32, 64, 128 or 256

#ifndef TWI_TX_BUFFER_SIZE
#define TWI_TX_BUFFER_SIZE ( 16 )
#endif
#define TWI_TX_BUFFER_MASK ( TWI_TX_BUFFER_SIZE - 1 )

#if ( TWI_TX_BUFFER_SIZE & TWI_TX_BUFFER_MASK )
#  error TWI TX buffer size is not a power of 2
#endif


void SET_USI_TO_SEND_ACK(void );
void SET_USI_TO_READ_ACK(void );
void SET_USI_TO_START_CONDITION_MODE(void ); 
void SET_USI_TO_SEND_DATA(void ); 
void SET_USI_TO_READ_DATA(void ); 


void usiTwi_flushTxBuffers(void);
void usiTwi_flushRxBuffers(void);
//Initialize TWI Slave
void    usiTwi_Slave_init( uint8_t ownAddress);

uint8_t usiTwi_is_DataInTxBuffer(void);

//the bytes available that haven't been read yet
//return a byte from received buffer, on at a time
uint8_t usiTwi_is_DataInRxBuffer(void);


void usiTwi_ByteToTxBuffer(uint8_t tx_data);

//return a byte from received buffer, on at a time
uint8_t usiTwi_ByteFromRxBuffer( void );

void usiTwi_Set_TxBuffer(void);

void usiTwi_is_Stop(void);



#endif /* MYUSITWISLAVE_H_ */