/*
 * myUsiTwiSlave.c
 *
 * Created: 4/10/2017 1:27:36 PM
 *  Author: min
 */ 

#include "myConfig.h"
#include "myUsiTwiSlave.h"

/********************************************************************************
                        functions implemented as macros
********************************************************************************/

#define SET_USI_TO_SEND_ACK( ) \
{ \
	/* prepare ACK */ \
	USIDR = 0; \
	/* set SDA as output */ \
	DDR_USI |= ( 1 << PORT_USI_SDA ); \
	/* clear all interrupt flags, except Start Cond */ \
	USISR = \
	( 0 << USISIF ) | ( 1 << USIOIF ) | ( 1 << USIPF ) | ( 1 << USIDC )| \
	/* set USI counter to shift 1 bit */ \
	( 0x0E << USICNT0 ); \
}

#define SET_USI_TO_READ_ACK( ) \
{ \
	/* set SDA as input */ \
	DDR_USI &= ~( 1 << PORT_USI_SDA ); \
	/* prepare ACK */ \
	USIDR = 0; \
	/* clear all interrupt flags, except Start Cond */ \
	USISR = \
	( 0 << USISIF ) | ( 1 << USIOIF ) | ( 1 << USIPF ) | ( 1 << USIDC ) | \
	/* set USI counter to shift 1 bit */ \
	( 0x0E << USICNT0 ); \
}

#define SET_USI_TO_START_CONDITION_MODE( ) \
{ \
	USICR = \
	/* enable Start Condition Interrupt, disable Overflow Interrupt */ \
	( 1 << USISIE ) | ( 0 << USIOIE ) | \
	/* set USI in Two-wire mode, no USI Counter overflow hold */ \
	( 1 << USIWM1 ) | ( 0 << USIWM0 ) | \
	/* Shift Register Clock Source = External, positive edge */ \
	/* 4-Bit Counter Source = external, both edges */ \
	( 1 << USICS1 ) | ( 0 << USICS0 ) | ( 0 << USICLK ) | \
	/* no toggle clock-port pin */ \
	( 0 << USITC ); \
	USISR = \
	/* clear all interrupt flags, except Start Cond */ \
	( 0 << USISIF ) | ( 1 << USIOIF ) | ( 1 << USIPF ) | \
	( 1 << USIDC ) | ( 0x0 << USICNT0 ); \
}

#define SET_USI_TO_SEND_DATA( ) \
{ \
	/* set SDA as output */ \
	DDR_USI |=  ( 1 << PORT_USI_SDA ); \
	/* clear all interrupt flags, except Start Cond */ \
	USISR    =  \
	( 0 << USISIF ) | ( 1 << USIOIF ) | ( 1 << USIPF ) | ( 1 << USIDC) | \
	/* set USI to shift out 8 bits */ \
	( 0x0 << USICNT0 ); \
}

#define SET_USI_TO_READ_DATA( ) \
{ \
	/* set SDA as input */ \
	DDR_USI &= ~( 1 << PORT_USI_SDA ); \
	/* clear all interrupt flags, except Start Cond */ \
	USISR    = \
	( 0 << USISIF ) | ( 1 << USIOIF ) | ( 1 << USIPF ) | ( 1 << USIDC ) | \
	/* set USI to shift out 8 bits */ \
	( 0x0 << USICNT0 ); \
}

/********************************************************************************
                                   typedef
********************************************************************************/
typedef enum
{
  USI_SLAVE_CHECK_ADDRESS               = 0x00,
  USI_SLAVE_SEND_DATA                   = 0x01,
  USI_SLAVE_1ST_CHECK_REPLY_SEND_DATA   = 0x02,
  USI_SLAVE_CHECK_REPLY_SEND_DATA		= 0x03,
  USI_SLAVE_REQUEST_DATA                = 0x04,
  USI_SLAVE_GET_DATA_AND_SEND_ACK       = 0x05
} overflowState_t;
static volatile overflowState_t overflowState;


/********************************************************************************
                                local variables
********************************************************************************/
static uint8_t          slaveAddress;

static uint8_t          rxBuf[ TWI_RX_BUFFER_SIZE ];
static volatile uint8_t rxHead;
static volatile uint8_t rxTail;
static volatile uint8_t rxCount;

static uint8_t          txBuf[ TWI_TX_BUFFER_SIZE ];
static volatile uint8_t txHead;
static volatile uint8_t txTail;
static volatile uint8_t txCount;



/********************************************************************************
                                local functions
********************************************************************************/
// flushes the TWI buffers
static void flushTwiBuffers(void)
{
  rxTail = 0;  rxHead = 0;  rxCount = 0;
  txTail = 0;  txHead = 0;  txCount = 0;
} 

/********************************************************************************
                                public functions
********************************************************************************/
// initialize USI for TWI slave mode

void usiTwiSlave_init(uint8_t ownAddress)
{
  flushTwiBuffers( );//<---------------------???

  slaveAddress = ownAddress;

  // In Two Wire mode (USIWM1, USIWM0 = 1X), the slave USI will pull SCL
  // low when a start condition is detected or a counter overflow (only
  // for USIWM1, USIWM0 = 11).  This inserts a wait state.  SCL is released
  // by the ISRs (USI_START_vect and USI_OVERFLOW_vect).

  //1. Set SCL and SDA as output
  DDR_USI |= ( 1 << PORT_USI_SCL ) | ( 1 << PORT_USI_SDA );
  //2. set SCL high
  PORT_USI |= ( 1 << PORT_USI_SCL );
  //3. set SDA high
  PORT_USI |= ( 1 << PORT_USI_SDA );

  //4. Set SDA as input
  DDR_USI &= ~( 1 << PORT_USI_SDA );
  //5. Set registers
  USICR =
       // enable Start Condition Interrupt
       ( 1 << USISIE ) |
       // disable Overflow Interrupt
       ( 0 << USIOIE ) |
       // set USI in Two-wire mode, no USI Counter overflow hold
       ( 1 << USIWM1 ) | ( 0 << USIWM0 ) |
       // Shift Register Clock Source = external, positive edge
       // 4-Bit Counter Source = external, both edges
       ( 1 << USICS1 ) | ( 0 << USICS0 ) | ( 0 << USICLK ) |
       // no toggle clock-port pin
       ( 0 << USITC );

  // clear all interrupt flags and reset overflow counter(USICNT3:0)
  USISR = ( 1 << USISIF ) | ( 1 << USIOIF ) | ( 1 << USIPF ) | ( 1 << USIDC ) | ( 0x0 << USICNT0);

}

uint8_t usiTwi_is_DataInTxBuffer(void)
{
	return txCount;
}
uint8_t usiTwi_is_DataInRxBuffer(void)
{
	return rxCount;
}



// put data(a byte) in the transmission buffer, wait if buffer is full
void usiTwi_ByteToTxBuffer(uint8_t tx_data)
{
	//wait for free space in buffer
	while ( txCount == TWI_TX_BUFFER_SIZE) ;

	// store data in buffer
	txBuf[ txHead ] = tx_data;
	txHead = ( txHead + 1 ) & TWI_TX_BUFFER_MASK;
	txCount++;
}

// return a byte from the receive buffer, wait if buffer is empty
uint8_t usiTwi_ByteFromRxBuffer(void)
{
	uint8_t rx_data;
	// wait for Rx data
	while ( !rxCount );

	rx_data = rxBuf [ rxTail ];
	rxTail = ( rxTail + 1 ) & TWI_RX_BUFFER_MASK;
	rxCount--;

	return rx_data;
}

/********************************************************************************
********************************************************************************/
void usiTwi_Set_TxBuffer(void)
{
	uint8_t amount = usiTwi_is_DataInRxBuffer();
	if(amount == 0){	return; }			//No data in buffer

	call_set_TxBuffer(amount);

	
}


void usiTwi_is_Stop(void)
{
	//if(!usi_onReceiverPtr()){ return; }	//No receive callback, Nothing to do
	if(!(USISR & (1<<USIPF))){	return; }	//Stop not detected

	usiTwi_Set_TxBuffer();
}
 
 
 
 
/********************************************************************************
                            USI Start Condition ISR
********************************************************************************/

ISR( USI_START_VECTOR )
{
  /*
  // This triggers on second write, but claims to the callback there is only *one* byte in buffer
  ONSTOP_USI_RECEIVE_CALLBACK();
  */
  /*
  // This triggers on second write, but claims to the callback there is only *one* byte in buffer
  USI_RECEIVE_CALLBACK();
  */

  // set default starting conditions for new TWI package
  overflowState = USI_SLAVE_CHECK_ADDRESS;

  //1. set SDA as input
  DDR_USI &= ~( 1 << PORT_USI_SDA );

  //2. wait for SCL to go low to ensure the Start Condition has completed (the
  // start detector will hold SCL low ) - if a Stop Condition arises then leave
  // the interrupt to prevent waiting forever - don't use USISR to test for Stop
  // Condition as in Application Note AVR312 because the Stop Condition Flag is
  // going to be set from the last TWI sequence  
       // SCL his high .........................// and SDA is low
  while (( PIN_USI & ( 1 << PIN_USI_SCL ) ) && !( ( PIN_USI & ( 1 << PIN_USI_SDA ) ) ) );
	// possible combinations
	//	sda = low	scl = low		break	start condition
	// 	sda = low	scl = high		loop
	//	sda = high	scl = low		break	stop condition
	//	sda = high	scl = high		break	stop condition
	
	
  if ( !( PIN_USI & ( 1 << PIN_USI_SDA ) ) ) {    // a Stop Condition did NOT occur
    USICR =
         // keep Start Condition Interrupt enabled to detect RESTART
         ( 1 << USISIE ) |
         // enable Overflow Interrupt
         ( 1 << USIOIE ) |
         // set USI in Two-wire mode, hold SCL low on USI Counter overflow
         ( 1 << USIWM1 ) | ( 1 << USIWM0 ) |
         // Shift Register Clock Source = External, positive edge
         // 4-Bit Counter Source = external, both edges
         ( 1 << USICS1 ) | ( 0 << USICS0 ) | ( 0 << USICLK ) |
         // no toggle clock-port pin
         ( 0 << USITC );

  }  else  {									// a Stop Condition did occur
    USICR =
         // enable Start Condition Interrupt
         ( 1 << USISIE ) |
         // disable Overflow Interrupt
         ( 0 << USIOIE ) |
         // set USI in Two-wire mode, no USI Counter overflow hold
         ( 1 << USIWM1 ) | ( 0 << USIWM0 ) |
         // Shift Register Clock Source = external, positive edge
         // 4-Bit Counter Source = external, both edges
         ( 1 << USICS1 ) | ( 0 << USICS0 ) | ( 0 << USICLK ) |
         // no toggle clock-port pin
         ( 0 << USITC );

  } // end if

  USISR =
       // clear interrupt flags - resetting the Start Condition Flag will release SCL
	   // set USI to sample 8 bits (count 16 external SCL pin toggles)
       ( 1 << USISIF ) | ( 1 << USIOIF ) | ( 1 << USIPF ) |( 1 << USIDC ) | ( 0x0 << USICNT0);


} // end ISR( USI_START_VECTOR )



/********************************************************************************
                                USI Overflow ISR
Handles all the communication.
Only disabled when waiting for a new Start Condition.
********************************************************************************/

ISR( USI_OVERFLOW_VECTOR )
{
  switch ( overflowState )
  {
	// -----------------------------------------------------------------------------
    // Address mode: check address and send ACK (and next USI_SLAVE_SEND_DATA) if OK,
    // else reset USI
    case USI_SLAVE_CHECK_ADDRESS:
      if ( ( USIDR == 0 ) || ( ( USIDR >> 1 ) == slaveAddress) )
      {
        //??? Note: USICR must be set to 14(0x1110) before releasing SCL
		if ( USIDR & 0x01 )	//----------------->1=[Master read] 
        {
          ///////////////////////////////////////
		  if( ! usiTwi_is_DataInTxBuffer() ){  usiTwi_Set_TxBuffer();  }		  
		  //////////////////////////////////////
          overflowState = USI_SLAVE_SEND_DATA; //Wait for send data~
		  
        } else {	//-------------------------> 0=[Master write] 
          overflowState = USI_SLAVE_REQUEST_DATA;
        }
        SET_USI_TO_SEND_ACK( ); //shift 1 bit
		
      } else {
        SET_USI_TO_START_CONDITION_MODE( );
      }
      break;

	// -----------------------------------------------------------------------------	  
	// -----------------------------------------------------------------------------
    // Master write data mode: check reply and goto USI_SLAVE_SEND_DATA if OK,
    // else reset USI
    case USI_SLAVE_CHECK_REPLY_SEND_DATA:
      if ( USIDR ){
        // if NACK, the master does not want more data
        SET_USI_TO_START_CONDITION_MODE( );
        return;
      }
      // from here we just drop straight into USI_SLAVE_SEND_DATA if the
      // master sent an ACK
	// -----------------------------------------------------------------------------
    // copy data from buffer to USIDR and set USI to shift byte
    // next USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA
    case USI_SLAVE_SEND_DATA:		/* #1 */
      // Get data from Buffer
      if ( txCount )
      {
        USIDR = txBuf[ txTail ];
        txTail = ( txTail + 1 ) & TWI_TX_BUFFER_MASK;
        txCount--;
      } else {
        // the buffer is empty
        SET_USI_TO_READ_ACK( ); // This might be neccessary sometimes see http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&p=805227#805227
        SET_USI_TO_START_CONDITION_MODE( );
        return;
      } // end if
      overflowState = USI_SLAVE_1ST_CHECK_REPLY_SEND_DATA;
      SET_USI_TO_SEND_DATA( );
      break;
	  
	// -----------------------------------------------------------------------------
    // set USI to sample reply from master
    // next USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA
    case USI_SLAVE_1ST_CHECK_REPLY_SEND_DATA:
      overflowState = USI_SLAVE_CHECK_REPLY_SEND_DATA;
      SET_USI_TO_READ_ACK( );
      break;

	// -----------------------------------------------------------------------------
	// -----------------------------------------------------------------------------
    // Master read data mode: set USI to sample data from master, next
    // USI_SLAVE_GET_DATA_AND_SEND_ACK
    case USI_SLAVE_REQUEST_DATA:
      overflowState = USI_SLAVE_GET_DATA_AND_SEND_ACK;
      SET_USI_TO_READ_DATA( );
      break;

	// -----------------------------------------------------------------------------
    // copy data from USIDR and send ACK
    // next USI_SLAVE_REQUEST_DATA
    case USI_SLAVE_GET_DATA_AND_SEND_ACK:
      // put data into buffer
      // check buffer size
      if ( rxCount < TWI_RX_BUFFER_SIZE )
      {
        rxBuf[ rxHead ] = USIDR;
        rxHead = ( rxHead + 1 ) & TWI_RX_BUFFER_MASK;
        rxCount++;
      } else {
        // overrun
        // drop data
      }
      // next USI_SLAVE_REQUEST_DATA
      overflowState = USI_SLAVE_REQUEST_DATA;
      SET_USI_TO_SEND_ACK( );
      break;

  } // end switch
	set_gWakeUpFlag_i2c();

} // end ISR( USI_OVERFLOW_VECTOR )