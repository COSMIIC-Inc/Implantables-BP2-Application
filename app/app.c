/**
 * @file   app.c
 * @author JDC
 * @brief entry point
 * @details System Resource Usage:
 * - tmr1, adc: used to time ADC sample time
 * - adc isr.
 * - tmr3: used to time canfestival operations.
 * - tmr3 isr.
*/

/******************************************************************************
Project description:

// Svn Version:   $Rev: 132 $     $Date: 2019-02-26 16:05:13 -0500 (Tue, 26 Feb 2019) $
******************************************************************************/
#include "sys.h"
#include "canfestival.h"
#include "runcanserver.h"
#include "emgTask.h"
#include "app.h"
#include "eedata.h"
#include "objdict.h"
#include "acceltemp.h"

#define CO_ENABLE_LSS

/******************************DATA*****************************************/

UNS16 blink = 0;
UNS8 tempNodeID = 5;
UNS8 lastRequestedAddress = 0;
UNS8 commandByte = 0;
/*****************************************************************************/

/******************************PROTOTYPES***********************************/
void sys_init( void );
void initNodeIDSerialNumber( void );
void ReadMemory( void );
UNS8 ReadLocalFlashMemory(void);
UNS8 ReadEprom(void);
UNS8 WriteEEProm ( void );

/***************************************************************************/

/**
 * @brief Application entry point 
 */
void main( void )
{
	sys_init(); 
        
	if (CheckRestoreFlag())	
        {
	  //Load custom settings from EEPROM into the OD
          RestoreValues();        
        }
        else
        {
          //Use default settings from OD rather than what is stored in EEPROM
          //Then, overwrite EEPROM with these default settings
          SaveValues();
        }
        
	
	initEmgTask();
        initAccelerometer();
        initTimer(); // timer for CAN Festival interrupts and alarms
	InitCANServerTask(); // also sets up CAN network -- 
        //needs to run after RestoreValues
        
	while (TRUE)
	{     
		runEmgTask();
                
                updateAccelerometer(); 
                
                //updateDiagnostics();
                	
		runTemperatureTask();
                
                RunCANServerTask();
                

                if (blink++ > 2000)
                {
                  if (getState( &ObjDict_Data ) == Waiting) // reduce overhead on AI processing
                  {
                    Status_TestValue++;
                    ReadMemory();
                  }
                  
                  PORTG &=~BIT2;  //turn off heartbeat LED 
                  blink = 0;
                  
                  /*DiagnosticsEnabled is set in OD 0x3000.1
                   *DiagnosticsEnabled = 0: ADC monitoring of MES only (ADC2,ADC3)
                   *DiagnosticsEnabled = 1: ADC montioring of VIC and VIN only (ADC0,ADC1)
                   *DiagnosticsEnabled = 2: ADC montioring of OFF1 and OFF2 only (ADC4,ADC5, requires disabling JTAG)
                   *DiagnosticsEnabled = 3: A simple counter is passed to ch1 and ch2 instead of ADC values
                   *DiagnosticsEnabled = 4: (default) ADC monitoring of MES and VIC/VIN/OFF1/OFF2 (ADC4,ADC5, requires disabling JTAG)
                   */
                  if(DiagnosticsEnabled == 2 || DiagnosticsEnabled == 4 )
                  {
                    //if JTAG is enabled, disable it
                    if( !(MCUCR & (1<<JTD)) ) 
                    {
                      MCUCR |= (1<<JTD);                      
                      MCUCR |= (1<<JTD);
                    }
                  }
                  else
                  {
                    //if JTAG is disabled, enable it
                    if( (MCUCR & (1<<JTD)) ) 
                    {
                      MCUCR &= ~(1<<JTD);                      
                      MCUCR &= ~(1<<JTD);
                    }
                  }
                }
                else if (blink > 1920)
                {
                  PORTG |= BIT2;  //turn on heartbeat LED 
                }
                
                //cannot Idle, because starting Idle will initiate an ADC conversion

                				
	}
}


//==================================
//    SYSTEM SPI AND TIMER
//==================================

#define SPI_DONE()			(BITS_TRUE( SPSR, B(SPIF) ))

/**
 * @brief R/W one byte via the SPI port, full duplex.  
 *   Assumes SPCR & SPSR are configured.
 * @param d data to be written
 */
UINT8 txRxSpi( UINT8 d )
{ 
	SPDR = d;		// put tx data
    while( !SPI_DONE() );	// wait for xfer done
    return( SPDR );		// get rx data 
}



/**
 * @brief <BRIEF> isTimedOut
 * @param *tRef
 * @param tAlarm
 */
UINT8 isTimedOut( UINT32 *tRef, UINT32 tAlarm )
{
	
	if( (getSystemTime() - *tRef) > tAlarm )
	{
		return 1;
	}
	
	else
		return 0;
}

/**
 * @brief <BRIEF> resetTimeOut
 * @param *tRef
 */
void resetTimeOut( UINT32 *tRef )
{
	*tRef = getSystemTime();
}


//==================================
//    APP SPECIFIC CANFESTIVAL
//==================================

/**
 * @brief Lets application process SYNC message if necessary
 * @param *m 8 byte message sent with SYNC objects from PM
 */
void processSYNCMessageForApp(Message* m)
{
  //MES app doesn't use sync message
}

/**
 * @brief Lets application process NMT_Start_Nodes
 * @param *d  Pointer to the CAN data structure
 * @param *m 3 byte message sent with NMT objects from PM
 */
void StartNodesFunc(CO_Data* d, Message *m)
{
  if ( d->nodeState == Hibernate) 
    setState(d,Waiting);
  else
    setState(d,Unknown_state);
}

/**
 * @brief Lets application process NMT_Stop_Nodes
 * @param *d  Pointer to the CAN data structure
 * @param *m 3 byte message sent with NMT objects from PM
 */
void StopNodesFunc(CO_Data* d, Message *m)
{
  //no restrictions on state entry
  setState(d,Stopped); 
  InitEmgTaskValues();
}

/**
 * @brief Lets application process NMT_Enter_Waiting
 * @param *d  Pointer to the CAN data structure
 * @param *m 3 byte message sent with NMT objects from PM
 */
void EnterWaitingFunc(CO_Data* d, Message *m)
{
  //no restrictions on state entry
  setState(d,Waiting); 
  StopWatchDog( d );
  setEmgAutoSync(0);
  InitEmgTaskValues();
}

/**
 * @brief Lets application process NMT_Enter_Patient_Operation
 * @param *d  Pointer to the CAN data structure
 * @param *m 3 byte message sent with NMT objects from PM
 */
void EnterPatientOperationFunc(CO_Data* d, Message *m)
{
  //UNS8 Param1;
  if (d->nodeState == Waiting && (*m).data[1] == 0) //must be NMT broadcast and in waiting
  {
    //Param1 = (*m).data[2];
    setState(d,Mode_Patient_Control);
    //Control_CurrentGroup = Param1;
  }
}

/**
 * @brief Lets application process NMT_Enter_X_Manual
 * @param *d  Pointer to the CAN data structure
 * @param *m 3 byte message sent with NMT objects from PM
 */
void EnterXManualFunc(CO_Data* d, Message *m)
{ 
  //UNS8 Param1;
  if (d->nodeState == Waiting  && (*m).data[1] == 0) //must be NMT broadcast and in waiting
  {
    //Param1 = (*m).data[2];
    setState(d,Mode_X_Manual);
     //Control_CurrentGroup = Param1;  //JML: not sure this is used anywhere
    StartWatchDog(d, 10000);
  }
}

/**
 * @brief Lets application process NMT_Enter_Y_Manual
 * @param *d  Pointer to the CAN data structure
 * @param *m 3 byte message sent with NMT objects from PM
 */
void EnterYManualFunc(CO_Data* d, Message *m)
{  
  if (d->nodeState == Waiting && (*m).data[1] == 0) //must be NMT broadcast and in waiting
  {
    setState(d,Mode_Y_Manual);
    StartWatchDog(d, 10000);
  }
}

/**
 * @brief Lets application process NMT_Enter_Stop_Stim
 * @param *d  Pointer to the CAN data structure
 * @param *m 3 byte message sent with NMT objects from PM
 */
void EnterStopStimFunc(CO_Data* d, Message *m)
{  
  //no restrictions on state entry
  setState(d,Stopped); 
  InitEmgTaskValues();
}

/**
 * @brief Lets application process NMT_Enter_Patient_Manual
 * @param *d  Pointer to the CAN data structure
 * @param *m 3 byte message sent with NMT objects from PM
 */
void EnterPatientManualFunc(CO_Data* d, Message *m)
{
  //UNS8 Param1;
  if (d->nodeState == Waiting && (*m).data[1] == 0) //must be NMT broadcast and in waiting
  {
    //Param1 = (*m).data[2];
    setState(d, Mode_Patient_Manual);
     //Control_CurrentGroup = Param1;
     StartWatchDog(d, 10000);
  }
}

/**
 * @brief Lets application process NMT_Produce_X_Manual
 * @param *d  Pointer to the CAN data structure
 * @param *m 3 byte message sent with NMT objects from PM
 */
void EnterProduceXManualFunc(CO_Data* d, Message *m)
{
  if (d->nodeState == Waiting )
  {
     if ((*m).data[1] == 0 ) //broadcast
     {
       setState(d, Mode_Produce_X_Manual);
     }
     else 
     {
       setState(d, Mode_Produce_X_Manual);
       setEmgAutoSync( 1 );
     }
   //Control_CurrentGroup = Param1;
  }
}

/**
 * @brief Lets application process NMT_Enter_Record_X
 * @param *d  Pointer to the CAN data structure
 * @param *m 3 byte message sent with NMT objects from PM
 */
void EnterRecordXFunc(CO_Data* d, Message *m)
{
  UNS8 Param1;
  if (d->nodeState == Waiting)  //must be in waiting
  {
    Param1 = (*m).data[2];
    // high speed recording can only be invoked by addressing the module
    if ( (Param1 & 0x0F) == *d->bDeviceNodeId )
    {
      setState(d, Mode_Record_X);
      CollectHSData( Param1 >> 4); // select channel                   
    }                
  }
}

/**
 * @brief Command processor to let CE read various memory locations
 * @param none
 */
void ReadMemory(void)
{
      // clear status
        if (memorySelect > 0 )
          statusByteMemory = 0;
        
        if (memorySelect == 1)
          statusByteMemory = ReadLocalFlashMemory();
        else if (memorySelect == 4)
          statusByteMemory = ReadEprom();
        else if (memorySelect == 8)
          statusByteMemory = WriteEEProm();
        else if (memorySelect != 0)
          statusByteMemory = 4;
      
        memorySelect = 0;
       
}
/**
 * @brief Read flash memory in CPU
 * @param none
 */
UNS8 ReadLocalFlashMemory(void)
{
  int recordSize = 32;
  int RMDataSize = sizeof(ReadMemoryData);
  
  if (AddressRequest > (0x01FFFF - recordSize))
    return 2;
  
  if (lastRequestedAddress != AddressRequest || (triggerReadMemory == 1))
  { 
    // set pattern to clear if read
    memset( ReadMemoryData, 0xA5, sizeof(ReadMemoryData));   
    ReadLocalFlashData( AddressRequest, ReadMemoryData, recordSize );
    // now copy requested address into the top of the array
    ReadMemoryData[RMDataSize - 4] = (UNS8)AddressRequest;
    ReadMemoryData[RMDataSize - 3] = (UNS8)(AddressRequest >> 8);
    ReadMemoryData[RMDataSize - 2] = (UNS8)(AddressRequest >> 16);
    ReadMemoryData[RMDataSize - 1] = (UNS8)(AddressRequest >> 24);
    
    lastRequestedAddress = AddressRequest;
    triggerReadMemory = 0;
    return 0;
  }   
  return 1;
}
/**
 * @ingroup eeprom
 * @brief used with handshake through the OD at 0x2020 to allow remote reading of
 *        values in the AVR eeprom memory
 */
UNS8 ReadEprom(void)
{
  UNS8 RMDataSize = sizeof(ReadMemoryData);
  UNS32 recordSize = 32;
  if (AddressRequest > (MAX_EEPROM_MEMORY - EEPROM_RECORD_SIZE))
    return 2;
  
  if (lastRequestedAddress != AddressRequest || (triggerReadMemory == 1))
  { 
    // set pattern to clear if read
    memset( ReadMemoryData, 0xA5, sizeof(ReadMemoryData));   
    EEPROM_read( (UNS16)AddressRequest, ReadMemoryData, recordSize );
    // now copy requested address into the top of the array
    ReadMemoryData[RMDataSize - 4] = (UNS8)AddressRequest;
    ReadMemoryData[RMDataSize - 3] = (UNS8)(AddressRequest >> 8);
    ReadMemoryData[RMDataSize - 2] = (UNS8)(AddressRequest >> 16);
    ReadMemoryData[RMDataSize - 1] = (UNS8)(AddressRequest >> 24);
    
    lastRequestedAddress = AddressRequest;
    triggerReadMemory = 0;
    return 0;
  }   
  return 1;
}
/**
 * @ingroup eeprom
 * @brief used with handshake through the OD at 0x2020 to allow remote writing of
 *        a single value in the AVR eeprom memory
 */
UNS8 WriteEEProm ( void )
{
 
  UNS8 status = 0;
  
  if (AddressRequest >= 0x1000) // nominal address space
    status = 2;
  else
    EEPROM_write((UINT16)AddressRequest, &writeByteMemory, 1);
  
  return status; 
}
//==================================
//    SYSTEM INIT
//==================================

/**
 * @brief Sets NodeId and S/N based on settings from bootloader.  Also has important role 
 * of delaying continuation based on node number
 */
void initNodeIDSerialNumber( void )
{
    UINT32 delay = START_DELAY_MS*1000UL; //convert to clock cycles, still running at Bootloader clock speed (1MHz)
           
    tempNodeID = *(UINT8   __farflash  *)0x1DF00;
    if (tempNodeID < 0x0F && tempNodeID > 0)
    {
      Status_NodeId = tempNodeID;
    }
    
    setNodeId(&ObjDict_Data, Status_NodeId);
    
    ObjDict_obj1018_Serial_Number = *(UINT8 __farflash *)0x1FFFE + \
      ((UINT16)(*(UINT8 __farflash *)0x1FFFD) << 8);
    
    //startup delay is START_DELAY_MS*(nodeID-1), 
    delay = delay*(Status_NodeId-1)/13;
    
    PORTE |= BIT1; //DEBUG ONLY set PE1 high
    while( delay-- ); //this line takes 13 clock ticks per iteration if delay is uint32
    PORTE &=~ BIT1; //DEBUG ONLY set PE1 low
}

/**
 * @brief Initiaize system IO and clock settings
 */
void sys_init( void )
{
	// initialize io pins to support MES PON default conditions
  
	// disable the wdt if it was enabled from the reset
        DISABLE_INTERRUPTS();
        WDTCR = (1 << WDCE)  | (1 << WDE);	
	WDTCR = 0; // clear the WDT enable bit
        
/*	//OLD PIN SETTINGS
	PORTA = 0x00;		// 
	DDRA  = 0xff;		// 1=output dir

	PORTB = 0x00;		// SPI MASTER, SS MUST BE OUTPUT OR MASTER WILL FAIL
	DDRB  = 0xf7;		// 1=output dir

	PORTC = 0x03;		// DIGPOT, tempr selects
	DDRC  = 0xff;		// 1=output dir

	PORTD = 0x00;		// accelr I2C, txRxCan
	DDRD  = 0x9C;		// 1=output dir

	PORTE = 0x03;		// tx, rx uart
	DDRE  = 0xfe;		// 1=output dir
	
	PORTF = 0x00;		// adc inputs wo/ pullups, jtag
	DDRF  = 0xf0;		// 1=output dir

	PORTG = 0x00;		// heartbeat led
	DDRG  = 0xff;		// 1=output dir
*/
        //SEE PRJ-NNPS-SYS-SPEC-21 Bootloader and App Specification, Remote Module, Pin Settings
        //Set unconnected pins (N.C.) to input with Pull-ups enabled
        //DDRx:  0=input, 1=output
        //PORTx: 0=low, 1=high (output), 0=no pullup, 1=pullup (input)
        
	PORTA = 0xFF;		// All N.C, so pullup
	DDRA  = 0x00;		// All N.C so input

	PORTB = 0xE7;		// MISO not pulled-up, AVDDCTRL set low
	DDRB  = 0x16;		// MOSI and SCK are outputs, AVDDCTRL is output

	PORTC = 0xFF;		// pulled-up or set high
	DDRC  = 0xC9;		// nCSLOGPOT, LFCH1, LFCH2 are outputs.  For BP2C, n3V3CTRL is output

	PORTD = 0xB8;		// SCL, SDA, ACCINT, and RXCAN not pulled up
	DDRD  = 0x20;		// TXCAN is output

	PORTE = 0xFF;		// All N.C. (TXD0, RXD0 unused), so pullup (see DIDR1)
	DDRE  = 0x00;		// All N.C so input
	
	PORTF = 0xC0;		// ADC inputs not pulled up, see DIDR0
	DDRF  = 0x40;		// TDO is output

	PORTG = 0x1B;		// HEARTBEAT low
	DDRG  = 0x04;		// HEARTBEAT is output
	
	DIDR0 = 0x3F;		// Disconnect digital I/O pins from ADC inputs (see PORTF)
	DIDR1 = 0x00;		// Connect I/O pins (disconnected in BL, see PORTE)
	
//	//>>For Debug ONLY!
//	//remap PORTA as outputs, low
	PORTA = 0x00;		
	DDRA  = 0xFF;	
        DDRE |= 0x03; //set PE0,1 (TX, RX) as output
        PORTE &=~ 0x03; //set PE0,1 (TX, RX) low
//        //<<For Debug ONLY!
        
        
        CANGCON &=~ B(ENASTB); //put CAN in standby while adjusting clock
        //while(CANGSTA & B(ENFG)){}; //wait until disabled;
        //CAN will be reenabled by canInit routine after bittimings have been set for new clock speed
        
        initNodeIDSerialNumber(); //initialize serial number, node number
                                  //delay based on node number before increasing clock
        
        CLKPR = B(CLKPCE); // enable scale clock, Interrupts must be off.
                           // Next step takes 4 clock cycles
        #if (FOSC == 8000)
           CLKPR = 0;// write sysclk prescaler 
        #endif
        #if (FOSC == 4000)                    
           CLKPR = 1;// write sysclk prescaler 
        #endif
        #if (FOSC == 2000)                     
           CLKPR = 2;// write sysclk prescaler 
        #endif  
        #if (FOSC == 1000)                     
           CLKPR = 3;// write sysclk prescaler 
        #endif
	

        ENABLE_INTERRUPTS();
        

	

/* THIS INIT CODE WORKS FOR DVK
	
	PORTA = 0x0f;		// chan selects
	DDRA  = 0xff;		// 1=output dir

	//SPI SS PB0 must be configed as output pin or master mode could fail
	PORTB = 0x11;		// miso=in, stim pulse control
	DDRB  = 0xf7;		// 1=output dir

	PORTC = 0x03;		// dac, vos, tempr selects
	DDRC  = 0xff;		// 1=output dir

	
	PORTE = 0x02;		// tx, rx selects
	DDRE  = 0xfe;		// 1=output dir
	
	PORTF = 0x00;		// adc inputs wo/ pullups
	DDRF  = 0xf0;		// 1=output dir
*/
	
	
}

