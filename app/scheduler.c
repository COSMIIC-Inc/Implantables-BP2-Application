
//    pulseGen: .c    .

/*	HOW IT WORKS:  Stim runs within the 1msec Timer0 interrupt, to assure timeliness.
	Here we generate the pulse waveforms on each channel per frame rates.
	Use the resident config functions to configure pulses and frame rates.

	THE ISR USES SPI TO CONFIG AMPLITUDE DAC SO BLOCK INTERRUPTS AROUND BACKGROUND
	DAC USAGE^^;
*/


#include <stdio.h>
#include <stdlib.h>
#include "sys.h"
#include "pulseGen.h"
#include "app.h"
#include "objdict.h"
#include "scheduler.h"
#include "stimTask.h"
#include "can_AVR.h"

// -------- DEFINITIONS ----------

#define TIME_CUSHION 10

// --------   DATA   ------------
volatile UINT8 workingFlag = 0;
volatile UINT8 counter = 0;
volatile UINT8 s_index  = 0;
volatile void (*funcArray[10])( UINT8 );
static UINT32 sysTimer;
volatile unsigned char startPulse = 0;


/*!
**
**
** @param none
** 
** @return
**/
void InitScheduler(void)
{
        
        
        /* create 1msec system tick on TMR0*/
	OCR0A = 125 - 1 ;					// TMR2: 1mSec rollover timer for RTI
	TCCR0A = B(WGM01) ;					// CTC no output pin
	TIMSK0 = B(OCIE0A) ;						// enable timer2 OC interrupt
	
	TCCR0A |= B(CS01) | B(CS00) ;		// START clock at 8MHz/64 = 8us period
        
        funcArray[0] = &runStimTask;
        funcArray[1] = &runStimTask;
        funcArray[2] = &runStimTask;
        funcArray[3] = &runStimTask;
        funcArray[4] = &StimPulse;
        funcArray[5] = &StimPulse;
        funcArray[6] = &StimPulse;
        funcArray[7] = &StimPulse;
}

//============================
//    APP TIMEOUT ACCESSORS
//============================


UINT32 getSystemTime( void )
{
	UINT32 t;
	
	DISABLE_INTERRUPTS();
	
	 t = sysTimer;
	 
	ENABLE_INTERRUPTS();
	
	return t;
	
}
UINT8 isTimedOut( UINT32 *tRef, UINT32 tAlarm )
{
	
	if( (getSystemTime() - *tRef) > tAlarm )
	{
		return 1;
	}
	
	else
		return 0;
}


void resetTimeOut( UINT32 *tRef )
{
	*tRef = getSystemTime();
}


//============================
//    INTERRUPT SERVICE ROUTINES
//============================
#pragma vector=TIMER0_COMP_vect
// 	This is a 1msec continuous tick to initiate pulses and 
	//	maintain system time.

__interrupt void stimTick_ISR(void)
{
  // make sure everything is finished, and the startTick is set.    
  if ( workingFlag || !startPulse )
        return;
      workingFlag = 1;
      
      
      // passing index is SPECIFIC to the implementation
      // check on destination of the parameter.
      
      if (counter == SyncTiming[s_index])
      {
         (*funcArray[s_index])(s_index);
         
         if (s_index < sizeof(SyncTiming))
            s_index++;
      }
  
  
  /* increment and check the period counter */
	if( counter >= frame.period - TIME_CUSHION)
	{
		counter = 0;	
                s_index = 0;
                startPulse = 0;  // get ready for next sync pulse
		PORTA ^= BIT7;  // blink light
                workingFlag = 0;
                return;

	}
	
        sysTimer++ ;
        counter++;
        workingFlag = 0; // clear the finish flag
}