/**
 * @file   meas.c
 * @author Jay Hardway, JDC
 * @brief - HOW IT WORKS:  

	EMG is measured by the cpu adc, MuxIns 0 & 1.  EMG_CH1 = ADC2, EMG_CH2 =
	ADC3.  

	The EMG_CH1 is triggered by Timer1 (2khz) and EMG_CH2 is triggered by 
	software upon entry into the adcISR on EMG_CH1 reading done.  The idea is 
	to keep chan samples close together. 
    
	The BP2 measurement window includes up to 4 blanking intervals, a window 
	skew (delay time) and a PDO transmission time.  These timings are programmable 
	via the object dictionary, and are referenced to the system sync pulse in 
	1 msec resolutions.   
 
	Measurements and feature accumulations take place in a timed ISR (foreground), 
	using sequencing techniques.   Feature calculations, smoothing, and PDO sending 
	run in a background task; after being flagged to run by the sequencer.

	Many routines work with global variables to run faster.


	REWRITE LATER^^

	
	Both chans are serviced by the adcISR.  Selected EMG features run within 
	the adcISR on each new reading. 

	The list of features is configured elsewhere to save execution time 
	in the ISR.

        EMG readings are scaled to 8bits, from -128 to 127.
        
        Selectable Features include: (note selection is hard coded, not an OD entry)
        - Mean Absolute Value: similar to bin-integrated rectified value
            min: 0, max: 512 (bit shift down by 1 to get into uint8)
        - Mean Value: non-rectified average.  Over a large window, for any EMG signal this value should be zero
            min: -512, max: 512 (typical: 0) (bit shift down by 1 to get into uint8)
        - Zero Crossings: number of zero-crossings within a window 
            min: 0, max: samples in window (255 max) 
        - Slope Sign Changes: number of times the slope sign changes within a window
            min: 0, max: samples in window (255 max)
        - Waveform length: estimate of total signal length within a window
            min: 0, max: 1024 (consecutive samples are max/min) (bit shift down by 1 and saturate to get into uint8)
        
        all of above can be represented in 8,9, or 10 bits.  At gains greater than 3000 (wiper=13), the bottom bit 
        is insignificant anyways due to the noise floor. If we shift features down to uint8s 
        we can get more data across network.  However, calculations are done with the raw data as int16s to minimize
        round off error.

        Within the feature calculations, multiple windows can be averaged together 
        using a sliding average of 1-10 windows.  The more windows, the smoother the output will be.
        A new feature output will be generated at the end of every window (1-255 samples).  
        For example, a 200 sample window, with a sliding average of 5 windows, will
        output new feature values every 100ms, but will smooth out the data using the past 0.5 s of data

        The X value for the network is taken from the mean absolute value, but is scaled and offset (deadzone).  
        This is used to determine stimulation commands (Y values) using fixed patterns.

        PDOs to transmit data to PM are not executed within the ISR, but rather by a fixed rate sync pulse 
        from the PM.  The sync rate should be determined based on the time between new feature outputs 
        (i.e., the number of samples in a window).  Because the PM sync pulse and the BP2 2kHz interrupt are asynchronous,
        there is a possibility for a delay.  Don't set sync pulse faster than 10ms (equivalent to 20 samples)
        Exception: if the device is in RawSignal mode, the ISR will generate a PDO every 6 times through (every 3ms), 
        outputting 6 previous raw ADC values  for a single channel.  The PM side of the RawSignal mode
        is not completely implemented yet.  The PM will need ample buffering or will need to quickly output data on the radio.  

*/


#include <stdio.h>
#include <stdlib.h>
#include "sys.h"
#include "meas.h"
#include "app.h"
#include "nmtSlave.h"
#include "objdict.h"
#include "can_AVR.h"


// -------- DEFINITIONS ----------


#define START_VIC_MEAS() { ADMUX = B(ADLAR) | B(REFS0) | 0; /* ref=Avcc, MUX=0 */	\
                            TIFR1 = B(OCF1B);	  /* clr tmr1 event */				\
                            ADCSRA |= B(ADATE);	  /* arm trigger */					}

#define START_VIN_MEAS() { ADMUX = B(ADLAR) | B(REFS0) | 1; /* ref=Avcc, MUX=1 */	\
                            ADCSRA |= B(ADSC);    /* start conversion */			}


#define START_OFF1_MEAS() { ADMUX = B(ADLAR) | B(REFS0) | 4; /* ref=Avcc, MUX=4 */	\
							TIFR1 = B(OCF1B);	  /* clr tmr1 event */				\
							ADCSRA |= B(ADATE);	  /* arm trigger */					}

#define START_OFF2_MEAS() { ADMUX = B(ADLAR) | B(REFS0) | 5; /* ref=Avcc, MUX=5 */	\
                            ADCSRA |= B(ADSC);    /* start conversion */			}


#define START_ADC_MEAS(ch) { ADMUX = B(ADLAR) | B(REFS0) | ch; /* ref=Avcc, MUX=5 */	\
                            ADCSRA |= B(ADSC);    /* start conversion */			}


#define START_EMG1_MEAS() { ADMUX = B(ADLAR) | B(REFS0) | 2; /* ref=Avcc, MUX=2 */	\
                            TIFR1 = B(OCF1B);	  /* clr tmr1 event */				\
                            ADCSRA |= B(ADATE);	  /* arm trigger */					}

#define START_EMG2_MEAS() { ADMUX = B(ADLAR) | B(REFS0) | 3; /* ref=Avcc, MUX=3 */ \
                            ADCSRA |= B(ADSC);    /* start conversion */			}

#define SYNC_EMG_MEAS()   { ADCSRA &= ~B(ADATE);      /* disarm auto trgr */		\
                            while( ADCSRA & B(ADSC));	/* let cnvr finish */		}


#define CONFIG_SPI_DIGPOT()		SPCR = B(SPE) | B(MSTR)			// 8MHz/4 

#define SELECT_SPI_DIGPOT()		CLR_BITS( PORTC, BIT0 ) //BP2B board uses C.0 (BP2A C.1) for logpot chip select
#define DESELECT_SPI_DIGPOT()	SET_BITS( PORTC, BIT0 )         //BP2B board uses C.0 (BP2A C.1) for logpot chip select



#define NUM_MEAS_NORMAL_STEPS	11

#if(FOSC==8000)
  #define SAMPLES_PER_MS  2L
#else
  #define SAMPLES_PER_MS  1L
#endif

#define AUTO_SYNC_PERIOD 		(125 * SAMPLES_PER_MS)	// time based on sample period



// --------   DATA   ------------

static void channelOff( void );
UINT8 Meas1, Meas2;
UINT8 activeHSChannel = 0;
static UINT32 sysTimer;

// ------------------------------

/**
 * @ingroup emg
 * @brief Measurement Sequencer counts at sample rate; running the sampling
 *   and calculation machines relative to the system SyncPulse in msecs.
 */
static struct
{
	UINT8 count;					// msec count, restarted by sync pulse
	UINT8 calcCount;				// match count to run feature calcs
	UINT8 maxCount;					// period limit based 125msec @ sample freq 
	
	void (*samplep[ NUM_EMG_CHAN ])(void);	        // sampling process
	void (*calcp)(void);				// corresponding calc process
	
} MeasSeq = 
	{
		0, 240, 250,
		channelOff, channelOff,
		channelOff
	};


INT8 MeasRawBuffer[6];			// raw sample buffer: 
								// normal mode  chan1=0-2, chan2=3-5; 
								// raw dump mode chan1or2, =0-5

/**
 * @ingroup emg
 * @brief Normal measurement period includes up to 4 blanking states, defined
 *   as a list of state transition steps at sequencer match-counts.
 */ 
static struct
{
	UINT8 isBlanking;
	UINT8 *stepp;
	UINT8 stepList[ NUM_MEAS_NORMAL_STEPS ];	// state transition points
	
} MeasNormal[ NUM_EMG_CHAN ];


EMG_FEATURE EmgFeature[ NUM_EMG_CHAN ];


static struct MeasParams
{
  UINT8 gain;
  
} measParams[ NUM_EMG_CHAN ];




// -------- PROTOTYPES ----------


static void configNormalMeas( UINT8 chan );

static void collectNormalDataChan1( void );
static void collectNormalDataChan2( void );
static void postNormalDataCalc( void );

static void runSampleFeatures( EMG_FEATURE *feature, INT8 *sample );
static void accumMAV( void );
static void accumMV( void );
static void accumZX( void );
static void accumSSC( void );
static void accumWFL( void );



 
//============================
//    GLOBAL CODE
//============================
/**
 * @ingroup emg
 * @brief initializes Timer and ADC for EMG measurements
 */
void initEmgMeas( void )
{
  // note dig pots (EMG programmable HW gains) set in initEmgTask
   
  
  
  TCCR1A = 0 ;						// stop tmr, CTC no output pin
  TCCR1B = B(WGM12) ;	                                // CTC

  // 2kHz sampling rate is only possible if running at 8MHz, and not in Mode_Record_X.
  // When running at 1kHz instead of 2kHz the antialiasing (LP) filter should be set to
  // lower frequency.
  #if(FOSC==8000) //Max Feature Calc + PDO: ~2ms (in SyncTiming, set calc < period-3)
    if (getState(&ObjDict_Data) == Mode_Record_X)
    {
      OCR1A  = 1000 - 1 ;  //1000 usec rollover
      ANTIALIAS_EMG1_LF();
      ANTIALIAS_EMG2_LF();
    }
    else
    {
      OCR1A  = 500 - 1 ;  //500 usec rollover
      ANTIALIAS_EMG1_HF();
      ANTIALIAS_EMG2_HF();
    }
  #endif
  #if(FOSC==4000) //Max  Feature Calc + PDO: ~3ms (in SyncTiming, set calc < period-4)
      OCR1A  = 500 - 1 ;  //1000 usec rollover
      ANTIALIAS_EMG1_LF();
      ANTIALIAS_EMG2_LF();
  #endif
  #if(FOSC==2000) //Max Feature Calc + PDO: ~4ms (in SyncTiming, set calc < period-5)
      OCR1A  = 250 - 1 ; //1000 usec rollover
      ANTIALIAS_EMG1_LF();
      ANTIALIAS_EMG2_LF();
  #endif
    
  OCR1B  = OCR1A;
  TCCR1B |= B(CS11) ;					// START clock at FOSC/8: 
                                                        // 8MHz:1us, 4MHz:2us, 2MHz:4us
  
  /* config ADC */
  ADCSRB = B(ADTS2) | B(ADTS0);				// trigger on tmr1B ocr
  ADCSRA = B(ADEN) | B(ADIE) | B(ADPS2) | B(ADPS1);	// 125KHz
  //DIDR0 = 0x0f;						// disconnect digital i/o pins //JML set by sys_init
  
  ACSR = B(ACD);  //Disable the Analog Comparator
  
  START_EMG1_MEAS();
  
  
  // cobID is a reserved value for HS recording
 
  
  
}


/**
 * @ingroup emg
 * @brief sets the channel gains, called via NMT.  Sets OD gains
 * @param 
 */
void setGains(UINT8 ch1, UINT8 ch2 )
{
  
	struct MeasParams *meas;
	ODVAR __flash  *odVar;
	
  
        meas = &measParams[ 0 ];
        odVar = &ChanOdVarMap[ 0 ];
        if( meas->gain != ch1 )
        {
                meas->gain = ch1 ;
                *odVar->gain = ch1;
                setDigPot( ((meas->gain - 1) & 0x1f)  ); //ch1
        }
        
        meas = &measParams[ 1 ];
        odVar = &ChanOdVarMap[ 1 ];
        if( meas->gain != ch2 )
        {
                meas->gain = ch2 ;
                *odVar->gain = ch2;
                setDigPot( ((meas->gain - 1) & 0x1f) | 0x40  ); //ch2
        }
                

}

/**
 * @ingroup emg
 * @brief sets the channel gains, call on each node State change
 * @param runHSD if in High Speed (Raw) mode, features are not calculated
 */
void configMeasParams( UINT8 runHSD )
{
  
	struct MeasParams *meas;
	ODVAR __flash  *odVar;
	UINT8 j;
	
  
	for( j = 0 ; j < NUM_EMG_CHAN ; j++ )
	{
		meas = &measParams[ j ];
		odVar = &ChanOdVarMap[ j ];
		
		if( meas->gain != *odVar->gain )
		{
			meas->gain = *odVar->gain ;
			setDigPot( ((meas->gain - 1) & 0x1f) | (0x40 * j) );
		}
		
	}
	
	if( runHSD )
	{
		
	}
	else
	{
		configNormalMeas(0);
		configNormalMeas(1);
		MeasSeq.calcp = postNormalDataCalc ;
		
	}
        
}

/**
 * @ingroup emg
 * @brief configures normal measurement.  This function loads the blanking intervals from SyncTiming
 * into MeasNormal.stepList for each channel, and sets MeasSeq.calcCount. It also 
 * @param chan 
 */
static void configNormalMeas( UINT8 chan )
{
	UINT8 j, *srcp, *destp, step ;
	
	
	if( chan == 0 )
	{
		srcp 	= &SyncTiming[0];
		MeasNormal[0].isBlanking = 1;
		//MeasNormal[0].stepp = &MeasNormal[0].stepList[0] ; //JML:already done 2 lines down
		MeasSeq.samplep[0] = collectNormalDataChan1 ;
		destp = MeasNormal[0].stepp = &MeasNormal[0].stepList[0];
		
	}
	else
	{
		srcp 	= &SyncTiming[ NUM_MEAS_NORMAL_STEPS ];	//^^fornow
		MeasNormal[1].isBlanking = 1;
		//MeasNormal[1].stepp = &MeasNormal[1].stepList[0] ; //JML:already done 2 lines down
		MeasSeq.samplep[1] = collectNormalDataChan2 ;
		destp = MeasNormal[1].stepp = &MeasNormal[1].stepList[0];
	}
		
	
	for( j=0 ; j < NUM_MEAS_NORMAL_STEPS - 1 ; j++ )
	{
		step = *srcp++ ;
		
		if( step > 125 )
			step = 255 / SAMPLES_PER_MS ;
		
                 *destp++ = step * SAMPLES_PER_MS ;
	}
	
	*destp = 255 ;		// force blanking at end of list
	
	  MeasSeq.calcCount = SAMPLES_PER_MS * SyncTiming[ NUM_MEAS_NORMAL_STEPS * 2 ] ;
}

/**
 * @ingroup emg
 * @brief initializes measurement
 */
void startMeasSampling( void )
{
       CLR_BITS( PORTA, BIT3 );	//^^test
        //this also happens in postNormalDataCalc, but may not get called if calc time>Period
        MeasNormal[0].stepp = &MeasNormal[0].stepList[0] ;
	MeasNormal[0].isBlanking = 1;
	MeasNormal[1].stepp = &MeasNormal[1].stepList[0] ;
	MeasNormal[1].isBlanking = 1;
        
        MeasSeq.count = 0;
	
}

/**
 * @ingroup emg
 * @brief sets the digital pots (channel gains)
 * @param value
 */
void setDigPot( UINT8 value )  //JML: moved out of local code
{
  CONFIG_SPI_DIGPOT();
  
  SELECT_SPI_DIGPOT();
  
  txRxSpi( value );
  
  DESELECT_SPI_DIGPOT();
  
}

//============================
//    TIMER FOR ACCEL & TEMP
//============================
/**
 * @brief gets system time
 * @return t time
 */
UINT32 getSystemTime( void )
{
  UINT32 t;
  
  
  DISABLE_INTERRUPTS();
  
  t = sysTimer;
  
  ENABLE_INTERRUPTS();
  
  return t;
  
}

//============================
//    LOCAL CODE
//============================



  

/**
 * @ingroup emg
 * @brief 
 */
static void channelOff( void )
{}

/**
 * @ingroup emg
 * @brief if not blanking, put current chan1 measurement into buffer and accumulate features
 */
static void collectNormalDataChan1( void )
{
	
	if( MeasSeq.count == *MeasNormal[0].stepp )
	{
		MeasNormal[0].stepp++ ;
		MeasNormal[0].isBlanking ^= 1 ;
	}
	
	if( !MeasNormal[0].isBlanking )
	{
		SET_BITS( PORTE, BIT0 );	//JML debug
		
		MeasRawBuffer[2] = MeasRawBuffer[1];
		MeasRawBuffer[1] = MeasRawBuffer[0];
		MeasRawBuffer[0] = Meas1 - 128;
		
		runSampleFeatures( &EmgFeature[0], &MeasRawBuffer[0] );
		
	}
	else                                    //  
		CLR_BITS( PORTE, BIT0 );	//JML debug
		
}

/**
 * @ingroup emg
 * @brief if not blanking, put current chan2 measurement into buffer and accumulate features
 */
static void collectNormalDataChan2( void )
{
	
	if( MeasSeq.count == *MeasNormal[1].stepp )
	{
		MeasNormal[1].stepp++ ;
		MeasNormal[1].isBlanking ^= 1 ;
	}
	
	if( !MeasNormal[1].isBlanking )
	{
                SET_BITS( PORTE, BIT1 );	//JML debug
		
		MeasRawBuffer[5] = MeasRawBuffer[4];
		MeasRawBuffer[4] = MeasRawBuffer[3];
		MeasRawBuffer[3] = Meas2 - 128;
		
		runSampleFeatures( &EmgFeature[1], &MeasRawBuffer[3] );
                
	}
        else
              CLR_BITS( PORTE, BIT1 );	//JML debug
	
	
}

/**
 * @ingroup emg
 * @brief pass on samples to background and begin feature calculations.
 *   reinit measurement sample period.
 */
static void postNormalDataCalc( void )
{
	
	
	MeasNormal[0].stepp = &MeasNormal[0].stepList[0] ;
	MeasNormal[0].isBlanking = 1;
	MeasNormal[1].stepp = &MeasNormal[1].stepList[0] ;
	MeasNormal[1].isBlanking = 1;
	
	putEmgCalcData();
	
}



/**
 * @ingroup emg
 * @brief feature params are global to speed execution on a per-channel basis 
*/
struct
{
	EMG_FEATURE *feature;
	INT8        *sample;
} mPar;

/**
 * @ingroup emg
 * @brief features use global vars to reduce execution time.
 * @param *feature
 * @param *sample
*/
static void runSampleFeatures( EMG_FEATURE *feature, INT8 *sample )
{

	UINT8 isActive;
	
	
	mPar.feature = feature;
	mPar.sample  = sample;
	
	mPar.feature->numPoints++ ;
	mPar.feature->lastRaw = *sample;
	
	isActive = mPar.feature->select ;
	
	if( isActive & FE_MAV )
		accumMAV();
	
	if( isActive & FE_MV )
		accumMV();
	
	if( isActive & FE_ZX )
		accumZX();
	
	if( isActive & FE_SSC )
		accumSSC();
	
	if( isActive & FE_WFL )
		accumWFL();
	
	
}


/*----------------------
	FEATURE PROCESSORS

	These functions use global 
	pointers to speed execution.

	ASSUME: unused values in bufs = 0.
	
----------------------*/


/**
 * @ingroup emg
 * @brief Mean Absolute Value
 */
static void accumMAV( void )
{
	// Accumulate the working sum (unsigned)
	
	INT8 raw = *mPar.sample;
	
	
	if( raw < 0 )
		raw = -raw;
	
	mPar.feature->MavSum += (UINT8)raw ;
	
}

/**
 * @ingroup emg
 * @brief Mean Value
 */
static void accumMV( void )
{
	// Accumulate the working sum (signed)
	
	
	mPar.feature->MvSum += (INT16)*mPar.sample ;
	
}

/**
 * @ingroup emg
 * @brief Zero Crossings
 */
static void accumZX( void )
{
	// count ZeroCrossings (signed)
	
	INT8 *raw = mPar.sample ;
	INT8 diff;
	
	
	if( (*raw ^ *(raw + 1)) & BIT7 )
	{
		diff = *raw - *(raw + 1) ;
		
		if( diff < 0 )
			diff = -diff;
		
		if( diff >= mPar.feature->ZXthresh )
			mPar.feature->ZXcount++ ;

	}

}

/**
 * @ingroup emg
 * @brief Slope Sign Changes
 */
static void accumSSC( void )
{
	// count slope sign changes (signed)
	
	INT8 *raw = mPar.sample ;
	INT16 diff1, diff2;
	UINT8 thresh = mPar.feature->SSCthresh ;
	
	
	diff1 = (INT16)*raw - *(raw + 1) ;
	diff2 = (INT16)*(raw + 1) - *(raw + 2) ;
	
	if( (diff1 ^ diff2) & BIT15 )
	{
		if( diff1 < 0 )
			diff1 = -diff1 ;
		
		if( diff2 < 0 )
			diff2 = -diff2 ;
		
		
		if( (UINT8)diff1 >= thresh || (UINT8)diff2 >= thresh )
			mPar.feature->SSCcount++ ;
		
	}
	
}

/**
 * @ingroup emg
 * @brief Waveform Length
 */
static void accumWFL( void )
{
	// accum WaveFormLength, dist between consecutive samples (signed)
	
	INT8  *raw = mPar.sample ;
	INT16 diff;
	
	
	diff = (INT16)*raw - *(raw + 1) ;
	
	if( diff < 0 )
		diff = -diff ;
		
	mPar.feature->WflSum += (UINT8)diff ;
	
}



//============================
//    INTERRUPT SERVICE ROUTINES
//============================



#pragma vector=ADC_vect
/**
 * @ingroup emg
 * @brief EMG Interrupt, ADC triggered by timer 1, then triggered by completion of chan 1 
 */
__interrupt void emgMeas_ISR(void)
{
 	// Timing: ISR execTime: st0=5us, st1=10,40,81us (no chan,1chan,2chan)
 	// Timing HSD: ISR execTime: st0=5us, st1=22us
	// Timing: ISR entry periods: st0=500us, EMG2meas=120uSec
 	
 	static UINT8 state;
        static UINT8 ch = 0;
        static Message HSPdoBuffer;
        static UINT8 tempBuffer0[8];
        static UINT8 tempBuffer1[8];
        static UINT8 bufferCounter = 0;
	UINT8 i;
        static UINT8 testbyte = 0;
        
        
        
        SET_BITS(PORTA, BIT0); //^^test
        HSPdoBuffer.cob_id = 0x490 + Status_NodeId ;	
        HSPdoBuffer.rtr = 0;
        HSPdoBuffer.len = 8;
        
	SYNC_EMG_MEAS();
	
        // reset the buffer counter
        if (bufferCounter >= 8)
        {
            bufferCounter = 0;     
        }
        
	if( state == 0 )
	{
                SET_BITS(PORTA, BIT2); //^^test
		
                sysTimer++;  //this timer is used by the temperature and accelerometer tasks.
                //PORTA ^= BIT1; //^^test
                
                /* read chan1 and trigger chan2 */
                //Next channel can be triggered before reading current ADC value, because 
                //it takes some time to get new measurement

                
                if( DiagnosticsEnabled == 1 )
                {
                  START_VIN_MEAS();
                }
                else if( DiagnosticsEnabled == 2 )
                {
                  START_OFF2_MEAS();
                }
		else //DiagnosticsEnabled == 0, 3, or 4
                {
                  START_EMG2_MEAS();

                  
                
                }
		state = 1;

                if( DiagnosticsEnabled == 3 )
		  tempBuffer0[bufferCounter] =  Meas1 = Chan1Data_ADC = testbyte++;	
                else
                  tempBuffer0[bufferCounter] =  Meas1 = Chan1Data_ADC = ADCH;
                
                CLR_BITS(PORTA, BIT2); //^^test
                

	}
	else if (state ==1)
	{
                SET_BITS(PORTA, BIT3); //^^test
		
                /* read chan2 and process both readings, and enable ch1 or diagnostics */
                //Next channel can be triggered before reading current ADC value, because 
                //it takes some time to get new measurement
                //only trigger diagnostics ADC read if we're not calculating or sending out PDO during this ISR already
                if(DiagnosticsEnabled == 4 && MeasSeq.count != (MeasSeq.calcCount-1) && !(getState(&ObjDict_Data) == Mode_Record_X && (bufferCounter == 7)))
                {
                  START_ADC_MEAS(ch);
                  state = 2;
                }
                else
                {
                  if( DiagnosticsEnabled == 1 )
                  {
                    START_VIC_MEAS();
                  }
                  else if( DiagnosticsEnabled == 2 )
                  {
                    START_OFF1_MEAS();
                  }
                  else //DiagnosticsEnabled == 0, 3, or 4
                  {
                    START_EMG1_MEAS();
                  }
                  state = 0;

                }
                
                if( DiagnosticsEnabled == 3 )
		  tempBuffer1[bufferCounter] = Meas2 = Chan2Data_ADC = testbyte;
                else
                  tempBuffer1[bufferCounter] = Meas2 = Chan2Data_ADC = ADCH;
               
                
		CLR_BITS(PORTA, BIT3); //^^test
		/* advance sequencer, inline for speed */
			
		if( MeasSeq.count < MeasSeq.maxCount )
		{
			MeasSeq.count++ ;
			
			if( isAutoSyncEnabled && MeasSeq.count == AUTO_SYNC_PERIOD )
				autoSyncPulse = 1;
			
			(*MeasSeq.samplep[0])();
			(*MeasSeq.samplep[1])();
			
			if( MeasSeq.count == MeasSeq.calcCount )
				(*MeasSeq.calcp)();
			
		}
                
                        /* HSrecording pdo generation                     */
                if (getState(&ObjDict_Data) == Mode_Record_X && (bufferCounter == 7))
                {
                    for (i = 0; i < HSPdoBuffer.len; i++)
                    {
                      if (activeHSChannel == 1)
                        HSPdoBuffer.data[i] = tempBuffer1[i];
                      else if (activeHSChannel == 0)
                        HSPdoBuffer.data[i] = tempBuffer0[i];
                    }
                    //SET_BITS(PORTA, BIT1); //^^test
                      sendPdo( &ObjDict_Data, 0L, &HSPdoBuffer );
                    //CLR_BITS(PORTA, BIT1); //^^test
                }
                
                bufferCounter++;
	}
        else //state == 2
        {
          SET_BITS(PORTA, BIT5); //^^test
          switch(ch)
          {
            case 0: Diagnostic_VIC = ADCH; 
                    ch = 1;
                    break;
            case 1: Diagnostic_VIN = ADCH; 
                    ch = 4;
                    break;
            case 4: Diagnostic_OFF1 = ADCH; 
                    ch = 5;
                    break;
            case 5: Diagnostic_OFF2 = ADCH; 
                    ch = 0;
                    break;
          }
          
          START_EMG1_MEAS();

          state=0;
          CLR_BITS(PORTA, BIT5); //^^test
        }
        CLR_BITS(PORTA, BIT0); //^^test
        
}
 
