/** 
 * @file emgTask.c
 * @author Jay Hardway, JDC
 * @brief HOW IT WORKS:  
	Emg runs from the profile in OD.  
	SDO can save/recall profiles to/from EEprom.
	Here we maintain profile memories per SDO commands and leave status for SDO.

	Many routines work with global variables to run faster.
*/

/**
 * @defgroup Sensors Sensing
 */
/**
 * @defgroup emg EMG Acquisition and Processing
 * @ingroup Sensors
 */

#include <stdio.h>
#include <stdlib.h>
#include "sys.h"
#include "meas.h"
#include "can_AVR.h"



// -------- DEFINITIONS ----------

/**
 * @ingroup emg
 * @brief Buffer area for feature math
*/
typedef struct
{
	UINT16 WflSum;
	UINT16 MavSum;
	INT16  MvSum;
	UINT8  numPoints;
	
	UINT8 ZXcount;
	UINT8 SSCcount;
	
	INT8 lastRaw;
	
	UINT8 MavBuffer[ NUM_EMG_HISTORY ];
	INT8  MvBuffer[ NUM_EMG_HISTORY ];
	UINT8 ZxBuffer[ NUM_EMG_HISTORY ];
	UINT8 SscBuffer[ NUM_EMG_HISTORY ];
	UINT8 WflBuffer[ NUM_EMG_HISTORY ];
	
	UINT8 bufIndex, bufLen;
	
} EMG_RESULTS;


typedef enum
{
	ERS_IDLE,
	ERS_RUN_NO_PDO,
	ERS_RUN_WITH_PDO,
        ERS_RUN_HS
		
} EMG_RUN_STATE;



// --------   DATA   ------------

UINT8 volatile isAutoSyncEnabled, autoSyncPulse, syncCount = 0, startPulse = 0;

static volatile UINT8 IsCalcReady;

static EMG_RESULTS EmgResults[ NUM_EMG_CHAN ] ;

static UINT8 IsConfigSendPdo;			

static Message PdoBuffer;		// transmitted PDO on sync. format CobID[rtr],data[8];

extern volatile UINT8 syncPulse;



ODVAR __flash ChanOdVarMap[ NUM_EMG_CHAN ] = 
{
	// channel-based pointers into OD
	
  { &Chan1Data_Raw, &Chan1Config_Gain, &Chan1FeatureSelect, \
    &Chan1Features_MeanAbsValue, &Chan1Features_WaveFormLength, &Chan1Features_ZeroCrossing, \
    &Chan1Features_SlopeSignChange, &Chan1Features_MeanValue,\
    &Chan1Config_AverageDepth, \
    &Chan1Config_Threshold_ZX, &Chan1Config_Threshold_SSC, \
    &Chan1Config_Gain_MAV, &Chan1Config_Gain_MV, &Chan1Config_Gain_WFL },
          
  { &Chan2Data_Raw, &Chan2Config_Gain, &Chan2FeatureSelect, \
    &Chan2Features_MeanAbsValue, &Chan2Features_WaveFormLength, &Chan2Features_ZeroCrossing, \
    &Chan2Features_SlopeSignChange, &Chan2Features_MeanValue,\
    &Chan2Config_AverageDepth, \
    &Chan2Config_Threshold_ZX, &Chan2Config_Threshold_SSC, \
    &Chan2Config_Gain_MAV, &Chan2Config_Gain_MV, &Chan2Config_Gain_WFL} 
};


// -------- PROTOTYPES ----------

static void buildOpParams( void );
static UINT8 isStateChangedToRun( EMG_RUN_STATE *runState );

static void runCalcFeatures( EMG_FEATURE *feature, ODVAR __flash *ODvar, EMG_RESULTS *results, UINT8 *pdoData );

static UINT8 calcMAV( void );
static INT8  calcMV( void );
static UINT8 calcZX( void );
static UINT8 calcSSC( void );
static UINT8 calcWFL( void );

void initEmgTaskValues( void );


//============================
//    GLOBAL CODE
//============================

UINT8 testHSD;	

/**
 * @ingroup emg
 * @brief 
 * @param on
 */
void setEmgAutoSync( UINT8 on )
{
	
	if( on )
	{
		isAutoSyncEnabled = 1;
		autoSyncPulse = 1;
		
	}
	else
	{
		isAutoSyncEnabled = 0;
		autoSyncPulse = 0;
	}
	
}

/**
 * @ingroup emg
 * @brief Initializes digital pot, initializes EMG measurement  
 */
void initEmgTask( void )
{
	
	buildOpParams();
	//JML: moved from initEMGmeas...only needs to happen on startup, not on mode changes
        setDigPot( 0x80 );		// turn off mute
        setDigPot( 0xA0 );		// turn off ZCross
  
        setDigPot( 0x00 );		// wiper 0 position = 0
        setDigPot( 0x40 );		// wiper 1 position = 0
	
        initEmgMeas();
	
}

/**
 * @ingroup emg
 * @brief resets sync count (called upon entry into waiting and stopped)
 */
void InitEmgTaskValues( void )
{
    syncCount = 0;	
}


/**
 * @ingroup emg
 * @brief Run EMG task  
 */
UINT8 oneShotStateChange = 0;

void runEmgTask( void )
{
	// Main task state machine
        // Timing: calc takes 600us all normal features;
	// 	or half that per channel.
	// Timing: calc takes 17 us HSD;
	
	EMG_RUN_STATE runState ;
        
	if( isStateChangedToRun( &runState )  ) 
	{
		configMeasParams( testHSD );
		buildOpParams();
		
                // state for sending PDO => 1 = True
		IsConfigSendPdo = (runState == ERS_RUN_WITH_PDO)?  1 : 0 ;
		oneShotStateChange = 1;
	}
	
        /* Check if past syncInterval - block sync pulse if not for this module */ 
	if ( syncPulse && IsConfigSendPdo )
        {
          if ( ++syncCount >= SyncInterval)
          {
            startPulse = 1;
            syncCount = 0;
            PORTA ^= 0x04;
          }
          syncPulse = 0;
        }
        
        /* Begin sampling      */
	if( startPulse > 0 && IsConfigSendPdo ) // autoSyncPulse was or'd with start pulse	
	{
		IsCalcReady = 0;  //JML: if starting new window before previous calc is ready, ignore it
                startMeasSampling();
		
		startPulse = 0;	            
	}

	if( IsCalcReady ) 
        {
		SET_BITS( PORTA, BIT1 );	
		
		runCalcFeatures( &EmgFeature[0], &ChanOdVarMap[0], &EmgResults[0], &PdoBuffer.data[0] );
                runCalcFeatures( &EmgFeature[1], &ChanOdVarMap[1], &EmgResults[1], &PdoBuffer.data[4] );
                
                if ( (EmgFeature[0].select & FE_ACC ) || EmgFeature[1].select & FE_ACC )
                {
                  PdoBuffer.data[2] = Accelerometers[0];
                  PdoBuffer.data[3] = Accelerometers[1];
                  PdoBuffer.data[6] = Accelerometers[2];
                  PdoBuffer.data[7] = Accelerometers[3];
                  
                }
               
		
                // not sent when HS recording is enabled
		if( IsConfigSendPdo  ) 
			sendPdo( &ObjDict_Data, 0L, &PdoBuffer );
		
		IsCalcReady = 0;
		
		CLR_BITS( PORTA, BIT1 );	
	}
	

}

/**
 * @ingroup emg
 * @brief transfer the Measurement buffers after all samples are gathered
 *  and ready to do feature calculations.  
 */
void putEmgCalcData( void )
{
  EmgResults[0].MavSum    = EmgFeature[0].MavSum;
  EmgResults[0].MvSum     = EmgFeature[0].MvSum;
  EmgResults[0].ZXcount   = EmgFeature[0].ZXcount;
  EmgResults[0].SSCcount  = EmgFeature[0].SSCcount;
  EmgResults[0].WflSum    = EmgFeature[0].WflSum;
  EmgResults[0].numPoints = EmgFeature[0].numPoints;
  EmgResults[0].lastRaw	= EmgFeature[0].lastRaw;
  
  EmgResults[1].MavSum    = EmgFeature[1].MavSum;
  EmgResults[1].MvSum     = EmgFeature[1].MvSum;
  EmgResults[1].ZXcount   = EmgFeature[1].ZXcount;
  EmgResults[1].SSCcount  = EmgFeature[1].SSCcount;
  EmgResults[1].WflSum    = EmgFeature[1].WflSum;
  EmgResults[1].numPoints = EmgFeature[1].numPoints;
  EmgResults[1].lastRaw	= EmgFeature[1].lastRaw;
  
  
  EmgFeature[0].MavSum   = 0 ;
  EmgFeature[0].MvSum    = 0 ;
  EmgFeature[0].ZXcount  = 0 ;
  EmgFeature[0].SSCcount = 0 ;
  EmgFeature[0].WflSum   = 0 ;
  EmgFeature[0].numPoints   = 0 ;
  
  EmgFeature[1].MavSum   = 0 ;
  EmgFeature[1].MvSum    = 0 ;
  EmgFeature[1].ZXcount  = 0 ;
  EmgFeature[1].SSCcount = 0 ;
  EmgFeature[1].WflSum   = 0 ;
  EmgFeature[1].numPoints   = 0 ;
  
  
  
  IsCalcReady = 1;
  
}



//============================
//    LOCAL CODE
//============================
/**
 * @ingroup statemachine
 * @brief refresh runState and return a nodestate change to a run mode
 * @param *runState
 */
static UINT8 isStateChangedToRun( EMG_RUN_STATE *runState )
{
	// refresh runState and return a nodestate change to a run mode
	
	e_nodeState nodeState;
	EMG_RUN_STATE rnState;
	static EMG_RUN_STATE lastRnState;
	UINT8 stateChange = 0;
	
	
	nodeState = getState(&ObjDict_Data);
	
	switch( nodeState )
	{
		case Mode_Produce_X_Manual:
		case Mode_Patient_Manual:
		case Mode_Patient_Control:
                  //initEmgMeas(); // set timer //JML: this should only happen on state change
		rnState = ERS_RUN_WITH_PDO ;
		break;
		
		case Mode_X_Manual:
		case Mode_Y_Manual:

			rnState = ERS_RUN_NO_PDO ;
			break;
                        
                case Mode_Record_X:
                        //initEmgMeas(); // set timer //JML: this should only happen on state change
                        rnState = ERS_RUN_HS ;
                        break;
		
		default:
			
			rnState = ERS_IDLE ;
			break;
	}
	
        // if not true, then return stateChange = 0;
	if( lastRnState != rnState
	&&	rnState != ERS_IDLE )
	{
		stateChange = 1;
                initEmgMeas(); //JML: moved here
	}
        
	
	*runState = rnState;
	lastRnState = rnState;
	
	return stateChange;
		
}

/**
 * @ingroup emg
 * @brief run before each session to reconfig the process
 */
static void buildOpParams( void )
{

	EMG_FEATURE *feature;
	EMG_RESULTS *results;
	ODVAR __flash *odVar ;
	UINT8 chan, i;
	
	
	for( chan=0 ; chan<NUM_EMG_CHAN ; chan++ )
	{
		feature = &EmgFeature[ chan ];
		results = &EmgResults[ chan ];
		odVar   = &ChanOdVarMap[ chan ];
		
		feature->select = *odVar->featureSelect ;
		
		/* limit Normal to 4 features */
		
		if( (feature->select & FE_ALL) == FE_ALL )
		{
			feature->select = FE_DEFAULT;
		}
		
		/* limit 1-5 so no divByZero or overrun */
		if( (results->bufLen = *odVar->avgDepth) < 1 )
			results->bufLen = 1;
		else if( results->bufLen > NUM_EMG_HISTORY )
			results->bufLen = NUM_EMG_HISTORY;
		
		feature->ZXthresh  = *odVar->thresholdZX;
		feature->SSCthresh = *odVar->thresholdSSC;
		
		feature->WflSum = 0;
		feature->MavSum = 0;
		feature->MvSum  = 0;
		feature->numPoints = 0;
		
		feature->ZXcount  = 0;
		feature->SSCcount = 0;
		results->bufIndex = 0;
		
		for( i=0 ; i<NUM_EMG_HISTORY ; i++ )
		{
			results->MavBuffer[i] = 0;
			results->MvBuffer[i]  = 0;
			results->ZxBuffer[i]  = 0;
			results->SscBuffer[i] = 0;
			results->WflBuffer[i] = 0;
		}
		
	}
	
	PdoBuffer.cob_id = 0x680 + Status_NodeId ;	
	PdoBuffer.rtr = 0;
	PdoBuffer.len = 8;
	
}


/**
 * @ingroup emg
 * @brief featureParameters use global vars to reduce execution time.
 */
struct
{
	EMG_FEATURE *feature;
	INT8        *sample;
	ODVAR __flash *ODvar;
	EMG_RESULTS *results;
	
} fPar;

/**
 * @ingroup emg
 * @brief calculate ODict feature results and build PDO data
 */
static void runCalcFeatures( EMG_FEATURE *feature, ODVAR __flash *ODvar, EMG_RESULTS *results, UINT8 *pdoData )
{	
	UINT8 isActive;
	UINT8 uResult;
	INT8  sResult;
	
	
	fPar.feature = feature;
	fPar.ODvar 	 = ODvar;
	fPar.results = results;
	
	if( fPar.results->numPoints == 0 )
		return;
	
	fPar.feature->select = *(fPar.ODvar->featureSelect);
	
	isActive = fPar.feature->select ;
		
	if( isActive & FE_MAV )
	{
		uResult = calcMAV();
		*fPar.ODvar->featureMAV = uResult;
		*pdoData++ = uResult;
	}
	
	if( isActive & FE_MV )
	{
		sResult = calcMV();
		*fPar.ODvar->featureMV = sResult ;
		*pdoData++ = (UINT8)sResult; 
	}
	
	if( isActive & FE_ZX )
	{
		uResult = calcZX();
		*fPar.ODvar->featureZX = uResult;
		*pdoData++ = uResult;
	}
	
	if( isActive & FE_SSC )
	{
		uResult = calcSSC();
		*fPar.ODvar->featureSSC = uResult;
		*pdoData++ = uResult;
	}
	
	if( isActive & FE_WFL )
	{
		uResult = calcWFL();
		*fPar.ODvar->featureWFL = uResult;
		*pdoData++ = uResult;
	}
	        
	
	/* init for next window */
	
	if( ++fPar.results->bufIndex >= fPar.results->bufLen )
		fPar.results->bufIndex = 0;
	
}

/*----------------------
	FEATURE PROCESSORS

	These functions use global 
	pointers to speed execution.

	ASSUME: unused values in bufs = 0.
	
----------------------*/




/** 
 * @ingroup emg
 * @brief Calculate the Mean Absolute Value, smooth and put in ODict
 *    ASSUME: unused values in buf = 0.
 */
static UINT8 calcMAV( void )
{

	UINT32  avg ;
	UINT8  *buf ;
	UINT16 sum ;
	
	
	avg = ((UINT32)fPar.results->MavSum * (UINT32)(*fPar.ODvar->gainMAV)) / fPar.results->numPoints ;
	
        if (avg > 255)
          avg = 255;
        
	fPar.results->MavBuffer[ fPar.results->bufIndex ] = (UINT8) avg;
	
	buf = &fPar.results->MavBuffer[0];
	
	sum  = *buf++ ;
	sum += *buf++ ;
	sum += *buf++ ;
	sum += *buf++ ;
	sum += *buf ;
	
	//*fPar.ODvar->featureMAV = sum / fPar.results->bufLen ;
	
	fPar.feature->MavSum = 0;
	
	return (sum / fPar.results->bufLen) ;
	
}

/** 
 * @ingroup emg
 * @brief Calculate the Mean Value, smooth and put in ODict
 *    ASSUME: unused values in buf = 0.
 */
static INT8 calcMV( void )
{

	INT32  avg ;
	INT8  *buf ;
	INT16 sum ;
	
	
	avg = ((INT32)fPar.results->MvSum * (INT32)(*fPar.ODvar->gainMV)) / fPar.results->numPoints;
	
        if (avg > 127)
          avg = 127;
        else if (avg < -128)
          avg = -128;
          
	fPar.results->MvBuffer[ fPar.results->bufIndex ] = (INT8) avg;
	
	buf = &fPar.results->MvBuffer[0];
	
	sum  = *buf++ ;
	sum += *buf++ ;
	sum += *buf++ ;
	sum += *buf++ ;
	sum += *buf ;
	
	//*fPar.ODvar->featureMV = sum / fPar.results->bufLen ;
	
	fPar.feature->MvSum = 0;
	
	return (sum / fPar.results->bufLen) ;
	
}


/** 
 * @ingroup emg
 * @brief Zero Crossing count, smooth and put in ODict
 *    ASSUME: unused values in buf = 0.
 */
static UINT8 calcZX( void )
{
	
	UINT8  *buf ;
	UINT16 sum;
	
	fPar.results->ZxBuffer[ fPar.results->bufIndex ] = fPar.results->ZXcount ;
	
	buf = &fPar.results->ZxBuffer[0];
	
	sum  = *buf++ ;
	sum += *buf++ ;
	sum += *buf++ ;
	sum += *buf++ ;
	sum += *buf ;
	
	//*fPar.ODvar->featureZX = sum / fPar.results->bufLen ;
	
	fPar.feature->ZXcount = 0;
	
	return (sum / fPar.results->bufLen) ;
	
}



/** 
 * @ingroup emg
 * @brief Slope Sign Change Count, smooth and put in ODict
 *    ASSUME: unused values in buf = 0.
 */
static UINT8 calcSSC( void )
{
	UINT8  *buf ;
	UINT16 sum;
	
	fPar.results->SscBuffer[ fPar.results->bufIndex ] = fPar.results->SSCcount ;
	
	buf = &fPar.results->SscBuffer[0];
	
	sum  = *buf++ ;
	sum += *buf++ ;
	sum += *buf++ ;
	sum += *buf++ ;
	sum += *buf ;
	
	//*fPar.ODvar->featureSSC = sum / fPar.results->bufLen ;
	
	fPar.feature->SSCcount = 0;
	
	return (sum / fPar.results->bufLen) ;
	
}


/** 
 * @ingroup emg
 * @brief Calculate the Waveform Length, smooth and put in ODict
 *    ASSUME: unused values in buf = 0.
 */
static UINT8 calcWFL( void )
{

	UINT32 avg;
	UINT8  *buf ;
	UINT16 sum;
	
	
	avg = ((UINT32)fPar.results->WflSum * (UINT32)(*fPar.ODvar->gainWFL)) / fPar.results->numPoints;
	
        if (avg > 255)
          avg = 255;
        
	fPar.results->WflBuffer[ fPar.results->bufIndex ] = (UINT8) avg;
	
	buf = &fPar.results->WflBuffer[0];
	
	sum  = *buf++ ;
	sum += *buf++ ;
	sum += *buf++ ;
	sum += *buf++ ;
	sum += *buf ;
	
	//*fPar.ODvar->featureWFL = sum / fPar.results->bufLen ;
	
	fPar.feature->WflSum = 0;
	
	return (sum / fPar.results->bufLen) ;
	
}


/**
 * @ingroup emg
 * @brief sets active channel for HIgh Speed mode
 * @param channelNumber Valid choices are 0, 1 for channel
 */
void CollectHSData( UNS8 channelNumber ) 
{
	activeHSChannel = channelNumber & 0x0F;       
}