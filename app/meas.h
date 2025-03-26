/**
 * @file   meas.h
 * @author Jay Hardway, JDC
 * @brief Header file for meas.c
*/

#ifndef _H
#define _H


#include "ObjDict.h"



// -------- DEFINITIONS ----------

#define NUM_EMG_CHAN		2
#define	NUM_EMG_HISTORY		5

#define ANTIALIAS_EMG1_LF()   SET_BITS( PORTC, BIT6 ) 
#define ANTIALIAS_EMG2_LF()   SET_BITS( PORTC, BIT7 ) 
#define ANTIALIAS_EMG1_HF()   CLR_BITS( PORTC, BIT6 ) 
#define ANTIALIAS_EMG2_HF()   CLR_BITS( PORTC, BIT7 ) 
/**
 * @brief pointers to OD data
 */
typedef struct
{
  INT8 *dataRaw;
  UNS8 *gain, *featureSelect;
  UNS8 *featureMAV, *featureWFL, *featureZX, *featureSSC;
  INT8 *featureMV;
  UNS8  *avgDepth;
  UNS8  *thresholdZX, *thresholdSSC;
  UNS8 *gainMAV, *gainMV, *gainWFL;
  
} ODVAR;

/**
 * @brief EMG feature struct
 */
typedef struct
{
	UINT8 select; 

	UINT16 WflSum;
	UINT16 MavSum;
	INT16  MvSum;
	UINT8  numPoints;
	
	UINT8 ZXthresh;
	UINT8 ZXcount;
	
	UINT8 SSCthresh;
	UINT8 SSCcount;
	
	INT8 lastRaw;
	
} EMG_FEATURE;

 // feature.select bits
 #define	FE_MAV		BIT0
 #define	FE_MV		BIT1
 #define	FE_ZX		BIT2
 #define	FE_SSC		BIT3
 #define	FE_WFL		BIT4
 #define	FE_HSD		BIT5
 #define        FE_ACC          BIT6

 #define	FE_ALL		(BIT0 | BIT1 | BIT2 | BIT3 | BIT4)
 #define	FE_DEFAULT	(BIT0 | BIT1 | BIT2 | BIT3)



// --------   DATA   ------------

extern EMG_FEATURE EmgFeature[ NUM_EMG_CHAN ];

extern ODVAR __flash ChanOdVarMap[ NUM_EMG_CHAN ]; 

extern INT8  MeasRawBuffer[6];
extern UINT8 dummyStateChange;  

extern UINT8 volatile isAutoSyncEnabled, autoSyncPulse;
extern UINT8 activeHSChannel;

// -------- PROTOTYPES ----------

void initEmgMeas( void );
void setDigPot( UINT8 value );  //JML
void configMeasParams( UINT8 runHSD );

void startMeasSampling( void ); 
void putEmgCalcData( void );
void setGains(UINT8 ch1, UINT8 ch2 );


#endif
 
