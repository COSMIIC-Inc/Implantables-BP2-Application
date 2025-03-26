/**
 * @file accelKxt.c (common to MES and Stim)
 * @author Jay Hardway, JDC
 * @brief interface to accelerometer
 */

/**
 * @defgroup accelerometer Accelerometer
 * @ingroup Sensors
 */

#include <stdio.h>
#include <stdlib.h>
#include "app.h"
#include "acceltemp.h"
#include "ObjDict.h"



// -------- DEFINITIONS ----------


#define ACC_SAD			0x0F
#define CTRL_REG1		0x1B
#define XOUT			0x12
#define TMP_SAD                 0x48
#define TEMP                    0x00
#define MAX_TEMPR				500			// 50.0 degC
#define MIN_TEMPR				100

//#define START_ADC_MEAS(ch) { ADMUX = B(ADLAR) | B(REFS0) | ch; /* ref=Avcc, MUX=ch*/	\
//		             ADCSRA &= ~B(ADATE); /* disarm auto trgr */  \
//                             ADCSRA &= ~B(ADIE); /* disble interrupt */  \
//                             ADCSRA |= B(ADSC);   /* start conversion */  }


// --------   DATA   ------------

#define NTAN 91  
static const /*__flash*/ UINT16 TAN[NTAN] = 
{      0,      4,      9,     13,     18,     22,     27,     31,     36,     41,
     45,     50,     54,     59,     64,     69,     73,     78,     83,     88,
     93,     98,    103,    109,    114,    119,    125,    130,    136,    142,
    148,    154,    160,    166,    173,    179,    186,    193,    200,    207,
    215,    223,    231,    239,    247,    256,    265,    275,    284,    294,
    305,    316,    328,    340,    352,    366,    380,    394,    410,    426,
    443,    462,    481,    502,    525,    549,    575,    603,    634,    667,
    703,    743,    788,    837,    893,    955,   1027,   1109,   1204,   1317,
   1452,   1616,   1822,   2085,   2436,   2926,   3661,   4885,   7331,  14666,     
  32767};

// -------- PROTOTYPES ----------

static UINT8 getAccelReg( UINT8 regNo, UINT8 *data, UINT8 n );
static UINT8 putAccelReg( UINT8 regNo, UINT8 *data, UINT8 n );
static UINT8 getTempReg( UINT8 regNo, UINT8 *data, UINT8 n );

static void startTwi( UINT8 *status );
static void stopTwi( UINT8 *status );
static void sendTwiData( UINT8 *data, UINT8 n, UINT8 *status );
static void readTwiData( UINT8 *data, UINT8 n, UINT8 *status );

static UINT8 atanT( UINT16 x);
static UINT8 intsqrt( UINT16 x);



//============================
//    GLOBAL CODE
//============================
/**
 * @ingroup accelerometer
 * @brief puts Accelerometer to sleep
 */
void sleepAccelerometer( void )
{
	UINT8 d = 0x00; // standby
	putAccelReg( CTRL_REG1, &d, 1 );
}

/**
 * @ingroup accelerometer
 * @brief Initializes Accelerometer
 */
void initAccelerometer( void )
{
	//UINT8 d = 0x90; 	// start at 10Hz
        //UINT8 d = BIT7 | BIT4; // start at 10Hz
        UINT8 d = BIT7 | BIT4 | BIT3; // start at 40Hz
	
	
	/* set bit rate = (1,2,4,8MHz) / (16 + 2*TWBR * 4^TWPS), TWPS=0 */
	//Max bitrate for accelerometer and temperature sensor is 400kbps
        //TWBR should be 10 or higher if the TWI operates in Master mode
        //1MHz/(16+2*10) = 27.7 kbps at 1MHz
        //2MHz/(16+2*10) = 55.5 kbps at 2MHz
        //4MHz/(16+2*10) = 111.1 kbps at 4MHz
        //8MHz/(16+2*10) = 222.2 kbps at 8 MHz   
	TWBR = 10; 
	
	
	/* enable operation to default settings */
	
	putAccelReg( CTRL_REG1, &d, 1 );
}

#define ACCEL_AVERAGE_DEPTH_BITS 0x1F
#define ACCEL_CALC_TILT_BIT      0x80
#define ACCEL_BUFFER_LENGTH 20  //Must be less than 256 so cnt doesn't rollover.   

/**
 * @ingroup accelerometer
 * @brief update Accelerometer
 */
void updateAccelerometer( void )
{
        static UINT8 taskDelayMSecs = 25; //JML, was 100 (10Hz), now 40Hz
        static UINT32 tDelayRef = 0; //JML: set to 0, so timesout first time through

        static INT8 xBuf[ACCEL_BUFFER_LENGTH];
        static INT8 yBuf[ACCEL_BUFFER_LENGTH];
        static INT8 zBuf[ACCEL_BUFFER_LENGTH];
        static UINT8 cnt = 0;
        UINT8 depth = AccelerometerSettings & ACCEL_AVERAGE_DEPTH_BITS;
        
        UINT8 accelData[3];  
        
	if( !isTimedOut( &tDelayRef, TIMEOUT_ms( taskDelayMSecs ) ) )
	{
                return;
	}
	
	//ELSE.. reading delay is over
	PORTA |= BIT6; //^^test
        
	resetTimeOut( &tDelayRef );
	
	
	/* read accelerometer x,y,z */

	getAccelReg( XOUT, accelData, 3 );
	
        DISABLE_INTERRUPTS();
	memcpy( &Accelerometers[0], accelData, 3 );
        Accelerometers[3]++; //use accelerometer 4th byte as counter
        ENABLE_INTERRUPTS();
        
if (depth > 0)
        {
          UINT8 i;  
          INT16 sumX=0, sumY=0, sumZ=0;
          INT8 x=0,y=0,z=0;
           
          if(depth > ACCEL_BUFFER_LENGTH)
            depth = ACCEL_BUFFER_LENGTH;
          
          //Shift values in buffer
          //destination, source
          memmove( &xBuf[1], &xBuf[0], depth-1);
          memmove( &yBuf[1], &yBuf[0], depth-1);
          memmove( &zBuf[1], &zBuf[0], depth-1);
          
          //accelData is 0 to 256
          //Add new value at beginning of buffer 
          //ignore bottom two bits and convert to signed (-32 to 31 range)
          xBuf[0]=(INT8)(accelData[0]>>2) - 32;
          yBuf[0]=(INT8)(accelData[1]>>2) - 32;
          zBuf[0]=(INT8)(accelData[2]>>2) - 32;
          
          if(cnt<depth) //if we haven't yet reached sampled depth
          {
            cnt++;
            depth = cnt;
          }
          
          for(i=0; i<depth; i++)
          {
            sumX += xBuf[i];
            sumY += yBuf[i];
            sumZ += zBuf[i];
          }
          x =(INT8)((sumX<<2)/depth);
          y =(INT8)((sumY<<2)/depth);
          z =(INT8)((sumZ<<2)/depth);
          //in order to make the division more accurate we shift the sum left two bits before dividing
          //outcome is now -128 to 127
          AccelerometersFiltered[0] = (UINT8)(x + 128);
          AccelerometersFiltered[1] = (UINT8)(y + 128);
          AccelerometersFiltered[2] = (UINT8)(z + 128);
          AccelerometersFiltered[3] = Accelerometers[3];
          

          
          if(AccelerometerSettings & ACCEL_CALC_TILT_BIT)
          {
            //calculate squares in advance
            UINT16 xx = x*x; 
            UINT16 yy = y*y;
            UINT16 zz = z*z;
            
            UINT16 yyzz = yy+zz;
            UINT16 xxzz = xx+zz;
            UINT16 xxyy = xx+yy;
            
              
            if( x < 0 )
            {
              if(yyzz==0) //avoid divide by zero
                AccelerometersTilt[0] = -90;
              else
                AccelerometersTilt[0] = - (INT8) atanT(    ((UINT16)(-x)<<8)          /  intsqrt(yyzz));
            }
            else
            {
              if(yyzz==0) //avoid divide by zero
                AccelerometersTilt[0] = 90; 
              else
                AccelerometersTilt[0] =   (INT8) atanT(    ((UINT16)( x)<<8)          /  intsqrt(yyzz));
            }
            if( y < 0 )
            {
              if(xxzz==0) //avoid divide by zero
                AccelerometersTilt[1] = -90;
              else
                AccelerometersTilt[1] = - (INT8) atanT(    ((UINT16)(-y)<<8)          /  intsqrt(xxzz));
            }
            else
            {
              if(xxzz==0) //avoid divide by zero
                AccelerometersTilt[1] = 90;
              else
                AccelerometersTilt[1] =  (INT8) atanT(    ((UINT16)( y)<<8)          /  intsqrt(xxzz));
            }
            if(z == 0) //avoid divide by zero
              AccelerometersTilt[2] = 90;
            else if(z < 0)
              AccelerometersTilt[2] = - (INT8) atanT( ((UINT16)intsqrt(xxyy)<<8)   /    (UINT8)(-z) );
            else
              AccelerometersTilt[2] =   (INT8) atanT( ((UINT16)intsqrt(xxyy)<<8)   /    (UINT8)( z) );
            
            AccelerometersTilt[3] = Accelerometers[3];
          }
          
          
        }
                  
        
        
	
}

/**
 * @ingroup temp
 * @brief Gets temperature reading via SPI
 */
void runTemperatureTask( void )
{
	static UINT8 taskDelaySecs = 1;		//^^fornow. later read from ObjDict; 0=off
        static UINT32 tDelayRef = 0;//JML: set to 0, so timesout first time through;
        UINT8 tempData[2];  
	INT16 degrC;
	
        
	
	if( !isTimedOut( &tDelayRef, TIMEOUT_sec( taskDelaySecs ) ) )
	{
		return;
	}
	
	//ELSE.. reading delay is over
	PORTA |= BIT7; //^^test
        
	resetTimeOut( &tDelayRef );
	
        /* read temp  x,y,z */

	getTempReg( TEMP, tempData, 2 );
	
        
	/* temp is measured in 1/16 degC above 0deg C */
	
	/* save temperature reading in 10ths degC */
	
	//Temperature = ((UINT16)tempData[0] << 4) + ((UINT16)tempData[1] >> 4 ); //Raw Reading

        degrC = ( (  (INT32)(  ((UINT16)tempData[0] << 4) + ((UINT16)tempData[1] >> 4 )  )  )* 160 ) >> 8 ;
	
	if( degrC > MAX_TEMPR )
	{
		degrC = MAX_TEMPR;
	}
	else if( degrC < MIN_TEMPR )
	{
		degrC = MIN_TEMPR;
	}
	
	Temperature = (UINT16)degrC;
	
        PORTA &=~ BIT7; //^^test

}

//Diagnostics are obtanined in an ISR like the EMG measurements...see meas.c
//void updateDiagnostics( void )
//{
//        static UINT8 taskDelayMSecs = 100;
//        static UINT32 tDelayRef = 0; //JML: set to 0, so timesout first time through
//        static UINT8 ch = 0;
//        UINT8 res;
//	
//	if( !isTimedOut( &tDelayRef, TIMEOUT_ms( taskDelayMSecs ) ) )
//	{
//                return;
//	}
//	
//	//ELSE.. reading delay is over
//	PORTA |= BIT5; //^^test
//        
//	resetTimeOut( &tDelayRef );
//	
//        if(DiagnosticsEnabled == 4)
//        {
//          while(ADCSRA & B(ADSC)); //If a conversion is in progress, wait until it is completed
//          
//          START_ADC_MEAS(ch);
//          while(ADCSRA & B(ADSC)); //wait until adc conversion is completed
//          res = ADCH;
//          
//          //set ch for next measurement  
//          switch(ch)
//          {
//            case 0: Diagnostic_VIC = res;
//                    ch=1; 
//                    break;
//            case 1: Diagnostic_VIN = res;
//                    ch=4; 
//                    break;
//            case 4: Diagnostic_OFF1 = res;
//                    ch=5; 
//                    break;
//            case 5: Diagnostic_OFF2 = res;
//                    ch=0; 
//                    break;
//            default: ch=0;
//          }	
//        }
//        PORTA &=~ BIT5; //^^test
//        
//        //reenable interrupts and trigger for EMG 
//        ADCSRA |= B(ADATE) | B(ADIE); 
//}

//============================
//    LOCAL CODE
//============================


//To get atan(Value), use atanT(Value<<8)
//result is in degrees
static UINT8 atanT( UINT16 x)
{
    UINT8 imax=NTAN;
    UINT8 imin=0;
    UINT8 i=0;
    UINT8 delta=NTAN;

    while(delta>1)
    {
        i=imin+(delta>>1);

        if(x==TAN[i])
            return i;//-TAN_IOFFSET;
        else if(x<TAN[i])
            imax = i;
        else
            imin = i;
        
        delta = imax-imin;
    }
    return imin;//-TAN_IOFFSET;
}

//we only need to go up to 64^2+64^2 = 8192
//modified from https://www.avrfreaks.net/forum/where-sqrt-routine
static UINT8 intsqrt(UINT16 val) {
    UINT8 mulMask = 0x0040;
    UINT8 retVal = 0;
    if (val > 8100)
      return 90;
    
    while (mulMask != 0) {
        retVal |= mulMask;
        if ((retVal * retVal) > val) {
            retVal &= ~mulMask;
        }

        mulMask >>= 1;
    }
    return retVal;
}

/**
 * @ingroup accelerometer
 * @brief write to Accelerometer register
 * @param regNo register number
 * @param *data pointer to data
 * @param n 
 * @return 
 */
static UINT8 putAccelReg( UINT8 regNo, UINT8 *data, UINT8 n )
{
	UINT8 status = 0;
	UINT8 header[2];
	
	
	startTwi( &status );
	
	header[0] = ACC_SAD << 1;
	header[1] = regNo;
	
	sendTwiData( header, 2, &status );
	
	sendTwiData( data, n, &status );
	
	stopTwi( &status );
	
	return status;
	
}

/**
 * @ingroup accelerometer
 * @brief read from Accelerometer register
 * @param regNo register number
 * @param *data pointer to data
 * @param n 
 * @return 
 */
static UINT8 getAccelReg( UINT8 regNo, UINT8 *data, UINT8 n )
{
	UINT8 status = 0;
	UINT8 header[2];
	//^^static UINT16 fakeData;
	
	startTwi( &status );
	
	header[0] = ACC_SAD << 1 ;
	header[1] = regNo;
	
	sendTwiData( header, 2, &status );
	
	startTwi( &status );
	
	header[0] = (ACC_SAD << 1) | 1 ;
	
	sendTwiData( header, 1, &status );
	
	readTwiData( data, n, &status );
	
	/*
	//^^fornow wo/ accelr chip..
	*data++ = fakeData >> 4;
	*data++ = (fakeData >> 4) + 2 ;
	*data++ = (fakeData >> 4) + 5 ;
	fakeData++;
	//^^ end of fornow code
	*/
	
	
	stopTwi( &status );
	
	return status;

}

/**
 * @ingroup accelerometer
 * @brief read from Accelerometer register
 * @param regNo register number
 * @param *data pointer to data
 * @param n 
 * @return 
 */
static UINT8 getTempReg( UINT8 regNo, UINT8 *data, UINT8 n )
{
	UINT8 status = 0;
	UINT8 header[2];
	
	startTwi( &status );
	
	header[0] = TMP_SAD << 1 ;
	header[1] = regNo;
	
	sendTwiData( header, 2, &status );
	
	startTwi( &status );
	
	header[0] = (TMP_SAD << 1) | 1 ;
	
	sendTwiData( header, 1, &status );
	
	readTwiData( data, n, &status );
	
	stopTwi( &status );
	
	return status;

}

//=====================================
//    TwoWireInterface TWI ROUTINES
//=====================================

/* theory of operation
	We are using a bit rate of 8M/(16 +20) = 41us for 9 bits; so we set
	a timeout for 60 * 12clockCycles = 90us in the routines where we
	could hang up waiting for a response.
*/

#define TWI_BUSY()		( !BITS_TRUE( TWCR, B(TWINT) ) )
#define MAX_TWI_TIME	60

/**
 * @ingroup accelerometer
 * @brief start TWI (I2C) inteface to accelerometer
 * @param *status 
 */
static void startTwi( UINT8 *status )
{
	UINT8 timeout=0;
	
	
	TWCR = B(TWINT) | B(TWSTA) | B(TWEN);
	
	while( TWI_BUSY() )
	{
		if( timeout++ > MAX_TWI_TIME )
			break;
	}
	
	*status = TWSR;
	
}

/**
 * @ingroup accelerometer
 * @brief stop TWI (I2C) inteface to accelerometer
 * @param *status 
 */
static void stopTwi( UINT8 *status )
{
	
	TWCR = B(TWINT) | B(TWSTO) | B(TWEN);
		
}

/**
 * @ingroup accelerometer
 * @brief send data via TWI (I2C) to accelerometer
 * @param *data 
 * @param n 
 * @param *status 
 */
static void sendTwiData( UINT8 *data, UINT8 n, UINT8 *status )
{
	
	UINT8 timeout;
	
	
	while( n-- > 0 )
	{
		TWDR = *data++ ;
		TWCR = B(TWINT) | B(TWEN);
		
		for( timeout=0; TWI_BUSY() ; )
		{
			if( timeout++ > MAX_TWI_TIME )
			{
				n=0;
				break;
			}
		}
	
		*status = TWSR;
		
	}
}

/**
 * @ingroup accelerometer
 * @brief receive data via TWI (I2C) from accelerometer
 * @param *data 
 * @param n 
 * @param *status 
 */
static void readTwiData( UINT8 *data, UINT8 n, UINT8 *status )
{
	
	UINT8 timeout;
	
	
	while( n-- > 0 )
	{
		/* return ACK with all but last data */
		
		if( n > 0 )
			TWCR = B(TWINT) | B(TWEN) | B(TWEA) ;
		else
			TWCR = B(TWINT) | B(TWEN) ;
		
		
		for( timeout=0; TWI_BUSY() ; )
		{
			if( timeout++ > MAX_TWI_TIME )
			{
				n=0;
				break;
			}
		}
		
		*data++ = TWDR ;
		
		*status = TWSR;
		
	}
}



//============================
//    INTERRUPT SERVICE ROUTINES
//============================


//============================
//    HARDWARE SPECIFIC CODE
//============================


