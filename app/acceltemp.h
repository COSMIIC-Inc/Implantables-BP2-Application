/**
 * @file acceltemp.h (common to MES and Stim)
 * @author Jay Hardway, JDC
 * @brief Header file for accelKxt.c
 */

#ifndef ACCELTEMP_H
#define ACCELTEMP_H


// -------- DEFINITIONS ----------


// --------   DATA   ------------


// -------- PROTOTYPES ----------

void sleepAccelerometer( void );
void initAccelerometer( void );
void updateAccelerometer( void );
void runTemperatureTask( void );
void updateDiagnostics( void );


#endif
 
