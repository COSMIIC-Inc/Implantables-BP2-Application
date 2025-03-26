/**
 * @file emgTask.h
 * @author Jay Hardway, JDC 
 * @brief Header file for emgTask.c
*/

#ifndef EMGTASK_H
#define EMGTASK_H


// -------- DEFINITIONS ----------


// --------   DATA   ------------


// -------- PROTOTYPES ----------
void initEmgTask( void );
void runEmgTask( void );
void InitEmgTaskValues( void );

void setEmgAutoSync( UINT8 on );
void CollectHSData( UNS8);

#endif
 
