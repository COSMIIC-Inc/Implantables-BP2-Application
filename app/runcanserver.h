/**
 * @file   runCANserver.h
 * @author Coburn
 * @brief Header file for runCANserver.c 
 */  

   
#ifndef RUNCANSERVER_H
#define RUNCANSERVER_H

/**********************************************/
/*              Prototypes                    */
/**********************************************/

void InitCANServerTask( void );
void RunCANServerTask( void );
UNS8 processBOOT( CO_Data* d, Message *m );

#endif