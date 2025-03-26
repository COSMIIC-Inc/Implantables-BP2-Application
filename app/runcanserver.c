/**
 * @file   runCANserver.c
 * @author JDC
 * @brief Task is set up to be an IO scanner. It is part of the OS While loop
 * in app.c -- App_TaskStart(). It also initializes the CAN node for the Power
 * Module.
 */      

/**
 * @defgroup scanmanagement Scan Management of CAN link
 * @ingroup userapi
 */
/**
 * @defgroup canscanner CAN Scanner
 * @ingroup scanmanagement
 */

#include <stdlib.h>
#include "canfestival.h"
#include "can_AVR.h"
#include "objdict.h"
#include "runcanserver.h"
#include "sys.h"



// -------- DEFINITIONS ----------



// --------  Static DATA   ------------
// canOpen
static Message m = Message_Initializer;

/******************************************/
/*           Data                  */
/******************************************/
unsigned char nodeID;




/******************************************/
/*           Prototypes                   */
/******************************************/



/******************************************/
/*           Tasks                   */
/******************************************/
/**
 * @brief Initializes CAN Server
 */
void InitCANServerTask( void )
{
        /* this sets the port info and resets the mob*/ 	
         canInit( CAN_BAUDRATE ); 
        /* Used to set the CAN server into waiting mode;  */
        setState( &ObjDict_Data, Waiting );
}



/**
 * @brief Runs CAN Server, CAN processing
 */
void RunCANServerTask(void) 
{
    if (canReceive( &m )) 	 	
    {			
      canDispatch( &ObjDict_Data, &m );	  
    }
  
    Status_modeSelect = (UNS8)getState(&ObjDict_Data);
    UpdateCANerrors();
                         
}

/**
 * @brief process boot command - allows scanning RMs that are running apps
 */
UNS8 processBOOT( CO_Data* d, Message *m )
{
  Message mr;  
  if (m->cob_id = 0x140 && m->len == 3 && m->data[0] == Status_NodeId && m->data[1]==0 && m->data[2]==0)
  { 
    /* message configuration */
    mr.cob_id = 0x150;
    mr.rtr = NOT_A_REQUEST;
    mr.len = 7;
    mr.data[0] = 5; //module type 
    mr.data[1] = 2; //indicates that node is running app
    mr.data[2] = *(unsigned char __farflash *)0x1FFFE; //serial number LB
    mr.data[3] = *(unsigned char __farflash *)0x1FFFD; //serial number HB
    mr.data[4] = APP_REV; //app version low byte
    mr.data[5] = APP_REV >> 8;  //app version high byte
    mr.data[6] = Status_NodeId;
    
    return canSend(d->canHandle,&mr);
  }
  else
    return 0;
}