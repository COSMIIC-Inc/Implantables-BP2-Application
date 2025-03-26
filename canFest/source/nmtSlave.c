/*
  This file is part of CanFestival, a library implementing CanOpen
  Stack.

  Copyright (C): Edouard TISSERANT and Francis DUPIN
  modified by JDC Consulting. Master copy kept at
  https://jdc-consulting.unfuddle.com/svn/jdc-consulting_mastercodecanfestival/
  See COPYING file for copyrights details.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307
  USA
*/

/**
 * @file   nmtSlave.c
 * @author Edouard TISSERANT and Francis DUPIN, modified by JDC
 * @brief This file is used only with the operational slave -
 * usually the Remote Modules.
*/

/** 
 * @defgroup nmtslave NMT Slave
 *  @brief The NMT Slave methods are called automatically when a NMT message from Master are received. 
 *  @ingroup networkmanagement
 */

#include "nmtSlave.h"
#include "states.h"
#include "canfestival.h"
#include "sysdep.h"
#include "objdict.h"
#include "eedata.h"
#include "app.h"
#include "acceltemp.h"
#include "meas.h"


/** 
 * @ingroup nmtslave
 * @brief 
 * Manage the reception of a NMT message from the master.
 * @param *d Pointer to the CAN data structure
 * @param *m Pointer to the message received
 * @return 
 *  -  0 if OK 
 *  - -1 if the slave is not allowed, by its state, to receive the message
 */
void processNMTstateChange(CO_Data* d, Message *m)
{
  
  // check to see if the system can handle external messages
  if( d->nodeState != Hibernate ||
      d->nodeState != BootCheckReset ) 
  {

    MSG_WAR(0x3400, "NMT received. for node :  ", (*m).data[1]);

    // Check if this NMT-message is for this node 
    // byte 1 = 0 : all the nodes are concerned (broadcast)  
    if( ( (*m).data[1] == 0 ) || ( (*m).data[1] == getNodeId(d)) )
    {
      switch( (*m).data[0]) /* NMT command specifier (cs) in def.h*/
      { 
        // command interpreter
        
        case NMT_Start_Nodes:
          StartNodesFunc(d, m);
          break;
          
        case NMT_Stop_Nodes:
          StopNodesFunc(d, m);
          break;
          
        case NMT_Enter_Wait_Mode: 
          EnterWaitingFunc(d, m);
          break;
          
        case NMT_Enter_Patient_Operation:
          EnterPatientOperationFunc(d, m);
          break;  
          
        case NMT_Enter_X_Manual:
          EnterXManualFunc(d, m);
          break;
          
        case NMT_Enter_Y_Manual:
          EnterYManualFunc(d, m);
          break;
          
        case NMT_Enter_Stop_Stim:  
          EnterStopStimFunc(d, m);
          break;
          
        case NMT_Enter_Patient_Manual:
          EnterPatientManualFunc(d, m);
          break;
          
        case NMT_Enter_Produce_X_Manual:
          EnterProduceXManualFunc(d, m);
          break;
          
        case NMT_Enter_Record_X:
          EnterRecordXFunc(d, m);
          break;
          
        case NMT_Start_Sync:
          startSYNC(d);
          break;
          
        case NMT_Stop_Sync:
          stopSYNC(d);
          break;
          
        case NMT_Start_PDO:
          PDOInit(d);
          break;
          
        case NMT_Stop_PDO:
          PDOStop(d);
          break;
          
        case NMT_Reset_Watchdog:
          if ( (*m).data[1] == *d->bDeviceNodeId) // must be targeted to this node
              StartWatchDog(d, 10000);
          break;  
          
        case NMT_Reset_Node:
           if(d->NMT_Slave_Node_Reset_Callback != NULL)
              d->NMT_Slave_Node_Reset_Callback(d);
           if ((*m).data[1] == *d->bDeviceNodeId)
              setState(d,Waiting);
          break;
          
        case NMT_Reset_OD_Defaults: 
          if (d->nodeState == Waiting || d->nodeState == Stopped )
           ResetToODDefault();
          break;
          
        case NMT_Reset_Module: 
          if (d->nodeState == Waiting || d->nodeState == Stopped )
           ResetModule();
          break;
          
        case NMT_Clear_CAN_Errors:
          CAN_FormErrors = 0;
          CAN_StuffErrors = 0;
          CAN_BitErrors = 0;
          CAN_OtherErrors = 0;
          CAN_TotalErrors = 0;
          CAN_Receive_BEI = 0;
          CAN_Receive_Messages = 0;
          CAN_Transmit_Messages = 0;
          break;   
          
       case NMT_Erase_Serial_Eprom:
          if (d->nodeState == Waiting) 
            EraseEprom();
          break;

       case NMT_Do_Save_Cmd:   
          if (d->nodeState == Waiting)
            SaveValues();		
          break;
        
       case NMT_Do_Restore_Cmd:  
          if (d->nodeState == Waiting)
            RestoreValues();
          break;
       
        case NMT_Init_NV_Memory: //JML: why? what for
            //if (d->nodeState == Waiting)
          break;
        case NMT_Set_Gains: 
          if ( (*m).data[1] == *d->bDeviceNodeId)  //must be targeted at this node
          {
            setGains((*m).data[2], (*m).data[3]); //Param1, Param2
          }
          break;
          
        case NMT_Set_AntiAliasing_High:
          if ( (*m).data[1] == *d->bDeviceNodeId)  //must be targeted at this node
          {
              ANTIALIAS_EMG1_HF();
              ANTIALIAS_EMG2_HF();
          }
          break;
        
        case NMT_Set_AntiAliasing_Low:
          if ( (*m).data[1] == *d->bDeviceNodeId)  //must be targeted at this node
          {
              ANTIALIAS_EMG1_LF();
              ANTIALIAS_EMG2_LF();
          }
          break; 
        case NMT_Enter_Low_Power:
          //must be in waiting and targeted to this node
          //assume that only way to come out of sleep mode is power cycle
          if ( ((*m).data[1] == *d->bDeviceNodeId) && (d->nodeState == Waiting))
          {
            //do other power savings things here before sleeping
            ADCSRA &= ~BIT7; //turn off ADC
            sleepAccelerometer();
            TWCR &= ~BIT2; //turn off I2C
            PORTG &= ~BIT2; //turn off Heartbeat LED
            
            SMCR |= BIT2|BIT0; //Set Sleep Enable in Power-down mode
            asm("SLEEP");
          }
          break;  
        case NMT_Reset_Comunication:
           if ( ((*m).data[1] == *d->bDeviceNodeId) && (d->nodeState == Waiting))
           {
            UNS8 newNodeId = (*m).data[2]; //Param1
           
              if(d->NMT_Slave_Communications_Reset_Callback != NULL)
                 d->NMT_Slave_Communications_Reset_Callback(d);
      #ifdef CO_ENABLE_LSS
              
              if(newNodeId != d->lss_transfer.nodeID)
                 d->lss_transfer.nodeID = newNodeId;
      #endif
                
              Status_NodeId = newNodeId;
              SaveValues();
              ResetModule();
           }
           break;

       case NMT_Set_Analog_Pin_Config:
          if ( (*m).data[1] == *d->bDeviceNodeId)  //must be targeted at this node
          {
              UNS8 val = (*m).data[2];
              //bit0:PB4/AVDD_CTRL PORTC
              //bit1:PB4/AVDD_CTRL DDRF
              //bit2:PC3/n3V3_CTRL PORTC
              //bit3:PC3/n3V3_CTRL DDRC
              //bit4:JTD
              //bit5:PF4/5 TCK/TMS PORTF
              //bit6:PF4/5 TCK/TMS DDRF
              if(val & BIT0)
                PORTB |= BIT4;
              else
                PORTB &= ~BIT4;
              
              if(val & BIT1)
                DDRB |= BIT4;
              else
                DDRB &= ~BIT4;
              
              if(val & BIT2)
                PORTC |= BIT3;
              else
                PORTC &= ~BIT3;
              
              if(val & BIT3)
                DDRC |= BIT3;
              else
                DDRC &= ~BIT3;

              if(val & BIT4)
              { 
                if ((MCUCR & BIT7)!=BIT7) //not already set
                {
                  MCUCR |= BIT7;  //set JTD (disable JTAG)
                  MCUCR |= BIT7;  //repeat command on JTD (see AVRAT90CAN128 manual)
                }
              }
              else 
              {
                if ((MCUCR & BIT7)==BIT7) //already set
                {
                  MCUCR &= ~BIT7; //clear JTD (enable JTAG)
                  MCUCR &= ~BIT7; //repeat command on JTD (see AVRAT90CAN128 manual)
                }
              }
              
              //TCK/TMS
              if(val & BIT5)
                PORTF |= (BIT4|BIT5);
              else
                PORTF &= ~(BIT4|BIT5);
              
              if(val & BIT6)
                DDRF |= (BIT4|BIT5);
              else
                DDRF &= ~(BIT4|BIT5);
          }
          break; 
          
      }/* end switch */

    }
  }
}


/** 
 * @ingroup nmtslave
 * @brief Transmit the boot-Up frame when the slave is moving from initialization
 * state to pre_operational state.
 * @param *d Pointer on the CAN data structure
 * @return canSend(bus_id,&m)
 */
UNS8 slaveSendBootUp(CO_Data* d)
{
  Message m;

#ifdef CO_ENABLE_LSS
  if(*d->bDeviceNodeId==0xFF)return 0;
#endif

  MSG_WAR(0x3407, "Send a Boot-Up msg ", 0);

  /* message configuration */
  {
	  UNS16 tmp = NODE_GUARD << 7 | *d->bDeviceNodeId; 
	  m.cob_id = UNS16_LE(tmp);
  }
  m.rtr = NOT_A_REQUEST;
  m.len = 1;
  m.data[0] = 0x00;

  return canSend(d->canHandle,&m);
}


//=====================
//	WATCH DOG ROUTINES
//=====================

static CO_Data *wdogCoData;

/** 
 * @ingroup nmtslave
 * @brief Starts watchdog on slave
 * @param *d Pointer on the CAN data structure
 * @param timebase
 */
void StartWatchDog( CO_Data* d, UNS16 timebase )
{
  	//setAppTimeOut1( timebase );
	
	wdogCoData = d;
  
}

/** 
 * @ingroup nmtslave
 * @brief Stops watchdog on slave
 * @param *d Pointer on the CAN data structure
 */
void StopWatchDog( CO_Data* d )
{
  	//setAppTimeOut1( 0 );
  
  	wdogCoData = d;
  
}

/** 
 * @ingroup nmtslave
 * @brief <BRIEF> setNodeStateToStopped
 */
void setNodeStateToStopped( void )
{
	if( wdogCoData != NULL )
	{
		setState( wdogCoData, Stopped);
	}
	
}



