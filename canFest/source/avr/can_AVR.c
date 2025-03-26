/*
This file is part of CanFestival, a library implementing CanOpen Stack.

Copyright (C): Edouard TISSERANT and Francis DUPIN
AVR Port: Andreas GLAUSER and Peter CHRISTEN

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
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

/**
 * @file   can_AVR.c
 * @author Edouard TISSERANT, modified by JDC
 * @brief <BRIEF> can_AVR.c
 */

//#define DEBUG_WAR_CONSOLE_ON
//#define DEBUG_ERR_CONSOLE_ON


#include "can_AVR.h"
#include "canfestival.h"
#include "iocan128.h"
#include "objdict.h"


// -- prototypes --
//unsigned char protCanSend(CAN_PORT notused, Message *m);

// -- data ---
volatile unsigned char msg_received = 0;

volatile UNS16 BitErrors = 0 ;
volatile UNS16 StuffErrors = 0;
volatile UNS16 FormErrors = 0;
volatile UNS16 OtherErrors = 0;
volatile UNS16 receivedMessages = 0;
volatile UNS8 syncPulse = 0;



/**
 * @brief Initialize the hardware to receive CAN messages and start the timer for the
 * @details
 *   CAN bit timing
 *    ---------------
 *     * the CAN clock determines the length of the quanta (tq)
 *	  * 1 CAN bit is composed of 8..25 quanta
 *        * the CAN clock and quanta per bit determine the CAN bitrate
 *  
 *         |<------------------- 1 CAN bit --------------------->|
 *         |<1tq>|                                               |
 *         |SYNC |      PRS      |     PHS1      |    PHS2       |
 *                                               ^
 *                                         sample point
 *  
 *   NNPS CAN clock = 1MHz (tq = 1/CAN clock)
 *    bit rate = CAN clock / tq per bit = 100 kbit
 *
 *   all values are added to 1 by hardware
 *   Synchronization segment              = 1 tq (set in hardware)
 *   Propagation Time Segment (PRS)       = 3 tq
 *   Phase Segment 1 (PHS1)               = 3 tq
 *   Phase Segment 2 (PHS2)               = 3 tq
 *   Total tq per bit                     = 10 tq
 *
 *   (re)Synchronisation Jump Width (SJW) = 3 tq (how many quanta the sample point can be moved to account for phase errors)
 *   SaMPle points (SMP)                  = 1 (0=single sample, 1=best 2 out of 3 samples)
 *
 *   CANBT1 = ((F_CPU/16/1000/bitrate-1) << BRP);	// set bitrate
 * CANopen stack.
 * @param bitrate (in kilobit)
 * @return 1 if successful	
 */
unsigned char canInit(unsigned int bitrate)
{
  unsigned char i,k,j;
  UNS32 PDOData[4];
  UNS8 type = 0;
  UNS32 size = 0;
  UNS32 * pdata;
  
  UNS16 mask0 = 0xFFFF;
  UNS16 mask1 = 0x000F;
  
  for ( j = 0; j < 4; j++)
  {
    getODentry( &ObjDict_Data,
                   0x1400 + j,
                   1,
                   pdata,
                   &size,
                   &type,
                   0);
    PDOData[j] = *pdata;
  }
  UNS16 identifiers[] = { 0x0080, (UNS16)getNodeId(&ObjDict_Data), (UNS16)PDOData[0], (UNS16)PDOData[1], (UNS16)PDOData[2], (UNS16)PDOData[3], \
    0x0000, 0x00140, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 };
  
    //- Pull-up on TxCAN & RxCAN one by one to use bit-addressing
  CAN_PORT_DIR &= ~(1<<CAN_INPUT_PIN );
  CAN_PORT_DIR &= ~(1<<CAN_OUTPUT_PIN);
  CAN_PORT_OUT |=  (1<<CAN_INPUT_PIN );
  CAN_PORT_OUT |=  (1<<CAN_OUTPUT_PIN);
  
  Can_reset();				// Reset the CAN controller
  
  if (bitrate <= 500)
  {

    #if (FOSC == 8000)
          CANBT1 = 0x0E;
          CANBT2 = ((3-1) << SJW) |((3-1) << PRS);	// set SJW, PRS
          CANBT3 = (((3-1) << PHS2) | ((3-1) << PHS1) | (1<<SMP)); // set PHS1, PHS2, 3 samples
    #endif
    #if (FOSC == 4000)
          CANBT1 = 0x06;
          CANBT2 = ((3-1) << SJW) |((3-1) << PRS);	// set SJW, PRS
          CANBT3 = (((3-1) << PHS2) | ((3-1) << PHS1) | (1<<SMP)); // set PHS1, PHS2, 3 samples
    #endif
    #if (FOSC == 2000)
          CANBT1 = 0x02;
          CANBT2 = ((3-1) << SJW) |((3-1) << PRS);	// set SJW, PRS
          CANBT3 = (((3-1) << PHS2) | ((3-1) << PHS1) | (1<<SMP)); // set PHS1, PHS2, 3 samples
    #endif
    #if (FOSC == 1000)
          CANBT1 = 0x00;
          CANBT2 = ((3-1) << SJW) |((3-1) << PRS);	// set SJW, PRS
          CANBT3 = (((3-1) << PHS2) | ((3-1) << PHS1) | (0<<SMP)); // set PHS1, PHS2, only 1 sample possible due to AVR constraints
    #endif
 
  }
  else 
    return 0;
  
  // Reset all mailsboxes (MObs), filters are zero (accept all) by clear all MOb
  // Set the lower MObs as rx buffer
  for (i = 0; i < NB_MOB; i++)
  {
    Can_set_mob(i);		// Change to MOb with the received message
    Can_clear_mob();		// All MOb Registers=0
    for (k = 0; k < NB_DATA_MAX; k++)
      CANMSG = 0;		// MOb data FIFO
    if (i < NB_RX_MOB)		// Is receive MOb
    {
      Can_config_rx_buffer();	// configure as receive buffer
      if (i == 1)
      {
        Can_set_std_msk(mask1);       // set mask for this mailbox to 0xFFFF (compare everything)
      }
      else
      {
        Can_set_std_msk(mask0);
      }
      
      Can_set_std_id(identifiers[i]); 
      
    }
    
  }
  // The tx MOb is still disabled, it will be set to tx mode when the first message will be sent
  // Enable the general CAN interrupts
  CANGIE = (1 << ENIT) | (1 << ENRX) | (1 << ENTX) | (0 << ENERR) | (0 << ENERG) | (0 << ENOVRT);
  CANIE1 = 0x7F;	// Enable the interrupts of all MObs (0..14)
  CANIE2 = 0xFF;   
  Can_enable();                                 // Enable the CAN bus controller
  return 1;
}

/**
 * @brief Initialize the hardware to receive CAN messages and set up filters based on OD settings
 */
void UpdateCANerrors(void)
{
 
      CAN_BitErrors = BitErrors;
      CAN_StuffErrors = StuffErrors;
      CAN_FormErrors = FormErrors;
      CAN_OtherErrors = OtherErrors;
      CAN_Rx_ErrCounter = (UNS16)CANREC;
      CAN_Tx_ErrCounter = (UNS16)CANTEC;
      CAN_Receive_BEI !=  (0xFF & CANGCON);
      CAN_Interrupts_Off |= (0xFF & CANGSTA);
      
  
}

//JML NOTE: This function is not in the official CANFestival code 
/*unsigned char canSend(CAN_PORT notused, Message *m)
{
	// Protect cansend from being reentered by heartbeat while processing for SDO, etc.
	
	unsigned char status;
	
	
	// DISAble the TIMEDISPATCH() interrupt
	TIMSK3 &= ~(1 << OCIE3B) ;     		
	
	
	status = protCanSend( notused, m );
		
	// ENAble the TIMEDISPATCH() interrupt
	TIMSK3 |= (1 << OCIE3B) ;
        
	CAN_Transmit_Messages++;	// diagnostic counter
	return status;
}*/


/**
 * @brief The driver send a CAN message passed from the CANopen stack
 * @param notused (only 1 avaiable)
 * @param *m pointer to message to send
 * @return 1 if  hardware -> CAN frame
 */
unsigned char canSend(CAN_PORT notused, Message *m)
{
  unsigned char i;
  
  cli(); //JML:DISABLE INTERRUPTS
  for (i = START_TX_MOB; i < NB_MOB; i++)	// Search the first free MOb
  {
    Can_set_mob(i);			// Change to MOb
    if ((CANCDMOB & CONMOB_MSK) == 0)	// MOb disabled = free
    {
      break;
    }
  }
  
  if (i < NB_MOB)			// free MOb found
  {
    Can_set_mob(i);			// Switch to the sending messagebox
    Can_set_std_id(m->cob_id);		// Set cob id
    if (m->rtr)				// Set remote transmission request
      Can_set_rtr();
    Can_set_dlc(m->len);		// Set data lenght code

    for (i= 0; i < (m->len); i++)	// Add data bytes to the MOb
      CANMSG = m->data[i];
  // Start sending by writing the MB configuration register to transmit
    Can_config_tx();		// Set the last MOb to transmit mode
    sei(); //JML:RE-ENABLE INTERRUPTS
    return 1;	// succesful
  }
  else
  {
    sei();  //JML:RE-ENABLE INTERRUPTS
    return 0;	// not succesful
  }
}

/**
 * @brief The driver passes a received CAN message to the stack
 * @param *m pointer to received CAN message
 * @return 1 if a message received
 */
unsigned char canReceive(Message *m)

{
  unsigned char i;
  cli();  //JML:DISABLE INTERRUPTS
  if (msg_received == 0) // check the stack for a received message (note this is a counter)
    return 0;		// Nothing received
  
  CAN_Receive_Messages++;         
  
  
  for (i = 0; i < NB_RX_MOB; i++)	// Search the first MOb received
  {
    Can_set_mob(i);			// Change to MOb
    if ((CANCDMOB & CONMOB_MSK) == 0)	// MOb disabled = received
    {
      msg_received--;
      break;
    }
  }
  
  
  if (i < NB_RX_MOB)			// message found
  { 
    Can_get_std_id(m->cob_id);		// Get cob id
    m->rtr = Can_get_rtr();		// Get remote transmission request
    m->len = Can_get_dlc();		// Get data length code
    for (i= 0; i < (m->len); i++)	// get data bytes from the MOb
      m->data[i] = CANMSG;
    Can_config_rx_buffer();		// reset the MOb for receive
                    // diagnostic counter
    sei();  //JML:RE-ENABLE INTERRUPTS
    return 1;                  		// message received
  }
  else					// no message found
  {
    msg_received = 0;			// reset counter
    sei(); //JML:RE-ENABLE INTERRUPTS
    return 0;                  		// no message received
  }
}

/**
 * @brief
 * @param fd
 * @param baud
 * @return always 0
 */
unsigned char canChangeBaudRate_driver( CAN_HANDLE fd, char* baud)
{

	return 0;
}



#pragma type_attribute = __interrupt
#pragma vector=CANIT_vect
/**
 * @brief CAN Interrupt -- single interrupt for all MOb
 */
void CANIT_interrupt(void)
{
  unsigned char i;
  
  if (CANGIT & (1 << CANIT))	// is a messagebox interrupt
  {
    if ((CANSIT1 & TX_INT_MSK) == 0)	// is a Rx interrupt
    {
        for (i = 0; (i < NB_RX_MOB) && (CANGIT & (1 << CANIT)); i++)	// Search the first MOb received
        {
          Can_set_mob(i);			// Change to MOb using page
          if (CANSTMOB & MOB_RX_COMPLETED)	// receive ok
          {
            Can_clear_status_mob();	    // Clear status register
            Can_mob_abort();		    // disable the MOb = received
            if ( CANIDT1 == 0x10 )          // check for sync pulse (0x80)
            {                               // note that cobID is made up of CANIDT1 and CANIDT2
              syncPulse = 1;
            }
            msg_received++;                  // notify message received (including sync pulse)
          }
          else if (CANSTMOB & ~MOB_RX_COMPLETED)	// error
          {
            switch (CANSTMOB)
            {
            case BERR:
              BitErrors++;
              break;
            case SERR:
              StuffErrors++;
              break;
            case CERR:
              OtherErrors++;
              break;
            case FERR:
              FormErrors++;
              break;
            case AERR:
              OtherErrors++;
              break;
            
            }         
            Can_clear_status_mob();	// Clear status register
            Can_config_rx_buffer();	// reconfigure as receive buffer
          }
        }
      
    }
    else				// is a Tx interrupt	 
    {
      for (i = NB_RX_MOB; i < NB_MOB; i++)	// Search the first MOb transmitted
      {
        Can_set_mob(i);			// change to MOb
        if (CANSTMOB)			// transmission ok or error
        {
          Can_clear_status_mob();	// clear status register
	  CANCDMOB = 0;			// disable the MOb
	  break;
        }
      }
    }
  }

  // Bus Off Interrupt Flag
  if (CANGIT & (1 << BOFFIT))    // Finaly clear the interrupt status register
  {
    CANGIT |= (1 << BOFFIT);                    // Clear the interrupt flag
  }
  else
    CANGIT |= (1 << BXOK) | (1 << SERG) | (1 << CERG) | (1 << FERG) | (1 << AERG);// Finaly clear other interrupts
  
}


#ifdef  __IAR_SYSTEMS_ICC__
#pragma type_attribute = __interrupt
#pragma vector=OVRIT_vect

/**
 * @brief CAN Timer Interrupt
 */
void OVRIT_interrupt(void)
#else	// GCC
ISR(OVRIT_vect)
#endif	// GCC

{
  CANGIT |= (1 << OVRTIM);
}

