/**
 * @file   eedata.c (common to MES and Stim)
 * @author JDC, J Hardway, 
 * @brief Select Data is saved to and restored from ee memory.	The data are identified
 *   by a list of Object indices.  The entire object (all sub indices) is stored/recalled.
 */

/** 
 * @defgroup eeprom EEPROM Save/Restore Custom Settings
 *   Custom (patient-specific) OD settings are stored in EEPROM and loaded at startup.  Can 
 *   also restore to OD defaults
 */
 

#include <stdio.h>
#include <stdlib.h>
#include "sys.h"
#include "objdict.h"
#include "objdictdef.h"
#include "eedata.h"
#include "iocan128.h"
#include "objacces.h"
#include "iar.h"



// --------   DATA   ------------



// -------- PROTOTYPES ----------




 
//============================
//    GLOBAL CODE
//============================


/**
 * @ingroup eeprom
 * @brief checks whether or not RestoreValues() should be run.  
 * @return 1=RestoreValues() should be run, 0=ResetToODDefault() was run, so don't restore from EEPROM
 */
UINT8 CheckRestoreFlag(void)
{
  UINT8 data[2];
  
  EEPROM_read(0x00, data, 2);
  if (data[0] == 0xFF && data[1] == 0xFF || data[0] == 0x00 && data[1] == 0x00)
    return 0;
  else
    return 1;
  
}




/*
 * @ingroup eeprom
 * @brief Saves the values of custom OD entries (specified in RestoreList OD index 0x2900) 
 *        to EEPROM from the OD.  Companion function to RestoreValues().  Note that if the RestoreList is changed, 
 *        RestoreValues() will not restore values to the correct subindices in the OD unless the EEPROM data 
 *        has been updated via SaveValues(). I.e., SaveValues() only stores the data bytes contained within the 
 *        OD subindices in the order specified by RestoreList; the indices and subindices themselves are not 
 *        stored in EEPROM.  
 * @details Called from NMT_Do_Restore_Cmd (only when in the Waiting Mode) and from the startup sequence.
 *          The first two bytes in EEPROM specify the amount EEPROM used by this function.  All OD indices used by the
 *          RestoreList MUST have more than one subindex, where the first subindex specifies the number of subindices.
 *          Takes ~9ms per byte that needs to be saved.
 */
void SaveValues( void )
{
  UINT8 nSubIndices;
  UNS32 size = 0;  
  UNS8  type = 0;  
  UINT8 data[MAX_BYTES_PER_SUBINDEX];
  UINT32 abortCode = 0;
  UINT16 counter = 2; //NOTE: counter starts at 2 (0 and 1 used to store size later)
  UINT8 i = 0;
  

  //loop through each index in RestoreList
  for (i = 0; i < sizeof(RestoreList); i++)
  {
    //don't let user save/restore below 0x1018 in OD, currently to make sure 1st subindex specifies nSubIndices
    if (RestoreList[i] < 0x1018)  
      continue; 
    
    //need to reset the size before the next "read" 
    size = 0;  
    type = 0;
    //read the 0th subindex (nSubIndices) at the current index
    abortCode = readLocalDict( &ObjDict_Data, RestoreList[i], 0, &nSubIndices, &size, &type, 0);  
    if ( abortCode != OD_SUCCESSFUL )
               break;

    if (type == uint8) // First subindex must be a UINT8 see objdictdef.h for datatypes
    {
        //loop through each subindex and write bytes to EEPROM
        for (int k = 1; k <= nSubIndices; k++) //the first element of subIndexSize must contain the number of subindices
        {
          size = 0;
          //?what happens here if subindex has more bytes than MAX_BYTES_PER_SUBINDEX?
          abortCode = readLocalDict( &ObjDict_Data, RestoreList[i], k, data, &size, &type, 0);
          if ( abortCode != OD_SUCCESSFUL )
                 break;
          
          //write all the subindex bytes to EEPROM
          if (counter+size < MAX_EEPROM_MEMORY )
          {
            EEPROM_write(counter, data, size );
            counter += size;
          }
          
        }
      }
  }
  //write the number of bytes used (current counter value) in the first 2 bytes of EEPROM
  data[0] = (UINT8)counter;
  data[1] = (UINT8)(counter >> 8);
  EEPROM_write(0, data, 2); 

  //NOTE: disabling/enabling interrupts is handled in EEPROM_write routine!
}




/*
 * @ingroup eeprom
 * @brief Restores the values of custom OD entries (specified in RestoreList OD index 0x2900) 
 *        to the OD from EEPROM.  Companion function to SaveValues().  Note that if the RestoreList is changed, 
 *        RestoreValues() will not restore values to the correct subindices in the OD unless the EEPROM data 
 *        has been updated via SaveValues(). I.e., SaveValues() only stores the data bytes contained within the 
 *        OD subindices in the order specified by RestoreList; the indices and subindices themselves are not 
 *        stored in EEPROM.  
 * @details Called from NMT_Do_Restore_Cmd (only when in the Waiting Mode) and from the startup sequence.
 *          All OD indices used by the RestoreList MUST have more than one subindex, where the first subindex 
 *          specifies the number of subindices.  
*/
void RestoreValues ( void )
{

  UINT8 nSubIndices;
  UNS32 size = 0;  
  UNS8  type = 0;  
  UINT8 data[MAX_BYTES_PER_SUBINDEX];  
  UINT32 abortCode = 0;;
  UINT16 counter = 2;
   
    for (int i = 0; i < sizeof(RestoreList); i++)
    {
      //don't let user save/restore below 0x1018 in OD, currently to make sure 1st subindex specifies nSubIndices
      if (RestoreList[i] < 0x1018)  
        continue; 
    
      size = 0;  
      type = 0;

      //read the 0th subindex (nSubIndices) at the current index
      abortCode = readLocalDict( &ObjDict_Data, RestoreList[i], 0, &nSubIndices, &size, &type, 0);
      if ( abortCode != OD_SUCCESSFUL )
               break;

      if ( type == uint8 ) 
      {
        //loop through each subindex and read bytes from EEPROM
        for (int k = 1; k <=  nSubIndices; k++)
        {
          size = 0;
          //read current values from OD to get length of subindex
          abortCode = readLocalDict( &ObjDict_Data, RestoreList[i], k, data, &size, &type, 0);
          if ( abortCode != OD_SUCCESSFUL )
                 break;
          
          //read all subindex bytes from EEPROM
          if( counter+size < MAX_EEPROM_MEMORY )
          {
              EEPROM_read( counter, data, size ); 
              counter += size;
          }
          //write data to OD
          abortCode = writeLocalDict( &ObjDict_Data, RestoreList[i], k, data, &size, 0);
          if ( abortCode != OD_SUCCESSFUL )
                 break;
        }
      }
    }  
    

}


/**
 * @ingroup eeprom
 * @brief  invoked by nmt_master -- writes 0's to size causing bypass of OD restore, then causes reset
 */
void ResetToODDefault(void)
{
  
  UINT8 data[2];
  data[0] = 0x00;
  data[1] = 0x00;
  EEPROM_write(0x00, data, 0x02);
  ResetModule();
}



/**
 * @ingroup eeprom
 * @brief resets module, called by ResetToODDefault()
 */
void ResetModule(void)
{
  DISABLE_INTERRUPTS();
  // disable the wdt if it was enabled from the reset
  WDTCR = (1 << WDCE);  
  WDTCR = 0; // clear the WDT enable bit
  
  
  WDTCR = (1 << WDCE)  | (1 << WDE);	
  WDTCR = 1 << WDE | (7);  // trigger set for 550msecs
  while(1); // loop until reset
  
}

/**
 * @ingroup eeprom
 * @brief reads CPU based flash data
 * @param nvAddress
 * @param pointer to data
 * @param size of data requested
 */
UNS8 ReadLocalFlashData( UNS32 nvAddress, UNS8 * data, UNS8 numData )
{ 
  UNS8 i = 0;
  
  if (nvAddress > 0xFFFF && nvAddress < (0x1FFFF - 32))
  {
    for (i = 0; i < numData; i++)
      data[i] = *(UINT8   __farflash  *)(nvAddress + i); 
  }
  else if ( nvAddress <= 0xFFFF )
  {
    memcpy( data, (UNS8 *)nvAddress , numData ); 
  }
  else
    return 2;
    
  return 0;
}


/**
 * @ingroup eeprom
 * @brief writes 0xFF to entire EEPROM (4KB). Takes 36s to complete! 
 */
void EraseEprom()
{
  UINT8 byteErase[EEPROM_ERASE_SIZE];

  for (int i = 0; i < sizeof(byteErase); i++)
    byteErase[i] = 0xFF; 
  
  for (int i = 0; i < MAX_EEPROM_MEMORY / sizeof(byteErase); i++)
    EEPROM_write(i*sizeof(byteErase), byteErase, sizeof(byteErase)); 

}

//============================
//    LOCAL CODE
//============================

/**
 * @ingroup eeprom
 * @brief Writes bytes to a specified location in EEPROM
 * @param address The address, relative to the start of the EEPROM where the data will be written.
 * @param *data pointer to the data (or array) to be written
 * @param length length of data to be written
 */
void EEPROM_write(UNS16 address, UNS8 * data, UNS16 length)
{
	int i = 0;
        while (length--)
        {
          while(EECR & (1<<EEWE));               
          EEAR = address++; 
          EEDR = data[i]; 
          
          DISABLE_INTERRUPTS();
          EECR |= (1<<EEMWE); /* Write logical one to EEMWE */ 
          EECR |= (1<<EEWE);  /* Start eeprom write by setting EEWE */ 
          ENABLE_INTERRUPTS();
          i++;
          
        }

}

/**
 * @ingroup eeprom
 * @brief Reads bytes from a specified location in EEPROM
 * @param address The address, relative to the start of the EEPROM where the data will be written.
 * @param *data pointer to the data (or array) to read to
 * @param length length of data to be read
 */
void EEPROM_read(UNS16 address, UNS8 * data, UNS16 length)
{
	while (length--)
        {
          while(EECR & (1<<EEWE));    /* Wait for completion of previous read */ 
        
          EEAR = address++; 
 
          EECR |= (1<<EERE);      
          *(data++) = EEDR;
        }

}


//============================
//    INTERRUPT SERVICE ROUTINES
//============================


//============================
//    HARDWARE SPECIFIC CODE
//============================







