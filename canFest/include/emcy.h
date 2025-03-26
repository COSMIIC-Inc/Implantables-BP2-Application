/*
This file is part of CanFestival, a library implementing CanOpen Stack. 

Copyright (C): Edouard TISSERANT and Francis DUPIN

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
 * @file   emcy.h
 * @author Luis Jimenez, modified by JDC
 * @brief Header file for emcy.c
 */
 
				 
#ifndef __emcy_h__
#define __emcy_h__


#include <applicfg.h>

/* The error states 
 * ----------------- */
typedef enum enum_errorState {
  Error_free		= 0x00, 
  Error_occurred	= 0x01
} e_errorState;

typedef struct {
	UNS16 errCode;
	UNS8 errRegMask;
	UNS8 active;
} s_errors;

#include "data.h"


typedef void (*post_emcy_t)(CO_Data* d, UNS8 nodeID, UNS16 errCode, UNS8 errReg);
void _post_emcy(CO_Data* d, UNS8 nodeID, UNS16 errCode, UNS8 errReg);

/*************************************************************************
 * Functions
 *************************************************************************/


UNS8 EMCY_setError(CO_Data* d, UNS16 errCode, UNS8 errRegMask, UNS16 addInfo);
void EMCY_errorRecovered(CO_Data* d, UNS16 errCode);
void emergencyInit(CO_Data* d);
void emergencyStop(CO_Data* d);
void proceedEMCY(CO_Data* d, Message* m);

#endif /*__emcy_h__ */
