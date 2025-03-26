/*
This file is part of CanFestival, a library implementing CanOpen Stack. 

Copyright (C): Edouard TISSERANT and Francis DUPIN
modified by JDC Consulting

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
 * @file   lifegrd.h
 * @author Edouard TISSERANT, modified by JDC
 * @brief Header file for lifegrd.c
 */
					 
#ifndef __lifegrd_h__
#define __lifegrd_h__


#include <applicfg.h>

typedef void (*heartbeatError_t)(CO_Data*, UNS8);
void _heartbeatError(CO_Data* d, UNS8 heartbeatID);

typedef void (*post_SlaveBootup_t)(CO_Data*, UNS8);
void _post_SlaveBootup(CO_Data* d, UNS8 SlaveID);

#include "data.h"

/*************************************************************************
 * Functions
 *************************************************************************/


e_nodeState getNodeState (CO_Data* d, UNS8 nodeId);
void heartbeatInit(CO_Data* d);
void heartbeatStop(CO_Data* d);
void processNODE_GUARD (CO_Data* d, Message* m);

#endif /*__lifegrd_h__ */
