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
 * @file   nmtSlave.h
 * @author Edouard TISSERANT and Francis DUPIN, modified by JDC
 * @brief Header file for nmtSlave.c
*/
 
#ifndef __nmtSlave_h__
#define __nmtSlave_h__

#include <applicfg.h>
#include "data.h"

static UNS8 skipTime = 0;
void processNMTstateChange (CO_Data* d, Message * m);
UNS8 slaveSendBootUp (CO_Data* d);
void StartWatchDog ( CO_Data* d, UNS16 timebase );
void StopWatchDog ( CO_Data* d );
void StartRecording (UNS8 data);
void setNodeStateToStopped( void ); 
#endif /* __nmtSlave_h__ */
