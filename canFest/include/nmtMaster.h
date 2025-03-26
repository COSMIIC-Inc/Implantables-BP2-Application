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
 * @file   nmtMaster.h
 * @author Edouard TISSERANT and Francis DUPIN, modified by JDC
 * @brief Header file for nmtMaster.c
*/


 
#ifndef __nmtMaster_h__
#define __nmtMaster_h__

#include "data.h"


UNS8 masterSendNMTstateChange (CO_Data* d, UNS8 nodeId, UNS8 cs);
UNS8 masterSendNMTnodeguard (CO_Data* d, UNS8 nodeId);
void masterRequestNodeState (CO_Data* d, UNS8 nodeId);


#endif /* __nmtMaster_h__ */
