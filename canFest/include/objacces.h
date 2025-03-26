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
 * @file   objacces.h
 * @author Edouard TISSERANT and Francis DUPIN, modified by JDC
 * @brief Header file for objacces.c
 */

 
#ifndef __objacces_h__
#define __objacces_h__

#include <applicfg.h>

typedef UNS32 (*valueRangeTest_t)(UNS8 typeValue, void *Value);
typedef void (* storeODSubIndex_t)(CO_Data* d, UNS16 wIndex, UNS8 bSubindex);
void _storeODSubIndex (CO_Data* d, UNS16 wIndex, UNS8 bSubindex);

UNS8 accessDictionaryError(UNS16 index, UNS8 subIndex, 
			   UNS32 sizeDataDict, UNS32 sizeDataGiven, UNS32 code);



UNS32 _getODentry( CO_Data* d, 
		  UNS16 wIndex,
		  UNS8 bSubindex,
		  void * pDestData,
		  UNS32 * pExpectedSize,
		  UNS8 * pDataType,
		  UNS8 checkAccess,
		  UNS8 endianize);


#define getODentry( OD, wIndex, bSubindex, pDestData, pExpectedSize, \
		          pDataType,  checkAccess)                         \
       _getODentry( OD, wIndex, bSubindex, pDestData, pExpectedSize, \
		          pDataType,  checkAccess, 1)            

#define readLocalDict( OD, wIndex, bSubindex, pDestData, pExpectedSize, \
		          pDataType,  checkAccess)                         \
       _getODentry( OD, wIndex, bSubindex, pDestData, pExpectedSize, \
		          pDataType,  checkAccess, 0)


UNS32 _setODentry( CO_Data* d,
                   UNS16 wIndex,
                   UNS8 bSubindex,
                   void * pSourceData,
                   UNS32 * pExpectedSize,
                   UNS8 checkAccess,
                   UNS8 endianize);


#define setODentry( d, wIndex, bSubindex, pSourceData, pExpectedSize, \
                  checkAccess) \
       _setODentry( d, wIndex, bSubindex, pSourceData, pExpectedSize, \
                  checkAccess, 1)

#define writeLocalDict( d, wIndex, bSubindex, pSourceData, pExpectedSize, checkAccess) \
       _setODentry( d, wIndex, bSubindex, pSourceData, pExpectedSize, checkAccess, 0)




 const indextable * scanIndexOD (CO_Data* d, UNS16 wIndex, UNS32 *errorCode, ODCallback_t **Callback);

UNS32 RegisterSetODentryCallBack(CO_Data* d, UNS16 wIndex, UNS8 bSubindex, ODCallback_t Callback);

#endif /* __objacces_h__ */
