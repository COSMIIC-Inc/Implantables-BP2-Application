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
 * @file   timerscfg.h
 * @author Edouard TISSERANT and Francis DUPIN, modified by JDC
 * @brief Header file for AVR configuration of timer for CAN Festival
 */

#ifndef __TIMERSCFG_H__
#define __TIMERSCFG_H__

// Whatever your microcontroller, the timer wont work if
// TIMEVAL is not at least on 32 bits
#define TIMEVAL UNS32

// The timer of the AVR counts from 0000 to 0xFFFF in CTC mode (it can be
// shortened setting OCR3A eg. to get 2ms instead of 2.048ms)
#define TIMEVAL_MAX 0xFFFF

#if (FOSC == 8000)
// The timer is incrementing every 8 us at 8 Mhz/64 clock.
#define MS_TO_TIMEVAL(ms) ((ms) * 125UL) //JML added UL otherwise may overflow
#define US_TO_TIMEVAL(us) ((us)>>3)
#endif
#if (FOSC == 4000)
// The timer is incrementing every 16 us at 4 Mhz/64 clock.
// clock will be slightly slow (63/62.5=1.008)*ms
#define MS_TO_TIMEVAL(ms) ((ms) * 63UL) //JML added UL otherwise may overflow
#define US_TO_TIMEVAL(us) ((us)>>4)
#endif
#if (FOSC == 2000)
// The timer is incrementing every 32 us at 2 Mhz/64 clock
// clock will be slightly fast (31/31.25=0.996)*ms
#define MS_TO_TIMEVAL(ms) ((ms) * 31UL) //JML added UL otherwise may overflow
#define US_TO_TIMEVAL(us) ((us)>>5)
#endif
#if (FOSC == 1000)
// The timer is incrementing every 64 us at 2 Mhz/64 clock
// clock will be slightly slow (16/15.625=1.024)*ms
#define MS_TO_TIMEVAL(ms) ((ms) * 16UL) //JML added UL otherwise may overflow
#define US_TO_TIMEVAL(us) ((us)>>6)
#endif

#endif
