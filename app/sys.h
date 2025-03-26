//    file: sys.h    System (common) HEADER FILE.

#ifndef _SYS_H
#define _SYS_H

#include <ioavr.h>
#include <intrinsics.h>
#include "iar.h"


#define OSC_FREQ	8000000L



// -------- DEFINITIONS ----------


typedef signed char 	INT8;
typedef signed int 		INT16;
typedef signed long 	INT32;

typedef unsigned char 	UINT8;
typedef unsigned int	UINT16;
typedef unsigned long	UINT32;

typedef union
{	UINT8  b[2];
	UINT16 w;
}BW16;

typedef union
{	UINT8  b[4];
	UINT16 u[2];
	UINT32 w;
}BW32;

#define TRUE    1
#define FALSE   0
 
#define ON      1
#define OFF     0

#define CLR_BITS(a,b)     ((a)&=~(b))
#define  SET_BITS(a,b)     ((a)|=(b))
#define BITS_TRUE(a,b)    ((a)&(b))

#define ENABLE_INTERRUPTS()		asm("sei")
#define DISABLE_INTERRUPTS()	asm("cli")
#define RESET_WDOG()			asm("wdr")

#define B(n)			(1<<(n))

#define BIT0			B(0)
#define BIT1			B(1)
#define BIT2			B(2)
#define BIT3			B(3)
#define BIT4			B(4)
#define BIT5			B(5)
#define BIT6			B(6)
#define BIT7			B(7)
#define BIT15			B(15)




#endif
 
