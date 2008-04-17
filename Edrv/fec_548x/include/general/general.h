/************************************************************************
*
*  FREESCALE INC.
*  ALL RIGHTS RESERVED
*  (c) Copyright 2003 FREESCALE, Inc.
*
*************************************************************************
*
*  FILE NAME: general.h
*
*  PURPOSE: General data types and definitions.
*
*  AUTHOR: Andrey Butok
*
***********************************************************************/
#ifndef _GENERAL_H_
#define	_GENERAL_H_

//#define __NET_HEAP_START   (0x00100000)
#define __NET_HEAP_START   (0x1020000)
#define __NET_HEAP_END     (0x1220000)



typedef unsigned char mac_addr[6];  // MAC address type.
typedef unsigned long ip_addr;      // IP addresss type.

#ifdef	FALSE
#undef	FALSE
#endif
#define FALSE	(0)

#ifdef	TRUE
#undef	TRUE
#endif
#define	TRUE	(1)

#ifdef	NULL
#undef	NULL
#endif
#define NULL	(0)

#define	INADDR_NONE		 (ERR)
#define	INADDR_ANY       (unsigned long)0x00000000
#define	INADDR_BROADCAST (unsigned long)0xffffffff
#define	ADDR_ANY         (unsigned long)0x00000000 

#define SCHAR_MAX   (127)           // Min value for a signed char.
#define SCHAR_MIN   (-128)          // Max value for a signed char.

#define UCHAR_MAX   (255)           // Max value for an unsigned char.
#define CHAR_MAX    (127)           // Max value for a char.
#define CHAR_MIN    (-128)          // Min value for a char.

#define USHRT_MAX   (65535)         // Max value for an unsigned short.
#define SHRT_MAX    (32767)         // Max value for a short.
#define SHRT_MIN    (-32768)        // Min value for a short.

#define UINT_MAX    (0xffffffff)    // Max value for an unsigned int.
#define INT_MAX	    (2147483647)    // Max value for an int.
#define INT_MIN	    (-2147483647-1) // Min value for an int.

#define ULONG_MAX   (0xffffffff)    // Max value for an unsigned long.
#define LONG_MAX    (2147483647)    // Max value for a long.
#define LONG_MIN    (-2147483647-1) // Min value for a long.


#endif
