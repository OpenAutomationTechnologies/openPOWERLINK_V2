/*-------------------------------------------*/
/* Integer type definitions for FatFs module */
/*-------------------------------------------*/

#ifndef _INTEGER
#define _INTEGER

#ifdef _WIN32   /* FatFs development platform */

#include <tchar.h>

#else           /* Embedded platform */

/* These types must be 16-bit, 32-bit or larger integer */
#ifndef INT
typedef int INT;
#endif
#ifndef UINT
typedef unsigned int UINT;
#endif
/* These types must be 8-bit integer */
#ifndef CHAR
typedef char CHAR;
#endif
#ifndef UCHAR
typedef unsigned char UCHAR;
#endif
#ifndef BYTE
typedef unsigned char BYTE;
#endif

/* These types must be 16-bit integer */
#ifndef SHORT
typedef short SHORT;
#endif
#ifndef USHORT
typedef unsigned short USHORT;
#endif
#ifndef WORD
typedef unsigned short WORD;
#endif
#ifndef WCHAR
typedef unsigned short WCHAR;
#endif

/* These types must be 32-bit integer */
#ifndef LONG
typedef long LONG;
#endif
#ifndef ULONG
typedef unsigned long ULONG;
#endif
#ifndef DWORD
typedef unsigned long DWORD;
#endif

#endif

#endif
