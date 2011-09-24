/****************************************************************************

   Trace-Funktion fuer Debugausgaben

   18.09.1996   -rs

****************************************************************************/


#include    <windows.h>
#include    <stdio.h>
#include    <stdarg.h>



void trace (const char *fmt, ...)
{

char    Buffer[0x400];
va_list argptr;


   va_start (argptr, fmt);
#if _MSC_VER >= 1400
   vsprintf_s (Buffer, sizeof (Buffer), fmt, argptr);
#else
   vsprintf (Buffer, fmt, argptr);
#endif
   va_end   (argptr);

   OutputDebugString ((LPSTR)&Buffer);

}


