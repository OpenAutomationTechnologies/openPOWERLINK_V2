/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  definitons for the psc uart

  -------------------------------------------------------------------------

                $RCSfile$

                $Author$

                $Revision$  $Date$

                $State$

                Build Environment:
                DevC++
                GNU C 3.3.2

  -------------------------------------------------------------------------

  Revision History:

  2005/08/04 -ct:   start of the implementation

****************************************************************************/

#ifndef _UARTDRV_H_
#define _UARTDRV_H_


//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------



//---------------------------------------------------------------------------
// types
//---------------------------------------------------------------------------



//---------------------------------------------------------------------------
// function prototypes
//---------------------------------------------------------------------------

void UartInit        (int Port_p, uint32 dwBaudrate_p);
char UartInChar      (int Port_p);
void UartOutChar     (int Port_p, char cData_p);
int  UartCharPresent (int Port_p);

#endif  // #ifndef _UARTDRV_H_

