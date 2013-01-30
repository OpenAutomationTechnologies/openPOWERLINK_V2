/* omethlib_phycfg.c - Ethernet Library for FPGA MAC Controller (Phy config) */
/* generic */
/*
------------------------------------------------------------------------------
Copyright (c) 2012, B&R
All rights reserved.

Redistribution and use in source and binary forms,
with or without modification,
are permitted provided that the following conditions are met:

- Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

- Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer
in the documentation and/or other materials provided with the distribution.

- Neither the name of the B&R nor the names of
its contributors may be used to endorse or promote products derived
from this software without specific prior written permission.


THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

------------------------------------------------------------------------------
 Module:    omethlib_phycfg
 File:      omethlib_phycfg.c
 Author:    Joerg Zelenka (zelenkaj)
 Created:   2012-06-04
 State:     -
------------------------------------------------------------------------------*/

#include <omethlib.h>
#include <omethlibint.h>
#include <omethlib_phycfg.h>

//-------------------------------------------------------------------------
// Add your defines below this line

// Add your defines above this line
//-------------------------------------------------------------------------

/*****************************************************************************
*
* omethPhyCfgUser - User callback for phy-specific configuration
*
*
* RETURN:
*     0    ... no error
*    -1    ... error
*        hEth invalid / port too high / reg too high
*
*/
int                     omethPhyCfgUser
(
    OMETH_H             hEth /* handle of ethernet driver, see omethCreate() */
)
{
    int iRet = 0;

    //-------------------------------------------------------------------------
    // Add your instructions below this line
/*
    //example with Marvell 88E1111 phys on TERASIC INK DE2-115
    //register = 24
    //LED_TX Control = "11" => "Transmit or receive activity - blink"
    //set bits 6 and 0
    int i;
    int iPhyCount = hEth->phyCount;
    unsigned short regData;

    //process all connected phys
    for(i=0; i<iPhyCount; i++)
    {
        //read phy register
        iRet = omethPhyRead(hEth, i, 24, &regData);
        if(iRet != 0) goto Exit;

        //set bits
        regData |= 0x0041;

        //write-back manipulated register content
        omethPhyWrite(hEth, i, 24, regData);
        if(iRet != 0) goto Exit;
    }
Exit:
*/
    // Add your instructions above this line
    //-------------------------------------------------------------------------

    return iRet;
}
