/* omethlib_phycfg.c - Ethernet Library for FPGA MAC Controller (Phy config) */
/* specific configuration of KSZ8051RNL on BeMicro Evaluation Board */
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
#define PHYCFG_KSZ8051RNL_PHYCFG2           0x1F
#define PHYCFG_KSZ8051RNL_PHYCFG2_RMIICLK   0x0080
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
    int iPhyCount = hEth->phyCount;

    //-------------------------------------------------------------------------
    /* Micrel phys on BeMicro use 50 MHz clock on X1
     * => manually set 0x1F.7
     */
    int i;
    unsigned short regData;

    //process all connected phys
    for(i=0; i<iPhyCount; i++)
    {
        //read phy register
        iRet = omethPhyRead(hEth, i, PHYCFG_KSZ8051RNL_PHYCFG2, &regData);
        if(iRet != 0) goto Exit;

        //set bit
        regData |= PHYCFG_KSZ8051RNL_PHYCFG2_RMIICLK;

        //write-back manipulated register content
        omethPhyWrite(hEth, i, PHYCFG_KSZ8051RNL_PHYCFG2, regData);
        if(iRet != 0) goto Exit;
    }
    //-------------------------------------------------------------------------

Exit:
    return iRet;
}
