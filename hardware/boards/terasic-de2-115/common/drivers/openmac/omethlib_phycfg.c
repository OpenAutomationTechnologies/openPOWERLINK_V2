/**
********************************************************************************
\file   omethlib_phycfg_ink.c

\brief  openMAC phy configuration for INK DE2-115 evaluation board

This file contains phy configuration callback for the TERASIC INK DE2-115
evaluation board.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holders nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
------------------------------------------------------------------------------*/

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <omethlib.h>
#include <omethlibint.h>
#include <omethlib_phycfg.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define PHYCFG_88E1111_CTRL_REG                 0
#define PHYCFG_88E1111_CTRL_REG_RESET           0x8000

#define PHYCFG_88E1111_SPECCTRL_REG             16
#define PHYCFG_88E1111_SPECCTRL_MDIX_AUTO       0x0060

#define PHYCFG_88E1111_EXTPHYST_REG             27
#define PHYCFG_88E1111_EXTPHYST_REG_HWCFG_MII   0x000F

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Phy configuration callback

This function configures the available 88E1111 phys on the INK DE2-115 board.

\param  pEth_p  Ethernet driver handle

\return The function returns 0 if success.
*/
//------------------------------------------------------------------------------
int omethPhyCfgUser(OMETH_H pEth_p)
{
    int             ret = 0;
    int             i;
    int             phyCount = pEth_p->phyCount;
    unsigned short  regData;
    unsigned short  regNumber;

    //process all connected phys
    for(i=0; i<phyCount; i++)
    {
        // Set MII mode
        regNumber = PHYCFG_88E1111_EXTPHYST_REG;
        // Read extended phy specific status register
        ret = omethPhyRead(pEth_p, i, regNumber, &regData);
        if(ret != 0)
            goto Exit;

        // Set bits for MII mode
        regData |= PHYCFG_88E1111_EXTPHYST_REG_HWCFG_MII;

        // Write-back manipulated register content
        omethPhyWrite(pEth_p, i, regNumber, regData);
        if(ret != 0)
            goto Exit;

        // Enable auto-crossover
        regNumber = PHYCFG_88E1111_SPECCTRL_REG;
        // Read phy specific control register
        ret = omethPhyRead(pEth_p, i, regNumber, &regData);
        if(ret != 0)
            goto Exit;

        // Set bits for auto-crossover
        regData |= PHYCFG_88E1111_SPECCTRL_MDIX_AUTO;

        // Write-back manipulated register content
        omethPhyWrite(pEth_p, i, regNumber, regData);
        if(ret != 0)
            goto Exit;

        // Trigger SW reset (Note: phy link will be lost!)
        regNumber = PHYCFG_88E1111_CTRL_REG;
        // Read control register
        ret = omethPhyRead(pEth_p, i, regNumber, &regData);
        if(ret != 0)
            goto Exit;

        // Set sw reset
        regData |= PHYCFG_88E1111_CTRL_REG_RESET;

        // Write-back manipulated register content
        omethPhyWrite(pEth_p, i, regNumber, regData);
        if(ret != 0)
            goto Exit;
    }

Exit:
    return ret;
}
