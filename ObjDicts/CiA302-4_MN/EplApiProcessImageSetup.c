/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  source file for setting up the process image

  License:

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.

    3. Neither the name of SYSTEC electronic GmbH nor the names of its
       contributors may be used to endorse or promote products derived
       from this software without prior written permission. For written
       permission, please contact info@systec-electronic.com.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    Severability Clause:

        If a provision of this License is or becomes illegal, invalid or
        unenforceable in any jurisdiction, that shall not affect:
        1. the validity or enforceability in that jurisdiction of any other
           provision of this License; or
        2. the validity or enforceability in other jurisdictions of that or
           any other provision of this License.

  -------------------------------------------------------------------------

                $RCSfile$

                $Author$

                $Revision$  $Date$

                $State$

                Build Environment:
                    GCC V3.4

  -------------------------------------------------------------------------

  Revision History:

  2006/10/10 d.k.:   start of the implementation, version 1.00

****************************************************************************/

#include "Epl.h"


/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          G L O B A L   D E F I N I T I O N S                            */
/*                                                                         */
/*                                                                         */
/***************************************************************************/

//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------

#define EPLAPI_PI_SUBINDEX_COUNT    252

//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// modul globale vars
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------


/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          C L A S S  EplApi                                              */
/*                                                                         */
/*                                                                         */
/***************************************************************************/
//
// Description:
//
//
/***************************************************************************/


//=========================================================================//
//                                                                         //
//          P R I V A T E   D E F I N I T I O N S                          //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// local vars
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------

static tEplKernel EplApiProcessImageLinkRange(
    unsigned int    uiObjIndexStart_p,
    unsigned int    uiObjIndexEnd_p,
    unsigned int    uiOffsetPI_p,
    BOOL            fOutputPI_p,
    tEplObdSize     EntrySize_p,
    unsigned int    uiSubindexCountPerIndex_p);


//=========================================================================//
//                                                                         //
//          P U B L I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//


//---------------------------------------------------------------------------
//
// Function:    EplApiProcessImageSetup()
//
// Description: sets up static process image
//
// Parameters:  (none)
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplApiProcessImageSetup(void)
{
tEplKernel      Ret = kEplSuccessful;

    Ret = EplApiProcessImageLinkRange(0xA000, 0xA00F, 0, FALSE, 1, EPLAPI_PI_SUBINDEX_COUNT);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

    Ret = EplApiProcessImageLinkRange(0xA040, 0xA04F, 0, FALSE, 1, EPLAPI_PI_SUBINDEX_COUNT);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

    Ret = EplApiProcessImageLinkRange(0xA0C0, 0xA0C7, 0, FALSE, 2, EPLAPI_PI_SUBINDEX_COUNT);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

    Ret = EplApiProcessImageLinkRange(0xA100, 0xA107, 0, FALSE, 2, EPLAPI_PI_SUBINDEX_COUNT);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

    Ret = EplApiProcessImageLinkRange(0xA1C0, 0xA1C3, 0, FALSE, 4, EPLAPI_PI_SUBINDEX_COUNT);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

    Ret = EplApiProcessImageLinkRange(0xA200, 0xA203, 0, FALSE, 4, EPLAPI_PI_SUBINDEX_COUNT);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

    Ret = EplApiProcessImageLinkRange(0xA480, 0xA48F, 0, TRUE, 1, EPLAPI_PI_SUBINDEX_COUNT);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

    Ret = EplApiProcessImageLinkRange(0xA4C0, 0xA4CF, 0, TRUE, 1, EPLAPI_PI_SUBINDEX_COUNT);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

    Ret = EplApiProcessImageLinkRange(0xA540, 0xA547, 0, TRUE, 2, EPLAPI_PI_SUBINDEX_COUNT);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

    Ret = EplApiProcessImageLinkRange(0xA580, 0xA587, 0, TRUE, 2, EPLAPI_PI_SUBINDEX_COUNT);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

    Ret = EplApiProcessImageLinkRange(0xA640, 0xA643, 0, TRUE, 4, EPLAPI_PI_SUBINDEX_COUNT);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

    Ret = EplApiProcessImageLinkRange(0xA680, 0xA683, 0, TRUE, 4, EPLAPI_PI_SUBINDEX_COUNT);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

Exit:
    return Ret;
}



//=========================================================================//
//                                                                         //
//          P R I V A T E   F U N C T I O N S                              //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//
// Function:    EplApiProcessImageSetup()
//
// Description: sets up static process image
//
// Parameters:  (none)
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplApiProcessImageLinkRange(
    unsigned int    uiObjIndexStart_p,
    unsigned int    uiObjIndexEnd_p,
    unsigned int    uiOffsetPI_p,
    BOOL            fOutputPI_p,
    tEplObdSize     EntrySize_p,
    unsigned int    uiSubindexCountPerIndex_p)
{
tEplKernel      Ret = kEplSuccessful;
unsigned int    uiVarEntries;

    for (; uiObjIndexStart_p <= uiObjIndexEnd_p;
        uiObjIndexStart_p++, uiOffsetPI_p += EntrySize_p * uiSubindexCountPerIndex_p)
    {
        uiVarEntries = uiSubindexCountPerIndex_p;
        Ret = EplApiProcessImageLinkObject(
						uiObjIndexStart_p,
						1,
                        uiOffsetPI_p,
                        fOutputPI_p,
                        EntrySize_p,
                        &uiVarEntries);
        if (((Ret == kEplSuccessful) && (uiVarEntries < uiSubindexCountPerIndex_p))
            || (Ret == kEplApiPISizeExceeded))
        {
            Ret = kEplSuccessful;
            break;
        }
        if (Ret != kEplSuccessful)
        {
            TRACE("EplApiProcessImageLinkObject returned: %xh for index %xh\n", Ret, uiObjIndexStart_p);
            goto Exit;
        }
    }

Exit:
    return Ret;
}



// EOF

