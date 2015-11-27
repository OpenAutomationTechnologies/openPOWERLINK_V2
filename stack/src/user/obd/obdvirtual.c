/**
********************************************************************************
\file   obdvirtual.c

\brief  Virtual object dictionary

This file implements the virtual object dictionary, which can be used in
addition to a local directly accessible object dictionary. The virtual
object dictionary is used for objects which do not exist in the default
object dictionary, or for objects which need as special functionality.

\ingroup module_obd
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <user/obdvirtual.h>

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
\brief  Process an object write access from SDO server

The function processes an WriteByIndex command layer of an SDO server.

\param  pSdoHdl_p       Connection handle to SDO server

\return The function returns a tOplkError error code.

\ingroup module_obd
*/
//------------------------------------------------------------------------------
tOplkError obdvrtl_proccessWrite(tSdoObdConHdl* pSdoHdl_p)
{
    tOplkError      ret = kErrorObdIndexNotExist;

    UNUSED_PARAMETER(pSdoHdl_p);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process an object read access from SDO server

The function processes an ReadByIndex command layer of an SDO server.

\param  pSdoHdl_p       Connection handle to SDO server
                        returns:
                         - totalPendSize, only for initial transfer: object size
                         - dataSize: size of copied data to provided buffer

\return The function returns a tOplkError error code.

\ingroup module_obd
*/
//------------------------------------------------------------------------------
tOplkError obdvrtl_proccessRead(tSdoObdConHdl* pSdoHdl_p)
{
    tOplkError      ret = kErrorObdIndexNotExist;

    UNUSED_PARAMETER(pSdoHdl_p);

    return ret;
}


//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
