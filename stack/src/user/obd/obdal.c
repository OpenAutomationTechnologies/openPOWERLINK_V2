/**
********************************************************************************
\file   obdal.c

\brief  Abstraction layer for object dictionary access

This file collects all existing objects and object dictionary parts,
and provides an interface to access them.

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
#include <oplk/obd.h>
#include <oplk/obdal.h>
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

\param  pSdoHdl_p           Connection handle to SDO server
\param  pfnFinishSdoCb_p    Callback for object dictionary to finish a read or
                            write access from SDO

\return The function returns a tOplkError error code.

\ingroup module_obd
*/
//------------------------------------------------------------------------------
tOplkError obdal_processWrite(tSdoObdConHdl* pSdoHdl_p,
                               tCmdLayerObdFinishedCb pfnFinishSdoCb_p)
{
    tOplkError      ret = kErrorOk;

    ret = obd_processWrite(pSdoHdl_p);

    if ((ret == kErrorObdIndexNotExist))
    {   // object not in the default obd, try the virtual obd
        ret = obdvrtl_processWrite(pSdoHdl_p, pfnFinishSdoCb_p);
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process an object read access from SDO server

The function processes an ReadByIndex command layer of an SDO server.

\param  pSdoHdl_p           Connection handle to SDO server
                            returns:
                            - totalPendSize, only for initial transfer: object
                              size
                            - dataSize: size of copied data to provided buffer
\param  pfnFinishSdoCb_p    Callback for object dictionary to finish a read or
                            write access from SDO

\return The function returns a tOplkError error code.

\ingroup module_obd
*/
//------------------------------------------------------------------------------
tOplkError obdal_processRead(tSdoObdConHdl* pSdoHdl_p,
                              tCmdLayerObdFinishedCb pfnFinishSdoCb_p)
{
    tOplkError      ret = kErrorOk;

    ret = obd_processRead(pSdoHdl_p);

    if ((ret == kErrorObdIndexNotExist))
    {   // object not in the default obd, try the virtual obd
        ret = obdvrtl_processRead(pSdoHdl_p, pfnFinishSdoCb_p);
    }

    return ret;
}


//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
