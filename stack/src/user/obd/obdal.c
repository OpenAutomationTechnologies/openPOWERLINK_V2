/**
********************************************************************************
\file   obdal.c

\brief  Abstraction layer for object dictionary access

This file collects all existing objects and object dictionary parts,
and provides an interface to access them.

\ingroup module_obdal
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, B&R Industrial Automation GmbH
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
#include <user/obdu.h>
#include <user/obdal.h>

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
/**
\brief Object dictionary abstraction layer instance type
*/
typedef struct
{
    tObdAlUserObdAccessCb   pfnCbUserObdAccess;     ///< Callback for user defined object dictionary access
    tCmdLayerObdFinishedCb  pfnCbFinishSdo;         ///< Callback for finishing an SDO access
    BOOL                    fUserObdAccessEnabled;  ///< Flag indicating if accessing user specific OD is enabled
    BOOL                    fInitialized;           ///< Flag to determine status of obdal module
} tObdAlInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tObdAlInstance              instance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError accessUserObdfromSdo(tSdoObdConHdl* pSdoHdl_p,
                                       tObdAlAccessType accessType_p);
static tOplkError finishSdo(tObdAlConHdl* pUserObdConHdl_p);


//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//
//------------------------------------------------------------------------------
/**
\brief  Initialize object dictionary abstraction layer module

The function initializes the object dictionary abstraction layer module.

\param  pfnUserObdAccess_p      Callback function for user defined OD access

\return The function returns a tOplkError error code.

\ingroup module_obdal
**/
//------------------------------------------------------------------------------
tOplkError obdal_init(tObdAlUserObdAccessCb pfnUserObdAccess_p)
{
    tOplkError  ret = kErrorOk;

    if (pfnUserObdAccess_p == NULL)
    {
        ret = kErrorInvalidInstanceParam;
        return ret;
    }

    OPLK_MEMSET(&instance_l, 0, sizeof(instance_l));
    instance_l.pfnCbUserObdAccess = pfnUserObdAccess_p;
    instance_l.fUserObdAccessEnabled = FALSE;
    instance_l.fInitialized = TRUE;

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Clean up object dictionary abstraction layer module

The function cleans up the object dictionary abstraction layer module.

\return The function returns a tOplkError error code.

\ingroup module_obdal
**/
//------------------------------------------------------------------------------
tOplkError obdal_exit(void)
{
    tOplkError      ret = kErrorOk;

    if (instance_l.fInitialized)
    {
        instance_l.pfnCbUserObdAccess = NULL;
        instance_l.pfnCbFinishSdo = NULL;
        instance_l.fUserObdAccessEnabled = FALSE;
        instance_l.fInitialized = FALSE;
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief Enables or disables forwarding object accesses to non-existing objects

This function enables or disables forwarding of object accesses to objects which
do not exist in the default object dictionary. Those accesses are forwarded
to the API, if the feature is activated, and the API needs to handle those
accesses appropriately.

\param fEnable_p    Flag for object access forwarding feature enabling
                    TRUE: enable, FALSE: disable

\return The function returns a tOplkError error code.

\ingroup module_obdal
*/
//------------------------------------------------------------------------------
tOplkError obdal_enableUserObdAccess(BOOL fEnable_p)
{
    if (!instance_l.fInitialized)
    {
        // not initialized
        return kErrorNoResource;
    }

    instance_l.fUserObdAccessEnabled = fEnable_p;
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Process an object write access from SDO server

The function processes an WriteByIndex command layer of an SDO server.

\param  pSdoHdl_p           Connection handle to SDO server.
\param  pfnFinishSdoCb_p    Callback for object dictionary to finish
                            a read or write access from SDO

\return The function returns a tOplkError error code.

\ingroup module_obd
*/
//------------------------------------------------------------------------------
tOplkError obdal_processSdoWrite(tSdoObdConHdl* pSdoHdl_p,
                                 tCmdLayerObdFinishedCb pfnFinishSdoCb_p)
{
    tOplkError      ret = kErrorOk;

    if (pSdoHdl_p == NULL)
    {
        ret = kErrorApiInvalidParam;
        goto Exit;
    }

    ret = obdu_processWrite(pSdoHdl_p);

    if (ret == kErrorObdIndexNotExist)
    {   // object not in the default object dictionary,
        // try the user specific object dictionary
        if (pfnFinishSdoCb_p != NULL)
        {
            if (instance_l.pfnCbFinishSdo != NULL)
            {   // only one connection supported by this module
                ret = kErrorApiSdoBusyIntern;
                goto Exit;
            }

            ret = accessUserObdfromSdo(pSdoHdl_p, kObdAlAccessTypeWrite);
            if (ret == kErrorReject)
            {   // save callback for delayed answer
                instance_l.pfnCbFinishSdo = pfnFinishSdoCb_p;
            }
        }
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process an object read access from SDO server

The function processes a ReadByIndex command layer of an SDO server.

\param      pSdoHdl_p       Connection handle to SDO server. Used members:
            \li [out] \ref  tSdoObdConHdl::totalPendSize
                            Object size, only for initial transfer
            \li [out] \ref  tSdoObdConHdl::dataSize
                            Size of copied data to provided buffer
            \li [in] all other members of \ref tSdoObdConHdl

\param[in]  pfnFinishSdoCb_p    Callback for object dictionary to finish
                                a read or write access from SDO

\return The function returns a tOplkError error code.

\ingroup module_obd
*/
//------------------------------------------------------------------------------
tOplkError obdal_processSdoRead(tSdoObdConHdl* pSdoHdl_p,
                                tCmdLayerObdFinishedCb pfnFinishSdoCb_p)
{
    tOplkError      ret = kErrorOk;

    if (pSdoHdl_p == NULL)
    {
        ret = kErrorApiInvalidParam;
        goto Exit;
    }

    ret = obdu_processRead(pSdoHdl_p);

    if (ret == kErrorObdIndexNotExist)
    {   // object not in the default object dictionary,
        // try the user specific object dictionary
        if (pfnFinishSdoCb_p != NULL)
        {
            if (instance_l.pfnCbFinishSdo != NULL)
            {   // only one connection supported by this module
                ret = kErrorApiSdoBusyIntern;
                goto Exit;
            }

            ret = accessUserObdfromSdo(pSdoHdl_p, kObdAlAccessTypeRead);
            if (ret == kErrorReject)
            {   // save callback for delayed answer
                instance_l.pfnCbFinishSdo = pfnFinishSdoCb_p;
            }
        }
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Finish a user specific object access

The function finishes a user specific object access, which returned
kErrorReject at the beginning of the access to signal a delayed answer.

\param  pUserObdConHdl_p  Connection handle from user OD

\return The function returns a tOplkError error code.

\ingroup module_obd
*/
//------------------------------------------------------------------------------
tOplkError obdal_finishUserObdAccess(tObdAlConHdl* pUserObdConHdl_p)
{
    tOplkError      ret = kErrorOk;

    if (pUserObdConHdl_p == NULL)
    {
        ret = kErrorApiInvalidParam;
        goto Exit;
    }

    switch (pUserObdConHdl_p->origin)
    {
        case kObdAlOriginSdo:
            ret = finishSdo(pUserObdConHdl_p);
            break;

        default:
            // unsupported origin
            ret = kErrorInvalidInstanceParam;
            break;
    }

Exit:
    return ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Access the user specific object dictionary from SDO

The function processes an SDO command layer access of an SDO server to
non-existing objects in the default object dictionary. Those accesses are
forwarded to the user specific object dictionary.

\param  pSdoHdl_p       Connection handle to SDO server
\param  accessType_p    Object access type

\return The function returns a tOplkError error code.

*/
//------------------------------------------------------------------------------
static tOplkError accessUserObdfromSdo(tSdoObdConHdl* pSdoHdl_p,
                                       tObdAlAccessType accessType_p)
{
    tOplkError      ret = kErrorOk;
    tObdAlConHdl    userObdConHdl;

    if ((instance_l.pfnCbUserObdAccess != NULL) &&
        instance_l.fUserObdAccessEnabled)
    {
        userObdConHdl.index = pSdoHdl_p->index;
        userObdConHdl.subIndex = pSdoHdl_p->subIndex;
        userObdConHdl.pSrcData = pSdoHdl_p->pSrcData;
        userObdConHdl.pDstData = pSdoHdl_p->pDstData;
        userObdConHdl.totalPendSize = pSdoHdl_p->totalPendSize;
        userObdConHdl.dataSize = pSdoHdl_p->dataSize;
        userObdConHdl.dataOffset = pSdoHdl_p->dataOffset;
        userObdConHdl.obdAlHdl = pSdoHdl_p->sdoHdl;
        userObdConHdl.plkError = pSdoHdl_p->plkError;
        userObdConHdl.origin = kObdAlOriginSdo;
        userObdConHdl.accessTyp = accessType_p;
        ret = instance_l.pfnCbUserObdAccess(&userObdConHdl);
        // assign returned members (only used by read access)
        if (accessType_p == kObdAlAccessTypeRead)
        {
            if (pSdoHdl_p->dataOffset == 0)
            {
                pSdoHdl_p->totalPendSize = userObdConHdl.totalPendSize;
            }
            pSdoHdl_p->dataSize = userObdConHdl.dataSize;
        }
    }
    else
    {   // No access to user OD means object does not exist
        ret = kErrorObdIndexNotExist;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Finish a user specific object access originated by SDO

The function finishes a user specific object access, originated by SDO,
which returned kErrorReject on the beginning of the access to signal a
delayed answer.

\param  pUserObdConHdl_p  Connection handle from user OD

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError finishSdo(tObdAlConHdl* pUserObdConHdl_p)
{
    tOplkError              ret = kErrorOk;
    tSdoObdConHdl           sdoHdl;
    tCmdLayerObdFinishedCb  pfnCbFinishSdo = NULL;

    if (instance_l.pfnCbFinishSdo != NULL)
    {
        sdoHdl.index = pUserObdConHdl_p->index;
        sdoHdl.subIndex = pUserObdConHdl_p->subIndex;
        sdoHdl.pSrcData = pUserObdConHdl_p->pSrcData;
        sdoHdl.pDstData = pUserObdConHdl_p->pDstData;
        sdoHdl.totalPendSize = pUserObdConHdl_p->totalPendSize;
        sdoHdl.dataSize = pUserObdConHdl_p->dataSize;
        sdoHdl.dataOffset = pUserObdConHdl_p->dataOffset;
        sdoHdl.sdoHdl = pUserObdConHdl_p->obdAlHdl;
        sdoHdl.plkError = pUserObdConHdl_p->plkError;
        // save and invalidate callback
        pfnCbFinishSdo =  instance_l.pfnCbFinishSdo;
        instance_l.pfnCbFinishSdo = NULL;

        ret = pfnCbFinishSdo(&sdoHdl);
    }
    else
    {
        ret = kErrorInvalidOperation;
    }

    return ret;
}

/// \}
