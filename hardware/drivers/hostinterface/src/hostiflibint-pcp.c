/**
********************************************************************************
\file   hostiflibint-pcp.c

\brief  Host Interface Library - High Level Driver Implementation for PCP

The file contains the high level driver for the host interface library for PCP.

\ingroup module_hostiflib
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, B&R Industrial Automation GmbH
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

/**
********************************************************************************
\defgroup   module_hostiflib    Host Interface Library
\ingroup    libraries

The host interface library provides a software interface for using the host
interface IP-Core. It provides several features like queues and linear memory
modules.
*******************************************************************************/

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include "hostiflib_l.h"
#include "hostiflib.h"
#include "hostiflib_target.h"
#include "hostiflibint.h"

#include <string.h>

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
static tHostifReturn controlBridge(tHostif* pHostif_p, BOOL fEnable_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Create a host interface instance for PCP

This function creates the PCP-specific host interface instance.

\param[in,out]  pHostif_p           The host interface instance for PCP.

\return The function returns a tHostifReturn error code.
\retval kHostifSuccessful           The host interface is configured successfully
                                    with the provided parameters.
\retval kHostifNoResource           Heap allocation was impossible or to many
                                    instances are present.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_createInt(tHostif* pHostif_p)
{
    tHostifReturn   ret = kHostifSuccessful;
    tHostifBufDesc  aInitVec[] = HOSTIF_INIT_VEC;
    UINT8           i;

    // Allocate init parameter memory
    pHostif_p->pInitParam = (tHostifInitParam*)HOSTIF_UNCACHED_MALLOC(sizeof(tHostifInitParam));

    if (pHostif_p->pInitParam == NULL)
    {
        ret = kHostifNoResource;
        goto Exit;
    }

    memset(pHostif_p->pInitParam, 0, sizeof(tHostifInitParam));

    // Initialize init parameter memory
    pHostif_p->pInitParam->initMemLength = HOSTIF_DYNBUF_COUNT + HOSTIF_BUF_COUNT;
    memcpy(pHostif_p->pInitParam->aInitMem, aInitVec, sizeof(aInitVec));

    // Now set init parameter address to hostif
    hostif_writeInitBase(pHostif_p->pBase, (UINT32)pHostif_p->pInitParam);

    // Write span of buffers into buf map table, malloc them and write to hostif
    for (i = 0; i < kHostifInstIdLast; i++)
    {
        pHostif_p->aBufMap[i].span = aInitVec[i + HOSTIF_DYNBUF_COUNT].span;
        pHostif_p->aBufMap[i].pBase = (void*)HOSTIF_UNCACHED_MALLOC(pHostif_p->aBufMap[i].span);

        if (pHostif_p->aBufMap[i].pBase == NULL)
        {
            ret = kHostifNoResource;
            goto Exit;
        }

        hostif_writeBufPcp(pHostif_p->pBase, i, (UINT32)pHostif_p->aBufMap[i].pBase);
    }

    // since everything is fine, activate bridge
    ret = controlBridge(pHostif_p, TRUE);

    if (ret != kHostifSuccessful)
    {
        goto Exit;
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Delete host interface instance for PCP

This function deletes a host interface instance.

\param[in]      pHostif_p           The host interface instance that should be
                                    deleted

\return The function returns a tHostifReturn error code.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_deleteInt(tHostif* pHostif_p)
{
    tHostifReturn   ret = kHostifSuccessful;
    UINT8           i;

    // deactivate bridge (ignore ret)
    ret = controlBridge(pHostif_p, FALSE);

    // Free init parameter
    if (pHostif_p->pInitParam != NULL)
    {
        HOSTIF_UNCACHED_FREE(pHostif_p->pInitParam);
    }

    hostif_writeInitBase(pHostif_p->pBase, (UINT32)NULL);

    // Free buffers
    for (i = 0; i < kHostifInstIdLast; i++)
    {
        if (pHostif_p->aBufMap[i].pBase != NULL)
        {
            HOSTIF_UNCACHED_FREE(pHostif_p->aBufMap[i].pBase);
        }

        hostif_writeBufPcp(pHostif_p->pBase, i, (UINT32)NULL);
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Check version of IP-Core

This function reads and verifies the version from the host interface.

\param[in]      pBase_p             Base address to host interface hardware
\param[in]      pSwVersion_p        Pointer to version provided by sw

\return The function returns a tHostifReturn error code.
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_checkVersion(const void* pBase_p,
                                  const tHostifVersion* pSwVersion_p)
{
    tHostifReturn           ret = kHostifSuccessful;
    UINT32                  versionField = hostif_readVersion(pBase_p);
    const tHostifHwVersion* pHwVersion = (const tHostifHwVersion*)&versionField;

    if (pHwVersion->cnt != HOSTIF_VERSION_COUNT)
        ret = kHostifWrongVersion;

    /* Check Revision, Minor and Major */
    if (pHwVersion->version.revision != pSwVersion_p->revision)
        ret = kHostifWrongVersion;

    if (pHwVersion->version.minor != pSwVersion_p->minor)
        ret = kHostifWrongVersion;

    if (pHwVersion->version.major != pSwVersion_p->major)
        ret = kHostifWrongVersion;

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  This function enables an IRQ source

This function enables an IRQ source from the PCP side.

\param[in]      pInstance_p         Host interface instance
\param[in]      irqSrc_p            IRQ source to be controlled
\param[in]      fEnable_p           Enable the IRQ source (TRUE)

\return The function returns a tHostifReturn error code.
\retval kHostifSuccessful           The process function exit without errors.
\retval kHostifInvalidParameter     The caller has provided incorrect parameters.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_irqSourceEnable(tHostifInstance pInstance_p,
                                     tHostifIrqSrc irqSrc_p,
                                     BOOL fEnable_p)
{
    tHostifReturn ret = kHostifSuccessful;
    tHostif*      pHostif = (tHostif*)pInstance_p;
    UINT16        irqEnableVal;

    if ((pInstance_p == NULL) || (irqSrc_p >= kHostifIrqSrcLast))
    {
        ret = kHostifInvalidParameter;
        goto Exit;
    }

    // get irq source enable from hw
    irqEnableVal = hostif_readIrqEnable(pHostif->pBase);

    if (fEnable_p != FALSE)
        irqEnableVal |= (1 << irqSrc_p);
    else
        irqEnableVal &= ~(1 << irqSrc_p);

    // write irq source enable back to hw
    hostif_writeIrqEnable(pHostif->pBase, irqEnableVal);

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  This function sets a state to the host interface

Note that only the PCP is allowed to write to this register!

\param[in]      pInstance_p         Host interface instance
\param[in]      sta_p               State

\return The function returns a tHostifReturn error code.
\retval kHostifSuccessful           The process function exit without errors.
\retval kHostifInvalidParameter     The caller has provided incorrect parameters.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_setState(tHostifInstance pInstance_p, tHostifState sta_p)
{
    tHostifReturn ret = kHostifSuccessful;
    tHostif*      pHostif = (tHostif*)pInstance_p;

    if (pInstance_p == NULL)
    {
        ret = kHostifInvalidParameter;
        goto Exit;
    }

    hostif_writeState(pHostif->pBase, (UINT16)sta_p);

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  This function sets the heart beat value to the host interface

Note that only the PCP is allowed to write to this register!

\param[in]      pInstance_p         Host interface instance
\param[in]      heartbeat_p         Heart beat value

\return The function returns a tHostifReturn error code.
\retval kHostifSuccessful           The process function exit without errors.
\retval kHostifInvalidParameter     The caller has provided incorrect parameters.
\retval kHostifWrongProcInst        The caller processor instance is not allowed.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_setHeartbeat(tHostifInstance pInstance_p, UINT16 heartbeat_p)
{
    tHostifReturn ret = kHostifSuccessful;
    tHostif*      pHostif = (tHostif*)pInstance_p;

    if (pInstance_p == NULL)
    {
        ret = kHostifInvalidParameter;
        goto Exit;
    }

    hostif_writeHeartbeat(pHostif->pBase, heartbeat_p);

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  This function returns the initialization parameters reference

This function returns the user part of the initialization parameters.

\param[in]      pInstance_p         Host interface instance
\param[out]     ppBufBase_p         Returned buffer base address

\return The function returns a tHostifReturn error code.
\retval kHostifSuccessful           Dynamic buffer freed
\retval kHostifInvalidParameter     The caller has provided incorrect parameters.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_getInitParam(tHostifInstance pInstance_p, void** ppBase_p)
{
    tHostifReturn   ret = kHostifSuccessful;
    tHostif*        pHostif = (tHostif*)pInstance_p;

    if ((pInstance_p == NULL) || (ppBase_p == NULL))
    {
        ret = kHostifInvalidParameter;
        goto Exit;
    }

    *ppBase_p = pHostif->pInitParam->aUser;

Exit:
    return ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Turn on/off the host interface bridge

This function turns on or off the bridge logic from PCP side.
This refuses the host accessing the PCP-memory space in case of an uninitialized
host interface.
The function writes the specific enable pattern to hardware and reads it back
again.

\param[in]      pHostif_p           Host interface instance
\param[in]      fEnable_p           Enable the bridge with TRUE

\return The function returns a tHostifReturn error code.
*/
//------------------------------------------------------------------------------
static tHostifReturn controlBridge(tHostif* pHostif_p, BOOL fEnable_p)
{
    tHostifReturn ret = kHostifSuccessful;
    UINT16        dst = 0;
    UINT16        src;

    if (fEnable_p != FALSE)
    {
        dst = HOSTIF_BRIDGE_ENABLE;
    }

    // set value to hw
    hostif_writeBridgeEnable(pHostif_p->pBase, dst);

    // read back value from hw and check if write was successful
    src = hostif_readBridgeEnable(pHostif_p->pBase);

    if ((src & HOSTIF_BRIDGE_ENABLE) != dst)
    {
        ret = kHostifHwWriteError;
        goto Exit;
    }

Exit:
    return ret;
}
