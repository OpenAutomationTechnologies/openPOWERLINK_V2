/**
********************************************************************************
\file   hostiflibint-host.c

\brief  Host Interface Library - High Level Driver Implementation for Host

The file contains the high level driver for the host interface library for Host.

\ingroup module_hostiflib
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

#ifndef HOSTIF_BRIDGE_INIT_TIMEOUT_MS
#define HOSTIF_BRIDGE_INIT_TIMEOUT_MS 10000     // 10 seconds
#endif
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
static void hostifIrqHandler(void* pArg_p);
static tHostifReturn controlIrqMaster(tHostif* pHostif_p, BOOL fEnable_p);
HOSTIF_INLINE static BOOL getBridgeEnabled(tHostif* pHostif_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Create a host interface instance for Host

This function creates the Pcp-specific host interface instance.

\param  pHostif_p               The host interface instance for PCP.

\return The function returns a tHostifReturn error code.
\retval kHostifSuccessful       The host interface is configured successfully
                                with the provided parameters.
\retval kHostifNoResource       Heap allocation was impossible or to many
                                instances are present.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_createInt(tHostif* pHostif_p)
{
    tHostifReturn       ret = kHostifSuccessful;
    UINT32              pcpAddr;
    tHostifInitParam*   pInitParam;
    UINT                loopCnt;

    // Busy wait for enabled bridge
    for (loopCnt = HOSTIF_BRIDGE_INIT_TIMEOUT_MS; loopCnt > 0; loopCnt--)
    {
        if (getBridgeEnabled(pHostif_p) == TRUE)
        {
            break;
        }

        HOSTIF_USLEEP(1000);
    }

    if (loopCnt == 0)
    {
        // Timeout waiting for bridge enable
        ret = kHostifBridgeDisabled;
        goto Exit;
    }

    // Get init param address in pcp memory space and write it to dyn buf 0
    pcpAddr = hostif_readInitBase(pHostif_p->pBase);
    hostif_writeDynBufHost(pHostif_p->pBase, 0, pcpAddr);

    // Point to address after status control registers (=dyn buf 0)
    pInitParam = (tHostifInitParam*)(pHostif_p->pBase + HOSTIF_STCTRL_SPAN);

    // Check if mem length is correct, otherwise version mismatch!
    if (pInitParam->initMemLength != HOSTIF_DYNBUF_COUNT + HOSTIF_BUF_COUNT)
    {
        ret = kHostifWrongVersion;
        goto Exit;
    }

    // And now, get the stuff
    for (loopCnt = 0; loopCnt < pInitParam->initMemLength; loopCnt++)
    {
        pHostif_p->aBufMap[loopCnt].pBase = pHostif_p->pBase + pInitParam->aInitMem[loopCnt].offset;
        pHostif_p->aBufMap[loopCnt].span = pInitParam->aInitMem[loopCnt].span;
    }

    // register isr in system
    if ((ret = hostif_sysIrqRegHandler(hostifIrqHandler, (void*)pHostif_p)) != kHostifSuccessful)
    {
        goto Exit;
    }

    // enable system irq
    ret = hostif_sysIrqEnable(TRUE);

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Delete host interface instance for Host

This function deletes a host interface instance.

\param  pHostif_p               The host interface instance that should be
                                deleted

\return The function returns a tHostifReturn error code.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_deleteInt(tHostif* pHostif_p)
{
    UNUSED_PARAMETER(pHostif_p);

    // enable system IRQ (ignore ret)
    hostif_sysIrqEnable(FALSE);

    // unregister ISR in system (ignore ret)
    hostif_sysIrqRegHandler(NULL, NULL);

    return kHostifSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Check version of ipcore

This function reads and verifies the version from the host interface.

\param  pBase_p         Base address to host interface hardware
\param  pSwVersion_p    Pointer to version provided by sw

\return The function returns a tHostifReturn error code.
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_checkVersion(UINT8* pBase_p, tHostifVersion* pSwVersion_p)
{
    tHostifReturn       ret = kHostifSuccessful;
    UINT32              versionField = hostif_readVersion(pBase_p);
    tHostifHwVersion*   pHwVersion = (tHostifHwVersion*)&versionField;

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
\brief  This function adds an IRQ handler for the corresponding IRQ source

This function adds an IRQ handler function for the corresponding IRQ source.
Note: The provided callback is invoked within the interrupt context!
If the provided callback is NULL, then the IRQ source is disabled.

\param  pInstance_p             Host interface instance
\param  irqSrc_p                IRQ source that should invoke the callback
\param  pfnCb_p                 Callback function that is invoked

\return The function returns a tHostifReturn error code.
\retval kHostifSuccessful       The process function exit without errors.
\retval kHostifInvalidParameter The caller has provided incorrect parameters.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_irqRegHdl(tHostifInstance pInstance_p,
                               tHostifIrqSrc irqSrc_p, tHostifIrqCb pfnCb_p)
{
    tHostifReturn ret = kHostifSuccessful;
    tHostif*      pHostif = (tHostif*)pInstance_p;
    UINT16        irqEnableVal;

    if (pInstance_p == NULL || irqSrc_p >= kHostifIrqSrcLast)
    {
        ret = kHostifInvalidParameter;
        goto Exit;
    }

    // get IRQ source enable from hw
    irqEnableVal = hostif_readIrqEnable(pHostif->pBase);

    // enable IRQ source if callback is not NULL
    if (pfnCb_p != NULL)
        irqEnableVal |= (1 << irqSrc_p);
    else
        irqEnableVal &= ~(1 << irqSrc_p);

    // store callback in instance
    pHostif->apfnIrqCb[irqSrc_p] = pfnCb_p;

    // write IRQ source enable back to hw
    hostif_writeIrqEnable(pHostif->pBase, irqEnableVal);

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  This function controls the master IRQ enable

This function allows the host to enable or disable all IRQ sources from the
host interface.

\param  pInstance_p             host interface instance
\param  fEnable_p               enable the master IRQ (TRUE)

\return The function returns a tHostifReturn error code.
\retval kHostifSuccessful       The process function exit without errors.
\retval kHostifInvalidParameter The caller has provided incorrect parameters.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_irqMasterEnable(tHostifInstance pInstance_p,
                                     BOOL fEnable_p)
{
    tHostifReturn ret = kHostifSuccessful;
    tHostif*      pHostif = pInstance_p;

    if (pInstance_p == NULL)
    {
        ret = kHostifInvalidParameter;
        goto Exit;
    }

    // activte master irq enable
    ret = controlIrqMaster(pHostif, fEnable_p);

    if (ret != kHostifSuccessful)
    {
        goto Exit;
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  This function gets the state from the host interface

\param  pInstance_p             host interface instance
\param  pSta_p                  state

\return The function returns a tHostifReturn error code.
\retval kHostifSuccessful       The process function exit without errors.
\retval kHostifInvalidParameter The caller has provided incorrect parameters.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_getState(tHostifInstance pInstance_p, tHostifState* pSta_p)
{
    tHostifReturn ret = kHostifSuccessful;
    tHostif*      pHostif = (tHostif*)pInstance_p;

    if (pInstance_p == NULL || pSta_p == NULL)
    {
        ret = kHostifInvalidParameter;
        goto Exit;
    }

    *pSta_p = hostif_readState(pHostif->pBase);

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  This function gets the error/return from the host interface

\param  pInstance_p             host interface instance
\param  pErr_p                  error/return

\return The function returns a tHostifReturn error code.
\retval kHostifSuccessful       The process function exit without errors.
\retval kHostifInvalidParameter The caller has provided incorrect parameters.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_getError(tHostifInstance pInstance_p, tHostifError* pErr_p)
{
    tHostifReturn ret = kHostifSuccessful;
    tHostif*      pHostif = (tHostif*)pInstance_p;

    if (pInstance_p == NULL || pErr_p == NULL)
    {
        ret = kHostifInvalidParameter;
        goto Exit;
    }

    *pErr_p = hostif_readReturn(pHostif->pBase);

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  This function gets the heart beat value from the host interface

\param  pInstance_p             Host interface instance
\param  pHeartbeat_p            Heart beat value

\return The function returns a tHostifReturn error code.
\retval kHostifSuccessful       The process function exit without errors.
\retval kHostifInvalidParameter The caller has provided incorrect parameters.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_getHeartbeat(tHostifInstance pInstance_p, UINT16* pHeartbeat_p)
{
    tHostifReturn ret = kHostifSuccessful;
    tHostif*      pHostif = (tHostif*)pInstance_p;

    if (pInstance_p == NULL || pHeartbeat_p == NULL)
    {
        ret = kHostifInvalidParameter;
        goto Exit;
    }

    *pHeartbeat_p = hostif_readHeartbeat(pHostif->pBase);

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  This function acquires a dynamic buffer for the host

\param  pInstance_p             Host interface instance
\param  pcpBaseAddr_p           Address in pcp memory space
\param  ppBufBase_p             Address to acquired memory

\return The function returns a tHostifReturn error code.
\retval kHostifSuccessful       Dynamic buffer acquired ppDynBufBase_p valid.
\retval kHostifInvalidParameter The caller has provided incorrect parameters.
\retval kHostifBridgeDisabled   The bridge is disabled.
\retval kHostifNoResource       No dynamic buffer is available

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_dynBufAcquire(tHostifInstance pInstance_p, UINT32 pcpBaseAddr_p,
                                   UINT8** ppBufBase_p)
{
    tHostifReturn ret;
    tHostif*      pHostif = (tHostif*)pInstance_p;
    UINT          i;

    if (pInstance_p == NULL || ppBufBase_p == NULL)
    {
        ret = kHostifInvalidParameter;
        goto Exit;
    }

    if (getBridgeEnabled(pHostif) == FALSE)
    {
        ret = kHostifBridgeDisabled;
        goto Exit;
    }

    ret = kHostifNoResource;

    for (i = 0; i < HOSTIF_DYNBUF_COUNT; i++)
    {
        if (pHostif->apDynBuf[i] == NULL)
        {
            // handle base address in pcp memory space
            pHostif->apDynBuf[i] = (UINT8*)pcpBaseAddr_p;

            hostif_writeDynBufHost(pHostif->pBase, (UINT8)i, pcpBaseAddr_p);

            // Get dynamic buffer address
            *ppBufBase_p = pHostif->aBufMap[i].pBase;

            ret = kHostifSuccessful;
            break;
        }
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  This function frees a dynamic buffer acquired by the host

\param  pInstance_p             Host interface instance
\param  pBufBase_p              Address to acquired memory being freed

\return The function returns a tHostifReturn error code.
\retval kHostifSuccessful       Dynamic buffer freed
\retval kHostifInvalidParameter The caller has provided incorrect parameters.
\retval kHostifNoResource       No dynamic buffer is available to be freed

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_dynBufFree(tHostifInstance pInstance_p, UINT8* pBufBase_p)
{
    tHostifReturn ret = kHostifSuccessful;
    tHostif*      pHostif = (tHostif*)pInstance_p;
    UINT          i;

    if (pInstance_p == NULL)
    {
        ret = kHostifInvalidParameter;
        goto Exit;
    }

    for (i = 0; i < HOSTIF_DYNBUF_COUNT; i++)
    {
        if (pHostif->aBufMap[i].pBase == pBufBase_p)
        {
            // Found dynamic buffer, free it
            pHostif->apDynBuf[i] = NULL;

            ret = kHostifSuccessful;
            break;
        }
    }

    if (ret == kHostifSuccessful)
        hostif_writeDynBufHost(pHostif->pBase, (UINT8)i, 0);

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  This function returns the initialization parameters reference

This function returns the user part of the initialization parameters.

\param  pInstance_p             Host interface instance
\param  ppBufBase_p             Returned buffer base address

\return The function returns a tHostifReturn error code.
\retval kHostifSuccessful       Dynamic buffer freed
\retval kHostifInvalidParameter The caller has provided incorrect parameters.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_getInitParam(tHostifInstance pInstance_p, UINT8** ppBase_p)
{
    tHostifReturn   ret = kHostifSuccessful;
    tHostif*        pHostif = (tHostif*)pInstance_p;

    if (pInstance_p == NULL || ppBase_p == NULL)
    {
        ret = kHostifInvalidParameter;
        goto Exit;
    }

    *ppBase_p = (UINT8*)(((tHostifInitParam*)hostif_readInitBase(pHostif->pBase))->aUser);

Exit:
    return ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Host Interrupt Handler

This is the host interrupt handler which should by called by the system if the
irq signal is asserted by the ipcore. This handler acknowledges the processed
interrupt sources and calls the corresponding callbacks registered with
hostif_irqRegHdl().

\param  pArg_p                  The system caller should provide the host
                                interface instance with this parameter.
*/
//------------------------------------------------------------------------------
static void hostifIrqHandler(void* pArg_p)
{
    tHostif* pHostif = (tHostif*)pArg_p;
    UINT16   pendings;
    UINT16   mask;
    int      i;

    if (pArg_p == NULL)
    {
        goto Exit;
    }

    pendings = hostif_readIrqPending(pHostif->pBase);

    for (i = 0; i < kHostifIrqSrcLast; i++)
    {
        mask = 1 << i;

        //ack IRQ source first
        if (pendings & mask)
            hostif_ackIrq(pHostif->pBase, mask);

        //then try to execute the callback
        if (pHostif->apfnIrqCb[i] != NULL)
            pHostif->apfnIrqCb[i](pArg_p);
    }

Exit:
    return;
}

//------------------------------------------------------------------------------
/**
\brief  Turn on/off the host interface interrupt master

This function turns on or off the interrupt master from host side.
The function writes the specific enable pattern to hardware and reads it back
again.

\param  pHostif_p               Host interface instance
\param  fEnable_p               Enable interrupt master with TRUE

\return The function returns a tHostifReturn error code.
*/
//------------------------------------------------------------------------------
static tHostifReturn controlIrqMaster(tHostif* pHostif_p, BOOL fEnable_p)
{
    tHostifReturn ret = kHostifSuccessful;
    UINT16        dst = 0;
    UINT16        src;

    if (fEnable_p != FALSE)
    {
        dst = HOSTIF_IRQ_MASTER_ENABLE;
    }

    // set value to hw
    hostif_writeIrqMasterEnable(pHostif_p->pBase, dst);

    // read back value from hw and check if write was successful
    src = hostif_readIrqMasterEnable(pHostif_p->pBase);

    if ((src & HOSTIF_IRQ_MASTER_ENABLE) != dst)
    {
        ret = kHostifHwWriteError;
        goto Exit;
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Get bridge turned on/off

This getter returns whether the bridge is turned on or off.

\param  pHostif_p               Host interface instance

\return The function returns TRUE if the bridge is turned on, otherwise FALSE.
*/
//------------------------------------------------------------------------------
static BOOL getBridgeEnabled(tHostif* pHostif_p)
{
    UINT16 val;

    val = hostif_readBridgeEnable(pHostif_p->pBase);

    if (val & HOSTIF_BRIDGE_ENABLE)
        return TRUE;
    else
        return FALSE;
}

