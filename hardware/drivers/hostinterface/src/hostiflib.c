/**
********************************************************************************
\file   hostiflib.c

\brief  Host Interface Library - High Level Driver Implementation

The file contains the high level driver for the host interface library.

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

#include <stdlib.h>
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
/**
\brief Instance array

This array holds all Host Interface instances available.
*/
static tHostif* paHostifInstance_l[HOSTIF_INSTANCE_COUNT] =
{
    NULL
};

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
/* Local functions for PCP and Host */
static tHostifReturn checkMagic(const void* pBase_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Create a host interface instance

This function creates a host interface instance, and initializes it depending
on the pConfig_p parameters.

\param[in]      pConfig_p           The caller provides the configuration
                                    parameters with this pointer.
\param[out]     ppInstance_p        The function returns with this double-pointer
                                    the created instance pointer. (return)

\return The function returns a tHostifReturn error code.
\retval kHostifSuccessful           The host interface is configured successfully
                                    with the provided parameters.
\retval kHostifInvalidParameter     The caller has provided incorrect parameters.
\retval kHostifNoResource           Heap allocation was impossible or to many
                                    instances are present.
\retval kHostifWrongMagic           Can't find a valid host interface (invalid
                                    magic).
\retval kHostifWrongVersion         The version fields in hardware mismatches those
                                    in software.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_create(const tHostifConfig* pConfig_p,
                            tHostifInstance* ppInstance_p)
{
    tHostifReturn   ret = kHostifSuccessful;
    tHostif*        pHostif = NULL;
    void*           pBase_p;
    int             i;

    if ((pConfig_p == NULL) || (ppInstance_p == NULL))
    {
        ret = kHostifInvalidParameter;
        goto Exit;
    }

    pBase_p = HOSTIF_MAKE_NONCACHEABLE(pConfig_p->pBase);

    // check magic
    ret = checkMagic(pBase_p);
    if (ret != kHostifSuccessful)
    {
        goto Exit;
    }

    // check version
    ret = hostif_checkVersion(pBase_p, &pConfig_p->version);
    if (ret != kHostifSuccessful)
    {
        goto Exit;
    }

    // create instance
    pHostif = (tHostif*)malloc(sizeof(tHostif));
    if (pHostif == NULL)
    {
        ret = kHostifNoResource;
        goto Exit;
    }

    memset(pHostif, 0, sizeof(tHostif));

    // initialize instance
    pHostif->config = *pConfig_p;

    // store hostif base
    pHostif->pBase = HOSTIF_MAKE_NONCACHEABLE(pHostif->config.pBase);

    // store instance in array
    for (i = 0; i < HOSTIF_INSTANCE_COUNT; i++)
    {
        if (paHostifInstance_l[i] == NULL)
        {
            // free entry found
            paHostifInstance_l[i] = pHostif;
            break;
        }
    }

    if (i == HOSTIF_INSTANCE_COUNT)
    {
        ret = kHostifNoResource;
        goto Exit;
    }

    if ((ret = hostif_createInt(pHostif)) != kHostifSuccessful)
    {
        goto Exit;
    }

    // return instance pointer
    *ppInstance_p = pHostif;

Exit:
    if (ret != kHostifSuccessful)
    {
        hostif_delete(pHostif);
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Delete host interface instance

This function deletes a host interface instance.

\param[in]      pInstance_p         The host interface instance that should be
                                    deleted

\return The function returns a tHostifReturn error code.
\retval kHostifSuccessful           The host interface is deleted successfully.
\retval kHostifInvalidParameter     The caller has provided incorrect parameters.
\retval kHostifHwWriteError         Deactivation of hardware is faulty.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_delete(tHostifInstance pInstance_p)
{
    tHostifReturn ret = kHostifSuccessful;
    tHostif*      pHostif = (tHostif*)pInstance_p;
    int           i;

    if (pInstance_p == NULL)
    {
        ret = kHostifInvalidParameter;
        goto Exit;
    }

    ret = hostif_deleteInt(pHostif);
    if (ret != kHostifSuccessful)
        goto Exit;

    // delete instance in instance array
    for (i = 0; i < HOSTIF_INSTANCE_COUNT; i++)
    {
        if (pHostif == paHostifInstance_l[i])
        {
            paHostifInstance_l[i] = NULL;
            break;
        }
    }

    free(pHostif);

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Returns the instance of the given processor instance

If the instance is not found NULL is returned.

\param[in]      instance_p          Processor instance

\return The function returns an host interface instance.
\retval NULL                        Host interface instance not found

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifInstance hostif_getInstance(UINT instance_p)
{
    tHostifInstance pHostif = NULL;
    int             i;

    // search through array and return the matching one
    for (i = 0; i < HOSTIF_INSTANCE_COUNT; i++)
    {
        if (paHostifInstance_l[i]->config.instanceNum == instance_p)
        {
            pHostif = (tHostifInstance)paHostifInstance_l[i];
            break;
        }
    }

    return pHostif;
}

//------------------------------------------------------------------------------
/**
\brief  This function sets a command to the host interface

\param[in]      pInstance_p         Host interface instance
\param[in]      cmd_p               Command

\return The function returns a tHostifReturn error code.
\retval kHostifSuccessful           The process function exit without errors.
\retval kHostifInvalidParameter     The caller has provided incorrect parameters.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_setCommand(tHostifInstance pInstance_p, tHostifCommand cmd_p)
{
    tHostifReturn ret = kHostifSuccessful;
    tHostif*      pHostif = (tHostif*)pInstance_p;

    if (pInstance_p == NULL)
    {
        ret = kHostifInvalidParameter;
        goto Exit;
    }

    hostif_writeCommand(pHostif->pBase, (UINT16)cmd_p);

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  This function gets a command from the host interface

\param[in]      pInstance_p         Host interface instance
\param[out]     pCmd_p              Command

\return The function returns a tHostifReturn error code.
\retval kHostifSuccessful           The process function exit without errors.
\retval kHostifInvalidParameter     The caller has provided incorrect parameters.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_getCommand(tHostifInstance pInstance_p, tHostifCommand* pCmd_p)
{
    tHostifReturn  ret = kHostifSuccessful;
    const tHostif* pHostif = (const tHostif*)pInstance_p;

    if ((pInstance_p == NULL) || (pCmd_p == NULL))
    {
        ret = kHostifInvalidParameter;
        goto Exit;
    }

    *pCmd_p = (tHostifCommand)hostif_readCommand(pHostif->pBase);

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  This function sets an error/return to the host interface

Note that only the PCP is allowed to write to this register!

\param[in]      pInstance_p         Host interface instance
\param[in]      err_p               Error/return code

\return The function returns a tHostifReturn error code.
\retval kHostifSuccessful           The process function exit without errors.
\retval kHostifInvalidParameter     The caller has provided incorrect parameters.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_setError(tHostifInstance pInstance_p, tHostifError err_p)
{
    tHostifReturn ret = kHostifSuccessful;
    tHostif*      pHostif = (tHostif*)pInstance_p;

    if (pInstance_p == NULL)
    {
        ret = kHostifInvalidParameter;
        goto Exit;
    }

    hostif_writeReturn(pHostif->pBase, (UINT16)err_p);

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  This function returns the instance buffer

This function gets the buffer base and size of the addressed instance.

\param[in]      pInstance_p         Host interface instance
\param[in]      instId_p            Addressed instance
\param[out]     ppBufBase_p         Returned buffer base address
\param[out]     pBufSize_p          Returned buffer size

\return The function returns a tHostifReturn error code.
\retval kHostifSuccessful           Dynamic buffer freed
\retval kHostifInvalidParameter     The caller has provided incorrect parameters.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_getBuf(tHostifInstance pInstance_p,
                            tHostifInstanceId instId_p,
                            void** ppBufBase_p,
                            size_t* pBufSize_p)
{
    tHostifReturn  ret = kHostifSuccessful;
    const tHostif* pHostif = (const tHostif*)pInstance_p;

    if ((pInstance_p == NULL) ||
        (ppBufBase_p == NULL) ||
        (pBufSize_p == NULL) ||
        !(instId_p < kHostifInstIdLast))
    {
        ret = kHostifInvalidParameter;
        goto Exit;
    }

    *ppBufBase_p = pHostif->aBufMap[instId_p].pBase;
    *pBufSize_p = (size_t)pHostif->aBufMap[instId_p].span;

Exit:
    return ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Check magic word of IP-Core

This function reads and verifies the magic word from the host interface.

\param[in]      pBase_p             Base address to host interface hardware

\return The function returns a tHostifReturn error code.
*/
//------------------------------------------------------------------------------
static tHostifReturn checkMagic(const void* pBase_p)
{
    if (hostif_readMagic(pBase_p) == HOSTIF_MAGIC)
        return kHostifSuccessful;
    else
        return kHostifWrongMagic;
}
