/**
********************************************************************************
\file   sdotest.c

\brief  SDO test api

This file contains the implementation of the SDO test functions.

\ingroup module_sdotest
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, B&R Industrial Automation GmbH
Copyright (c) 2013, SYSTEC electronic GmbH
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
#include <common/oplkinc.h>
#include <oplk/oplk.h>
#include <oplk/event.h>
#include <user/eventu.h>

#include <oplk/sdo.h>
#include <user/sdotest.h>

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
static tOplkApiInitParam    initParam_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError oplkCbSdoTestCom(const tAsySdoCom* asySdoCom_p, size_t dataSize_p);
static tOplkError oplkCbSdoTestSeq(const tAsySdoSeq* asySdoSeq_p, size_t dataSize_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//
/// \name Functions for testing the SDO stack
/// These functions are only needed for testing the SDO stack. They shouldn't be used for
/// a project implementation.
/// \{

//------------------------------------------------------------------------------
/**
\brief  Stores the init parameters

The function stores the init paramter struct for further use.

\param[in]      pInitParam_p        Pointer to the init parameters. The init
                                    parameters must be set by the application.

\ingroup module_sdotest
*/
//------------------------------------------------------------------------------
void oplk_testSdoSetVal(const tOplkApiInitParam* pInitParam_p)
{
    /* reset init parameter*/
    OPLK_MEMSET(&initParam_l, 0, sizeof(tOplkApiInitParam));
    OPLK_MEMCPY(&initParam_l,
                pInitParam_p,
                min(sizeof(tOplkApiInitParam), (size_t)pInitParam_p->sizeOfInitParam));
}

//------------------------------------------------------------------------------
/**
\brief  Initialize the SDO test command layer

The function initializes the SDO test command layer stack.

\return The function returns a \ref tOplkError error code.
\retval kErrorOk                    Stack was successfully initialized.
\retval other                       Error occurred while initializing stack.

\ingroup module_sdotest
*/
//------------------------------------------------------------------------------
tOplkError oplk_testSdoComInit(void)
{
    tOplkError  ret;

    ret = sdotestcom_init(oplkCbSdoTestCom);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Initialize the SDO test sequence layer

The function initializes the SDO test sequence layer stack.

\return The function returns a \ref tOplkError error code.
\retval kErrorOk                    Stack was successfully initialized.
\retval Other                       Error occurred while initializing stack.

\ingroup module_sdotest
*/
//------------------------------------------------------------------------------
tOplkError oplk_testSdoSeqInit(void)
{
    tOplkError  ret;

    ret = sdotestseq_init(oplkCbSdoTestSeq);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  SDO command layer test send frame

The function forwards the Send request to the SDO command testing layer.

\param[in]      targetNodeId_p      Node ID of target node
\param[in]      sdoType_p           Type of SDO lower layer (ASnd/UDP)
\param[in]      pSdoCom_p           Pointer to SDO command layer frame
\param[in]      sdoSize_p           Size of SDO command layer frame

\return The function returns a \ref tOplkError error code.
\retval kErrorOk                    The function returns successfully if the request was forwarded.
\retval Other                       Error occurred while sending the frame.

\ingroup module_sdotest
*/
//------------------------------------------------------------------------------
tOplkError oplk_testSdoComSend(UINT targetNodeId_p,
                               tSdoType sdoType_p,
                               const tAsySdoCom* pSdoCom_p,
                               size_t sdoSize_p)
{
    tOplkError  ret;

    // Forward request to testing module
    ret = sdotestcom_sendFrame(targetNodeId_p, sdoType_p, pSdoCom_p, sdoSize_p);
    if (ret != kErrorOk)
        ret = kErrorInvalidOperation;

    return ret;
}
//------------------------------------------------------------------------------
/**
\brief  SDO command layer test delete connection

The functions deletes the SDO command testing layer.

\return The function returns a \ref tOplkError error code.
\retval kErrorOk                    The function returns successfully if the connection was deleted.
\retval kErrorInvalidOperation      Error occurred while deleting the command layer connection.

\ingroup module_sdotest
*/
//------------------------------------------------------------------------------
tOplkError oplk_testSdoComDelCon(void)
{
    tOplkError  ret;

    ret = sdotestcom_closeCon();
    if (ret != kErrorOk)
       ret = kErrorInvalidOperation;

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  SDO sequence layer test send frame

The functions forwards the send request to the SDO sequence testing layer.

\param[in]      targetNodeId_p      Node ID of target node
\param[in]      sdoType_p           Type of SDO lower layer (ASnd/UDP)
\param[in]      pSdoSeq_p           Pointer to SDO sequence layer frame
\param[in]      sdoSize_p           Size of SDO sequence layer frame

\return The function returns a \ref tOplkError error code.
\retval kErrorOk                    The function returns successfully if the request was forwarded.
\retval Other                       Error occurred while sending the frame.

\ingroup module_sdotest
*/
//------------------------------------------------------------------------------
tOplkError oplk_testSdoSeqSend(UINT targetNodeId_p,
                               tSdoType sdoType_p,
                               const tAsySdoSeq* pSdoSeq_p,
                               size_t sdoSize_p)
{
    tOplkError  ret;

    // Forward request to testing module
    ret = sdotestseq_sendFrame(targetNodeId_p, sdoType_p, pSdoSeq_p, sdoSize_p);
    if (ret != kErrorOk)
       ret = kErrorInvalidOperation;

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  SDO sequence layer test delete connection

The function deletes the SDO sequence testing layer.

\return The function returns a \ref tOplkError error code.
\retval kErrorOk                    The function returns successfully if the connection was deleted.
\retval kErrorInvalidOperation      Error occurred while deleting the sequence layer connection.

\ingroup module_sdotest
*/
//------------------------------------------------------------------------------
tOplkError oplk_testSdoSeqDelCon(void)
{
    tOplkError  ret;

    ret = sdotestseq_closeCon();
    if (ret != kErrorOk)
       ret = kErrorInvalidOperation;

    return ret;
}

/// \}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Callback function for SDO command layer test module

Callback function for SDO command layer test module.

\param[in]      pAsySdoCom_p        Pointer to SDO command layer frame
\param[in]      dataSize_p          Size of SDO command layer frame

\return The function returns a \ref tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError oplkCbSdoTestCom(const tAsySdoCom* pAsySdoCom_p,
                                   size_t dataSize_p)
{
    tOplkError          ret;
    tOplkApiEventArg    eventArg;

    eventArg.receivedSdoCom.pAsySdoCom = (tAsySdoCom*)pAsySdoCom_p;
    eventArg.receivedSdoCom.dataSize = (UINT)dataSize_p;

    ret = initParam_l.pfnCbEvent(kOplkApiEventReceivedSdoCom,
                                 &eventArg,
                                 initParam_l.pEventUserArg);
    if (ret != kErrorOk)
        ret = kErrorInvalidEvent;

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Callback function for SDO sequence layer test module

Callback function for SDO command layer test module.

\param[in]      pAsySdoSeq_p        Pointer to SDO sequence layer frame
\param[in]      dataSize_p          Size of SDO sequence layer frame

\return The function returns a \ref tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError oplkCbSdoTestSeq(const tAsySdoSeq* pAsySdoSeq_p,
                                   size_t dataSize_p)
{
    tOplkError          ret = kErrorOk;
    tOplkApiEventArg    eventArg;

    eventArg.receivedSdoSeq.pAsySdoSeq = (tAsySdoSeq*)pAsySdoSeq_p;
    eventArg.receivedSdoSeq.dataSize = (UINT)dataSize_p;

    ret = initParam_l.pfnCbEvent(kOplkApiEventReceivedSdoSeq,
                                 &eventArg,
                                 initParam_l.pEventUserArg);
    if (ret != kErrorOk)
        ret = kErrorInvalidEvent;

    return ret;
}

/// \}
