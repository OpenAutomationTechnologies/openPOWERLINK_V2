/**
********************************************************************************
\file   user/dllucal.h

\brief  Definitions for user DLL CAL module

This header file contains definitions for the user DLL CAL module.

Copyright (c) 2012, SYSTEC electronik GmbH
Copyright (c) 2016, B&R Industrial Automation GmbH
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
*******************************************************************************/
#ifndef _INC_user_dllucal_H_
#define _INC_user_dllucal_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>
#include <oplk/dll.h>
#include <oplk/event.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

/**
* \brief Callback function for handling received ASnd frames
*
* This function type is used for callback functions handling received ASnd
* frames.
*
* \param[in]    pFrameInfo_p        Information about received ASnd frame.
*
* \return The function returns a tOplkError error code
*/
typedef tOplkError (*tDlluCbAsnd)(const tFrameInfo* pFrameInfo_p);

/**
* \brief Callback function for handling received Non-PLK frames
*
* This function type is used for callback functions handling received non-
* POWERLINK (Ethernet) frames.
*
* \param[in]    pFrameInfo_p        Information about received frame.
*
* \return The function returns a tOplkError error code
*/
typedef tOplkError (*tDlluCbNonPlk)(const tFrameInfo* pFrameInfo_p);

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------
#ifdef __cplusplus
extern "C"
{
#endif

tOplkError dllucal_init(void);
tOplkError dllucal_exit(void);
tOplkError dllucal_config(const tDllConfigParam* pDllConfigParam_p);
tOplkError dllucal_setIdentity(const tDllIdentParam* pDllIdentParam_p);

#if defined(CONFIG_INCLUDE_VETH)
tOplkError dllucal_regNonPlkHandler(tDlluCbNonPlk pfnNonPlkCb_p);
#endif

tOplkError dllucal_regAsndService(tDllAsndServiceId ServiceId_p,
                                  tDlluCbAsnd pfnDlluCbAsnd_p,
                                  tDllAsndFilter Filter_p);
tOplkError dllucal_sendAsyncFrame(const tFrameInfo* pFrameInfo,
                                  tDllAsyncReqPriority priority_p);
tOplkError dllucal_process(const tEvent* pEvent_p);

#if (NMT_MAX_NODE_ID > 0)
tOplkError dllucal_configNode(const tDllNodeInfo* pNodeInfo_p);
tOplkError dllucal_addNode(const tDllNodeOpParam* pNodeOpParam_p);
tOplkError dllucal_deleteNode(const tDllNodeOpParam* pNodeOpParam_p);
#endif

#if defined(CONFIG_INCLUDE_NMT_MN)
tOplkError dllucal_issueRequest(tDllReqServiceId service_p,
                                UINT nodeId_p,
                                UINT8 soaFlag1_p);
tOplkError dllucal_issueSyncRequest(const tDllSyncRequest* pSyncRequest_p,
                                    size_t size_p);
#endif

#ifdef __cplusplus
}
#endif

#endif /* _INC_user_dllucal_H_ */
