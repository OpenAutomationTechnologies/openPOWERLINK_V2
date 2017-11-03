/**
********************************************************************************
\file   user/sdotest.h

\brief  Definitions for SDO test functions

This file contains the function declaration for the SDO sequence layer test.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, B&R Industrial Automation GmbH
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
#ifndef _INC_user_sdotest_H_
#define _INC_user_sdotest_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/ami.h>
#include <common/circbuffer.h>
#include <oplk/dll.h>
#include <user/eventu.h>
#include <user/sdoudp.h>
#include <user/sdoasnd.h>
#include <user/sdoseq.h>
#include <user/eventu.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

/**
\brief SDO sequence layer test callback function.

This type defines a function pointer for the SDO sequence layer to
call the SDO command layer for the connection status.

\param[in]      pSdoSeqData_p       Pointer to SDO sequence layer frame
\param[in]      dataSize_p          Size of SDO sequence layer frame

\return The function returns a tOplkError error code
*/
typedef tOplkError (*sdoApiCbSeqTest)(const tAsySdoSeq* pSdoSeqData_p,
                                      size_t dataSize_p);

/**
\brief SDO command layer test callback function.

This type defines a function pointer for the SDO command layer to
call the SDO sequence layer for the connection status.

\param[in]      pSdoComData_p       Pointer to SDO command layer frame
\param[in]      dataSize_p          Size of SDO command layer frame

\return The function returns a tOplkError error code
*/
typedef tOplkError (*sdoApiCbComTest)(const tAsySdoCom* pSdoComData_p,
                                      size_t dataSize_p);

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------
#ifdef __cplusplus
extern "C"
{
#endif

tOplkError sdotestseq_init(sdoApiCbSeqTest sdoSequCbApi_p);
tOplkError sdotestseq_exit(void);
tOplkError sdotestseq_sendFrame(UINT nodeId_p,
                                tSdoType sdo_type_p,
                                const tAsySdoSeq* pSdoSeq_p,
                                size_t sdoSize_p);
tOplkError sdotestseq_closeCon(void);

tOplkError sdotestcom_init(sdoApiCbComTest sdoComuCbApi_p);
tOplkError sdotestcom_exit(void);
tOplkError sdotestcom_sendFrame(UINT nodeId_p,
                                tSdoType sdo_type_p,
                                const tAsySdoCom* pSdoCom_p,
                                size_t sdoSize_p);
tOplkError sdotestcom_closeCon(void);
tOplkError sdotestcom_cbEvent(const tEvent* pEvent_p);

#ifdef __cplusplus
}
#endif

#endif /* _INC_user_sdotest_H_ */
