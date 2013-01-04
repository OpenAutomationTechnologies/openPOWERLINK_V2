/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  include file for PDO module

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

  2006/05/22 d.k.:   start of the implementation, version 1.00


****************************************************************************/

#ifndef _EPL_PDO_H_
#define _EPL_PDO_H_

#include "EplInc.h"

//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------

// invalid PDO-NodeId
#define EPL_PDO_INVALID_NODE_ID     0xFF
// NodeId for PReq RPDO
#define EPL_PDO_PREQ_NODE_ID        0x00
// NodeId for PRes TPDO
#define EPL_PDO_PRES_NODE_ID        0x00

#define EPL_PDO_COMMUNICATION_PROFILE_START 0x1000

#if EPL_D_PDO_RPDOChannelObjects_U8 > EPL_D_PDO_TPDOChannelObjects_U8
#define EPL_PDO_CHANNEL_OBJECT_COUNT    EPL_D_PDO_RPDOChannelObjects_U8
#else
#define EPL_PDO_CHANNEL_OBJECT_COUNT    EPL_D_PDO_TPDOChannelObjects_U8
#endif


#define EPL_PDO_MAPPOBJECT_IS_NUMERIC(pPdoMappObject_p) \
            (pPdoMappObject_p->m_wByteSizeOrType < EPL_PDO_COMMUNICATION_PROFILE_START)

#define EPL_PDO_MAPPOBJECT_GET_VAR(pPdoMappObject_p) \
            pPdoMappObject_p->m_pVar

#define EPL_PDO_MAPPOBJECT_SET_VAR(pPdoMappObject_p, pVar_p) \
            (pPdoMappObject_p->m_pVar = pVar_p)

#define EPL_PDO_MAPPOBJECT_GET_BITOFFSET(pPdoMappObject_p) \
            pPdoMappObject_p->m_wBitOffset

#define EPL_PDO_MAPPOBJECT_SET_BITOFFSET(pPdoMappObject_p, wBitOffset_p) \
            (pPdoMappObject_p->m_wBitOffset = wBitOffset_p)

#define EPL_PDO_MAPPOBJECT_GET_BYTESIZE(pPdoMappObject_p) \
            (pPdoMappObject_p->m_wByteSizeOrType - EPL_PDO_COMMUNICATION_PROFILE_START)

#define EPL_PDO_MAPPOBJECT_GET_TYPE(pPdoMappObject_p) \
            ((tEplObdType) pPdoMappObject_p->m_wByteSizeOrType)

#define EPL_PDO_MAPPOBJECT_SET_BYTESIZE_OR_TYPE(pPdoMappObject_p, wByteSize_p, ObdType_p) \
            if ((ObdType_p == kEplObdTypVString) || (ObdType_p == kEplObdTypOString) || (ObdType_p == kEplObdTypDomain)) \
            { \
                pPdoMappObject_p->m_wByteSizeOrType = wByteSize_p + EPL_PDO_COMMUNICATION_PROFILE_START; \
            } \
            else \
            { \
                pPdoMappObject_p->m_wByteSizeOrType = ObdType_p; \
            }



//---------------------------------------------------------------------------
// typedef
//---------------------------------------------------------------------------

// d.k. optimization idea: use bit-field for WORDs and BOOL.
//      This will not be portable, because only unsigned int is allowed for bit-fields
//      and unsigned int has only 16 bit width on 16 bit CPUs.

typedef struct
{
    void*               m_pVar;
    WORD                m_wBitOffset;   // in Bits
    WORD                m_wByteSizeOrType;

} tEplPdoMappObject;


typedef struct
{
    unsigned int        m_uiNodeId;
    // 0xFF=invalid; RPDO: 0x00=PReq, localNodeId=PRes, remoteNodeId=PRes;
    //               TPDO: 0x00=PRes, MN: CnNodeId=PReq

    WORD                m_wPdoSize;
    BYTE                m_bMappingVersion;
    unsigned int        m_uiMappObjectCount;    // actual number of used mapped objects

} tEplPdoChannel;


typedef struct
{
    // m_fTx and m_uiChannelId form the unique key
    unsigned int        m_uiChannelId;
    BOOL                m_fTx;              // TRUE = TPDO, FALSE = RPDO

    tEplPdoChannel      m_PdoChannel;

    tEplPdoMappObject   m_aMappObject[EPL_PDO_CHANNEL_OBJECT_COUNT];

} tEplPdoChannelConf;


typedef struct
{
    unsigned int        m_uiRxPdoChannelCount;  // max. number of RPDO channels
    unsigned int        m_uiTxPdoChannelCount;  // max. number of TPDO channels

} tEplPdoAllocationParam;


//---------------------------------------------------------------------------
// function prototypes
//---------------------------------------------------------------------------


#endif  // #ifndef _EPL_PDO_H_


