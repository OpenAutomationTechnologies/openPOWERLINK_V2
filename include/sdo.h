/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  include file for api function of the sdo module

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

 2006/06/26 k.t.:   start of the implementation


****************************************************************************/

#ifndef _EPLSDO_H_
#define _EPLSDO_H_

#include "EplInc.h"
#include "EplFrame.h"
#include "EplSdoAc.h"


//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------
// global defines
#ifndef SDO_MAX_SEGMENT_SIZE
#define SDO_MAX_SEGMENT_SIZE        256
#endif

// handle between Protocol Abstraction Layer and asynchronous SDO Sequence Layer
#define SDO_UDP_HANDLE              0x8000
#define SDO_ASND_HANDLE             0x4000
#define SDO_ASY_HANDLE_MASK         0xC000
#define SDO_ASY_INVALID_HDL         0x3FFF

// handle between SDO Sequence Layer and SDO command layer
#define SDO_ASY_HANDLE              0x8000
#define SDO_PDO_HANDLE              0x4000
#define SDO_SEQ_HANDLE_MASK         0xC000
#define SDO_SEQ_INVALID_HDL         0x3FFF

#define ASND_HEADER_SIZE            4

#define SEQ_NUM_MASK                0xFC

// size for send buffer and history
#define SDO_MAX_FRAME_SIZE          EPL_C_IP_MIN_MTU
// size for receive frame
// -> needed because SND-Kit sends up to 1518 Byte
//    without Sdo-Command: Maximum Segment Size
#define SDO_MAX_REC_FRAME_SIZE      EPL_C_IP_MAX_MTU

//---------------------------------------------------------------------------
// typedef
//---------------------------------------------------------------------------
// handle between Protocol Abstraction Layer and asynchronous SDO Sequence Layer
typedef unsigned int tSdoConHdl;

// callback function pointer for Protocol Abstraction Layer to call
// asynchronous SDO Sequence Layer
typedef tEplKernel (PUBLIC* tSequLayerReceiveCb ) (
    tSdoConHdl          ConHdl_p,
    tAsySdoSeq*         pSdoSeqData_p,
    unsigned int        uiDataSize_p);

// handle between asynchronous SDO Sequence Layer and SDO Command layer
typedef unsigned int tSdoSeqConHdl;

// callback function pointer for asynchronous SDO Sequence Layer to call
// SDO Command layer for received data
typedef tEplKernel (PUBLIC* tSdoComReceiveCb) (
    tSdoSeqConHdl       SdoSeqConHdl_p,
    tAsySdoCom*         pAsySdoCom_p,
    unsigned int        uiDataSize_p);

// status of connection
typedef enum
{
    kAsySdoConStateConnected    = 0x00,
    kAsySdoConStateInitError    = 0x01,
    kAsySdoConStateConClosed    = 0x02,
    kAsySdoConStateAckReceived  = 0x03,
    kAsySdoConStateFrameSended  = 0x04,
    kAsySdoConStateTimeout      = 0x05,
    kAsySdoConStateTransferAbort= 0x06,

}tAsySdoConState;

// callback function pointer for asynchronous SDO Sequence Layer to call
// SDO Command layer for connection status
typedef tEplKernel (PUBLIC* tSdoComConCb) (
    tSdoSeqConHdl    SdoSeqConHdl_p,
    tAsySdoConState  AsySdoConState_p);

// handle between  SDO Command layer and application
typedef unsigned int tSdoComConHdl;

// status of connection
typedef enum
{
    kEplSdoComTransferNotActive         =   0x00,
    kEplSdoComTransferRunning           =   0x01,
    kEplSdoComTransferTxAborted         =   0x02,
    kEplSdoComTransferRxAborted         =   0x03,
    kEplSdoComTransferFinished          =   0x04,
    kEplSdoComTransferLowerLayerAbort   =   0x05

} tSdoComConState;

// SDO Services and Command-Ids from DS 1.0.0 p.152
typedef enum
{
    kSdoServiceNIL                  = 0x00,
    kSdoServiceWriteByIndex         = 0x01,
    kSdoServiceReadByIndex          = 0x02,
    // the following services are optional and are not supported now
    kSdoServiceWriteAllByIndex      = 0x03,
    kSdoServiceReadAllByIndex       = 0x04,
    kSdoServiceWriteByName          = 0x05,
    kSdoServiceReadByName           = 0x06,
    kSdoServiceFileWrite            = 0x20,
    kSdoServiceFileRead             = 0x21,
    kSdoServiceWriteMultiByIndex    = 0x31,
    kSdoServiceReadMultiByIndex     = 0x32,
    kSdoServiceMaxSegSize           = 0x70
    // 0x80 - 0xFF manufacturer specific
} tSdoServiceType;

// describes if read or write access
typedef enum
{
    kSdoAccessTypeRead              = 0x00,
    kSdoAccessTypeWrite             = 0x01

} tSdoAccessType;

typedef enum
{
    kSdoTypeAuto                    = 0x00,
    kSdoTypeUdp                     = 0x01,
    kSdoTypeAsnd                    = 0x02,
    kSdoTypePdo                     = 0x03

}tSdoType;

typedef enum
{
    kSdoTransAuto                   = 0x00,
    kSdoTransExpedited              = 0x01,
    kSdoTransSegmented              = 0x02


} tSdoTransType;


// structure to inform application about finish of SDO transfer
typedef struct
{
    tSdoComConHdl       m_SdoComConHdl;
    tSdoComConState     m_SdoComConState;
    DWORD               m_dwAbortCode;
    tSdoAccessType      m_SdoAccessType;
    unsigned int        m_uiNodeId;         // NodeId of the target
    unsigned int        m_uiTargetIndex;    // index which was accessed
    unsigned int        m_uiTargetSubIndex; // subindex which was accessed
    unsigned int        m_uiTransferredByte; // number of bytes transferred
    void*               m_pUserArg;         // user definable argument pointer

} tSdoComFinished;


// callback function pointer to inform application about connection
typedef tEplKernel (PUBLIC* tSdoFinishedCb) (
    tSdoComFinished* pSdoComFinished_p);


// structure to init SDO transfer to Read or Write by Index
typedef struct
{
    tSdoComConHdl       m_SdoComConHdl;
    unsigned int        m_uiIndex;
    unsigned int        m_uiSubindex;
    void*               m_pData;
    unsigned int        m_uiDataSize;
    unsigned int        m_uiTimeout;    // not used in this version
    tSdoAccessType      m_SdoAccessType;
    tSdoFinishedCb      m_pfnSdoFinishedCb;
    void*               m_pUserArg;         // user definable argument pointer

} tSdoComTransParamByIndex;




//---------------------------------------------------------------------------
// function prototypes
//---------------------------------------------------------------------------


#endif  // #ifndef _EPLSDO_H_


