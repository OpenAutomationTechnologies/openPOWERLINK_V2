/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com
  (c) Bernecker + Rainer Industrie-Elektronik Ges.m.b.H.
      A-5142 Eggelsberg, B&R Strasse 1
      www.br-automation.com

  Project:      openPOWERLINK

  Description:  interface for Ethernet driver

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


****************************************************************************/

#ifndef _EDRV_H_
#define _EDRV_H_

#include "EplInc.h"
#include "EplFrame.h"

//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------

#define MAX_ETH_DATA_SIZE       1500
#define MIN_ETH_DATA_SIZE         46

#define ETH_HDR_OFFSET      0      // Ethernet header at the top of the frame
#define ETH_HDR_SIZE        14      // size of Ethernet header
#define MIN_ETH_SIZE        (MIN_ETH_DATA_SIZE + ETH_HDR_SIZE)    // without CRC

#define ETH_CRC_SIZE        4      // size of Ethernet CRC, i.e. FCS


#define EDRV_FILTER_CHANGE_VALUE                0x01  // filter value changed
#define EDRV_FILTER_CHANGE_MASK                 0x02  // filter mask changed
#define EDRV_FILTER_CHANGE_STATE                0x04  // filter state changed
#define EDRV_FILTER_CHANGE_AUTO_RESPONSE        0x08  // filter auto-resp. state changed
#if EDRV_AUTO_RESPONSE_DELAY != FALSE
#define EDRV_FILTER_CHANGE_AUTO_RESPONSE_DELAY  0x10  // filter auto-resp. delay changed
#define EDRV_FILTER_CHANGE_AUTO_RESPONSE_DELAY_DEF  EDRV_FILTER_CHANGE_AUTO_RESPONSE_DELAY
#else
#define EDRV_FILTER_CHANGE_AUTO_RESPONSE_DELAY_DEF  0
#endif

#define EDRV_FILTER_CHANGE_ALL      (EDRV_FILTER_CHANGE_VALUE \
                                     | EDRV_FILTER_CHANGE_MASK \
                                     | EDRV_FILTER_CHANGE_STATE \
                                     | EDRV_FILTER_CHANGE_AUTO_RESPONSE \
                                     | EDRV_FILTER_CHANGE_AUTO_RESPONSE_DELAY_DEF)


#ifndef EDRV_USE_DIAGNOSTICS
#define EDRV_USE_DIAGNOSTICS                    FALSE
#endif

#ifndef EDRV_CYCLIC_USE_DIAGNOSTICS
#define EDRV_CYCLIC_USE_DIAGNOSTICS             FALSE
#endif
#ifndef EDRV_CYCLIC_SAMPLE_NUM
#define EDRV_CYCLIC_SAMPLE_NUM                  501
#endif


//---------------------------------------------------------------------------
// types
//---------------------------------------------------------------------------

typedef struct _tEdrvTxBuffer tEdrvTxBuffer;
typedef struct _tEdrvRxBuffer tEdrvRxBuffer;

typedef enum
{
    kEdrvReleaseRxBufferImmediately = 0x00,
    kEdrvReleaseRxBufferLater       = 0x01

} tEdrvReleaseRxBuffer;

typedef tEdrvReleaseRxBuffer (*tEdrvRxHandler) (tEdrvRxBuffer * pRxBuffer_p);
typedef void (*tEdrvTxHandler) (tEdrvTxBuffer * pTxBuffer_p);
typedef tEplKernel (* tEdrvCyclicCbSync) (void);
typedef tEplKernel (* tEdrvCyclicCbError) (tEplKernel ErrorCode_p, tEdrvTxBuffer * pTxBuffer_p);


// position of a buffer in an ethernet-frame
typedef enum
{
    kEdrvBufferFirstInFrame   = 0x01,  // first data buffer in an ethernet frame
    kEdrvBufferMiddleInFrame  = 0x02,  // a middle data buffer in an ethernet frame
    kEdrvBufferLastInFrame    = 0x04   // last data buffer in an ethernet frame
} tEdrvBufferInFrame;


// format of a tx-buffer
struct _tEdrvTxBuffer
{
    unsigned int    m_uiTxMsgLen;           // IN: length of message to be send (set for each transmit call)
    DWORD           m_dwTimeOffsetNs;       // IN: delay to a previous frame after which this frame will be transmitted
    DWORD           m_dwTimeOffsetAbsTk;    // IN: absolute time when frame will be transmitted (in MAC ticks)
    tEdrvTxHandler  m_pfnTxHandler;         // IN: special Tx callback function
    // ----------------------
    union
    {
        unsigned int    m_uiVal;
        DWORD           m_dwVal;
        void*           m_pVal;
    } m_BufferNumber;                       // OUT: number of the buffer, set by ethernetdriver
    BYTE*           m_pbBuffer;             // OUT: pointer to the buffer, set by ethernetdriver
    tEplNetTime     m_NetTime;              // OUT: Timestamp of end of transmission, set by ethernetdriver
    // ----------------------
    unsigned int    m_uiMaxBufferLen;       // IN/OUT: maximum length of the buffer

};


// format of a rx-buffer
struct _tEdrvRxBuffer
{
    tEdrvBufferInFrame  m_BufferInFrame;    // OUT position of received buffer in an ethernet-frame
    unsigned int        m_uiRxMsgLen;       // OUT: length of received buffer (without CRC)
    BYTE  *             m_pbBuffer;         // OUT: pointer to the buffer, set by ethernetdriver
    tEplTgtTimeStamp*   m_pTgtTimeStamp;    // OUT: pointer to Timestamp of end of receiption

};



// format of init structure
typedef struct
{
    BYTE            m_abMyMacAddr[6];       // the own MAC address

//    BYTE            m_bNoOfRxBuffDescr;     // number of entries in rx bufferdescriptor table
//    tBufferDescr *  m_pRxBuffDescrTable;    // rx bufferdescriptor table
//    WORD            m_wRxBufferSize;        // size of the whole rx buffer

    tEdrvRxHandler  m_pfnRxHandler;
    tEdrvTxHandler  m_pfnTxHandler;

    tEplHwParam     m_HwParam;

} tEdrvInitParam;


typedef struct
{
    unsigned int    m_uiHandle;   // [in/out] set by Edrv
    BOOL            m_fEnable;
    BYTE            m_abFilterValue[22];
    BYTE            m_abFilterMask[22];
    tEdrvTxBuffer*  m_pTxBuffer;  // Auto response
#if (EDRV_FILTER_WITH_RX_HANDLER != FALSE)
    tEdrvRxHandler  m_pfnRxHandler;
#endif

} tEdrvFilter;


typedef struct
{
    // continuous min/max/avg measurement
    unsigned long long  m_ullCycleCount;
    DWORD               m_dwCycleTimeMin;
    DWORD               m_dwCycleTimeMax;
    unsigned long long  m_ullCycleTimeMeanSum;  // sums run over after some years for ct=400
    DWORD               m_dwUsedCycleTimeMin;
    DWORD               m_dwUsedCycleTimeMax;
    unsigned long long  m_ullUsedCycleTimeMeanSum;
    DWORD               m_dwSpareCycleTimeMin;
    DWORD               m_dwSpareCycleTimeMax;
    unsigned long long  m_ullSpareCycleTimeMeanSum;

    // sampling of runaway cycles
    unsigned int        m_uiSampleNum;
    unsigned int        m_uiSampleBufferedNum;
    unsigned long long  m_aullSampleTimeStamp[EDRV_CYCLIC_SAMPLE_NUM];  // SOC send
    DWORD               m_adwCycleTime[EDRV_CYCLIC_SAMPLE_NUM];         // until next SOC send
    DWORD               m_adwUsedCycleTime[EDRV_CYCLIC_SAMPLE_NUM];
    DWORD               m_adwSpareCycleTime[EDRV_CYCLIC_SAMPLE_NUM];

} tEdrvCyclicDiagnostics;


//---------------------------------------------------------------------------
// function prototypes
//---------------------------------------------------------------------------


tEplKernel EdrvInit                   (tEdrvInitParam* pEdrvInitParam_p);

tEplKernel EdrvShutdown               (void);

tEplKernel EdrvDefineRxMacAddrEntry   (BYTE * pbMacAddr_p);
tEplKernel EdrvUndefineRxMacAddrEntry (BYTE * pbMacAddr_p);

tEplKernel EdrvAllocTxMsgBuffer     (tEdrvTxBuffer* pBuffer_p);
tEplKernel EdrvReleaseTxMsgBuffer   (tEdrvTxBuffer* pBuffer_p);
tEplKernel EdrvUpdateTxMsgBuffer    (tEdrvTxBuffer* pBuffer_p);

//tEplKernel EdrvWriteMsg             (tBufferDescr* pbBuffer_p);
tEplKernel EdrvSendTxMsg            (tEdrvTxBuffer* pBuffer_p);
tEplKernel EdrvTxMsgReady           (tEdrvTxBuffer* pBuffer_p);
tEplKernel EdrvTxMsgStart           (tEdrvTxBuffer* pBuffer_p);

tEplKernel EdrvReleaseRxBuffer      (tEdrvRxBuffer* pBuffer_p);

tEplKernel EdrvChangeFilter(tEdrvFilter*    pFilter_p,
                            unsigned int    uiCount_p,
                            unsigned int    uiEntryChanged_p,
                            unsigned int    uiChangeFlags_p);


int EdrvGetDiagnostics(char* pszBuffer_p, int iSize_p);

// EdrvCyclic module
tEplKernel EdrvCyclicInit(void);
tEplKernel EdrvCyclicShutdown(void);
tEplKernel EdrvCyclicSetCycleLenUs(DWORD dwCycleLenUs_p);
tEplKernel EdrvCyclicStartCycle(void);
tEplKernel EdrvCyclicStopCycle(void);
tEplKernel EdrvCyclicSetMaxTxBufferListSize(unsigned int uiMaxListSize_p);
tEplKernel EdrvCyclicSetNextTxBufferList(tEdrvTxBuffer** apTxBuffer_p, unsigned int uiTxBufferCount_p);
tEplKernel EdrvCyclicRegSyncHandler(tEdrvCyclicCbSync pfnEdrvCyclicCbSync_p);
tEplKernel EdrvCyclicRegErrorHandler(tEdrvCyclicCbError pfnEdrvCyclicCbError_p);
tEplKernel EdrvCyclicGetDiagnostics(tEdrvCyclicDiagnostics** ppDiagnostics_p);


// interrupt handler called by target specific interrupt handler
void        EdrvInterruptHandler       (void);



#endif  // #ifndef _EDRV_H_


