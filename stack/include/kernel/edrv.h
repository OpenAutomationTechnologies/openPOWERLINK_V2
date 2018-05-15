/**
********************************************************************************
\file   kernel/edrv.h

\brief  Definitions for Ethernet driver module

This file contains definitions for the Ethernet driver module.
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
#ifndef _INC_kernel_edrv_H_
#define _INC_kernel_edrv_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define EDRV_MAX_MTU            1500
#define EDRV_MIN_MTU            46
#define EDRV_ETH_HDR_OFFSET     0   // Ethernet header at the top of the frame
#define EDRV_ETH_HDR_SIZE       14  // size of Ethernet header
#define EDRV_ETH_CRC_SIZE       4   // size of Ethernet CRC, i.e. FCS

#define EDRV_MAX_ETH_SIZE       (EDRV_MAX_MTU + EDRV_ETH_HDR_SIZE)  // without CRC
#define EDRV_MIN_ETH_SIZE       (EDRV_MIN_MTU + EDRV_ETH_HDR_SIZE)  // without CRC

#define EDRV_FILTER_CHANGE_VALUE                        0x01        // filter value changed
#define EDRV_FILTER_CHANGE_MASK                         0x02        // filter mask changed
#define EDRV_FILTER_CHANGE_STATE                        0x04        // filter state changed
#define EDRV_FILTER_CHANGE_AUTO_RESPONSE                0x08        // filter auto-resp. state changed

#if (CONFIG_EDRV_AUTO_RESPONSE_DELAY != FALSE)
#define EDRV_FILTER_CHANGE_AUTO_RESPONSE_DELAY          0x10        // filter auto-resp. delay changed
#define EDRV_FILTER_CHANGE_AUTO_RESPONSE_DELAY_DEF      EDRV_FILTER_CHANGE_AUTO_RESPONSE_DELAY
#else
#define EDRV_FILTER_CHANGE_AUTO_RESPONSE_DELAY_DEF      0
#endif

#define EDRV_FILTER_CHANGE_ALL  (0 |                                        \
                                 EDRV_FILTER_CHANGE_VALUE |                 \
                                 EDRV_FILTER_CHANGE_MASK |                  \
                                 EDRV_FILTER_CHANGE_STATE |                 \
                                 EDRV_FILTER_CHANGE_AUTO_RESPONSE |         \
                                 EDRV_FILTER_CHANGE_AUTO_RESPONSE_DELAY_DEF \
                                )

#ifndef CONFIG_EDRV_USE_DIAGNOSTICS
#define CONFIG_EDRV_USE_DIAGNOSTICS                     FALSE
#endif

#ifndef EDRV_USE_TTTX
#define EDRV_USE_TTTX                                   FALSE
#endif

//------------------------------------------------------------------------------
// Type definitions
//------------------------------------------------------------------------------
typedef struct sEdrvTxBuffer tEdrvTxBuffer;
typedef struct sEdrvRxBuffer tEdrvRxBuffer;

/**
\brief Enumeration for Rx buffer release

This enumeration lists the Rx buffer release commands.
*/
typedef enum
{
    kEdrvReleaseRxBufferImmediately = 0x00, ///< Release the Rx buffer immediately
    kEdrvReleaseRxBufferLater       = 0x01  ///< The Rx buffer is released later
} eEdrvReleaseRxBuffer;

/**
\brief Rx buffer release data type

Data type for the enumerator \ref eEdrvReleaseRxBuffer.
*/
typedef UINT32 tEdrvReleaseRxBuffer;

/// Callback function pointer for Rx frames
typedef tEdrvReleaseRxBuffer (*tEdrvRxHandler)(tEdrvRxBuffer* pRxBuffer_p);

/// Callback function pointer for Tx frames
typedef void (*tEdrvTxHandler)(tEdrvTxBuffer* pTxBuffer_p);

/**
\brief Enumeration for Rx buffer transfer state

This enumeration lists the transfer state of the Rx buffer.
Note that the Ethernet controller must support early Rx interrupts to access
the first or middle data of a frame!
*/
typedef enum
{
    kEdrvBufferFirstInFrame     = 0x01,     ///< First data of frame received
    kEdrvBufferMiddleInFrame    = 0x02,     ///< Middle data of frame received
    kEdrvBufferLastInFrame      = 0x04      ///< Last data of frame received
} eEdrvBufferInFrame;

/**
\brief Rx buffer transfer state data type

Data type for the enumerator \ref eEdrvBufferInFrame.
*/
typedef UINT32 tEdrvBufferInFrame;

/**
\brief Union for Tx buffer number

This union is used to identify the Tx buffer in the Ethernet driver module.
*/
typedef union
{
    UINT    value;                          ///< Number of the TX buffer
    void*   pArg;                           ///< Pointer to the TX buffer
} tEdrvTxBufferNumber;

/**
\brief Structure for Tx buffer

This structure is the Tx buffer descriptor.
*/
struct sEdrvTxBuffer
{
    size_t              txFrameSize;        ///< Size of Tx frame (without CRC)
    UINT32              timeOffsetNs;       ///< Tx delay to a previously sent frame [ns]
    BOOL                fLaunchTimeValid;   ///< Flag to identify a valid launch time
    union
    {
        UINT32          ticks;              ///< Launch time of the frame in ticks
        UINT64          nanoseconds;        ///< Launch time of the frame in nano seconds
    } launchTime;

    tEdrvTxHandler      pfnTxHandler;       ///< Tx callback function
    tEdrvTxBufferNumber txBufferNumber;     ///< Edrv Tx buffer number
    void*               pBuffer;            ///< Pointer to the Tx buffer
    size_t              maxBufferSize;      ///< Maximum size of the Tx buffer
};

/**
\brief Structure for Rx buffer

This structure is the Rx buffer descriptor.
*/
struct sEdrvRxBuffer
{
    tEdrvBufferInFrame  bufferInFrame;      ///< Position of Rx buffer in a frame
    size_t              rxFrameSize;        ///< Size of Rx frame (without CRC)
    void*               pBuffer;            ///< Pointer to the Rx buffer
    tTimestamp*         pRxTimeStamp;       ///< Pointer to Rx time stamp
};

/**
\brief Structure for initialization

This structure is used to initialize the Ethernet driver module.
*/
typedef struct
{
    UINT8                   aMacAddr[6];    ///< Ethernet controller MAC address
    tEdrvRxHandler          pfnRxHandler;   ///< Rx frame callback function pointer
    const char*             pDevName;       ///< Device name of the network interface card (valid if non-null)
    OPLK_DEPRECATED UINT    devNum;         ///< Device number (deprecated)
} tEdrvInitParam;

/**
\brief Structure for Rx filter

This structure is used to control the Rx filters.
*/
typedef struct
{
    UINT            handle;                 ///< Handle to Rx filter
    BOOL            fEnable;                ///< Enable the Rx filter
    UINT8           aFilterValue[22];       ///< Rx filter values
    UINT8           aFilterMask[22];        ///< Rx filter mask
    tEdrvTxBuffer*  pTxBuffer;              ///< Tx frame to be transmitted when filter matches
#if (EDRV_FILTER_WITH_RX_HANDLER != FALSE)
    tEdrvRxHandler  pfnRxHandler;           ///< Rx frame callback function pointer for this filter
#endif
} tEdrvFilter;

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------
#ifdef __cplusplus
extern "C"
{
#endif

tOplkError   edrv_init(const tEdrvInitParam* pEdrvInitParam_p);
tOplkError   edrv_exit(void);
const UINT8* edrv_getMacAddr(void);
tOplkError   edrv_setRxMulticastMacAddr(const UINT8* pMacAddr_p);
tOplkError   edrv_clearRxMulticastMacAddr(const UINT8* pMacAddr_p);
tOplkError   edrv_changeRxFilter(tEdrvFilter* pFilter_p,
                                 UINT count_p,
                                 UINT entryChanged_p,
                                 UINT changeFlags_p);
tOplkError   edrv_allocTxBuffer(tEdrvTxBuffer* pBuffer_p);
tOplkError   edrv_freeTxBuffer(tEdrvTxBuffer* pBuffer_p);
tOplkError   edrv_sendTxBuffer(tEdrvTxBuffer* pBuffer_p);

#if ((CONFIG_DLL_DEFERRED_RXFRAME_RELEASE_SYNC != FALSE) || (CONFIG_DLL_DEFERRED_RXFRAME_RELEASE_ASYNC != FALSE))
tOplkError   edrv_releaseRxBuffer(tEdrvRxBuffer* pBuffer_p);
#endif

#if (CONFIG_EDRV_AUTO_RESPONSE != FALSE)
tOplkError   edrv_updateTxBuffer(tEdrvTxBuffer* pBuffer_p);
#endif

#if (EDRV_USE_TTTX != FALSE)
tOplkError   edrv_getMacTime(UINT64* pCurtime_p);
#endif

#if (CONFIG_EDRV_USE_DIAGNOSTICS != FALSE)
int          edrv_getDiagnostics(char* pBuffer_p, size_t size_p);
#endif

#ifdef __cplusplus
}
#endif

#endif /* _INC_kernel_edrv_H_ */
