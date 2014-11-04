/**
********************************************************************************
\file   ndis-imInternal.h

\brief  Internal header file for NDIS driver

This file contains the common typedefs, data types and constant declarations
to be used across the NDIS driver library.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Kalycito Infotech Private Limited
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

#ifndef _INC_ndis_imInternal_H_
#define _INC_ndis_imInternal_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <ndis.h>
#include "ndis-im.h"

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define OPLK_MEM_TAG           'klpO'
#define OPLK_MAX_FRAME_SIZE    1536
#define OPLK_ALLOCATED_NBL     0x10000000
#define OPLK_MAX_VETH_BUFF     40           // Number of receive buffers for VEth interface.

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

/**
\brief NDIS driver instance structure

This structure holds the parameters for the NDIS driver instance.

*/
typedef struct
{
    tDrvIntfRegister      pfnDrvIntfRegCb;          ///< Driver interface register callback.
    tDrvIntfDeregister    pfnDrvIntfDeregisterCb;   ///< Driver interface deregister callback.
    NDIS_HANDLE           hMiniportHandle;          ///< Miniport driver handle returned by OS.
    NDIS_HANDLE           hProtocolHandle;          ///< Protocol driver handle returned by OS.
} tNdisDriverInstance;

/**
\brief NDIS OID request structure

This structure holds the parameters for an OID request.

*/
typedef struct
{
    NDIS_STATUS         status;                 ///< Completion status of the OID.
    NDIS_EVENT          waitEvent;              ///< Event handle to block completion.
    NDIS_OID_REQUEST    oidRequest;             ///< NDIS OID request structure variable for the request.
} tNdisOidRequest;

/**
\brief Receive buffer information

Receive buffer information structure for NDIS driver.

*/
typedef struct
{
    void*      pData;           ///< Pointer to Receive buffer.
    ULONG      maxLength;       ///< Max length of the buffer.
    ULONG      length;          ///< Length of the buffer in receive frame.
    BOOLEAN    fFree;           ///< Flag to identify buffer access status.
} tRxBufInfo;

/**
\brief Transmit buffer information

Transmit buffer information structure for NDIS miniport.

*/
typedef struct
{
    LIST_ENTRY          txLink;         ///< List entry for the transmit buffer info.
    PNET_BUFFER_LIST    pNbl;           ///< Pointer to NET_BUFFER_LIST this transmit buffer is mapped to.
    PMDL                pMdl;           ///< MDL describing the transmit buffer.
    BOOLEAN             fFree;          ///< Flag to identify buffer access status.
    ULONG               maxLength;      ///< Max length of the buffer.
    ULONG               length;         ///< Length of the buffer in transmit frame.
    void*               pToken;         ///< Identification token for the buffer.
    void*               pData;          ///< Pointer to transmit buffer.
} tTxBufInfo;

/**
\brief Receive buffer information for VEth frames

Receive buffer information structure for VEth frames.

*/
typedef struct
{
    LIST_ENTRY          rxLink;             ///< List entry for the receive buffer.
    PNET_BUFFER_LIST    pNbl;               ///< Pointer to NetBufferList for the buffer.
    PMDL                pMdl;               ///< MDL describing the receive buffer.
    BOOLEAN             fFree;              ///< Flag to identify buffer access status.
    ULONG               maxLength;          ///< Max length of the buffer.
    ULONG               length;             ///< Length of the buffer in transmit frame.
    void*               pData;              ///< Pointer to receive buffer.
} tVEthRcvBufInfo;

/**
\brief NDIS protocol instance structure

The structure holds all the parameters required by the NDIS protocol instance.

*/
typedef struct
{
    NDIS_HANDLE                hBindingHandle;          ///< NDIS handle for lower binding to the protocol.
    NDIS_HANDLE                hSendNblPool;            ///< Send NET_BUFFER_LIST pool handle.
    NDIS_EVENT                 adapterEvent;            ///< Adapter initialization event.
    PNDIS_EVENT                pPauseEvent;             ///< Adapter pause event.
    PNDIS_EVENT                pOidCompleteEvent;       ///< OID complete synchronization event.
    NDIS_SPIN_LOCK             pauseEventLock;          ///< NDIS lock for pause event.
    NDIS_SPIN_LOCK             driverLock;              ///< NDIS lock for adapter initialization and exit event.
    NDIS_STATUS                adapterInitStatus;       ///< Adapter initialization status.
    NDIS_LINK_STATE            lastLinkState;           ///< Last updated link state information.
    tNdisBindingState          bindingState;            ///< Binding state of the protocol instance.
    NDIS_BIND_PARAMETERS       bindParameters;          ///< NDIS bind parameters for the protocol to miniport binding.
    ULONG                      oidReq;                  ///< OID request count.
    ULONG                      sendRequest;             ///< Send request count.
    void*                      pVEthInstance;           ///< Pointer to virtual Ethernet context structure.
    tNdisReceiveCb             pfnReceiveCb;            ///< Function pointer for receive callback.
    tNdisTransmitCompleteCb    pfnTransmitCompleteCb;   ///< Function pointer for transmit complete callback.
    tRxBufInfo*                pReceiveBufInfo;         ///< Pointer to receive buffer information list.
    ULONG                      receiveHead;             ///< Current receive head.
    ULONG                      receiveBufCount;         ///< Total receive count for statistical information.
    ULONG                      transmitBufCount;        ///< Total transmit count for statistical information.
    void*                      pTransmitBuf;            ///< Pointer to transmit buffer.
    tTxBufInfo*                pTxBuffInfo;             ///< Pointer to list of transmit buffer information.
    void*                      pReceiveBuf;             ///< Pointer to receive buffer.
    LIST_ENTRY                 txList;                  ///< Transmit queue.
    NDIS_SPIN_LOCK             txListLock;              ///< Transmit queue access lock.
} tProtocolInstance;

/**
\brief Virtual Ethernet module instance structure

This structure holds the variables used by the virtual Ethernet adapter instance.

*/
typedef struct
{
    NDIS_HANDLE                   hMiniportAdapterHandle;   ///< Adapter handle for the NDIS miniport.
    NDIS_HANDLE                   hBindingHandle;           ///< NDIS handle to the upper binding.
    NDIS_HANDLE                   hReceiveNblPool;          ///< Receive NetBufferLists pool.
    BOOLEAN                       fMiniportHalting;         ///< Flag to identify miniport halting status.
    BOOLEAN                       fMiniportPaused;          ///< Flag to identify paused status.
    NDIS_STRING                   cfgDeviceName;            ///< Used as the unique ID for the VETH.
    // Some standard miniport parameters (OID values).
    ULONG                         packetFilter;             ///< Current packet filter for the miniport.
    ULONG                         lookAhead;                ///< Lookahead flags for the miniport.
    ULONG64                       linkSpeed;                ///< Link speed of the miniport to broadcast.
    ULONG                         maxBusySends;             ///< Max busy send count.
    ULONG                         maxBusyRecvs;             ///< Max busy receive count.
    // Packet counts (Used for handling OIDs)
    ULONG64                       goodTransmits;            ///< Total count of good transmits.
    ULONG64                       goodReceives;             ///< Total count of good receives.
    NDIS_LINK_STATE               lastPendingLinkState;     ///< Last pending link state request.
    NDIS_STATUS                   pendingStatusIndication;  ///< Last pending status indication request.
    NDIS_STATUS                   lastLinkStatus;           ///< Last link status received.
    NDIS_LINK_STATE               lastLinkState;            ///< Last link state change.
    UCHAR                         aPermanentAddress[ETH_LENGTH_OF_ADDRESS];  ///< Permanent MAC address of the NIC.
    UCHAR                         aCurrentAddress[ETH_LENGTH_OF_ADDRESS];    ///< Current MAC used by miniport.
    ULONG                         state;                    ///< Current miniport state.
    NDIS_EVENT                    miniportInitEvent;        ///< Miniport initialization complete event.
    BOOLEAN                       fMiniportInitPending;     ///< Flag to identify miniport initialization completion.
    BOOLEAN                       fOidRequestPending;       ///< Flag for pending OID request handling.
    NDIS_SPIN_LOCK                miniportLock;             ///< Global miniport lock.
    NDIS_SPIN_LOCK                pauseLock;                ///< Lock for miniport pause event.
    tNdisOidRequest               ndisOidReq;               ///< NDIS OID request information.
    NDIS_STATUS                   status;                   ///< Miniport status.
    tProtocolInstance*            pProtocolInstance;        ///< Pointer to protocol instance in a NDIS intermediate driver.
    NET_IFINDEX                   ifIndex;                  ///< Interface index in the Windows network stack.
    ULONG                         sendRequests;             ///< Total send requests handled.
    ULONG                         receiveIndication;        ///< Total receive indications handled.
    tVEthSendCb                   pfnVEthSendCb;            ///< Callback routine for VEth transmit.
    tVEthRcvBufInfo*              pReceiveBufInfo;          ///< Pointer to receive buffer information.
    void*                         pReceiveBuf;              ///< Pointer to the memory for VEth receive.
    LIST_ENTRY                    rxList;                   ///< List entry for VEth receive queue.
    NDIS_SPIN_LOCK                rxListLock;               ///< Spin lock for VEth receive queue.
} tVEthInstance;

//------------------------------------------------------------------------------
// global defines
//------------------------------------------------------------------------------
extern tNdisDriverInstance    driverInstance_g;

//------------------------------------------------------------------------------
// macros
//------------------------------------------------------------------------------
#ifndef NDEBUG
#define TRACE(...)    DbgPrint(__VA_ARGS__)
#else
#define TRACE(...)
#endif

#define TXINFO_FROM_NBL(_NBL)    ((tTxBufInfo*)((_NBL)->ProtocolReserved[0]))
#define VETHINFO_FROM_NBL(_NBL)    ((tVEthRcvBufInfo*)((_NBL)->MiniportReserved[0]))

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------
#ifdef __cplusplus
extern "C"
{
#endif

// Miniport driver prototypes
DRIVER_DISPATCH                            miniportIoDispatch;
DRIVER_DISPATCH                            miniportDeviceIoControl;
MINIPORT_SET_OPTIONS                       miniportSetOptions;
MINIPORT_INITIALIZE                        miniportInitialize;
MINIPORT_HALT                              miniportHalt;
MINIPORT_UNLOAD                            miniportUnload;
MINIPORT_PAUSE                             miniportPause;
MINIPORT_RESTART                           miniportRestart;
MINIPORT_OID_REQUEST                       miniportOidRequest;
MINIPORT_SEND_NET_BUFFER_LISTS             miniportSendNetBufferLists;
MINIPORT_RETURN_NET_BUFFER_LISTS           miniportReturnNetBufferLists;
MINIPORT_CANCEL_SEND                       miniportCancelSendNetBufferLists;
MINIPORT_DEVICE_PNP_EVENT_NOTIFY           miniportPnpEventNotify;
MINIPORT_SHUTDOWN                          miniportShutdown;
MINIPORT_CANCEL_OID_REQUEST                miniportCancelOidRequest;

// Protocol driver prototypes
PROTOCOL_SET_OPTIONS                       protocolSetOptions;
PROTOCOL_OPEN_ADAPTER_COMPLETE_EX          protocolOpenAdapterComplete;
PROTOCOL_CLOSE_ADAPTER_COMPLETE_EX         protocolCloseAdapterComplete;
PROTOCOL_OID_REQUEST_COMPLETE              protocolRequestComplete;
PROTOCOL_STATUS_EX                         protocolStatus;
PROTOCOL_BIND_ADAPTER_EX                   protocolBindAdapter;
PROTOCOL_UNBIND_ADAPTER_EX                 protocolUnbindAdapter;
PROTOCOL_NET_PNP_EVENT                     protocolPnpHandler;
PROTOCOL_RECEIVE_NET_BUFFER_LISTS          protocolReceiveNbl;
PROTOCOL_SEND_NET_BUFFER_LISTS_COMPLETE    protocolSendNblComplete;

// Global prototypes
void        protocol_freeVEthInstance(tVEthInstance* pVEthInstance_p);
BOOLEAN     protocol_checkBindingState(void);
void        protocol_setBindingState(ULONG state_p);
void        protocol_registerTxRxHandler(tNdisTransmitCompleteCb pfnTxCallback_p,
                                         tNdisReceiveCb pfnRxCallback_p);
NDIS_STATUS protocol_allocateTxRxBuf(ULONG txBufCount_p, ULONG rxBufCount_p);
void        protocol_freeTxRxBuffers(void);
tTxBufInfo* protocol_getTxBuff(size_t size_p);
void        protocol_freeTxBuff(PVOID pTxLink_p);
NDIS_STATUS protocol_sendOidRequest(NDIS_REQUEST_TYPE requestType_p, NDIS_OID oid_p,
                                    PVOID oidReqBuffer_p, ULONG oidReqBufferLength_p);
NDIS_STATUS protocol_sendPacket(void* pToken_p, size_t size_p, void* pTxLink_p);
UCHAR*      protocol_getCurrentMac(void);
void        protocol_registerVEthHandler(tVEthSendCb pfnTxCallback_p);
NDIS_STATUS miniport_handleReceive(UINT8* pDataBuff_p, size_t size_p);

#ifdef __cplusplus
}
#endif

#endif /* _INC_ndis_imInternal_H_ */
