/**
********************************************************************************
\file   ndisDriver.h

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

#ifndef _INC_ndisdriver_H_
#define _INC_ndisdriver_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <ndis.h>
#include "ndis-intf.h"

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define OPLK_MEM_TAG           'klpO'
#define OPLK_MAX_FRAME_SIZE    1546

#define OPLK_MTU_SIZE          1500                 ///< Bytes
#define OPLK_LINK_SPEED        100000000            ///< 100 Mbps Rx and Tx
#define OPLK_MAX_BAR_COUNT     6
#define OPLK_ALLOCATED_NBL     0x10000000
#define OPLK_MAX_VETH_BUFF     40

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

/**
\brief NDIS driver instance structure

The structure holds the parameters for the NDIS driver instance.

*/
typedef struct
{
    tDrvIntfRegister      pfnDrvIntfRegCb;          ///< Driver interface register callback.
    tDrvIntfDeregister    pfnDrvIntfDeregisterCb;   ///< Driver interface de-register callback.
    NDIS_HANDLE           pMiniportHandle;          ///< Miniport driver handle returned by OS.
    NDIS_HANDLE           pProtocolHandle;          ///< Protocol driver handle returned by OS.
} tNdisDriverInstance;

/**
\brief PCIe BAR information

The structure holds the information of the PCIe BAR mapped by the NDIS miniport.

*/
typedef struct
{
    PHYSICAL_ADDRESS              phyAddr;          ///< Physical address of the BAR.
    PULONG                        pVirtualAddr;     ///< Virtual address of the BAR in kernel memory.
    ULONG                         length;           ///< Length of the BAR.
} tBarInfo;

/**
\brief VEth receive buffer information

Virtual Ethernet receive buffer information structure for
NDIS driver.

*/
typedef struct
{
    LIST_ENTRY          rxLink;             ///< List entry for the receive buffer.
    PNET_BUFFER_LIST    pNbl;               ///< Pointer to NetBufferList for the buffer.
    PMDL                pMdl;               ///< MDL describing the receive buffer.
    BOOLEAN             free;               ///< Flag to identify buffer access status.
    ULONG               maxLength;          ///< Max length of the buffer.
    ULONG               length;             ///< Length of the receive frame.
    void*               pData;              ///< Pointer to receive data.
} tVEthRxBufInfo;

/**
\brief NDIS miniport driver instance

Structure to hold the global variables used by miniport driver instance.
The miniport acts as the virtual Ethernet interface shared with OS.

*/
typedef struct
{
    NDIS_HANDLE                   miniportAdapterHandle;        ///< Adapter handle for the NDIS miniport.
    NDIS_HANDLE                   interruptHandle;              ///< NDIS handle for interrupts.
    BOOLEAN                       miniportHalting;              ///< Flag to identify miniport halting status.
    BOOLEAN                       miniportPaused;               ///< Flag to identify paused status.
    ULONG                         state;                        ///< Current miniport state.
    NDIS_SPIN_LOCK                miniportLock;                 ///< Global miniport lock.
    NDIS_SPIN_LOCK                pauseLock;                    ///< Lock for miniport pause event.
    NDIS_STATUS                   status;                       ///< Miniport status.
    ULONG                         sendRequests;                 ///< Total send requests handled.
    ULONG                         receiveIndication;            ///< Total receive indications handled.
    tIntrHandler                  pfnIntrCb;                    ///< Function pointer to synchronization callback.
    NDIS_INTERRUPT_TYPE           interruptType;                ///< NDIS interrupt type for the registered interrupt.
    PIO_INTERRUPT_MESSAGE_INFO    intrMsgInfo;                  ///< Message interrupt information.
    tBarInfo                      barInfo[OPLK_MAX_BAR_COUNT];  ///< PCIe BAR information for all available BARs.
    ULONG                         msiVector;                    ///< MSI vector assigned for the miniport.
    tVEthSendCb                   pfnVEthSendCb;                ///< Callback routine for VEth transmit.
    NDIS_HANDLE                   receiveNblPool;               ///< Receive NetBufferLists pool.
    tVEthRxBufInfo*               pReceiveBufInfo;              ///< Pointer to receive buffer information.
    void*                         pReceiveBuf;                  ///< Pointer to the memory for received VEth frame.
    LIST_ENTRY                    rxList;                       ///< List entry for VEth receive queue.
    NDIS_SPIN_LOCK                rxListLock;                   ///< Spin lock for VEth receive queue.
    NDIS_LINK_STATE               lastLinkState;                ///< Last link state change.
} tVEthInstance;

//------------------------------------------------------------------------------
// global defines
//------------------------------------------------------------------------------
extern tNdisDriverInstance                 driverInstance_g;
extern tVEthInstance                       vethInstance_g;

//------------------------------------------------------------------------------
// Macros
//------------------------------------------------------------------------------
#ifndef NDEBUG
#define TRACE(...)    DbgPrint(__VA_ARGS__)
#else
#define TRACE(...)
#endif

#define VETHINFO_FROM_NBL(_NBL)    ((tVEthRxBufInfo*)((_NBL)->MiniportReserved[0]))

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------
#ifdef __cplusplus
extern "C"
{
#endif

// Miniport driver prototypes
DRIVER_DISPATCH                     miniportIoDispatch;
DRIVER_DISPATCH                     miniportDeviceIoControl;
MINIPORT_SET_OPTIONS                miniportSetOptions;
MINIPORT_INITIALIZE                 miniportInitialize;
MINIPORT_HALT                       miniportHalt;
MINIPORT_UNLOAD                     miniportUnload;
MINIPORT_PAUSE                      miniportPause;
MINIPORT_RESTART                    miniportRestart;
MINIPORT_OID_REQUEST                miniportOidRequest;
MINIPORT_SEND_NET_BUFFER_LISTS      miniportSendNetBufferLists;
MINIPORT_RETURN_NET_BUFFER_LISTS    miniportReturnNetBufferLists;
MINIPORT_CANCEL_SEND                miniportCancelSendNetBufferLists;
MINIPORT_DEVICE_PNP_EVENT_NOTIFY    miniportPnpEventNotify;
MINIPORT_SHUTDOWN                   miniportShutdown;
MINIPORT_CANCEL_OID_REQUEST         miniportCancelOidRequest;
MINIPORT_CHECK_FOR_HANG             miniportCheckForHang;
MINIPORT_RESET                      miniportReset;
NDIS_STATUS                         miniport_handleReceive(UINT8* pDataBuff_p, size_t size_p);
void                                miniport_setAdapterState(ULONG state_p);

#ifdef __cplusplus
}
#endif

#endif /* _INC_ndisdriver_H_ */
