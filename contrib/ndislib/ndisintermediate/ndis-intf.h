/**
********************************************************************************
\file   ndis-intf.h

\brief  NDIS intermediate driver interface header

This file contains the set of routines exported from the NDIS intermediate driver.
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

#ifndef _INC_ndis_intf_H_
#define _INC_ndis_intf_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <ndis.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

/**
\brief Bind states of Miniport adapter

This enumeration defines the binding states that the miniport adapter can enter
during execution.

*/
typedef enum
{
    kNdisBindingPaused,              ///< Lower end binding is in paused state.
    kNdisBindingPausing,             ///< Lower end binding is entering into paused state.
    kNdisBindingReady,               ///< Lower end binding is ready to run.
    kNdisBindingRunning              ///< Lower end binding is running.
} eNdisBindingState;

/**
\brief Data type for miniport adapter bind states

Data type for the enumerator \ref eNdisBindingState.

*/
typedef UINT8 tNdisBindingState;

/**
\brief Error codes for NDIS drivers

This enumeration lists the error codes for NDIS drivers.

*/
typedef enum
{
    kNdisStatusSuccess,              ///< No error, successful run.
    kNdisStatusInitFailed,           ///< Error in initialization.
    kNdisStatusNoResource,           ///< Resources not available to complete request.
    kNdisStatusTxError,              ///< Error in transmit path.
    kNdisStatusRxError,              ///< Error in receive path.
    kNdisStatusInvalidParams         ///< Invalid parameters to complete request.
} eNdisErrorStatus;

/**
\brief Data type for NDIS drivers error codes

Data type for the enumerator \ref eNdisErrorStatus.

*/
typedef UINT8 tNdisErrorStatus;

/**
\brief Registration handler for driver interface

\param driverHandle_p   Handle of the calling Miniport driver for which
                        the interface routine is registered.

*/
typedef void (*tDrvIntfRegister)(NDIS_HANDLE driverHandle_p);

/**
\brief Deregistration handler for driver interface

*/
typedef void (*tDrvIntfDeregister)(void);

/**
\brief Callback routine to handle packet receive

\param pRxData_p    Pointer to the receive buffer.
\param size_p       Size of the receive buffer.

*/
typedef void (*tNdisReceiveCb)(void* pRxData_p, size_t size_p);

/**
\brief Callback routine to handle transmit completion

\param pTxBuffer_p      Pointer to the transmit buffer.

*/
typedef void(*tNdisTransmitCompleteCb)(void* pTxBuffer_p);

/**
\brief Callback routine to handle send request for VEth interface

\param pVEthTxData_p    Pointer to the VEth send buffer.
\param size_p           Size of the transmit buffer.

*/
typedef void (*tVEthSendCb)(void* pVEthTxData_p, size_t size_p);

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C"
{
#endif

NDIS_STATUS      ndis_initDriver(PDRIVER_OBJECT pDriverObject_p, PUNICODE_STRING pRegistryPath_p);
void             ndis_registerDrvIntf(tDrvIntfRegister pDrvIntfRegCb_p,
                                      tDrvIntfDeregister pDrvIntfDeregCb_p);
BOOLEAN          ndis_checkBindingState(void);
void             ndis_setBindingState(ULONG state_p);
tNdisErrorStatus ndis_allocateTxRxBuff(UINT txBuffCount_p, UINT rxBuffCount_p);
void             ndis_freeTxRxBuff(void);
tNdisErrorStatus ndis_getTxBuff(void** ppData_p, size_t size_p, void** ppTxLink_p);
void             ndis_freeTxBuff(void* pTxLink_p);
tNdisErrorStatus ndis_sendPacket(void* pData_p, size_t size_p, void* pTxLink_p);
void             ndis_registerTxRxHandler(tNdisTransmitCompleteCb pfnTxCallback_p,
                                          tNdisReceiveCb pfnRxCallback_p);
void             ndis_createDrvIntf(void);
void             ndis_closeDrvIntf(void);
NDIS_HANDLE      ndis_getAdapterHandle(void);
void             ndis_getMacAddress(UCHAR*  pMac_p);
void             ndis_registerVethHandler(tVEthSendCb pfnVEthTxCallback_p);
tNdisErrorStatus ndis_vethReceive(void* pData_p, size_t size_p);

#ifdef __cplusplus
}
#endif

#endif /* _INC_ndis-intf_H_ */
