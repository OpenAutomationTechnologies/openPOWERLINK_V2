/**
********************************************************************************
\file   sim.h

\brief  Include file for type defines regarding all simulation interfaces

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2016, Franz Profelt (franz.profelt@gmail.com)
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
#ifndef _INC_sim_H_
#define _INC_sim_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/led.h>
#include <kernel/edrv.h>
#include <kernel/hrestimer.h>
#include <user/timeru.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------
/**
 Handle for identification of multiple stack instances within a simulation
 */
typedef UINT32 tSimulationInstanceHdl;

//------------------------------------------------------------------------------
// hrestimer types
//------------------------------------------------------------------------------

/**
\brief Type for HresTimer init and exit function

This type defines a function pointer for the simulation interface functions
\ref sim_initHresTimer and \ref sim_exitHresTimer.

\param[in]      simInstanceHdl_p    The handle of the currently simulated stack instance

\return The function returns a tOplkError error code
*/
typedef tOplkError (*tInitExitHresTimerFunc)(tSimulationInstanceHdl simInstanceHdl_p);

/**
\brief Type for HresTimer modifyTimer function

This type defines a function pointer for the simulation interface function
 \ref sim_modifyHresTimer.

\param[in]      simInstanceHdl_p    The handle of the currently simulated stack instance
\param[in,out]  pTimerHdl_p         Pointer to timer handle
\param[in]      time_p              Relative timeout [ns]
\param[in]      pfnCallback_p       Callback function, which is called when timer expires
                                    (The function is called mutually exclusive with the Edrv
                                    callback functions (Rx and Tx))
\param[in]      argument_p          User-specific argument
\param[in]      fContinue_p         If TRUE, the callback function will be called continuously
                                    Otherwise, it is a one-shot timer

\return The function returns a tOplkError error code
*/
typedef tOplkError (*tModifyHresTimerFunc)(tSimulationInstanceHdl simInstanceHdl_p,
                                           tTimerHdl* pTimerHdl_p,
                                           ULONGLONG time_p,
                                           tTimerkCallback pfnCallback_p,
                                           ULONG argument_p,
                                           BOOL fContinue_p);

/**
\brief Type for HresTimer deleterTimer function

This type defines a function pointer for the simulation interface function
 \ref sim_deleteHresTimer.

\param[in]      simInstanceHdl_p    The handle of the currently simulated stack instance
\param[in,out]  pTimerHdl_p         Pointer to timer handle

\return The function returns a tOplkError error code
*/
typedef tOplkError (*tDeleteHresTimerFunc)(tSimulationInstanceHdl simInstanceHdl_p,
                                           tTimerHdl* pTimerHdl_p);

/**
\brief Structure holding all function pointers for hresTimer

This struct holds all function pointers to the hresTimer functions used in the
 simulation interface (\ref sim-hrestimer.h).
*/
typedef struct
{
    tInitExitHresTimerFunc  pfnInitHresTimer;   ///< Pointer to the initHresTimer function
    tInitExitHresTimerFunc  pfnExitHresTimer;   ///< Pointer to the exitHresTimer function
    tModifyHresTimerFunc    pfnModifyHresTimer; ///< Pointer to the modifyHresTimer function
    tDeleteHresTimerFunc    pfnDeleteHresTimer; ///< Pointer to the deleteHresTimer function
} tHresTimerFunctions;

//------------------------------------------------------------------------------
// target types
//------------------------------------------------------------------------------
/**
\brief Type for targets init and exit function

This type defines a function pointer for the simulation interface functions
 \ref sim_initTarget and \ref sim_exitTarget.

\param[in]      simInstanceHdl_p    The handle of the currently simulated stack instance

\return The function returns a tOplkError error code
*/
typedef tOplkError (*tInitExitTargetFunc)(tSimulationInstanceHdl simInstanceHdl_p);

/**
\brief Type for targets msleep function

This type defines a function pointer for the simulation interface function
 \ref sim_msleep.

\param[in]      simInstanceHdl_p    The handle of the currently simulated stack instance
\param[in]      milliSeconds_p      Number of milliseconds to sleep
*/
typedef void (*tMsleepFunc)(tSimulationInstanceHdl simInstanceHdl_p,
                            UINT32 milliSeconds_p);

//------------------------------------------------------------------------------
/**
\brief Type for targets setIpAdrs function

This type defines a function pointer for the simulation interface function
 for setIpAdrs.

\param[in]      simInstanceHdl_p    The handle of the currently simulated stack instance
\param[in]      ifName_p            Name of Ethernet interface
\param[in]      ipAddress_p         IP address to set for interface
\param[in]      subnetMask_p        Subnet mask to set for interface
\param[in]      mtu_p               MTU to set for interface

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
typedef tOplkError (*tSetIpFunc)(tSimulationInstanceHdl simInstanceHdl_p,
                                 const char* ifName_p,
                                 UINT32 ipAddress_p,
                                 UINT32 subnetMask_p,
                                 UINT16 mtu_p);

//------------------------------------------------------------------------------
/**
\brief Type for targets setDefaultGateway function

This type defines a function pointer for the simulation interface function
 \ref sim_setDefaultGateway.

\param[in]      simInstanceHdl_p    The handle of the currently simulated stack instance
\param[in]      defaultGateway_p    Default gateway to set

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
typedef tOplkError (*tSetDefaultGateWayFunc)(tSimulationInstanceHdl simInstanceHdl_p,
                                             UINT32 defaultGateway_p);

//------------------------------------------------------------------------------
/**
\brief Type for targets getTick function

This type defines a function pointer for the simulation interface functions
 \ref sim_getTickCount.

\param[in]      simInstanceHdl_p    The handle of the currently simulated stack instance

\return The function returns the current tick count.
*/
//------------------------------------------------------------------------------
typedef UINT32 (*tGetTickFunc)(tSimulationInstanceHdl simInstanceHdl_p);

//------------------------------------------------------------------------------
/**
\brief Type for targets setLed function

This type defines a function pointer for the simulation interface function
 \ref sim_setLed.

\param[in]      simInstanceHdl_p    The handle of the currently simulated stack instance
\param[in]      ledType_p           Determines which LED shall be set/reset
\param[in]      fLedOn_p            Set the addressed LED on (TRUE) or off (FALSE)

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
typedef tOplkError (*tSetLedFunc)(tSimulationInstanceHdl simInstanceHdl_p,
                                  tLedType ledType_p,
                                  BOOL fLedOn_p);

/**
\brief Structure holding all function pointers for target

This struct holds all function pointers to the target functions used in the
 simulation interface (\ref sim-target.h).
*/
typedef struct
{
    tInitExitTargetFunc     pfnInit;                ///< Pointer to the initTarget function
    tInitExitTargetFunc     pfnExit;                ///< Pointer to the exitTarget function
    tMsleepFunc             pfnMsleep;              ///< Pointer to the msleep function
    tSetIpFunc              pfnSetIp;               ///< Pointer to the setIp function
    tSetDefaultGateWayFunc  pfnSetDefaultGateway;   ///< Pointer to the setDefaultGateway function
    tGetTickFunc            pfnGetTick;             ///< Pointer to the getTick function
    tSetLedFunc             pfnSetLed;              ///< Pointer to the setLed function
} tTargetFunctions;

//------------------------------------------------------------------------------
// trace types
//------------------------------------------------------------------------------
/**
\brief Type for the simulated trace function

This type defines a function pointer for the simulation interface function
 \ref sim_trace.

\param[in]      simInstanceHdl_p    The handle of the currently simulated stack instance
\param[in]      pMessage_p          Format string
*/
typedef void (*tTraceFunc)(tSimulationInstanceHdl simInstanceHdl_p,
                           const char* pMessage_p);

/**
\brief Structure holding all function pointer for trace

This struct holds all function pointers to the trace functions used in the
 simulation interface (\ref sim-trace.h).
*/
typedef struct
{
    tTraceFunc  pfnTrace;                           ///< Pointer to the trace function
} tTraceFunctions;

//------------------------------------------------------------------------------
// edrv types
//------------------------------------------------------------------------------
/**
\brief Type for initEdrv function pointer

This type defines a function pointer for the simulation interface function
 \ref sim_initEdrv.

\param[in]      simInstanceHdl_p    The handle of the currently simulated stack instance
\param[in]      pEdrvInitParam_p    Pointer to the edrv init parameter

\return The function returns a tOplkError error code
*/
typedef tOplkError (*tInitEdrvFunc)(tSimulationInstanceHdl simInstanceHdl_p,
                                    const tEdrvInitParam* pEdrvInitParam_p);

/**
\brief Type for exitEdrv function pointer

This type defines a function pointer for the simulation interface function
 \ref sim_exitEdrv.

\param[in]      simInstanceHdl_p    The handle of the currently simulated stack instance

\return The function returns a tOplkError error code
*/
typedef tOplkError (*tExitEdrvFunc)(tSimulationInstanceHdl simInstanceHdl_p);

/**
\brief  Type for getMacAddr function pointer

This type defines a function pointer for the simulation interface function
 \ref sim_getMacAddr.

\param[in]      simInstanceHdl_p    The handle of the currently simulated stack instance

\return The function returns a pointer to the MAC address.
*/
typedef const UINT8* (*tGetMacAddrFunc)(tSimulationInstanceHdl simInstanceHdl_p);

/**
\brief   Type for Ethernet txBuffer function pointer

This type defines a function pointer for the simulation interface functions
 \ref sim_allocTxBuffer and \ref sim_freeTxBuffer.

\param[in]      simInstanceHdl_p    The handle of the currently simulated stack instance
\param[in,out]  pBuffer_p           Tx buffer descriptor

\return The function returns a tOplkError error code.
*/
typedef tOplkError (*tTxBufferFunc)(tSimulationInstanceHdl simInstanceHdl_p,
                                    tEdrvTxBuffer* pBuffer_p);

/**
\brief  Type for change RxFilter function pointer

This type defines a function pointer for the simulation interface function
 \ref sim_changeRxFilter.

\param[in]      simInstanceHdl_p    The handle of the currently simulated stack instance
\param[in,out]  pFilter_p           Base pointer of Rx filter array
\param[in]      count_p             Number of Rx filter array entries
\param[in]      entryChanged_p      Index of Rx filter entry that shall be changed
\param[in]      changeFlags_p       Bit mask that selects the changing Rx filter property

\return The function returns a tOplkError error code.
*/
typedef tOplkError (*tChangeRxFilterFunc)(tSimulationInstanceHdl simInstanceHdl_p,
                                          tEdrvFilter* pFilter_p,
                                          UINT count_p,
                                          UINT entryChanged_p,
                                          UINT changeFlags_p);

/**
\brief  Type for Ethernet multicast function pointer

This type defines a function pointer for the simulation interface functions
 \ref sim_setRxMulticastMacAddr and \ref sim_clearRxMulticastMacAddr.

\param[in]      simInstanceHdl_p    The handle of the currently simulated stack instance
\param[in]      pMacAddr_p          Base pointer of Rx filter array

\return The function returns a tOplkError error code.
*/
typedef tOplkError (*tMulticastFunc)(tSimulationInstanceHdl simInstanceHdl_p,
                                     const UINT8* pMacAddr_p);

/**
\brief Edrv function pointers

This struct holds all function pointers to the edrv functions used in the
 simulation interface (\ref sim-edrv.h).
*/
typedef struct
{
    tInitEdrvFunc       pfnInit;                    ///< Pointer to the initEdrv function
    tExitEdrvFunc       pfnExit;                    ///< Pointer to the exitEdrv function
    tGetMacAddrFunc     pfnGetMacAddr;              ///< Pointer to the getMacAddr function
    tTxBufferFunc       pfnSendTxBuffer;            ///< Pointer to the sendTxBuffer function
    tTxBufferFunc       pfnAllocTxBuffer;           ///< Pointer to the allocTxBuffer function
    tTxBufferFunc       pfnFreeTxBuffer;            ///< Pointer to the freeTxBuffer function
    tChangeRxFilterFunc pfnChangeRxFilter;          ///< Pointer to the changeRxBufferFilter function
    tMulticastFunc      pfnSetMulticastMacAddr;     ///< Pointer to the setMulticastMacAddr function
    tMulticastFunc      pfnClearMulticastMacAddr;   ///< Pointer to the clearMulticastMacAddr function
} tEdrvFunctions;

//------------------------------------------------------------------------------
// process sync types
//------------------------------------------------------------------------------
/**
\brief Type for the simulated process sync function

This type defines a function pointer for the simulation interface function
 \ref sim_processSyncCb.

\param[in]      simInstanceHdl_p    The handle of the currently simulated stack instance

\return The function returns a tOplkError error code
*/
typedef tOplkError (*tSimProcessSyncCb)(tSimulationInstanceHdl simInstanceHdl_p);

/**
\brief Process sync function pointer

This struct holds all function pointer to the process sync functions used in the
 simulation interface (\ref sim-processsync.h).
*/
typedef struct
{
    tSimProcessSyncCb   pfnCbProcessSync;           ///< Pointer to the processSync function
} tProcessSyncFunctions;

//------------------------------------------------------------------------------
// api event types
//------------------------------------------------------------------------------
/**
\brief Type for the simulated api event function

This type defines a function pointer for the simulation interface function
 \ref sim_eventCb.

\param[in]      simInstanceHdl_p    The handle of the currently simulated stack instance
\param[in]      eventType_p         The type of the event
\param[in]      pEventArg_p         Pointer to the event argument
\param[in]      pUserArg_p          Pointer to the user defined argument

\return The function returns a tOplkError error code
*/
typedef tOplkError (*tSimEventCb)(tSimulationInstanceHdl simInstanceHdl_p,
                                  tOplkApiEventType eventType_p,
                                  const tOplkApiEventArg* pEventArg_p,
                                  void* pUserArg_p);
/**
\brief Api event function pointer

This struct holds all funtion pointer to the api event functions used in the
 simulation interface (\ref sim-apievent.h).
*/
typedef struct
{
    tSimEventCb pfnCbEvent;                         ///< Pointer to the apiEvent function
} tApiEventFunctions;

//------------------------------------------------------------------------------
// user timer types
//------------------------------------------------------------------------------
/**
\brief Type for the simulated user timer init and exit functions

This type defines a function pointer for the simulation interface functions
 \ref sim_initTimer and \ref sim_exitTimer.

\param[in]      simInstanceHdl_p    The handle of the currently simulated stack instance

\return The function returns a tOplkError error code
*/
typedef tOplkError (*tInitExitTimerFunc)(tSimulationInstanceHdl simInstanceHdl_p);

/**
\brief Type for the simulated user timer setTimer function

This type defines a function pointer for the simulation interface function
 \ref sim_setTimer.

\param[in]      simInstanceHdl_p    The handle of the currently simulated stack instance
\param[out]     pTimerHdl_p         Pointer to store the timer handle.
\param[in]      timeInMs_p          Timeout in milliseconds.
\param[in]      argument_p          User definable argument for timer.

\return The function returns a tOplkError error code
*/
typedef tOplkError (*tSetTimerFunc)(tSimulationInstanceHdl simInstanceHdl_p,
                                    tTimerHdl* pTimerHdl_p,
                                    ULONG timeInMs_p,
                                    tTimerArg argument_p);

/**
\brief Type for the simulated user timer modifyTimer function

This type defines a function pointer for the simulation interface function
 \ref sim_modifyTimer.

\param[in]      simInstanceHdl_p    The handle of the currently simulated stack instance
\param[in,out]  pTimerHdl_p         Pointer to store the timer handle.
\param[in]      timeInMs_p          Timeout in milliseconds.
\param[in]      argument_p          User definable argument for timer.

\return The function returns a tOplkError error code
*/
typedef tOplkError (*tModifyTimerFunc)(tSimulationInstanceHdl simInstanceHdl_p,
                                       tTimerHdl* pTimerHdl_p,
                                       ULONG timeInMs_p,
                                       tTimerArg argument_p);

/**
\brief Type for the simulated user timer deleteTimer function

This type defines a function pointer for the simulation interface function
 \ref sim_deleteTimer.

\param[in]      simInstanceHdl_p    The handle of the currently simulated stack instance
\param[in,out]  pTimerHdl_p         Pointer to timer handle of timer to delete.

\return The function returns a tOplkError error code
*/
typedef tOplkError (*tDeleteTimerFunc)(tSimulationInstanceHdl simInstanceHdl_p,
                                       tTimerHdl* pTimerHdl_p);

/**
\brief Type for the simulated user timer isTimerActive function

This type defines a function pointer for the simulation interface function
 \ref sim_isTimerActive.

\param[in]      simInstanceHdl_p    The handle of the currently simulated stack instance
\param[in]      timerHdl_p          Handle of timer to check.

\return The function returns TRUE if the timer is active, otherwise FALSE.

*/
typedef BOOL (*tIsTimerActiveFunc)(tSimulationInstanceHdl simInstanceHdl_p,
                                   tTimerHdl timerHdl_p);
/**
\brief User timer function pointers

This struct holds all function pointers to the user timer functions used in the
 simulation interface (\ref sim-apievent.h).
*/
typedef struct
{
    tInitExitTimerFunc  pfnInitTimer;       ///< Pointer to the initTimer function
    tInitExitTimerFunc  pfnExitTimer;       ///< Pointer to the exitTimer function
    tSetTimerFunc       pfnSetTimer;        ///< Pointer to the setTimer function
    tModifyTimerFunc    pfnModifyTimer;     ///< Pointer to the modifyTimer function
    tDeleteTimerFunc    pfnDeleteTimer;     ///< Pointer to the deleteTimer function
    tIsTimerActiveFunc  pfnIsTimerActive;   ///< Pointer to the isTimerActive function
} tTimerFunctions;

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------
#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __cplusplus
}
#endif

#endif /* _INC_sim_H_ */
