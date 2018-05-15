/**
********************************************************************************
\file   hrestimer-ndistimer.c

\brief  High-resolution timer module for Windows kernel-space

This file contains the implementation of the timer module for the stack in
Windows kernel space. The NDIS timer objects are used as the timers. The
minimum resolution achievable by NDIS timer is 1ms.

\ingroup module_hrestimer
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Kalycito Infotech Private Limited
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
------------------------------------------------------------------------------*/

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>
#include <kernel/hrestimer.h>

#include <ndisintermediate/ndis-im.h>
#include <ndis.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define TIMER_COUNT             2            // openPOWERLINK uses a max of two timers
                                             // so we limit the max count. The implementation
                                             // may be extended to handle more timers.
#define TIMER_MIN_VAL_SINGLE    500000       // min 500us
#define TIMER_MIN_VAL_CYCLE     1000000      // min 1000us
#define TIMER_MEM_TAG           'TirH'       // Timer memory tag

#define TIMERHDL_MASK           0x0FFFFFFF
#define TIMERHDL_SHIFT          28
#define HDL_TO_IDX(hdl)         ((hdl >> TIMERHDL_SHIFT) - 1)
#define HDL_INIT(idx)           ((idx + 1) << TIMERHDL_SHIFT)
#define HDL_INC(hdl)            (((hdl + 1) & TIMERHDL_MASK) |\
                                 (hdl & ~TIMERHDL_MASK))

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
/**
\brief  High-resolution timer information structure

The structure contains all necessary information for a high-resolution timer.
*/
typedef struct
{
    tTimerEventArg      eventArg;                   ///< Timer event Argument.
    tTimerkCallback     pfnCallback;                ///< Timer callback.
    NDIS_HANDLE         timerObjHandle;             ///< NDIS Timer object handle.
    LARGE_INTEGER       dueTime;                    ///< Time to expire in 100ns resolution.
    BOOL                fContinuously;              ///< Determines if it is a continuous or one-shot timer.
} tHresTimerInfo;

/**
\brief  High-resolution timer instance

The structure defines a high-resolution timer module instance.
*/
typedef struct
{
    tHresTimerInfo      aTimerInfo[TIMER_COUNT];    ///< Array of timer info structure.
    BOOLEAN             fInitialized;               ///< Flag to identify if module is initialized.
} tHresTimerInstance;
//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tHresTimerInstance   hresTimerInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static NDIS_TIMER_FUNCTION  timerDpc;

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Initialize high-resolution timer module

The function initializes the high-resolution timer module.

\return Returns a tOplkError error code.

\ingroup module_hrestimer
*/
//------------------------------------------------------------------------------
tOplkError hrestimer_init(void)
{
    NDIS_TIMER_CHARACTERISTICS    timerChars;
    NDIS_STATUS                   status = NDIS_STATUS_SUCCESS;
    NDIS_HANDLE                   adapterHandle = ndis_getAdapterHandle();
    UINT                          index;

    OPLK_MEMSET(&hresTimerInstance_l, 0, sizeof(hresTimerInstance_l));

    for (index = 0; index < TIMER_COUNT; index++)
    {
        NdisZeroMemory(&timerChars, sizeof(timerChars));

        {C_ASSERT(NDIS_SIZEOF_TIMER_CHARACTERISTICS_REVISION_1 <= sizeof(timerChars)); }
        timerChars.Header.Type = NDIS_OBJECT_TYPE_TIMER_CHARACTERISTICS;
        timerChars.Header.Size = NDIS_SIZEOF_TIMER_CHARACTERISTICS_REVISION_1;
        timerChars.Header.Revision = NDIS_TIMER_CHARACTERISTICS_REVISION_1;

        timerChars.TimerFunction = timerDpc;
        timerChars.FunctionContext = &hresTimerInstance_l.aTimerInfo[index];
        timerChars.AllocationTag = TIMER_MEM_TAG;

        status = NdisAllocateTimerObject(adapterHandle,
                                         &timerChars,
                                         &hresTimerInstance_l.aTimerInfo[index].timerObjHandle);
        if (status != NDIS_STATUS_SUCCESS)
        {
            DEBUG_LVL_ERROR_TRACE("%s() Timer Creation Failed %x\n", __func__, status);
            return kErrorNoResource;
        }
    }

    ExSetTimerResolution(10000, TRUE);

    hresTimerInstance_l.fInitialized = TRUE;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief    Shut down high-resolution timer module

The function shuts down the high-resolution timer module.

\return Returns a tOplkError error code.

\ingroup module_hrestimer
*/
//------------------------------------------------------------------------------
tOplkError hrestimer_exit(void)
{
    tHresTimerInfo*   pTimerInfo;
    UINT              index;

    if (!hresTimerInstance_l.fInitialized)
        return kErrorOk;

    for (index = 0; index < TIMER_COUNT; index++)
    {
        pTimerInfo = &hresTimerInstance_l.aTimerInfo[index];
        NdisCancelTimerObject(pTimerInfo->timerObjHandle);
        NdisFreeTimerObject(pTimerInfo->timerObjHandle);

        pTimerInfo->dueTime.QuadPart = 0;
        pTimerInfo->eventArg.timerHdl.handle = 0;
        pTimerInfo->pfnCallback = NULL;
    }

    hresTimerInstance_l.fInitialized = FALSE;
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief    Modify a high-resolution timer

The function modifies the timeout of the timer with the specified handle.
If the handle to which the pointer points to is zero, the timer must be created
first. If it is not possible to stop the old timer, this function always assures
that the old timer does not trigger the callback function with the same handle
as the new timer. That means the callback function must check the passed handle
with the one returned by this function. If these are unequal, the call can be
discarded.

\param[in,out]  pTimerHdl_p         Pointer to timer handle.
\param[in]      time_p              Relative timeout in [ns].
\param[in]      pfnCallback_p       Callback function, which is called when timer expires.
                                    (The function is called mutually exclusive with
                                    the Edrv callback functions (Rx and Tx)).
\param[in]      argument_p          User-specific argument.
\param[in]      fContinue_p         If TRUE, the callback function will be called continuously.
                                    Otherwise, it is a one-shot timer.

\return Returns a tOplkError error code.

\ingroup module_hrestimer
*/
//------------------------------------------------------------------------------
tOplkError hrestimer_modifyTimer(tTimerHdl* pTimerHdl_p,
                                 ULONGLONG time_p,
                                 tTimerkCallback pfnCallback_p,
                                 ULONG argument_p,
                                 BOOL fContinue_p)
{
    UINT              index;
    tHresTimerInfo*   pTimerInfo;
    LONGLONG          relTime;

    if (pTimerHdl_p == NULL)
        return kErrorTimerInvalidHandle;

    if (*pTimerHdl_p == 0)
    {
        // search free timer info structure
        pTimerInfo = &hresTimerInstance_l.aTimerInfo[0];
        for (index = 0; index < TIMER_COUNT; index++, pTimerInfo++)
        {
            if (pTimerInfo->eventArg.timerHdl.handle == 0)
                break;
        }

        if (index >= TIMER_COUNT)
            return kErrorTimerNoTimerCreated;

        pTimerInfo->eventArg.timerHdl.handle = HDL_INIT(index);
    }
    else
    {
        index = (UINT)HDL_TO_IDX(*pTimerHdl_p);
        if (index >= TIMER_COUNT)
            return kErrorTimerInvalidHandle;

        pTimerInfo = &hresTimerInstance_l.aTimerInfo[index];
    }

    /* increment timer handle
     * (if timer expires right after this statement, the user
     * would detect an unknown timer handle and discard it) */
    pTimerInfo->eventArg.timerHdl.handle = HDL_INC(pTimerInfo->eventArg.timerHdl.handle);
    *pTimerHdl_p = pTimerInfo->eventArg.timerHdl.handle;

    // increase too small time values
    if (fContinue_p != FALSE)
    {
        if (time_p < TIMER_MIN_VAL_CYCLE)
            time_p = TIMER_MIN_VAL_CYCLE;
    }
    else
    {
        if (time_p < TIMER_MIN_VAL_SINGLE)
            time_p = TIMER_MIN_VAL_SINGLE;
    }

    pTimerInfo->eventArg.argument.value = argument_p;
    pTimerInfo->pfnCallback = pfnCallback_p;
    pTimerInfo->fContinuously = fContinue_p;

    relTime = time_p / 100LL;
    if (relTime < 0)
    {
        // Negative value is not allowed
        return kErrorTimerNoTimerCreated;
    }

    pTimerInfo->dueTime.QuadPart = -(relTime);
    NdisSetTimerObject(pTimerInfo->timerObjHandle, pTimerInfo->dueTime, 0, pTimerInfo);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief    Delete a high-resolution timer

The function deletes a created high-resolution timer. The timer is specified
by its timer handle. After deleting, the handle is reset to zero.

\param[in,out]  pTimerHdl_p         Pointer to timer handle.

\return Returns a tOplkError error code.

\ingroup module_hrestimer
*/
//------------------------------------------------------------------------------
tOplkError hrestimer_deleteTimer(tTimerHdl* pTimerHdl_p)
{
    tOplkError        ret = kErrorOk;
    UINT              index;
    tHresTimerInfo*   pTimerInfo;

    if (pTimerHdl_p == NULL)
        return kErrorTimerInvalidHandle;

    if (*pTimerHdl_p == 0)
    {
        // no timer created yet
        return ret;
    }
    else
    {
        index = (UINT)HDL_TO_IDX(*pTimerHdl_p);
        if (index >= TIMER_COUNT)
        {
            // invalid handle
            return kErrorTimerInvalidHandle;
        }

        pTimerInfo = &hresTimerInstance_l.aTimerInfo[index];
        if (pTimerInfo->eventArg.timerHdl.handle != *pTimerHdl_p)
        {
            // invalid handle
            return ret;
        }
    }

    *pTimerHdl_p = 0;
    pTimerInfo->eventArg.timerHdl.handle = 0;
    pTimerInfo->pfnCallback = NULL;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Control external synchronization interrupt

This function enables/disables the external synchronization interrupt. If the
external synchronization interrupt is not supported, the call is ignored.

\param[in]      fEnable_p           Flag determines if sync should be enabled or disabled.

\ingroup module_hrestimer
*/
//------------------------------------------------------------------------------
void hrestimer_controlExtSyncIrq(BOOL fEnable_p)
{
    UNUSED_PARAMETER(fEnable_p);
}

//------------------------------------------------------------------------------
/**
\brief  Set external synchronization interrupt time

This function sets the time when the external synchronization interrupt shall
be triggered to synchronize the host processor. If the external synchronization
interrupt is not supported, the call is ignored.

\param[in]      time_p              Time when the sync shall be triggered

\ingroup module_hrestimer
*/
//------------------------------------------------------------------------------
void hrestimer_setExtSyncIrqTime(tTimestamp time_p)
{
    UNUSED_PARAMETER(time_p);
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief    Deferred procedure callback routine for timer

This is the common callback routine for the timers. The NDIS framework will
call this routine once the timer dueTime has expired.

\param[in]      pSystemParameter1_p System parameter 1.
\param[in]      pFunctionContext_p  Pointer to context memory for the timer.
\param[in]      pSystemParameter2_p System parameter 2.
\param[in]      pSystemParameter3_p System parameter 3.

*/
//------------------------------------------------------------------------------
static void timerDpc(PVOID pSystemParameter1_p,
                     PVOID pFunctionContext_p,
                     PVOID pSystemParameter2_p,
                     PVOID pSystemParameter3_p)
{
    tHresTimerInfo*   pTimerInfo = (tHresTimerInfo*)pFunctionContext_p;
    tTimerHdl         orgTimerHdl;
    UINT              index;

    UNREFERENCED_PARAMETER(pSystemParameter1_p);
    UNREFERENCED_PARAMETER(pSystemParameter2_p);
    UNREFERENCED_PARAMETER(pSystemParameter3_p);

    index = (UINT)HDL_TO_IDX(pTimerInfo->eventArg.timerHdl.handle);

    if (index >= TIMER_COUNT)
        return;      // invalid handle

    orgTimerHdl = pTimerInfo->eventArg.timerHdl.handle;

    if (pTimerInfo->pfnCallback != NULL)
        pTimerInfo->pfnCallback(&pTimerInfo->eventArg);

    if (orgTimerHdl != pTimerInfo->eventArg.timerHdl.handle)
    {
        /* modified timer has already been restarted */
        return;
    }

    if (pTimerInfo->fContinuously)
    {
        NdisSetTimerObject(pTimerInfo->timerObjHandle, pTimerInfo->dueTime, 0, pTimerInfo);
    }
}

/// \}
