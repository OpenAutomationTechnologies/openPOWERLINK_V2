/**
********************************************************************************
\file   windows/target-mutex.c

\brief  Architecture specific mutex implementation

This file contains the mutex implementation for Windows. It uses
Windows mutexes for the implementation and can therefore synchronize different
threads or processes.

\ingroup module_target
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <common/target.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

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

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Create Mutex

The function creates a mutex.

\param[in]      mutexName_p         The name of the mutex to create.
\param[out]     pMutex_p            Pointer to store the created mutex.

\return The function returns a tOplkError error code.
\retval kErrorOk                    Mutex was successfully created.
\retval kErrorNoFreeInstance        An error occurred while creating the mutex.

\ingroup module_target
*/
//------------------------------------------------------------------------------
tOplkError target_createMutex(const char* mutexName_p,
                              OPLK_MUTEX_T* pMutex_p)
{
    HANDLE  hMutex;

    hMutex = CreateMutex(NULL, FALSE, mutexName_p);
    if (hMutex == NULL)
        return kErrorNoFreeInstance;

    *pMutex_p = hMutex;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Destroy Mutex

The function destroys a mutex.

\param[in]      mutexId_p           The ID of the mutex to destroy.

\ingroup module_target
*/
//------------------------------------------------------------------------------
void target_destroyMutex(OPLK_MUTEX_T mutexId_p)
{
    CloseHandle(mutexId_p);
}

//------------------------------------------------------------------------------
/**
\brief  Lock Mutex

The function locks a mutex.

\param[in]      mutexId_p           The ID of the mutex to lock.

\return The function returns a tOplkError error code.
\retval kErrorOk                    Mutex was successfully locked.
\retval kErrorNoFreeInstance        An error occurred while locking the mutex.

\ingroup module_target
*/
//------------------------------------------------------------------------------
tOplkError target_lockMutex(OPLK_MUTEX_T mutexId_p)
{
    UINT32      result;
    tOplkError  ret;

    result = WaitForSingleObject(mutexId_p, INFINITE);
    switch (result)
    {
        // The thread got ownership of the mutex
        case WAIT_OBJECT_0:
            ret = kErrorOk;
            break;

        default:
            ret = kErrorIllegalInstance;
            break;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Unlock Mutex

The function unlocks a mutex.

\param[in]      mutexId_p           The ID of the mutex to unlock.

\ingroup module_target
*/
//------------------------------------------------------------------------------
void target_unlockMutex(OPLK_MUTEX_T mutexId_p)
{
    ReleaseMutex(mutexId_p);
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
