/**
********************************************************************************
\file   system-zynqarm.c

\brief  System specific functions for Zynq ARM

The file implements the system specific functions for Zynq ARM used by the
openPOWERLINK demo applications.

\ingroup module_app_common
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

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <unistd.h>
#include "system.h"

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define TGTCONIO_MS_IN_US(x)    (x * 1000U)

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
\brief  Initialize system

The function initializes important stuff on the system for openPOWERLINK to
work correctly.

\return The function returns 0 if the initialization has been successful,
        otherwise -1.

\ingroup module_app_common
*/
//------------------------------------------------------------------------------
int system_init(void)
{
    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Shutdown system

The function shuts down the system.

\ingroup module_app_common
*/
//------------------------------------------------------------------------------
void system_exit(void)
{

}

/**
\brief  Determines whether a termination signal has been received

The function can be used by the application to react on termination request.
On Zynq ARM, this function is only implemented as a stub.


\ingroup module_app_common
*/
//------------------------------------------------------------------------------
BOOL system_getTermSignalState(void)
{
    return FALSE;
}

//------------------------------------------------------------------------------
/**
\brief Sleep for the specified number of milliseconds

The function makes the calling thread sleep until the number of specified
milliseconds have elapsed.

\param  milliSeconds_p      Number of milliseconds to sleep

\ingroup module_app_common
*/
//------------------------------------------------------------------------------
void system_msleep(unsigned int milliSeconds_p)
{
    usleep(TGTCONIO_MS_IN_US(milliSeconds_p));
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
