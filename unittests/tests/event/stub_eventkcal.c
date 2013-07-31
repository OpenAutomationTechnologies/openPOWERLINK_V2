/**
********************************************************************************
\file   stub_eventkcal.c

\brief  Stub file for kernel event CAL module

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2012, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

#include <kernel/eventk.h>
#include <kernel/eventkcal.h>

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
\brief    Initialize kernel event CAL module

The function initializes the kernel event CAL module. Depending on the
configuration it gets the function pointer interface of the used queue
implementations and calls the appropriate init functions.

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          If function executes correctly
\retval other error codes       If an error occurred
*/
//------------------------------------------------------------------------------
tEplKernel eventkcal_init (void)
{
    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief    Cleanup kernel event CAL module

The function cleans up the kernel event CAL module. For cleanup it calls the exit
functions of the queue implementations for each used queue.

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          If function executes correctly
\retval other error codes       If an error occurred
*/
//------------------------------------------------------------------------------
tEplKernel eventkcal_exit (void)
{
    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief    Post kernel event

This function posts a event to a queue. It is called from the generic kernel
event post function in the event handler. Depending on the sink the appropriate
queue post function is called.

\param  pEvent_p                Event to be posted.

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          If function executes correctly
\retval other error codes       If an error occurred
*/
//------------------------------------------------------------------------------
tEplKernel eventkcal_postEvent (tEplEvent *pEvent_p)
{
    tEplKernel      ret = kEplSuccessful;

    // split event post to user internal and user to kernel
    switch(pEvent_p->m_EventSink)
    {
        // user layer modules
        case kEplEventSinkNmtMnu:
        case kEplEventSinkNmtu:
        case kEplEventSinkSdoAsySeq:
        case kEplEventSinkApi:
        case kEplEventSinkDlluCal:
        case kEplEventSinkErru:
        case kEplEventSinkLedu:
            ret = kEplSuccessful;
            break;

        // kernel layer modules
        case kEplEventSinkSync:
        case kEplEventSinkNmtk:
        case kEplEventSinkDllk:
        case kEplEventSinkDllkCal:
        case kEplEventSinkPdok:
        case kEplEventSinkPdokCal:
        case kEplEventSinkErrk:
            ret = kEplSuccessful;
            break;

        default:
            ret = kEplEventUnknownSink;
            break;
    }


    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Kernel event CAL receive handler

This is the event receive function for events posted to the kernel layer.

\param  pEvent_p                Received event to be processed.

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          If function executes correctly
\retval other error codes       If an error occurred
*/
//------------------------------------------------------------------------------
tEplKernel eventkcal_rxHandler (tEplEvent *pEvent_p)
{
    tEplKernel ret = kEplSuccessful;

    ret = eventk_process(pEvent_p);

    return ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//

