/**
********************************************************************************
\file   tests.c

\brief  Unit test functions for event module

This file contains the unit test functions for the event module.

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
#include <CUnit/CUnit.h>

#include <kernel/eventk.h>
#include "common/event/event.h"

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
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError processHandler1(tEvent* pEvent_p);
static tOplkError processHandler2(tEvent* pEvent_p);

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tEventDispatchEntry tstEventDispatchTbl_l[] =
{
    { kEventSinkNmtu,        kEventSourceNmtu,        processHandler1 },
    { kEventSinkNmtu,        kEventSourceNmtMnu,      processHandler2 },
    { kEventSinkNmtMnu,      kEventSourceNmtMnu,      processHandler2 },
    { kEventSinkInvalid,     kEventSourceInvalid,     NULL }
};

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Test event_getHandlerForSink() with existing entry
*/
//------------------------------------------------------------------------------
void test_getHandlerForSink_FirstExist(void)
{
    tOplkError              ret = kErrorIllegalInstance;
    tEventSource            eventSource = kEventSourceInvalid;
    tProcessEventCb         pfnEventHandler = NULL;
    tEventDispatchEntry*    pDispatchEntry;

    /* test search of existing entry */
    pDispatchEntry = &tstEventDispatchTbl_l[0];
    ret = event_getHandlerForSink(&pDispatchEntry, kEventSinkNmtu,
                                  &pfnEventHandler, &eventSource);

    CU_ASSERT_EQUAL(ret, kErrorOk);
    CU_ASSERT_EQUAL(pfnEventHandler, processHandler1);
    CU_ASSERT_EQUAL(eventSource, kEventSourceNmtu);
    CU_ASSERT_EQUAL(pDispatchEntry, &tstEventDispatchTbl_l[1]);
}


//------------------------------------------------------------------------------
/**
\brief  Test event_getHandlerForSink() with further existing entry
*/
//------------------------------------------------------------------------------
void test_getHandlerForSink_FurtherExist(void)
{
    tOplkError              ret = kErrorIllegalInstance;
    tEventSource            eventSource = kEventSourceInvalid;
    tProcessEventCb         pfnEventHandler = NULL;
    tEventDispatchEntry*    pDispatchEntry;

    /* test search of existing entry */
    pDispatchEntry = &tstEventDispatchTbl_l[0];
    event_getHandlerForSink(&pDispatchEntry, kEventSinkNmtu,
                                  &pfnEventHandler, &eventSource);

    ret = event_getHandlerForSink(&pDispatchEntry, kEventSinkNmtu,
                                  &pfnEventHandler, &eventSource);

    CU_ASSERT_EQUAL(ret, kErrorOk);
    CU_ASSERT_EQUAL(pfnEventHandler, processHandler2);
    CU_ASSERT_EQUAL(eventSource, kEventSourceNmtMnu);
    CU_ASSERT_EQUAL(pDispatchEntry, &tstEventDispatchTbl_l[2]);
}

//------------------------------------------------------------------------------
/**
\brief  Test event_getHandlerForSink() with not existing entry
*/
//------------------------------------------------------------------------------
void test_getHandlerForSink_NotExist(void)
{
    tOplkError              ret = kErrorIllegalInstance;
    tEventSource            eventSource = kEventSourceInvalid;
    tProcessEventCb         pfnEventHandler = NULL;
    tEventDispatchEntry*    pDispatchEntry;

    /* test search of existing entry */
    pDispatchEntry = &tstEventDispatchTbl_l[0];
    ret = event_getHandlerForSink(&pDispatchEntry, kEventSinkDllk,
                                  &pfnEventHandler, &eventSource);

    CU_ASSERT_EQUAL(ret, kErrorEventUnknownSink);
    CU_ASSERT_EQUAL(pfnEventHandler, NULL);
    CU_ASSERT_EQUAL(eventSource, kEventSourceInvalid);
}


//------------------------------------------------------------------------------
/**
\brief  Test eventk_process()
*/
//------------------------------------------------------------------------------
void test_eventk_process(void)
{
    tEvent          event;

    event.eventSink = kEventSinkDllk;
    CU_ASSERT_EQUAL(eventk_process(&event), kErrorOk);

    event.eventSink = kEventSinkDllkCal;
    CU_ASSERT_EQUAL(eventk_process(&event), kErrorOk);

    event.eventSink = kEventSinkNmtk;
    CU_ASSERT_EQUAL(eventk_process(&event), kErrorOk);

    event.eventSink = kEventSinkErrk;
    CU_ASSERT_EQUAL(eventk_process(&event), kErrorOk);

    event.eventSink = kEventSinkPdokCal;
    CU_ASSERT_EQUAL(eventk_process(&event), kErrorOk);

    event.eventSink = kEventSinkPdok;
    CU_ASSERT_EQUAL(eventk_process(&event), kErrorEventUnknownSink);

}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Dummy process handler function
*/
//------------------------------------------------------------------------------
static tOplkError processHandler1(tEvent* pEvent_p)
{
    UNUSED_PARAMETER(pEvent_p);
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Dummy process handler function
*/
//------------------------------------------------------------------------------
static tOplkError processHandler2(tEvent* pEvent_p)
{
    UNUSED_PARAMETER(pEvent_p);
    return kErrorOk;
}








