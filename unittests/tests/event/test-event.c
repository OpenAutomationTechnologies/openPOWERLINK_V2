/**
********************************************************************************
\file   test-event.c

\brief  Unit test suite for unit test of event module

This file contains the basic functions for the unit tests of the event module.

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
#include <stddef.h>
#include <CUnit/CUnit.h>
#include "test-event.h"

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
static int eventTestsInit(void);
static int eventTestsCleanup(void);

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

static CU_TestInfo eventTests[] = {
    { "Test event_getHandlerForSink() with existing entry",             test_getHandlerForSink_FirstExist },
    { "Test event_getHandlerForSink() with further existing entry",     test_getHandlerForSink_FurtherExist },
    { "Test event_getHandlerForSink() with not existing entry",         test_getHandlerForSink_NotExist },
    { "Test eventk_process()",                                          test_eventk_process },
    CU_TEST_INFO_NULL,
};

static CU_SuiteInfo suites[] = {
    { "Event Test Suite",       eventTestsInit,         eventTestsCleanup,      eventTests },
    CU_SUITE_INFO_NULL,
};

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Get testsuite info pointer

The function returns a pointer to the testsuite of this unit test.

\return Pointer to testsuite info
*/
//------------------------------------------------------------------------------
CU_pSuiteInfo test_getSuiteInfo(void)
{
    return &suites[0];
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//


//------------------------------------------------------------------------------
/**
\brief  Init function of testsuite

The function does all initializations needed for the tests in this testsuite.

\return Returns an status code
*/
//------------------------------------------------------------------------------
static int eventTestsInit(void)
{
    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Cleanup function of testsuite

The function does all cleanups needed for the tests in this testsuite.

\return Returns an status code
*/
//------------------------------------------------------------------------------
static int eventTestsCleanup(void)
{
    return 0;
}



