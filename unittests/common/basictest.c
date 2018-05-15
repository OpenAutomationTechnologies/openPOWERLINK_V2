/**
********************************************************************************
\file   basictest.c

\brief  Main function for the basic mode unit test execution

This file contains the main function to call the unit tests of a module in
the basic mode.

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2012, B&R Industrial Automation GmbH
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
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <CUnit/Basic.h>

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
extern CU_pSuiteInfo test_getSuiteInfo(void);

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
CU_ErrorCode addTests(void);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

/**
 * Test program entry point. Calls CUnit initialization routines, adds all tests to the
 * CUnit registry and executes them.
 */
//------------------------------------------------------------------------------
/**
\brief  Main function for unit tests

This function is the main function of the unit test executable.

\param  argc            Number of arguments (not used)
\param  argv            Pointer to arguments (not used)

\return CUnit error code
*/
//------------------------------------------------------------------------------
int main(int argc __attribute__((unused)), char **argv __attribute__((unused)))
{
    // set test mode
    CU_basic_set_mode(CU_BRM_VERBOSE);
    // exit on errors in the framework
    CU_set_error_action(CUEA_ABORT);

    if(CU_initialize_registry() != CUE_SUCCESS)
    {
        printf("\nInitialization of test registry failed");
        return CU_get_error();
    }

    // Add all tests
    if (addTests() != CUE_SUCCESS)
    {
        CU_cleanup_registry();
        return CU_get_error();
    }

    // execute tests
    if(CU_basic_run_tests() != CUE_SUCCESS)
    {
        printf("\n Test run failed");
    }

    // cleanup CUnit registry
    CU_cleanup_registry();
    return CU_get_error();
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Add  CUnit test suites

This function adds the test suites for the unit test to the CUnit registry.
It calls test_getSuiteInfo() of the unit test module to get the appropriate
testsuite information.

\return Returns a CUnit error code
*/
//------------------------------------------------------------------------------
CU_ErrorCode addTests(void)
{
    CU_ErrorCode    ret;

    assert(NULL != CU_get_registry());
    assert(!CU_is_test_running());

    /* Register suites. */
    if ((ret = CU_register_suites(test_getSuiteInfo())) != CUE_SUCCESS)
    {
        fprintf(stderr, "suite registration failed - %s\n", CU_get_error_msg());
    }
    return ret;
}




