/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  source file for EPL API module (process image)

  License:

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.

    3. Neither the name of SYSTEC electronic GmbH nor the names of its
       contributors may be used to endorse or promote products derived
       from this software without prior written permission. For written
       permission, please contact info@systec-electronic.com.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    Severability Clause:

        If a provision of this License is or becomes illegal, invalid or
        unenforceable in any jurisdiction, that shall not affect:
        1. the validity or enforceability in that jurisdiction of any other
           provision of this License; or
        2. the validity or enforceability in other jurisdictions of that or
           any other provision of this License.

  -------------------------------------------------------------------------

                $RCSfile$

                $Author$

                $Revision$  $Date$

                $State$

                Build Environment:
                    GCC V3.4

  -------------------------------------------------------------------------

  Revision History:

  2006/10/10 d.k.:   start of the implementation, version 1.00

****************************************************************************/

#include "Epl.h"

#if (TARGET_SYSTEM == _LINUX_) && defined(__KERNEL__)
#include <asm/uaccess.h>
#endif


/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          G L O B A L   D E F I N I T I O N S                            */
/*                                                                         */
/*                                                                         */
/***************************************************************************/

//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// module global vars
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------


/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          C L A S S  EplApi                                              */
/*                                                                         */
/*                                                                         */
/***************************************************************************/
//
// Description:
//
//
/***************************************************************************/


//=========================================================================//
//                                                                         //
//          P R I V A T E   D E F I N I T I O N S                          //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

#if ((EPL_API_PROCESS_IMAGE_SIZE_IN > 0) || (EPL_API_PROCESS_IMAGE_SIZE_OUT > 0))
    typedef struct
    {
        #if EPL_API_PROCESS_IMAGE_SIZE_IN > 0
            BYTE        m_abProcessImageInput[EPL_API_PROCESS_IMAGE_SIZE_IN];
        #endif
        #if EPL_API_PROCESS_IMAGE_SIZE_OUT > 0
            BYTE        m_abProcessImageOutput[EPL_API_PROCESS_IMAGE_SIZE_OUT];
        #endif

    } tEplApiProcessImageInstance;

//---------------------------------------------------------------------------
// local vars
//---------------------------------------------------------------------------

static tEplApiProcessImageInstance  EplApiProcessImageInstance_g;
#endif


BYTE    abDomain_l[3000];

//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------


//=========================================================================//
//                                                                         //
//          P U B L I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//


//---------------------------------------------------------------------------
//
// Function:    EplApiProcessImageSetup()
//
// Description: sets up static process image
//
// Parameters:  (none)
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplApiProcessImageSetup(void)
{
tEplKernel      Ret = kEplSuccessful;
#if ((EPL_API_PROCESS_IMAGE_SIZE_IN > 0) || (EPL_API_PROCESS_IMAGE_SIZE_OUT > 0))
unsigned int    uiVarEntries;
tEplObdSize     ObdSize;
#endif

#if EPL_API_PROCESS_IMAGE_SIZE_IN > 0
#if 0
    uiVarEntries = EPL_API_PROCESS_IMAGE_SIZE_IN;
    ObdSize = 1;
    Ret = EplApiLinkObject(
                            0x2000,
                            EplApiProcessImageInstance_g.m_abProcessImageInput,
                            &uiVarEntries,
                            &ObdSize,
                            1);

    uiVarEntries = EPL_API_PROCESS_IMAGE_SIZE_IN;
    ObdSize = 1;
    Ret = EplApiLinkObject(
                            0x2001,
                            EplApiProcessImageInstance_g.m_abProcessImageInput,
                            &uiVarEntries,
                            &ObdSize,
                            1);

    ObdSize = 2;
    uiVarEntries = EPL_API_PROCESS_IMAGE_SIZE_IN / ObdSize;
    Ret = EplApiLinkObject(
                            0x2010,
                            EplApiProcessImageInstance_g.m_abProcessImageInput,
                            &uiVarEntries,
                            &ObdSize,
                            1);

    ObdSize = 2;
    uiVarEntries = EPL_API_PROCESS_IMAGE_SIZE_IN / ObdSize;
    Ret = EplApiLinkObject(
                            0x2011,
                            EplApiProcessImageInstance_g.m_abProcessImageInput,
                            &uiVarEntries,
                            &ObdSize,
                            1);

    ObdSize = 4;
    uiVarEntries = EPL_API_PROCESS_IMAGE_SIZE_IN / ObdSize;
    Ret = EplApiLinkObject(
                            0x2020,
                            EplApiProcessImageInstance_g.m_abProcessImageInput,
                            &uiVarEntries,
                            &ObdSize,
                            1);

    ObdSize = 4;
    uiVarEntries = EPL_API_PROCESS_IMAGE_SIZE_IN / ObdSize;
    Ret = EplApiLinkObject(
                            0x2021,
                            EplApiProcessImageInstance_g.m_abProcessImageInput,
                            &uiVarEntries,
                            &ObdSize,
                            1);

#endif

    // temp link EplMesseDemo process variables to OD

    // link process variables used by CN to object dictionary
    printk("6000 1 ");
    ObdSize = sizeof(EplApiProcessImageInstance_g.m_abProcessImageInput[0]);
    uiVarEntries = 1;
    Ret = EplApiLinkObject(0x6000,
                              &EplApiProcessImageInstance_g.m_abProcessImageInput[0],
                              &uiVarEntries,
                              &ObdSize,
                              0x01);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

    printk("OK\n2200 1 ");
    ObdSize = sizeof(EplApiProcessImageInstance_g.m_abProcessImageInput[1]);
    uiVarEntries = 3;
    Ret = EplApiLinkObject(0x2200,
                              &EplApiProcessImageInstance_g.m_abProcessImageInput[1],
                              &uiVarEntries,
                              &ObdSize,
                              0x01);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

#endif


#if EPL_API_PROCESS_IMAGE_SIZE_OUT > 0
#if 0
    uiVarEntries = EPL_API_PROCESS_IMAGE_SIZE_OUT;
    ObdSize = 1;
    Ret = EplApiLinkObject(
                            0x2030,
                            EplApiProcessImageInstance_g.m_abProcessImageOutput,
                            &uiVarEntries,
                            &ObdSize,
                            1);

    uiVarEntries = EPL_API_PROCESS_IMAGE_SIZE_OUT;
    ObdSize = 1;
    Ret = EplApiLinkObject(
                            0x2031,
                            EplApiProcessImageInstance_g.m_abProcessImageOutput,
                            &uiVarEntries,
                            &ObdSize,
                            1);

    ObdSize = 2;
    uiVarEntries = EPL_API_PROCESS_IMAGE_SIZE_OUT / ObdSize;
    Ret = EplApiLinkObject(
                            0x2040,
                            EplApiProcessImageInstance_g.m_abProcessImageOutput,
                            &uiVarEntries,
                            &ObdSize,
                            1);

    ObdSize = 2;
    uiVarEntries = EPL_API_PROCESS_IMAGE_SIZE_OUT / ObdSize;
    Ret = EplApiLinkObject(
                            0x2041,
                            EplApiProcessImageInstance_g.m_abProcessImageOutput,
                            &uiVarEntries,
                            &ObdSize,
                            1);

    ObdSize = 4;
    uiVarEntries = EPL_API_PROCESS_IMAGE_SIZE_OUT / ObdSize;
    Ret = EplApiLinkObject(
                            0x2050,
                            EplApiProcessImageInstance_g.m_abProcessImageOutput,
                            &uiVarEntries,
                            &ObdSize,
                            1);

    ObdSize = 4;
    uiVarEntries = EPL_API_PROCESS_IMAGE_SIZE_OUT / ObdSize;
    Ret = EplApiLinkObject(
                            0x2051,
                            EplApiProcessImageInstance_g.m_abProcessImageOutput,
                            &uiVarEntries,
                            &ObdSize,
                            1);
#endif

    // temp link EplMesseDemo process variables to OD

    // link process variables used by CN to object dictionary
    printk("OK\n6200 1 ");
    ObdSize = sizeof(EplApiProcessImageInstance_g.m_abProcessImageOutput[0]);
    uiVarEntries = 1;
    Ret = EplApiLinkObject(0x6200,
                              &EplApiProcessImageInstance_g.m_abProcessImageOutput[0],
                              &uiVarEntries,
                              &ObdSize,
                              0x01);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }
    printk("OK\n");

#endif

    printk("2000 1 ");
    // link process variables used by MN to object dictionary
    ObdSize = sizeof(EplApiProcessImageInstance_g.m_abProcessImageOutput[1]);
    uiVarEntries = 1;
    Ret = EplApiLinkObject(0x2000,
                              &EplApiProcessImageInstance_g.m_abProcessImageOutput[1],
                              &uiVarEntries,
                              &ObdSize,
                              0x01);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

    printk("OK\n2000 2 ");
    ObdSize = sizeof(EplApiProcessImageInstance_g.m_abProcessImageOutput[2]);
    uiVarEntries = 1;
    Ret = EplApiLinkObject(0x2000,
                              &EplApiProcessImageInstance_g.m_abProcessImageOutput[2],
                              &uiVarEntries,
                              &ObdSize,
                              0x02);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

    printk("OK\n6100 0 (DOMAIN) ");
    // link a DOMAIN to object 0x6100, but do not exit, if it is missing
    ObdSize = sizeof(abDomain_l);
    uiVarEntries = 1;
    Ret = EplApiLinkObject(0x6100, &abDomain_l, &uiVarEntries, &ObdSize, 0x00);
    if (Ret != kEplSuccessful)
    {
        printk("EplApiDefineObject(0x6100): returns 0x%X\n", Ret);
    }
    printk("OK\n");


Exit:

    return Ret;
}

//----------------------------------------------------------------------------
// Function:    EplApiProcessImageExchangeIn()
//
// Description: replaces passed input process image with the one of EPL stack
//
// Parameters:  pPI_p                   = input process image
//
// Returns:     tEplKernel              = error code
//
// State:
//----------------------------------------------------------------------------

tEplKernel PUBLIC EplApiProcessImageExchangeIn(tEplApiProcessImage* pPI_p)
{
tEplKernel      Ret = kEplSuccessful;

#if EPL_API_PROCESS_IMAGE_SIZE_IN > 0
    #if (TARGET_SYSTEM == _LINUX_) && defined(__KERNEL__)
        copy_to_user(pPI_p->m_pImage,
            EplApiProcessImageInstance_g.m_abProcessImageInput,
            min(pPI_p->m_uiSize, sizeof (EplApiProcessImageInstance_g.m_abProcessImageInput)));
    #else
        EPL_MEMCPY(pPI_p->m_pImage,
            EplApiProcessImageInstance_g.m_abProcessImageInput,
            min(pPI_p->m_uiSize, sizeof (EplApiProcessImageInstance_g.m_abProcessImageInput)));
    #endif
#endif

    return Ret;
}


//----------------------------------------------------------------------------
// Function:    EplApiProcessImageExchangeOut()
//
// Description: copies passed output process image to EPL stack and marks
//              TPDOs as valid.
//
// Parameters:  pPI_p                   = output process image
//
// Returns:     tEplKernel              = error code
//
// State:
//----------------------------------------------------------------------------

tEplKernel PUBLIC EplApiProcessImageExchangeOut(tEplApiProcessImage* pPI_p)
{
tEplKernel      Ret = kEplSuccessful;

#if EPL_API_PROCESS_IMAGE_SIZE_OUT > 0
    #if (TARGET_SYSTEM == _LINUX_) && defined(__KERNEL__)
        copy_from_user(EplApiProcessImageInstance_g.m_abProcessImageOutput,
            pPI_p->m_pImage,
            min(pPI_p->m_uiSize, sizeof (EplApiProcessImageInstance_g.m_abProcessImageOutput)));
    #else
        EPL_MEMCPY(EplApiProcessImageInstance_g.m_abProcessImageOutput,
            pPI_p->m_pImage,
            min(pPI_p->m_uiSize, sizeof (EplApiProcessImageInstance_g.m_abProcessImageOutput)));
    #endif
#endif

    return Ret;
}


//=========================================================================//
//                                                                         //
//          P R I V A T E   F U N C T I O N S                              //
//                                                                         //
//=========================================================================//



// EOF

