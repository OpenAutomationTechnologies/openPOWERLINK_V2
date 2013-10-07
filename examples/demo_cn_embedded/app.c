/**
********************************************************************************
\file   app.c

\brief  Demo CN application which implements an digital input/output node

This file contains a demo application for digital input/output data.

\ingroup module_demo_cn_embedded
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2013, SYSTEC electronic GmbH
Copyright (c) 2013, Kalycito Infotech Private Ltd.
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
#include <Epl.h>
#include <EplTarget.h>

#include <gpio.h>
#include "app.h"
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
/* structure for input process image */
typedef struct
{
   BYTE    digitalIn[4];
} PI_IN;

/* structure for output process image */
typedef struct
{
   BYTE    digitalOut[4];
} PI_OUT;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
/* process image */
static PI_IN*   pProcessImageIn_l;
static PI_OUT*  pProcessImageOut_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tEplKernel initProcessImage(void);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize the synchronous data application

The function initializes the synchronous data application

\return The function returns a tEplKernel error code.

\ingroup module_demo_cn_embedded
*/
//------------------------------------------------------------------------------
tEplKernel initApp(void)
{
    tEplKernel ret = kEplSuccessful;

    ret = initProcessImage();

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Shutdown the synchronous data application

The function shut's down the synchronous data application

\return The function returns a tEplKernel error code.

\ingroup module_demo_cn_embedded
*/
//------------------------------------------------------------------------------
void shutdownApp (void)
{
    oplk_freeProcessImage();
}

//------------------------------------------------------------------------------
/**
\brief  Synchronous data handler

The function implements the synchronous data handler.

\return The function returns a tEplKernel error code.

\ingroup module_demo_cn_embedded
*/
//------------------------------------------------------------------------------
tEplKernel processSync(void)
{
    tEplKernel      ret = kEplSuccessful;
    UINT32          appOutVal;
    UINT8           appInVal;

    ret = oplk_exchangeProcessImageOut();
    if (ret != kEplSuccessful)
        return ret;

    /* read input image - digital outputs */
    appOutVal = pProcessImageOut_l->digitalOut[0] << 0  |
                pProcessImageOut_l->digitalOut[1] << 8  |
                pProcessImageOut_l->digitalOut[2] << 16 |
                pProcessImageOut_l->digitalOut[3] << 24 ;

    gpio_setAppOutputs(appOutVal);

    /* setup output image - digital inputs */
    appInVal = gpio_getAppInput();

    pProcessImageIn_l->digitalIn[0] = appInVal;
    pProcessImageIn_l->digitalIn[1] = appInVal;
    pProcessImageIn_l->digitalIn[2] = appInVal;
    pProcessImageIn_l->digitalIn[3] = appInVal;

    ret = oplk_exchangeProcessImageIn();

    return ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Initialize process image

The function initializes the process image of the application.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel initProcessImage(void)
{
    tEplKernel      ret = kEplSuccessful;
    UINT            varEntries;
    tObdSize        obdSize;

    /* Allocate process image */
    PRINTF("Initializing process image...\n");
    PRINTF("Size of input process image: %ld\n", sizeof(PI_IN));
    PRINTF("Size of output process image: %ld\n", sizeof (PI_OUT));
    ret = oplk_allocProcessImage(sizeof(PI_IN), sizeof(PI_OUT));
    if (ret != kEplSuccessful)
    {
        return ret;
    }

    pProcessImageIn_l = oplk_getProcessImageIn();
    pProcessImageOut_l = oplk_getProcessImageOut();

    /* link process variables used by CN to object dictionary */
    PRINTF("Linking process image vars:\n");

    obdSize = sizeof(pProcessImageIn_l->digitalIn[0]);
    varEntries = 4;
    ret = oplk_linkProcessImageObject(0x6000, 0x01, offsetof(PI_IN, digitalIn),
                                     FALSE, obdSize, &varEntries);
    if (ret != kEplSuccessful)
    {
        PRINTF("linking process vars ... error %04x\n\n", ret);
        return ret;
    }

    obdSize = sizeof(pProcessImageOut_l->digitalOut[0]);
    varEntries = 4;
    ret = oplk_linkProcessImageObject(0x6200, 0x01, offsetof(PI_OUT, digitalOut),
                                     TRUE, obdSize, &varEntries);
    if (ret != kEplSuccessful)
    {
        PRINTF("linking process vars ... error %04x\n\n", ret);
        return ret;
    }

    PRINTF("Linking process vars... ok\n\n");

    return kEplSuccessful;
}

///\}
