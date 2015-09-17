/**
********************************************************************************
\file   app.c

\brief  Demo CN application which implements a digital input/output node

This file contains a demo application for digital input/output data.

\ingroup module_demo_cn_console
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <stddef.h>

#include <oplk/oplk.h>
#include <oplk/debugstr.h>

#include <eventlog/eventlog.h>

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
   BYTE    digitalIn;
} PI_IN;

/* structure for output process image */
typedef struct
{
   BYTE    digitalOut;
} PI_OUT;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
/* process image */
static PI_IN*   pProcessImageIn_l;
static PI_OUT*  pProcessImageOut_l;

/* application variables */
static BYTE    digitalIn_g;                 // 8 bit digital input
static BYTE    digitalOut_g;                // 8 bit digital output

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError initProcessImage(void);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize the synchronous data application

The function initializes the synchronous data application

\return The function returns a tOplkError error code.

\ingroup module_demo_cn_console
*/
//------------------------------------------------------------------------------
tOplkError initApp(void)
{
    tOplkError ret = kErrorOk;

    ret = initProcessImage();

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Shutdown the synchronous data application

The function shuts down the synchronous data application

\return The function returns a tOplkError error code.

\ingroup module_demo_cn_console
*/
//------------------------------------------------------------------------------
void shutdownApp(void)
{
    oplk_freeProcessImage();
}

//------------------------------------------------------------------------------
/**
\brief  Synchronous data handler

The function implements the synchronous data handler.

\return The function returns a tOplkError error code.

\ingroup module_demo_cn_console
*/
//------------------------------------------------------------------------------
tOplkError processSync(void)
{
    tOplkError      ret = kErrorOk;

    if (oplk_waitSyncEvent(100000) != kErrorOk)
        return ret;

    ret = oplk_exchangeProcessImageOut();
    if (ret != kErrorOk)
        return ret;

    /* read input image - digital outputs */
    digitalOut_g = pProcessImageOut_l->digitalOut;

    /* setup output image - digital inputs */
    pProcessImageIn_l->digitalIn = digitalIn_g;

    ret = oplk_exchangeProcessImageIn();

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Setup inputs

The function initializes the digital input port.

\ingroup module_demo_cn_console
*/
//------------------------------------------------------------------------------
void setupInputs(void)
{
    digitalIn_g = 1;
}

//------------------------------------------------------------------------------
/**
\brief  Increase inputs

The function changes the digital input port by shifting the set bit to the
left (increase the value).

\ingroup module_demo_cn_console
*/
//------------------------------------------------------------------------------
void increaseInputs(void)
{
    if (digitalIn_g == 128)
        digitalIn_g = 1;
    else
        digitalIn_g = digitalIn_g << 1;
    printf("\b \b");
    printInputs();
}

//------------------------------------------------------------------------------
/**
\brief  Decrease inputs

The function changes the digital input port by shifting the set bit to the
right (decrease the value).

\ingroup module_demo_cn_console
*/
//------------------------------------------------------------------------------
void decreaseInputs(void)
{
    if (digitalIn_g == 1)
        digitalIn_g = 128;
    else
        digitalIn_g = digitalIn_g >> 1;
    printf("\b \b");
    printInputs();
}


//------------------------------------------------------------------------------
/**
\brief  Print outputs

The function prints the value of the digital output port on the console.

\ingroup module_demo_cn_console
*/
//------------------------------------------------------------------------------
void printOutputs(void)
{
    int i;

    printf("\b \b");
    printf("Digital Outputs: ");
    for (i = 0; i < 8; i++)
    {
        if (((digitalOut_g >> i) & 1) == 1)
            printf("*");
        else
            printf("-");
    }
    printf("\n");
}

//------------------------------------------------------------------------------
/**
\brief  Print inputs

The function prints the value of the digital input port on the console.

\ingroup module_demo_cn_console
*/
//------------------------------------------------------------------------------
void printInputs(void)
{
    int i;

    printf("Digital Inputs: ");
    for (i = 0; i < 8; i++)
    {
        if (((digitalIn_g >> i) & 1) == 1)
            printf("*");
        else
            printf("-");
    }
    printf("\n");
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

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError initProcessImage(void)
{
    tOplkError      ret = kErrorOk;
    UINT            varEntries;
    tObdSize        obdSize;

    /* Allocate process image */
    printf("Initializing process image...\n");
    printf("Size of process image: Input = %ld Output = %ld\n", sizeof(PI_IN), sizeof(PI_OUT));
    eventlog_printMessage(kEventlogLevelInfo, kEventlogCategoryGeneric,
                          "Allocating process image: Input:%d Output:%d", sizeof(PI_IN), sizeof(PI_OUT));

    ret = oplk_allocProcessImage(sizeof(PI_IN), sizeof(PI_OUT));
    if (ret != kErrorOk)
    {
        return ret;
    }

    pProcessImageIn_l = (PI_IN*)oplk_getProcessImageIn();
    pProcessImageOut_l = (PI_OUT*)oplk_getProcessImageOut();

    /* link process variables used by CN to object dictionary */
    fprintf(stderr, "Linking process image vars:\n");

    obdSize = sizeof(pProcessImageIn_l->digitalIn);
    varEntries = 1;
    ret = oplk_linkProcessImageObject(0x6000, 0x01, offsetof(PI_IN, digitalIn),
                                      FALSE, obdSize, &varEntries);
    if (ret != kErrorOk)
    {
        fprintf(stderr, "linking process vars failed with \"%s\" (0x%04x)\n", debugstr_getRetValStr(ret), ret);
        return ret;
    }

    obdSize = sizeof(pProcessImageOut_l->digitalOut);
    varEntries = 1;
    ret = oplk_linkProcessImageObject(0x6200, 0x01, offsetof(PI_OUT, digitalOut),
                                      TRUE, obdSize, &varEntries);
    if (ret != kErrorOk)
    {
        fprintf(stderr, "linking process vars failed with \"%s\" (0x%04x)\n", debugstr_getRetValStr(ret), ret);
        return ret;
    }

    fprintf(stderr, "Linking process vars... ok\n\n");

    return kErrorOk;
}

/// \}
