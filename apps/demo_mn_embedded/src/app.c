/**
********************************************************************************
\file   app.c

\brief  Demo MN application which implements a running light

This file contains a demo application for digital input/output data.
It implements a running light on the digital outputs. The speed of
the running light is controlled by the read digital inputs.

\ingroup module_demo_mn_console
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2013, SYSTEC electronik GmbH
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
#include "app.h"

#include <oplk/oplk.h>
#include <obdpi.h>

#include "xap.h"

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define DEFAULT_MAX_CYCLE_COUNT 20      // 6 is very fast
#define APP_LED_COUNT_1         8       // number of LEDs for CN1
#define APP_LED_MASK_1          (1 << (APP_LED_COUNT_1 - 1))
#define MAX_NODES               255

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
typedef struct
{
    UINT                leds;
    UINT                ledsOld;
    UINT                input;
    UINT                inputOld;
    UINT                period;
    int                 toggle;
} APP_NODE_VAR_T;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static int              aUsedNodeIds_l[] = {1, 32, 110, 0};
static UINT             cnt_l;
static APP_NODE_VAR_T   aNodeVar_l[MAX_NODES];
static PI_IN*           pProcessImageIn_l;
static const PI_OUT*    pProcessImageOut_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError       initProcessImage(void);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize the synchronous data application

The function initializes the synchronous data application

\return The function returns a tOplkError error code.

\ingroup module_demo_mn_console
*/
//------------------------------------------------------------------------------
tOplkError initApp(void)
{
    tOplkError  ret = kErrorOk;
    int         i;

    cnt_l = 0;

    for (i = 0; (i < MAX_NODES) && (aUsedNodeIds_l[i] != 0); i++)
    {
        aNodeVar_l[i].leds = 0;
        aNodeVar_l[i].ledsOld = 0;
        aNodeVar_l[i].input = 0;
        aNodeVar_l[i].inputOld = 0;
        aNodeVar_l[i].toggle = 0;
        aNodeVar_l[i].period = 0;
    }

    ret = initProcessImage();

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Shutdown the synchronous data application

The function shuts down the synchronous data application

\return The function returns a tOplkError error code.

\ingroup module_demo_mn_console
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

\ingroup module_demo_mn_console
*/
//------------------------------------------------------------------------------
tOplkError processSync(void)
{
    tOplkError  ret;
    int         i;

    ret = oplk_exchangeProcessImageOut();
    if (ret != kErrorOk)
        return ret;

    cnt_l++;

    aNodeVar_l[0].input = pProcessImageOut_l->CN1_M00_DigitalInput_00h_AU8_DigitalInput;
    aNodeVar_l[1].input = pProcessImageOut_l->CN32_M00_DigitalInput_00h_AU8_DigitalInput;
    aNodeVar_l[2].input = pProcessImageOut_l->CN110_M00_DigitalInput_00h_AU8_DigitalInput;

    for (i = 0; (i < MAX_NODES) && (aUsedNodeIds_l[i] != 0); i++)
    {
        /* Running LEDs */
        /* period for LED flashing determined by inputs */
        aNodeVar_l[i].period = (aNodeVar_l[i].input == 0) ? 1 : (aNodeVar_l[i].input * 20);
        if (cnt_l % aNodeVar_l[i].period == 0)
        {
            if (aNodeVar_l[i].leds == 0x00)
            {
                aNodeVar_l[i].leds = 0x1;
                aNodeVar_l[i].toggle = 1;
            }
            else
            {
                if (aNodeVar_l[i].toggle)
                {
                    aNodeVar_l[i].leds <<= 1;
                    if (aNodeVar_l[i].leds == APP_LED_MASK_1)
                        aNodeVar_l[i].toggle = 0;
                }
                else
                {
                    aNodeVar_l[i].leds >>= 1;
                    if (aNodeVar_l[i].leds == 0x01)
                        aNodeVar_l[i].toggle = 1;
                }
            }
        }

        if (aNodeVar_l[i].input != aNodeVar_l[i].inputOld)
            aNodeVar_l[i].inputOld = aNodeVar_l[i].input;

        if (aNodeVar_l[i].leds != aNodeVar_l[i].ledsOld)
            aNodeVar_l[i].ledsOld = aNodeVar_l[i].leds;
    }

    pProcessImageIn_l->CN1_M00_DigitalOutput_00h_AU8_DigitalOutput = aNodeVar_l[0].leds;
    pProcessImageIn_l->CN32_M00_DigitalOutput_00h_AU8_DigitalOutput = aNodeVar_l[1].leds;
    pProcessImageIn_l->CN110_M00_DigitalOutput_00h_AU8_DigitalOutput = aNodeVar_l[2].leds;

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

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError initProcessImage(void)
{
    tOplkError  ret = kErrorOk;
    UINT        errorIndex = 0;

    PRINTF("Initializing process image...\n");
    PRINTF("Size of process image: Input = %lu Output = %lu\n",
           (ULONG)sizeof(PI_IN),
           (ULONG)sizeof(PI_OUT));
    ret = oplk_allocProcessImage(sizeof(PI_IN), sizeof(PI_OUT));
    if (ret != kErrorOk)
        return ret;

    pProcessImageIn_l = (PI_IN*)oplk_getProcessImageIn();
    pProcessImageOut_l = (const PI_OUT*)oplk_getProcessImageOut();

    errorIndex = obdpi_setupProcessImage();
    if (errorIndex != 0)
    {
        PRINTF("Setup process image failed at index 0x%04x\n", errorIndex);
        ret = kErrorApiPINotAllocated;
    }

    return ret;
}

/// \}
