/**
********************************************************************************
\file   DataInOutThread.cpp

\brief  Implementation of the DataInOutThread class

This file implements the data Input/Output thread clas.
*******************************************************************************/
/*------------------------------------------------------------------------------
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2013, SYSTEC electronic GmbH
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
#include <QWidget>
#include <QThread>
#include <QString>

#include "Api.h"

//------------------------------------------------------------------------------
// definitions
//------------------------------------------------------------------------------
#define DEFAULT_MAX_CYCLE_COUNT 20      // 6 is very fast
#define APP_LED_COUNT_1         8       // number of LEDs for CN1
#define APP_LED_MASK_1          (1 << (APP_LED_COUNT_1 - 1))


//------------------------------------------------------------------------------
// global variables
//------------------------------------------------------------------------------
DataInOutThread*  pDataInOutThread_g;

static int        usedNodeIds_g[] = {1, 32, 110, 0};

/* process images, structures defined in xap.h from openCONFIGURATOR */
static PI_IN*     pProcessImageIn_l;
static PI_OUT*    pProcessImageOut_l;

//============================================================================//
//            S T A T I C   M E M B E R   F U N C T I O N S                   //
//============================================================================//

tOplkError DataInOutThread::AppCbSync(void)
{
    return pDataInOutThread_g->processSync();
}

//============================================================================//
//            P U B L I C   M E M B E R   F U N C T I O N S                   //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Constructor

Constructs a DataInOutThread object.
*/
//------------------------------------------------------------------------------
DataInOutThread::DataInOutThread()
{
    int         i;

    /* initialize all application variables */
    for (i = 0; (i < MAX_NODES) && (usedNodeIds_g[i] != 0); i++)
    {
        leds[i] = 0;
        ledsOld[i] = 0;
        input[i] = 0;
        inputOld[i] = 0;
        toggle[i] = 0;
        period[i] = 0;
    }

    fStop = false;
    fMnActive = false;
    pDataInOutThread_g = this;
}

//------------------------------------------------------------------------------
/**
\brief  Synchronous data callback

The function implements the handling of synchronous data. It will be called
from the stack at the synchronisation time.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError DataInOutThread::processSync(void)
{
    tOplkError          ret;
    int                 i;

    ret = oplk_exchangeProcessImageOut();
    if (ret != kErrorOk)
    {
        return ret;
    }

    cnt++;

    input[0] = pProcessImageOut_l->CN1_M00_DigitalInput_00h_AU8_DigitalInput;
    input[1] = pProcessImageOut_l->CN32_M00_DigitalInput_00h_AU8_DigitalInput;
    input[2] = pProcessImageOut_l->CN110_M00_DigitalInput_00h_AU8_DigitalInput;

    for (i = 0; (i < MAX_NODES) && (usedNodeIds_g[i] != 0); i++)
    {

        // If we are not in an active MN state we don't need to
        // do the processing of the outputs!
        if (fMnActive)
        {
            /* Running Leds */
            /* period for LED flashing determined by inputs */
            period[i] = (input[i] == 0) ? 1 : (input[i] * 20);
            if (cnt % period[i] == 0)
            {
                if (leds[i] == 0x00)
                {
                    leds[i] = 0x1;
                    toggle[i] = 1;
                }
                else
                {
                    if (toggle[i])
                    {
                        leds[i] <<= 1;
                        if (leds[i] == APP_LED_MASK_1)
                        {
                            toggle[i] = 0;
                        }
                    }
                    else
                    {
                        leds[i] >>= 1;
                        if (leds[i] == 0x01)
                        {
                            toggle[i] = 1;
                        }
                    }
                }
            }

            outChanged(leds[i], usedNodeIds_g[i]);
            ledsOld[i] = leds[i];
        }
        else
        {
            // We are not controlling the outputs. We show this, by disable the output Leds.
            emit disableOutputs(usedNodeIds_g[i]);
        }

        inChanged(input[i], usedNodeIds_g[i]);
        inputOld[i] = input[i];
    }

    // If we are not in an active MN state we don't need to update the outputs
    if (fMnActive)
    {
        pProcessImageIn_l->CN1_M00_DigitalOutput_00h_AU8_DigitalOutput = leds[0];
        pProcessImageIn_l->CN32_M00_DigitalOutput_00h_AU8_DigitalOutput = leds[1];
        pProcessImageIn_l->CN110_M00_DigitalOutput_00h_AU8_DigitalOutput = leds[2];

        ret = oplk_exchangeProcessImageIn();
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  setup the process image

The function sets up the process image used by the application.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError DataInOutThread::setupProcessImage()
{
    tOplkError          ret;

    ret = oplk_allocProcessImage(sizeof(PI_IN), sizeof(PI_OUT));
    if (ret != kErrorOk)
    {
        return ret;
    }

    pProcessImageIn_l = (PI_IN*)oplk_getProcessImageIn();
    pProcessImageOut_l = (PI_OUT*)oplk_getProcessImageOut();

    ret = oplk_setupProcessImage();

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  signal change of input process image

inChanged() signals that there are changes in the input process image.

\param  input_p         Input data
\param  usedNodeId_p    Node ID the data belongs to
*/
//------------------------------------------------------------------------------
void DataInOutThread::inChanged(int input_p, int usedNodeId_p)
{
    emit processImageInChanged(input_p, usedNodeId_p);
}

//------------------------------------------------------------------------------
/**
\brief  signal change of output process image

The function signals that there are changes in the output process image.

\param  led_p           Output data
\param  usedNodeId_p    Node ID the data belongs to
*/
//------------------------------------------------------------------------------
void DataInOutThread::outChanged(int led_p, int usedNodeId_p)
{
    emit processImageOutChanged(led_p, usedNodeId_p);
}

//------------------------------------------------------------------------------
/**
\brief  thread starting point

The function implements the starting point for the data input/output thread.
*/
//------------------------------------------------------------------------------
void DataInOutThread::run()
{
    tOplkError  ret;

    this->fStop = false;
    while (!this->fStop)
    {
        if (oplk_waitSyncEvent(10000) != kErrorOk)
            continue;

        ret = processSync();
        if (ret != kErrorOk)
        {
            return;
        }
    }
}

//------------------------------------------------------------------------------
/**
\brief  returns pointer to callback

The function returns the address of the synchronous data callback function.

\return address of synchronous data callback function
*/
//------------------------------------------------------------------------------
tSyncCb DataInOutThread::getSyncCbFunc()
{
    return AppCbSync;
}

//------------------------------------------------------------------------------
/**
\brief  Stop synchronous data thread

The function stops the synchronous data thread.
*/
//------------------------------------------------------------------------------
void DataInOutThread::stop()
{
    this->fStop = true;
}

//------------------------------------------------------------------------------
/**
\brief  Set MN Active flag

The function sets the MN active flag which shows if the MN is in an
active state and therefore controlling the synchronous output data.
*/
//------------------------------------------------------------------------------
void DataInOutThread::setMnActiveFlag(bool fMnActive_p)
{
    this->fMnActive = fMnActive_p;
}
