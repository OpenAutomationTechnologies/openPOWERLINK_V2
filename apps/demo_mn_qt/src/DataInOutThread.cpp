/**
********************************************************************************
\file   DataInOutThread.cpp

\brief  Implementation of the DataInOutThread class

This file implements the data Input/Output thread clas.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <DataInOutThread.h>
#include <xap.h>

#include <obdpi.h>
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
DataInOutThread*        pDataInOutThread_g;

static unsigned int     aUsedNodeIds_l[] = {1, 32, 110, 0};

/* process images, structures defined in xap.h from openCONFIGURATOR */
static PI_IN*           pProcessImageIn_l;
static const PI_OUT*    pProcessImageOut_l;
//============================================================================//
//            S T A T I C   M E M B E R   F U N C T I O N S                   //
//============================================================================//
tOplkError DataInOutThread::appCbSync(void)
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
    int i;

    /* initialize all application variables */
    for (i = 0; (i < MAX_NODES) && (aUsedNodeIds_l[i] != 0); i++)
    {
        this->aLeds[i] = 0;
        this->aLedsOld[i] = 0;
        this->aInput[i] = 0;
        this->aInputOld[i] = 0;
        this->aToggle[i] = 0;
        this->aPeriod[i] = 0;
    }

    this->fStop = false;
    this->fMnActive = false;

    pDataInOutThread_g = this;
}

//------------------------------------------------------------------------------
/**
\brief  Synchronous data callback

The function implements the handling of synchronous data. It will be called
from the stack at the synchronization time.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError DataInOutThread::processSync(void)
{
    tOplkError  ret;
    int         i;

    ret = oplk_exchangeProcessImageOut();
    if (ret != kErrorOk)
        return ret;

    this->cnt++;

    this->aInput[0] = pProcessImageOut_l->CN1_M00_DigitalInput_00h_AU8_DigitalInput;
    this->aInput[1] = pProcessImageOut_l->CN32_M00_DigitalInput_00h_AU8_DigitalInput;
    this->aInput[2] = pProcessImageOut_l->CN110_M00_DigitalInput_00h_AU8_DigitalInput;

    for (i = 0; (i < MAX_NODES) && (aUsedNodeIds_l[i] != 0); i++)
    {

        // If we are not in an active MN state we don't need to
        // do the processing of the outputs!
        if (this->fMnActive)
        {
            /* Running LEDs */
            /* period for LED flashing determined by inputs */
            this->aPeriod[i] = (this->aInput[i] == 0) ? 1 : (this->aInput[i] * 20);
            if (this->cnt % this->aPeriod[i] == 0)
            {
                if (this->aLeds[i] == 0x00)
                {
                    this->aLeds[i] = 0x1;
                    this->aToggle[i] = 1;
                }
                else
                {
                    if (this->aToggle[i])
                    {
                        this->aLeds[i] <<= 1;
                        if (this->aLeds[i] == APP_LED_MASK_1)
                            this->aToggle[i] = 0;
                    }
                    else
                    {
                        this->aLeds[i] >>= 1;
                        if (this->aLeds[i] == 0x01)
                            this->aToggle[i] = 1;
                    }
                }
            }

            this->outChanged(aUsedNodeIds_l[i], this->aLeds[i]);
            this->aLedsOld[i] = this->aLeds[i];
        }
        else
        {
            // We are not controlling the outputs. We show this, by disable the output LEDs.
            emit disableOutputs(aUsedNodeIds_l[i]);
        }

        this->inChanged(aUsedNodeIds_l[i], this->aInput[i]);
        this->aInputOld[i] = this->aInput[i];
    }

    // If we are not in an active MN state we don't need to update the outputs
    if (this->fMnActive)
    {
        pProcessImageIn_l->CN1_M00_DigitalOutput_00h_AU8_DigitalOutput = this->aLeds[0];
        pProcessImageIn_l->CN32_M00_DigitalOutput_00h_AU8_DigitalOutput = this->aLeds[1];
        pProcessImageIn_l->CN110_M00_DigitalOutput_00h_AU8_DigitalOutput = this->aLeds[2];

        ret = oplk_exchangeProcessImageIn();
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Setup the process image

The function sets up the process image used by the application.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError DataInOutThread::setupProcessImage()
{
    tOplkError  ret;
    UINT        errorIndex = 0;

    ret = oplk_allocProcessImage(sizeof(PI_IN), sizeof(PI_OUT));
    if (ret != kErrorOk)
        return ret;

    pProcessImageIn_l = (PI_IN*)oplk_getProcessImageIn();
    pProcessImageOut_l = (const PI_OUT*)oplk_getProcessImageOut();

    errorIndex = obdpi_setupProcessImage();
    if (errorIndex != 0)
        ret = kErrorApiPINotAllocated;

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Signal change of input process image

inChanged() signals that there are changes in the input process image.

\param[in]      usedNodeId_p        Node ID the data belongs to
\param[in]      input_p             Input data
*/
//------------------------------------------------------------------------------
void DataInOutThread::inChanged(unsigned int usedNodeId_p,
                                unsigned int input_p)
{
    emit processImageInChanged(usedNodeId_p, input_p);
}

//------------------------------------------------------------------------------
/**
\brief  Signal change of output process image

The function signals that there are changes in the output process image.

\param[in]      usedNodeId_p        Node ID the data belongs to
\param[in]      output_p            Output data
*/
//------------------------------------------------------------------------------
void DataInOutThread::outChanged(unsigned int usedNodeId_p,
                                 unsigned int output_p)
{
    emit processImageOutChanged(usedNodeId_p, output_p);
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
            return;
    }
}

//------------------------------------------------------------------------------
/**
\brief  returns pointer to callback

The function returns the address of the synchronous data callback function.

\return address of synchronous data callback function
*/
//------------------------------------------------------------------------------
tSyncCb DataInOutThread::getSyncCbFunc() const
{
    return appCbSync;
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

\param[in]      fMnActive_p         Determines whether the MN is active

*/
//------------------------------------------------------------------------------
void DataInOutThread::setMnActiveFlag(bool fMnActive_p)
{
    this->fMnActive = fMnActive_p;
}
