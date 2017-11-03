/**
********************************************************************************
\file   SyncEventHandler.cpp

\brief  openPOWERLINK sync event handler

This file contains the implementation of the synchronous event handler of the
openPOWERLINK stack.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, B&R Industrial Automation GmbH
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
#include <SyncEventHandler.h>

#include <obdpi.h>

//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
const uint  SyncEventHandler::aUsedNodeIds[] = {1, 32, 110, 0};
const uint  SyncEventHandler::APP_LED_COUNT = 8;                    // number of LEDs for CN1

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
//            S T A T I C   M E M B E R   F U N C T I O N S                   //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief   The main sync callback function

The main sync callback is accessed as a function pointer from the stack.

\return a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError SyncEventHandler::appCbSync()
{
    // This method is called from a thread within the stack.
    // Therefore, the thread of this class is woken up to handle the sync event.
    SyncEventHandler::getInstance().stackSync.wakeAll();

    return kErrorOk;
}

//============================================================================//
//            P U B L I C    M E M B E R    F U N C T I O N S                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Get instance of the class

Gets the single instance of this class.

\return Reference to the class instance.
*/
//------------------------------------------------------------------------------
SyncEventHandler& SyncEventHandler::getInstance()
{
    // Local static object (Not thread safe!)
    static SyncEventHandler instance;

    return instance;
}

//------------------------------------------------------------------------------
/**
\brief  Gets the minimum synchronization period

Returns the minimum time difference between two synchronization events in micro seconds.

\return Minimum synchronization period [in us]
*/
//------------------------------------------------------------------------------
ulong SyncEventHandler::getMinSyncPeriod() const
{
    return this->minSyncPeriod;
}

//------------------------------------------------------------------------------
/**
\brief  Sets the minimum synchronization period

Sets the minimum time difference between two synchronization events in micro seconds.

\param[in]      minSyncPeriod_p     Minimum synchronization period [in us]
*/
//------------------------------------------------------------------------------
void SyncEventHandler::setMinSyncPeriod(ulong minSyncPeriod_p)
{
    this->minSyncPeriod = minSyncPeriod_p;
}

//------------------------------------------------------------------------------
/**
\brief  Setup the process image

The function sets up the process image used by the application.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError SyncEventHandler::setupProcessImage()
{
    tOplkError  ret;

    ret = oplk_allocProcessImage(sizeof(PI_IN), sizeof(PI_OUT));
    if (ret != kErrorOk)
        return ret;

    this->pProcessImageIn = static_cast<PI_IN*>(oplk_getProcessImageIn());
    this->pProcessImageOut = static_cast<const PI_OUT*>(oplk_getProcessImageOut());

    UINT errorIndex = obdpi_setupProcessImage();
    if (errorIndex != 0)
        ret = kErrorApiPINotAllocated;

    return ret;
}

//============================================================================//
//          P R O T E C T E D    M E M B E R    F U N C T I O N S             //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Destructor

The destructor of this class is provided to allow overriding it by derived
classes.
*/
//------------------------------------------------------------------------------
SyncEventHandler::~SyncEventHandler()
{
}

//------------------------------------------------------------------------------
/**
\brief  The main loop of this thread.
*/
//------------------------------------------------------------------------------
void SyncEventHandler::run()
{
    while (!this->currentThread()->isInterruptionRequested())
    {
#if defined(CONFIG_KERNELSTACK_DIRECTLINK)
        // Block this thread until the next sync event occurs
        // Note: QWaitCondition::wait() takes milliseconds!
        this->mutex.lock();
        if (!this->stackSync.wait(&this->mutex, 1000))
        {
            // Timeout occured
            // (Probably the stack was stopped, so we should check
            // whether the thread shall be shut down)
            this->mutex.unlock();
            continue;
        }
        // Mutex is locked here...
#else
        // Do not lock the mutex, since the stack function itself is
        // taking care of the thread synchronization

        // Block the thread until the next sync event occurs
        tOplkError oplkRet = oplk_waitSyncEvent(1000);
        if (oplkRet == kErrorGeneralError)
        {
            // Timeout occured
            // (Probably the stack was stopped, so we should check
            // whether the thread shall be shut down)
            continue;
        }

        // Lock the mutex
        this->mutex.lock();
#endif
        // Process the sync event
        this->processSyncEvent();

        // Unlock the mutex
        this->mutex.unlock();

        // Sleep for the given time
        QThread::usleep(this->minSyncPeriod);
    }
}

//============================================================================//
//            P R I V A T E    M E M B E R    F U N C T I O N S               //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Constructor

Constructs a SyncEventHandler object.
*/
//------------------------------------------------------------------------------
SyncEventHandler::SyncEventHandler() :
    fOperational(false),
    minSyncPeriod(0)
{
    // initialize all application variables
    for (int i = 0;
         SyncEventHandler::aUsedNodeIds[i] != 0;
         i++)
    {
        this->leds.insert(i, 0);
        this->input.insert(i, 0);
        this->toggle.insert(i, false);
    }
}

//------------------------------------------------------------------------------
/**
\brief  Set the operational flag

This method sets the operational flag. The flag shows whether the node is in an
active state and therefore controlling the synchronous output data.

\param[in]      fOperational_p      Determines whether the node is active

*/
//------------------------------------------------------------------------------
void SyncEventHandler::setOperational(bool fOperational_p)
{
    this->fOperational = fOperational_p;
}

//------------------------------------------------------------------------------
/**
\brief  Synchronous data callback

The function implements the handling of synchronous data. It will be called
from the stack at the synchronization time.
*/
//------------------------------------------------------------------------------
void SyncEventHandler::processSyncEvent()
{
    tOplkError  ret;

    // Read the inputs
    ret = oplk_exchangeProcessImageOut();
    if (ret != kErrorOk)
        return;

    this->input[0] = this->pProcessImageOut->CN1_DigitalInput_00h_AU8_DigitalInput;
    this->input[1] = this->pProcessImageOut->CN32_DigitalInput_00h_AU8_DigitalInput;
    this->input[2] = this->pProcessImageOut->CN110_DigitalInput_00h_AU8_DigitalInput;

    this->cnt++;

    for (int i = 0;
         SyncEventHandler::aUsedNodeIds[i] != 0;
         i++)
    {
        // If we are not in an active MN state we don't need to
        // do the processing of the outputs!
        if (this->fOperational)
        {
            // Running LEDs
            // period for LED flashing determined by inputs
            uint period = (this->input[i] == 0) ? 1 : (this->input[i] * 20);
            if ((this->cnt % period) == 0)
            {
                if (this->leds[i] == 0x00)
                {
                    this->leds[i] = 0x01;
                    this->toggle[i] = true;
                }
                else
                {
                    if (this->toggle[i])
                    {
                        this->leds[i] <<= 1;
                        if (this->leds[i] == (1 << (SyncEventHandler::APP_LED_COUNT - 1)))
                            this->toggle[i] = false;
                    }
                    else
                    {
                        this->leds[i] >>= 1;
                        if (this->leds[i] == 0x01)
                            this->toggle[i] = true;
                    }
                }
            }

            emit processImageOutChanged(SyncEventHandler::aUsedNodeIds[i], this->leds[i]);
        }
        else
        {
            // We are not controlling the outputs. We show this, by disable the output LEDs.
            emit disableOutputs(SyncEventHandler::aUsedNodeIds[i]);
        }

        emit processImageInChanged(SyncEventHandler::aUsedNodeIds[i], this->input[i]);
    }

    // If we are not in an active MN state we don't need to update the outputs
    if (this->fOperational)
    {
        // Write the outputs
        this->pProcessImageIn->CN1_DigitalOutput_00h_AU8_DigitalOutput = this->leds[0];
        this->pProcessImageIn->CN32_DigitalOutput_00h_AU8_DigitalOutput = this->leds[1];
        this->pProcessImageIn->CN110_DigitalOutput_00h_AU8_DigitalOutput = this->leds[2];

        ret = oplk_exchangeProcessImageIn();
        if (ret != kErrorOk)
            return;
    }
}
