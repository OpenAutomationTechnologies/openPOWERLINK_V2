/**
********************************************************************************
\file   SyncEventHandler.h

\brief  openPOWERLINK sync event handler

This file contains the declaration of the synchronous event handler of the
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
#ifndef _INC_demo_SyncEventHandler_H_
#define _INC_demo_SyncEventHandler_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include <QMap>

#include <oplk/oplk.h>
#include <xap.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// class definitions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/**
\brief  SyncEventHandler class

The Class implements the thread used to transfer synchronous
data between the CNs and the MN.
*/
//------------------------------------------------------------------------------
class SyncEventHandler : public QThread
{
    Q_OBJECT

public:
    tOplkError  setupProcessImage();
    ulong       getMinSyncPeriod() const;
    void        setMinSyncPeriod(ulong minSyncPeriod_p);

    // static members
    static tOplkError           appCbSync();
    static SyncEventHandler&    getInstance();

public slots:
    void setOperational(bool fOperational_p);

signals:
    void processImageInChanged(unsigned int nodeId_p,
                               unsigned int data_p);
    void processImageOutChanged(unsigned int nodeId_p,
                                unsigned int data_p);
    void disableOutputs(unsigned int nodeId_p);

protected:
    virtual ~SyncEventHandler();
    virtual void run() Q_DECL_OVERRIDE;

private:
    SyncEventHandler();
    Q_DISABLE_COPY(SyncEventHandler)

    void processSyncEvent();

    bool                fOperational;
    QMutex              mutex;              ///< Mutex for locking the thread until the wait condition is met
    QWaitCondition      stackSync;          ///< Wait condition for a stack synchronization event
    ulong               minSyncPeriod;

    // process images, structures defined in xap.h from openCONFIGURATOR
    PI_IN*              pProcessImageIn;
    const PI_OUT*       pProcessImageOut;

    // App specific
    uint                cnt;
    QMap<uint, uint>    leds;
    QMap<uint, uint>    input;
    QMap<uint, bool>    toggle;

    static const uint   aUsedNodeIds[];
    static const uint   APP_LED_COUNT;
};

#endif //_INC_demo_SyncEventHandler_H_
