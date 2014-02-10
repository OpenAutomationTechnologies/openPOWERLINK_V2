/**
********************************************************************************
\file   DataInOutThread.h

\brief  Header file for Data Input/Output class

This file implements the header file of the Data Input/Output class.
*******************************************************************************/
/*------------------------------------------------------------------------------
Copyright (c) 2013, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

#ifndef _INC_DataInOutThread_H_
#define _INC_DataInOutThread_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <QThread>

#include <oplk/oplk.h>
#include "xap.h"

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define MAX_NODES       255

//------------------------------------------------------------------------------
// class definitions
//------------------------------------------------------------------------------
class QWidget;
class QString;

//------------------------------------------------------------------------------
/**
\brief  DataInOutThread class

The Class implements the thread used to transfer synchronous
data between the CNs and the MN.
*/
//------------------------------------------------------------------------------
class DataInOutThread : public QThread
{
    Q_OBJECT

public:
    DataInOutThread();

    void run();
    void acknowledge();
    void inChanged(int input_p, int usedNodeId_p);
    void outChanged(int led_p, int usedNodeId_p);
    tOplkError setupProcessImage();
    tSyncCb getSyncCbFunc();
    tOplkError processSync(void);
    static tOplkError AppCbSync(void);

signals:
    void processImageInChanged(int data_p, int nodeId_p);
    void processImageOutChanged(int data_p, int nodeId_p);

private:
    //    volatile UINT   ackCount;

    UINT            cnt;
    UINT            leds[MAX_NODES];
    UINT            ledsOld[MAX_NODES];
    UINT            input[MAX_NODES];
    UINT            inputOld[MAX_NODES];
    UINT            period[MAX_NODES];
    int             toggle[MAX_NODES];

};

#endif //_INC_DataInOutThread_H_

