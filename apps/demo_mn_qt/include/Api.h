/**
********************************************************************************
\file   Api.h

\brief  Header file for openPOWERLINK API class

This file contains the definitions of the openPOWERLINK API class.
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
#ifndef _INC_demo_Api_H_
#define _INC_demo_Api_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <QObject>
#include <QString>

#include <oplk/oplk.h>

//------------------------------------------------------------------------------
// class declarations
//------------------------------------------------------------------------------
class QWidget;

class MainWindow;
class ProcessThread;
class DataInOutThread;

//------------------------------------------------------------------------------
/**
\brief  Api class

Class Api implements the API interface to the openPOWERLINK stack.
*/
//------------------------------------------------------------------------------
class Api : public QObject
{
    Q_OBJECT

public:
    Api(MainWindow* pMainWindow_p,
        UINT nodeId_p,
        const QString& rDevName_p);
    ~Api();

    static UINT defaultNodeId();

signals:
    void userDefEvent(void* pUserArg_p);
    void sdoFinished(tSdoComFinished sdoInfo_p);

private:
    tOplkApiInitParam   initParam;

    ProcessThread*      pProcessThread;
    DataInOutThread*    pDataInOutThread;

    const char*         pCdcFilename;
};

#endif /*_INC_demo_Api_H_*/
