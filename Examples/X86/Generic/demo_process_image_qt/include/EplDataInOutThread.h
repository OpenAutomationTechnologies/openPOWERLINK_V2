/**
********************************************************************************

  \file           EplDataInOutThread.h

  \brief          Header file for Data Input/Output class

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  (c) Bernecker + Rainer Ges.m.b.H.
      B&R Strasse 1, 5142 Eggelsberg, Austria
      www.br-automation.com

  License:

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.

    3. Neither the name of Bernecker + Rainer Ges.m.b.H nor the names of its
       contributors may be used to endorse or promote products derived
       from this software without prior written permission. For written
       permission, please contact office@br-automation.com.

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

*******************************************************************************/

#ifndef EPL_DATA_IN_OUT_THREAD_H
#define EPL_DATA_IN_OUT_THREAD_H

/******************************************************************************/
/* includes */
#include <QThread>

#include "Epl.h"
#include "xap.h"

/******************************************************************************/
/* definitions */
#define MAX_NODES       255

/******************************************************************************/
/* class definitions */
class QWidget;
class QString;

/**
********************************************************************************
\brief  EplDataInOutThread class

Class EplDataInOutThread implements the thread used to transfer synchronous
data between the CNs and the MN.
*******************************************************************************/
class EplDataInOutThread : public QThread
{
    Q_OBJECT

public:
    EplDataInOutThread();

    void run();

    void acknowledge();
    void inChanged(unsigned int uiInput_p, int iUsedNodeId_p);
    void outChanged(unsigned int uiLed_p, int iUsedNodeId_p);

    tEplKernel SetupProcessImage();
    tEplSyncCb getSyncCbFunc();

    unsigned int            m_uiCnt;

    unsigned int            m_uiLeds[MAX_NODES];
    unsigned int            m_uiLedsOld[MAX_NODES];
    unsigned int            m_uiInput[MAX_NODES];
    unsigned int            m_uiInputOld[MAX_NODES];
    unsigned int            m_uiPeriod[MAX_NODES];
    int                     m_iToggle[MAX_NODES];

signals:
    void processImageInChanged(unsigned int uiData_p, unsigned int uiNodeId_p);
    void processImageOutChanged(unsigned int uiData_p, unsigned int uiNodeId_p);

private:
    volatile unsigned int   m_uiAckCount;
    unsigned int            m_uiCount;

};

#endif

