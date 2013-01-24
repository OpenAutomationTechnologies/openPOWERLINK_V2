/**
********************************************************************************

  \file           MainWindow.h

  \brief          Header file for main window class

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
#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

/******************************************************************************/
/* includes */
#include <QWidget>

#include "EplApi.h"
#include "EplState.h"
#include "EplInput.h"
#include "EplCnState.h"
#include "EplOutput.h"

/******************************************************************************/
/* class declarations */
class QLineEdit;
class QPushButton;
class QToolButton;
class QLabel;
class QFrame;

/**
********************************************************************************
\brief  MainWindow class

Class MainWindow implements the main window class of the demo application.
*******************************************************************************/
class MainWindow : public QWidget
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = 0);

    void        resizeHeadRegion();
    EplState*   getEplStateWidget() {return pEplState;};
    EplOutput*  getEplOutputWidget() {return pEplOutput;};
    EplInput*   getEplInputWidget() {return pEplInput;};
    EplCnState*   getEplCnStateWidget() {return pEplCnState;};

private slots:
    void toggleWindowState();
    void startPowerlink();
    void stopPowerlink();

private:
    QHBoxLayout *pHeadRegion;
    QFrame      *pHeadBackground;

    QPixmap     *pRightLogo;
    QLabel      *pLeftLabel;
    QLabel      *pRightLabel;

    EplState    *pEplState;
    EplCnState  *pEplCnState;
    EplInput    *pEplInput;
    EplOutput   *pEplOutput;

    QLineEdit   *pNodeIdEdit;

    QPushButton *pToggleMax;
    QPushButton *pStartStopEpl;

    QFrame      *pFrameSepHeadMiddle;
    QFrame      *pFrameSepMiddle;
    QFrame      *pFrameSepMiddle2;
    QFrame      *pFrameSepMiddleStatus;
    QFrame      *pFrameSepStatusFoot;

    EplApi      *pEplApi;

protected:
    void resizeEvent(QResizeEvent *event);
};

#endif

