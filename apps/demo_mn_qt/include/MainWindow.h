/**
********************************************************************************
\file   MainWindow.h

\brief  Header file for main window class

This file contains the definitions of the main window class.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#ifndef _INC_demo_MainWindow_H_
#define _INC_demo_MainWindow_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <QWidget>
#include <QString>

#include <oplk/oplk.h>

//------------------------------------------------------------------------------
// class definitions
//------------------------------------------------------------------------------
class QVBoxLayout;
class QHBoxLayout;
class QPixmap;
class QLabel;
class QLineEdit;
class QPushButton;
class QFrame;
class QTextEdit;

class NmtStateWidget;
class Output;
class Input;
class CnListWidget;
class Api;
class SdoDialog;

//------------------------------------------------------------------------------
/**
\brief  MainWindow class

Class MainWindow implements the main window class of the demo application.
*/
//------------------------------------------------------------------------------
class MainWindow : public QWidget
{
    Q_OBJECT

public:
    MainWindow(QWidget* pParent_p = 0);

    NmtStateWidget* getNmtStateWidget() const {return this->pNmtStateWidget;}
    Output*         getOutputWidget() const {return this->pOutputWidget;}
    Input*          getInputWidget() const {return this->pInputWidget;}
    CnListWidget*   getCnStateWidget() const {return this->pCnStateWidget;}

private slots:
    void         toggleWindowState();
    void         startStopStack();
    void         showSdoDialog();
    void         printlog(const QString& str);
    void         execNmtCmd();

private:
    // General layout
    QVBoxLayout*    pWindowLayout;

    // Head region
    QHBoxLayout*    pHeadRegionLayout;
    QLabel*         pLogo;
    QLabel*         pVersionLabel;

    // Separator line
    QFrame*         pFrameSepHeadMiddle;

    // Middle region
    QHBoxLayout*    pMiddleRegion;

    QWidget*        pCnStateRegion;
    QVBoxLayout*    pCnStateLayout;
    QLabel*         pCnStateWidgetLabel;
    CnListWidget*   pCnStateWidget;

    QFrame*         pFrameSepMiddle;    // Vertical separator line
    Input*          pInputWidget;
    QFrame*         pFrameSepMiddle2;   // Vertical separator line
    Output*         pOutputWidget;

    // Separator line
    QFrame*         pFrameSepMiddleStatus;

    // Status region
    QHBoxLayout*    pStatusRegion;
    QLabel*         pNmtStateLabel;
    NmtStateWidget* pNmtStateWidget;

    // Separator line
    QFrame*         pFrameSepStatusFoot;

    // Foot region
    QHBoxLayout*    pFootRegion;
    QLabel*         pNodeIdLabel;
    QLineEdit*      pNodeIdEdit;
    QPushButton*    pStartStopOplk;
    QPushButton*    pShowSdoDialog;
    QPushButton*    pNmtCmd;
    QPushButton*    pToggleMax;
    QPushButton*    pQuitButton;

    // Text console region
    QHBoxLayout*    pTextConsoleRegion;
    QTextEdit*      pTextEdit;

    Api*            pApi;
    SdoDialog*      pSdoDialog;

    QString         devName;
    tNmtEvent       nmtEvent;
    bool            stackIsRunning;

    // Private methods
    void startPowerlink();
    void stopPowerlink();
    void setupUi();
};

#endif /* _INC_demo_MainWindow_H_ */
