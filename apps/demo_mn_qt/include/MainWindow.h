/**
********************************************************************************
\file   MainWindow.h

\brief  Header file for main window class

This file contains the definitions of the main window class.
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
class QHBoxLayout;
class QPixmap;
class QLabel;
class QLineEdit;
class QPushButton;
class QFrame;
class QTextEdit;

class State;
class Output;
class Input;
class CnState;
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
    MainWindow(QWidget* parent = 0);

    State*       getStateWidget() const {return this->pState;}
    Output*      getOutputWidget() const {return this->pOutput;}
    Input*       getInputWidget() const {return this->pInput;}
    CnState*     getCnStateWidget() const {return this->pCnState;}

private slots:
    void         toggleWindowState();
    void         startPowerlink();
    void         stopPowerlink();
    void         showSdoDialog();
    void         printlog(const QString& str);
    void         execNmtCmd();

private:
    QHBoxLayout* pHeadRegion;
    QPixmap*     pLogo;
    QLabel*      pLabel;

    State*       pState;
    CnState*     pCnState;
    Input*       pInput;
    Output*      pOutput;

    QLineEdit*   pNodeIdEdit;

    QPushButton* pToggleMax;
    QPushButton* pShowSdoDialog;
    QPushButton* pStartStopOplk;
    QPushButton* pNmtCmd;

    QFrame*      pFrameSepHeadMiddle;
    QFrame*      pFrameSepMiddle;
    QFrame*      pFrameSepMiddle2;
    QFrame*      pFrameSepMiddleStatus;
    QFrame*      pFrameSepStatusFoot;

    QTextEdit*   pTextEdit;

    Api*         pApi;
    SdoDialog*   pSdoDialog;

    UINT32       version;

    QString      devName;

    tNmtEvent    nmtEvent;
};

#endif /* _INC_demo_MainWindow_H_ */
