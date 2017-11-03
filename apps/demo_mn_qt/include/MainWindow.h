/**
********************************************************************************
\file   MainWindow.h

\brief  Header file for main window class

This file contains the definitions of the main window class.
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
#ifndef _INC_demo_MainWindow_H_
#define _INC_demo_MainWindow_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <QMainWindow>
#include "ui_MainWindow.h"

#include <QString>
#include <oplk/oplk.h>

//------------------------------------------------------------------------------
// class definitions
//------------------------------------------------------------------------------
class Api;
class SdoTransferDialog;

//------------------------------------------------------------------------------
/**
\brief  MainWindow class

Class MainWindow implements the main window class of the demo application.
*/
//------------------------------------------------------------------------------
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget* pParent_p = 0);
    ~MainWindow();

    IoWidget*       getOutputWidget() const {return this->ui.pCnOutputWidget;}
    IoWidget*       getInputWidget() const {return this->ui.pCnInputWidget;}

private slots:
    void toggleWindowState();
    void startStopStack();
    void showSdoDialog();
    void execNmtCmd();
    void printLogMessage(const QString& msg_p);
    void nmtStateChanged(tNmtState nmtState_p);
    void nodeNmtStateChanged(unsigned int nodeId_p,
                             tNmtState state_p);

private:
    Ui::MainWindow      ui;

    Api*                pApi;
    SdoTransferDialog*  pSdoDialog;
    QString             devName;
    bool                fStackIsRunning;

    // Private methods
    void startPowerlink();
    void stopPowerlink();
};

#endif /* _INC_demo_MainWindow_H_ */
