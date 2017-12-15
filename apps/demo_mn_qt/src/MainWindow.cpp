/**
********************************************************************************
\file   MainWindow.cpp

\brief  Implementation of main window class

This file contains the implementation of the main window class.
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

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <MainWindow.h>
#include <QMessageBox>
#include <Api.h>
#include <SdoTransferDialog.h>
#include <NmtCommandDialog.h>
#include <InterfaceSelectionDialog.h>

//============================================================================//
//            P U B L I C    M E M B E R    F U N C T I O N S                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  constructor

Constructor of main window class.

\param[in,out]  pParent_p           Pointer to the parent window
*/
//------------------------------------------------------------------------------
MainWindow::MainWindow(QWidget* pParent_p) :
    QMainWindow(pParent_p),
    fStackIsRunning(false),
    pSdoDialog(NULL)
{
    // Setup UI elements
    this->ui.setupUi(this);

    // Set dynamic GUI information
    UINT32 oplkVersion = Api::getVersion();
    QString versionString = QString("Version " +
                            QString::number(PLK_STACK_VER(oplkVersion)) + "." +
                            QString::number(PLK_STACK_REF(oplkVersion)) + "." +
                            QString::number(PLK_STACK_REL(oplkVersion)));
    this->ui.pVersionLabel->setText(versionString);

    this->ui.pNmtStateWidget->showNmtStateText();

    // Init stack
    this->pApi = new Api(this);
}

//------------------------------------------------------------------------------
/**
\brief  Destructor

Destructs the main window class.
*/
//------------------------------------------------------------------------------
MainWindow::~MainWindow()
{
    delete this->pApi;
}

//------------------------------------------------------------------------------
/**
\brief  Toggle window state

Toggles between fullscreen and normal window state.
*/
//------------------------------------------------------------------------------
void MainWindow::toggleWindowState()
{
    // Toggle the window state
    this->setWindowState(this->windowState() ^ Qt::WindowFullScreen);

    // Change the labeling of the button
    if (this->windowState() & Qt::WindowFullScreen)
        this->ui.pToggleMax->setText(tr("Window"));
    else
        this->ui.pToggleMax->setText(tr("Full Screen"));
}

//------------------------------------------------------------------------------
/**
\brief  Start or stop POWERLINK

Starts or stops the openPOWERLINK stack.
*/
//------------------------------------------------------------------------------
void MainWindow::startStopStack()
{
    if (!this->fStackIsRunning)
        this->startPowerlink();
    else
        this->stopPowerlink();
}

//------------------------------------------------------------------------------
/**
\brief  Start POWERLINK

Starts the openPOWERLINK stack.
*/
//------------------------------------------------------------------------------
void MainWindow::startPowerlink()
{
    // start the selection dialog
    InterfaceSelectionDialog    interfaceDialog;

    if (interfaceDialog.fillList() < 0)
    {
        QMessageBox::warning(this,
                             "Interface selection not working!",
                             "No interfaces found!\n"
                             "Make sure that you have got root permissions!",
                             QMessageBox::Close);
        return;
    }

    interfaceDialog.setActive(this->devName);
    if (interfaceDialog.exec() == QDialog::Rejected)
        return;

    this->devName = interfaceDialog.getDevName();

    // Update GUI elements to started stack
    this->ui.pNodeIdInput->setEnabled(false);
    this->ui.pExecNmtCmd->setEnabled(true);
    this->ui.pStartStopOplk->setText(tr("Stop POWERLINK"));

    // Start the stack
    this->pApi->start((unsigned int)this->ui.pNodeIdInput->value(), this->devName);

    // Connect some signals/slots for the SDO dialog
    if (this->pSdoDialog)
    {
        QObject::connect(this->pApi,
                         SIGNAL(userDefEvent(void*)),
                         this->pSdoDialog,
                         SLOT(userDefEvent(void*)),
                         Qt::DirectConnection);
        QObject::connect(this->pApi,
                         SIGNAL(sdoFinished(tSdoComFinished)),
                         this->pSdoDialog,
                         SLOT(sdoFinished(tSdoComFinished)));
    }

    this->fStackIsRunning = true;
}

//------------------------------------------------------------------------------
/**
\brief  Stop POWERLINK

Stops the openPOWERLINK stack.
*/
//------------------------------------------------------------------------------
void MainWindow::stopPowerlink()
{
    this->fStackIsRunning = false;

    // Stop the stack
    this->pApi->stop();

    // Update GUI elements to stopped stack
    this->ui.pStartStopOplk->setText(tr("Start POWERLINK"));
    this->ui.pNodeIdInput->setEnabled(true);
    this->ui.pExecNmtCmd->setEnabled(false);
}

//------------------------------------------------------------------------------
/**
\brief  Execute NMT command dialog

Execute NMT command/event entered in dialog.
*/
//------------------------------------------------------------------------------
void MainWindow::execNmtCmd()
{
    NmtCommandDialog    nmtCommandDialog;

    if (nmtCommandDialog.exec() == QDialog::Accepted)
    {
        tNmtEvent   nmtCommand = nmtCommandDialog.getNmtEvent();

        if (nmtCommand != kNmtEventNoEvent)
            Api::execNmtCommand(nmtCommand);
    }
}

//------------------------------------------------------------------------------
/**
\brief  Show SDO dialog

Show dialog to perform SDO transfers.
*/
//------------------------------------------------------------------------------
void MainWindow::showSdoDialog()
{
    if (!this->pSdoDialog)
    {
        this->pSdoDialog = new SdoTransferDialog();
        if (this->pApi)
        {
            QObject::connect(this->pApi,
                             SIGNAL(userDefEvent(void*)),
                             this->pSdoDialog,
                             SLOT(userDefEvent(void*)),
                             Qt::DirectConnection);
            QObject::connect(this->pApi,
                             SIGNAL(sdoFinished(tSdoComFinished)),
                             this->pSdoDialog,
                             SLOT(sdoFinished(tSdoComFinished)));
        }
    }
    if (this->pSdoDialog->isVisible())
    {
        this->pSdoDialog->showNormal();
        this->pSdoDialog->activateWindow();
        this->pSdoDialog->raise();
    }
    else
        this->pSdoDialog->show();
}

//------------------------------------------------------------------------------
/**
\brief  Print a log message

The function prints a log message.

\param[in]      msg_p               String to print.
*/
//------------------------------------------------------------------------------
void MainWindow::printLogMessage(const QString& msg_p)
{
    this->ui.pTextEdit->append(msg_p);
}

//------------------------------------------------------------------------------
/**
\brief  NMT state changed

Handle an NMT state change of this node.

\param[in]      nmtState_p          New NMT state
*/
//------------------------------------------------------------------------------
void MainWindow::nmtStateChanged(tNmtState nmtState_p)
{
    // Show the new state in the widget
    this->ui.pNmtStateWidget->setNmtState(nmtState_p);

    // Handle the new NMT state
    if (nmtState_p == kNmtGsOff)
    {
        this->ui.pCnListWidget->removeAllNodes();
        this->ui.pCnInputWidget->removeAllNodes();
        this->ui.pCnOutputWidget->removeAllNodes();
    }
}

//------------------------------------------------------------------------------
/**
\brief  NMT state of another node changed

Handle an NMT state change of another node in the network.

\param[in]      nodeId_p            Node ID of CN
\param[in]      state_p             State of CN
*/
//------------------------------------------------------------------------------
void MainWindow::nodeNmtStateChanged(unsigned int nodeId_p, tNmtState state_p)
{
    // Show the new state in the widget
    this->ui.pCnListWidget->setState(nodeId_p, state_p);

    // Reflect the changes on the other widgets
    switch (state_p)
    {
        case kNmtCsBasicEthernet:
        case kNmtCsStopped:
        case kNmtGsInitialising:
        case kNmtGsResetApplication:
        case kNmtGsResetCommunication:
        case kNmtGsResetConfiguration:
        case kNmtCsPreOperational1:
        case kNmtCsPreOperational2:
        case kNmtCsReadyToOperate:
        case kNmtCsOperational:
            this->ui.pCnListWidget->addNode(nodeId_p);
            this->ui.pCnInputWidget->addNode(nodeId_p);
            this->ui.pCnOutputWidget->addNode(nodeId_p);
            break;

        case kNmtCsNotActive:
        case kNmtGsOff:
        default:
            this->ui.pCnListWidget->removeNode(nodeId_p);
            this->ui.pCnInputWidget->removeNode(nodeId_p);
            this->ui.pCnOutputWidget->removeNode(nodeId_p);
            break;
    }
}
