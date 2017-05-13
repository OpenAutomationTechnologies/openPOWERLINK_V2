/**
********************************************************************************
\file   MainWindow.cpp

\brief  Implementation of main window class

This file contains the implementation of the main window class.
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

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <QtGui>
#include <MainWindow.h>

#include <QHBoxLayout>
#include <QPixmap>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QFrame>
#include <QTextEdit>
#include <QMessageBox>


#include <State.h>
#include <Output.h>
#include <Input.h>
#include <CnState.h>
#include <Api.h>
#include <SdoDialog.h>
#include <NmtCommandDialog.h>

#if defined(CONFIG_USE_PCAP)
#include <InterfaceSelectDialog.h>
#endif


//============================================================================//
//            P U B L I C    M E M B E R    F U N C T I O N S                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  constructor

Constructor of main window class.

\param[in,out]  parent              Pointer to the parent window
*/
//------------------------------------------------------------------------------
MainWindow::MainWindow(QWidget* parent)
    : QWidget(parent)
{
    this->pApi = NULL;
    this->pSdoDialog = NULL;
    this->nmtEvent = kNmtEventResetNode;

    resize(1000, 600);

    QVBoxLayout* pWindowLayout = new QVBoxLayout();
    pWindowLayout->setObjectName("MainLayout");
    setLayout(pWindowLayout);

    // ---------------------------------------------------------------------
    // Head Region
    // ---------------------------------------------------------------------
    this->pHeadRegion = new QHBoxLayout();
    this->pHeadRegion->setObjectName("HeadRegion");

    this->pLogo = new QPixmap(":/img/powerlink_open_source.png");
    this->pLabel = new QLabel();
    this->pLabel->setPixmap(*pLogo);
    this->pHeadRegion->addWidget(this->pLabel, 1, Qt::AlignLeft);

    this->version = oplk_getVersion();
    QLabel* pVersion = new QLabel("openPOWERLINK MN QT Demo\nVersion " +
                                   QString::number(PLK_STACK_VER(this->version)) + "." +
                                   QString::number(PLK_STACK_REF(this->version)) + "." +
                                   QString::number(PLK_STACK_REL(this->version)));
    this->pHeadRegion->addWidget(pVersion, Qt::AlignCenter,Qt::AlignLeft);

    pWindowLayout->addLayout(this->pHeadRegion);
    pWindowLayout->addSpacing(10);

    this->pFrameSepHeadMiddle = new QFrame();
    this->pFrameSepHeadMiddle->setFrameStyle(QFrame::HLine);
    pWindowLayout->addWidget(this->pFrameSepHeadMiddle);

    // ---------------------------------------------------------------------
    // Middle Region
    // ---------------------------------------------------------------------

    // separate in left and right side
    QHBoxLayout* pMiddleRegion = new QHBoxLayout();
    pMiddleRegion->setObjectName("MiddleRegion");

    this->pCnState = new CnState;
    pMiddleRegion->addWidget(this->pCnState);

    pFrameSepMiddle = new QFrame();
    pFrameSepMiddle->setFrameStyle(QFrame::VLine);
    pMiddleRegion->addWidget(pFrameSepMiddle);

    this->pInput = new Input;
    pMiddleRegion->addWidget(this->pInput);

    this->pFrameSepMiddle2 = new QFrame();
    this->pFrameSepMiddle2->setFrameStyle(QFrame::VLine);
    pMiddleRegion->addWidget(this->pFrameSepMiddle2);


    this->pOutput = new Output;
    pMiddleRegion->addWidget(this->pOutput);

    pWindowLayout->addLayout(pMiddleRegion, 8);
    pWindowLayout->addStretch(10);

    this->pFrameSepMiddleStatus = new QFrame();
    this->pFrameSepMiddleStatus->setFrameStyle(QFrame::HLine);
    pWindowLayout->addWidget(this->pFrameSepMiddleStatus);

    // ---------------------------------------------------------------------
    // Status Region
    // ---------------------------------------------------------------------
    QHBoxLayout* pStatusRegion = new QHBoxLayout();
    pStatusRegion->setObjectName("StatusRegion");

    // POWERLINK network state
    this->pState = new State;
    pStatusRegion->addWidget(this->pState);

    pWindowLayout->addLayout(pStatusRegion, 0);

    this->pFrameSepStatusFoot = new QFrame();
    this->pFrameSepStatusFoot->setFrameStyle(QFrame::HLine);
    pWindowLayout->addWidget(this->pFrameSepStatusFoot);

    // ---------------------------------------------------------------------
    // Foot Region
    // ---------------------------------------------------------------------
    QHBoxLayout* pFootRegion = new QHBoxLayout();
    pFootRegion->setObjectName("FootRegion");

    QLabel* pNodeIdLabel = new QLabel("Node-ID:");
    pFootRegion->addWidget(pNodeIdLabel);

    QString sNodeId;
    sNodeId.setNum(Api::defaultNodeId());

    QValidator* pValidator = new QIntValidator(1, 254, this);
    this->pNodeIdEdit = new QLineEdit(sNodeId);
    this->pNodeIdEdit->setValidator(pValidator);
    pFootRegion->addWidget(this->pNodeIdEdit);

    pFootRegion->addSpacing(20);

    this->pStartStopOplk = new QPushButton(tr("Start POWERLINK"));
    pFootRegion->addWidget(this->pStartStopOplk);
    connect(this->pStartStopOplk,
            SIGNAL(clicked()),
            this,
            SLOT(startPowerlink()));

    this->pShowSdoDialog = new QPushButton(tr("SDO..."));
    pFootRegion->addWidget(this->pShowSdoDialog);
    connect(this->pShowSdoDialog,
            SIGNAL(clicked()),
            this,
            SLOT(showSdoDialog()));

    this->pNmtCmd = new QPushButton(tr("Exec NMT command"));
    this->pNmtCmd->setEnabled(false);
    pFootRegion->addWidget(this->pNmtCmd);
    connect(this->pNmtCmd,
            SIGNAL(clicked()),
            this,
            SLOT(execNmtCmd()));

    pFootRegion->addStretch();

    this->pToggleMax = new QPushButton(tr("Full Screen"));
    pFootRegion->addWidget(this->pToggleMax);
    connect(this->pToggleMax,
            SIGNAL(clicked()),
            this,
            SLOT(toggleWindowState()));

    QPushButton* pQuit = new QPushButton(tr("Quit"));
    pFootRegion->addWidget(pQuit);
    connect(pQuit,
            SIGNAL(clicked()),
            qApp,
            SLOT(quit()));

    pWindowLayout->addLayout(pFootRegion, 0);

    // ---------------------------------------------------------------------
    // Text Console Region
    // ---------------------------------------------------------------------
    QHBoxLayout* pTextRegion = new QHBoxLayout();
    pTextRegion->setObjectName("TextRegion");

    this->pTextEdit = new QTextEdit();
    pTextRegion->addWidget(this->pTextEdit);
    this->pTextEdit->setReadOnly(true);

    pWindowLayout->addLayout(pTextRegion, 0);
}

//------------------------------------------------------------------------------
/**
\brief  Toggle window state

toggleWindowState() toggles between fullscreen and normal window state.
*/
//------------------------------------------------------------------------------
void MainWindow::toggleWindowState()
{
    setWindowState(windowState() ^ Qt::WindowFullScreen);
    if (windowState() & Qt::WindowFullScreen)
        this->pToggleMax->setText(tr("Window"));
    else
        this->pToggleMax->setText(tr("Full Screen"));
}

//------------------------------------------------------------------------------
/**
\brief  Start POWERLINK

Starts the openPOWERLINK stack.
*/
//------------------------------------------------------------------------------
void MainWindow::startPowerlink()
{
    bool            fConvOk;
    unsigned int    nodeId;

#if defined(CONFIG_USE_PCAP)
    // start the selection dialog
    InterfaceSelectDialog* pInterfaceDialog = new InterfaceSelectDialog();
    if (pInterfaceDialog->fillList(this->devName) < 0)
    {
        QMessageBox::warning(this,
                             "PCAP not working!",
                             "No PCAP interfaces found!\n"
                             "Make sure LibPcap is installed and you have root permissions!",
                             QMessageBox::Close);
        return;
    }

    if (pInterfaceDialog->exec() == QDialog::Rejected)
        return;

    this->devName = pInterfaceDialog->getDevName();
    delete pInterfaceDialog;
#else
    this->devName = "plk";
#endif

    this->pNodeIdEdit->setEnabled(false);
    nodeId = this->pNodeIdEdit->text().toUInt(&fConvOk);
    if (fConvOk == false)
        nodeId = Api::defaultNodeId();

    this->pNmtCmd->setEnabled(true);

    // change the button to stop
    this->pStartStopOplk->setText(tr("Stop POWERLINK"));
    this->pStartStopOplk->disconnect(this, SLOT(startPowerlink()));
    connect(this->pStartStopOplk,
            SIGNAL(clicked()),
            this,
            SLOT(stopPowerlink()));

    this->pApi = new Api(this, nodeId, devName);

    if (pSdoDialog)
    {
        QObject::connect(this->pApi,
                         SIGNAL(userDefEvent(void*)),
                         pSdoDialog,
                         SLOT(userDefEvent(void*)),
                         Qt::DirectConnection);
        QObject::connect(this->pApi,
                         SIGNAL(sdoFinished(tSdoComFinished)),
                         pSdoDialog,
                         SLOT(sdoFinished(tSdoComFinished)));
    }
}

//------------------------------------------------------------------------------
/**
\brief  Stop POWERLINK

Stops the openPOWERLINK stack.
*/
//------------------------------------------------------------------------------
void MainWindow::stopPowerlink()
{
    delete this->pApi;

    this->pStartStopOplk->setText(tr("Start POWERLINK"));
    this->pStartStopOplk->disconnect(this, SLOT(stopPowerlink()));
    connect(this->pStartStopOplk,
            SIGNAL(clicked()),
            this,
            SLOT(startPowerlink()));

    this->pNodeIdEdit->setEnabled(true);
    this->pNmtCmd->setEnabled(false);
}

//------------------------------------------------------------------------------
/**
\brief  Execute NMT command dialog

Execute NMT command/event entered in dialog.
*/
//------------------------------------------------------------------------------
void MainWindow::execNmtCmd()
{
    NmtCommandDialog* pDialog = new NmtCommandDialog(this->nmtEvent);

    if (pDialog->exec() == QDialog::Rejected)
    {
        delete pDialog;
        return;
    }

    this->nmtEvent = pDialog->getNmtEvent();
    delete pDialog;

    if (this->nmtEvent == kNmtEventNoEvent)
        return;

    oplk_execNmtCommand(this->nmtEvent);
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
        this->pSdoDialog = new SdoDialog();
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
\brief  Print a log entry

The function prints a log entry.

\param[in]      str                 String to print.
*/
//------------------------------------------------------------------------------
void MainWindow::printlog(const QString& str)
{
    this->pTextEdit->append(str);
}
