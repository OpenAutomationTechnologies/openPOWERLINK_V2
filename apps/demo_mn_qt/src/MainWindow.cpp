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


#include <NmtStateWidget.h>
#include <IoWidget.h>
#include <CnListWidget.h>
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

\param[in,out]  pParent_p           Pointer to the parent window
*/
//------------------------------------------------------------------------------
MainWindow::MainWindow(QWidget* pParent_p) :
    QWidget(pParent_p),
    stackIsRunning(false)
{
    // Initialize
    this->pApi = NULL;
    this->pSdoDialog = NULL;
    this->nmtEvent = kNmtEventResetNode;

    // Setup UI elements
    this->setupUi();

    // Set dynamic GUI information
    UINT32 oplkVersion = oplk_getVersion();
    QString versionString = QString("openPOWERLINK MN QT Demo\nVersion " +
                            QString::number(PLK_STACK_VER(oplkVersion)) + "." +
                            QString::number(PLK_STACK_REF(oplkVersion)) + "." +
                            QString::number(PLK_STACK_REL(oplkVersion)));
    this->pVersionLabel->setText(versionString);

    this->pNodeIdEdit->setText(QString::number(Api::defaultNodeId()));

    this->pNmtStateWidget->showNmtStateText();
}

//------------------------------------------------------------------------------
/**
\brief  Setup the user interface

Initializes the GUI elements of the user interface
*/
//------------------------------------------------------------------------------
void MainWindow::setupUi()
{
    // ---------------------------------------------------------------------
    // General window settings
    // ---------------------------------------------------------------------
    this->resize(1000, 600);

    this->pWindowLayout = new QVBoxLayout(this);
    this->pWindowLayout->setObjectName("MainLayout");

    // ---------------------------------------------------------------------
    // Head Region
    // ---------------------------------------------------------------------
    this->pHeadRegionLayout = new QHBoxLayout();
    this->pHeadRegionLayout->setObjectName("HeadRegion");

    // Logo
    this->pLogo = new QLabel();
    this->pLogo->setPixmap(QPixmap(":/img/powerlink_open_source.png"));
    this->pHeadRegionLayout->addWidget(this->pLogo, 1, Qt::AlignLeft);

    // Version information
    this->pVersionLabel = new QLabel();
    this->pHeadRegionLayout->addWidget(this->pVersionLabel, Qt::AlignCenter, Qt::AlignLeft);

    // Add region to layout
    this->pWindowLayout->addLayout(this->pHeadRegionLayout);
    this->pWindowLayout->addSpacing(10);

    // ---------------------------------------------------------------------
    // Separator line
    // ---------------------------------------------------------------------
    this->pFrameSepHeadMiddle = new QFrame();
    this->pFrameSepHeadMiddle->setFrameStyle(QFrame::HLine);
    this->pWindowLayout->addWidget(this->pFrameSepHeadMiddle);

    // ---------------------------------------------------------------------
    // Middle Region
    // ---------------------------------------------------------------------
    this->pMiddleRegion = new QHBoxLayout();
    this->pMiddleRegion->setObjectName("MiddleRegion");

    // CN state region
    this->pCnStateRegion = new QWidget();
    this->pCnStateLayout = new QVBoxLayout();
    this->pCnStateWidgetLabel = new QLabel("Node / NMT State:");
    this->pCnStateWidgetLabel->setFont(QFont("Arial", 18, QFont::Bold));
    this->pCnStateLayout->addWidget(this->pCnStateWidgetLabel);
    this->pCnStateWidget = new CnListWidget();
    this->pCnStateLayout->addWidget(this->pCnStateWidget);
    this->pCnStateRegion->setLayout(this->pCnStateLayout);
    this->pMiddleRegion->addWidget(this->pCnStateRegion);

    // Vertical separator line
    this->pFrameSepMiddle = new QFrame();
    this->pFrameSepMiddle->setFrameStyle(QFrame::VLine);
    this->pMiddleRegion->addWidget(this->pFrameSepMiddle);

    // CN input region
    this->pCnInputRegion = new QWidget();
    this->pCnInputLayout = new QVBoxLayout();
    this->pCnInputWidgetLabel = new QLabel("Digital Inputs:");
    this->pCnInputWidgetLabel->setFont(QFont("Arial", 18, QFont::Bold));
    this->pCnInputLayout->addWidget(this->pCnInputWidgetLabel);
    this->pInputWidget = new IoWidget();
    this->pCnInputLayout->addWidget(this->pInputWidget);
    this->pCnInputRegion->setLayout(this->pCnInputLayout);
    this->pMiddleRegion->addWidget(this->pCnInputRegion);

    // Vertical separator line
    this->pFrameSepMiddle2 = new QFrame();
    this->pFrameSepMiddle2->setFrameStyle(QFrame::VLine);
    this->pMiddleRegion->addWidget(this->pFrameSepMiddle2);

    // CN output region
    this->pCnOutputRegion = new QWidget();
    this->pCnOutputLayout = new QVBoxLayout();
    this->pCnOutputWidgetLabel = new QLabel("Digital Outputs:");
    this->pCnOutputWidgetLabel->setFont(QFont("Arial", 18, QFont::Bold));
    this->pCnOutputLayout->addWidget(this->pCnOutputWidgetLabel);
    this->pOutputWidget = new IoWidget();
    this->pCnOutputLayout->addWidget(this->pOutputWidget);
    this->pCnOutputRegion->setLayout(this->pCnOutputLayout);
    this->pMiddleRegion->addWidget(this->pCnOutputRegion);

    // Add region to layout
    this->pWindowLayout->addLayout(this->pMiddleRegion, 8);
    this->pWindowLayout->addStretch(10);

    // ---------------------------------------------------------------------
    // Separator line
    // ---------------------------------------------------------------------
    this->pFrameSepMiddleStatus = new QFrame();
    this->pFrameSepMiddleStatus->setFrameStyle(QFrame::HLine);
    this->pWindowLayout->addWidget(this->pFrameSepMiddleStatus);

    // ---------------------------------------------------------------------
    // Status Region
    // ---------------------------------------------------------------------
    this->pStatusRegion = new QHBoxLayout();
    this->pStatusRegion->setObjectName("StatusRegion");

    // NMT state label
    this->pNmtStateLabel = new QLabel("NMT State:");
    this->pNmtStateLabel->setFont(QFont("Arial", 18, QFont::Bold));
    this->pStatusRegion->addWidget(this->pNmtStateLabel);

    // Current node network state
    this->pNmtStateWidget = new NmtStateWidget();
    this->pStatusRegion->addWidget(this->pNmtStateWidget);

    // Add region to layout
    this->pWindowLayout->addLayout(this->pStatusRegion, 0);

    // ---------------------------------------------------------------------
    // Separator line
    // ---------------------------------------------------------------------
    this->pFrameSepStatusFoot = new QFrame();
    this->pFrameSepStatusFoot->setFrameStyle(QFrame::HLine);
    this->pWindowLayout->addWidget(this->pFrameSepStatusFoot);

    // ---------------------------------------------------------------------
    // Foot Region
    // ---------------------------------------------------------------------
    this->pFootRegion = new QHBoxLayout();
    this->pFootRegion->setObjectName("FootRegion");

    // Node ID label
    this->pNodeIdLabel = new QLabel("Node-ID:");
    this->pFootRegion->addWidget(this->pNodeIdLabel);

    // Node ID edit
    this->pNodeIdEdit = new QLineEdit();
    this->pNodeIdEdit->setValidator(new QIntValidator(1, 254, this->pNodeIdEdit));
    this->pFootRegion->addWidget(this->pNodeIdEdit);

    // Add spacing
    this->pFootRegion->addSpacing(20);

    // Add start/stop button
    this->pStartStopOplk = new QPushButton(tr("Start POWERLINK"));
    this->pFootRegion->addWidget(this->pStartStopOplk);
    QObject::connect(this->pStartStopOplk,
                     SIGNAL(clicked()),
                     this,
                     SLOT(startStopStack()));

    // Add SDO dialog button
    this->pShowSdoDialog = new QPushButton(tr("SDO..."));
    this->pFootRegion->addWidget(this->pShowSdoDialog);
    QObject::connect(this->pShowSdoDialog,
                     SIGNAL(clicked()),
                     this,
                     SLOT(showSdoDialog()));

    // Add NMT command button
    this->pNmtCmd = new QPushButton(tr("Exec NMT command"));
    this->pNmtCmd->setEnabled(false);
    this->pFootRegion->addWidget(this->pNmtCmd);
    QObject::connect(this->pNmtCmd,
                     SIGNAL(clicked()),
                     this,
                     SLOT(execNmtCmd()));

    // Add a stretch
    this->pFootRegion->addStretch();

    // Add full screen button
    this->pToggleMax = new QPushButton(tr("Full Screen"));
    this->pFootRegion->addWidget(this->pToggleMax);
    QObject::connect(this->pToggleMax,
                     SIGNAL(clicked()),
                     this,
                     SLOT(toggleWindowState()));

    // Quit button
    this->pQuitButton = new QPushButton(tr("Quit"));
    this->pFootRegion->addWidget(this->pQuitButton);
    QObject::connect(this->pQuitButton,
                     SIGNAL(clicked()),
                     qApp,
                     SLOT(quit()));

    // Add region to layout
    this->pWindowLayout->addLayout(this->pFootRegion, 0);

    // ---------------------------------------------------------------------
    // Text Console Region
    // ---------------------------------------------------------------------
    this->pTextConsoleRegion = new QHBoxLayout();
    this->pTextConsoleRegion->setObjectName("TextRegion");

    // Text edit
    this->pTextEdit = new QTextEdit();
    this->pTextConsoleRegion->addWidget(this->pTextEdit);
    this->pTextEdit->setReadOnly(true);

    // Add region to layout
    this->pWindowLayout->addLayout(this->pTextConsoleRegion, 0);
}

//------------------------------------------------------------------------------
/**
\brief  Toggle window state

toggleWindowState() toggles between fullscreen and normal window state.
*/
//------------------------------------------------------------------------------
void MainWindow::toggleWindowState()
{
    this->setWindowState(this->windowState() ^ Qt::WindowFullScreen);

    if (this->windowState() & Qt::WindowFullScreen)
        this->pToggleMax->setText(tr("Window"));
    else
        this->pToggleMax->setText(tr("Full Screen"));
}

//------------------------------------------------------------------------------
/**
\brief  Start or stop POWERLINK

Starts or stops the openPOWERLINK stack.
*/
//------------------------------------------------------------------------------
void MainWindow::startStopStack()
{
    if (!this->stackIsRunning)
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

    // Read the node ID
    nodeId = this->pNodeIdEdit->text().toUInt(&fConvOk);
    if (fConvOk == false)
        nodeId = Api::defaultNodeId();

    // Update GUI elements to started stack
    this->pNodeIdEdit->setEnabled(false);
    this->pNmtCmd->setEnabled(true);
    this->pStartStopOplk->setText(tr("Stop POWERLINK"));

    // Start the stack
    this->pApi = new Api(this, nodeId, this->devName);

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

    this->stackIsRunning = true;
}

//------------------------------------------------------------------------------
/**
\brief  Stop POWERLINK

Stops the openPOWERLINK stack.
*/
//------------------------------------------------------------------------------
void MainWindow::stopPowerlink()
{
    this->stackIsRunning = false;

    // Stop the stack
    delete this->pApi;

    // Update GUI elements to stopped stack
    this->pStartStopOplk->setText(tr("Start POWERLINK"));
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
