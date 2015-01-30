/**
********************************************************************************
\file   MainWindow.cpp

\brief  Implementation of main window class

This file contains the implementation of the main window class.
*******************************************************************************/
/*------------------------------------------------------------------------------
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

#include "MainWindow.h"
#include "State.h"
#include "Api.h"
#include "Input.h"
#include "Output.h"

#ifdef CONFIG_USE_PCAP
#include "InterfaceSelectDialog.h"
#endif

//============================================================================//
//            P U B L I C    M E M B E R    F U N C T I O N S                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  constructor

Constructor of main window class.

\param  parent      Pointer to the parent window
*/
//------------------------------------------------------------------------------
MainWindow::MainWindow(QWidget* parent)
    : QWidget(parent)
{
    pApi = NULL;

    resize(1000, 600);

    QVBoxLayout* pWindowLayout = new QVBoxLayout();
    pWindowLayout->setObjectName("MainLayout");
    setLayout(pWindowLayout);

    // ---------------------------------------------------------------------
    // Head Region
    // ---------------------------------------------------------------------
    pHeadRegion = new QHBoxLayout();
    pHeadRegion->setObjectName("HeadRegion");

    pLogo  = new QPixmap(":/img/powerlink_open_source.png");
    pLabel = new QLabel();
    pLabel->setPixmap(*pLogo);
    pHeadRegion->addWidget(pLabel, 1, Qt::AlignLeft);

    version = oplk_getVersion();
    QLabel* pVersion = new QLabel("openPOWERLINK MN QT Demo\nVersion "
                                  + QString::number(PLK_STACK_VER(version))+ "."
                                  + QString::number(PLK_STACK_REF(version)) + "."
                                  + QString::number(PLK_STACK_REL(version)));
    pHeadRegion->addWidget(pVersion, Qt::AlignCenter,Qt::AlignLeft);

    pWindowLayout->addLayout(pHeadRegion);
    pWindowLayout->addSpacing(10);

    pFrameSepHeadMiddle = new QFrame();
    pFrameSepHeadMiddle->setFrameStyle(QFrame::HLine);
    pWindowLayout->addWidget(pFrameSepHeadMiddle);

    // ---------------------------------------------------------------------
    // Middle Region
    // ---------------------------------------------------------------------

    // separate in left and right side
    QHBoxLayout* pMiddleRegion = new QHBoxLayout();
    pMiddleRegion->setObjectName("MiddleRegion");

    pCnState = new CnState;
    pMiddleRegion->addWidget(pCnState);

    pFrameSepMiddle = new QFrame();
    pFrameSepMiddle->setFrameStyle(QFrame::VLine);
    pMiddleRegion->addWidget(pFrameSepMiddle);

    pInput = new Input;
    pMiddleRegion->addWidget(pInput);

    pFrameSepMiddle2 = new QFrame();
    pFrameSepMiddle2->setFrameStyle(QFrame::VLine);
    pMiddleRegion->addWidget(pFrameSepMiddle2);


    pOutput = new Output;
    pMiddleRegion->addWidget(pOutput);

    pWindowLayout->addLayout(pMiddleRegion, 8);
    pWindowLayout->addStretch(10);

    pFrameSepMiddleStatus = new QFrame();
    pFrameSepMiddleStatus->setFrameStyle(QFrame::HLine);
    pWindowLayout->addWidget(pFrameSepMiddleStatus);

    // ---------------------------------------------------------------------
    // Status Region
    // ---------------------------------------------------------------------
    QHBoxLayout* pStatusRegion = new QHBoxLayout();
    pStatusRegion->setObjectName("StatusRegion");

    // POWERLINK network state
    pState = new State;
    pStatusRegion->addWidget(pState);

    pWindowLayout->addLayout(pStatusRegion, 0);

    pFrameSepStatusFoot = new QFrame();
    pFrameSepStatusFoot->setFrameStyle(QFrame::HLine);
    pWindowLayout->addWidget(pFrameSepStatusFoot);

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
    pNodeIdEdit = new QLineEdit(sNodeId);
    pNodeIdEdit->setValidator(pValidator);
    pFootRegion->addWidget(pNodeIdEdit);

    pFootRegion->addSpacing(20);

    pStartStopOplk = new QPushButton(tr("Start POWERLINK"));
    pFootRegion->addWidget(pStartStopOplk);
    connect(pStartStopOplk, SIGNAL(clicked()), this, SLOT(startPowerlink()));

    pFootRegion->addStretch();

    pToggleMax = new QPushButton(tr("Full Screen"));
    pFootRegion->addWidget(pToggleMax);
    connect(pToggleMax, SIGNAL(clicked()),
            this, SLOT(toggleWindowState()));

    QPushButton* pQuit = new QPushButton(tr("Quit"));
    pFootRegion->addWidget(pQuit);
    connect(pQuit, SIGNAL(clicked()), qApp, SLOT(quit()));

    pWindowLayout->addLayout(pFootRegion, 0);

    // ---------------------------------------------------------------------
    // Text Console Region
    // ---------------------------------------------------------------------
    QHBoxLayout* pTextRegion = new QHBoxLayout();
    pTextRegion->setObjectName("TextRegion");

    pTextEdit = new QTextEdit();
    pTextRegion->addWidget(pTextEdit);
    pTextEdit->setReadOnly(true);

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
    {
        pToggleMax->setText(tr("Window"));
    }
    else
    {
        pToggleMax->setText(tr("Full Screen"));
    }
}

//------------------------------------------------------------------------------
/**
\brief  Start POWERLINK

Starts the openPOWERLINK stack.
*/
//------------------------------------------------------------------------------
void MainWindow::startPowerlink()
{
    bool fConvOk;
    unsigned int nodeId;
    QString devName;

#if defined(CONFIG_USE_PCAP)
    // start the selection dialog
    InterfaceSelectDialog* pInterfaceDialog = new InterfaceSelectDialog();
    if (pInterfaceDialog->fillList() < 0)
    {
        QMessageBox::warning(this, "PCAP not working!",
                             "No PCAP interfaces found!\n"
                             "Make sure LibPcap is installed and you have root permissions!",
                             QMessageBox::Close);
        return;
    }

    if (pInterfaceDialog->exec() == QDialog::Rejected)
    {
        return;
    }

    devName = pInterfaceDialog->getDevName();
    delete pInterfaceDialog;
#else
    devName = "plk";
#endif

    pNodeIdEdit->setEnabled(false);
    nodeId = pNodeIdEdit->text().toUInt(&fConvOk);
    if (fConvOk == false)
    {
        nodeId = Api::defaultNodeId();
    }

    // change the button to stop
    pStartStopOplk->setText(tr("Stop POWERLINK"));
    pStartStopOplk->disconnect(this, SLOT(startPowerlink()));
    connect(pStartStopOplk, SIGNAL(clicked()), this, SLOT(stopPowerlink()));

    pApi = new Api(this, nodeId, devName);

}

//------------------------------------------------------------------------------
/**
\brief  Stop POWERLINK

Stops the openPOWERLINK stack.
*/
//------------------------------------------------------------------------------
void MainWindow::stopPowerlink()
{
    delete pApi;
    pStartStopOplk->setText(tr("Start POWERLINK"));
    pStartStopOplk->disconnect(this, SLOT(stopPowerlink()));
    connect(pStartStopOplk, SIGNAL(clicked()), this, SLOT(startPowerlink()));
    pNodeIdEdit->setEnabled(true);
}

//------------------------------------------------------------------------------
/**
\brief  Print a log entry

The function prints a log entry.

\param  str     String to print.
*/
//------------------------------------------------------------------------------
void MainWindow::printlog(QString str)
{
    pTextEdit->append(str);
}

