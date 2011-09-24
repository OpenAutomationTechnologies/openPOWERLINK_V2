/**
********************************************************************************

  \file           MainWindow.h

  \brief          MainWindow class header of demo application

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

/******************************************************************************/
/* includes */
#include <QtGui>

#include "MainWindow.h"
#include "EplState.h"
#include "EplApi.h"
#include "EplInput.h"
#include "EplOutput.h"

#ifdef CONFIG_POWERLINK_USERSTACK
#include "InterfaceSelectDialog.h"
#endif

/******************************************************************************/
/* definitions */


/******************************************************************************/
/* member functions */

/**
********************************************************************************
\brief	constructor

Constructor of main window class.

\param		parent			pointer parent window
*******************************************************************************/
MainWindow::MainWindow(QWidget *parent)
    : QWidget(parent)
{
    pEplApi = NULL;

    resize(1000, 600);

    QVBoxLayout *pWindowLayout = new QVBoxLayout();
    pWindowLayout->setObjectName("MainLayout");
    setLayout(pWindowLayout);

    // ---------------------------------------------------------------------
    // Head Region
    // ---------------------------------------------------------------------
    pHeadBackground = new QFrame();
    pHeadBackground->setContentsMargins(0,0,0,0);
    QPalette tmpPal(pHeadBackground->palette());
    tmpPal.setColor(QPalette::Window, Qt::white);
    pHeadBackground->setPalette(tmpPal);
    pHeadBackground->setAutoFillBackground(true);

    pHeadRegion = new QHBoxLayout();
    pHeadRegion->setObjectName("HeadRegion");

    pRightLogo  = new QPixmap(":/img/powerlink_open_source.png");
    pRightLabel = new QLabel();
    pHeadRegion->addWidget(pRightLabel, 1, Qt::AlignLeft);

    pHeadBackground->setLayout(pHeadRegion);
    pWindowLayout->addWidget(pHeadBackground, 0);

    pWindowLayout->addSpacing(10);

    pFrameSepHeadMiddle = new QFrame();
    pFrameSepHeadMiddle->setFrameStyle(QFrame::HLine);
    pWindowLayout->addWidget(pFrameSepHeadMiddle);

    // ---------------------------------------------------------------------
    // Middle Region
    // ---------------------------------------------------------------------

    // separate in left and right side
    QHBoxLayout *pMiddleRegion = new QHBoxLayout();
    pMiddleRegion->setObjectName("MiddleRegion");

    pEplCnState = new EplCnState;
    pMiddleRegion->addWidget(pEplCnState);

    pFrameSepMiddle = new QFrame();
    pFrameSepMiddle->setFrameStyle(QFrame::VLine);
    pMiddleRegion->addWidget(pFrameSepMiddle);

    pEplInput = new EplInput;
    pMiddleRegion->addWidget(pEplInput);

    pFrameSepMiddle2 = new QFrame();
    pFrameSepMiddle2->setFrameStyle(QFrame::VLine);
    pMiddleRegion->addWidget(pFrameSepMiddle2);


    pEplOutput = new EplOutput;
    pMiddleRegion->addWidget(pEplOutput);

    pWindowLayout->addLayout(pMiddleRegion, 8);
    pWindowLayout->addStretch(10);

    pFrameSepMiddleStatus = new QFrame();
    pFrameSepMiddleStatus->setFrameStyle(QFrame::HLine);
    pWindowLayout->addWidget(pFrameSepMiddleStatus);

    // ---------------------------------------------------------------------
    // Status Region
    // ---------------------------------------------------------------------
    QHBoxLayout *pStatusRegion = new QHBoxLayout();
    pStatusRegion->setObjectName("StatusRegion");

    // EPL network state
    pEplState = new EplState;
    pStatusRegion->addWidget(pEplState);

    pWindowLayout->addLayout(pStatusRegion, 0);

    pFrameSepStatusFoot = new QFrame();
    pFrameSepStatusFoot->setFrameStyle(QFrame::HLine);
    pWindowLayout->addWidget(pFrameSepStatusFoot);

    // ---------------------------------------------------------------------
    // Foot Region
    // ---------------------------------------------------------------------

    QHBoxLayout *pFootRegion = new QHBoxLayout();
    pFootRegion->setObjectName("FootRegion");

    QLabel* pNodeIdLabel = new QLabel("Node-ID:");
    pFootRegion->addWidget(pNodeIdLabel);

    QString sNodeId;
    sNodeId.setNum(EplApi::defaultNodeId());

    QValidator *pValidator = new QIntValidator(1, 254, this);
    pNodeIdEdit = new QLineEdit(sNodeId);
    pNodeIdEdit->setValidator(pValidator);
    pFootRegion->addWidget(pNodeIdEdit);

    pFootRegion->addSpacing(20);

    pStartStopEpl = new QPushButton(tr("Start POWERLINK"));
    pFootRegion->addWidget(pStartStopEpl);
    connect(pStartStopEpl, SIGNAL(clicked()), this, SLOT(startPowerlink()));

    pFootRegion->addStretch();

    pToggleMax = new QPushButton(tr("Full Screen"));
    pFootRegion->addWidget(pToggleMax);
    connect(pToggleMax, SIGNAL(clicked()),
            this, SLOT(toggleWindowState()));

    QPushButton *pQuit = new QPushButton(tr("Quit"));
    pFootRegion->addWidget(pQuit);
    connect(pQuit, SIGNAL(clicked()), qApp, SLOT(quit()));

    pWindowLayout->addLayout(pFootRegion, 0);
}

/**
********************************************************************************
\brief  toggle window state

toggleWindowState() toggles between fullscreen and normal window state.
*******************************************************************************/
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

//---------------------------------------------------------------------------
// Function:            resizeEvent
//
// Description:         handler for resize events
//
// Parameters:          event           pointer to resize event
//
// Returns:             N/A
//---------------------------------------------------------------------------
/**
********************************************************************************
\brief  handle resize event

Handle the resize Events.

\param  event           pointer to resize event
*******************************************************************************/
void MainWindow::resizeEvent(QResizeEvent *event)
{
    QWidget::resizeEvent(event);

    resizeHeadRegion();
}

/**
********************************************************************************
\brief  resize head region

resizes the head region of the window
*******************************************************************************/
void MainWindow::resizeHeadRegion()
{
int iWidth;
int iPreferredWidth;
static int iPrevWidth = 0;

    iWidth = pHeadBackground->width();

    if (iPrevWidth != iWidth)
    {
        // calculate the remaining size for the left and right logo
        // considering the recommended width of the middle label and some space
        // (the recommended and not the actual width of the middle label
        // has to be used, because the latter one is no up to date on the
        // first calls to this function)
        iPreferredWidth = (iWidth - 60) / 4;

        pRightLabel->setPixmap(pRightLogo->scaledToWidth(iPreferredWidth, Qt::SmoothTransformation));
        iPrevWidth = iWidth;
        pHeadRegion->update();
    }
}

/**
********************************************************************************
\brief  start POWERLINK

Starts the openPOWERLINK stack.
*******************************************************************************/
void MainWindow::startPowerlink()
{
    bool fConvOk;
    unsigned int uiNodeId;
    QString devName;

#ifdef CONFIG_POWERLINK_USERSTACK
    // start the selection dialog
    InterfaceSelectDialog *pInterfaceDialog = new InterfaceSelectDialog();
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
    devName = "epl";
#endif

    pNodeIdEdit->setEnabled(false);
    uiNodeId = pNodeIdEdit->text().toUInt(&fConvOk);
    if (fConvOk == false)
    {
        uiNodeId = EplApi::defaultNodeId();
    }

    // change the button to stop
    pStartStopEpl->setText(tr("Stop POWERLINK"));
    pStartStopEpl->disconnect(this, SLOT(startPowerlink()));
    connect(pStartStopEpl, SIGNAL(clicked()), this, SLOT(stopPowerlink()));

    pEplApi = new EplApi(this, uiNodeId, devName);

}

/**
********************************************************************************
\brief  stop POWERLINK

Stops the openPOWERLINK stack.
*******************************************************************************/
void MainWindow::stopPowerlink()
{
    delete pEplApi;
    pStartStopEpl->setText(tr("Start POWERLINK"));
    pStartStopEpl->disconnect(this, SLOT(stopPowerlink()));
    connect(pStartStopEpl, SIGNAL(clicked()), this, SLOT(startPowerlink()));
    pNodeIdEdit->setEnabled(true);
}



