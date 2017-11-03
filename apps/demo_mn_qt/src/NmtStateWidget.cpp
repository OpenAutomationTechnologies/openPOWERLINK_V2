/**
********************************************************************************
\file   NmtStateWidget.cpp

\brief  Implementation of the NMT state widget

This file contains the implementation of the NMT state widget.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, B&R Industrial Automation GmbH
Copyright (c) 2013, Kalycito Infotech Private Ltd.All rights reserved.
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
#include <NmtStateWidget.h>

#include <QHBoxLayout>
#include <QLabel>
#include <MultiColorLed.h>

//============================================================================//
//            P U B L I C    M E M B E R    F U N C T I O N S                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Constructor

Constructs an NMT state widget. The widget is used to show the POWERLINK NMT
state of a node.

\param[in]      pParent_p           Pointer to the parent widget.
*/
//------------------------------------------------------------------------------
NmtStateWidget::NmtStateWidget(QWidget* pParent_p) :
    QWidget(pParent_p)
{
    this->setupUi();

    this->hideNmtStateText();
    this->setNmtState(kNmtGsOff);
}

//------------------------------------------------------------------------------
/**
\brief  Setup the user interface

Initializes the GUI elements of the user interface
*/
//------------------------------------------------------------------------------
void NmtStateWidget::setupUi()
{
    // ---------------------------------------------------------------------
    // General widget settings
    // ---------------------------------------------------------------------
    this->setContentsMargins(0, 0, 0, 0);
    this->pWidgetLayout = new QHBoxLayout(this);

    // Current Node ID
    this->pWidgetLabel = new QLabel();
    this->pWidgetLabel->setFont(QFont("Arial", 18, QFont::Bold));
    this->pWidgetLayout->addWidget(this->pWidgetLabel);

    // Current NMT state (LED)
    this->pCurrentNmtStateLed = new MultiColorLed();
    this->pWidgetLayout->addWidget(this->pCurrentNmtStateLed);

    // Current NMT state (text)
    this->pCurrentNmtStateText = new QLabel();
    this->pCurrentNmtStateText->setFont(QFont("Arial", 16, QFont::Bold));
    this->pWidgetLayout->addWidget(this->pCurrentNmtStateText);

    // Add stretch to left-align the widgets
    this->pWidgetLayout->addStretch();
}

//------------------------------------------------------------------------------
/**
\brief  Set the widget label

Sets the widget label text.

\param[in]      text_p              Node ID text
*/
//------------------------------------------------------------------------------
void NmtStateWidget::setWidgetLabel(const QString& text_p)
{
    this->pWidgetLabel->setText(text_p);
}

//------------------------------------------------------------------------------
/**
\brief  Show the NMT state text

Shows the NMT state text.
*/
//------------------------------------------------------------------------------
void NmtStateWidget::showNmtStateText()
{
    this->pCurrentNmtStateText->show();
}

//------------------------------------------------------------------------------
/**
\brief  Hide the NMT state text

Hides the NMT state text.
*/
//------------------------------------------------------------------------------
void NmtStateWidget::hideNmtStateText()
{
    this->pCurrentNmtStateText->hide();
}

//------------------------------------------------------------------------------
/**
\brief  Set NMT state

Sets the NMT state of a node. Depending on the state a different image is shown.

\param[in]      nmtState_p          NMT state to be set
*/
//------------------------------------------------------------------------------
void NmtStateWidget::setNmtState(tNmtState nmtState_p)
{
    switch (nmtState_p)
    {
        case kNmtGsOff:
            this->pCurrentNmtStateText->setText(QString("Off"));
            this->pCurrentNmtStateLed->setColor(MultiColorLed::Red);
            break;

        case kNmtGsInitialising:
            this->pCurrentNmtStateText->setText(QString("Initializing"));
            this->pCurrentNmtStateLed->setColor(MultiColorLed::Red);
            break;

        case kNmtGsResetApplication:
            this->pCurrentNmtStateText->setText(QString("Reset Application"));
            this->pCurrentNmtStateLed->setColor(MultiColorLed::Red);
            break;

        case kNmtGsResetCommunication:
            this->pCurrentNmtStateText->setText(QString("Reset Communication"));
            this->pCurrentNmtStateLed->setColor(MultiColorLed::Red);
            break;

        case kNmtGsResetConfiguration:
            this->pCurrentNmtStateText->setText(QString("Reset Configuration"));
            this->pCurrentNmtStateLed->setColor(MultiColorLed::Red);
            break;

        // CN states
        case kNmtCsNotActive:
            this->pCurrentNmtStateText->setText(QString("CN Not Active"));
            this->pCurrentNmtStateLed->setColor(MultiColorLed::Red);
            break;

        case kNmtCsPreOperational1:
            this->pCurrentNmtStateText->setText(QString("CN Pre-Operational 1"));
            this->pCurrentNmtStateLed->setColor(MultiColorLed::Yellow);
            break;

        case kNmtCsPreOperational2:
            this->pCurrentNmtStateText->setText(QString("CN Pre-Operational 2"));
            this->pCurrentNmtStateLed->setColor(MultiColorLed::Yellow);
            break;

        case kNmtCsReadyToOperate:
            this->pCurrentNmtStateText->setText(QString("CN ReadyToOperate"));
            this->pCurrentNmtStateLed->setColor(MultiColorLed::Yellow);
            break;

        case kNmtCsOperational:
            this->pCurrentNmtStateText->setText(QString("CN Operational"));
            this->pCurrentNmtStateLed->setColor(MultiColorLed::Green);
            break;

        case kNmtCsStopped:
            this->pCurrentNmtStateText->setText(QString("CN Stopped"));
            this->pCurrentNmtStateLed->setColor(MultiColorLed::Red);
            break;

        case kNmtCsBasicEthernet:
            this->pCurrentNmtStateText->setText(QString("CN Basic Ethernet"));
            this->pCurrentNmtStateLed->setColor(MultiColorLed::Red);
            break;

        // MN states
        case kNmtMsNotActive:
        case kNmtRmsNotActive:
            this->pCurrentNmtStateText->setText(QString("MN Not Active"));
            this->pCurrentNmtStateLed->setColor(MultiColorLed::Red);
            break;

        case kNmtMsPreOperational1:
            this->pCurrentNmtStateText->setText(QString("MN Pre-Operational 1"));
            this->pCurrentNmtStateLed->setColor(MultiColorLed::Yellow);
            break;

        case kNmtMsPreOperational2:
            this->pCurrentNmtStateText->setText(QString("MN Pre-Operational 2"));
            this->pCurrentNmtStateLed->setColor(MultiColorLed::Yellow);
            break;

        case kNmtMsReadyToOperate:
            this->pCurrentNmtStateText->setText(QString("MN ReadyToOperate"));
            this->pCurrentNmtStateLed->setColor(MultiColorLed::Yellow);
            break;

        case kNmtMsOperational:
            this->pCurrentNmtStateText->setText("MN Operational");
            this->pCurrentNmtStateLed->setColor(MultiColorLed::Green);
            break;

        case kNmtMsBasicEthernet:
            this->pCurrentNmtStateText->setText(QString("CN Basic Ethernet"));
            this->pCurrentNmtStateLed->setColor(MultiColorLed::Red);
            break;

        default:
            this->pCurrentNmtStateText->setText(QString("Undefined"));
            this->pCurrentNmtStateLed->setColor(MultiColorLed::Red);
            break;
    }
}
