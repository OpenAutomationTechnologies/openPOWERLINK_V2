/**
********************************************************************************
\file   State.cpp

\brief  Implementation of State class

This file contains the implementation of the State class
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
#include <State.h>
#include <Leds.h>

#include <QHBoxLayout>
#include <QLabel>
#include <QToolButton>
#include <QPixmap>


//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------


//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C    M E M B E R    F U N C T I O N S                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  constructor

Constructs a State object

\param  parent          pointer to parent widget
*/
//------------------------------------------------------------------------------
State::State(QWidget* parent)
    : QWidget(parent)
{
    QFont LabelFont;
    LabelFont.setBold(true);
    LabelFont.setPointSize(18);

    pRedLed = new QPixmap(":/img/ledred.png");
    pYellowLed = new QPixmap(":/img/ledyellow.png");
    pGreenLed = new QPixmap(":/img/ledgreen.png");

    QHBoxLayout* pStateLayout = new QHBoxLayout;
    setLayout(pStateLayout);

    pStateLayout->addStretch(0);

    pNmtSectionLabel = new QLabel("NMT State:");
    pNmtSectionLabel->setFont(LabelFont);
    pStateLayout->addWidget(pNmtSectionLabel);
    pStateLayout->addSpacing(20);

    pStatusLed = new QLabel();
    pStatusLed->setPixmap(*pRedLed);
    pStateLayout->addSpacing(10);
    pStateLayout->addWidget(pStatusLed);

    pNmtStateLabel = new QLabel("Off");
    QFont tmpFont1("Arial", 16, QFont::Bold);
    pNmtStateLabel->setFont(tmpFont1);
    pStateLayout->addSpacing(10);
    pStateLayout->addWidget(pNmtStateLabel);

    pStateLayout->addStretch(1);

}

//------------------------------------------------------------------------------
/**
\brief  Set POWERLINK status LED

Sets the POWERLINK status LED depending on the POWERLINK state.

\param  status_p        POWERLINK status
*/
//------------------------------------------------------------------------------
void State::setStatusLed(int status_p)
{
    pStatusLed->show();
    switch (status_p)
    {
        case 0:
            pStatusLed->setPixmap(*pRedLed);
            break;

        case 1:
            pStatusLed->setPixmap(*pYellowLed);
            break;

        case 2:
            pStatusLed->setPixmap(*pGreenLed);
            break;

        case -1:
        default:
            pStatusLed->setPixmap(*pRedLed);
            break;
    }
}

//------------------------------------------------------------------------------
/**
\brief  Sets the text to show

Sets the text to show for the POWERLINK state.

\param  strState_p       POWERLINK status text
*/
//------------------------------------------------------------------------------
void State::setNmtStateText(const QString& strState_p)
{
    pNmtStateLabel->show();
    pNmtStateLabel->setText(strState_p);
}
