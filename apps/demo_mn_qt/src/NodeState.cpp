/**
********************************************************************************
\file   NodeState.cpp

\brief  Implementation of NodeState class

This file contains the implementation of the NodeState class.
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
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QToolButton>
#include <QPalette>
#include <QColor>
#include <QLabel>

#include "NodeState.h"

//============================================================================//
//            P U B L I C    M E M B E R    F U N C T I O N S                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Constructor

Constructs a NodeState widget. The NodeState widget is used to show the
POWERLINK state of a node.

\param  label_p     Label for the nodestate widget.
\param  parent_p    Pointer to the parent widget.
*/
//------------------------------------------------------------------------------
NodeState::NodeState(const QString& label_p, QWidget* parent_p)
    : QWidget(parent_p)
{
    pRedLed  = new QPixmap(":/img/ledred.png");
    pYellowLed = new QPixmap(":/img/ledyellow.png");
    pGreenLed = new QPixmap(":/img/ledgreen.png");

    // ---------------------------------------------------------------------
    // Layout
    // ---------------------------------------------------------------------
    QHBoxLayout* pStateLayout = new QHBoxLayout;
    setLayout(pStateLayout);

    setContentsMargins(0, 0, 0, 0);

    QLabel* pStateLabel = new QLabel(label_p);
    QFont tmpFont1("Arial", 18, QFont::Bold);
    pStateLabel->setFont(tmpFont1);
    pStateLayout->addWidget(pStateLabel);

    pStateLayout->addStretch();

    // create array for pointers to LedButtons
    pLedLabel = new QLabel();
    pLedLabel->setPixmap(*pRedLed);
    pStateLayout->addWidget(pLedLabel);

    pStateLayout->update();
}

//------------------------------------------------------------------------------
/**
\brief  set node state

Sets the state of a node. Depending on the state a different image is shown.

\param  state_p               state to be set
*/
//------------------------------------------------------------------------------
void NodeState::setState(int state_p)
{
    switch (state_p)
    {
        case 1:
            pLedLabel->setPixmap(*pYellowLed);
            break;

        case 2:
            pLedLabel->setPixmap(*pGreenLed);
            break;

        default:
            pLedLabel->setPixmap(*pRedLed);
            break;
    }
}

