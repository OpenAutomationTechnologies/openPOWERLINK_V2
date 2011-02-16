/**
********************************************************************************

  \file           NodeState.cpp

  \brief          Implementation of the NodeState class

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
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QToolButton>
#include <QPalette>
#include <QColor>
#include <QLabel>

#include "NodeState.h"

/**
********************************************************************************
\brief  constructor

Constructs a NodeState widget. The NodeState widget is used to show the
POWERLINK state of a node.
*******************************************************************************/
NodeState::NodeState(const QString &label_p, QWidget *parent)
    : QWidget(parent)
{
    m_pRedLed  = new QPixmap(":/img/ledred.png");
    m_pYellowLed = new QPixmap(":/img/ledyellow.png");
    m_pGreenLed = new QPixmap(":/img/ledgreen.png");

    // ---------------------------------------------------------------------
    // Layout
    // ---------------------------------------------------------------------
    QHBoxLayout *pStateLayout = new QHBoxLayout;
    setLayout(pStateLayout);

    setContentsMargins(0, 0, 0, 0);

    QLabel *pStateLabel = new QLabel(label_p);
    QFont tmpFont1("Arial", 18, QFont::Bold);
    pStateLabel->setFont(tmpFont1);
    pStateLayout->addWidget(pStateLabel);

    pStateLayout->addStretch();

    // create array for pointers to LedButtons
    m_pLedLabel = new QLabel();
    m_pLedLabel->setPixmap(*m_pRedLed);
    pStateLayout->addWidget(m_pLedLabel);

    pStateLayout->update();
}

/**
********************************************************************************
\brief  set node state

Sets the state of a node. Depending on the state a different image is shown.

\param  uiState_p               state to be set
*******************************************************************************/
void NodeState::setState(unsigned int uiState_p)
{
    switch (uiState_p)
    {
        case 1:
            m_pLedLabel->setPixmap(*m_pYellowLed);
            break;
        case 2:
            m_pLedLabel->setPixmap(*m_pGreenLed);
            break;
        default:
            m_pLedLabel->setPixmap(*m_pRedLed);
            break;
    }
}


