/**
********************************************************************************

  \file           EplState.cpp

  \brief          Implementation of the EplState class

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

#include "EplState.h"
#include "Leds.h"

/******************************************************************************/
/* definitions */

/**
********************************************************************************
\brief  constructor

Constructs an EplState object

\param  parent          pointer to parent widget
*******************************************************************************/
EplState::EplState(QWidget *parent)
    : QWidget(parent)
{
    QFont LabelFont;
    LabelFont.setBold(true);
    LabelFont.setPointSize(18);

    pRedLed = new QPixmap(":/img/ledred.png");
    pYellowLed = new QPixmap(":/img/ledyellow.png");
    pGreenLed = new QPixmap(":/img/ledgreen.png");

    // ---------------------------------------------------------------------
    // Layout
    // ---------------------------------------------------------------------
    QHBoxLayout *pEplStateLayout = new QHBoxLayout;
    setLayout(pEplStateLayout);

    pEplStateLayout->addStretch(0);

    pNmtSectionLabel = new QLabel("NMT State:");
    pNmtSectionLabel->setFont(LabelFont);
    pEplStateLayout->addWidget(pNmtSectionLabel);
    pEplStateLayout->addSpacing(20);

    pEplStatusLed = new QLabel();
    pEplStatusLed->setPixmap(*pRedLed);
    pEplStateLayout->addSpacing(10);
    pEplStateLayout->addWidget(pEplStatusLed);

    pNmtStateLabel = new QLabel("Off");
    QFont tmpFont1("Arial", 16, QFont::Bold);
    pNmtStateLabel->setFont(tmpFont1);
    pEplStateLayout->addSpacing(10);
    pEplStateLayout->addWidget(pNmtStateLabel);

    pEplStateLayout->addStretch(1);

}

/**
********************************************************************************
\brief  set POWERLINK status LED

Sets the POWERLINK status LED depending on the POWERLINK state.

\param  iStatus_p       POWERLINK status
*******************************************************************************/
void EplState::setEplStatusLed(int iStatus_p)
{
    pEplStatusLed->show();
    switch (iStatus_p)
    {
        case 0:
        {
            pEplStatusLed->setPixmap(*pRedLed);
            break;
        }
        case 1:
        {
            pEplStatusLed->setPixmap(*pYellowLed);
            break;
        }
        case 2:
        {
            pEplStatusLed->setPixmap(*pGreenLed);
            break;
        }
        case -1:
        default:
        {
            pEplStatusLed->setPixmap(*pRedLed);
        }
    }
}

/**
********************************************************************************
\brief  sets the text to show

Sets the text to show for the POWERLINK state.

\param  strState_p       POWERLINK status text
*******************************************************************************/
void EplState::setNmtStateText(const QString &strState_p)
{
    pNmtStateLabel->show();
    pNmtStateLabel->setText(strState_p);
}




