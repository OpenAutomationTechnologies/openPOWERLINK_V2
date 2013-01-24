/**
********************************************************************************

  \file           Leds.cpp

  \brief          Implementation of the LED widget class

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

#include "Leds.h"

/**
********************************************************************************
\brief  constructor

Constructs a LED widget

\param  iCount_p        number of LEDs to show
\param  parent          pointer to parent widget
*******************************************************************************/
Leds::Leds(int iCount_p, QWidget *parent)
    : QWidget(parent)
{
    int nIdx;

    m_iCount = iCount_p;

    // ---------------------------------------------------------------------
    // Layout
    // ---------------------------------------------------------------------
    QHBoxLayout *pLedsLayout = new QHBoxLayout;
    setLayout(pLedsLayout);

    setContentsMargins(0, 0, 0, 0);

    // create array for pointers to LedButtons
    m_ppLedLabels = new QLabel*[iCount_p];

    m_pActiveLed  = new QPixmap(":/img/ledred.png");
    m_pInactiveLed = new QPixmap(":/img/ledgreen.png");

    for (nIdx = 0; nIdx < iCount_p; nIdx++)
    {
        m_ppLedLabels[nIdx] = new QLabel(parent);
        m_ppLedLabels[nIdx]->setPixmap(*m_pInactiveLed);
        pLedsLayout->addWidget(m_ppLedLabels[nIdx]);
    }

    pLedsLayout->update();
}

/**
********************************************************************************
\brief  set LEDs

setLeds() sets the leds according to the data value.

\param  uiDataIn_p      data value to show
*******************************************************************************/
void Leds::setLeds(unsigned int uiDataIn_p)
{
    int nIdx;

    for (nIdx=0; nIdx < m_iCount; nIdx++)
    {
        if (uiDataIn_p & (1 << nIdx))
        {
            m_ppLedLabels[nIdx]->setPixmap(*m_pActiveLed);
        }
        else
        {
            m_ppLedLabels[nIdx]->setPixmap(*m_pInactiveLed);
        }
    }
}


