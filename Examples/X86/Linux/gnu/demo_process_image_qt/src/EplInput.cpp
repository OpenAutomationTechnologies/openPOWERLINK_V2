/**
********************************************************************************

  \file           EplInput.cpp

  \brief          Implementation of the EplInput widget

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

#include "EplInput.h"
#include "Leds.h"

/******************************************************************************/
/* definitions */


/******************************************************************************/
/* member functions */

/**
********************************************************************************
\brief  constructor

Constructs an EplInput widget.

\param  parent                  pointer to parent widget
*******************************************************************************/
EplInput::EplInput(QWidget *parent)
    : QWidget(parent)
{
    QFont LabelFont;
    LabelFont.setBold(true);
    LabelFont.setPointSize(18);

    // ---------------------------------------------------------------------
    // Layout
    // ---------------------------------------------------------------------
    pEplInputLayout = new QVBoxLayout;
    setLayout(pEplInputLayout);

    pEplInputLayout->addStretch(0);

    // process value LEDs at upper half
    QLabel* pDigiInLabel = new QLabel("Digital Inputs:");
    pDigiInLabel->setFont(LabelFont);
    pEplInputLayout->addWidget(pDigiInLabel);

    m_ppLeds = new Leds*[NODE_ID_MAX];
    for (int i = 0; i < NODE_ID_MAX; i++)
    {
        m_ppLeds[i] = new Leds(LED_NUM);
        m_ppLeds[i]->hide();
        pEplInputLayout->addWidget(m_ppLeds[i]);
    }

    pEplInputLayout->addStretch(1);
}

/**
********************************************************************************
\brief  set LEDs

Sets the input LEDs of a CN.

\param  uiDataIn_p      input data to show on LEDs
\param  iNodeId_p       node ID for which to set LEDs
*******************************************************************************/
void EplInput::setLeds(unsigned int uiDataIn_p, unsigned int iNodeId_p)
{
    m_ppLeds[iNodeId_p]->setLeds(uiDataIn_p);
}

/**
********************************************************************************
\brief  add a CN

Adds a controlled node to the node list.

\param          iNodeId_p               node ID of CN
*******************************************************************************/
void EplInput::addNode(int iNodeId_p)
{
    if ((iNodeId_p >= 0) && (iNodeId_p <= NODE_ID_MAX))
    {
        m_ppLeds[iNodeId_p]->show();
        //apNodes[iNodeId_p]->setFixedSize(NODE_WIDTH, NODE_HEIGHT);
        pEplInputLayout->update();
    }

}

/**
********************************************************************************
\brief  remove a CN

Removes a controlled node from the node list.

\param          iNodeId_p               node ID of CN
*******************************************************************************/
void EplInput::removeNode(int iNodeId_p)
{
    if ((iNodeId_p >= 0) && (iNodeId_p <= NODE_ID_MAX))
    {
        m_ppLeds[iNodeId_p]->hide();
    }
}

/**
********************************************************************************
\brief  remove all CNs

Removes all controlled nodes from the node list.
*******************************************************************************/
void EplInput::removeAllNodes()
{
    int nIdx;

    // count() gives all widgets (hidden ones too)
    for(nIdx=0; nIdx < NODE_ID_MAX; nIdx++)
    {
        m_ppLeds[nIdx]->hide();
    }
}
