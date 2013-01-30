/**
********************************************************************************

  \file           EplCnState.cpp

  \brief          Implementation of the EplCnState class

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

#include "EplCnState.h"
#include "NodeState.h"

/**
********************************************************************************
\brief  constructor

Constructs an EplCnState widget.

\param          parent                  pointer parent window
*******************************************************************************/
EplCnState::EplCnState(QWidget *parent)
    : QWidget(parent)
{
    QFont LabelFont;
    LabelFont.setBold(true);
    LabelFont.setPointSize(18);

    // ---------------------------------------------------------------------
    // Layout
    // ---------------------------------------------------------------------
    m_pEplStateLayout = new QVBoxLayout;
    setLayout(m_pEplStateLayout);

    m_pEplStateLayout->addStretch(0);

    // process value LEDs at upper half
    QLabel* pStateLabel = new QLabel("Node/State");
    pStateLabel->setFont(LabelFont);
    m_pEplStateLayout->addWidget(pStateLabel);

    m_ppNodeState = new NodeState*[NODE_ID_MAX];
    for (int i = 0; i < NODE_ID_MAX; i++)
    {
        m_ppNodeState[i] = new NodeState(QString("CN%1").arg(i));
        m_ppNodeState[i]->hide();
        m_pEplStateLayout->addWidget(m_ppNodeState[i]);
    }

    m_pEplStateLayout->addStretch(1);
}

/**
********************************************************************************
\brief  set CN state

Sets the state of the CN

\param          iNodeId_p               node ID of CN
\param          iState_p                state of CN
*******************************************************************************/
void EplCnState::setState(int iNodeId_p, int iState_p)
{
    m_ppNodeState[iNodeId_p]->setState(iState_p);
}

/**
********************************************************************************
\brief  add a CN

Adds a controlled node to the node list.

\param          iNodeId_p               node ID of CN
*******************************************************************************/
void EplCnState::addNode(int iNodeId_p)
{
    if ((iNodeId_p >= 0) && (iNodeId_p <= NODE_ID_MAX))
    {
        m_ppNodeState[iNodeId_p]->show();
        //apNodes[iNodeId_p]->setFixedSize(NODE_WIDTH, NODE_HEIGHT);
        m_pEplStateLayout->update();
    }

}

/**
********************************************************************************
\brief  remove a CN

Removes a controlled node from the node list.

\param          iNodeId_p               node ID of CN
*******************************************************************************/
void EplCnState::removeNode(int iNodeId_p)
{
    if ((iNodeId_p >= 0) && (iNodeId_p <= NODE_ID_MAX))
    {
        m_ppNodeState[iNodeId_p]->hide();
    }
}

/**
********************************************************************************
\brief  remove all CNs

Removes all controlled nodes from the node list.
*******************************************************************************/
void EplCnState::removeAllNodes()
{
    int nIdx;

    // count() gives all widgets (hidden ones too)
    for(nIdx=0; nIdx < NODE_ID_MAX; nIdx++)
    {
        m_ppNodeState[nIdx]->hide();
    }
}
