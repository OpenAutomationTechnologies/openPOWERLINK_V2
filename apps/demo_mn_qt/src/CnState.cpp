/**
********************************************************************************
\file   CnState.cpp

\brief  openPOWERLINK CnState class

This file implements the CnState class.
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
#include <CnState.h>
#include <NodeState.h>

#include <QVBoxLayout>
#include <QLabel>

//------------------------------------------------------------------------------
/**
\brief  Constructor

Constructs an CnState widget.

\param[in]      parent              Pointer to the parent window
*/
//------------------------------------------------------------------------------
CnState::CnState(QWidget* parent)
    : QWidget(parent)
{
    QFont   LabelFont;

    LabelFont.setBold(true);
    LabelFont.setPointSize(18);

    this->pStateLayout = new QVBoxLayout;
    setLayout(this->pStateLayout);

    this->pStateLayout->addStretch(0);

    // process value LEDs at upper half
    QLabel* pStateLabel = new QLabel("Node/State");
    pStateLabel->setFont(LabelFont);
    this->pStateLayout->addWidget(pStateLabel);

    this->ppNodeState = new NodeState*[NODE_ID_MAX];
    for (int i = 0; i < NODE_ID_MAX; i++)
    {
        this->ppNodeState[i] = new NodeState(QString("CN%1").arg(i));
        this->ppNodeState[i]->hide();
        this->pStateLayout->addWidget(this->ppNodeState[i]);
    }

    this->pStateLayout->addStretch(1);
}

//------------------------------------------------------------------------------
/**
\brief  Set CN state

Sets the state of the CN

\param[in]      nodeId_p            Node ID of CN
\param[in]      state_p             State of CN
*/
//------------------------------------------------------------------------------
void CnState::setState(int nodeId_p,
                       int state_p)
{
    this->ppNodeState[nodeId_p]->setState(state_p);
}

//------------------------------------------------------------------------------
/**
\brief  Add a CN

Adds a controlled node to the node list.

\param[in]      nodeId_p            Node ID of CN
*/
//------------------------------------------------------------------------------
void CnState::addNode(int nodeId_p)
{
    if ((nodeId_p >= 0) &&
        (nodeId_p <= NODE_ID_MAX))
    {
        this->ppNodeState[nodeId_p]->show();
        this->pStateLayout->update();
    }
}

//------------------------------------------------------------------------------
/**
\brief  Remove a CN

Removes a controlled node from the node list.

\param[in]      nodeId_p            Node ID of CN
*/
//------------------------------------------------------------------------------
void CnState::removeNode(int nodeId_p)
{
    if ((nodeId_p >= 0) &&
        (nodeId_p <= NODE_ID_MAX))
    {
        this->ppNodeState[nodeId_p]->hide();
    }
}

//------------------------------------------------------------------------------
/**
\brief  Remove all CNs

Removes all controlled nodes from the node list.
*/
//------------------------------------------------------------------------------
void CnState::removeAllNodes(void)
{
    int nIdx;

    // count() gives all widgets (hidden ones too)
    for (nIdx = 0; nIdx < NODE_ID_MAX; nIdx++)
    {
        this->ppNodeState[nIdx]->hide();
    }
}
