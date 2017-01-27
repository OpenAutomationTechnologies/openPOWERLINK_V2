/**
********************************************************************************
\file   CnListWidget.cpp

\brief  Implementation of the CN list widget

This file implements the CN list widget.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <CnListWidget.h>

#include <QVBoxLayout>
#include <NmtStateWidget.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
const int CnListWidget::MAX_NODE_ID = 255;

//------------------------------------------------------------------------------
/**
\brief  Constructor

Constructs a CnListWidget.

\param[in]      pParent_p           Pointer to the parent window
*/
//------------------------------------------------------------------------------
CnListWidget::CnListWidget(QWidget* pParent_p) :
    QWidget(pParent_p)
{
    this->setupUi();
}

//------------------------------------------------------------------------------
/**
\brief  Setup the user interface

Initializes the GUI elements of the user interface
*/
//------------------------------------------------------------------------------
void CnListWidget::setupUi()
{
    // ---------------------------------------------------------------------
    // General widget settings
    // ---------------------------------------------------------------------
    this->pStateLayout = new QVBoxLayout(this);

    // LEDs for CN states
    for (int i = 0; i < CnListWidget::MAX_NODE_ID; i++)
    {
        NmtStateWidget* nmtStateWidget = new NmtStateWidget();
        nmtStateWidget->setWidgetLabel(QString("CN#: %1").arg(i));
        nmtStateWidget->hide();
        this->pStateLayout->addWidget(nmtStateWidget);
        this->nodeStates.append(nmtStateWidget);
    }

    // Add stretch to left-align the widgets
    this->pStateLayout->addStretch();
}

//------------------------------------------------------------------------------
/**
\brief  Add a CN

Adds a controlled node to the node list.

\param[in]      nodeId_p            Node ID of CN
*/
//------------------------------------------------------------------------------
void CnListWidget::addNode(int nodeId_p)
{
    if ((nodeId_p >= 0) &&
        (nodeId_p <= CnListWidget::MAX_NODE_ID))
    {
        this->nodeStates[nodeId_p]->show();
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
void CnListWidget::removeNode(int nodeId_p)
{
    if ((nodeId_p >= 0) &&
        (nodeId_p <= CnListWidget::MAX_NODE_ID))
    {
        this->nodeStates[nodeId_p]->hide();
    }
}

//------------------------------------------------------------------------------
/**
\brief  Set CN state

Sets the state of the CN

\param[in]      nodeId_p            Node ID of CN
\param[in]      state_p             State of CN
*/
//------------------------------------------------------------------------------
void CnListWidget::setState(int nodeId_p, tNmtState state_p)
{
    this->nodeStates[nodeId_p]->setNmtState(state_p);
}

//------------------------------------------------------------------------------
/**
\brief  Remove all CNs

Removes all controlled nodes from the node list.
*/
//------------------------------------------------------------------------------
void CnListWidget::removeAllNodes()
{
    for (int i = 0; i < this->nodeStates.size(); i++)
        this->nodeStates[i]->hide();
}
