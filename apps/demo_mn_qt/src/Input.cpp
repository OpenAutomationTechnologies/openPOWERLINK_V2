/**
********************************************************************************
\file   Input.cpp

\brief  Implementation of input widget

The file contains the implementation of the input widget.
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
#include <oplk/oplk.h>

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QToolButton>
#include <QPalette>
#include <QColor>
#include <QLabel>

#include "Input.h"
#include "Leds.h"

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

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   M E M B E R   F U N C T I O N S                   //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  constructor

Constructs an Input widget.

\param  parent                  pointer to parent widget
*/
//------------------------------------------------------------------------------
Input::Input(QWidget* parent)
    : QWidget(parent)
{
    QFont LabelFont;
    LabelFont.setBold(true);
    LabelFont.setPointSize(18);

    pInputLayout = new QVBoxLayout;
    setLayout(pInputLayout);

    pInputLayout->addStretch(0);

    QLabel* pDigiInLabel = new QLabel("Digital Inputs:");
    pDigiInLabel->setFont(LabelFont);
    pInputLayout->addWidget(pDigiInLabel);

    ppLeds = new Leds*[NODE_ID_MAX];
    for (int i = 0; i < NODE_ID_MAX; i++)
    {
        ppLeds[i] = new Leds(LED_NUM);
        ppLeds[i]->hide();
        pInputLayout->addWidget(ppLeds[i]);
    }

    pInputLayout->addStretch(1);
}

//------------------------------------------------------------------------------
/**
\brief  set LEDs

Sets the input LEDs of a CN.

\param  dataIn_p        Input data to show on LEDs
\param  nodeId_p        Node ID for which to set LEDs
*/
//------------------------------------------------------------------------------
void Input::setLeds(int dataIn_p, int nodeId_p)
{
    ppLeds[nodeId_p]->setLeds(dataIn_p);
}

//------------------------------------------------------------------------------
/**
\brief  add a CN

Adds a controlled node to the node list.

\param          nodeId_p                Node ID of CN

\ingroup module_demo_mn_qt
*/
//------------------------------------------------------------------------------
void Input::addNode(int nodeId_p)
{
    if ((nodeId_p >= 0) && (nodeId_p <= NODE_ID_MAX))
    {
        ppLeds[nodeId_p]->show();
        ppLeds[nodeId_p]->disableLeds();
        //apNodes[iNodeId_p]->setFixedSize(NODE_WIDTH, NODE_HEIGHT);
        pInputLayout->update();
    }
}

//------------------------------------------------------------------------------
/**
\brief  remove a CN

Removes a controlled node from the node list.

\param          nodeId_p               Node ID of CN
*/
//------------------------------------------------------------------------------
void Input::removeNode(int nodeId_p)
{
    if ((nodeId_p >= 0) && (nodeId_p <= NODE_ID_MAX))
    {
        ppLeds[nodeId_p]->hide();
    }
}

//------------------------------------------------------------------------------
/**
\brief  remove all CNs

Removes all controlled nodes from the node list.
*/
//------------------------------------------------------------------------------
void Input::removeAllNodes()
{
    int nIdx;

    // count() gives all widgets (hidden ones too)
    for (nIdx = 0; nIdx < NODE_ID_MAX; nIdx++)
    {
        ppLeds[nIdx]->hide();
    }
}

