/**
********************************************************************************
\file   Output.cpp

\brief  Implementation of the Ouptput class

This file contains the implementation of the Output class.
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
#include <Output.h>
#include <Leds.h>

#include <QVBoxLayout>
#include <QLabel>


//============================================================================//
//            P U B L I C    M E M B E R    F U N C T I O N S                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  constructor

Constructs an Output widget.

\param[in]      parent              Pointer to parent window
*/
//------------------------------------------------------------------------------
Output::Output(QWidget* parent)
    : QWidget(parent)
{
    this->pOutputLayout = new QVBoxLayout;
    setLayout(this->pOutputLayout);

    QFont LabelFont;
    LabelFont.setBold(true);
    LabelFont.setPointSize(18);

    QLabel* pDigiOutLabel = new QLabel("Digital Outputs:");
    pDigiOutLabel->setFont(LabelFont);
    this->pOutputLayout->addWidget(pDigiOutLabel);

    this->ppLeds = new Leds*[NODE_ID_MAX];
    for (int i = 0; i < NODE_ID_MAX; i++)
    {
        this->ppLeds[i] = new Leds(LED_NUM);
        this->ppLeds[i]->hide();
        this->pOutputLayout->addWidget(ppLeds[i]);
    }

    this->pOutputLayout->addStretch(1);
}

//------------------------------------------------------------------------------
/**
\brief  Set output value

Sets the output value of a CN

\param[in]      dataIn_p            Output Value to set
\param[in]      nodeId_p            Node ID of CN
*/
//------------------------------------------------------------------------------
void Output::setValue(int dataIn_p, int nodeId_p)
{
    this->ppLeds[nodeId_p]->setLeds(dataIn_p);
}

//------------------------------------------------------------------------------
/**
\brief  Disable the output LEDs

Disable the output LEDs to show that they are not actively controlled by the
application.

\param[in]      nodeId_p            Node ID of CN
*/
//------------------------------------------------------------------------------
void Output::disable(int nodeId_p)
{
    this->ppLeds[nodeId_p]->disableLeds();
}

//------------------------------------------------------------------------------
/**
\brief  Add a CN

Adds a controlled node to the node list.

\param[in]      nodeId_p            Node ID of CN
*/
//------------------------------------------------------------------------------
void Output::addNode(int nodeId_p)
{
    if ((nodeId_p >= 0) &&
        (nodeId_p <= NODE_ID_MAX))
    {
        this->ppLeds[nodeId_p]->show();
        this->ppLeds[nodeId_p]->disableLeds();
        this->pOutputLayout->update();
    }
}

//------------------------------------------------------------------------------
/**
\brief  Remove a CN

Removes a controlled node from the node list.

\param[in]      nodeId_p            Node ID of CN
*/
//------------------------------------------------------------------------------
void Output::removeNode(int nodeId_p)
{
    if ((nodeId_p >= 0) &&
        (nodeId_p <= NODE_ID_MAX))
    {
        this->ppLeds[nodeId_p]->hide();
    }
}

//------------------------------------------------------------------------------
/**
\brief  Remove all CNs

Removes all controlled nodes from the node list.
*/
//------------------------------------------------------------------------------
void Output::removeAllNodes()
{
    int nIdx;

    // count() gives all widgets (hidden ones too)
    for (nIdx = 0; nIdx < NODE_ID_MAX; nIdx++)
    {
        this->ppLeds[nIdx]->hide();
    }
}
