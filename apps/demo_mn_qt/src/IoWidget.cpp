/**
********************************************************************************
\file   IoWidget.cpp

\brief  Implementation of the I/O widget

The file contains the implementation of the I/O widget.
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
#include <IoWidget.h>
#include <BinaryLedWidget.h>

#include <QVBoxLayout>
//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
const int IoWidget::MAX_NODE_ID = 255;

//============================================================================//
//            P U B L I C    M E M B E R    F U N C T I O N S                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  constructor

Constructs an IoWidget.

\param[in]      pParent_p           Pointer to parent widget
*/
//------------------------------------------------------------------------------
IoWidget::IoWidget(QWidget* pParent_p) :
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
void IoWidget::setupUi()
{
    // ---------------------------------------------------------------------
    // General widget settings
    // ---------------------------------------------------------------------
    this->pWidgetLayout = new QVBoxLayout(this);

    // LEDs for CN I/Os
    for (int i = 0; i < IoWidget::MAX_NODE_ID; i++)
    {
        BinaryLedWidget* binaryLedWidget = new BinaryLedWidget();
        binaryLedWidget->hide();
        this->pWidgetLayout->addWidget(binaryLedWidget);
        this->leds.append(binaryLedWidget);
    }

    // Add stretch to left-align the widgets
    this->pWidgetLayout->addStretch();
}

//------------------------------------------------------------------------------
/**
\brief  Add a CN

Add a controlled node to the node list.

\param[in]      nodeId_p            Node ID of CN
*/
//------------------------------------------------------------------------------
void IoWidget::addNode(unsigned int nodeId_p)
{
    if ((nodeId_p >= 0) &&
        (nodeId_p <= IoWidget::MAX_NODE_ID))
    {
        this->leds[nodeId_p]->show();
        this->leds[nodeId_p]->setValue(BinaryLedWidget::UNDEFINED_VALUE);
        this->pWidgetLayout->update();
    }
}

//------------------------------------------------------------------------------
/**
\brief  Remove a CN

Removes a controlled node from the node list.

\param[in]      nodeId_p            Node ID of CN
*/
//------------------------------------------------------------------------------
void IoWidget::removeNode(unsigned int nodeId_p)
{
    if ((nodeId_p >= 0) &&
        (nodeId_p <= IoWidget::MAX_NODE_ID))
    {
        this->leds[nodeId_p]->hide();
    }
}

//------------------------------------------------------------------------------
/**
\brief  Disable a CN

Disable the LEDs to show that they are not actively controlled by the
application.

\param[in]      nodeId_p            Node ID of CN
*/
//------------------------------------------------------------------------------
void IoWidget::disableNode(unsigned int nodeId_p)
{
    this->leds[nodeId_p]->setValue(BinaryLedWidget::UNDEFINED_VALUE);
}

//------------------------------------------------------------------------------
/**
\brief  Set the value

Sets the value of a CN

\param[in]      nodeId_p            Node ID of CN
\param[in]      dataIn_p            Value to set
*/
//------------------------------------------------------------------------------
void IoWidget::setValue(unsigned int nodeId_p,
                        unsigned int dataIn_p)
{
    this->leds[nodeId_p]->setValue((int)dataIn_p);
}

//------------------------------------------------------------------------------
/**
\brief  Remove all CNs

Removes all controlled nodes from the node list.
*/
//------------------------------------------------------------------------------
void IoWidget::removeAllNodes()
{
    for (int i = 0; i < this->leds.size(); i++)
        this->leds[i]->hide();
}
