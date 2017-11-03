/**
********************************************************************************
\file   BinaryLedWidget.cpp

\brief  Implementation of the binary LED widget class

The file contains the implementation of the binary LED widget class.

\ingroup module_demo_mn_qt
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, B&R Industrial Automation GmbH
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
#include <BinaryLedWidget.h>
#include <MultiColorLed.h>

#include <QHBoxLayout>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
const int BinaryLedWidget::UNDEFINED_VALUE = -1;    // Undefined value
const int BinaryLedWidget::LED_COUNT = 8;           // Number of LEDs = Bits of 1 Byte

//============================================================================//
//            P U B L I C    M E M B E R    F U N C T I O N S                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Constructor

Constructs a binary LED widget

\param[in]      pParent_p           Pointer to parent widget
*/
//------------------------------------------------------------------------------
BinaryLedWidget::BinaryLedWidget(QWidget* pParent_p) :
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
void BinaryLedWidget::setupUi()
{
    // ---------------------------------------------------------------------
    // General widget settings
    // ---------------------------------------------------------------------
    this->setContentsMargins(0, 0, 0, 0);
    this->pWidgetLayout = new QHBoxLayout(this);

    // Add LEDs
    for (int i = 0; i < BinaryLedWidget::LED_COUNT; i++)
    {
        MultiColorLed* led = new MultiColorLed();
        this->pWidgetLayout->addWidget(led);
        this->leds.append(led);
    }
}

//------------------------------------------------------------------------------
/**
\brief  Sets a value

Sets a value to visualize on the LEDs.

\param[in]      value_p             Value to show
*/
//------------------------------------------------------------------------------
void BinaryLedWidget::setValue(int value_p)
{
    // Only show values that are in the defined range
    if ((value_p >= 0) &&
        (value_p < (1 << BinaryLedWidget::LED_COUNT)))
    {
        for (int i = 0; i < this->leds.size(); i++)
        {
            if (value_p & (1 << i))
                this->leds[i]->setColor(MultiColorLed::Green);
            else
                this->leds[i]->setColor(MultiColorLed::Red);
        }
    }
    else
    {
        // Indicate an undefined value
        for (int i = 0; i < this->leds.size(); i++)
            this->leds[i]->setColor(MultiColorLed::Gray);
    }
}
