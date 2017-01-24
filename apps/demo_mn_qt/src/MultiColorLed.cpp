/**
********************************************************************************
\file   MultiColorLed.cpp

\brief  Implementation of the MultiColorLed widget

This file contains the implementation of the MultiColorLed widget.
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
#include <MultiColorLed.h>
#include <QBoxLayout>
#include <QLabel>
#include <QPixmap>

//============================================================================//
//            P U B L I C    M E M B E R    F U N C T I O N S                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Constructor

Constructs a MultiColorLed widget. The MultiColorLed widget is used to show an
LED widget in different colors.

\param[in]      pParent_p           Pointer to the parent widget.
*/
//------------------------------------------------------------------------------
MultiColorLed::MultiColorLed(QWidget* pParent_p) :
    QWidget(pParent_p),
    color(None)
{
    // Load the images into pixmaps
    this->ledImages.insert(Gray, new QPixmap(QString::fromUtf8(":/img/ledgray.png")));
    this->ledImages.insert(Red, new QPixmap(QString::fromUtf8(":/img/ledred.png")));
    this->ledImages.insert(Yellow, new QPixmap(QString::fromUtf8(":/img/ledyellow.png")));
    this->ledImages.insert(Green, new QPixmap(QString::fromUtf8(":/img/ledgreen.png")));

    this->setupUi();
}

//------------------------------------------------------------------------------
/**
\brief  Destructor

Destructs a MultiColorLed widget.
*/
//------------------------------------------------------------------------------
MultiColorLed::~MultiColorLed()
{
    // Delete the QPixmap objects containing the LED images
    foreach (QPixmap* img, this->ledImages)
        delete img;
}

//------------------------------------------------------------------------------
/**
\brief  Setup the user interface

Initializes the GUI elements of the user interface
*/
//------------------------------------------------------------------------------
void MultiColorLed::setupUi()
{
    // ---------------------------------------------------------------------
    // General widget settings
    // ---------------------------------------------------------------------
    this->setContentsMargins(0, 0, 0, 0);
    this->pLayout = new QBoxLayout(QBoxLayout::LeftToRight, this);
    this->pLayout->setContentsMargins(0, 0, 0, 0);

    // LED image
    this->pLedLabel = new QLabel();
    this->pLayout->addWidget(this->pLedLabel);
}

//------------------------------------------------------------------------------
/**
\brief  Get LED color

Gets the color of the LED.

\return Returns the currently set color of the LED.
*/
//------------------------------------------------------------------------------
MultiColorLed::LedColor MultiColorLed::getColor() const
{
    return this->color;
}

//------------------------------------------------------------------------------
/**
\brief  Set LED color

Sets the color of the LED.

\param[in]      color_p             Color to be displayed
*/
//------------------------------------------------------------------------------
void MultiColorLed::setColor(MultiColorLed::LedColor color_p)
{
    this->color = color_p;

    if ((color_p == MultiColorLed::None) || (!this->ledImages.contains(color_p)))
        this->pLedLabel->clear();
    else
        this->pLedLabel->setPixmap(*this->ledImages.value(color_p));
}
