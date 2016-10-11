/**
********************************************************************************
\file   Leds.cpp

\brief  Implementation of the LED widget class

The file contains the implementation of the LED widget class.

\ingroup module_demo_mn_qt
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
#include <Leds.h>

#include <QLabel>
#include <QPixmap>
#include <QHBoxLayout>


//============================================================================//
//            P U B L I C    M E M B E R    F U N C T I O N S                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Constructor

Constructs a LED widget

\param[in]      count_p             Number of LEDs to show
\param[in]      parent_p            Pointer to parent widget
*/
//------------------------------------------------------------------------------
Leds::Leds(int count_p, QWidget* parent_p)
    : QWidget(parent_p)
{
    int nIdx;

    this->count = count_p;

    QHBoxLayout* pLedsLayout = new QHBoxLayout;
    setLayout(pLedsLayout);
    setContentsMargins(0, 0, 0, 0);

    // create array for pointers to LedButtons
    this->ppLedLabels = new QLabel*[count_p];

    this->pActiveLed  = new QPixmap(":/img/ledred.png");
    this->pInactiveLed = new QPixmap(":/img/ledgreen.png");
    this->pNoLed = new QPixmap(":/img/ledgray.png");

    for (nIdx = 0; nIdx < count_p; nIdx++)
    {
        this->ppLedLabels[nIdx] = new QLabel(parent_p);
        this->ppLedLabels[nIdx]->setPixmap(*this->pNoLed);
        pLedsLayout->addWidget(this->ppLedLabels[nIdx]);
    }

    pLedsLayout->update();
}

//------------------------------------------------------------------------------
/**
\brief  Set LEDs

setLeds() sets the LEDs according to the data value.

\param[in]      dataIn_p            Data value to show
*/
//------------------------------------------------------------------------------
void Leds::setLeds(unsigned int dataIn_p)
{
    int nIdx;

    for (nIdx = 0; nIdx < count; nIdx++)
    {
        if (dataIn_p & (1 << nIdx))
            this->ppLedLabels[nIdx]->setPixmap(*this->pActiveLed);
        else
            this->ppLedLabels[nIdx]->setPixmap(*this->pInactiveLed);
    }
}

//------------------------------------------------------------------------------
/**
\brief  Disable all LEDs

Disables all LEDs.

*/
//------------------------------------------------------------------------------
void Leds::disableLeds(void)
{
    int nIdx;

    for (nIdx = 0; nIdx < count; nIdx++)
    {
        this->ppLedLabels[nIdx]->setPixmap(*this->pNoLed);
    }
}
