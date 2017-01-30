/**
********************************************************************************
\file   BinaryLedWidget.h

\brief  Header file for the binary LED widget

This file contains the definitions for the binary LED widget.
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
#ifndef _INC_demo_BinaryLedWidget_H_
#define _INC_demo_BinaryLedWidget_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <QWidget>
#include <QVector>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// class definitions
//------------------------------------------------------------------------------
class QHBoxLayout;
class MultiColorLed;

//------------------------------------------------------------------------------
/**
\brief  Binary LED widget class

The class implements the binary LED widget.
*/
//------------------------------------------------------------------------------
class BinaryLedWidget : public QWidget
{
    Q_OBJECT

public:
    BinaryLedWidget(QWidget* pParent_p = 0);

    // Static members
    static const int        UNDEFINED_VALUE;

public slots:
    void setValue(int value_p);

private:
    void setupUi();

    // General layout
    QHBoxLayout*            pWidgetLayout;
    QVector<MultiColorLed*> leds;

    // Static members
    static const int        LED_COUNT;
};

#endif /* _INC_demo_BinaryLedWidget_H_ */
