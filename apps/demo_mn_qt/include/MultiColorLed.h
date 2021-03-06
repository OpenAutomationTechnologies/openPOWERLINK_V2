/**
********************************************************************************
\file   MultiColorLed.h

\brief  Header file for MultiColorLed widget

The file contains the definitions of the MultiColorLed widget.
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
#ifndef _INC_demo_MultiColorLed_H_
#define _INC_demo_MultiColorLed_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <QWidget>
#include <QHash>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// class definitions
//------------------------------------------------------------------------------
class QBoxLayout;
class QLabel;
class QPixmap;

//------------------------------------------------------------------------------
/**
\brief  MultiColorLed class

The class implements the MultiColorLed widget.
*/
//------------------------------------------------------------------------------
class MultiColorLed : public QWidget
{
    Q_OBJECT

public:
    enum LedColor
    {
        None,
        Gray,
        Red,
        Yellow,
        Green
    };

    MultiColorLed(QWidget* pParent_p = 0);
    ~MultiColorLed();

    LedColor getColor() const;
    void setColor(LedColor color_p);

private:
    void setupUi();

    QHash<LedColor, QPixmap*>   ledImages;
    LedColor                    color;

    // GUI elements
    QBoxLayout*                 pLayout;
    QLabel*                     pLedLabel;
};

#endif /* _INC_demo_MultiColorLed_H_ */
