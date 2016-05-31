/**
********************************************************************************
\file   State.h

\brief  Header file for POWERLINK MN state widget

This file contains the definitions for the POWERLINK MN state widget.
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

#ifndef _INC_State_H_
#define _INC_State_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <oplk/oplk.h>

#include <QWidget>
#include <QString>
#include <QPalette>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define LED_NUM        8
#define NODE_ID_MAX    255

//------------------------------------------------------------------------------
// class definitions
//------------------------------------------------------------------------------
class QHBoxLayout;
class QLabel;
class QToolButton;
class Leds;
class QPixmap;

//------------------------------------------------------------------------------
/**
\brief  State widget class

The class implements the POWERLINK MN state widget.
*/
//------------------------------------------------------------------------------
class State : public QWidget
{
    Q_OBJECT

public:
    State(QWidget* parent = 0);

public slots:
    void setStatusLed(int status_p);
    void setNmtStateText(const QString& strState_p);

private:
    QPalette     PalGreenButton;
    QPalette     PalYellowButton;
    QPalette     PalRedButton;

    QHBoxLayout* pNodesLayout;

    QLabel*      pStatusLed;
    QLabel*      pNmtStateLabel;
    QHBoxLayout* pNmtStateLayout;

    QToolButton* apNodes[NODE_ID_MAX + 1];

    Leds*        pLeds;

    QLabel*      pNmtSectionLabel;
    QLabel*      pNodesSectionLabel;

    int          iNodeWidth;
    int          iNodeHeight;

    QPixmap*     pRedLed;
    QPixmap*     pYellowLed;
    QPixmap*     pGreenLed;
};

#endif /* _INC_State_H_ */
