/**
********************************************************************************
\file   NmtStateWidget.h

\brief  Header file for the NMT state widget

The file contains the definitions for the NMT state widget.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2013, Kalycito Infotech Private Ltd.All rights reserved.
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
#ifndef _INC_demo_NmtStateWidget_H_
#define _INC_demo_NmtStateWidget_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <oplk/oplk.h>
#include <QWidget>
#include <QString>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// class definitions
//------------------------------------------------------------------------------
class QHBoxLayout;
class QLabel;
class MultiColorLed;
//------------------------------------------------------------------------------
/**
\brief  NmtStateWidget class

The Class implements the node state widget.
*/
//------------------------------------------------------------------------------
class NmtStateWidget : public QWidget
{
    Q_OBJECT

public:
    NmtStateWidget(QWidget* pParent_p = 0);

    void setWidgetLabel(const QString& text_p);
    void showNmtStateText();
    void hideNmtStateText();

public slots:
    void setNmtState(tNmtState nmtState_p);

private:
    void setupUi();

    // General layout
    QHBoxLayout*    pWidgetLayout;
    QLabel*         pWidgetLabel;
    MultiColorLed*  pCurrentNmtStateLed;
    QLabel*         pCurrentNmtStateText;
};

#endif /* _INC_demo_NmtStateWidget_H_ */
