/**
********************************************************************************
\file   Output.h

\brief  Header file for Output Widget

This file contains the definitions of the Output widget.
*******************************************************************************/
/*------------------------------------------------------------------------------
Copyright (c) 2016, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2013, SYSTEC electronic GmbH
Copyright (c) 2013, Kalycito Infotech Private Ltd.All rights reserved.
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
#ifndef _INC_demo_Output_H_
#define _INC_demo_Output_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <oplk/oplk.h>

#include <QWidget>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define LED_NUM         8
#define NODE_ID_MAX     255

//------------------------------------------------------------------------------
// class definitions
//------------------------------------------------------------------------------
class QVBoxLayout;
class Leds;

//------------------------------------------------------------------------------
/**
\brief  Output class

The class implements the output data widget.
*/
//------------------------------------------------------------------------------
class Output : public QWidget
{
    Q_OBJECT

public:
    Output(QWidget* parent = 0);

public slots:
    void setValue(int dataIn_p, int nodeId_p);
    void addNode(int nodeId_p);
    void removeNode(int nodeId_p);
    void removeAllNodes();
    void disable(int nodeId_p);

private:
    QVBoxLayout*    pOutputLayout;
    Leds**          ppLeds;

};

#endif /* _INC_demo_Output_H_ */
