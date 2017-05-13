/**
********************************************************************************
\file   CnState.h

\brief  Definitions for CnState class

This file contains the definitions of the CnState class.
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
#ifndef _INC_demo_CnState_h_
#define _INC_demo_CnState_h_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <QWidget>

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------
#define NODE_ID_MAX     255

//------------------------------------------------------------------------------
// Class declarations
//------------------------------------------------------------------------------
class QVBoxLayout;
class NodeState;

//------------------------------------------------------------------------------
/**
\brief  CnState class

Class CnState implements the CN state widget. It represents the state of a CN.
*/
//------------------------------------------------------------------------------
class CnState : public QWidget
{
    Q_OBJECT

public:
    CnState(QWidget* parent = 0);

public slots:
    void setState(int nodeId_p,
                  int state_p);
    void addNode(int nodeId_p);
    void removeNode(int nodeId_p);
    void removeAllNodes(void);

private:
    QVBoxLayout*    pStateLayout;
    NodeState**     ppNodeState;
};

#endif // _INC_demo_CnState_h_
