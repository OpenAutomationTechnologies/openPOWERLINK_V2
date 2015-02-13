/**
********************************************************************************
\file   NmtCommandDialog.cpp

\brief  Implementation of the dialog class to execute local NMT commands/events

This file contains the implementation of the NMT command dialog class.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2015, SYSTEC electronic GmbH
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
#include <QtGui>
#include "NmtCommandDialog.h"

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// class definitions
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   M E M B E R   F U N C T I O N S                   //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Constructor

\param  nmtEvent_p  The NMT event which is used to initialize the NMT command
                    dialog.

Constructs an NMT command dialog.
*/
//------------------------------------------------------------------------------
NmtCommandDialog::NmtCommandDialog(tNmtEvent nmtEvent_p)
{
    QVBoxLayout* mainLayout = new QVBoxLayout;

    /* create labels */
    QLabel*      label = new QLabel("Enter NMT event number (type tNmtEvent):");
    QPushButton* okButton = new QPushButton("OK");
    QPushButton* cancelButton = new QPushButton("Cancel");


    /* create lineedit */
    pNmtCmdEdit = new QLineEdit(QString::number((UINT)nmtEvent_p, 16).prepend("0x"));

    /* create buttons */
    QHBoxLayout* buttonLayout = new QHBoxLayout();
    buttonLayout->addStretch(1);
    buttonLayout->addWidget(okButton);
    buttonLayout->addWidget(cancelButton);

    connect(okButton, SIGNAL(clicked()), this, SLOT(accept()));
    connect(cancelButton, SIGNAL(clicked()), this, SLOT(reject()));

    /* create main layout */
    mainLayout->addWidget(label);
    mainLayout->addWidget(pNmtCmdEdit);
    mainLayout->addStretch(0);
    mainLayout->addLayout(buttonLayout);

    setLayout(mainLayout);
    setWindowTitle("Select NMT command to execute");
}

//------------------------------------------------------------------------------
/**
\brief  Get NMT event for local execution

The function returns NMT command/event for local execution by NMT
state machine.

\return The function returns the NMT event to be executed.
*/
//------------------------------------------------------------------------------
tNmtEvent NmtCommandDialog::getNmtEvent(void)
{
    bool fConvOk;
    UINT nmtEventId;

    nmtEventId = pNmtCmdEdit->text().toUInt(&fConvOk, 0);

    if (fConvOk)
    {
        return (tNmtEvent)nmtEventId;
    }
    else
    {
        return kNmtEventNoEvent;
    }
}
