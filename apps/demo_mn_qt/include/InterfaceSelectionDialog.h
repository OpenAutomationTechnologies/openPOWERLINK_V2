/**
********************************************************************************
\file   InterfaceSelectionDialog.h

\brief  Header file for interface selection dialog

The file contains the definitions for the interface selection dialog
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, B&R Industrial Automation GmbH
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
#ifndef _INC_demo_InterfaceSelectDialog_H_
#define _INC_demo_InterfaceSelectDialog_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <QDialog>
#include "ui_InterfaceSelectionDialog.h"

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// class definitions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/**
\brief  InterfaceSelectDialog class

The class implements the interface selection dialog.
*/
//------------------------------------------------------------------------------
class InterfaceSelectionDialog : public QDialog
{
    Q_OBJECT

public:
    InterfaceSelectionDialog(QWidget* pParent_p = 0);

    int             fillList();
    void            setActive(const QString& devName_p);
    const QString   getDevName() const;

private:
    Ui::InterfaceSelectionDialog    ui;
    static const int                MAX_INTERFACES;
};

#endif /* _INC_demo_InterfaceSelectDialog_H_ */
