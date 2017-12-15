/**
********************************************************************************
\file   InterfaceSelectionDialog.cpp

\brief  Implementation of the interface selection dialog class

This file contains the implementation of the interface selection dialog class.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <InterfaceSelectionDialog.h>
#include <oplk/oplk.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
const int InterfaceSelectionDialog::MAX_INTERFACES = 10;    // Define the max. number of interfaces shown to the user

//------------------------------------------------------------------------------
// class definitions
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   M E M B E R   F U N C T I O N S                   //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Constructor

Constructs an interface selection dialog.

\param[in]      pParent_p           Pointer to parent widget
*/
//------------------------------------------------------------------------------
InterfaceSelectionDialog::InterfaceSelectionDialog(QWidget* pParent_p) :
    QDialog(pParent_p)
{
    this->ui.setupUi(this);
}

//------------------------------------------------------------------------------
/**
\brief  Fill the list of available interfaces.

Fill the list of available interfaces.

\retval -1                          Interface list couldn't be filled
\retval 0                           Interface list was successfully filled
*/
//------------------------------------------------------------------------------
int InterfaceSelectionDialog::fillList()
{
    tNetIfId    aInterfaces[InterfaceSelectionDialog::MAX_INTERFACES];
    size_t      noInterfaces = InterfaceSelectionDialog::MAX_INTERFACES;
    size_t      i = 0;

    if (oplk_enumerateNetworkInterfaces(aInterfaces, &noInterfaces) == kErrorOk)
    {
        for (i = 0; i < noInterfaces; i++)
        {
            QListWidgetItem*    pNewItem = new QListWidgetItem();
            QString             devDesc;

            if (aInterfaces[i].aDeviceDescription)
                devDesc = QString(aInterfaces[i].aDeviceDescription);
            else
                devDesc = QString(aInterfaces[i].aDeviceName);

            pNewItem->setData(Qt::UserRole, QString(aInterfaces[i].aDeviceName));
            pNewItem->setText(devDesc);

            this->ui.pDeviceListWidget->addItem(pNewItem);
        }
    }

    if (i > 0)
        return 0;
    else
        return -1;
}

//------------------------------------------------------------------------------
/**
\brief  Select the active interface in the list of available interfaces.

Select the active interface in the list of available interfaces.

\param[in]      devName_p           Reference to the device name which shall be
                                    selected.
*/
//------------------------------------------------------------------------------
void InterfaceSelectionDialog::setActive(const QString& devName_p)
{
    for (int index = 0;
         index < this->ui.pDeviceListWidget->count();
         index++)
    {
        QListWidgetItem* pItem = this->ui.pDeviceListWidget->item(index);

        if (pItem->data(Qt::UserRole).toString() == devName_p)
            this->ui.pDeviceListWidget->setCurrentItem(pItem);
    }
}

//------------------------------------------------------------------------------
/**
\brief  Get interface device name

Returns the name of the selected interface.

\return Name of the selected interface
*/
//------------------------------------------------------------------------------
const QString InterfaceSelectionDialog::getDevName() const
{
    return ui.pDeviceListWidget->currentItem()->data(Qt::UserRole).toString();
}
