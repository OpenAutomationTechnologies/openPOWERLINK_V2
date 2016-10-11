/**
********************************************************************************
\file   InterfaceSelectDialog.cpp

\brief  Implementation of the interface selection dialog class

This file contains the implementation of the interface selection dialog class.
*******************************************************************************/
/*------------------------------------------------------------------------------
Copyright (c) 2016, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#if (TARGET_SYSTEM == _WIN32_)
#define _WINSOCKAPI_            // prevent windows.h from including winsock.h
#endif  // (TARGET_SYSTEM == _WIN32_)

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <InterfaceSelectDialog.h>

#include <QListWidget>
#include <QListWidgetItem>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QPushButton>

#include <pcap.h>

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
\brief  constructor

Constructs an interface selection dialog.
*/
//------------------------------------------------------------------------------
InterfaceSelectDialog::InterfaceSelectDialog()
{
    QVBoxLayout* mainLayout = new QVBoxLayout;

    /* create labels */
    QLabel*      label = new QLabel("Select network interface:");
    QPushButton* okButton = new QPushButton("OK");
    QPushButton* cancelButton = new QPushButton("Cancel");


    /* create listbox */
    this->deviceListWidget = new QListWidget;
    this->deviceListWidget->setSelectionMode(QAbstractItemView::SingleSelection);

    /* create buttons */
    QHBoxLayout* buttonLayout = new QHBoxLayout();
    buttonLayout->addStretch(1);
    buttonLayout->addWidget(okButton);
    buttonLayout->addWidget(cancelButton);

    connect(okButton,
            SIGNAL(clicked()),
            this,
            SLOT(accept()));
    connect(cancelButton,
            SIGNAL(clicked()),
            this,
            SLOT(reject()));
    connect(deviceListWidget,
            SIGNAL(currentItemChanged(QListWidgetItem*, QListWidgetItem*)),
            this,
            SLOT(itemChanged(QListWidgetItem*, QListWidgetItem*)));

    /* create main layout */
    mainLayout->addWidget(label);
    mainLayout->addWidget(deviceListWidget);
    mainLayout->addStretch(0);
    mainLayout->addLayout(buttonLayout);

    setLayout(mainLayout);
    setWindowTitle("Select PCAP interface");
}

//------------------------------------------------------------------------------
/**
\brief  Setup the process image

SetupProcessImage() sets up the process image used by the application.

\param[in]      rDevName_p          Reference to the device name which shall be
                                    selected by default.

\retval -1                          Interface list couldn't be filled
\retval 0                           Interface list was successfully filled
*/
//------------------------------------------------------------------------------
int InterfaceSelectDialog::fillList(const QString& rDevName_p)
{
    char        sErr_Msg[PCAP_ERRBUF_SIZE];
    pcap_if_t*  alldevs;
    pcap_if_t*  seldev;
    int         numIntf = 0;

    /* Retrieve the device list on the local machine */
    if (pcap_findalldevs(&alldevs, sErr_Msg) == -1)
        return -1;

    /* Add the list to the listbox */
    for (seldev = alldevs; seldev != NULL; seldev = seldev->next)
    {
        numIntf++;
        QListWidgetItem* newItem = new QListWidgetItem;

        QString devName(seldev->name);
        QVariant data(devName);

        QString devDesc;
        if (seldev->description)
            this->devDesc = seldev->description;
        else
            this->devDesc = seldev->name;

        newItem->setData(Qt::UserRole, data);
        newItem->setText(this->devDesc);
        deviceListWidget->addItem(newItem);

        if (rDevName_p == devName)
            deviceListWidget->setCurrentItem(newItem);
    }
    pcap_freealldevs(alldevs);

    if (numIntf > 0)
        return 0;
    else
        return -1;
}

//------------------------------------------------------------------------------
/**
\brief  itemChanged handler

Will be called if another list item was selected.

\param[in]      current             Current selected list item
\param[in]      previous            Previous selected list item
*/
//------------------------------------------------------------------------------
void InterfaceSelectDialog::itemChanged(QListWidgetItem* current,
                                        QListWidgetItem* previous)
{
    // set devName to the current Item
    this->devDesc = current->text();
    this->devName = current->data(Qt::UserRole).toString();
}

//------------------------------------------------------------------------------
/**
\brief  get interface device name

getDevName() returns the name of the selected interface.

\return Reference to the name of selected interface
*/
//------------------------------------------------------------------------------
const QString& InterfaceSelectDialog::getDevName(void) const
{
    return this->devName;
}
