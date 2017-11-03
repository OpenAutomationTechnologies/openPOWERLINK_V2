/**
********************************************************************************
\file   SdoTransferDialog.h

\brief  Header file for SDO transfer dialog

The file contains the definitions for the SDO transfer dialog
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, B&R Industrial Automation GmbH
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
#ifndef _INC_demo_SdoTransferDialog_H_
#define _INC_demo_SdoTransferDialog_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <QDialog>
#include <QString>
#include <QRegExp>
#include "ui_SdoTransferDialog.h"

#include <oplk/oplk.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// class definitions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/**
\brief  SdoTransferDialog class

The class implements the SDO transfer dialog.
*/
//------------------------------------------------------------------------------
class SdoTransferDialog : public QDialog
{
    Q_OBJECT

public:
    SdoTransferDialog(QWidget* pParent_p = 0);

signals:
    void sigUpdateData(const QString& abortCode_p);

private slots:
    void startRead();
    void startWrite();
    void dataTypeChanged(int index_p);
    void updateData(const QString& abortCode_p);
    void userDefEvent(void* pUserArg_p);
    void sdoFinished(tSdoComFinished sdoInfo_p);

private:
    Ui::SdoTransferDialog   ui;

    QByteArray              data;
    uint                    targetNodeId;
    uint                    targetIndex;
    uint                    targetSubindex;
    eSdoType                sdoType;

    void enableFields(bool enable_p);
    void readFields();

    // Static members
    static const int        NUMBER_BASE_HEX;
    static const QRegExp    REGEX_HEX_NUMBER;
    static const size_t     MAX_SIZE_OF_DOMAIN;

    static const QString    TEXT_ERR_TOO_LESS_DATA;
    static const QString    TEXT_UNSIGNED8;
    static const QString    TEXT_UNSIGNED16;
    static const QString    TEXT_UNSIGNED32;
    static const QString    TEXT_UNSIGNED64;
    static const QString    TEXT_INTEGER8;
    static const QString    TEXT_INTEGER16;
    static const QString    TEXT_INTEGER32;
    static const QString    TEXT_INTEGER64;
    static const QString    TEXT_VSTRING;
    static const QString    TEXT_OSTRING_DOMAIN;

    static size_t           getSizeOfObdType(eObdType obdType_p);
};

#endif // _INC_demo_SdoTransferDialog_H_
