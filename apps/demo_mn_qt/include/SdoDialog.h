/**
********************************************************************************
\file   SdoDialog.h

\brief  Header file for SDO execution dialog

The file contains the definitions for the SDO execution dialog
*******************************************************************************/
/*------------------------------------------------------------------------------
Copyright (c) 2016, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#ifndef _INC_demo_SdoDialog_H_
#define _INC_demo_SdoDialog_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <oplk/oplk.h>

#include <QDialog>
#include <QString>
#include <QByteArray>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// class definitions
//------------------------------------------------------------------------------
class QPushButton;
class QLineEdit;
class QComboBox;
class QLabel;


//------------------------------------------------------------------------------
/**
\brief  SdoDialog class

The class implements the SDO execution dialog.
*/
//------------------------------------------------------------------------------
class SdoDialog : public QDialog
{
    Q_OBJECT

public:
    SdoDialog();

signals:
    void sigUpdateData(const QString& abortCode_p);

private slots:
    void startWrite();
    void startRead();
    void dataTypeChanged(int index);
    void userDefEvent(void* pUserArg_p);
    void sdoFinished(tSdoComFinished sdoInfo_p);
    void updateData(const QString& abortCode_p);

private:
    QPushButton*    readButton;
    QPushButton*    writeButton;
    QLineEdit*      pNodeIdEdit;
    QLineEdit*      pObjectEdit;
    QLineEdit*      pDataEdit;
    QComboBox*      pDataTypeBox;
    QComboBox*      pSdoTypeBox;
    QLabel*         pAbortCodeLabel;
    QByteArray      data;
    UINT            targetNodeId;
    UINT            targetIndex;
    UINT            targetSubindex;
    eSdoType        sdoType;

    void enableFields(bool enable_p);
    bool readFields(void);
};

#endif /* _INC_demo_SdoDialog_H_ */
