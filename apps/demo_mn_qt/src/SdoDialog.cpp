/**
********************************************************************************
\file   SdoDialog.cpp

\brief  Implementation of the SDO execution dialog class

This file contains the implementation of the SDO execution dialog class.
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
#if (TARGET_SYSTEM == _WIN32_)
#define _WINSOCKAPI_                // prevent windows.h from including winsock.h
#endif  // (TARGET_SYSTEM == _WIN32_)

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <QtGui>
#include <SdoDialog.h>

#include <QPushButton>
#include <QLineEdit>
#include <QComboBox>
#include <QLabel>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QMessageBox>
#include <QtEndian>

#include <oplk/debugstr.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// class definitions
//------------------------------------------------------------------------------
Q_DECLARE_METATYPE(eObdType)
Q_DECLARE_METATYPE(eSdoType)

//============================================================================//
//            P U B L I C   M E M B E R   F U N C T I O N S                   //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  constructor

Constructs an SDO execution dialog.
*/
//------------------------------------------------------------------------------
SdoDialog::SdoDialog()
{
    QVBoxLayout* mainLayout = new QVBoxLayout;

    /* create labels and buttons */
    QLabel*      label = new QLabel("Enter parameters for SDO transfer");
    QPushButton* closeButton = new QPushButton("&Close");
    this->readButton = new QPushButton("&Read");
    this->writeButton = new QPushButton("&Write");

    /* create edit fields */
    this->pNodeIdEdit = new QLineEdit("0");
    this->pNodeIdEdit->setToolTip("Target node-ID. Hexadecimal numbers shall be prefixed with 0x. Local node is addressed with node-ID 0");

    this->pObjectEdit = new QLineEdit("0x1006/0");
    this->pObjectEdit->setToolTip("Object index and sub-index separated by / or space. Hexadecimal numbers shall be prefixed with 0x.");

    this->pDataEdit = new QLineEdit("");

    this->pDataTypeBox = new QComboBox();
    this->pDataTypeBox->addItem("UNSIGNED8", QVariant::fromValue(kObdTypeUInt8));
    this->pDataTypeBox->addItem("UNSIGNED16", QVariant::fromValue(kObdTypeUInt16));
    this->pDataTypeBox->addItem("UNSIGNED32", QVariant::fromValue(kObdTypeUInt32));
    this->pDataTypeBox->addItem("UNSIGNED64", QVariant::fromValue(kObdTypeUInt64));
    this->pDataTypeBox->addItem("INTEGER8", QVariant::fromValue(kObdTypeInt8));
    this->pDataTypeBox->addItem("INTEGER16", QVariant::fromValue(kObdTypeInt16));
    this->pDataTypeBox->addItem("INTEGER32", QVariant::fromValue(kObdTypeInt32));
    this->pDataTypeBox->addItem("INTEGER64", QVariant::fromValue(kObdTypeInt64));
    this->pDataTypeBox->addItem("VSTRING", QVariant::fromValue(kObdTypeVString));
    this->pDataTypeBox->addItem("DOMAIN/OSTRING/Hex string", QVariant::fromValue(kObdTypeDomain));
    this->pDataTypeBox->setCurrentIndex(2);

    this->pSdoTypeBox = new QComboBox();
    this->pSdoTypeBox->addItem("ASnd", QVariant::fromValue(kSdoTypeAsnd));
    this->pSdoTypeBox->addItem("UDP", QVariant::fromValue(kSdoTypeUdp));

    this->pAbortCodeLabel = new QLabel("Not yet transferred.");

    /* create form */
    QFormLayout *formLayout = new QFormLayout;
    formLayout->addRow(tr("&Node-ID:"), this->pNodeIdEdit);
    formLayout->addRow(tr("&Object:"), this->pObjectEdit);
    formLayout->addRow(tr("Data &type:"), this->pDataTypeBox);
    formLayout->addRow(tr("&Data:"), this->pDataEdit);
    formLayout->addRow(tr("&SDO type:"), this->pSdoTypeBox);
    formLayout->addRow(tr("Abort Code:"), this->pAbortCodeLabel);

    /* create buttons */
    QHBoxLayout* buttonLayout = new QHBoxLayout();
    buttonLayout->addStretch(1);
    buttonLayout->addWidget(this->readButton);
    buttonLayout->addWidget(this->writeButton);
    buttonLayout->addWidget(closeButton);

    connect(this->readButton, SIGNAL(clicked()), this, SLOT(startRead()));
    connect(this->writeButton, SIGNAL(clicked()), this, SLOT(startWrite()));
    connect(closeButton, SIGNAL(clicked()), this, SLOT(reject()));
    connect(this->pDataTypeBox,
            SIGNAL(currentIndexChanged(int)),
            this,
            SLOT(dataTypeChanged(int)));
    connect(this,
            SIGNAL(sigUpdateData(const QString&)),
            this,
            SLOT(updateData(const QString&)),
            Qt::QueuedConnection);

    /* create main layout */
    mainLayout->addWidget(label);
    mainLayout->addLayout(formLayout);
    mainLayout->addStretch(0);
    mainLayout->addLayout(buttonLayout);

    setLayout(mainLayout);
    setWindowTitle("Perform SDO Transfer");
    setSizeGripEnabled(true);
}

//------------------------------------------------------------------------------
/**
\brief  Enable/Disable fields in formular

\param[in]      enable_p            true = enable, false = disable
*/
//------------------------------------------------------------------------------
void SdoDialog::enableFields(bool enable_p)
{
    this->pNodeIdEdit->setEnabled(enable_p);
    this->pObjectEdit->setEnabled(enable_p);
    this->pDataTypeBox->setEnabled(enable_p);
    this->pSdoTypeBox->setEnabled(enable_p);
    this->readButton->setEnabled(enable_p);
    this->writeButton->setEnabled(enable_p);
}

//------------------------------------------------------------------------------
/**
\brief  Read fields from form

\returns    bool    true, if all fields are valid
                    false, otherwise
*/
//------------------------------------------------------------------------------
bool SdoDialog::readFields(void)
{
    bool        fConvOk;
    QStringList list;

    /* node-ID */
    this->targetNodeId = pNodeIdEdit->text().toUInt(&fConvOk, 0);
    if (!fConvOk)
    {
        QMessageBox::critical(this,
                              "Node-ID invalid",
                              "The specified node-ID format is invalid. Please correct and try again.");
        return false;
    }

    /* object index and sub-index */
    list = pObjectEdit->text().split(QRegExp("[^0-9a-fA-Fx]+"), QString::SkipEmptyParts);
    if (list.count() != 2)
    {
        QMessageBox::critical(this,
                              "Object invalid",
                              "The specified object format does not consist of two numbers (index and sub-index). Please correct and try again.");
        return false;
    }

    targetIndex = list[0].toUInt(&fConvOk, 0);
    if (!fConvOk)
    {
        QMessageBox::critical(this,
                              "Object invalid",
                              "The specified object index format is invalid. Please correct and try again.");
        return false;
    }

    targetSubindex = list[1].toUInt(&fConvOk, 0);
    if (!fConvOk)
    {
        QMessageBox::critical(this,
                              "Object invalid",
                              "The specified object sub-index format is invalid. Please correct and try again.");
        return false;
    }

    this->sdoType = this->pSdoTypeBox->itemData(this->pSdoTypeBox->currentIndex()).value<eSdoType>();

    return true;
}

//------------------------------------------------------------------------------
/**
\brief  Start SDO read

Start SDO transfer within stack event thread.
*/
//------------------------------------------------------------------------------
void SdoDialog::startRead()
{
    eObdType    obdType;
    int         size = 0;
    tOplkError  ret;

    this->enableFields(false);
    this->readFields();

    obdType = this->pDataTypeBox->itemData(this->pDataTypeBox->currentIndex()).value<eObdType>();

    switch (obdType)
    {
        case kObdTypeUInt8:
        case kObdTypeInt8:
            size = 1;
            break;

        case kObdTypeUInt16:
        case kObdTypeInt16:
            size = 2;
            break;

        case kObdTypeUInt32:
        case kObdTypeInt32:
            size = 4;
            break;

        case kObdTypeUInt64:
        case kObdTypeInt64:
            size = 8;
            break;

        case kObdTypeVString:
        case kObdTypeDomain:
            size = 4096*1024;
            break;
    }
    this->data.resize(size);

    ret = oplk_postUserEvent(this->readButton);
    if (ret != kErrorOk)
        this->enableFields(true);
}

//------------------------------------------------------------------------------
/**
\brief  Start SDO write

Start SDO transfer within stack event thread.
*/
//------------------------------------------------------------------------------
void SdoDialog::startWrite()
{
    eObdType    obdType;
    int         size = 0;
    tOplkError  ret;

    this->enableFields(false);
    this->readFields();

    obdType = this->pDataTypeBox->itemData(this->pDataTypeBox->currentIndex()).value<eObdType>();

    switch (obdType)
    {
        case kObdTypeVString:
            this->data = this->pDataEdit->text().toLatin1();
            break;

        case kObdTypeDomain:
            this->data = QByteArray::fromHex(this->pDataEdit->text().toLatin1());
            break;

        default:
            QStringList list = this->pDataEdit->text().split(QRegExp("[^0-9a-fA-Fx]+"), QString::SkipEmptyParts);

            if (list.count() < 1)
            {
                QMessageBox::critical(this,
                                      "Data invalid",
                                      "The specified data field does not consist of one number. Please correct and try again.");
                this->enableFields(true);
                return;
            }

            bool    fConvOk = false;
            quint64 uval;
            qint64  val;

            switch (obdType)
            {
                case kObdTypeUInt8:
                case kObdTypeUInt16:
                case kObdTypeUInt32:
                case kObdTypeUInt64:
                    uval = list[0].toULongLong(&fConvOk, 0);
                    if (!fConvOk)
                    {
                        QMessageBox::critical(this,
                                              "Data invalid",
                                              "The specified data format is not an unsigned integer. Please correct and try again.");
                        this->enableFields(true);
                        return;
                    }

                    this->data.resize(8);
                    qToLittleEndian<quint64>(uval, (uchar*)this->data.data());
                    break;

                case kObdTypeInt8:
                case kObdTypeInt16:
                case kObdTypeInt32:
                case kObdTypeInt64:
                    val = list[0].toLongLong(&fConvOk, 0);
                    if (!fConvOk)
                    {
                        QMessageBox::critical(this,
                                              "Data invalid",
                                              "The specified data format is not a signed integer. Please correct and try again.");
                        this->enableFields(true);
                        return;
                    }

                    this->data.resize(8);
                    qToLittleEndian<qint64>(val, (uchar*)this->data.data());
                    break;
            }

            switch (obdType)
            {
                case kObdTypeUInt8:
                case kObdTypeInt8:
                    size = 1;
                    break;

                case kObdTypeUInt16:
                case kObdTypeInt16:
                    size = 2;
                    break;

                case kObdTypeUInt32:
                case kObdTypeInt32:
                    size = 4;
                    break;

                case kObdTypeUInt64:
                case kObdTypeInt64:
                    size = 8;
                    break;
            }

            this->data.resize(size);
    }

    ret = oplk_postUserEvent(this->writeButton);
    if (ret != kErrorOk)
        this->enableFields(true);
}

//------------------------------------------------------------------------------
/**
\brief  dataTypeChanged handler

Will be called if another list item was selected.

\param[in]      index               current selected item
*/
//------------------------------------------------------------------------------
void SdoDialog::dataTypeChanged(int index)
{
    this->updateData("");
}

//------------------------------------------------------------------------------
/**
\brief  Update data from internal byte array

\param[in]      abortCode_p         SDO abort code string
*/
//------------------------------------------------------------------------------
void SdoDialog::updateData(const QString& abortCode_p)
{
    QString     dataString;
    eObdType    obdType;

    if (abortCode_p != "")
        this->pAbortCodeLabel->setText(abortCode_p);

    obdType = this->pDataTypeBox->itemData(this->pDataTypeBox->currentIndex()).value<eObdType>();

    switch (obdType)
    {
        case kObdTypeUInt8:
            if (this->data.size() >= 1)
            {
                unsigned int val = *(unsigned char*)this->data.data();
                dataString = QString("0x%1 = %2")
                                .arg(val, 2, 16, QLatin1Char('0'))
                                .arg(val);
            }
            else
                dataString = "Too less data for UNSIGNED8";
            break;

        case kObdTypeInt8:
            if (this->data.size() >= 1)
            {
                int val = *(INT8*)this->data.data();
                dataString = QString("%1")
                                .arg(val);
            }
            else
                dataString = "Too less data for INTEGER8";
            break;

        case kObdTypeUInt16:
            if (this->data.size() >= 2)
            {
                unsigned int val = qFromLittleEndian<quint16>((const uchar*)this->data.data());
                dataString = QString("0x%1 = %2")
                                .arg(val, 4, 16, QLatin1Char('0'))
                                .arg(val);
            }
            else
                dataString = "Too less data for UNSIGNED16";
            break;

        case kObdTypeInt16:
            if (this->data.size() >= 2)
            {
                int val = qFromLittleEndian<qint16>((const uchar*)this->data.data());
                dataString = QString("%1")
                                .arg(val);
            }
            else
                dataString = "Too less data for INTEGER16";
            break;

        case kObdTypeUInt32:
            if (this->data.size() >= 4)
            {
                unsigned int val = qFromLittleEndian<quint32>((const uchar*)this->data.data());
                dataString = QString("0x%1 = %2")
                                .arg(val, 8, 16, QLatin1Char('0'))
                                .arg(val);
            }
            else
                dataString = "Too less data for UNSIGNED32";
            break;

        case kObdTypeInt32:
            if (this->data.size() >= 4)
            {
                int val = qFromLittleEndian<qint32>((const uchar*)this->data.data());
                dataString = QString("%1")
                                .arg(val);
            }
            else
                dataString = "Too less data for INTEGER32";
            break;

        case kObdTypeUInt64:
            if (this->data.size() >= 8)
            {
                qulonglong val = qFromLittleEndian<quint64>((const uchar*)this->data.data());
                dataString = QString("0x%1 = %2")
                                .arg(val, 16, 16, QLatin1Char('0'))
                                .arg(val);
            }
            else
                dataString = "Too less data for UNSIGNED64";
            break;

        case kObdTypeInt64:
            if (this->data.size() >= 8)
            {
                qlonglong val = qFromLittleEndian<qint64>((const uchar*)this->data.data());
                dataString = QString("%1")
                                .arg(val);
            }
            else
                dataString = "Too less data for INTEGER64";
            break;

        case kObdTypeVString:
            dataString = QString(this->data);
            break;

        case kObdTypeDomain:
            dataString = QString(this->data.toHex());
            break;
    }

    this->pDataEdit->setText(dataString);

    this->enableFields(true);
}

//------------------------------------------------------------------------------
/**
\brief  userDefEvent handler

\param[in,out]  pUserArg_p          user-defined argument
*/
//------------------------------------------------------------------------------
void SdoDialog::userDefEvent(void* pUserArg_p)
{
    tSdoComConHdl   sdoComConHdl;
    tObdSize        obdSize;
    tOplkError      ret;

    obdSize = this->data.size();

    if (pUserArg_p == readButton)
    {
        ret = oplk_readObject(&sdoComConHdl,
                              targetNodeId,
                              targetIndex,
                              targetSubindex,
                              this->data.data(),
                              &obdSize,
                              (tSdoType)sdoType,
                              NULL);
        if (ret == kErrorApiTaskDeferred)
        {   // SDO transfer started
            return;
        }
        else if (ret == kErrorOk)
        {   // local OD access
            this->data.resize(obdSize);

            emit sigUpdateData("Successfully read from local OD");
        }
        else
        {
            this->data.resize(0);

            emit sigUpdateData(QString("Error 0x%1: %2")
                                .arg(ret, 0, 16)
                                .arg(debugstr_getRetValStr(ret)));
        }
    }
    else if (pUserArg_p == writeButton)
    {
        ret = oplk_writeObject(&sdoComConHdl,
                               targetNodeId,
                               targetIndex,
                               targetSubindex,
                               this->data.data(),
                               obdSize,
                               (tSdoType)sdoType,
                               NULL);
        if (ret == kErrorApiTaskDeferred)
        {   // SDO transfer started
            return;
        }
        else if (ret == kErrorOk)
        {   // local OD access
            this->data.resize(obdSize);

            emit sigUpdateData("Successfully written to local OD");
        }
        else
        {
            this->data.resize(0);

            emit sigUpdateData(QString("Error 0x%1: %2")
                                .arg(ret, 0, 16)
                                .arg(debugstr_getRetValStr(ret)));
        }
    }
}

//------------------------------------------------------------------------------
/**
\brief  sdoFinished handler

\param[in]      sdoInfo_p           information about finished SDO transfer
*/
//------------------------------------------------------------------------------
void SdoDialog::sdoFinished(tSdoComFinished sdoInfo_p)
{
    switch (sdoInfo_p.sdoComConState)
    {
        case kSdoComTransferFinished:
            this->data.resize(sdoInfo_p.transferredBytes);

            emit sigUpdateData(QString("Successfully %1 data")
                    .arg((sdoInfo_p.sdoAccessType == kSdoAccessTypeWrite ? "written" : "read")));
            break;

        default:
        case kSdoComTransferRxAborted:
        case kSdoComTransferTxAborted:
            emit sigUpdateData(QString("0x%1 = %2")
                                .arg(sdoInfo_p.abortCode, 8, 16, QLatin1Char('0'))
                                .arg(debugstr_getAbortCodeStr(sdoInfo_p.abortCode)));
            break;

        case kSdoComTransferLowerLayerAbort:
            emit sigUpdateData(QString("Lower layer abort (timeout)"));
            break;
    }
}
