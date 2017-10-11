/**
********************************************************************************
\file   SdoTransferDialog.cpp

\brief  Implementation of the SDO transfer dialog class

This file contains the implementation of the SDO transfer dialog class.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <SdoTransferDialog.h>
#include <QMessageBox>
#include <QtEndian>

#include <oplk/debugstr.h>

//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
const int       SdoTransferDialog::NUMBER_BASE_HEX = 16;
const QRegExp   SdoTransferDialog::REGEX_HEX_NUMBER("[^0-9a-fA-Fx]+");

const size_t    SdoTransferDialog::MAX_SIZE_OF_DOMAIN = 4096 * 1024;

const QString   SdoTransferDialog::TEXT_ERR_TOO_LESS_DATA("Too less data for ");
const QString   SdoTransferDialog::TEXT_UNSIGNED8("UNSIGNED8");
const QString   SdoTransferDialog::TEXT_UNSIGNED16("UNSIGNED16");
const QString   SdoTransferDialog::TEXT_UNSIGNED32("UNSIGNED32");
const QString   SdoTransferDialog::TEXT_UNSIGNED64("UNSIGNED64");
const QString   SdoTransferDialog::TEXT_INTEGER8("INTEGER8");
const QString   SdoTransferDialog::TEXT_INTEGER16("INTEGER16");
const QString   SdoTransferDialog::TEXT_INTEGER32("INTEGER32");
const QString   SdoTransferDialog::TEXT_INTEGER64("INTEGER64");
const QString   SdoTransferDialog::TEXT_VSTRING("VSTRING");
const QString   SdoTransferDialog::TEXT_OSTRING_DOMAIN("OSTRING/DOMAIN");

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
Q_DECLARE_METATYPE(eObdType)
Q_DECLARE_METATYPE(eSdoType)

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   M E M B E R   F U N C T I O N S                   //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Constructor

Constructs an SDO transfer dialog.

\param[in]      pParent_p           Pointer to parent widget
*/
//------------------------------------------------------------------------------
SdoTransferDialog::SdoTransferDialog(QWidget* pParent_p) :
    QDialog(pParent_p)
{
    this->ui.setupUi(this);

    // Add available data types
    this->ui.pDatatype->addItem(SdoTransferDialog::TEXT_UNSIGNED8, QVariant::fromValue(kObdTypeUInt8));
    this->ui.pDatatype->addItem(SdoTransferDialog::TEXT_UNSIGNED16, QVariant::fromValue(kObdTypeUInt16));
    this->ui.pDatatype->addItem(SdoTransferDialog::TEXT_UNSIGNED32, QVariant::fromValue(kObdTypeUInt32));
    this->ui.pDatatype->addItem(SdoTransferDialog::TEXT_UNSIGNED64, QVariant::fromValue(kObdTypeUInt64));
    this->ui.pDatatype->addItem(SdoTransferDialog::TEXT_INTEGER8, QVariant::fromValue(kObdTypeInt8));
    this->ui.pDatatype->addItem(SdoTransferDialog::TEXT_INTEGER16, QVariant::fromValue(kObdTypeInt16));
    this->ui.pDatatype->addItem(SdoTransferDialog::TEXT_INTEGER32, QVariant::fromValue(kObdTypeInt32));
    this->ui.pDatatype->addItem(SdoTransferDialog::TEXT_INTEGER64, QVariant::fromValue(kObdTypeInt64));
    this->ui.pDatatype->addItem(SdoTransferDialog::TEXT_VSTRING, QVariant::fromValue(kObdTypeVString));
    this->ui.pDatatype->addItem(SdoTransferDialog::TEXT_OSTRING_DOMAIN, QVariant::fromValue(kObdTypeDomain));
    this->ui.pDatatype->setCurrentIndex(2);

    // Add available transfer types
    this->ui.pMethod->addItem(QString("ASnd"), QVariant::fromValue(kSdoTypeAsnd));
    this->ui.pMethod->addItem(QString("UDP"), QVariant::fromValue(kSdoTypeUdp));

    // Custom signal/slot
    QObject::connect(this,
                     SIGNAL(sigUpdateData(const QString&)),
                     this,
                     SLOT(updateData(const QString&)),
                     Qt::QueuedConnection);
}

//============================================================================//
//            P R I V A T E   M E M B E R   F U N C T I O N S                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Start SDO read

Start SDO transfer within stack event thread.
*/
//------------------------------------------------------------------------------
void SdoTransferDialog::startRead()
{
    this->enableFields(false);
    this->readFields();

    // Resize the data buffer according to the data type
    eObdType obdType = this->ui.pDatatype->itemData(this->ui.pDatatype->currentIndex()).value<eObdType>();
    this->data.resize(static_cast<int>(SdoTransferDialog::getSizeOfObdType(obdType)));

    // Trigger the read (send a request to the event thread of the stack)
    tOplkError ret = oplk_postUserEvent(this->ui.pReadButton);
    if (ret != kErrorOk)
        this->enableFields(true);
}

//------------------------------------------------------------------------------
/**
\brief  Start SDO write

Start SDO transfer within stack event thread.
*/
//------------------------------------------------------------------------------
void SdoTransferDialog::startWrite()
{
    this->enableFields(false);
    this->readFields();

    eObdType obdType = this->ui.pDatatype->itemData(this->ui.pDatatype->currentIndex()).value<eObdType>();
    switch (obdType)
    {
        case kObdTypeVString:
            this->data = this->ui.pData->text().toLatin1();
            break;

        case kObdTypeDomain:
            this->data = QByteArray::fromHex(this->ui.pData->text().toLatin1());
            break;

        default:
            QStringList list = this->ui.pData->text().split(SdoTransferDialog::REGEX_HEX_NUMBER, QString::SkipEmptyParts);

            if (list.count() < 1)
            {
                QMessageBox::critical(this,
                                      QString("Data invalid"),
                                      QString("The specified data field does not consist of one number. Please correct and try again."));
                this->enableFields(true);
                return;
            }

            switch (obdType)
            {
                case kObdTypeUInt8:
                case kObdTypeUInt16:
                case kObdTypeUInt32:
                case kObdTypeUInt64:
                {
                    bool fConvOk = false;
                    quint64 uval = list[0].toULongLong(&fConvOk, 0);

                    if (!fConvOk)
                    {
                        QMessageBox::critical(this,
                                              QString("Data invalid"),
                                              QString("The specified data format is not an unsigned integer. Please correct and try again."));
                        this->enableFields(true);
                        return;
                    }

                    this->data.resize(sizeof(quint64));
                    qToLittleEndian<quint64>(uval, reinterpret_cast<uchar*>(this->data.data()));
                    break;
                }

                case kObdTypeInt8:
                case kObdTypeInt16:
                case kObdTypeInt32:
                case kObdTypeInt64:
                {
                    bool fConvOk = false;
                    qint64 val = list[0].toLongLong(&fConvOk, 0);
                    if (!fConvOk)
                    {
                        QMessageBox::critical(this,
                                              QString("Data invalid"),
                                              QString("The specified data format is not a signed integer. Please correct and try again."));
                        this->enableFields(true);
                        return;
                    }

                    this->data.resize(sizeof(qint64));
                    qToLittleEndian<qint64>(val, reinterpret_cast<uchar*>(this->data.data()));
                    break;
                }
            }

            // Resize the data buffer according to the data type
            this->data.resize(static_cast<int>(SdoTransferDialog::getSizeOfObdType(obdType)));
    }

    // Trigger the write (send a request to the event thread of the stack)
    tOplkError ret = oplk_postUserEvent(this->ui.pWriteButton);
    if (ret != kErrorOk)
        this->enableFields(true);
}

//------------------------------------------------------------------------------
/**
\brief  dataTypeChanged handler

Will be called if another list item was selected.

\param[in]      index_p             current selected item
*/
//------------------------------------------------------------------------------
void SdoTransferDialog::dataTypeChanged(int index_p)
{
    this->updateData(QString());
}

//------------------------------------------------------------------------------
/**
\brief  Update data from internal byte array

\param[in]      abortCode_p         SDO abort code string
*/
//------------------------------------------------------------------------------
void SdoTransferDialog::updateData(const QString& abortCode_p)
{
    QString dataString;

    if (!abortCode_p.isEmpty())
        this->ui.pAbortCode->setText(abortCode_p);

    eObdType obdType = this->ui.pDatatype->itemData(this->ui.pDatatype->currentIndex()).value<eObdType>();
    switch (obdType)
    {
        case kObdTypeUInt8:
            if (this->data.size() >= sizeof(quint8))
            {
                quint8 val = *reinterpret_cast<const quint8*>(this->data.data());
                dataString = QString("0x%1 = %2")
                                     .arg(val, 2 * sizeof(quint8), SdoTransferDialog::NUMBER_BASE_HEX, QLatin1Char('0'))
                                     .arg(val);
            }
            else
                dataString = SdoTransferDialog::TEXT_ERR_TOO_LESS_DATA + SdoTransferDialog::TEXT_UNSIGNED8;
            break;

        case kObdTypeInt8:
            if (this->data.size() >= sizeof(qint8))
            {
                qint8 val = *reinterpret_cast<qint8*>(this->data.data());
                dataString = QString("%1").arg(val);
            }
            else
                dataString = SdoTransferDialog::TEXT_ERR_TOO_LESS_DATA + SdoTransferDialog::TEXT_INTEGER8;
            break;

        case kObdTypeUInt16:
            if (this->data.size() >= sizeof(quint16))
            {
                quint16 val = qFromLittleEndian<quint16>(reinterpret_cast<const uchar*>(this->data.data()));
                dataString = QString("0x%1 = %2")
                                     .arg(val, 2 * sizeof(quint16), SdoTransferDialog::NUMBER_BASE_HEX, QLatin1Char('0'))
                                     .arg(val);
            }
            else
                dataString = SdoTransferDialog::TEXT_ERR_TOO_LESS_DATA + SdoTransferDialog::TEXT_UNSIGNED16;
            break;

        case kObdTypeInt16:
            if (this->data.size() >= sizeof(qint16))
            {
                qint16 val = qFromLittleEndian<qint16>(reinterpret_cast<const uchar*>(this->data.data()));
                dataString = QString("%1").arg(val);
            }
            else
                dataString = SdoTransferDialog::TEXT_ERR_TOO_LESS_DATA + SdoTransferDialog::TEXT_INTEGER16;
            break;

        case kObdTypeUInt32:
            if (this->data.size() >= sizeof(quint32))
            {
                quint32 val = qFromLittleEndian<quint32>(reinterpret_cast<const uchar*>(this->data.data()));
                dataString = QString("0x%1 = %2")
                                     .arg(val, 2 * sizeof(quint32), SdoTransferDialog::NUMBER_BASE_HEX, QLatin1Char('0'))
                                     .arg(val);
            }
            else
                dataString = SdoTransferDialog::TEXT_ERR_TOO_LESS_DATA + SdoTransferDialog::TEXT_UNSIGNED32;
            break;

        case kObdTypeInt32:
            if (this->data.size() >= sizeof(qint32))
            {
                qint32 val = qFromLittleEndian<qint32>(reinterpret_cast<const uchar*>(this->data.data()));
                dataString = QString("%1").arg(val);
            }
            else
                dataString = SdoTransferDialog::TEXT_ERR_TOO_LESS_DATA + SdoTransferDialog::TEXT_INTEGER32;
            break;

        case kObdTypeUInt64:
            if (this->data.size() >= sizeof(quint64))
            {
                quint64 val = qFromLittleEndian<quint64>(reinterpret_cast<const uchar*>(this->data.data()));
                dataString = QString("0x%1 = %2")
                                     .arg(val, 2 * sizeof(quint64), SdoTransferDialog::NUMBER_BASE_HEX, QLatin1Char('0'))
                                     .arg(val);
            }
            else
                dataString = SdoTransferDialog::TEXT_ERR_TOO_LESS_DATA + SdoTransferDialog::TEXT_UNSIGNED64;
            break;

        case kObdTypeInt64:
            if (this->data.size() >= sizeof(qint64))
            {
                qint64 val = qFromLittleEndian<qint64>(reinterpret_cast<const uchar*>(this->data.data()));
                dataString = QString("%1").arg(val);
            }
            else
                dataString = SdoTransferDialog::TEXT_ERR_TOO_LESS_DATA + SdoTransferDialog::TEXT_INTEGER64;
            break;

        case kObdTypeVString:
            dataString = QString(this->data);
            break;

        case kObdTypeDomain:
            dataString = QString(this->data.toHex());
            break;
    }

    this->ui.pData->setText(dataString);
    this->enableFields(true);
}

//------------------------------------------------------------------------------
/**
\brief  userDefEvent handler

\param[in,out]  pUserArg_p          user-defined argument
*/
//------------------------------------------------------------------------------
void SdoTransferDialog::userDefEvent(void* pUserArg_p)
{
    tSdoComConHdl   sdoComConHdl;
    tObdSize        obdSize;
    tOplkError      ret;

    obdSize = this->data.size();

    if (pUserArg_p == this->ui.pReadButton)
    {
        ret = oplk_readObject(&sdoComConHdl,
                              this->targetNodeId,
                              this->targetIndex,
                              this->targetSubindex,
                              this->data.data(),
                              &obdSize,
                              static_cast<tSdoType>(this->sdoType),
                              NULL);
        if (ret == kErrorApiTaskDeferred)
        {   // SDO transfer started
            return;
        }
        else if (ret == kErrorOk)
        {   // local OD access
            this->data.resize(static_cast<int>(obdSize));

            emit sigUpdateData(QString("Successfully read from local OD"));
        }
        else
        {
            this->data.resize(0);

            emit sigUpdateData(QString("Error 0x%1: %2")
                                       .arg(ret, 0, SdoTransferDialog::NUMBER_BASE_HEX)
                                       .arg(debugstr_getRetValStr(ret)));
        }
    }
    else if (pUserArg_p == this->ui.pWriteButton)
    {
        ret = oplk_writeObject(&sdoComConHdl,
                               this->targetNodeId,
                               this->targetIndex,
                               this->targetSubindex,
                               this->data.data(),
                               obdSize,
                               static_cast<tSdoType>(this->sdoType),
                               NULL);
        if (ret == kErrorApiTaskDeferred)
        {   // SDO transfer started
            return;
        }
        else if (ret == kErrorOk)
        {   // local OD access
            this->data.resize(static_cast<int>(obdSize));

            emit sigUpdateData(QString("Successfully written to local OD"));
        }
        else
        {
            this->data.resize(0);

            emit sigUpdateData(QString("Error 0x%1: %2")
                                       .arg(ret, 0, SdoTransferDialog::NUMBER_BASE_HEX)
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
void SdoTransferDialog::sdoFinished(tSdoComFinished sdoInfo_p)
{
    switch (sdoInfo_p.sdoComConState)
    {
        case kSdoComTransferFinished:
            this->data.resize(static_cast<int>(sdoInfo_p.transferredBytes));

            emit sigUpdateData(QString("Successfully %1 data")
                                       .arg((sdoInfo_p.sdoAccessType == kSdoAccessTypeWrite ? "written" : "read")));
            break;

        default:
        case kSdoComTransferRxAborted:
        case kSdoComTransferTxAborted:
            emit sigUpdateData(QString("0x%1 = %2")
                                       .arg(sdoInfo_p.abortCode, 2 * sizeof(quint64), SdoTransferDialog::NUMBER_BASE_HEX, QLatin1Char('0'))
                                       .arg(debugstr_getAbortCodeStr(sdoInfo_p.abortCode)));
            break;

        case kSdoComTransferLowerLayerAbort:
            emit sigUpdateData(QString("Lower layer abort (timeout)"));
            break;
    }
}

//------------------------------------------------------------------------------
/**
\brief  Enable/disable fields in form

\param[in]      enable_p            true = enable, false = disable
*/
//------------------------------------------------------------------------------
void SdoTransferDialog::enableFields(bool enable_p)
{
    this->ui.pNodeId->setEnabled(enable_p);
    this->ui.pObject->setEnabled(enable_p);
    this->ui.pSubobject->setEnabled(enable_p);
    this->ui.pDatatype->setEnabled(enable_p);
    this->ui.pMethod->setEnabled(enable_p);
    this->ui.pReadButton->setEnabled(enable_p);
    this->ui.pWriteButton->setEnabled(enable_p);
}

//------------------------------------------------------------------------------
/**
\brief  Read fields from form

Read the values from the UI elements and store them in class members.
*/
//------------------------------------------------------------------------------
void SdoTransferDialog::readFields()
{
    // node-ID
    this->targetNodeId = this->ui.pNodeId->value();

    // object index and sub-index
    this->targetIndex = this->ui.pObject->value();
    this->targetSubindex = this->ui.pSubobject->value();

    // SDO type
    this->sdoType = this->ui.pMethod->itemData(this->ui.pMethod->currentIndex()).value<eSdoType>();
}

//------------------------------------------------------------------------------
/**
\brief  Calculate the size of a given OBD type.

\param[in]      obdType_p          OBD type of which the size shall be returned

\retuns Returns the size of the given OBD type.
*/
//------------------------------------------------------------------------------
size_t SdoTransferDialog::getSizeOfObdType(eObdType obdType_p)
{
    size_t  size = 0;

    switch (obdType_p)
    {
        case kObdTypeUInt8:
        case kObdTypeInt8:
            size = sizeof(quint8);
            break;

        case kObdTypeUInt16:
        case kObdTypeInt16:
            size = sizeof(quint16);
            break;

        case kObdTypeUInt32:
        case kObdTypeInt32:
            size = sizeof(quint32);
            break;

        case kObdTypeUInt64:
        case kObdTypeInt64:
            size = sizeof(quint64);
            break;

        case kObdTypeVString:
        case kObdTypeDomain:
            size = SdoTransferDialog::MAX_SIZE_OF_DOMAIN;
            break;
    }

    return size;
}
