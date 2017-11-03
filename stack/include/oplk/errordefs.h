/**
********************************************************************************
\file   oplk/errordefs.h

\brief  openPOWERLINK error definitions

Definitions of return values and errors.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, B&R Industrial Automation GmbH
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
#ifndef _INC_oplk_errordefs_H_
#define _INC_oplk_errordefs_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Emergency error codes
#define E_NO_ERROR                          0x0000
// 0xFxxx manufacturer specific error codes
#define E_NMT_NO_IDENT_RES                  0xF001
#define E_NMT_NO_STATUS_RES                 0xF002

// 0x816x HW errors
#define E_DLL_BAD_PHYS_MODE                 0x8161
#define E_DLL_COLLISION                     0x8162
#define E_DLL_COLLISION_TH                  0x8163
#define E_DLL_CRC_TH                        0x8164
#define E_DLL_LOSS_OF_LINK                  0x8165
#define E_DLL_MAC_BUFFER                    0x8166
// 0x82xx Protocol errors
#define E_DLL_ADDRESS_CONFLICT              0x8201
#define E_DLL_MULTIPLE_MN                   0x8202
// 0x821x Frame size errors
#define E_PDO_SHORT_RX                      0x8210
#define E_PDO_MAP_VERS                      0x8211
#define E_NMT_ASND_MTU_DIF                  0x8212
#define E_NMT_ASND_MTU_LIM                  0x8213
#define E_NMT_ASND_TX_LIM                   0x8214
// 0x823x Timing errors
#define E_NMT_CYCLE_LEN                     0x8231
#define E_DLL_CYCLE_EXCEED                  0x8232
#define E_DLL_CYCLE_EXCEED_TH               0x8233
#define E_NMT_IDLE_LIM                      0x8234
#define E_DLL_JITTER_TH                     0x8235
#define E_DLL_LATE_PRES_TH                  0x8236
#define E_NMT_PREQ_CN                       0x8237
#define E_NMT_PREQ_LIM                      0x8238
#define E_NMT_PRES_CN                       0x8239
#define E_NMT_PRES_RX_LIM                   0x823A
#define E_NMT_PRES_TX_LIM                   0x823B
// 0x824x Frame errors
#define E_DLL_INVALID_FORMAT                0x8241
#define E_DLL_LOSS_PREQ_TH                  0x8242
#define E_DLL_LOSS_PRES_TH                  0x8243
#define E_DLL_LOSS_SOA_TH                   0x8244
#define E_DLL_LOSS_SOC_TH                   0x8245
// 0x84xx BootUp Errors
#define E_NMT_BA1                           0x8410  // other MN in MsNotActive active
#define E_NMT_BA1_NO_MN_SUPPORT             0x8411  // MN is not supported
#define E_NMT_BPO1                          0x8420  // mandatory CN was not found or failed in BootStep1
#define E_NMT_BPO1_GET_IDENT                0x8421  // IdentRes was not received
#define E_NMT_BPO1_DEVICE_TYPE              0x8422  // wrong device type
#define E_NMT_BPO1_VENDOR_ID                0x8423  // wrong vendor ID
#define E_NMT_BPO1_PRODUCT_CODE             0x8424  // wrong product code
#define E_NMT_BPO1_REVISION_NO              0x8425  // wrong revision number
#define E_NMT_BPO1_SERIAL_NO                0x8426  // wrong serial number
#define E_NMT_BPO1_CF_VERIFY                0x8428  // verification of configuration failed
#define E_NMT_BPO1_SW_UPDATE                0x842B  // CN software update failed
#define E_NMT_BPO2                          0x8430  // mandatory CN failed in BootStep2
#define E_NMT_BRO                           0x8440  // CheckCommunication failed for mandatory CN
#define E_NMT_WRONG_STATE                   0x8480  // mandatory CN has wrong NMT state


//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

/**
\brief openPOWERLINK function error codes

The following enumeration specifies the error codes for return values of
openPOWERLINK functions.
*/
typedef enum
{
    // area for generic errors 0x0000 - 0x000F
    kErrorOk                        = 0x0000,       ///< No error/successful run
    kErrorIllegalInstance           = 0x0001,       ///< The called instance does not exist
    kErrorInvalidInstanceParam      = 0x0002,       ///< There was an invalid instance parameter
    kErrorNoFreeInstance            = 0x0003,       ///< XxxAddInstance was called but no free instance is available
    kErrorWrongSignature            = 0x0004,       ///< Wrong signature while writing to object 0x1010 or 0x1011
    kErrorInvalidOperation          = 0x0005,       ///< Operation is not allowed in this situation
    kErrorInvalidNodeId             = 0x0007,       ///< An invalid NodeId was specified
    kErrorNoResource                = 0x0008,       ///< The resource could not be created
    kErrorShutdown                  = 0x0009,       ///< Stack is shutting down
    kErrorReject                    = 0x000A,       /**< \li Reject the subsequent command
                                                         \li  OD access will not be completed immediately, but by callback */
    kErrorRetry                     = 0x000B,       ///< Retry this command
    kErrorInvalidEvent              = 0x000C,       ///< Invalid event was posted
    kErrorGeneralError              = 0x000D,       ///< General error
    kErrorFeatureMismatch           = 0x000E,       ///< Features of user and kernel stack are mismatched

    // area for EDRV module 0x0010 - 0x001F
    kErrorEdrvNoFreeTxDesc          = 0x0011,       ///< No free TX descriptor available
    kErrorEdrvInvalidCycleLen       = 0x0012,       ///< Invalid cycle length (e.g. 0)
    kErrorEdrvInit                  = 0x0013,       ///< Edrv initialization error
    kErrorEdrvNoFreeBufEntry        = 0x0014,       ///< No free entry in internal buffer table for TX frames
    kErrorEdrvBufNotExisting        = 0x0015,       ///< The specified TX buffer does not exist
    kErrorEdrvInvalidRxBuf          = 0x0016,       ///< The specified RX buffer is invalid
    kErrorEdrvInvalidParam          = 0x001C,       ///< Invalid parameter in function call
    kErrorEdrvNextTxListNotEmpty    = 0x001D,       ///< Next TX buffer list is not empty, i.e. still in use
    kErrorEdrvCurTxListEmpty        = 0x001E,       ///< Current TX buffer list is empty, i.e. DLL didn't provide one
    kErrorEdrvTxListNotFinishedYet  = 0x001F,       ///< Current TX buffer list has not been finished yet, but new cycle has started

    // area for DLL module 0x0020 - 0x002F
    kErrorDllOutOfMemory            = 0x0021,       ///< DLL is out of memory
    kErrorDllIllegalHdl             = 0x0022,       ///< Illegal handle for a TxFrame was passed
    kErrorDllCbAsyncRegistered      = 0x0023,       ///< Handler for non-POWERLINK frames was already registered before
    kErrorDllAsyncSyncReqFull       = 0x0024,       ///< The buffer for SyncRequests is full
    kErrorDllAsyncTxBufferEmpty     = 0x0025,       ///< The transmit buffer for asynchronous frames is empty
    kErrorDllAsyncTxBufferFull      = 0x0026,       ///< Transmit buffer for asynchronous frames is full
    kErrorDllNoNodeInfo             = 0x0027,       ///< MN: too less space in the internal node info structure
    kErrorDllInvalidParam           = 0x0028,       ///< Invalid parameters passed to DLL function
    kErrorDllInvalidAsndServiceId   = 0x0029,       ///< Invalid AsndServiceId specified
    kErrorDllTxBufNotReady          = 0x002E,       ///< TxBuffer (e.g. for PReq) is not ready yet
    kErrorDllTxFrameInvalid         = 0x002F,       ///< TxFrame (e.g. for PReq) is invalid or does not exist

    // area for OBD module 0x0030 - 0x003F
    kErrorObdIllegalPart            = 0x0030,       ///< Unknown OD part
    kErrorObdIndexNotExist          = 0x0031,       ///< Object index does not exist in OD
    kErrorObdSubindexNotExist       = 0x0032,       ///< Sub-index does not exist in object index
    kErrorObdReadViolation          = 0x0033,       ///< Read access to a write-only object
    kErrorObdWriteViolation         = 0x0034,       ///< Write access to a read-only object
    kErrorObdAccessViolation        = 0x0035,       ///< Access not allowed
    kErrorObdUnknownObjectType      = 0x0036,       ///< The object type is not defined/known
    kErrorObdVarEntryNotExist       = 0x0037,       ///< The object does not contain a VarEntry structure
    kErrorObdValueTooLow            = 0x0038,       ///< The value to write to an object is too low
    kErrorObdValueTooHigh           = 0x0039,       ///< The value to write to an object is too high
    kErrorObdValueLengthError       = 0x003A,       ///< The value to write is to long or to short
    kErrorObdErrnoSet               = 0x003B,       ///< File I/O error occurred and errno is set
    kErrorObdInvalidDcf             = 0x003C,       ///< The device configuration file (CDC) is not valid
    kErrorObdOutOfMemory            = 0x003D,       ///< Out of memory
    kErrorObdNoConfigData           = 0x003E,       ///< No configuration data present (CDC is empty)

    // area for NMT module 0x0040 - 0x004F
    kErrorNmtUnknownCommand         = 0x0040,       ///< Unknown NMT command
    kErrorNmtInvalidFramePointer    = 0x0041,       ///< Pointer to the frame is not valid
    kErrorNmtInvalidEvent           = 0x0042,       ///< Invalid event send to NMT-module
    kErrorNmtInvalidState           = 0x0043,       ///< Unknown state in NMT state machine
    kErrorNmtInvalidParam           = 0x0044,       ///< Invalid parameters specified
    kErrorNmtSyncReqRejected        = 0x0045,       ///< SyncReq could not be issued

    // area for SDO/UDP module 0x0050 - 0x005F
    kErrorSdoUdpMissCb              = 0x0050,       ///< Missing callback-function pointer during init of module
    kErrorSdoUdpNoSocket            = 0x0051,       ///< Error during init of socket
    kErrorSdoUdpSocketError         = 0x0052,       ///< Error during usage of socket
    kErrorSdoUdpThreadError         = 0x0053,       ///< Error during start of listen thread
    kErrorSdoUdpNoFreeHandle        = 0x0054,       ///< No free connection handle for Udp
    kErrorSdoUdpSendError           = 0x0055,       ///< Error during sending of frame
    kErrorSdoUdpInvalidHdl          = 0x0056,       ///< The connection handle is invalid
    kErrorSdoUdpArpInProgress       = 0x0057,       ///< ARP request is in progress or target node MAC unknown

    // area for SDO Sequence layer module 0x0060 - 0x006F
    kErrorSdoSeqMissCb              = 0x0060,       ///< No SDO callback function is assigned
    kErrorSdoSeqNoFreeHandle        = 0x0061,       ///< No free handle for connection
    kErrorSdoSeqInvalidHdl          = 0x0062,       ///< Invalid handle in SDO sequence layer
    kErrorSdoSeqUnsupportedProt     = 0x0063,       ///< Unsupported Protocol selected
    kErrorSdoSeqNoFreeHistory       = 0x0064,       ///< No free entry in history
    kErrorSdoSeqFrameSizeError      = 0x0065,       ///< The size of the frames is not correct
    kErrorSdoSeqRequestAckNeeded    = 0x0066,       ///< Indicates that the history buffer is full and a ack request is needed
    kErrorSdoSeqInvalidFrame        = 0x0067,       ///< The frame is not valid
    kErrorSdoSeqConnectionBusy      = 0x0068,       ///< Connection is busy -> retry later
    kErrorSdoSeqInvalidEvent        = 0x0069,       ///< Invalid event received

    // area for SDO Command Layer module 0x0070 - 0x007F
    kErrorSdoComUnsupportedProt     = 0x0070,       ///< Unsupported Protocol selected
    kErrorSdoComNoFreeHandle        = 0x0071,       ///< No free handle for connection
    kErrorSdoComInvalidServiceType  = 0x0072,       ///< Invalid SDO service type specified
    kErrorSdoComInvalidHandle       = 0x0073,       ///< Handle is invalid
    kErrorSdoComInvalidSendType     = 0x0074,       ///< The state of frame to send is not possible
    kErrorSdoComNotResponsible      = 0x0075,       ///< Internal error: command layer handle is not responsible for this event from sequence layer
    kErrorSdoComHandleExists        = 0x0076,       ///< Handle to same node already exists
    kErrorSdoComHandleBusy          = 0x0077,       ///< Transfer via this handle is already running
    kErrorSdoComInvalidParam        = 0x0078,       ///< Invalid parameters passed to function

    // area for openPOWERLINK event module 0x0080 - 0x008F
    kErrorEventUnknownSink          = 0x0080,       ///< Unknown sink for event
    kErrorEventPostError            = 0x0081,       ///< Error during post of event
    kErrorEventReadError            = 0x0082,       ///< Error during reading of event from queue
    kErrorEventWrongSize            = 0x0083,       ///< Event arg has wrong size

    // area for openPOWERLINK timer module 0x0090 - 0x009F
    kErrorTimerInvalidHandle        = 0x0090,       ///< Invalid handle for timer
    kErrorTimerNoTimerCreated       = 0x0091,       ///< No timer was created caused by an error
    kErrorTimerThreadError          = 0x0092,       ///< Process thread could not be created

    // area for openPOWERLINK SDO/Asnd module 0x00A0 - 0x0AF
    kErrorSdoAsndInvalidNodeId      = 0x00A0,       ///< Node-ID is invalid
    kErrorSdoAsndNoFreeHandle       = 0x00A1,       ///< No free handle for connection
    kErrorSdoAsndInvalidHandle      = 0x00A2,       ///< Handle for connection is invalid

    // area for PDO module 0x00B0 - 0x00BF
    kErrorPdoNotExist               = 0x00B0,       ///< Selected PDO does not exist
    kErrorPdoLengthExceeded         = 0x00B1,       ///< Length of PDO mapping exceeds the current payload limit
    kErrorPdoGranularityMismatch    = 0x00B2,       ///< Configured PDO granularity is not equal to supported granularity
    kErrorPdoInitError              = 0x00B3,       ///< Error during initialization of PDO module
    kErrorPdoConfWhileEnabled       = 0x00B7,       ///< PDO configuration cannot be changed while it is enabled
    kErrorPdoErrorMapp              = 0x00B8,       ///< Invalid PDO mapping
    kErrorPdoVarNotFound            = 0x00B9,       ///< The referenced object in a PDO mapping does not exist
    kErrorPdoVarNotMappable         = 0x00BA,       ///< The referenced object in a PDO mapping is not mappable
    kErrorPdoSizeMismatch           = 0x00BC,       ///< Bit size of object mapping is larger than the object size
    kErrorPdoTooManyTxPdos          = 0x00BD,       ///< There exits more than one TPDO on CN
    kErrorPdoInvalidObjIndex        = 0x00BE,       ///< Invalid object index used for PDO mapping or communication parameter
    kErrorPdoTooManyPdos            = 0x00BF,       ///< Too many PDOs do exist

    // Configuration manager module 0x00C0 - 0x00CF
    kErrorCfmConfigError            = 0x00C0,       ///< Error in configuration manager
    kErrorCfmSdocTimeOutError       = 0x00C1,       ///< Error in configuration manager, SDO timeout
    kErrorCfmInvalidDcf             = 0x00C2,       ///< Device configuration file (CDC) is not valid
    kErrorCfmUnsupportedDcf         = 0x00C3,       ///< Unsupported Dcf format
    kErrorCfmConfigWithErrors       = 0x00C4,       ///< Configuration finished with errors
    kErrorCfmNoFreeConfig           = 0x00C5,       ///< No free configuration entry
    kErrorCfmNoConfigData           = 0x00C6,       ///< No configuration data present
    kErrorCfmUnsuppDatatypeDcf      = 0x00C7,       ///< Unsupported datatype found in dcf -> this entry was not configured

    // area for OD configuration store restore module 0x0D0 - 0x0DF
    kErrorObdStoreHwError           = 0x00D0,       ///< HW error while accessing non-volatile memory
    kErrorObdStoreInvalidState      = 0x00D1,       ///< Non-volatile memory is in invalid state (nothing saved)
    kErrorObdStoreDataLimitExceeded = 0x00D2,       ///< Data count is less than the expected size
    kErrorObdStoreDataObsolete      = 0x00D3,       ///< Data stored in the archive is obsolete

    kErrorApiTaskDeferred           = 0x0140,       ///< openPOWERLINK performs task in background and informs the application (or vice-versa), when it is finished
    kErrorApiInvalidParam           = 0x0142,       ///< Passed invalid parameters to a function (e.g. invalid node id)
    kErrorApiNoObdInitRam           = 0x0143,       ///< No function pointer for ObdInitRam supplied
    kErrorApiSdoBusyIntern          = 0x0144,       /**< The SDO channel to this node is internally used by the stack (e.g. the CFM)
                                                         and currently not available for the application (or vice versa). */
    kErrorApiPIAlreadyAllocated     = 0x0145,       ///< Process image is already allocated
    kErrorApiPIOutOfMemory          = 0x0146,       ///< Process image: out of memory
    kErrorApiPISizeExceeded         = 0x0147,       ///< Process image: variable linking or copy job exceeds the size of the PI
    kErrorApiPINotAllocated         = 0x0148,       ///< Process image is not allocated
    kErrorApiPIJobQueueFull         = 0x0149,       ///< Process image: job queue is full
    kErrorApiPIJobQueueEmpty        = 0x014A,       ///< Process image: job queue is empty
    kErrorApiPIInvalidJobSize       = 0x014B,       ///< Process image: invalid job size
    kErrorApiPIInvalidPIPointer     = 0x014C,       ///< Process image: pointer to application's process image is invalid
    kErrorApiPINonBlockingNotSupp   = 0x014D,       ///< Process image: non-blocking copy jobs are not supported on this target
    kErrorApiNotInitialized         = 0x014E,       ///< API called but stack is not initialized/running
    kErrorApiNotSupported           = 0x014F,       ///< API call requires unsupported feature

    // area until 0x07FF is reserved
    // area for user application from 0x0800 to 0x7FFF
} eOplkError;

/**
\brief openPOWERLINK function error code data type

Data type for the enumerator \ref eOplkError.
*/
typedef UINT32 tOplkError;

#endif /* _INC_oplk_errordefs_H_ */
