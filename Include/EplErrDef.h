/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  definitions for all EPL-function return codes

  License:

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.

    3. Neither the name of SYSTEC electronic GmbH nor the names of its
       contributors may be used to endorse or promote products derived
       from this software without prior written permission. For written
       permission, please contact info@systec-electronic.com.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    Severability Clause:

        If a provision of this License is or becomes illegal, invalid or
        unenforceable in any jurisdiction, that shall not affect:
        1. the validity or enforceability in that jurisdiction of any other
           provision of this License; or
        2. the validity or enforceability in other jurisdictions of that or
           any other provision of this License.

  -------------------------------------------------------------------------

                $RCSfile$

                $Author$

                $Revision$  $Date$

                $State$

                Build Environment:
                    all

  -------------------------------------------------------------------------

  Revision History:

  2005/12/05 -as:   start of the implementation, version 1.00

****************************************************************************/

#ifndef _EPL_ERRORDEF_H_
#define _EPL_ERRORDEF_H_


//---------------------------------------------------------------------------
// return codes
//---------------------------------------------------------------------------

typedef enum
{
    // area for generic errors 0x0000 - 0x000F
    kEplSuccessful              = 0x0000,       // no error/successful run
    kEplIllegalInstance         = 0x0001,       // the called instance does not exist
    kEplInvalidInstanceParam    = 0x0002,       //
    kEplNoFreeInstance          = 0x0003,       // XxxAddInstance was called but no free instance is available
    kEplWrongSignature          = 0x0004,       // wrong signature while writing to object 0x1010 or 0x1011
    kEplInvalidOperation        = 0x0005,       // operation not allowed in this situation
    kEplInvalidNodeId           = 0x0007,       // invalid NodeId was specified
    kEplNoResource              = 0x0008,       // resource could not be created (Windows, PxROS, ...)
    kEplShutdown                = 0x0009,       // stack is shutting down
    kEplReject                  = 0x000A,       // reject the subsequent command
    kEplRetry                   = 0x000B,       // retry this command
    kEplInvalidEvent            = 0x000C,       // invalid event was posted to process function

    // area for EDRV module 0x0010 - 0x001F
    kEplEdrvNoFreeTxDesc        = 0x0011,       // no free Tx descriptor available
    kEplEdrvInvalidCycleLen     = 0x0012,       // invalid cycle length (e.g. 0)
    kEplEdrvInitError           = 0x0013,       // initialisation error
    kEplEdrvNoFreeBufEntry      = 0x0014,       // no free entry in internal buffer table for Tx frames
    kEplEdrvBufNotExisting      = 0x0015,       // specified Tx buffer does not exist
    kEplEdrvInvalidRxBuf        = 0x0016,       // specified Rx buffer is invalid
    kEplEdrvInvalidParam        = 0x001C,       // invalid parameter in function call
    kEplEdrvNextTxListNotEmpty  = 0x001D,       // next Tx buffer list is not empty, i.e. still in use
    kEplEdrvCurTxListEmpty      = 0x001E,       // current Tx buffer list is empty, i.e. DLL didn't provide one
    kEplEdrvTxListNotFinishedYet= 0x001F,       // current Tx buffer list has not been finished yet, but new cycle has started

    // area for DLL module 0x0020 - 0x002F
    kEplDllOutOfMemory          = 0x0021,       // out of memory
    kEplDllIllegalHdl           = 0x0022,       // illegal handle for a TxFrame was passed
    kEplDllCbAsyncRegistered    = 0x0023,       // handler for non-EPL frames was already registered before
    kEplDllAsyncSyncReqFull     = 0x0024,       // buffer for SyncRequests is full
    kEplDllAsyncTxBufferEmpty   = 0x0025,       // transmit buffer for asynchronous frames is empty
    kEplDllAsyncTxBufferFull    = 0x0026,       // transmit buffer for asynchronous frames is full
    kEplDllNoNodeInfo           = 0x0027,       // MN: too less space in the internal node info structure
    kEplDllInvalidParam         = 0x0028,       // invalid parameters passed to function
    kEplDllInvalidAsndServiceId = 0x0029,       // invalid AsndServiceId specified
    kEplDllTxBufNotReady        = 0x002E,       // TxBuffer (e.g. for PReq) is not ready yet
    kEplDllTxFrameInvalid       = 0x002F,       // TxFrame (e.g. for PReq) is invalid or does not exist

    // area for OBD module 0x0030 - 0x003F
    kEplObdIllegalPart          = 0x0030,       // unknown OD part
    kEplObdIndexNotExist        = 0x0031,       // object index does not exist in OD
    kEplObdSubindexNotExist     = 0x0032,       // subindex does not exist in object index
    kEplObdReadViolation        = 0x0033,       // read access to a write-only object
    kEplObdWriteViolation       = 0x0034,       // write access to a read-only object
    kEplObdAccessViolation      = 0x0035,       // access not allowed
    kEplObdUnknownObjectType    = 0x0036,       // object type not defined/known
    kEplObdVarEntryNotExist     = 0x0037,       // object does not contain VarEntry structure
    kEplObdValueTooLow          = 0x0038,       // value to write to an object is too low
    kEplObdValueTooHigh         = 0x0039,       // value to write to an object is too high
    kEplObdValueLengthError     = 0x003A,       // value to write is to long or to short
    kEplObdErrnoSet             = 0x003B,       // file I/O error occurred and errno is set
    kEplObdInvalidDcf           = 0x003C,       // device configuration file (CDC) is not valid
    kEplObdOutOfMemory          = 0x003D,       // out of memory
    kEplObdNoConfigData         = 0x003E,       // no configuration data present (CDC is empty)
//    kEplObdIllegalFloat         = 0x003B,       // illegal float variable
//    kEplObdWrongOdBuilderKey    = 0x003F,       // OD was generated with demo version of tool ODBuilder

    // area for NMT module 0x0040 - 0x004F
    kEplNmtUnknownCommand       = 0x0040,       // unknown NMT command
    kEplNmtInvalidFramePointer  = 0x0041,       // pointer to the frame is not valid
    kEplNmtInvalidEvent         = 0x0042,       // invalid event send to NMT-modul
    kEplNmtInvalidState         = 0x0043,       // unknown state in NMT-State-Maschine
    kEplNmtInvalidParam         = 0x0044,       // invalid parameters specified
    kEplNmtSyncReqRejected      = 0x0045,       // SyncReq could not be issued

    // area for SDO/UDP module 0x0050 - 0x005F
    kEplSdoUdpMissCb            = 0x0050,       // missing callback-function pointer during inti of
                                                // module
    kEplSdoUdpNoSocket          = 0x0051,       // error during init of socket
    kEplSdoUdpSocketError       = 0x0052,       // error during usage of socket
    kEplSdoUdpThreadError       = 0x0053,       // error during start of listen thread
    kEplSdoUdpNoFreeHandle      = 0x0054,       // no free connection handle for Udp
    kEplSdoUdpSendError         = 0x0055,       // Error during send of frame
    kEplSdoUdpInvalidHdl        = 0x0056,       // the connection handle is invalid

    // area for SDO Sequence layer module 0x0060 - 0x006F
    kEplSdoSeqMissCb            = 0x0060,       // no callback-function assign
    kEplSdoSeqNoFreeHandle      = 0x0061,       // no free handle for connection
    kEplSdoSeqInvalidHdl        = 0x0062,       // invalid handle in SDO sequence layer
    kEplSdoSeqUnsupportedProt   = 0x0063,       // unsupported Protocol selected
    kEplSdoSeqNoFreeHistory     = 0x0064,       // no free entry in history
    kEplSdoSeqFrameSizeError    = 0x0065,       // the size of the frames is not correct
    kEplSdoSeqRequestAckNeeded  = 0x0066,       // indicates that the history buffer is full
                                                // and a ack request is needed
    kEplSdoSeqInvalidFrame      = 0x0067,       // frame not valid
    kEplSdoSeqConnectionBusy    = 0x0068,       // connection is busy -> retry later
    kEplSdoSeqInvalidEvent      = 0x0069,       // invalid event received

    // area for SDO Command Layer Module 0x0070 - 0x007F
    kEplSdoComUnsupportedProt   = 0x0070,       // unsupported Protocol selected
    kEplSdoComNoFreeHandle      = 0x0071,       // no free handle for connection
    kEplSdoComInvalidServiceType= 0x0072,       // invalid SDO service type specified
    kEplSdoComInvalidHandle     = 0x0073,       // handle invalid
    kEplSdoComInvalidSendType   = 0x0074,       // the stated to of frame to send is
                                                // not possible
    kEplSdoComNotResponsible    = 0x0075,       // internal error: command layer handle is
                                                // not responsible for this event from sequence layer
    kEplSdoComHandleExists      = 0x0076,       // handle to same node already exists
    kEplSdoComHandleBusy        = 0x0077,       // transfer via this handle is already running
    kEplSdoComInvalidParam      = 0x0078,       // invalid parameters passed to function

    // area for EPL Event-Modul 0x0080 - 0x008F
    kEplEventUnknownSink        = 0x0080,       // unknown sink for event
    kEplEventPostError          = 0x0081,       // error during post of event
    kEplEventReadError          = 0x0082,       // error during reading of event from queue
    kEplEventWrongSize          = 0x0083,       // event arg has wrong size

    // area for EPL Timer Modul 0x0090 - 0x009F
    kEplTimerInvalidHandle      = 0x0090,       // invalid handle for timer
    kEplTimerNoTimerCreated     = 0x0091,       // no timer was created caused by an error
    kEplTimerThreadError        = 0x0092,       // process thread could not be created

    // area for EPL SDO/Asnd Module 0x00A0 - 0x0AF
    kEplSdoAsndInvalidNodeId    = 0x00A0,       // node-ID is invalid
    kEplSdoAsndNoFreeHandle     = 0x00A1,       // no free handle for connection
    kEplSdoAsndInvalidHandle    = 0x00A2,       // handle for connection is invalid

    // area for PDO module 0x00B0 - 0x00BF
    kEplPdoNotExist             = 0x00B0,       // selected PDO does not exist
    kEplPdoLengthExceeded       = 0x00B1,       // length of PDO mapping exceeds the current payload limit
    kEplPdoGranularityMismatch  = 0x00B2,       // configured PDO granularity is not equal to supported granularity
    kEplPdoInitError            = 0x00B3,       // error during initialisation of PDO module
//    kEplPdoErrorPdoEncode       = 0x00B4,       // error during encoding a PDO
//    kEplPdoErrorPdoDecode       = 0x00B5,       // error during decoding a PDO
//    kEplPdoErrorSend            = 0x00B6,       // error during sending a PDO
    kEplPdoConfWhileEnabled     = 0x00B7,       // PDO configuration cannot be changed while it is enabled
    kEplPdoErrorMapp            = 0x00B8,       // invalid PDO mapping
    kEplPdoVarNotFound          = 0x00B9,       // the referenced object in a PDO mapping does not exist
    kEplPdoVarNotMappable       = 0x00BA,       // the referenced object in a PDO mapping is not mappable
//    kEplPdoErrorEmcyPdoLen      = 0x00BA,       // the length of a received PDO is unequal to the expected value
//    kEplPdoWriteConstObject     = 0x00BB,       // constant object can not be written
                                                // (only TxType, Inhibit-, Event Time for CANopen Kit)
    kEplPdoSizeMismatch         = 0x00BC,       // bit size of object mapping is larger than the object size
    kEplPdoTooManyTxPdos        = 0x00BD,       // there exits more than one TPDO on CN
    kEplPdoInvalidObjIndex      = 0x00BE,       // invalid object index used for PDO mapping or communication parameter
    kEplPdoTooManyPdos          = 0x00BF,       // there exist too many PDOs

    // Configuration manager module 0x00C0 - 0x00CF
    kEplCfmConfigError          = 0x00C0,       // error in configuration manager
    kEplCfmSdocTimeOutError     = 0x00C1,       // error in configuration manager, Sdo timeout
    kEplCfmInvalidDcf           = 0x00C2,       // device configuration file (CDC) is not valid
    kEplCfmUnsupportedDcf       = 0x00C3,       // unsupported Dcf format
    kEplCfmConfigWithErrors     = 0x00C4,       // configuration finished with errors
    kEplCfmNoFreeConfig         = 0x00C5,       // no free configuration entry
    kEplCfmNoConfigData         = 0x00C6,       // no configuration data present
    kEplCfmUnsuppDatatypeDcf    = 0x00C7,       // unsupported datatype found in dcf
                                                // -> this entry was not configured

    kEplApiTaskDeferred         = 0x0140,       // EPL performs task in background and informs the application (or vice-versa), when it is finished
    kEplApiInvalidParam         = 0x0142,       // passed invalid parameters to a function (e.g. invalid node id)
    kEplApiNoObdInitRam         = 0x0143,       // no function pointer for ObdInitRam supplied
    kEplApiSdoBusyIntern        = 0x0144,       // the SDO channel to this node is internally used by the stack (e.g. the CFM) and currently not available for the application.
    kEplApiPIAlreadyAllocated   = 0x0145,       // process image is already allocated
    kEplApiPIOutOfMemory        = 0x0146,       // process image: out of memory
    kEplApiPISizeExceeded       = 0x0147,       // process image: variable linking or copy job exceeds the size of the PI
    kEplApiPINotAllocated       = 0x0148,       // process image is not allocated
    kEplApiPIJobQueueFull       = 0x0149,       // process image: job queue is full
    kEplApiPIJobQueueEmpty      = 0x014A,       // process image: job queue is empty
    kEplApiPIInvalidJobSize     = 0x014B,       // process image: invalid job size
    kEplApiPIInvalidPIPointer   = 0x014C,       // process image: pointer to application's process image is invalid
    kEplApiPINonBlockingNotSupp = 0x014D,       // process image: non-blocking copy jobs are not supported on this target

    // area until 0x07FF is reserved
    // area for user application from 0x0800 to 0x7FFF

} tEplKernel;


#endif
//EOF

// Die letzte Zeile muﬂ unbedingt eine leere Zeile sein, weil manche Compiler
// damit ein Problem haben, wenn das nicht so ist (z.B. GNU oder Borland C++ Builder).

