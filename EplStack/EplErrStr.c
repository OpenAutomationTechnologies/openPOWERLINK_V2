/* EplErrStr.c
 * ---------------------------------------------------------------------------
 * History
 * 16/06/2010 Original version ...................................... E. Dumas
 * 24/06/2010 Rename EplGetStrError as EplErrStrGetMessage .......... E. Dumas
 * ---------------------------------------------------------------------------
 * $Revision$
 */

/* ---------------------------------------------------------------------------
 * Includes
 * ---------------------------------------------------------------------------
 */

#include "global.h"
#include "EplErrStr.h"


/* ---------------------------------------------------------------------------
 * local types
 * ---------------------------------------------------------------------------
 */


typedef struct _tEplErrStrDefinition
{
    tEplKernel  m_ErrorCode;
    char*       m_pszMsg;

} tEplErrStrDefinition;


/* ---------------------------------------------------------------------------
 * const defines
 * ---------------------------------------------------------------------------
 */

#define EPL_ERRSTR_LAST_ERROR 0xffffffff


static CONST tEplErrStrDefinition ROM aEplErrStrDefinition_g[] =
{
    /* area for generic errors 0x0000 - 0x000F */
    {kEplSuccessful,           "no error/successful run"},
    {kEplIllegalInstance,      "the called instance does not exist"},
    {kEplInvalidInstanceParam, "invalid instance parameter"},
    {kEplNoFreeInstance,       "XxxAddInstance was called but no free instance is available"},
    {kEplWrongSignature,       "wrong signature while writing to object 0x1010 or 0x1011"},
    {kEplInvalidOperation,     "operation not allowed in this situation"},
    {kEplInvalidNodeId,        "invalid NodeId was specified"},
    {kEplNoResource,           "resource could not be created (Windows, PxROS, ...)"},
    {kEplShutdown,             "stack is shutting down"},
    {kEplReject,               "reject the subsequent command"},
    {kEplRetry,                "retry this command"},
    {kEplInvalidEvent,         "invalid event was posted to process function"},

    /* area for EDRV module 0x0010 - 0x001F */
    {kEplEdrvInitError,         "initialisation error"},
    {kEplEdrvNoFreeBufEntry,    "no free entry in internal buffer table for Tx frames"},
    {kEplEdrvBufNotExisting,    "specified Tx buffer does not exist"},
    {kEplEdrvInvalidParam,      "invalid parameter in function call"},

    /* area for DLL module 0x0020 - 0x002F */
    {kEplDllIllegalHdl,         "illegal handle for a TxFrame was passed"},
    {kEplDllCbAsyncRegistered,  "handler for non-EPL frames was already registered before"},
    {kEplDllAsyncTxBufferEmpty, "transmit buffer for asynchronous frames is empty"},
    {kEplDllAsyncTxBufferFull,  "transmit buffer for asynchronous frames is full"},
    {kEplDllNoNodeInfo,         "MN: too less space in the internal node info structure"},
    {kEplDllInvalidParam,       "invalid parameters passed to function"},
    {kEplDllTxBufNotReady,      "TxBuffer (e.g. for PReq) is not ready yet"},
    {kEplDllTxFrameInvalid,     "TxFrame (e.g. for PReq) is invalid or does not exist"},

    /* area for OBD module 0x0030 - 0x003F */
    {kEplObdIllegalPart,        "unknown OD part"},
    {kEplObdIndexNotExist,      "object index does not exist in OD"},
    {kEplObdSubindexNotExist,   "subindex does not exist in object index"},
    {kEplObdReadViolation,       "read access to a write-only object"},
    {kEplObdWriteViolation,      "write access to a read-only object"},
    {kEplObdAccessViolation,     "access not allowed"},
    {kEplObdUnknownObjectType,   "object type not defined/known"},
    {kEplObdVarEntryNotExist,    "object does not contain VarEntry structure"},
    {kEplObdValueTooLow,         "value to write to an object is too low"},
    {kEplObdValueTooHigh,        "value to write to an object is too high"},
    {kEplObdValueLengthError,    "value to write is to long or to short"},
    {kEplObdErrnoSet,            "file I/O error occurred and errno is set"},
    {kEplObdInvalidDcf,          "device configuration file (CDC) is not valid"},
    {kEplObdOutOfMemory,         "out of memory"},
    {kEplObdNoConfigData,        "no configuration data present (CDC is empty)"},


    /* area for NMT module 0x0040 - 0x004F */
    {kEplNmtUnknownCommand,      "unknown NMT command"},
    {kEplNmtInvalidFramePointer, "pointer to the frame is not valid"},
    {kEplNmtInvalidEvent,        "invalid event send to NMT-modul"},
    {kEplNmtInvalidState,        "unknown state in NMT-State-Maschine"},
    {kEplNmtInvalidParam,        "invalid parameters specified"},

    /* area for SDO/UDP module 0x0050 - 0x005F */
    {kEplSdoUdpMissCb,           "missing callback-function pointer during init of module"},
    {kEplSdoUdpNoSocket,         "error during init of socket"},
    {kEplSdoUdpSocketError,      "error during usage of socket"},
    {kEplSdoUdpThreadError,      "error during start of listen thread"},
    {kEplSdoUdpNoFreeHandle,     "no free connection handle for Udp"},
    {kEplSdoUdpSendError,        "Error during send of frame"},
    {kEplSdoUdpInvalidHdl,       "the connection handle is invalid"},

    /* area for SDO Sequence layer module 0x0060 - 0x006F */
    {kEplSdoSeqMissCb,           "no callback-function assign"},
    {kEplSdoSeqNoFreeHandle,     "no free handle for connection"},
    {kEplSdoSeqInvalidHdl,       "invalid handle in SDO sequence layer"},
    {kEplSdoSeqUnsupportedProt,  "unsupported Protocol selected"},
    {kEplSdoSeqNoFreeHistory,    "no free entry in history"},
    {kEplSdoSeqFrameSizeError,   "the size of the frames is not correct"},
    {kEplSdoSeqRequestAckNeeded, "indicates that the history buffer is full and a ack request is needed"},
    {kEplSdoSeqInvalidFrame,     "frame not valid"},
    {kEplSdoSeqConnectionBusy,   "connection is busy -> retry later"},
    {kEplSdoSeqInvalidEvent,     "invalid event received"},

    /* area for SDO Command Layer Module 0x0070 - 0x007F */
    {kEplSdoComUnsupportedProt,  "unsupported Protocol selected"},
    {kEplSdoComNoFreeHandle,     "no free handle for connection"},
    {kEplSdoComInvalidServiceType, "invalid SDO service type specified"},
    {kEplSdoComInvalidHandle,    "handle invalid"},
    {kEplSdoComInvalidSendType,  "the stated to of frame to send is not possible"},
    {kEplSdoComNotResponsible,   "internal error: command layer handle is not responsible for this event from sequence layer"},
    {kEplSdoComHandleExists,     "handle to same node already exists"},
    {kEplSdoComHandleBusy,       "transfer via this handle is already running"},
    {kEplSdoComInvalidParam,     "invalid parameters passed to function"},

    /* area for EPL Event-Modul 0x0080 - 0x008F */
    {kEplEventUnknownSink,       "unknown sink for event"},
    {kEplEventPostError,         "error during post of event"},

    /* area for EPL Timer Modul 0x0090 - 0x009F */
    {kEplTimerInvalidHandle,     "invalid handle for timer"},
    {kEplTimerNoTimerCreated,    "no timer was created caused by an error"},
    {kEplTimerThreadError,       "process thread could not be created"},

    /* area for EPL SDO/Asnd Module 0x00A0 - 0x0AF */
    {kEplSdoAsndInvalidNodeId,   "node id is invalid"},
    {kEplSdoAsndNoFreeHandle,    "no free handle for connection"},
    {kEplSdoAsndInvalidHandle,   "handle for connection is invalid"},

    /* area for PDO module 0x00B0 - 0x00BF  */
    {kEplPdoNotExist,            "selected PDO does not exist"},
    {kEplPdoLengthExceeded,      "length of PDO mapping exceeds the current payload limit"},
    {kEplPdoGranularityMismatch, "configured PDO granularity is not equal to supported granularity"},
    {kEplPdoInitError,           "error during initialisation of PDO module"},
    {kEplPdoConfWhileEnabled,    "PDO configuration cannot be changed while it is enabled"},
    {kEplPdoErrorMapp,           "invalid PDO mapping"},
    {kEplPdoVarNotFound,         "the referenced object in a PDO mapping does not exist"},
    {kEplPdoSizeMismatch,        "bit size of object mapping is larger than the object size"},
    {kEplPdoTooManyTxPdos,       "there exits more than one TPDO on CN"},
    {kEplPdoInvalidObjIndex,     "invalid object index used for PDO mapping or communication parameter"},
    {kEplPdoTooManyPdos,         "there exit to many PDOs"},


    /* Configuration manager module 0x00C0 - 0x00CF */
    {kEplCfmConfigError,         "error in configuration manager"},
    {kEplCfmSdocTimeOutError,    "error in configuration manager, Sdo timeout"},
    {kEplCfmInvalidDcf,          "device configuration file (CDC) is not valid"},
    {kEplCfmUnsupportedDcf,      "unsupported DCF format"},
    {kEplCfmConfigWithErrors,    "configuration finished with errors"},
    {kEplCfmNoFreeConfig,        "no free configuration entry"},
    {kEplCfmNoConfigData,        "no configuration data present"},
    {kEplCfmUnsuppDatatypeDcf,   "unsupported datatype found in DCF -> this entry was not configured"},


    {kEplApiTaskDeferred,        "EPL performs task in background and informs the application (or vice-versa), when it is finished"},
    {kEplApiInvalidParam,        "passed invalid parameters to a function (e.g. invalid node id)"},
    {kEplApiNoObdInitRam,        "no function pointer for ObdInitRam supplied"},
    {kEplApiSdoBusyIntern,       "the SDO channel to this node is internally used by the stack (e.g. the CFM) and currently not available for the application."},

    /* area untill 0x07FF is reserved
    * area for user application from 0x0800 to 0x7FFF
    */

    {EPL_ERRSTR_LAST_ERROR, "undefined error"}
};


/* ---------------------------------------------------------------------------
 * local function prototypes
 * ---------------------------------------------------------------------------
 */


/* ---------------------------------------------------------------------------
 * function
 * ---------------------------------------------------------------------------
 */

/* Convert error code into a human readable string
 * E. Dumas 16/06/2010
 */
const char* EplErrStrGetMessage(tEplKernel Error_p)
{
    int nIndex;

    nIndex = 0;
    while (aEplErrStrDefinition_g[nIndex].m_ErrorCode != EPL_ERRSTR_LAST_ERROR)
    {
        if (aEplErrStrDefinition_g[nIndex].m_ErrorCode == Error_p)
        {
            return aEplErrStrDefinition_g[nIndex].m_pszMsg;
        }
        nIndex++;
    }

    return aEplErrStrDefinition_g[nIndex].m_pszMsg;
} /* EplErrStrGetMessage() */


/* end of file */

