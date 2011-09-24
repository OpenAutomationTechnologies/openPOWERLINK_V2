/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  source file for OBD-CDC module
                This module implements OBD import functionality
                of ConciseDCF (CDC files).

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
                    GCC V3.4

  -------------------------------------------------------------------------

  Revision History:

  2010/01/08 d.k.:   start of the implementation, version 1.00

****************************************************************************/

#define _CRT_NONSTDC_NO_WARNINGS    // for MSVC 2005 or higher
#define _CRT_SECURE_NO_WARNINGS     // for MSVC 2005 or higher

#include "EplInc.h"
#include "EplObd.h"
#include "EplObdCdc.h"
#include "kernel/EplObdk.h"
#include "user/EplEventu.h"

#if (EPL_OBD_USE_LOAD_CONCISEDCF != FALSE)

#if !((TARGET_SYSTEM == _LINUX_) && \
      (defined(__KERNEL__)))
#include <sys/stat.h>
#include <assert.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#endif

#if (TARGET_SYSTEM == _WIN32_)

    #include <io.h>
    #include <sys/types.h>
    #include <sys/utime.h>
    #include <sys/timeb.h>
    #include <time.h>
    #include <direct.h>
    #include <string.h>

#elif (TARGET_SYSTEM == _LINUX_)

    #ifdef __arm__
        #include <sys/io.h>
    #else
        #ifdef __KERNEL__
        #include <asm/io.h>
    #endif
    #endif
    #ifdef __KERNEL__
        #include "PosixFileLinuxKernel.h"
    #else
        #include <unistd.h>
        #include <sys/vfs.h>
        #include <sys/types.h>
        #include <sys/timeb.h>
        #include <utime.h>
        #include <limits.h>
    #endif
#elif (TARGET_SYSTEM == _VXWORKS_)
	#include "ioLib.h"
#elif (DEV_SYSTEM == _DEV_PAR_BECK1X3_)

    #include <io.h>
    #include <string.h>
#endif

#if (TARGET_SYSTEM == _WIN32_)

    #define flush  _commit
    #define mode_t int

#elif (TARGET_SYSTEM == _LINUX_)

    #define O_BINARY 0
    #define _MAX_PATH PATH_MAX
    #define flush  fsync
#elif (TARGET_SYSTEM == _VXWORKS_)
    #define O_BINARY 0
#elif (DEV_SYSTEM == _DEV_PAR_BECK1X3_)

    #define flush(h)                    // #define flush() to nothing
    #define mode_t int

#endif

#ifndef FD_TYPE
#define FD_TYPE     int
#endif

#ifndef IS_FD_VALID
#define IS_FD_VALID(iFd_p)  ((iFd_p) >= 0)
#endif


/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          G L O B A L   D E F I N I T I O N S                            */
/*                                                                         */
/*                                                                         */
/***************************************************************************/

//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// module global vars
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------


/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          C L A S S  EplObdCdc                                           */
/*                                                                         */
/*                                                                         */
/***************************************************************************/
//
// Description:
//
//
/***************************************************************************/


//=========================================================================//
//                                                                         //
//          P R I V A T E   D E F I N I T I O N S                          //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

typedef enum
{
    kEplObdCdcTypeFile      = 0,
    kEplObdCdcTypeBuffer    = 1,

} tEplObdCdcType;


typedef struct
{
    tEplObdCdcType  m_Type;
    union
    {
        FD_TYPE     m_hCdcFile;
        BYTE*       m_pbNextBuffer;
    } m_Handle;
    size_t          m_iCdcSize;
    size_t          m_iBufferSize;
    BYTE*           m_pbCurBuffer;

} tEplObdCdcInfo;


//---------------------------------------------------------------------------
// local vars
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------

static tEplKernel EplObdCdcProcess(tEplObdCdcInfo* pCdcInfo_p);



//=========================================================================//
//                                                                         //
//          P U B L I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//
// Function:    EplObdCdcLoadFile()
//
// Description: loads the CDC file specified by the file name into the local OD.
//
// Parameters:  pszCdcFilename_p    = file name of the CDC file
//
// Returns:     tEplKernel          = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EplObdCdcLoadFile(char* pszCdcFilename_p)
{
tEplKernel      Ret = kEplSuccessful;
tEplObdCdcInfo  CdcInfo;
DWORD           dwErrno;

    EPL_MEMSET(&CdcInfo, 0, sizeof (CdcInfo));
    CdcInfo.m_Type = kEplObdCdcTypeFile;
    CdcInfo.m_Handle.m_hCdcFile = open(pszCdcFilename_p, O_RDONLY | O_BINARY, 0666);
    if (!IS_FD_VALID(CdcInfo.m_Handle.m_hCdcFile))
    {   // error occurred
        dwErrno = (DWORD) errno;
        Ret = EplEventuPostError(kEplEventSourceObdu, kEplObdErrnoSet, sizeof (dwErrno), &dwErrno);
        goto Exit;
    }

    CdcInfo.m_iCdcSize = lseek(CdcInfo.m_Handle.m_hCdcFile, 0, SEEK_END);
    lseek(CdcInfo.m_Handle.m_hCdcFile, 0, SEEK_SET);

    Ret = EplObdCdcProcess(&CdcInfo);

    if (CdcInfo.m_pbCurBuffer != NULL)
    {
        EPL_FREE(CdcInfo.m_pbCurBuffer);
        CdcInfo.m_pbCurBuffer = NULL;
        CdcInfo.m_iBufferSize = 0;
    }

    close(CdcInfo.m_Handle.m_hCdcFile);

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplObdCdcLoadBuffer()
//
// Description: loads the CDC specified by the buffer into the local OD.
//
// Parameters:  pbCdc_p         = pointer to buffer containing the CDC
//              uiCdcSize_p     = size of the CDC
//
// Returns:     tEplKernel      = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EplObdCdcLoadBuffer(BYTE* pbCdc_p, unsigned int uiCdcSize_p)
{
tEplKernel      Ret = kEplSuccessful;
tEplObdCdcInfo  CdcInfo;

    EPL_MEMSET(&CdcInfo, 0, sizeof (CdcInfo));
    CdcInfo.m_Type = kEplObdCdcTypeBuffer;
    CdcInfo.m_Handle.m_pbNextBuffer = pbCdc_p;
    if (CdcInfo.m_Handle.m_pbNextBuffer == NULL)
    {   // error occurred
        Ret = EplEventuPostError(kEplEventSourceObdu, kEplObdInvalidDcf, 0, NULL);
        goto Exit;
    }

    CdcInfo.m_iCdcSize = (size_t) uiCdcSize_p;
    CdcInfo.m_iBufferSize = CdcInfo.m_iCdcSize;

    Ret = EplObdCdcProcess(&CdcInfo);

Exit:
    return Ret;
}



//=========================================================================//
//                                                                         //
//          P R I V A T E   F U N C T I O N S                              //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//
// Function:    EplObdCdcLoadNextBuffer
//
// Description: loads the next part of the buffer
//
// Parameters:  pCdcInfo_p          = pointer to CDC info structure
//              iBufferSize         = number of bytes to load
//
// Returns:     tEplKernel          = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplObdCdcLoadNextBuffer(tEplObdCdcInfo* pCdcInfo_p, size_t iBufferSize)
{
tEplKernel  Ret = kEplSuccessful;
int         iReadSize;
BYTE*       pbBuffer;

    switch (pCdcInfo_p->m_Type)
    {
        case kEplObdCdcTypeFile:
        {
            if (pCdcInfo_p->m_iBufferSize < iBufferSize)
            {
                if (pCdcInfo_p->m_pbCurBuffer != NULL)
                {
                    EPL_FREE(pCdcInfo_p->m_pbCurBuffer);
                    pCdcInfo_p->m_pbCurBuffer = NULL;
                    pCdcInfo_p->m_iBufferSize = 0;
                }
                pCdcInfo_p->m_pbCurBuffer = EPL_MALLOC(iBufferSize);
                if (pCdcInfo_p->m_pbCurBuffer == NULL)
                {
                    Ret = EplEventuPostError(kEplEventSourceObdu, kEplObdOutOfMemory, 0, NULL);
                    if (Ret != kEplSuccessful)
                    {
                        goto Exit;
                    }
                    Ret = kEplReject;
                    goto Exit;
                }
                pCdcInfo_p->m_iBufferSize = iBufferSize;
            }
            pbBuffer = pCdcInfo_p->m_pbCurBuffer;
            do
            {
                // warning C4267: 'Funktion': Konvertierung von 'size_t' nach 'unsigned int', Datenverlust möglich
                iReadSize = read(pCdcInfo_p->m_Handle.m_hCdcFile, pbBuffer, iBufferSize);
                if (iReadSize <= 0)
                {
                    Ret = EplEventuPostError(kEplEventSourceObdu, kEplObdInvalidDcf, 0, NULL);
                    if (Ret != kEplSuccessful)
                    {
                        goto Exit;
                    }
                    Ret = kEplReject;
                    goto Exit;
                }
                pbBuffer += iReadSize;
                iBufferSize -= iReadSize;
                pCdcInfo_p->m_iCdcSize -= iReadSize;
            }
            while (iBufferSize > 0);
            break;
        }

        case kEplObdCdcTypeBuffer:
        {
            if (pCdcInfo_p->m_iBufferSize < iBufferSize)
            {
                Ret = EplEventuPostError(kEplEventSourceObdu, kEplObdInvalidDcf, 0, NULL);
                if (Ret != kEplSuccessful)
                {
                    goto Exit;
                }
                Ret = kEplReject;
                goto Exit;
            }
            pCdcInfo_p->m_pbCurBuffer = pCdcInfo_p->m_Handle.m_pbNextBuffer;
            pCdcInfo_p->m_Handle.m_pbNextBuffer += iBufferSize;
            pCdcInfo_p->m_iBufferSize -= iBufferSize;
            break;
        }
    }

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplObdCdcProcess
//
// Description: processes the specified CDC
//
// Parameters:  pCdcInfo_p          = pointer to CDC info structure
//
// Returns:     tEplKernel          = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplObdCdcProcess(tEplObdCdcInfo* pCdcInfo_p)
{
tEplKernel      Ret = kEplSuccessful;
DWORD           dwEntriesRemaining;
unsigned int    uiObjectIndex;
unsigned int    uiObjectSubIndex;
size_t          iCurDataSize;

    Ret = EplObdCdcLoadNextBuffer(pCdcInfo_p, sizeof (DWORD));
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }
    dwEntriesRemaining = AmiGetDwordFromLe(pCdcInfo_p->m_pbCurBuffer);

    if (dwEntriesRemaining == 0)
    {
        Ret = EplEventuPostError(kEplEventSourceObdu, kEplObdNoConfigData, 0, NULL);
        goto Exit;
    }

    for ( ; dwEntriesRemaining != 0; dwEntriesRemaining--)
    {
        Ret = EplObdCdcLoadNextBuffer(pCdcInfo_p, EPL_CDC_OFFSET_DATA);
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }

        uiObjectIndex = AmiGetWordFromLe(&pCdcInfo_p->m_pbCurBuffer[EPL_CDC_OFFSET_INDEX]);
        uiObjectSubIndex = AmiGetByteFromLe(&pCdcInfo_p->m_pbCurBuffer[EPL_CDC_OFFSET_SUBINDEX]);
        iCurDataSize = (size_t) AmiGetDwordFromLe(&pCdcInfo_p->m_pbCurBuffer[EPL_CDC_OFFSET_SIZE]);

        EPL_DBGLVL_OBD_TRACE4("%s: Reading object 0x%04X/%u with size %u from CDC\n", __func__, uiObjectIndex, uiObjectSubIndex, iCurDataSize);
        Ret = EplObdCdcLoadNextBuffer(pCdcInfo_p, iCurDataSize);
        if (Ret != kEplSuccessful)
        {
            EPL_DBGLVL_OBD_TRACE2("%s: Reading the corresponding data from CDC failed with 0x%02X\n", __func__, Ret);
            goto Exit;
        }

        Ret = EplObdWriteEntryFromLe(uiObjectIndex, uiObjectSubIndex, pCdcInfo_p->m_pbCurBuffer, (tEplObdSize) iCurDataSize);
        if (Ret != kEplSuccessful)
        {
        tEplEventObdError   ObdError;

            ObdError.m_uiIndex = uiObjectIndex;
            ObdError.m_uiSubIndex = uiObjectSubIndex;

            EPL_DBGLVL_OBD_TRACE4("%s: Writing object 0x%04X/%u to local OBD failed with 0x%02X\n", __func__, uiObjectIndex, uiObjectSubIndex, Ret);
            Ret = EplEventuPostError(kEplEventSourceObdu, Ret, sizeof (ObdError), &ObdError);
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }
        }
    }

Exit:
    return Ret;
}




#endif

// EOF

