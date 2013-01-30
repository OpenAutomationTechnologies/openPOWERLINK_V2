/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  include file for EPL API layer for Linux (kernel and user space)

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
                KEIL uVision 2

  -------------------------------------------------------------------------

  Revision History:

  2006/10/11 d.k.:   start of the implementation, version 1.00


****************************************************************************/

#ifndef _EPL_API_LINUX_H_
#define _EPL_API_LINUX_H_


//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------

#define EPLLIN_DEV_NAME     "epl"              // used for "/dev" and "/proc" entry


//---------------------------------------------------------------------------
//  Commands for <ioctl>
//---------------------------------------------------------------------------

#define EPLLIN_CMD_INITIALIZE               _IOWR('=',  0, tEplApiInitParam)
#define EPLLIN_CMD_PI_IN                    _IOW ('=',  1, tEplApiProcessImage)
#define EPLLIN_CMD_PI_OUT                   _IOW ('=',  2, tEplApiProcessImage)
#define EPLLIN_CMD_WRITE_OBJECT             _IOWR('=',  3, tEplLinSdoObject)
#define EPLLIN_CMD_READ_OBJECT              _IOWR('=',  4, tEplLinSdoObject)
#define EPLLIN_CMD_WRITE_LOCAL_OBJECT       _IOW ('=',  5, tEplLinLocalObject)
#define EPLLIN_CMD_READ_LOCAL_OBJECT        _IOWR('=',  6, tEplLinLocalObject)
#define EPLLIN_CMD_FREE_SDO_CHANNEL         _IO  ('=',  7)  // ulArg_p ~ tEplSdoComConHdl
#define EPLLIN_CMD_NMT_COMMAND              _IO  ('=',  8)  // ulArg_p ~ tEplNmtEvent
#define EPLLIN_CMD_GET_EVENT                _IOWR('=',  9, tEplLinEvent)
#define EPLLIN_CMD_MN_TRIGGER_STATE_CHANGE  _IOW ('=', 10, tEplLinNodeCmdObject)
#define EPLLIN_CMD_PI_SETUP                 _IO  ('=', 11)
#define EPLLIN_CMD_SHUTDOWN                 _IO  ('=', 12)
#define EPLLIN_CMD_POST_USER_EVENT          _IO  ('=', 13)  // ulArg_p ~ void* = pUserArg_p
#define EPLLIN_CMD_PI_ALLOC                 _IOW ('=', 14, tEplLinProcessImageAlloc)
#define EPLLIN_CMD_PI_FREE                  _IO  ('=', 15)
#define EPLLIN_CMD_PI_EXCHANGE              _IOW ('=', 16, tEplApiProcessImageCopyJob)
#define EPLLIN_CMD_PI_LINKOBJECT            _IOWR('=', 17, tEplLinProcessImageLinkObject)
#define EPLLIN_CMD_SET_CDC_FILENAME         _IOW ('=', 18, tEplLinCdcFilename)
#define EPLLIN_CMD_SEND_ASND                _IOW ('=', 19, tEplLinSendAsnd)
#define EPLLIN_CMD_SET_ASND_FORWARD         _IOW ('=', 20, tEplLinAsndForward)

//---------------------------------------------------------------------------
// typedef
//---------------------------------------------------------------------------

typedef struct
{
    unsigned int        m_uiEventArgSize;
    tEplApiEventArg*    m_pEventArg;
    tEplApiEventType*   m_pEventType;
    tEplKernel          m_RetCbEvent;
    BYTE                *m_pUserBuf;
    size_t              m_UserBufSize;
} tEplLinEvent;

typedef struct
{
    tEplSdoComConHdl    m_SdoComConHdl;
    BOOL                m_fValidSdoComConHdl;
    unsigned int        m_uiNodeId;
    unsigned int        m_uiIndex;
    unsigned int        m_uiSubindex;
    void*               m_le_pData;
    unsigned int        m_uiSize;
    tEplSdoType         m_SdoType;
    void*               m_pUserArg;

} tEplLinSdoObject;

typedef struct
{
    unsigned int        m_uiIndex;
    unsigned int        m_uiSubindex;
    void*               m_pData;
    unsigned int        m_uiSize;

} tEplLinLocalObject;

typedef struct
{
    unsigned int        m_uiNodeId;
    tEplNmtNodeCommand  m_NodeCommand;

} tEplLinNodeCmdObject;

typedef struct
{
    unsigned int        m_uiSizeProcessImageIn;
    unsigned int        m_uiSizeProcessImageOut;
    unsigned int        m_uiQueueEntriesLo;
    unsigned int        m_uiQueueEntriesHi;

} tEplLinProcessImageAlloc;

typedef struct
{
    unsigned int        m_uiObjIndex;
    unsigned int        m_uiFirstSubindex;
    unsigned int        m_uiOffsetPI;
    BOOL                m_fOutputPI;
    tEplObdSize         m_EntrySize;
    unsigned int*       m_puiVarEntries;

} tEplLinProcessImageLinkObject;

typedef struct
{
    size_t              m_uiFilenameSize;
    char*               m_pszCdcFilename;

} tEplLinCdcFilename;

typedef struct
{
    BYTE                m_bDstNodeId;
    tEplAsndFrame       *m_pAsndFrame;
    size_t              m_uiAsndSize;
}
tEplLinSendAsnd;

typedef struct
{
    BYTE                m_bServiceId;
    tEplApiAsndFilter   m_FilterType;
}
tEplLinAsndForward;

//---------------------------------------------------------------------------
// function prototypes
//---------------------------------------------------------------------------



#endif  // #ifndef _EPL_API_LINUX_H_


