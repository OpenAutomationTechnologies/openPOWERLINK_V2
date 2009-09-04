/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  source file for Epl Userspace-Timermodule
                Implementation for use without any operating system

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

  2009/08/31 m.u.:   start of the implementation

****************************************************************************/

#include "user/EplTimeru.h"

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

typedef struct _tTimerEntry
{
    struct _tTimerEntry* m_pNext;       // this must be the first element
    DWORD           m_dwTimeoutMs;      // timeout in [ms]
    tEplTimerArg    m_TimerArg;

} tTimerEntry;

typedef struct
{
    tTimerEntry*    m_pEntries;         // pointer to array of all timer entries
    unsigned int    m_uiMaxEntries;
    tTimerEntry*    m_pListTimerFirst;
    tTimerEntry*    m_pListFreeTimerFirst;
    unsigned int    m_uiFreeEntries;
    DWORD           m_dwStartTimeMs;    // start time when the first timeout in list is based on
    unsigned int    m_uiMinFreeEntries; // minimum number of free entries

} tEplTimeruInstance;

//---------------------------------------------------------------------------
// modul globale vars
//---------------------------------------------------------------------------

tEplTimeruInstance EplTimeruInstance_g;

//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------

/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          C L A S S  <Epl Userspace-Timermodule NoOS>                    */
/*                                                                         */
/*                                                                         */
/***************************************************************************/
//
// Description: Epl Userspace-Timermodule Implementation for use without
//              any operating system
//
/***************************************************************************/

//=========================================================================//
//                                                                         //
//          P U B L I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//
// Function:    EplTimeruInit
//
// Description: function init first instance
//
// Parameters:
//
// Returns:     tEplKernel  = errorcode
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel PUBLIC EplTimeruInit()
{
tEplKernel  Ret;

    Ret = EplTimeruAddInstance();

return Ret;
}

//---------------------------------------------------------------------------
//
// Function:    EplTimeruAddInstance
//
// Description: function init additional instance
//
// Parameters:
//
// Returns:     tEplKernel  = errorcode
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel PUBLIC EplTimeruAddInstance()
{
int nIdx;
tEplKernel Ret;

    Ret = kEplSuccessful;

    // reset instance structure
    EPL_MEMSET(&EplTimeruInstance_g, 0, sizeof (EplTimeruInstance_g));

    EplTimeruInstance_g.m_uiMaxEntries = EPL_TIMERU_MAX_ENTRIES;
    EplTimeruInstance_g.m_uiFreeEntries = EPL_TIMERU_MAX_ENTRIES;
    EplTimeruInstance_g.m_uiMinFreeEntries = EPL_TIMERU_MAX_ENTRIES;

    EplTimeruInstance_g.m_pEntries = EPL_MALLOC(sizeof (tTimerEntry) * EplTimeruInstance_g.m_uiMaxEntries);
    if (EplTimeruInstance_g.m_pEntries == NULL)
    {   // allocating of queue entries failed
        Ret = kEplNoResource;
        goto Exit;
    }

    // fill free timer list
    for (nIdx = 0; nIdx < EplTimeruInstance_g.m_uiMaxEntries-1; nIdx++)
    {
        EplTimeruInstance.m_pEntries[nIdx].m_pNext = EplTimeruInstance.m_pEntries[nIdx+1];
        EplTimeruInstance.m_pEntries[nIdx].m_pfnTimer = NULL;
    }
    EplTimeruInstance.m_pEntries[EplTimeruInstance_g.m_uiMaxEntries-1].m_pNext = NULL;
    EplTimeruInstance.m_pEntries[EplTimeruInstance_g.m_uiMaxEntries-1].m_pfnTimer = NULL;

    EplTimeruInstance_g.m_pListTimerFirst = NULL;
    EplTimeruInstance_g.m_pListFreeTimerFirst = EplTimeruInstance_g.m_pEntries;

    // set start time to a value which is in any case less or equal than GetTickCount()
    // -> the only solution = 0
    EplTimeruInstance.m_dwStartTimeMs = 0;

Exit:
    return Ret;

}


//---------------------------------------------------------------------------
//
// Function:    EplTimeruDelInstance
//
// Description: function deletes instance
//
// Parameters:
//
// Returns:     tEplKernel  = errorcode
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel PUBLIC EplTimeruDelInstance()
{
tEplKernel  Ret;

    Ret = kEplSuccessful;

    EPL_FREE(EplTimeruInstance_g.m_pEntries);

    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplTimeruSetTimerMs
//
// Description: function creates a timer and returns a handle to the pointer
//
// Parameters:  pTimerHdl_p = pointer to a buffer to fill in the handle
//              ulTimeMs_p  = time for timer in ms
//              Argument_p  = argument for timer
//
// Returns:     tEplKernel  = errorcode
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel PUBLIC EplTimeruSetTimerMs(  tEplTimerHdl*   pTimerHdl_p,
                                        unsigned long   ulTimeMs_p,
                                        tEplTimerArg    Argument_p)
{
tTimerEntry*    pNewEntry;
tTimerEntry**   ppEntry;
tEplKernel      Ret;


    Ret = kEplSuccessful;

    // check pointer to handle
    if(pTimerHdl_p == NULL)
    {
        Ret = kEplTimerInvalidHandle;
        goto Exit;
    }

    // fetch first entry from free timer list
    pNewEntry = EplTimeruInstance_g.m_pListFreeTimerFirst;
    if (pNewEntry == NULL)
    {   // sorry, no free entry
        Ret = kEplTimerNoTimerCreated;
        goto Exit;
    }
    EplTimeruInstance_g.m_pListFreeTimerFirst = pNewEntry->m_pNext;

    if (EplTimeruInstance_g.m_uiMinFreeEntries > EplTimeruInstance_g.m_uiFreeEntries)
    {
        EplTimeruInstance_g.m_uiMinFreeEntries = EplTimeruInstance_g.m_uiFreeEntries;
    }

    // fill entry
    EPL_MEMCPY(&pNewEntry->m_TimerArg, &Argument_p, sizeof(tEplTimerArg));
    // calculate timeout based on start time
    pNewEntry->m_dwTimeoutMs = (EplTgtGetTickCountMs() - EplTimeruInstance_g.m_dwStartTimeMs) + ulTimeMs_p;

    *pTimerHdl_p = (tEplTimerHdl) pNewEntry;

    // insert entry into timer list
    ppEntry = &EplTimeruInstance_g.m_pListTimerFirst;
    while (*ppEntry != NULL)
    {
        if ((*ppEntry)->m_dwTimeoutMs > pNewEntry->m_dwTimeoutMs)
        {
            (*ppEntry)->m_dwTimeoutMs -= pNewEntry->m_dwTimeoutMs;
            break;
        }
        pNewEntry->m_dwTimeoutMs -= (*ppEntry)->m_dwTimeoutMs;
        ppEntry = &(*ppEntry)->m_pNext;
    }
    pNewEntry->m_pNext = *ppEntry;
    *ppEntry = pNewEntry;

Exit:
    return Ret;

}


//---------------------------------------------------------------------------
//
// Function:    EplTimeruModifyTimerMs
//
// Description: function change a timer and return a handle to the pointer
//
// Parameters:  pTimerHdl_p = pointer to a buffer to fill in the handle
//              ulTime_p    = time for timer in ms
//              Argument_p  = argument for timer
//
// Returns:     tEplKernel  = errorcode
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel PUBLIC EplTimeruModifyTimerMs(tEplTimerHdl*    pTimerHdl_p,
                                        unsigned long     ulTimeMs_p,
                                        tEplTimerArg      Argument_p)
{
tEplKernel          Ret;

    Ret = kEplSuccessful;

    // check parameter
    if(pTimerHdl_p == NULL)
    {
        Ret = kEplTimerInvalidHandle;
        goto Exit;
    }


Exit:
    return Ret;
}

//---------------------------------------------------------------------------
//
// Function:    EplTimeruDeleteTimer
//
// Description: function delte a timer
//
//
//
// Parameters:  pTimerHdl_p = pointer to a buffer to fill in the handle
//
//
// Returns:     tEplKernel  = errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel PUBLIC EplTimeruDeleteTimer(tEplTimerHdl*     pTimerHdl_p)
{
tEplKernel  Ret;

    Ret = kEplSuccessful;

    // check parameter
    if(pTimerHdl_p == NULL)
    {
        Ret = kEplTimerInvalidHandle;
        goto Exit;
    }

    // set handle invalide
    *pTimerHdl_p = 0;


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
// Function:     SLLappend
//
// Description:  Puts a new item at the end of the list.
//               2 mal verwendet (jeweils beim Entfernen eines Timers (Delete
//               or Execute) um den Eintrag in die leere Liste zurückzulegen.
//
//
// Parameters:
//
//
// Returns:
//
//
// State:
//
//---------------------------------------------------------------------------
int SLLappend( tSLList *pThis_p, void *pNewItem_p )
{

    #ifdef WIN32
        EnterCriticalSection(&pThis_p->m_CriticalSection);
    #endif

    SLL_ITER(pNewItem_p)->next = NULL;
    pThis_p->last = pThis_p->last->next = SLL_ITER(pNewItem_p);
    if( !pThis_p->head )
    {
        pThis_p->head = SLL_ITER(pNewItem_p);
    }
    pThis_p->size++;


    #ifdef WIN32
        LeaveCriticalSection(&pThis_p->m_CriticalSection);
    #endif
    
return (pThis_p->size);
}

//---------------------------------------------------------------------------
//
// Function:     SLLremove()
//
// Description:  Removes an item from the List
//               2x verwendet um Timer Element aus Liste zu entfernen.
//
// Parameters:
//
//
// Returns:
//
//
// State:
//
//---------------------------------------------------------------------------
int SLLremove( struct SLList *pThis_p, const void *pItem_p )
{
tSLLiterator *SLLiter;
int iId;


    #ifdef WIN32
        EnterCriticalSection(&pThis_p->m_CriticalSection);
    #endif

    iId = 0;
    for( SLLiter = SLL_ITER(pThis_p); SLLiter->next; SLLiter=SLLiter->next )
    {
        if( SLLiter->next==pItem_p )
        {
            SLLiter->next = SLL_ITER(pItem_p)->next;
            if( !SLLiter->next )
            {
                pThis_p->last = SLLiter;
            }
            pThis_p->size--;
            goto Exit;
        }
        iId++;
    }
    iId = (-1);

Exit:
    
    #ifdef WIN32
        LeaveCriticalSection(&pThis_p->m_CriticalSection);
    #endif

return (iId);
}


// EOF

