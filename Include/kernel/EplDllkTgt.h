/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  target specific include file for kernel part DLL module

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

  2009/06/25 d.k.:   start of the implementation, version 1.00


****************************************************************************/

#ifndef _EPL_DLLK_TGT_H_
#define _EPL_DLLK_TGT_H_


//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------


#if (TARGET_SYSTEM == _LINUX_) \
    && defined(__KERNEL__) \
    && !defined(CONFIG_COLDFIRE)
// Linux kernel except MCF5484 needs spinlock
// for DLL's callback functions

#include <linux/spinlock.h>

#define TGT_DLLK_DECLARE_CRITICAL_SECTION \
        DEFINE_SPINLOCK(TgtDllkCriticalSection_l);

#define TGT_DLLK_DECLARE_FLAGS \
        unsigned long ulTgtDllkFlags;

#define TGT_DLLK_ENTER_CRITICAL_SECTION() \
    spin_lock_irqsave(&TgtDllkCriticalSection_l, ulTgtDllkFlags)

#define TGT_DLLK_LEAVE_CRITICAL_SECTION() \
    spin_unlock_irqrestore(&TgtDllkCriticalSection_l, ulTgtDllkFlags)



#else   // all other targets do not need the critical section within DLL

#define TGT_DLLK_DECLARE_CRITICAL_SECTION
#define TGT_DLLK_DECLARE_FLAGS

#define TGT_DLLK_ENTER_CRITICAL_SECTION()

#define TGT_DLLK_LEAVE_CRITICAL_SECTION()


#endif

#endif  // #ifndef _EPL_DLLK_TGT_H_


