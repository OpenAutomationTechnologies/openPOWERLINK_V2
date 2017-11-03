/**
********************************************************************************
\file   common/debug.h

\brief  Definitions for debugging

The file contains definitions used the debugging.
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
#ifndef _INC_common_debug_H_
#define _INC_common_debug_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>
#include <trace/trace.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// These definitions are important for level-debug traces.
// A macro DEBUG_GLB_LVL defines the current debug-level using following bis.
// If the corresponding bit is set then trace message will be printed out
// (only if NDEBUG is not defined). The upper debug-levels are reserved for
// the debug-levels ALWAYS, ERROR and ASSERT.
#define DEBUG_LVL_EDRV                  0x00000001
#define DEBUG_LVL_DLL                   0x00000002
#define DEBUG_LVL_OBD                   0x00000004
#define DEBUG_LVL_NMTK                  0x00000008
#define DEBUG_LVL_NMTCN                 0x00000010
#define DEBUG_LVL_NMTU                  0x00000020
#define DEBUG_LVL_NMTMN                 0x00000040
#define DEBUG_LVL_CFM                   0x00000080
#define DEBUG_LVL_TIMERU                0x00000100
#define DEBUG_LVL_TIMERH                0x00000200
#define DEBUG_LVL_CTRL                  0x00000400
#define DEBUG_LVL_DRVINTF               0x00000800

#define DEBUG_LVL_PDO                   0x00800000
#define DEBUG_LVL_SDO                   0x01000000
#define DEBUG_LVL_VETH                  0x02000000
#define DEBUG_LVL_EVENTK                0x04000000
#define DEBUG_LVL_EVENTU                0x08000000

#define DEBUG_LVL_ASSERT                0x20000000
#define DEBUG_LVL_ERROR                 0x40000000
#define DEBUG_LVL_ALWAYS                0x80000000


//---------------------------------------------------------------------------
// The default debug-level is: ERROR and ALWAYS.
// You can define an other debug-level in project settings.
#ifndef DEF_DEBUG_LVL
#define DEF_DEBUG_LVL                   (DEBUG_LVL_ALWAYS | DEBUG_LVL_ERROR)
#endif

#ifndef DEBUG_GLB_LVL
#define DEBUG_GLB_LVL                   (DEF_DEBUG_LVL)
#endif

//---------------------------------------------------------------------------

// At micro controllers we do reduce the memory usage by deleting DEBUG_TRACE-lines
// (compiler does delete the lines).
// Here the parameter 'lvl' can only be used with one debug-level.
// Example: DEBUG_TRACE(DEBUG_LVL_ERROR, "error code %d", dwRet);

#if (DEBUG_GLB_LVL & DEBUG_LVL_ALWAYS)
#define DEBUG_LVL_ALWAYS_TRACE(...)     TRACE(__VA_ARGS__)
#else
#define DEBUG_LVL_ALWAYS_TRACE(...)
#endif

#if (DEBUG_GLB_LVL & DEBUG_LVL_ERROR)
#define DEBUG_LVL_ERROR_TRACE(...)      TRACE(__VA_ARGS__)
#else
#define DEBUG_LVL_ERROR_TRACE(...)
#endif

#if (DEBUG_GLB_LVL & DEBUG_LVL_ASSERT)
#define DEBUG_LVL_ASSERT_TRACE(...)     TRACE(__VA_ARGS__)
#else
#define DEBUG_LVL_ASSERT_TRACE(...)
#endif

#if (DEBUG_GLB_LVL & DEBUG_LVL_EVENTU)
#define DEBUG_LVL_EVENTU_TRACE(...)     TRACE(__VA_ARGS__)
#else
#define DEBUG_LVL_EVENTU_TRACE(...)
#endif

#if (DEBUG_GLB_LVL & DEBUG_LVL_EVENTK)
#define DEBUG_LVL_EVENTK_TRACE(...)     TRACE(__VA_ARGS__)
#else
#define DEBUG_LVL_EVENTK_TRACE(...)
#endif

#if (DEBUG_GLB_LVL & DEBUG_LVL_VETH)
#define DEBUG_LVL_VETH_TRACE(...)       TRACE(__VA_ARGS__)
#else
#define DEBUG_LVL_VETH_TRACE(...)
#endif

#if (DEBUG_GLB_LVL & DEBUG_LVL_SDO)
#define DEBUG_LVL_SDO_TRACE(...)        TRACE(__VA_ARGS__)
#else
#define DEBUG_LVL_SDO_TRACE(...)
#endif

#if (DEBUG_GLB_LVL & DEBUG_LVL_PDO)
#define DEBUG_LVL_PDO_TRACE(...)        TRACE(__VA_ARGS__)
#else
#define DEBUG_LVL_PDO_TRACE(...)
#endif

#if (DEBUG_GLB_LVL & DEBUG_LVL_TIMERH)
#define DEBUG_LVL_TIMERH_TRACE(...)     TRACE(__VA_ARGS__)
#else
#define DEBUG_LVL_TIMERH_TRACE(...)
#endif

#if (DEBUG_GLB_LVL & DEBUG_LVL_CTRL)
#define DEBUG_LVL_CTRL_TRACE(...)       TRACE(__VA_ARGS__)
#else
#define DEBUG_LVL_CTRL_TRACE(...)
#endif

#if (DEBUG_GLB_LVL & DEBUG_LVL_DRVINTF)
#define DEBUG_LVL_DRVINTF_TRACE(...)    TRACE(__VA_ARGS__)
#else
#define DEBUG_LVL_DRVINTF_TRACE(...)
#endif

#if (DEBUG_GLB_LVL & DEBUG_LVL_TIMERU)
#define DEBUG_LVL_TIMERU_TRACE(...)     TRACE(__VA_ARGS__)
#else
#define DEBUG_LVL_TIMERU_TRACE(...)
#endif

#if (DEBUG_GLB_LVL & DEBUG_LVL_CFM)
#define DEBUG_LVL_CFM_TRACE(...)        TRACE(__VA_ARGS__)
#else
#define DEBUG_LVL_CFM_TRACE(...)
#endif

#if (DEBUG_GLB_LVL & DEBUG_LVL_NMTMN)
#define DEBUG_LVL_NMTMN_TRACE(...)      TRACE(__VA_ARGS__)
#else
#define DEBUG_LVL_NMTMN_TRACE(...)
#endif

#if (DEBUG_GLB_LVL & DEBUG_LVL_NMTU)
#define DEBUG_LVL_NMTU_TRACE(...)       TRACE(__VA_ARGS__)
#else
#define DEBUG_LVL_NMTU_TRACE(...)
#endif

#if (DEBUG_GLB_LVL & DEBUG_LVL_NMTCN)
#define DEBUG_LVL_NMTCN_TRACE(...)      TRACE(__VA_ARGS__)
#else
#define DEBUG_LVL_NMTCN_TRACE(...)
#endif

#if (DEBUG_GLB_LVL & DEBUG_LVL_NMTK)
#define DEBUG_LVL_NMTK_TRACE(...)       TRACE(__VA_ARGS__)
#else
#define DEBUG_LVL_NMTK_TRACE(...)
#endif

#if (DEBUG_GLB_LVL & DEBUG_LVL_OBD)
#define DEBUG_LVL_OBD_TRACE(...)        TRACE(__VA_ARGS__)
#else
#define DEBUG_LVL_OBD_TRACE(...)
#endif

#if (DEBUG_GLB_LVL & DEBUG_LVL_DLL)
#define DEBUG_LVL_DLL_TRACE(...)        TRACE(__VA_ARGS__)
#else
#define DEBUG_LVL_DLL_TRACE(...)
#endif

#if (DEBUG_GLB_LVL & DEBUG_LVL_EDRV)
#define DEBUG_LVL_EDRV_TRACE(...)       TRACE(__VA_ARGS__)
#else
#define DEBUG_LVL_EDRV_TRACE(...)
#endif

//------------------------------------------------------------------------------
// Definition of ASSERT
//
// assert.h should be available on all platforms - at least in user-space.
// For Linux kernel, WARN_ON() is called for failed assertions.
// For Windows kernel, ASSERT() is already defined in Ntddk.h
//------------------------------------------------------------------------------
#ifndef ASSERT
// In Linux kernel, use WARN_ON() macro for failed assertions
#if ((TARGET_SYSTEM == _LINUX_) && defined(__KERNEL__))
#if !defined(NDEBUG)
#define ASSERT(p)   WARN_ON(!(p))
#else
#define ASSERT(p)
#endif /* NDEBUG */
#else
#include <assert.h>
#define ASSERT(p)   assert(p)
#endif
#endif /* ASSERT */

#endif /* _INC_common_debug_H_ */
