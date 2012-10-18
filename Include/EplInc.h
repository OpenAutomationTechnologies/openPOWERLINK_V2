/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  basic include file for internal EPL stack modules

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

  2006/05/22 d.k.:   start of the implementation, version 1.00


****************************************************************************/

#ifndef _EPL_INC_H_
#define _EPL_INC_H_

// ============================================================================
// include files
// ============================================================================
#if defined(WIN32) || defined(_WIN32)

    #ifdef UNDER_RTSS
        // RTX header
        #include <windows.h>
        #include <process.h>
        #include <rtapi.h>

    #elif __BORLANDC__
        // borland C header
        #include <windows.h>
        #include <process.h>

    #elif WINCE
        #include <windows.h>

    #else
        // MSVC needs to include windows.h at first
        // the following defines are necessary for function prototypes for waitable timers
        #define _WIN32_WINDOWS 0x0401
        #define _WIN32_WINNT   0x0501

        #include <windows.h>
        #include <process.h>
    #endif

#endif

// defines for module integration
// possible other include file needed
// These constants defines modules which can be included in the Epl application.
// Use this constants for define EPL_MODULE_INTEGRATION in file EplCfg.h.
#define EPL_MODULE_OBDK        0x00000001L // OBD kernel part module
#define EPL_MODULE_PDOK        0x00000002L // PDO kernel part module
#define EPL_MODULE_NMT_MN      0x00000004L // NMT MN module
#define EPL_MODULE_SDOS        0x00000008L // SDO Server module
#define EPL_MODULE_SDOC        0x00000010L // SDO Client module
#define EPL_MODULE_SDO_ASND    0x00000020L // SDO over Asnd module
#define EPL_MODULE_SDO_UDP     0x00000040L // SDO over UDP module
#define EPL_MODULE_SDO_PDO     0x00000080L // SDO in PDO module
#define EPL_MODULE_NMT_CN      0x00000100L // NMT CN module
#define EPL_MODULE_NMTU        0x00000200L // NMT user part module
#define EPL_MODULE_NMTK        0x00000400L // NMT kernel part module
#define EPL_MODULE_DLLK        0x00000800L // DLL kernel part module
#define EPL_MODULE_DLLU        0x00001000L // DLL user part module
#define EPL_MODULE_OBDU        0x00002000L // OBD user part module
#define EPL_MODULE_CFM         0x00004000L // Configuration Manager module
#define EPL_MODULE_VETH        0x00008000L // virtual ethernet driver module
#define EPL_MODULE_PDOU        0x00010000L // PDO user part module
#define EPL_MODULE_LEDU        0x00020000L // LED user part module
#define EPL_MODULE_GW309ASCII  0x00040000L // ASCII Gateway according to CiA309 part 3

#include "EplCfg.h"     // EPL configuration file (configuration from application)

#include "global.h"     // global definitions

#include "EplDef.h"     // EPL configuration file (default configuration)
#include "EplInstDef.h" // defines macros for instance types and table
#include "Debug.h"      // debug definitions
#include "ftraceDebug.h"

#include "EplErrDef.h"  // EPL error codes for API functions

//---------------------------------------------------------------------------
// typedef
//---------------------------------------------------------------------------

// IEEE 1588 conforming net time structure
typedef struct
{
    DWORD                   m_dwSec;
    DWORD                   m_dwNanoSec;

} tEplNetTime;


// forward declaration of target specific timestamp
typedef struct _tEplTgtTimeStamp tEplTgtTimeStamp;


// Hardware parameter structure
typedef struct
{
    unsigned int    m_uiDevNumber;  // device number for selecting Ethernet controller
    const char*     m_pszDevName;   // device name (valid if non-null)

} tEplHwParam;


// user argument union
typedef union
{
    unsigned int    m_uiValue;
    void*           m_pValue;

} tEplUserArg;


#include "EplTarget.h"  // target specific functions and definitions

#include "EplAmi.h"

// -------------------------------------------------------------------------
// macros
// -------------------------------------------------------------------------

#define EPL_SPEC_VERSION                    0x20    // Ethernet POWERLINK V 2.0
#define EPL_STACK_VERSION(ver,rev,rel)      (((((DWORD)(ver)) & 0xFF)<<24)|((((DWORD)(rev))&0xFF)<<16)|(((DWORD)(rel))&0xFFFF))
#define EPL_OBJ1018_VERSION(ver,rev,rel)    ((((DWORD)(ver))<<16) |(((DWORD)(rev))&0xFFFF))
#define EPL_STRING_VERSION(ver,rev,rel)     "V" #ver "." #rev " r" #rel

#include "EplVersion.h"

// defines for EPL FeatureFlags
#define EPL_FEATURE_ISOCHR          0x00000001
#define EPL_FEATURE_SDO_UDP         0x00000002
#define EPL_FEATURE_SDO_ASND        0x00000004
#define EPL_FEATURE_SDO_PDO         0x00000008
#define EPL_FEATURE_NMT_INFO        0x00000010
#define EPL_FEATURE_NMT_EXT         0x00000020
#define EPL_FEATURE_PDO_DYN         0x00000040
#define EPL_FEATURE_NMT_UDP         0x00000080
#define EPL_FEATURE_CFM             0x00000100
#define EPL_FEATURE_DLL_MULTIPLEX   0x00000200  // CN specific
#define EPL_FEATURE_NODEID_SW       0x00000400
#define EPL_FEATURE_NMT_BASICETH    0x00000800  // MN specific
#define EPL_FEATURE_RT1             0x00001000
#define EPL_FEATURE_RT2             0x00002000
#define EPL_FEATURE_PRES_CHAINING   0x00040000


// generate EPL NMT_FeatureFlags_U32
#ifndef EPL_DEF_FEATURE_ISOCHR
    #if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_DLLK)) != 0)
        #define EPL_DEF_FEATURE_ISOCHR          (EPL_FEATURE_ISOCHR)
    #else
        #define EPL_DEF_FEATURE_ISOCHR          0
    #endif
#endif

#ifndef EPL_DEF_FEATURE_SDO_ASND
    #if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDO_ASND)) != 0)
        #define EPL_DEF_FEATURE_SDO_ASND        (EPL_FEATURE_SDO_ASND)
    #else
        #define EPL_DEF_FEATURE_SDO_ASND        0
    #endif
#endif

#ifndef EPL_DEF_FEATURE_SDO_UDP
    #if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDO_UDP)) != 0)
        #define EPL_DEF_FEATURE_SDO_UDP         (EPL_FEATURE_SDO_UDP)
    #else
        #define EPL_DEF_FEATURE_SDO_UDP         0
    #endif
#endif

#ifndef EPL_DEF_FEATURE_SDO_PDO
    #if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDO_PDO)) != 0)
        #define EPL_DEF_FEATURE_SDO_PDO         (EPL_FEATURE_SDO_PDO)
    #else
        #define EPL_DEF_FEATURE_SDO_PDO         0
    #endif
#endif

#ifndef EPL_DEF_FEATURE_PDO_DYN
    #if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_PDOU)) != 0)
        #define EPL_DEF_FEATURE_PDO_DYN         (EPL_FEATURE_PDO_DYN)
    #else
        #define EPL_DEF_FEATURE_PDO_DYN         0
    #endif
#endif

#ifndef EPL_DEF_FEATURE_CFM
    #if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_CFM)) != 0)
        #define EPL_DEF_FEATURE_CFM           (EPL_FEATURE_CFM)
    #else
        #define EPL_DEF_FEATURE_CFM           0
    #endif
#endif

#ifndef EPL_DEF_FEATURE_DLL_MULTIPLEX
    #define EPL_DEF_FEATURE_DLL_MULTIPLEX       (EPL_FEATURE_DLL_MULTIPLEX)
#endif

#ifndef EPL_DEF_FEATURE_PRES_CHAINING
    #if EPL_DLL_PRES_CHAINING_CN != FALSE
        #define EPL_DEF_FEATURE_PRES_CHAINING   (EPL_FEATURE_PRES_CHAINING)
    #else
        #define EPL_DEF_FEATURE_PRES_CHAINING   0
    #endif
#endif

#define EPL_DEF_FEATURE_FLAGS                   (EPL_DEF_FEATURE_ISOCHR \
                                                | EPL_DEF_FEATURE_SDO_ASND \
                                                | EPL_DEF_FEATURE_SDO_UDP \
                                                | EPL_DEF_FEATURE_SDO_PDO \
                                                | EPL_DEF_FEATURE_PDO_DYN \
                                                | EPL_DEF_FEATURE_CFM \
                                                | EPL_DEF_FEATURE_DLL_MULTIPLEX \
                                                | EPL_DEF_FEATURE_PRES_CHAINING)


#ifndef tabentries
#define tabentries(aVar_p)  (sizeof(aVar_p)/sizeof(*(aVar_p)))
#endif

#ifndef memberoffs
#define memberoffs(base_type, member_name)  (size_t)&(((base_type *)0)->member_name)
#endif

#ifndef min
#define min(a,b)            (((a) < (b)) ? (a) : (b))
#endif

#ifndef max
#define max(a,b)            (((a) > (b)) ? (a) : (b))
#endif

// The EPL_STATIC_ASSERT macro can be used for static assertions.
//
// Example usage: EPL_STATIC_ASSERT( sizeof(WORD) == 2 )
//
// For a detailed explanation on this topic, and the macro used here,
// see http://en.wikipedia.org/wiki/Assertion_(computing)#Static_assertions
//
#ifndef EPL_STATIC_ASSERT
#define EPL_STATIC_ASSERT(cond) switch(0){case 0:case cond:;}
#endif

//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------

// definitions for DLL export
#if ((DEV_SYSTEM == _DEV_WIN32_) || (DEV_SYSTEM == _DEV_WIN_CE_)) && defined (_WINDLL)

    #define EPLDLLEXPORT extern __declspec(dllexport)

#else

    #define EPLDLLEXPORT

#endif


// ============================================================================
// common debug macros
// ============================================================================
// for using macro DEBUG_TRACEx()
//
// Example:
//      DEBUG_TRACE (EPL_DBGLVL_OBD, "Value is %d\n" , wObjectIndex);
//
// This message only will be printed if:
//      - NDEBUG is not defined     AND !!!
//      - flag 0x00000004L is set in DEF_DEBUG_LVL (can be defined in copcfg.h)
//
// default level is defined in copdef.h

// debug-level and TRACE-macros         // standard-level   // flags for DEF_DEBUG_LVL
#define EPL_DBGLVL_EDRV                 DEBUG_LVL_01        // 0x00000001L
#define EPL_DBGLVL_EDRV_TRACE          DEBUG_LVL_01_TRACE

#define EPL_DBGLVL_DLL                  DEBUG_LVL_02        // 0x00000002L
#define EPL_DBGLVL_DLL_TRACE           DEBUG_LVL_02_TRACE

#define EPL_DBGLVL_OBD                  DEBUG_LVL_03        // 0x00000004L
#define EPL_DBGLVL_OBD_TRACE           DEBUG_LVL_03_TRACE

#define EPL_DBGLVL_NMTK                 DEBUG_LVL_04        // 0x00000008L
#define EPL_DBGLVL_NMTK_TRACE          DEBUG_LVL_04_TRACE

#define EPL_DBGLVL_NMTCN                DEBUG_LVL_05        // 0x00000010L
#define EPL_DBGLVL_NMTCN_TRACE         DEBUG_LVL_05_TRACE

#define EPL_DBGLVL_NMTU                 DEBUG_LVL_06        // 0x00000020L
#define EPL_DBGLVL_NMTU_TRACE          DEBUG_LVL_06_TRACE

#define EPL_DBGLVL_NMTMN                DEBUG_LVL_07        // 0x00000040L
#define EPL_DBGLVL_NMTMN_TRACE         DEBUG_LVL_07_TRACE

#define EPL_DBGLVL_CFM                  DEBUG_LVL_08        // 0x00000080L
#define EPL_DBGLVL_CFM_TRACE           DEBUG_LVL_08_TRACE

#define EPL_DBGLVL_TIMERU               DEBUG_LVL_09        // 0x00000100L
#define EPL_DBGLVL_TIMERU_TRACE        DEBUG_LVL_09_TRACE

#define EPL_DBGLVL_TIMERH               DEBUG_LVL_10        // 0x00000200L
#define EPL_DBGLVL_TIMERH_TRACE        DEBUG_LVL_10_TRACE

//...
#define EPL_DBGLVL_PDO                  DEBUG_LVL_24        // 0x00800000
#define EPL_DBGLVL_PDO_TRACE           DEBUG_LVL_24_TRACE

#define EPL_DBGLVL_SDO                  DEBUG_LVL_25        // 0x01000000
#define EPL_DBGLVL_SDO_TRACE           DEBUG_LVL_25_TRACE

#define EPL_DBGLVL_VETH                 DEBUG_LVL_26        // 0x02000000
#define EPL_DBGLVL_VETH_TRACE          DEBUG_LVL_26_TRACE

#define EPL_DBGLVL_EVENTK               DEBUG_LVL_27        // 0x04000000
#define EPL_DBGLVL_EVENTK_TRACE        DEBUG_LVL_27_TRACE

#define EPL_DBGLVL_EVENTU               DEBUG_LVL_28        // 0x08000000
#define EPL_DBGLVL_EVENTU_TRACE        DEBUG_LVL_28_TRACE

// SharedBuff
#define EPL_DBGLVL_SHB                  DEBUG_LVL_29        // 0x10000000
#define EPL_DBGLVL_SHB_TRACE           DEBUG_LVL_29_TRACE

#define EPL_DBGLVL_ASSERT               DEBUG_LVL_ASSERT    // 0x20000000L
#define EPL_DBGLVL_ASSERT_TRACE        DEBUG_LVL_ASSERT_TRACE

#define EPL_DBGLVL_ERROR                DEBUG_LVL_ERROR     // 0x40000000L
#define EPL_DBGLVL_ERROR_TRACE         DEBUG_LVL_ERROR_TRACE

#define EPL_DBGLVL_ALWAYS               DEBUG_LVL_ALWAYS    // 0x80000000L
#define EPL_DBGLVL_ALWAYS_TRACE        DEBUG_LVL_ALWAYS_TRACE


//---------------------------------------------------------------------------
// typedef
//---------------------------------------------------------------------------



//---------------------------------------------------------------------------
// function prototypes
//---------------------------------------------------------------------------


#endif  // #ifndef _EPL_INC_H_


