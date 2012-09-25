/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  Debug interface

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
                    ...

  -------------------------------------------------------------------------

  Revision History:

****************************************************************************/

#ifndef _DEBUG_H_
#define _DEBUG_H_

#include "global.h"


/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          G L O B A L   D E F I N I T I O N S                            */
/*                                                                         */
/*                                                                         */
/***************************************************************************/

//---------------------------------------------------------------------------
// global const defines
//---------------------------------------------------------------------------

// These definitions are important for level-debug traces.
// A macro DEBUG_GLB_LVL defines the current debug-level using following bis.
// If the corresponding bit is set then trace message will be printed out
// (only if NDEBUG is not defined). The upper debug-levels are reserved for
// the debug-levels ALWAYS, ERROR and ASSERT.
#define DEBUG_LVL_01                    0x00000001
#define DEBUG_LVL_02                    0x00000002
#define DEBUG_LVL_03                    0x00000004
#define DEBUG_LVL_04                    0x00000008
#define DEBUG_LVL_05                    0x00000010
#define DEBUG_LVL_06                    0x00000020
#define DEBUG_LVL_07                    0x00000040
#define DEBUG_LVL_08                    0x00000080
#define DEBUG_LVL_09                    0x00000100
#define DEBUG_LVL_10                    0x00000200
#define DEBUG_LVL_11                    0x00000400
#define DEBUG_LVL_12                    0x00000800
#define DEBUG_LVL_13                    0x00001000
#define DEBUG_LVL_14                    0x00002000
#define DEBUG_LVL_15                    0x00004000
#define DEBUG_LVL_16                    0x00008000
#define DEBUG_LVL_17                    0x00010000
#define DEBUG_LVL_18                    0x00020000
#define DEBUG_LVL_19                    0x00040000
#define DEBUG_LVL_20                    0x00080000
#define DEBUG_LVL_21                    0x00100000
#define DEBUG_LVL_22                    0x00200000
#define DEBUG_LVL_23                    0x00400000
#define DEBUG_LVL_24                    0x00800000
#define DEBUG_LVL_25                    0x01000000
#define DEBUG_LVL_26                    0x02000000
#define DEBUG_LVL_27                    0x04000000
#define DEBUG_LVL_28                    0x08000000
#define DEBUG_LVL_29                    0x10000000
#define DEBUG_LVL_ASSERT                0x20000000
#define DEBUG_LVL_ERROR                 0x40000000
#define DEBUG_LVL_ALWAYS                0x80000000


//---------------------------------------------------------------------------
// global types
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
// global vars
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
// global function prototypes
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// global macros
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// this macro defines a version string
#define MAKE_VERSION_STRING(product,appname,verstr,author) \
    "§prd§:" product ",§app§:" appname ",§ver§:" verstr ",§dat§:" __DATE__ ",§aut§:" author


//---------------------------------------------------------------------------
// this macro defines a build info string (e.g. for using in printf())
#define DEBUG_MAKE_BUILD_INFO(prefix,product,prodid,descr,verstr,author) "\n" \
    prefix "***************************************************\n" \
    prefix "Project:   " product ", " prodid                  "\n" \
    prefix "Descript.: " descr                                "\n" \
    prefix "Author:    " author                               "\n" \
    prefix "Date:      " __DATE__                             "\n" \
    prefix "Version:   " verstr                               "\n" \
    prefix "***************************************************\n\n"


//---------------------------------------------------------------------------
// The default debug-level is: ERROR and ALWAYS.
// You can define an other debug-level in project settings.
#ifndef DEF_DEBUG_LVL
    #define DEF_DEBUG_LVL                   (DEBUG_LVL_ALWAYS | DEBUG_LVL_ERROR)
#endif
#ifndef DEBUG_GLB_LVL
    #define DEBUG_GLB_LVL                 (DEF_DEBUG_LVL)
#endif


//---------------------------------------------------------------------------
#if (DEV_SYSTEM == _DEV_WIN32_) && defined (TRACE_MSG)

    // For WIN32 the macro DEBUG_TRACE can be defined as function call TraceLvl()
    // or as macro TRACE().
    //
    // Here the parameter 'lvl' can be used with more than one
    // debug-level (using OR).
    //
    // Example: DEBUG_TRACE(DEBUG_LVL_30 | DEBUG_LVL_02, "Hello %d", bCount);

    #define DEBUG_TRACE(lvl,...)             TraceLvl((lvl),__VA_ARGS__)
    #define DEBUG_GLB_LVL                     dwDebugLevel_g

#else

    // At microcontrollers we do reduce the memory usage by deleting DEBUG_TRACE-lines
    // (compiler does delete the lines).
    //
    // Here the parameter 'lvl' can only be used with one debug-level.
    //
    // Example: DEBUG_TRACE(DEBUG_LVL_ERROR, "error code %d", dwRet);

    #if (DEBUG_GLB_LVL & DEBUG_LVL_ALWAYS)
        #define DEBUG_LVL_ALWAYS_TRACE(...)                TRACE(__VA_ARGS__)
    #else
        #define DEBUG_LVL_ALWAYS_TRACE(...)
    #endif

    #if (DEBUG_GLB_LVL & DEBUG_LVL_ERROR)
        #define DEBUG_LVL_ERROR_TRACE(...)                 TRACE(__VA_ARGS__)
    #else
        #define DEBUG_LVL_ERROR_TRACE(...)
    #endif

    #if (DEBUG_GLB_LVL & DEBUG_LVL_ASSERT)
        #define DEBUG_LVL_ASSERT_TRACE(...)                TRACE(__VA_ARGS__)
    #else
        #define DEBUG_LVL_ASSERT_TRACE(...)
    #endif

    #if (DEBUG_GLB_LVL & DEBUG_LVL_29)
        #define DEBUG_LVL_29_TRACE(...)                    TRACE(__VA_ARGS__)
    #else
        #define DEBUG_LVL_29_TRACE(...)
    #endif

    #if (DEBUG_GLB_LVL & DEBUG_LVL_28)
        #define DEBUG_LVL_28_TRACE(...)                    TRACE(__VA_ARGS__)
    #else
        #define DEBUG_LVL_28_TRACE(...)
    #endif

    #if (DEBUG_GLB_LVL & DEBUG_LVL_27)
        #define DEBUG_LVL_27_TRACE(...)                    TRACE(__VA_ARGS__)
    #else
        #define DEBUG_LVL_27_TRACE(...)
    #endif

    #if (DEBUG_GLB_LVL & DEBUG_LVL_26)
        #define DEBUG_LVL_26_TRACE(...)                    TRACE(__VA_ARGS__)
    #else
        #define DEBUG_LVL_26_TRACE(...)
    #endif

    #if (DEBUG_GLB_LVL & DEBUG_LVL_25)
        #define DEBUG_LVL_25_TRACE(...)                    TRACE(__VA_ARGS__)
    #else
        #define DEBUG_LVL_25_TRACE(...)
    #endif

    #if (DEBUG_GLB_LVL & DEBUG_LVL_24)
        #define DEBUG_LVL_24_TRACE(...)                    TRACE(__VA_ARGS__)
    #else
        #define DEBUG_LVL_24_TRACE(...)
    #endif

    #if (DEBUG_GLB_LVL & DEBUG_LVL_23)
        #define DEBUG_LVL_23_TRACE(...)                    TRACE(__VA_ARGS__)
    #else
        #define DEBUG_LVL_23_TRACE(...)
    #endif

    #if (DEBUG_GLB_LVL & DEBUG_LVL_22)
        #define DEBUG_LVL_22_TRACE(...)                    TRACE(__VA_ARGS__)
    #else
        #define DEBUG_LVL_22_TRACE(...)
    #endif

    #if (DEBUG_GLB_LVL & DEBUG_LVL_21)
        #define DEBUG_LVL_21_TRACE(...)                    TRACE(__VA_ARGS__)
    #else
        #define DEBUG_LVL_21_TRACE(...)
    #endif

    #if (DEBUG_GLB_LVL & DEBUG_LVL_20)
        #define DEBUG_LVL_20_TRACE(...)                    TRACE(__VA_ARGS__)
    #else
        #define DEBUG_LVL_20_TRACE(...)
    #endif

    #if (DEBUG_GLB_LVL & DEBUG_LVL_19)
        #define DEBUG_LVL_19_TRACE(...)                    TRACE(__VA_ARGS__)
    #else
        #define DEBUG_LVL_19_TRACE(...)
    #endif

    #if (DEBUG_GLB_LVL & DEBUG_LVL_18)
        #define DEBUG_LVL_18_TRACE(...)                    TRACE(__VA_ARGS__)
    #else
        #define DEBUG_LVL_18_TRACE(...)
    #endif

    #if (DEBUG_GLB_LVL & DEBUG_LVL_17)
        #define DEBUG_LVL_17_TRACE(...)                    TRACE(__VA_ARGS__)
    #else
        #define DEBUG_LVL_17_TRACE(...)
    #endif

    #if (DEBUG_GLB_LVL & DEBUG_LVL_16)
        #define DEBUG_LVL_16_TRACE(...)                    TRACE(__VA_ARGS__)
    #else
        #define DEBUG_LVL_16_TRACE(...)
    #endif

    #if (DEBUG_GLB_LVL & DEBUG_LVL_15)
        #define DEBUG_LVL_15_TRACE(...)                    TRACE(__VA_ARGS__)
    #else
        #define DEBUG_LVL_15_TRACE(...)
    #endif

    #if (DEBUG_GLB_LVL & DEBUG_LVL_14)
        #define DEBUG_LVL_14_TRACE(...)                    TRACE(__VA_ARGS__)
    #else
        #define DEBUG_LVL_14_TRACE(...)
    #endif

    #if (DEBUG_GLB_LVL & DEBUG_LVL_13)
        #define DEBUG_LVL_13_TRACE(...)                    TRACE(__VA_ARGS__)
    #else
        #define DEBUG_LVL_13_TRACE(...)
    #endif

    #if (DEBUG_GLB_LVL & DEBUG_LVL_12)
        #define DEBUG_LVL_12_TRACE(...)                    TRACE(__VA_ARGS__)
    #else
        #define DEBUG_LVL_12_TRACE(...)
    #endif

    #if (DEBUG_GLB_LVL & DEBUG_LVL_11)
        #define DEBUG_LVL_11_TRACE(...)                    TRACE(__VA_ARGS__)
    #else
        #define DEBUG_LVL_11_TRACE(...)
    #endif

    #if (DEBUG_GLB_LVL & DEBUG_LVL_10)
        #define DEBUG_LVL_10_TRACE(...)                    TRACE(__VA_ARGS__)
    #else
        #define DEBUG_LVL_10_TRACE(...)
    #endif

    #if (DEBUG_GLB_LVL & DEBUG_LVL_09)
        #define DEBUG_LVL_09_TRACE(...)                    TRACE(__VA_ARGS__)
    #else
        #define DEBUG_LVL_09_TRACE(...)
    #endif

    #if (DEBUG_GLB_LVL & DEBUG_LVL_08)
        #define DEBUG_LVL_08_TRACE(...)                    TRACE(__VA_ARGS__)
    #else
        #define DEBUG_LVL_08_TRACE(...)
    #endif

    #if (DEBUG_GLB_LVL & DEBUG_LVL_07)
        #define DEBUG_LVL_07_TRACE(...)                    TRACE(__VA_ARGS__)
    #else
        #define DEBUG_LVL_07_TRACE(...)
    #endif

    #if (DEBUG_GLB_LVL & DEBUG_LVL_06)
        #define DEBUG_LVL_06_TRACE(...)                    TRACE(__VA_ARGS__)
    #else
        #define DEBUG_LVL_06_TRACE(...)
    #endif

    #if (DEBUG_GLB_LVL & DEBUG_LVL_05)
        #define DEBUG_LVL_05_TRACE(...)                    TRACE(__VA_ARGS__)
    #else
        #define DEBUG_LVL_05_TRACE(...)
    #endif

    #if (DEBUG_GLB_LVL & DEBUG_LVL_04)
        #define DEBUG_LVL_04_TRACE(...)                    TRACE(__VA_ARGS__)
    #else
        #define DEBUG_LVL_04_TRACE(...)
    #endif

    #if (DEBUG_GLB_LVL & DEBUG_LVL_03)
        #define DEBUG_LVL_03_TRACE(...)                    TRACE(__VA_ARGS__)
    #else
        #define DEBUG_LVL_03_TRACE(...)
    #endif

    #if (DEBUG_GLB_LVL & DEBUG_LVL_02)
        #define DEBUG_LVL_02_TRACE(...)                    TRACE(__VA_ARGS__)
    #else
        #define DEBUG_LVL_02_TRACE(...)
    #endif

    #if (DEBUG_GLB_LVL & DEBUG_LVL_01)
        #define DEBUG_LVL_01_TRACE(...)                    TRACE(__VA_ARGS__)
    #else
        #define DEBUG_LVL_01_TRACE(...)
    #endif

    #define DEBUG_TRACE(lvl,...)                           lvl##_TRACE(__VA_ARGS__)

#endif


//---------------------------------------------------------------------------
// The macro DEBUG_DUMP_DATA() can be used with the same debug-levels to dump
// out data bytes. Function DumpData() has to be included.
// NOTE: DUMP_DATA has to be defined in project settings.
#if (!defined (NDEBUG) && defined (DUMP_DATA)) || (DEV_SYSTEM == _DEV_WIN32_)

    #ifdef __cplusplus
    extern "C"
    {
    #endif

        void DumpData (char* szStr_p, BYTE MEM* pbData_p, WORD wSize_p);

    #ifdef __cplusplus
    } // von extern "C"
    #endif

    #define DEBUG_DUMP_DATA(lvl,str,ptr,siz)    if ((DEBUG_GLB_LVL & (lvl))==(lvl)) \
                                                    DumpData (str, (BYTE MEM*) (ptr), (WORD) (siz));

#else

    #define DEBUG_DUMP_DATA(lvl,str,ptr,siz)

#endif


//---------------------------------------------------------------------------
// The macro DEBUG_ASSERT() can be used to print out an error string if the
// parametered expresion does not result TRUE.
// NOTE: If DEBUG_KEEP_ASSERT is defined, then DEBUG_ASSERT-line will not be
//       deleted from compiler (in release version too).
#if !defined (NDEBUG) || defined (DEBUG_KEEP_ASSERT)

    #if (DEV_SYSTEM == _DEV_WIN32_)

        // For WIN32 process will be killed after closing message box.

        #define DEBUG_ASSERT(expr,str)         if (!(expr ) && ((DEBUG_GLB_LVL & DEBUG_LVL_ASSERT)!=0)) { \
                                                    MessageBox (NULL, \
                                                        "Assertion failed: line " __LINE__ " file " __FILE__ \
                                                        "\n    -> " str "\n"); \
                                                    ExitProcess (-1); }

        #define DEBUG_ASSERT1(expr,str,p1)      if (!(expr ) && ((DEBUG_GLB_LVL & DEBUG_LVL_ASSERT)!=0)) { \
                                                    MessageBox (NULL, \
                                                        "Assertion failed: line " __LINE__ " file " __FILE__ \
                                                        "\n    -> " str "\n"); \
                                                    ExitProcess (-1); }

    #else

        // For microcontrollers process will be stopped using endless loop.

        #define DEBUG_ASSERT0(expr,str)         if (!(expr )) { \
                                                    DEBUG_LVL_ASSERT_TRACE ( \
                                                        "Assertion failed: line %d file '%s'\n" \
                                                        "    -> '%s'\n", __LINE__, __FILE__, str); \
                                                    while (1); }

        #define DEBUG_ASSERT1(expr,str,p1)      if (!(expr )) { \
                                                    DEBUG_LVL_ASSERT_TRACE ( \
                                                        "Assertion failed: line %d file '%s'\n" \
                                                        "    -> '%s'\n" \
                                                        "    -> 0x%08lX\n", __LINE__, __FILE__, str, (DWORD) p1); \
                                                    while (1); }

    #endif

#else

    #define DEBUG_ASSERT0(expr,str)
    #define DEBUG_ASSERT1(expr,str,p1)

#endif


//---------------------------------------------------------------------------
// The macro DEBUG_ONLY() implements code, if NDEBUG is not defined.
#if !defined (DEBUG_ONLY)
    #if !defined (NDEBUG)

        #define DEBUG_ONLY(expr)    expr

    #else

        #define DEBUG_ONLY(expr)

    #endif
#endif


#endif // _DEBUG_H_
