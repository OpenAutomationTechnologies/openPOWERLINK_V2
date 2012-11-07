/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  (c) Bernecker + Rainer Ges.m.b.H.,  B&R Strasse 1, 5142 Eggelsberg, Austria
      www.br-automation.com

  Project:      openPOWERLINK

  Description:  global include file

  License:

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its
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

****************************************************************************/

#ifndef _GLOBAL_H_
#define _GLOBAL_H_

//---------------------------------------------------------------------------
//  elements of defines for development system
//---------------------------------------------------------------------------

// these defines are necessary to check some of characteristics of the development system

// endianness
#define _DEV_BIGEND_            0x80000000L     // big endian (motorola format)

// alignment
#define _DEV_ALIGNMENT_4_       0x00400000L     // the CPU needs alignment of 4 bytes

// add support
#define _DEV_ONLY_INT_MAIN_     0x00004000L     // the compiler needs "int main(int)" instead of "void main(void)"
#define _DEV_COMMA_EXT_         0x00002000L     // support of last comma in struct predefinition
#define _DEV_64BIT_SUPPORT_     0x00001000L     // support of 64 bit operations

// count of bits:
#define _DEV_BIT64_             0x00000400L     // 64 bit
#define _DEV_BIT32_             0x00000300L     // 32 bit

// compilers
#define _DEV_GNUC_MICROBLAZE_   0x00000020L     // Xilinx Microblaze GCC
#define _DEV_GNUC_NIOS2_        0x0000001FL     // Altera Nios II GCC
#define _DEV_GNUC_X86_          0x00000017L     // GNU for I386
#define _DEV_GNUC_CF_           0x00000014L     // GNU for Coldfire
#define _DEV_MSEVC_             0x00000012L     // Microsoft embedded Visual C/C++#define _DEV_GNUC_ARM7_         0x0000000EL     // GNU Compiler gcc for ARM7
#define _DEV_LINUX_GCC_         0x0000000AL     // Linux GNU Compiler gcc
#define _DEV_MSVC32_            0x00000000L     // Microsoft Visual C/C++

// these defines can be used to mask previous elements
#define _DEV_MASK_COMPILER      0x000000FFL
#define _DEV_MASK_BITCOUNT      0x00000F00L
#define _DEV_MASK_ADDSUPPORT    0x0000F000L
#define _DEV_MASK_ALIGNMENT     0x00F00000L

//---------------------------------------------------------------------------
//  defines for development system (DEV_SYSTEM) including previous elements
//---------------------------------------------------------------------------
#define _DEV_WIN32_             (_DEV_BIT32_ | _DEV_MSVC32_                   | _DEV_64BIT_SUPPORT_ | _DEV_COMMA_EXT_)
#define _DEV_WIN_CE_            (_DEV_BIT32_ | _DEV_MSEVC_                    | _DEV_64BIT_SUPPORT_ | _DEV_COMMA_EXT_)
#define _DEV_LINUX_             (_DEV_BIT32_ | _DEV_LINUX_GCC_                | _DEV_64BIT_SUPPORT_ | _DEV_COMMA_EXT_)
#define _DEV_GNU_CF548X_        (_DEV_BIT32_ | _DEV_GNUC_CF_   | _DEV_BIGEND_ | _DEV_64BIT_SUPPORT_ | _DEV_COMMA_EXT_)
#define _DEV_GNU_I386_          (_DEV_BIT32_ | _DEV_GNUC_X86_                 | _DEV_64BIT_SUPPORT_ | _DEV_COMMA_EXT_ | _DEV_ONLY_INT_MAIN_)
#define _DEV_NIOS2_             (_DEV_BIT32_ | _DEV_GNUC_NIOS2_               | _DEV_64BIT_SUPPORT_ | _DEV_COMMA_EXT_ | _DEV_ONLY_INT_MAIN_ | _DEV_ALIGNMENT_4_ )
#define _DEV_VXWORKS_           (_DEV_BIT32_ | _DEV_LINUX_GCC_                | _DEV_64BIT_SUPPORT_ | _DEV_COMMA_EXT_)
#define _DEV_MICROBLAZE_BIG_    (_DEV_BIT32_ | _DEV_GNUC_MICROBLAZE   | _DEV_BIGEND_ | _DEV_64BIT_SUPPORT_ | _DEV_COMMA_EXT_ | _DEV_ONLY_INT_MAIN_ | _DEV_ALIGNMENT_4_ )
#define _DEV_MICROBLAZE_LITTLE_ (_DEV_BIT32_ | _DEV_GNUC_MICROBLAZE   | _DEV_64BIT_SUPPORT_ | _DEV_COMMA_EXT_ | _DEV_ONLY_INT_MAIN_ | _DEV_ALIGNMENT_4_ )

//---------------------------------------------------------------------------
//  useful macros
//---------------------------------------------------------------------------
#define CHECK_MEMORY_ALIGNMENT()     ((DEV_SYSTEM & _DEV_MASK_ALIGNMENT) != 0)
#define CHECK_IF_BIG_ENDIAN()       ((DEV_SYSTEM & _DEV_BIGEND_) != 0)


//---------------------------------------------------------------------------
//  defines for target system (TARGET_SYSTEM)
//---------------------------------------------------------------------------

#define _WIN32_             32
#define _WINCE_            (32 + 0x20000)
#define _NO_OS_              0
#define _LINUX_              1
#define _PXROS_              2
#define _ECOSPRO_            3
#define _VXWORKS_            4


//---------------------------------------------------------------------------
//  definitions for function inlining
//---------------------------------------------------------------------------

#define INLINE_FUNCTION             // empty define
#undef  INLINE_ENABLED              // disable actual inlining of functions
#undef  INLINE_FUNCTION_DEF         // disable inlining for all compilers per default

//---------------------------------------------------------------------------
//  check for used compiler
//---------------------------------------------------------------------------
#if defined (__GNUC__)

    // GNU C compiler supports function inlining
    #define INLINE_FUNCTION_DEF extern inline

    // to actually enable inlining just include the following three lines
    // #undef INLINE_FUNCTION
    // #define INLINE_FUNCTION     INLINE_FUNCTION_DEF
    // #define INLINE_ENABLED      TRUE

    // ------------------ definition target system --------------------------
    #if defined (LINUX) || defined (__linux__)
        #define TARGET_SYSTEM   _LINUX_     // Linux definition
        #define DEV_SYSTEM      _DEV_LINUX_

    #elif defined (GNU_CF548X)
        #define TARGET_SYSTEM   _NO_OS_
        #define DEV_SYSTEM      _DEV_GNU_CF548X_

    #elif defined (__NIOS2__)
        #define TARGET_SYSTEM   _NO_OS_
        #define DEV_SYSTEM      _DEV_NIOS2_

    #elif defined (__MICROBLAZE__)
        #define TARGET_SYSTEM   _NO_OS_
        // Microblaze could be either big or little endian
        #if (__BIG_ENDIAN__ == 1)
            #define DEV_SYSTEM      _DEV_MICROBLAZE_BIG_
        #else
            #define DEV_SYSTEM      _DEV_MICROBLAZE_LITTLE_
        #endif

    #elif defined (__VXWORKS__)
        #define TARGET_SYSTEM   _VXWORKS_
        #define DEV_SYSTEM      _DEV_VXWORKS_

    #else
        #error 'ERROR: TARGET_SYSTEM / DEV_SYSTEM not found!'
    #endif

    #ifndef NO_QWORD
    #ifndef QWORD
        #define QWORD long long int
    #endif
    #endif

#elif defined(_MSC_VER)

    #if _MSC_VER < 1400         // requires visual studio 2005 or higher
        #error 'ERROR: Microsoft Visual Studio 2005 or higher is needed!'
    #endif

    // ------------------ definition target system --------------------------
    #if defined(_WIN32)
        #define TARGET_SYSTEM   _WIN32_     // WIN32 definition
        #define DEV_SYSTEM      _DEV_WIN32_
    #elif defined (_WIN32_WCE)
        #define TARGET_SYSTEM   _WINCE_
        #define DEV_SYSTEM      _DEV_WIN_CE_
    #else
        #error 'ERROR: TARGET_SYSTEM / DEV_SYSTEM not found!'
    #endif

    #define __func__ __FUNCTION__

#endif /*#elif defined (_MSC_VER) */

//---------------------------------------------------------------------------
//  TARGET_SYSTEM specific definitions
//---------------------------------------------------------------------------

// ------------------ GNUC for Linux ----------------------------------------
#if (TARGET_SYSTEM == _LINUX_)

    #ifndef __KERNEL__
        #include <string.h>
    #endif

    #define ROM_INIT                // variables will be initialized directly in ROM (means no copy from RAM in startup)
    #define ROM                     // code or variables mapped to ROM (i.e. flash)
                                    // usage: CONST BYTE ROM foo = 0x00;
    #define HWACC                   // hardware access through external memory (i.e. CAN)

    // These types can be adjusted by users to match application requirements. The goal is to
    // minimize code memory and maximize speed.
    #define GENERIC                 // generic pointer to point to application data
                                    // Variables with this attribute can be located in external
                                    // or internal data memory.
    #define MEM                     // Memory attribute to optimize speed and code of pointer access.

    #ifndef NEAR
        #define NEAR                // variables mapped to internal data storage location
    #endif

    #ifndef FAR
        #define FAR                 // variables mapped to external data storage location
    #endif

    #ifndef CONST
        #define CONST const         // variables mapped to ROM (i.e. flash)
    #endif

    #define LARGE

    #define REENTRANT
    #define PUBLIC

    #ifndef NDEBUG
        #ifndef __KERNEL__
            #include <stdio.h>              // prototype printf() (for TRACE)
            #define TRACE(...)  printf(__VA_ARGS__)
        #else
            #define TRACE(...)  printk(__VA_ARGS__)
        #endif
    #else
            #define TRACE(...)
    #endif

    #define UNUSED_PARAMETER(par)

// ------------------ GNUC for VxWorks ---------------------------------------
#elif (TARGET_SYSTEM == _VXWORKS_)

    #define ROM_INIT                // variables will be initialized directly in ROM (means no copy from RAM in startup)
    #define ROM                     // code or variables mapped to ROM (i.e. flash)
                                    // usage: CONST BYTE ROM foo = 0x00;
    #define HWACC                   // hardware access through external memory (i.e. CAN)

    // These types can be adjusted by users to match application requirements. The goal is to
    // minimize code memory and maximize speed.
    #define GENERIC                 // generic pointer to point to application data
                                    // Variables with this attribute can be located in external
                                    // or internal data memory.
    #define MEM                     // Memory attribute to optimize speed and code of pointer access.

    #ifndef NEAR
        #define NEAR                // variables mapped to internal data storage location
    #endif

    #ifndef FAR
        #define FAR                 // variables mapped to external data storage location
    #endif

    #ifndef CONST
        #define CONST const         // variables mapped to ROM (i.e. flash)
    #endif

    #define LARGE

    #define REENTRANT
    #define PUBLIC

    #ifndef NDEBUG
        #include <stdio.h>              // prototype printf() (for TRACE)
        #define TRACE(...)  printf(__VA_ARGS__)
    #else
        #define TRACE(...)
    #endif

    #define UNUSED_PARAMETER(par)

// ------------------ GNU without OS ---------------------------------------
#elif (TARGET_SYSTEM == _NO_OS_)

    #define ROM_INIT                // variables will be initialized directly in ROM (means no copy from RAM in startup)
    #define ROM                     // code or variables mapped to ROM (i.e. flash)
                                    // usage: CONST BYTE ROM foo = 0x00;
    #define HWACC                   // hardware access through external memory (i.e. CAN)

    // These types can be adjusted by users to match application requirements. The goal is to
    // minimize code memory and maximize speed.
    #define GENERIC                 // generic pointer to point to application data
                                    // Variables with this attribute can be located in external
                                    // or internal data memory.
    #define MEM                     // Memory attribute to optimize speed and code of pointer access.

    #ifndef NEAR
        #define NEAR                // variables mapped to internal data storage location
    #endif

    #ifndef FAR
        #define FAR                 // variables mapped to external data storage location
    #endif

    #ifndef CONST
        #define CONST const         // variables mapped to ROM (i.e. flash)
    #endif

    #define LARGE

    #define REENTRANT
    #define PUBLIC

    #ifndef NDEBUG
        #include <stdio.h>              // prototype printf() (for TRACE)
        #define TRACE(...)  printf(__VA_ARGS__)
    #else
        #define TRACE(...)
    #endif

    #define UNUSED_PARAMETER(par)

    #if (DEV_SYSTEM == _DEV_NIOS2_)
        #if !defined(__OPTIMIZE__)
             //restore default: disable inlining if optimization is disabled
            #define INLINE_FUNCTION
            #undef  INLINE_ENABLED
            #undef  INLINE_FUNCTION_DEF
        #endif
    #endif

// ------------------ WIN32 ---------------------------------------------
#elif (TARGET_SYSTEM == _WIN32_)

    #define ROM_INIT                // variables will be initialized directly in ROM (means no copy from RAM in startup)
    #define ROM                     // code or variables mapped to ROM (i.e. flash)
                                    // usage: CONST BYTE ROM foo = 0x00;
    #define HWACC                   // hardware access through external memory (i.e. CAN)

    // These types can be adjusted by users to match application requirements. The goal is to
    // minimize code memory and maximize speed.
    #define GENERIC                 // generic pointer to point to application data
                                    // Variables with this attribute can be located in external
                                    // or internal data memory.
    #define MEM                     // Memory attribute to optimize speed and code of pointer access.

    #ifndef NEAR
        #define NEAR                // variables mapped to internal data storage location
    #endif

    #ifndef FAR
        #define FAR                 // variables mapped to external data storage location
    #endif

    #ifndef CONST
        #define CONST const         // variables mapped to ROM (i.e. flash)
    #endif

    #define LARGE

    #define REENTRANT
    #define PUBLIC __stdcall

    #ifndef NO_QWORD
    #ifndef QWORD
      //#define QWORD long long int // MSVC .NET can use "long long int" too (like GNU)
        #define QWORD __int64
    #endif
    #endif

    #ifndef NDEBUG
        #define TRACE(...) trace(__VA_ARGS__)
        #ifdef __cplusplus
            extern "C"
            {
        #endif
            void trace (const char *fmt, ...);
        #ifdef __cplusplus
            }
        #endif
    #else
        #define TRACE(...)
    #endif

    #define UNUSED_PARAMETER(par) par

    // MS Visual C++ compiler supports function inlining
    #define INLINE_FUNCTION_DEF __forceinline

    // Redefine TRUE/FALSE if necessary
    #ifdef FALSE
    #undef FALSE
    #endif
    #define FALSE 0
    #ifdef TRUE
    #undef TRUE
    #endif
    #define TRUE 1

    // to actually enable inlining just include the following two lines
    // #define INLINE_FUNCTION     INLINE_FUNCTION_DEF
    // #define INLINE_ENABLED      TRUE

// ------------------ WINCE ---------------------------------------------
#elif (TARGET_SYSTEM == _WINCE_)

    #define ROM_INIT                // variables will be initialized directly in ROM (means no copy from RAM in startup)
    #define ROM                     // code or variables mapped to ROM (i.e. flash)
                                    // usage: CONST BYTE ROM foo = 0x00;
    #define HWACC                   // hardware access through external memory (i.e. CAN)

    // These types can be adjusted by users to match application requirements. The goal is to
    // minimize code memory and maximize speed.
    #define GENERIC                 // generic pointer to point to application data
                                    // Variables with this attribute can be located in external
                                    // or internal data memory.
    #define MEM                     // Memory attribute to optimize speed and code of pointer access.

    #ifndef NEAR
        #define NEAR                // variables mapped to internal data storage location
    #endif

    #ifndef FAR
        #define FAR                 // variables mapped to external data storage location
    #endif

    #ifndef CONST
        #define CONST const         // variables mapped to ROM (i.e. flash)
    #endif

    #define LARGE

    #ifndef NO_QWORD
    #ifndef QWORD
      //#define QWORD long long int // MSVC .NET can use "long long int" too (like GNU)
        #define QWORD __int64
    #endif
    #endif

    #define REENTRANT
    #define PUBLIC __cdecl

    #ifdef ASSERTMSG
        #undef ASSERTMSG
    #endif

    #ifndef NDEBUG
        #define TRACE(...) printf(__VA_ARGS__)
    #else
        #define TRACE(...)
    #endif

    #define UNUSED_PARAMETER(par)

    #define __func__ __FUNCTION__

#endif

//---------------------------------------------------------------------------
//  definitions of basic types
//---------------------------------------------------------------------------

#ifndef _WINDEF_        // defined in WINDEF.H, included by <windows.h>

    // --- arithmetic types ---
    #ifndef SHORT
        #define SHORT short int
    #endif

    #ifndef USHORT
        #define USHORT unsigned short int
    #endif

    #ifndef INT
        #define INT int
    #endif

    #ifndef UINT
        #define UINT unsigned int
    #endif

    #ifndef LONG
        #define LONG long int
    #endif

    #ifndef ULONG
        #define ULONG unsigned long int
    #endif

    #ifndef ULONGLONG
        #define ULONGLONG unsigned long long int
    #endif

    // --- logic types ---
    #ifndef BYTE
        #define BYTE unsigned char
    #endif

    #ifndef WORD
        #define WORD unsigned short int
    #endif

    #ifndef DWORD
        #if defined (__LP64__) || defined (_LP64)
            #define DWORD unsigned int
        #else
            #define DWORD unsigned long int
        #endif
    #endif

    #ifndef BOOL
        #define BOOL unsigned char
    #endif

    // --- alias types ---
    #ifndef TRUE
        #define TRUE  0xFF
    #endif

    #ifndef FALSE
        #define FALSE 0x00
    #endif

    #ifndef NULL
        #define NULL ((void *) 0)
    #endif

#endif

#ifndef _TIME_OF_DAY_DEFINED_

    typedef struct
    {
        unsigned long  int  m_dwMs;
        unsigned short int  m_wDays;

    } tTimeOfDay;

    #define _TIME_OF_DAY_DEFINED_

#endif

//---------------------------------------------------------------------------
//  definition of ASSERT
//---------------------------------------------------------------------------

#ifndef ASSERT
    #if !defined (__linux__) && !defined (__KERNEL__)
        #include <assert.h>
        #ifndef ASSERT
            #define ASSERT(p)    assert(p)
        #endif
    #else
        #define ASSERT(p)
    #endif
#endif


//---------------------------------------------------------------------------
//  SYS TEC extensions
//---------------------------------------------------------------------------

// This macro doesn't print out C-file and line number of the failed assertion
// but a string, which exactly names the mistake.
#ifndef ASSERTMSG
    #ifndef NDEBUG

            #define ASSERTMSG(expr,string)  if (!(expr)) { \
                                                PRINTF ("Assertion failed: " string);\
                                                for ( ; ; );}
    #else
        #define ASSERTMSG(expr,string)
    #endif
#endif

//---------------------------------------------------------------------------

#endif  // #ifndef _GLOBAL_H_

// EOF

