/**
********************************************************************************
\file   oplk/targetsystem.h

\brief  Target system definitions

The file contains definitions for selecting the target and development system
and including the suitable target specific definitions.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, B&R Industrial Automation GmbH
Copyright (c) 2013, SYSTEC electronic GmbH
Copyright (c) 2016, Kalycito Infotech Private Limited
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
#ifndef _INC_oplk_targetsystem_H_
#define _INC_oplk_targetsystem_H_

//------------------------------------------------------------------------------
// elements of defines for development system
// these defines are necessary to check some of the characteristics of the
// development system

// endianness
#define _DEV_BIGEND_            0x80000000L     // big endian (Motorola format)

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
#define _DEV_GNUC_MINGW_        0x00000025L     // MinGW GCC
#define _DEV_GNUC_ARM_ALTERA_   0x00000024L     // Altera toolchain mentor ARM EABI GCC
#define _DEV_GNUC_MICROBLAZE_   0x00000020L     // Xilinx Microblaze GCC
#define _DEV_GNUC_NIOS2_        0x0000001FL     // Altera Nios II GCC
#define _DEV_GNUC_X86_          0x00000017L     // GNU for I386
#define _DEV_GNUC_CF_           0x00000014L     // GNU for Coldfire
#define _DEV_MSEVC_             0x00000012L     // Microsoft embedded Visual C/C++
#define _DEV_GNUC_ARM7_         0x0000000EL     // GNU Compiler gcc for ARM7
#define _DEV_LINUX_GCC_         0x0000000AL     // Linux GNU Compiler gcc
#define _DEV_MSVC32_            0x00000000L     // Microsoft Visual C/C++

// these defines can be used to mask previous elements
#define _DEV_MASK_COMPILER      0x000000FFL
#define _DEV_MASK_BITCOUNT      0x00000F00L
#define _DEV_MASK_ADDSUPPORT    0x0000F000L
#define _DEV_MASK_ALIGNMENT     0x00F00000L

//------------------------------------------------------------------------------
//  defines for development system (DEV_SYSTEM) including previous elements

#define _DEV_WIN32_             (_DEV_BIT32_ | _DEV_MSVC32_                         | _DEV_64BIT_SUPPORT_ | _DEV_COMMA_EXT_)
#define _DEV_WIN_CE_            (_DEV_BIT32_ | _DEV_MSEVC_                          | _DEV_64BIT_SUPPORT_ | _DEV_COMMA_EXT_)
#define _DEV_WIN32_MINGW_       (_DEV_BIT32_ | _DEV_GNUC_MINGW_                     | _DEV_64BIT_SUPPORT_ | _DEV_COMMA_EXT_)
#define _DEV_LINUX_             (_DEV_BIT32_ | _DEV_LINUX_GCC_                      | _DEV_64BIT_SUPPORT_ | _DEV_COMMA_EXT_)
#define _DEV_GNU_CF548X_        (_DEV_BIT32_ | _DEV_GNUC_CF_        | _DEV_BIGEND_  | _DEV_64BIT_SUPPORT_ | _DEV_COMMA_EXT_)
#define _DEV_GNU_I386_          (_DEV_BIT32_ | _DEV_GNUC_X86_                       | _DEV_64BIT_SUPPORT_ | _DEV_COMMA_EXT_ | _DEV_ONLY_INT_MAIN_)
#define _DEV_NIOS2_             (_DEV_BIT32_ | _DEV_GNUC_NIOS2_                     | _DEV_64BIT_SUPPORT_ | _DEV_COMMA_EXT_ | _DEV_ONLY_INT_MAIN_ | _DEV_ALIGNMENT_4_ )
#define _DEV_VXWORKS_           (_DEV_BIT32_ | _DEV_LINUX_GCC_                      | _DEV_64BIT_SUPPORT_ | _DEV_COMMA_EXT_)
#define _DEV_MICROBLAZE_BIG_    (_DEV_BIT32_ | _DEV_GNUC_MICROBLAZE_ | _DEV_BIGEND_ | _DEV_64BIT_SUPPORT_ | _DEV_COMMA_EXT_ | _DEV_ONLY_INT_MAIN_ | _DEV_ALIGNMENT_4_ )
#define _DEV_MICROBLAZE_LITTLE_ (_DEV_BIT32_ | _DEV_GNUC_MICROBLAZE_                | _DEV_64BIT_SUPPORT_ | _DEV_COMMA_EXT_ | _DEV_ONLY_INT_MAIN_ | _DEV_ALIGNMENT_4_ )
#define _DEV_ARM_ALTERA_EABI_   (_DEV_BIT32_ | _DEV_GNUC_ARM_ALTERA_                | _DEV_64BIT_SUPPORT_ | _DEV_COMMA_EXT_ | _DEV_ONLY_INT_MAIN_)
//------------------------------------------------------------------------------
//  useful macros

#define CHECK_MEMORY_ALIGNMENT()    ((DEV_SYSTEM & _DEV_MASK_ALIGNMENT) != 0)
#define CHECK_IF_BIG_ENDIAN()       ((DEV_SYSTEM & _DEV_BIGEND_) != 0)

//------------------------------------------------------------------------------
//  defines for target system (TARGET_SYSTEM)

#define _NO_OS_             0
#define _LINUX_             1
#define _PXROS_             2
#define _ECOSPRO_           3
#define _VXWORKS_           4
#define _WIN32_             32
#define _WINCE_             (32 + 0x20000)

#if defined (__GNUC__)

//------------------------------------------------------------------------------
// GNU C compiler
//------------------------------------------------------------------------------
#if (defined(LINUX) || defined(__linux__))                  // x86 / Linux
#define TARGET_SYSTEM   _LINUX_
#define DEV_SYSTEM      _DEV_LINUX_
#elif defined(__NIOS2__)                                    // NIOS II / no-os
#define TARGET_SYSTEM   _NO_OS_
#define DEV_SYSTEM      _DEV_NIOS2_
#elif defined(__MICROBLAZE__)                               // Microblaze / no-os
#define TARGET_SYSTEM   _NO_OS_
// Microblaze could be either big or little endian
#if (__BIG_ENDIAN__ == 1)
#define DEV_SYSTEM      _DEV_MICROBLAZE_BIG_
#else /* (__BIG_ENDIAN__ == 1) */
#define DEV_SYSTEM      _DEV_MICROBLAZE_LITTLE_
#endif /* (__BIG_ENDIAN__ == 1) */
#elif defined(__VXWORKS__)                                  // x86 / VxWorks
#define TARGET_SYSTEM   _VXWORKS_
#define DEV_SYSTEM      _DEV_VXWORKS_
#elif defined(__ALTERA_ARM__)                               // Altera ARM / no-os
#define TARGET_SYSTEM   _NO_OS_
#define DEV_SYSTEM      _DEV_ARM_ALTERA_EABI_
#elif (defined (_WIN32) || defined (__MINGW32__))
#define TARGET_SYSTEM   _WIN32_                             // WIN32 definition
#define DEV_SYSTEM      _DEV_WIN32_MINGW_
#else /* (defined (_WIN32) || defined (__MINGW32__)) */     // unsupported
#error 'ERROR: TARGET_SYSTEM / DEV_SYSTEM not found!'
#endif

#define OPLK_DEPRECATED      __attribute__((deprecated))

#elif defined(_MSC_VER)
//------------------------------------------------------------------------------
// MS C compiler
//------------------------------------------------------------------------------
#if (_MSC_VER < 1400)         // requires visual studio 2005 or higher
#error 'ERROR: Microsoft Visual Studio 2005 or higher is needed!'
#endif

#if defined(_WIN32)
#define TARGET_SYSTEM   _WIN32_                             // x86 / Win32
#define DEV_SYSTEM      _DEV_WIN32_
#elif defined (_WIN32_WCE)                                  // x86 / Win CE
#define TARGET_SYSTEM   _WINCE_
#define DEV_SYSTEM      _DEV_WIN_CE_
#else /* defined (_WIN32_WCE) */                            // unsupported
#error 'ERROR: TARGET_SYSTEM / DEV_SYSTEM not found!'
#endif

#define OPLK_DEPRECATED     __declspec(deprecated)

#if !defined(__func__)
#define __func__            __FUNCTION__                    // VS2012 or older
#endif

#endif /* defined(_MSC_VER) */

//------------------------------------------------------------------------------
//  TARGET_SYSTEM specific definitions
//------------------------------------------------------------------------------
#if (TARGET_SYSTEM == _LINUX_)

#include <oplk/targetdefs/linux.h>

#elif (TARGET_SYSTEM == _VXWORKS_)

#include <oplk/targetdefs/vxworks.h>

#elif (TARGET_SYSTEM == _NO_OS_)

#if (DEV_SYSTEM == _DEV_NIOS2_)
#include <oplk/targetdefs/nios2.h>
#elif ((DEV_SYSTEM == _DEV_MICROBLAZE_BIG_) || (DEV_SYSTEM == _DEV_MICROBLAZE_LITTLE_))
#include <oplk/targetdefs/microblaze.h>
#elif (DEV_SYSTEM == _DEV_ARM_ALTERA_EABI_)
#include <oplk/targetdefs/c5socarm.h>
#else
#error "ERROR Target no OS System is not supported"
#endif

#elif (TARGET_SYSTEM == _WIN32_)

#if (DEV_SYSTEM == _DEV_WIN32_)
#ifdef _KERNEL_MODE
#include <oplk/targetdefs/winkernel.h>
#else /* _KERNEL_MODE */
#include <oplk/targetdefs/windows.h>
#endif /* _KERNEL_MODE */
#elif (DEV_SYSTEM == _DEV_WIN32_MINGW_)
#include <oplk/targetdefs/windows-mingw.h>
#else
#error "ERROR Development platform is not supported for this target"
#endif

#elif (TARGET_SYSTEM == _WINCE_)

#include <oplk/targetdefs/wince.h>

#else /* (TARGET_SYSTEM == _WINCE_) */

#error "ERROR Target platform is not supported"

#endif

#endif /* _INC_oplk_targetsystem_H_ */
