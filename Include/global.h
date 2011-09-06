/****************************************************************************

    global project definition file

    12.06.1998   -rs
    11.02.2002   r.d. Erweiterungen, Ergaenzungen
    20.08.2002   SYS TEC electronic -as
                 Definition Schluesselwort 'GENERIC'
                 fuer das Erzeugen von Generic Pointer
    28.08.2002   r.d. erweiterter SYS TEC Debug Code
    16.09.2002   r.d. komplette Uebersetzung in Englisch
    11.04.2003   f.j. Ergaenzung fuer Mitsubishi NC30 Compiler
    17.06.2003   -rs  Definition von Basistypen in <#ifndef _WINDEF_> gesetzt
    16.04.2004   r.d. Ergaenzung fuer Borland C++ Builder
    30.08.2004   -rs  TRACE5 eingefügt
    23.12.2005   d.k. Definitions for IAR compiler

    $Id$

****************************************************************************/

#ifndef _GLOBAL_H_
#define _GLOBAL_H_


//---------------------------------------------------------------------------
//  elements of defines for development system
//---------------------------------------------------------------------------

// these defines are necessary to check some of characteristics of the development system
#define _DEV_BIGEND_            0x80000000L     // big endian (motorolla format)
#define _DEV_DSP_               0x40000000L     // DSP which implements sizeof(BYTE)=1 and sizeof(WORD)=1 with 16bit and sizeof(DWORD)=2 with 32bit
#define _DEV_ALIGNMENT_4_       0x00400000L     //                  the CPU needs alignment of 4 bytes
#define _DEV_ONLY_INT_MAIN_     0x00004000L     //                  the compiler needs "int main(int)" instead of "void main(void)"
#define _DEV_COMMA_EXT_         0x00002000L     //                  support of last comma in struct predefinition
#define _DEV_64BIT_SUPPORT_     0x00001000L     //                  support of 64 bit operations
#define _DEV_BIT64_             0x00000400L     // count of bits:   64 bit
#define _DEV_BIT32_             0x00000300L     //                  32 bit
#define _DEV_BIT16_             0x00000200L     //                  16 bit
#define _DEV_BIT8_              0x00000100L     //                  8 bit
#define _DEV_GNUC_MICROBLAZE    0x00000020L     //                  Xilinx Microblaze GCC //not present in altera but needed
#define _DEV_GNUC_NIOS2_        0x0000001FL     //                  Altera Nios II GCC
#define _DEV_TI_CCS_            0x0000001EL     //                  TI's Code Composer
#define _DEV_GNUC_AVR_          0x0000001DL     //                  WinAVR and AVRStudio4
#define _DEV_RVCT_ARM_          0x0000001CL     //                  Keil RealView ARM
#define _DEV_RENESASM32C        0x0000001BL     // compiler from:   Renesas
#define _DEV_GNUC_MIPS2_        0x0000001AL     //                  GNU for MIPS2
#define _DEV_MPLAB_C30_         0x00000019L     //                  MPLAB C30 for Microchip dsPIC33F series
#define _DEV_GNUC_TC_           0x00000018L     //                  GNU for Infineon TriCore
#define _DEV_GNUC_X86_          0x00000017L     //                  GNU for I386
#define _DEV_IAR_ARM_           0x00000016L     //                  ARM IAR C/C++ Compiler
#define _DEV_PARADGM_X86        0x00000015L     //                  Paradigm C/C++ for Beck 1x3
#define _DEV_GNUC_CF_           0x00000014L     //                  GNU for Coldfire
#define _DEV_KEIL_ARM_          0x00000013L     //                  Keil ARM
#define _DEV_MSEVC_             0x00000012L     //                  Microsoft embedded Visual C/C++
#define _DEV_HIGHTEC_GNUC_X86_  0x00000011L     //                  Hightec elf386 gcc
#define _DEV_MSVC_RTX_          0x00000010L     //                  VC600 + RTX
#define _DEV_MSVC_V1_5_         0x0000000FL     //                  Microsoft Visual C/C++ V1.5
#define _DEV_GNUC_ARM7_         0x0000000EL     //                  GNU Compiler gcc for ARM7
#define _DEV_METROWERKS_CW_     0x0000000DL     //                  Metrowerks Code Warrior
#define _DEV_MITSUBISHIM16C_    0x0000000CL     //compiler from:    Mitsubishi
#define _DEV_GNUC_C16X_         0x0000000BL     //                  GNU Compiler gcc166 for Infineon C16x
#define _DEV_LINUX_GCC_         0x0000000AL     //                  Linux GNU Compiler gcc
#define _DEV_GNUC_MPC5X5        0x00000009L     //                  GNU for Motorola PPC5x5
#define _DEV_TASKINGM16C_       0x00000008L     //                  Tasking for Mitsubishi M16C
#define _DEV_FUJITSU_           0x00000007L     //                  Fujitsu
#define _DEV_TASKING8_          0x00000006L     //                  Tasking 8051
#define _DEV_TASKING16_         0x00000005L     //                  Tasking 166
#define _DEV_KEIL8_             0x00000004L     //                  Keil C51
#define _DEV_KEIL16_            0x00000003L     //                  Keil C166
#define _DEV_BORLANDC_          0x00000002L     //                  Borland C/C++
#define _DEV_MSVC16_            0x00000001L     //                  Microsoft Visual C/C++
#define _DEV_MSVC32_            0x00000000L     //                  Microsoft Visual C/C++

// these defines can be used to mask previous elements
#define _DEV_MASK_COMPILER      0x000000FFL
#define _DEV_MASK_BITCOUNT      0x00000F00L
#define _DEV_MASK_ADDSUPPORT    0x0000F000L
#define _DEV_MASK_ALIGNMENT     0x00F00000L


//---------------------------------------------------------------------------
//  defines for development system (DEV_SYSTEM) including previous elements
//---------------------------------------------------------------------------

#define _DEV_WIN16_             (_DEV_BIT16_ | _DEV_MSVC16_                  )
#define _DEV_WIN32_             (_DEV_BIT32_ | _DEV_MSVC32_                   | _DEV_64BIT_SUPPORT_ | _DEV_COMMA_EXT_)
#define _DEV_MSVC_DOS_          (_DEV_BIT32_ | _DEV_MSVC_V1_5_               )
#define _DEV_BORLAND_DOS_       (_DEV_BIT32_ | _DEV_BORLANDC_                ) //| _DEV_64BIT_SUPPORT_ | _DEV_COMMA_EXT_)
#define _DEV_KEIL_C51X_         (_DEV_BIT8_  | _DEV_KEIL8_     | _DEV_BIGEND_ | _DEV_COMMA_EXT_) // at least C51 version 7.05 supports comma extension
#define _DEV_KEIL_C16X_         (_DEV_BIT16_ | _DEV_KEIL16_                   | _DEV_COMMA_EXT_) // at least C166 version 5.03 supports comma extension
#define _DEV_TASKING_C51X_      (_DEV_BIT8_  | _DEV_TASKING8_  | _DEV_BIGEND_)
#define _DEV_TASKING_C16X_      (_DEV_BIT16_ | _DEV_TASKING16_               )
#define _DEV_FUJITSU_F590_      (_DEV_BIT8_  | _DEV_FUJITSU_                  | _DEV_COMMA_EXT_) // softune is not able to support 64 bit variables QWORD !!!
//f.j.29.04.03 M16C kann effektiv mit Bytes umgehen
//#define _DEV_TASKING_M16C_      (_DEV_BIT16_ | _DEV_TASKINGM16C_             )
#define _DEV_TASKING_M16C_      (_DEV_BIT8_  | _DEV_TASKINGM16C_             )
#define _DEV_MITSUBISHI_M16C_   (_DEV_BIT8_  | _DEV_MITSUBISHIM16C_          )
#define _DEV_GNU_MPC5X5_        (_DEV_BIT32_ | _DEV_GNUC_MPC5X5| _DEV_BIGEND_ | _DEV_64BIT_SUPPORT_ | _DEV_COMMA_EXT_)
#define _DEV_LINUX_             (_DEV_BIT32_ | _DEV_LINUX_GCC_                | _DEV_64BIT_SUPPORT_ | _DEV_COMMA_EXT_)
#define _DEV_GNU_C16X_          (_DEV_BIT16_ | _DEV_GNUC_C16X_               ) //| _DEV_COMMA_EXT_)
#define _DEV_MCW_MPC5X5_        (_DEV_BIT32_ | _DEV_METROWERKS_CW_           ) //| _DEV_64BIT_SUPPORT_ | _DEV_COMMA_EXT_)
#define _DEV_GNU_ARM7_          (_DEV_BIT32_ | _DEV_GNUC_ARM7_                | _DEV_64BIT_SUPPORT_ | _DEV_COMMA_EXT_ | _DEV_ONLY_INT_MAIN_)
#define _DEV_WIN32_RTX_         (_DEV_BIT32_ | _DEV_MSVC_RTX_                ) //| _DEV_64BIT_SUPPORT_ | _DEV_COMMA_EXT_)
#define _DEV_HIGHTEC_X86_       (_DEV_BIT32_ | _DEV_HIGHTEC_GNUC_X86_        ) //| _DEV_64BIT_SUPPORT_ | _DEV_COMMA_EXT_)
#define _DEV_WIN_CE_            (_DEV_BIT32_ | _DEV_MSEVC_                    | _DEV_64BIT_SUPPORT_ | _DEV_COMMA_EXT_)
#define _DEV_KEIL_CARM_         (_DEV_BIT32_ | _DEV_KEIL_ARM_                 | _DEV_64BIT_SUPPORT_ | _DEV_COMMA_EXT_)
#define _DEV_IAR_CARM_          (_DEV_BIT32_ | _DEV_IAR_ARM_                  | _DEV_64BIT_SUPPORT_ | _DEV_COMMA_EXT_)
#define _DEV_RVCT_CARM_         (_DEV_BIT32_ | _DEV_RVCT_ARM_                 | _DEV_64BIT_SUPPORT_ | _DEV_COMMA_EXT_ | _DEV_ONLY_INT_MAIN_)
#define _DEV_MCW_MCF5XXX_       (_DEV_BIT32_ | _DEV_METROWERKS_CW_           ) //| _DEV_64BIT_SUPPORT_ | _DEV_COMMA_EXT_)
#define _DEV_GNU_CF5282_        (_DEV_BIT32_ | _DEV_GNUC_CF_   | _DEV_BIGEND_)
#define _DEV_PAR_BECK1X3_       (_DEV_BIT16_ | _DEV_PARADGM_X86)
#define _DEV_GNU_CF548X_        (_DEV_BIT32_ | _DEV_GNUC_CF_   | _DEV_BIGEND_ | _DEV_64BIT_SUPPORT_ | _DEV_COMMA_EXT_)
#define _DEV_GNU_I386_          (_DEV_BIT32_ | _DEV_GNUC_X86_                 | _DEV_64BIT_SUPPORT_ | _DEV_COMMA_EXT_ | _DEV_ONLY_INT_MAIN_)
#define _DEV_GNU_TRICORE_       (_DEV_BIT32_ | _DEV_GNUC_TC_                  | _DEV_64BIT_SUPPORT_ | _DEV_COMMA_EXT_ | _DEV_ONLY_INT_MAIN_ | _DEV_ALIGNMENT_4_)
#define _DEV_MPLAB_DSPIC33F_    (_DEV_BIT16_ | _DEV_MPLAB_C30_                | _DEV_64BIT_SUPPORT_ | _DEV_COMMA_EXT_ | _DEV_ONLY_INT_MAIN_)
#define _DEV_GNU_MIPSEL_        (_DEV_BIT32_ | _DEV_GNUC_MIPS2_     | _DEV_BIGEND_ | _DEV_64BIT_SUPPORT_ | _DEV_COMMA_EXT_ | _DEV_ONLY_INT_MAIN_)
#define _DEV_GNU_WINAVR8_       (_DEV_BIT8_  | _DEV_GNUC_AVR_                 |                       _DEV_COMMA_EXT_ | _DEV_ONLY_INT_MAIN_)
// note: WinAVR is supports 64 bit variables but it needs to much memory resources!

#define _DEV_RENESAS_M32C_      (_DEV_BIT32_ | _DEV_RENESASM32C)
#define _DEV_CCS_TMS_           (_DEV_BIT16_ | _DEV_TI_CCS_  | _DEV_COMMA_EXT_ | _DEV_DSP_)

#define _DEV_NIOS2_             (_DEV_BIT32_ | _DEV_GNUC_NIOS2_               | _DEV_64BIT_SUPPORT_ | _DEV_COMMA_EXT_ | _DEV_ONLY_INT_MAIN_ | _DEV_ALIGNMENT_4_ )
#define _DEV_MICROBLAZE_        (_DEV_BIT32_ | _DEV_GNUC_MICROBLAZE   | _DEV_BIGEND_ | _DEV_64BIT_SUPPORT_ | _DEV_COMMA_EXT_ | _DEV_ONLY_INT_MAIN_ | _DEV_ALIGNMENT_4_ ) //not in altera but needed


//---------------------------------------------------------------------------
//  usefull macros
//---------------------------------------------------------------------------

#define CHECK_IF_ONLY_INT_MAIN()    ((DEV_SYSTEM & _DEV_ONLY_INT_MAIN_) != 0)
#define CHECK_MEMORY_ALINMENT()     ((DEV_SYSTEM & _DEV_MASK_ALIGNMENT) != 0)
#define CHECK_IF_BIG_ENDIAN()       ((DEV_SYSTEM & _DEV_BIGEND_) != 0)


//---------------------------------------------------------------------------
//  defines for target system (TARGET_SYSTEM)
//---------------------------------------------------------------------------

#define _DOS_              (16 + 0x10000)
#define _WIN16_             16
#define _WIN32_             32
#define _WINCE_            (32 + 0x20000)
#define _NO_OS_              0
#define _LINUX_              1
#define _PXROS_              2
#define _ECOSPRO_            3


//---------------------------------------------------------------------------
//  definitions for function inlining
//---------------------------------------------------------------------------

#define INLINE_FUNCTION             // empty define
#undef  INLINE_ENABLED              // disable actual inlining of functions
#undef  INLINE_FUNCTION_DEF         // disable inlining for all compilers per default

//---------------------------------------------------------------------------
//  definitions for Keil C51
//---------------------------------------------------------------------------

#ifdef  __C51__

    #define TARGET_SYSTEM   _NO_OS_
    #define DEV_SYSTEM      _DEV_KEIL_C51X_

    #pragma DEBUG OBJECTEXTEND
    #pragma WARNINGLEVEL(2)             // maximum warning level

    #define NEAR            idata       // variables mapped to internal data storage location
    #define FAR             xdata       // variables mapped to external data storage location
    #define CONST           const       // variables mapped to ROM (i.e. flash)
    #define ROM_INIT                    // variables will be initialized directly in ROM (means no copy from RAM in startup)
    #define ROM             code        // code or variables mapped to ROM (i.e. flash)
                                        // usage: CONST BYTE ROM foo = 0x00;
    #define HWACC           xdata       // hardware access through external memory (i.e. CAN)
    #define LARGE           large       // functions set parameters to external data storage location

    // These types can be adjusted by users to match application requirements. The goal is to
    // minimize code memory and maximize speed.
    #define GENERIC                     // generic pointer to point to application data
                                        // Variables with this attribute can be located in external
                                        // or internal data memory.
    #define MEM             xdata       // Memory attribute to optimize speed and code of pointer access.

    #define REENTRANT       reentrant
    #define PUBLIC

    #ifndef NDEBUG
        #include <stdio.h>              // prototype printf() (for TRACE)
        #define TRACE  printf
    #endif

    #define UNUSED_PARAMETER(par)   par = par


//---------------------------------------------------------------------------
//  definitions for GNU Compiler for Infineon C16x
//  - it have to be befor Keil (it has __C166__ too)
//---------------------------------------------------------------------------
#elif  defined (__GNUC__) && defined (__C166__)

    #define TARGET_SYSTEM   _NO_OS_
    #define DEV_SYSTEM      _DEV_GNU_C16X_

    #define NEAR            near       // variables mapped to internal data storage location
    #define FAR             huge       // variables mapped to external data storage location
    #define CONST           const       // variables mapped to ROM (i.e. flash)
    #define ROM_INIT                    // variables will be initialized directly in ROM (means no copy from RAM in startup)
    #define ROM                         // code or variables mapped to ROM (i.e. flash)
                                        // usage: CONST BYTE ROM foo = 0x00;
    #define HWACC           huge       // hardware access through external memory (i.e. CAN)
    #define LARGE                       // functions set parameters to external data storage location

    // These types can be adjusted by users to match application requirements. The goal is to
    // minimize code memory and maximize speed.
    #define GENERIC         huge       // generic pointer to point to application data
                                        // Variables with this attribute can be located in external
                                        // or internal data memory.
    #define MEM                         // Memory attribute to optimize speed and code of pointer access.

    #define REENTRANT
    #define PUBLIC

    #ifndef NDEBUG
        #include <stdio.h>              // prototype printf() (for TRACE)
        #define TRACE  printf

        #define ASSERT(p)  \
            if (p)         \
            {              \
                ;          \
            }              \
            else           \
            {              \
                PRINTF0("Assert failed: " #p " (file %s line %d)\n", __FILE__, (int) __LINE__ ); \
                while (1); \
            }
    #else
        #define ASSERT(p)
    #endif

    #define UNUSED_PARAMETER(par)

//---------------------------------------------------------------------------
//  definitions for Keil C166
//---------------------------------------------------------------------------
#elif  defined (__C166__)               // 24.01.2005 r.d.: Keil ARM7 needs directive 'defined'

    #define TARGET_SYSTEM   _NO_OS_
    #define DEV_SYSTEM      _DEV_KEIL_C16X_

    #pragma CODE
    #pragma MOD167
    #pragma NOINIT
    #pragma DEBUG
    #pragma WARNINGLEVEL(3)             // maximum warning level
    #pragma WARNING DISABLE = 47        // warning <unreferenced parameter> = OFF
    #pragma WARNING DISABLE = 38        // warning <empty translation unit> = OFF
//  #pragma WARNING DISABLE = 102       // warning <different const/volatile qualifiers> = OFF
    #pragma WARNING DISABLE = 174       // warning <unreferenced 'static' function> = OFF
    #pragma WARNING DISABLE = 183       // warning <dead assignement eliminated> = OFF

    #define NEAR            idata       // variables mapped to internal data storage location
    #define FAR             xhuge       // variables mapped to external data storage location
    #define CONST           const       // variables mapped to ROM (i.e. flash)
    #define ROM_INIT                    // variables will be initialized directly in ROM (means no copy from RAM in startup)
    #define ROM                         // code or variables mapped to ROM (i.e. flash)
                                        // usage: CONST BYTE ROM foo = 0x00;
//    #define HWACC           sdata       // hardware access through external memory (i.e. CAN)
    #define HWACC           huge       // hardware access through external memory (i.e. CAN)
    #define LARGE                       // functions set parameters to external data storage location

    // These types can be adjusted by users to match application requirements. The goal is to
    // minimize code memory and maximize speed.
    #define GENERIC         xhuge       // generic pointer to point to application data
                                        // Variables with this attribute can be located in external
                                        // or internal data memory.
    #define MEM                         // Memory attribute to optimize speed and code of pointer access.

    #define REENTRANT
    #define PUBLIC

    #ifndef NDEBUG
        #include <stdio.h>              // prototype printf() (for TRACE)
        #define TRACE  printf
    #endif

    #define UNUSED_PARAMETER(par)

//---------------------------------------------------------------------------
//  definitions for MPLAB C30 for dsPIC33F series
//---------------------------------------------------------------------------
#elif  defined (__C30__)

    #define TARGET_SYSTEM   _NO_OS_
    #define DEV_SYSTEM      _DEV_MPLAB_DSPIC33F_

    #define NEAR                        // variables mapped to internal data storage location
    #define FAR                         // variables mapped to external data storage location
    #define CONST        const          // variables mapped to ROM (i.e. flash)
    #define ROM_INIT                    // variables will be initialized directly in ROM (means no copy from RAM in startup)
    #define ROM                         // code or variables mapped to ROM (i.e. flash)
                                        // usage: CONST BYTE ROM foo = 0x00;
    #define HWACC                       // hardware access through external memory (i.e. CAN)
    #define LARGE                       // functions set parameters to external data storage location

    // These types can be adjusted by users to match application requirements. The goal is to
    // minimize code memory and maximize speed.
    #define GENERIC                     // generic pointer to point to application data
                                        // Variables with this attribute can be located in external
                                        // or internal data memory.
    #define MEM                         // Memory attribute to optimize speed and code of pointer access.

    #define REENTRANT
    #define PUBLIC

    #ifndef NO_QWORD
    #ifndef QWORD
        #define QWORD long long
    #endif
    #endif


    #ifndef NDEBUG
        #include <stdio.h>              // prototype printf() (for TRACE)
        #define TRACE  printf
    #endif

    #define UNUSED_PARAMETER(par)

//---------------------------------------------------------------------------
//  definitions for Keil ARM
//---------------------------------------------------------------------------
#elif  defined (__CA__)

    #define TARGET_SYSTEM   _NO_OS_
    #define DEV_SYSTEM      _DEV_KEIL_CARM_

    #define NEAR                        // variables mapped to internal data storage location
    #define FAR                         // variables mapped to external data storage location
    #define CONST        const          // variables mapped to ROM (i.e. flash)
    #define ROM_INIT                    // variables will be initialized directly in ROM (means no copy from RAM in startup)
    #define ROM                         // code or variables mapped to ROM (i.e. flash)
                                        // usage: CONST BYTE ROM foo = 0x00;
    #define HWACC                       // hardware access through external memory (i.e. CAN)
    #define LARGE                       // functions set parameters to external data storage location

    // These types can be adjusted by users to match application requirements. The goal is to
    // minimize code memory and maximize speed.
    #define GENERIC                     // generic pointer to point to application data
                                        // Variables with this attribute can be located in external
                                        // or internal data memory.
    #define MEM                         // Memory attribute to optimize speed and code of pointer access.

    #define REENTRANT
    #define PUBLIC

    #ifndef NO_QWORD
    #ifndef QWORD
        #define QWORD long long
    #endif
    #endif

    #ifndef NDEBUG
        #include <stdio.h>              // prototype printf() (for TRACE)
        #define TRACE  printf
    #endif

    #define UNUSED_PARAMETER(par)       par

//---------------------------------------------------------------------------
//  definitions for RealView ARM compilation tools (provided by recent Keil Microcontroller Development Kits)
//---------------------------------------------------------------------------
#elif  defined (__ARMCC_VERSION)

    #define TARGET_SYSTEM   _NO_OS_
    #define DEV_SYSTEM      _DEV_RVCT_CARM_

    #define NEAR                        // variables mapped to internal data storage location
    #define FAR                         // variables mapped to external data storage location
    #define CONST        const          // variables mapped to ROM (i.e. flash)
    #define ROM_INIT                    // variables will be initialized directly in ROM (means no copy from RAM in startup)
    #define ROM                         // code or variables mapped to ROM (i.e. flash)
                                        // usage: CONST BYTE ROM foo = 0x00;
    #define HWACC                       // hardware access through external memory (i.e. CAN)
    #define LARGE                       // functions set parameters to external data storage location

    // These types can be adjusted by users to match application requirements. The goal is to
    // minimize code memory and maximize speed.
    #define GENERIC                     // generic pointer to point to application data
                                        // Variables with this attribute can be located in external
                                        // or internal data memory.
    #define MEM                         // Memory attribute to optimize speed and code of pointer access.

    #define REENTRANT
    #define PUBLIC

    #ifndef NO_QWORD
    #ifndef QWORD
        #define QWORD long long
    #endif
    #endif

    #ifndef NDEBUG
        #define ASSERT(expr)    if (!(expr)) {\
                                   TRACE0 ("Assertion failed: " #expr );\
                                   while (1);}
    #else
        #define ASSERT(expr)
    #endif

    #ifndef NDEBUG
        #include <stdio.h>              // prototype printf() (for TRACE)
        #define TRACE  printf
    #endif

    #define UNUSED_PARAMETER(par)

//---------------------------------------------------------------------------
//  definitions for ARM IAR C Compiler
//---------------------------------------------------------------------------
#elif  defined (__ICCARM__)

    #define TARGET_SYSTEM   _NO_OS_
    #define DEV_SYSTEM      _DEV_IAR_CARM_

    #define NEAR                        // variables mapped to internal data storage location
    #define FAR                         // variables mapped to external data storage location
    #define CONST        const          // variables mapped to ROM (i.e. flash)
    #define ROM_INIT                    // variables will be initialized directly in ROM (means no copy from RAM in startup)
    #define ROM                         // code or variables mapped to ROM (i.e. flash)
                                        // usage: CONST BYTE ROM foo = 0x00;
    #define HWACC                       // hardware access through external memory (i.e. CAN)
    #define LARGE                       // functions set parameters to external data storage location

    // These types can be adjusted by users to match application requirements. The goal is to
    // minimize code memory and maximize speed.
    #define GENERIC                     // generic pointer to point to application data
                                        // Variables with this attribute can be located in external
                                        // or internal data memory.
    #define MEM                         // Memory attribute to optimize speed and code of pointer access.

    #define REENTRANT
    #define PUBLIC

    #ifndef NO_QWORD
    #ifndef QWORD
        #define QWORD long long
    #endif
    #endif

    // Workaround:
    // If we use IAR and want to debug but don't want to use C-Spy Debugger
    // assert() doesn't work in debug mode because it needs support for FILE descriptors
    // (_DLIB_FILE_DESCRIPTOR == 1).
    #ifndef NDEBUG
        #define ASSERT(expr)    if (!(expr)) {\
                                   TRACE0 ("Assertion failed: " #expr );\
                                   while (1);}
    #else
        #define ASSERT(expr)
    #endif

    #ifndef NDEBUG
        #include <stdio.h>              // prototype printf() (for TRACE)
        #define TRACE  printf
//        #define TRACE  PRINTF4
    #endif

    #define UNUSED_PARAMETER(par)

//---------------------------------------------------------------------------
//  definitions for TI DSP using COde Composer Studio
//---------------------------------------------------------------------------
#elif  defined (__TMS320C2000__)

    #define TARGET_SYSTEM   _NO_OS_
    #define DEV_SYSTEM      _DEV_CCS_TMS_

    #define NEAR         near           // variables mapped to internal data storage location
    #define FAR                         // variables mapped to external data storage location
    #define CONST        const          // variables mapped to ROM (i.e. flash)
    #define ROM_INIT                    // variables will be initialized directly in ROM (means no copy from RAM in startup)
    #define ROM                         // code or variables mapped to ROM (i.e. flash)
                                        // usage: CONST BYTE ROM foo = 0x00;
    #define HWACC                       // hardware access through external memory (i.e. CAN)
    #define LARGE                       // functions set parameters to external data storage location

    // These types can be adjusted by users to match application requirements. The goal is to
    // minimize code memory and maximize speed.
    #define GENERIC                     // generic pointer to point to application data
                                        // Variables with this attribute can be located in external
                                        // or internal data memory.
    #define MEM          near           // Memory attribute to optimize speed and code of pointer access.

    #define REENTRANT
    #define PUBLIC       //near

    #ifndef BYTE
        #define BYTE unsigned char
    #endif

    #ifndef WORD
        #define WORD unsigned short int
    #endif

    #ifndef DWORD
        //#define DWORD unsigned int      // THIS IS IMPORTANT HERE: "unsigned long int" is a 64-bit variable for Code Composer !!!
    #endif

    #ifndef BOOL
        #define BOOL unsigned char
    #endif

    #ifndef NO_QWORD
    #ifndef QWORD
        #define QWORD long long
    #endif
    #endif

    // Workaround:
    // If we use IAR and want to debug but don't want to use C-Spy Debugger
    // assert() doesn't work in debug mode because it needs support for FILE descriptors
    // (_DLIB_FILE_DESCRIPTOR == 1).
    #ifndef NDEBUG
        #define ASSERT(expr)    if (!(expr)) {\
                                   TRACE0 ("Assertion failed: " #expr );\
                                   while (1);}
    #else
        #define ASSERT(expr)
    #endif

    #ifndef NDEBUG
        //#include <stdio.h>              // prototype printf() (for TRACE)
        int TgtPrintf (const char *format, ...);
        #define TRACE  TgtPrintf
    #endif

    #define UNUSED_PARAMETER(par)

//---------------------------------------------------------------------------
//  definitions for Tasking 8051
//---------------------------------------------------------------------------

#elif defined (_CC51)

    #include <cc51.h>

    #define TARGET_SYSTEM   _NO_OS_
    #define DEV_SYSTEM      _DEV_TASKING_C51X_

    #define NEAR            _data       // variables mapped to internal data storage location
    #define FAR             _xdat       // variables mapped to external data storage location
    #define CONST           const       // variables mapped to ROM (i.e. flash)
    #define ROM_INIT                    // variables will be initialized directly in ROM (means no copy from RAM in startup)
    #define ROM                         // code or variables mapped to ROM (i.e. flash)
                                        // usage: CONST BYTE ROM foo = 0x00;
    #define HWACC           _xdat       // hardware access through external memory (i.e. CAN)
    #define LARGE                       // functions set parameters to external data storage location

    // These types can be adjusted by users to match application requirements. The goal is to
    // minimize code memory and maximize speed.
    #define GENERIC                     // generic pointer to point to application data
                                        // Variables with this attribute can be located in external
                                        // or internal data memory.
    #define MEM             _xdat       // Memory attribute to optimize speed and code of pointer access.

    #define REENTRANT       _reentrant
    #define PUBLIC

    #ifndef NDEBUG
        #include <stdio.h>              // prototype printf() (for TRACE)
        #define TRACE  printf
    #endif

    #define UNUSED_PARAMETER(par)

//---------------------------------------------------------------------------
//  definitions for Tasking C167CR and C164CI
//---------------------------------------------------------------------------

#elif defined (_C166)

    #define TARGET_SYSTEM   _NO_OS_
    #define DEV_SYSTEM      _DEV_TASKING_C16X_

    #define NEAR            near        // variables mapped to internal data storage location
    #define FAR             far         // variables mapped to external data storage location
    #define CONST           const       // variables mapped to ROM (i.e. flash)
    #define ROM_INIT                    // variables will be initialized directly in ROM (means no copy from RAM in startup)
    #define ROM                         // code or variables mapped to ROM (i.e. flash)
                                        // usage: CONST BYTE ROM foo = 0x00;
    #define HWACC   /* to be defined */ // hardware access through external memory (i.e. CAN)
    #define LARGE                       // functions set parameters to external data storage location

    // These types can be adjusted by users to match application requirements. The goal is to
    // minimize code memory and maximize speed.
    #define GENERIC                     // generic pointer to point to application data
                                        // Variables with this attribute can be located in external
                                        // or internal data memory.
    #define MEM                         // Memory attribute to optimize speed and code of pointer access.

    #define REENTRANT
    #define PUBLIC

    // Stdio.h has to be alway included here. If printf() is used stdio.h defines NULL
    // without checking if it is already included. So an error occurs while compiling.
    // (r.d.)
    #include <stdio.h>                  // prototype printf() (for TRACE)
    #ifndef NDEBUG
        #define TRACE  printf
    #endif

    #define UNUSED_PARAMETER(par)

//---------------------------------------------------------------------------
//  definitions for FUJITSU FFMC-16LX MB90590
//---------------------------------------------------------------------------

//#elif (defined (F590) || defined (F543) || defined (F598) || defined (F495) || defined (F350))
#elif defined(__COMPILER_FCC907__)

    #define TARGET_SYSTEM   _NO_OS_
    #define DEV_SYSTEM      _DEV_FUJITSU_F590_

    #define NEAR    /* to be defined */ // variables mapped to internal data storage location
    #define FAR     /* to be defined */ // variables mapped to external data storage location
    #define CONST           const       // variables mapped to ROM (i.e. flash)
    #define ROM_INIT                    // variables will be initialized directly in ROM (means no copy from RAM in startup)
    #define ROM     /* to be defined */ // code or variables mapped to ROM (i.e. flash)
                                        // usage: CONST BYTE ROM foo = 0x00;
    #define HWACC   /* to be defined */ // hardware access through external memory (i.e. CAN)
    #define LARGE                       // functions set parameters to external data storage location

    // These types can be adjusted by users to match application requirements. The goal is to
    // minimize code memory and maximize speed.
    #define GENERIC                     // generic pointer to point to application data
                                        // Variables with this attribute can be located in external
                                        // or internal data memory.
    #define MEM                         // Memory attribute to optimize speed and code of pointer access.

    // softune is not able to support 64 bit variables QWORD !!!

    #define REENTRANT
    #define PUBLIC

    #ifndef NDEBUG
        #include <stdio.h>              // prototype printf() (for TRACE)
        #define TRACE  printf
    #endif

    #define UNUSED_PARAMETER(par)

//---------------------------------------------------------------------------
//  definitions for Mitsubishi M16C family for TASKING Compiler CM16
//---------------------------------------------------------------------------

#elif defined (_CM16C)

    #define TARGET_SYSTEM   _NO_OS_
    #define DEV_SYSTEM      _DEV_TASKING_M16C_

    #define NEAR            _near       // variables mapped to internal data storage location
    #define FAR             _far        // variables mapped to external data storage location
    #define CONST           _farrom       // variables mapped to ROM (i.e. flash)
    #define ROM_INIT                    // variables will be initialized directly in ROM (means no copy from RAM in startup)
    #define ROM                         // code or variables mapped to ROM (i.e. flash)
                                        // usage: CONST BYTE ROM foo = 0x00;
    #define HWACC           _near       // hardware access through external memory (i.e. CAN)
    #define LARGE                       // functions set parameters to external data storage location

    // These types can be adjusted by users to match application requirements. The goal is to
    // minimize code memory and maximize speed.
    #define GENERIC         _far        // generic pointer to point to application data
                                        // Variables with this attribute can be located in external
                                        // or internal data memory.
                                        // do you use memory model SMALL, than you have to set _far
    #define MEM                         // Memory attribute to optimize speed and code of pointer access.

    #define REENTRANT
    #define PUBLIC

    // Stdio.h has to be alway included here. If printf() is used stdio.h defines NULL
    // without checking if it is already included. So an error occurs while compiling.
    // (r.d.)
    #include <stdio.h>                  // prototype printf() (for TRACE)
    #ifndef NDEBUG
        #define TRACE  printf
    #endif

    #define UNUSED_PARAMETER(par)

//---------------------------------------------------------------------------
//  definitions for Mitsubishi M16C family for Mitsubishi Compiler NC30
//---------------------------------------------------------------------------
// name NC30, andere Form will der Compiler nicht !!
#elif defined (NC30)

    #define TARGET_SYSTEM   _NO_OS_
    #define DEV_SYSTEM      _DEV_MITSUBISHI_M16C_

    #define NEAR            near        // variables mapped to internal data storage location
    #define FAR             far         // variables mapped to external data storage location
    #define CONST           const       // variables mapped to ROM (i.e. flash)
    #define ROM_INIT                    // variables will be initialized directly in ROM (means no copy from RAM in startup)
    #define ROM                      // code or variables mapped to ROM (i.e. flash)
                                        // usage: CONST BYTE ROM foo = 0x00;
    #define HWACC           near        // hardware access through external memory (i.e. CAN)
    #define LARGE                       // functions set parameters to external data storage location

    // These types can be adjusted by users to match application requirements. The goal is to
    // minimize code memory and maximize speed.
    #define GENERIC         far         // generic pointer to point to application data
                                        // Variables with this attribute can be located in external
                                        // or internal data memory.
    #define MEM                         // Memory attribute to optimize speed and code of pointer access.

    #define REENTRANT
    #define PUBLIC

    #ifndef NDEBUG
        #include <stdio.h>                  // prototype printf() (for TRACE)
        #define TRACE  printf
    #endif

    #define UNUSED_PARAMETER(par)
//---------------------------------------------------------------------------
//  definitions for Renesas M32C family for Renesas Compiler
//---------------------------------------------------------------------------
#elif defined (NC308)

    #define TARGET_SYSTEM   _NO_OS_
    #define DEV_SYSTEM      _DEV_RENESAS_M32C_

    #define NEAR             near       // variables mapped to internal data storage location
    #define FAR              far        // variables mapped to external data storage location
    #define CONST            const      // variables mapped to ROM (i.e. flash)
    #define ROM_INIT                    // variables will be initialized directly in ROM (means no copy from RAM in startup)
    #define ROM                         // code or variables mapped to ROM (i.e. flash)
    #define HWACC                       // hardware access through external memory (i.e. CAN)
    #define LARGE                       // functions set parameters to external data storage location

    // These types can be adjusted by users to match application requirements. The goal is to
    // minimize code memory and maximize speed.
    #define GENERIC                     // generic pointer to point to application data
                                        // Variables with this attribute can be located in external
                                        // or internal data memory.
    #define MEM              far        // Memory attribute to optimize speed and code of pointer access.

    #define REENTRANT
    #define PUBLIC

    #ifndef NDEBUG
        #include <stdio.h>                  // prototype printf() (for TRACE)
        #define TRACE  printf
    #endif

    #define UNUSED_PARAMETER(par)

//---------------------------------------------------------------------------
//  definitions for ARM7 family with GNU compiler
//---------------------------------------------------------------------------

#elif defined(__GNUC__) && (defined(__arm__) || defined(__thumb__)) && !defined(__LINUX_ARM_ARCH__)

#ifdef __ECOS__
    #define TARGET_SYSTEM   _ECOSPRO_
    #define DEV_SYSTEM      _DEV_GNU_ARM7_
#else
    #define TARGET_SYSTEM   _NO_OS_
    #define DEV_SYSTEM      _DEV_GNU_ARM7_
#endif

    #define NEAR                        // variables mapped to internal data storage location
    #define FAR                         // variables mapped to external data storage location
    #define CONST           const       // variables mapped to ROM (i.e. flash)
    #define ROM_INIT                    // variables will be initialized directly in ROM (means no copy from RAM in startup)
    #define ROM                         // code or variables mapped to ROM (i.e. flash)
                                        // usage: CONST BYTE ROM foo = 0x00;
    #define HWACC                       // hardware access through external memory (i.e. CAN)
    #define LARGE                       // functions set parameters to external data storage location

    // These types can be adjusted by users to match application requirements. The goal is to
    // minimize code memory and maximize speed.
    #define GENERIC                     // generic pointer to point to application data
                                        // Variables with this attribute can be located in external
                                        // or internal data memory.
    #define MEM                         // Memory attribute to optimize speed and code of pointer access.
    #define HWACC                       // hardware access through external memory (i.e. CAN)

    #define REENTRANT
    #define PUBLIC

    #ifndef NO_QWORD
    #ifndef QWORD
        #define QWORD long long    // i.A. durch Herr Kuschel
    #endif
    #endif

    #ifndef NDEBUG
        #ifdef __ECOS__
            //#include <cyg/infra/diag.h>
            //#define TRACE  printf
            //#define TRACE  diag_printf
             extern int TgtPrintf (const char *format, ...);
             #define TRACE  TgtPrintf
       #else
        #include <stdio.h>                  // prototype printf() (for TRACE)
        #define TRACE  printf
        #endif
    #endif

    #define UNUSED_PARAMETER(par)


//---------------------------------------------------------------------------
//  definitions for WinAVR compiler e.g. for Atmel's AT90CAN128
//---------------------------------------------------------------------------

#elif defined(__GNUC__) && defined (__AVR__) // NOTE: has to be defined before __GNUC__ section!

    #define TARGET_SYSTEM           _NO_OS_
    #define DEV_SYSTEM              _DEV_GNU_WINAVR8_

    //#include <avr/pgmspace.h>
    #define ROM_INIT __attribute__((__progmem__))   // variables will be initialized directly in ROM (means no copy from RAM in startup)
    //#define ROM_INIT                // variables will be initialized directly in ROM (means no copy from RAM in startup)
    #define ROM                     // code or variables mapped to ROM (i.e. flash)
    #define HWACC                   // hardware access through external memory (i.e. CAN)
    #define MEM                     // Memory attribute to optimize speed and code of pointer access.
    #define NEAR                    // variables mapped to internal data storage location
    #define FAR                     // variables mapped to external data storage location
    #define CONST const             // variables mapped to ROM (i.e. flash)
    #define LARGE

    // These types can be adjusted by users to match application requirements. The goal is to
    // minimize code memory and maximize speed.
    #define GENERIC                 // generic pointer to point to application data
                                    // Variables with this attribute can be located in external
                                    // or internal data memory.

    #define REENTRANT
    #define PUBLIC

    // note: WinAVR is supports 64 bit variables but it needs to much memory resources!
    // Thats why NO_QWORD should be set in project settings!
    #ifndef NO_QWORD
    #ifndef QWORD
        #define QWORD unsigned long long int
    #endif
    #endif

    #ifndef NDEBUG
        #include <stdio.h>              // prototype printf() (for TRACE)
        //#define TRACE  printf
        #define TRACE(arg, ...)         do { static char __s[] ROM_INIT = (arg); \
                                        printf_P(__s, ## __VA_ARGS__); } while (0)
    #endif

    #define UNUSED_PARAMETER(par)


//---------------------------------------------------------------------------
//  definitions for Motorola PowerPC family 5x5 (555/565)
//  definitions Linux-PC
//---------------------------------------------------------------------------

#elif defined (__GNUC__)

    #if defined (LINUX) || defined (linux) || defined (__linux__)
        #define LINUX_SYSTEM            // define 'LINUX_SYSTEM' uniform for all Linux based systems
        // r.d.: We will need an other solution here! There are two sections here which do check the preproc-definitions:
        //     LINUX and __linux__ . The first one was Linux for PC, the second one is this section for embedded Linux (MCF5xxx).
        //     But Linux for PC does not need the definitions for embedded Linux.
    #endif

    // GNU C compiler supports function inlining
    #define INLINE_FUNCTION_DEF extern inline

    // to actually enable inlining just include the following three lines
    // #undef INLINE_FUNCTION
    // #define INLINE_FUNCTION     INLINE_FUNCTION_DEF
    // #define INLINE_ENABLED      TRUE

    #ifdef PXROS
        #define TARGET_SYSTEM       _PXROS_
        #ifdef __i386__
            #undef LINUX // this define seems to be set from compiler
            #define DEV_SYSTEM      _DEV_HIGHTEC_X86_
	    #elif defined (__tricore__)
            #define DEV_SYSTEM      _DEV_GNU_TRICORE_
        #else // MPC5x5
            #define DEV_SYSTEM      _DEV_GNU_MPC5X5_
        #endif

    #elif defined (LINUX) || defined (__linux__)
        #define TARGET_SYSTEM       _LINUX_     // Linux definition
        #define DEV_SYSTEM          _DEV_LINUX_

    #elif defined (GNU_CF5282)
        #define TARGET_SYSTEM       _NO_OS_
        #define DEV_SYSTEM          _DEV_GNU_CF5282_

    #elif defined (ECOSPRO_I386_PEAK_PCI)
        #define TARGET_SYSTEM       _ECOSPRO_
        #define DEV_SYSTEM          _DEV_GNU_I386_

    #elif defined (GNU_CF548X)
        #define TARGET_SYSTEM       _NO_OS_
        #define DEV_SYSTEM          _DEV_GNU_CF548X_

    #elif defined (__NIOS2__)
        #define TARGET_SYSTEM       _NO_OS_
        #define DEV_SYSTEM          _DEV_NIOS2_

    #elif defined (__MICROBLAZE__)
        #define TARGET_SYSTEM       _NO_OS_
        #define DEV_SYSTEM          _DEV_MICROBLAZE_

    #else
        #error 'ERROR: DEV_SYSTEM not found!'
    #endif

    #ifndef NO_QWORD
    #ifndef QWORD
        #define QWORD long long int
    #endif
    #endif

    #if (TARGET_SYSTEM == _PXROS_)

        #ifndef __KERNEL__
            #include <string.h>
        #endif


        #define NEAR                        // variables mapped to internal data storage location
        #define FAR                         // variables mapped to external data storage location
        #define CONST           const       // variables mapped to ROM (i.e. flash)
        #define ROM_INIT                    // variables will be initialized directly in ROM (means no copy from RAM in startup)
        #define ROM     /* to be defined */ // code or variables mapped to ROM (i.e. flash)
                                            // usage: CONST BYTE ROM foo = 0x00;
        #define LARGE                       // functions set parameters to external data storage location

        // These types can be adjusted by users to match application requirements. The goal is to
        // minimize code memory and maximize speed.
        #define GENERIC                     // generic pointer to point to application data
                                            // Variables with this attribute can be located in external
                                            // or internal data memory.
        #define MEM                         // Memory attribute to optimize speed and code of pointer access.

        #define HWACC                       // hardware access through external memory (i.e. CAN)

        #define REENTRANT
        #define PUBLIC

        #ifndef NO_QWORD
        #ifndef QWORD
            #define QWORD long long int
        #endif
        #endif

        #ifndef NDEBUG
            #include <stdio.h>              // prototype printf() (for TRACE)
            #define TRACE  printf
        #endif

        #define UNUSED_PARAMETER(par)

    #endif

    // ------------------ GNUC for I386 ---------------------------------------------

    #if (TARGET_SYSTEM == _LINUX_) || (TARGET_SYSTEM == _ECOSPRO_)

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
                #define TRACE  printf
            #else
                #define TRACE  printk
            #endif
        #endif

        #define UNUSED_PARAMETER(par)

    #endif

    // ------------------ GNU without OS ---------------------------------------------

    #if (TARGET_SYSTEM == _NO_OS_)

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
//            #include "xuartdrv.h"
            #include <stdio.h>              // prototype printf() (for TRACE)
            #define TRACE  printf
//            #define TRACE  mprintf
//            #ifndef TRACE
//                #define TRACE trace
//                void trace (char *fmt, ...);
//            #endif
        #endif

        #define UNUSED_PARAMETER(par)

    #endif

//---------------------------------------------------------------------------
//  definitions for MPC565
//---------------------------------------------------------------------------
#elif defined (__MWERKS__)

#ifdef __MC68K__

    #define TARGET_SYSTEM = _MCF548X_
    #define DEV_SYSTEM      _DEV_MCW_MCF5XXX_

#else
    #define TARGET_SYSTEM = _MPC565_
    #define DEV_SYSTEM      _DEV_MCW_MPC5X5_
#endif

    #define NEAR                        // variables mapped to internal data storage location
    #define FAR                         // variables mapped to external data storage location
    #define CONST           const       // variables mapped to ROM (i.e. flash)
    #define ROM_INIT                    // variables will be initialized directly in ROM (means no copy from RAM in startup)
    #define ROM                         // code or variables mapped to ROM (i.e. flash)
                                        // usage: CONST BYTE ROM foo = 0x00;
    #define LARGE                       // functions set parameters to external data storage location

    // These types can be adjusted by users to match application requirements. The goal is to
    // minimize code memory and maximize speed.
    #define GENERIC                     // generic pointer to point to application data
                                        // Variables with this attribute can be located in external
                                        // or internal data memory.
    #define MEM                         // Memory attribute to optimize speed and code of pointer access.

    #define HWACC                       // hardware access through external memory (i.e. CAN)

    #define REENTRANT
    #define PUBLIC

    #ifndef NDEBUG
        #include <stdio.h>              // prototype printf() (for TRACE)
        #define TRACE  printf
    #endif

    #define UNUSED_PARAMETER(par)

//---------------------------------------------------------------------------
//  definitions for BECK 1x3
//---------------------------------------------------------------------------
#elif defined (__BORLANDC__) && defined (__PARADIGM__)


    #define TARGET_SYSTEM      _NO_OS_
    #define DEV_SYSTEM         _DEV_PAR_BECK1X3_



    #define ROM_INIT                    // variables will be initialized directly in ROM (means no copy from RAM in startup)
    #define ROM                         // code or variables mapped to ROM (i.e. flash)
                                        // usage: CONST BYTE ROM foo = 0x00;
    #define HWACC                       // hardware access through external memory (i.e. CAN)

    // These types can be adjusted by users to match application requirements. The goal is to
    // minimize code memory and maximize speed.
    #define GENERIC                     // generic pointer to point to application data
                                        // Variables with this attribute can be located in external
                                        // or internal data memory.
    #define MEM                         // Memory attribute to optimize speed and code of pointer access.
    #define NEAR __near                 // variables mapped to internal data storage location
    #define FAR  __far                  // variables mapped to external data storage location
    #define CONST const                 // variables mapped to ROM (i.e. flash)
    #define LARGE

    #define REENTRANT
    #define PUBLIC

    #ifndef NDEBUG
        #ifndef TRACE
            #include <stdio.h>
            #define TRACE printf
        #endif
    #endif

    #define UNUSED_PARAMETER(par)


//---------------------------------------------------------------------------
//  definitions for PC
//---------------------------------------------------------------------------

#elif defined (__BORLANDC__)

    // ------------------ definition target system --------------------------

    #ifdef _WIN32
        #define TARGET_SYSTEM   _WIN32_     // WIN32 definition
        #define DEV_SYSTEM      _DEV_WIN32_
    #else
        #define TARGET_SYSTEM   _DOS_
        #define DEV_SYSTEM      _DEV_BORLAND_DOS_
    #endif


    // ------------------ WIN32 ---------------------------------------------

    #if (TARGET_SYSTEM == _WIN32_)

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

        #ifndef NDEBUG
            #ifndef TRACE
                    #include <stdio.h>
            #define TRACE printf
            #endif
        #endif

        #define UNUSED_PARAMETER(par)

    #elif (TARGET_SYSTEM == _DOS_)

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
        #define NEAR near               // variables mapped to internal data storage location
        #define FAR  far                // variables mapped to external data storage location
        #define CONST const             // variables mapped to ROM (i.e. flash)
        #define LARGE

        #define REENTRANT
        #define PUBLIC

        #ifndef NDEBUG
            #ifndef TRACE
                #include <stdio.h>
                #define TRACE printf
            #endif
        #endif

        #define UNUSED_PARAMETER(par)

    #endif

#elif (defined (_MSC_VER) && (_MSC_VER == 800)) // PC MS Visual C/C++ for DOS applications

    #define TARGET_SYSTEM   _DOS_
    #define DEV_SYSTEM      _DEV_MSVC_DOS_

    #define ROM_INIT                // variables will be initialized directly in ROM (means no copy from RAM in startup)
    #define ROM                     // code or variables mapped to ROM (i.e. flash)
                                    // usage: CONST BYTE ROM foo = 0x00;
    #define HWACC near              // hardware access through external memory (i.e. CAN)

    // These types can be adjusted by users to match application requirements. The goal is to
    // minimize code memory and maximize speed.
    #define GENERIC                 // generic pointer to point to application data
                                    // Variables with this attribute can be located in external
                                    // or internal data memory.
    #define MEM                     // Memory attribute to optimize speed and code of pointer access.
    #define NEAR near               // variables mapped to internal data storage location
    #define FAR  far                // variables mapped to external data storage location
    #define CONST const             // variables mapped to ROM (i.e. flash)
    #define LARGE

    #define REENTRANT
    #define PUBLIC

    #ifndef NDEBUG
        #ifndef TRACE
            #include <stdio.h>
            #define TRACE printf
        #endif
    #endif

    #define UNUSED_PARAMETER(par)

//---------------------------------------------------------------------------
// definitions for RTX under WIN32
//---------------------------------------------------------------------------
#elif (defined (UNDER_RTSS) && defined (WIN32))

    // ------------------ definition target system --------------------------
    #define TARGET_SYSTEM   _WIN32_RTX_
    #define DEV_SYSTEM      _DEV_WIN32_RTX_

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

    #ifndef NDEBUG
        #ifndef TRACE
            #define TRACE RtPrintf
        #endif
    #endif

    #define UNUSED_PARAMETER(par)

//---------------------------------------------------------------------------
// definitions for WinCE
//---------------------------------------------------------------------------
#elif defined (_WIN32_WCE)

    // ------------------ definition target system --------------------------
    #define TARGET_SYSTEM           _WINCE_
    #define DEV_SYSTEM              _DEV_WIN_CE_

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
        #ifndef TRACE
            #define TRACE printf
//            void trace (char *fmt, ...);
        #endif
    #endif

    #define UNUSED_PARAMETER(par)

    #define __func__ __FUNCTION__

#else   // ===> PC MS Visual C/C++

    // ------------------ definition target system --------------------------

    #ifdef _WIN32
        #define TARGET_SYSTEM   _WIN32_     // WIN32 definition
        #define DEV_SYSTEM      _DEV_WIN32_
    #else
        #define TARGET_SYSTEM   _WIN16_     // WIN16 definition
        #define DEV_SYSTEM      _DEV_WIN16_
    #endif



    #define __func__ __FUNCTION__

    // ------------------ WIN16 ---------------------------------------------

    #if (TARGET_SYSTEM == _WIN16_)

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
            #define FAR far             // variables mapped to external data storage location
        #endif

        #ifndef CONST
            #define CONST const         // variables mapped to ROM (i.e. flash)
        #endif

        #define LARGE

        #define REENTRANT
        #define PUBLIC _far _pascal _export

        #ifndef NDEBUG
            #ifndef TRACE
                #define TRACE trace
                #ifdef __cplusplus
                    extern "C"
                    {
                #endif
                    void trace (const char *fmt, ...);
                #ifdef __cplusplus
                    }
                #endif
            #endif
        #endif

        #define UNUSED_PARAMETER(par)

    #endif


    // ------------------ WIN32 ---------------------------------------------

   #if (TARGET_SYSTEM == _WIN32_)

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
            #ifndef TRACE
                #define TRACE trace
                #ifdef __cplusplus
                    extern "C"
                    {
                #endif
                    void trace (const char *fmt, ...);
                #ifdef __cplusplus
                    }
                #endif
            #endif
        #endif

        #define UNUSED_PARAMETER(par) par

        // MS Visual C++ compiler supports function inlining
        #define INLINE_FUNCTION_DEF __forceinline

        // to actually enable inlining just include the following two lines
        // #define INLINE_FUNCTION     INLINE_FUNCTION_DEF
        // #define INLINE_ENABLED      TRUE

    #endif

#endif  // ===> PC


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
//  Definition von TRACE
//---------------------------------------------------------------------------

#ifndef NDEBUG

    #ifndef TRACE
        #define TRACE
    #endif

    #ifndef TRACE0
        #define TRACE0(p0)                      TRACE(p0)
    #endif

    #ifndef TRACE1
        #define TRACE1(p0, p1)                  TRACE(p0, p1)
    #endif

    #ifndef TRACE2
        #define TRACE2(p0, p1, p2)              TRACE(p0, p1, p2)
    #endif

    #ifndef TRACE3
        #define TRACE3(p0, p1, p2, p3)          TRACE(p0, p1, p2, p3)
    #endif

    #ifndef TRACE4
        #define TRACE4(p0, p1, p2, p3, p4)      TRACE(p0, p1, p2, p3, p4)
    #endif

    #ifndef TRACE5
        #define TRACE5(p0, p1, p2, p3, p4, p5)  TRACE(p0, p1, p2, p3, p4, p5)
    #endif

    #ifndef TRACE6
        #define TRACE6(p0, p1, p2, p3, p4, p5, p6)  TRACE(p0, p1, p2, p3, p4, p5, p6)
    #endif

#else

    #ifndef TRACE
        #define TRACE
    #endif

    #ifndef TRACE0
        #define TRACE0(p0)
    #endif

    #ifndef TRACE1
        #define TRACE1(p0, p1)
    #endif

    #ifndef TRACE2
        #define TRACE2(p0, p1, p2)
    #endif

    #ifndef TRACE3
        #define TRACE3(p0, p1, p2, p3)
    #endif

    #ifndef TRACE4
        #define TRACE4(p0, p1, p2, p3, p4)
    #endif

    #ifndef TRACE5
        #define TRACE5(p0, p1, p2, p3, p4, p5)
    #endif

    #ifndef TRACE6
        #define TRACE6(p0, p1, p2, p3, p4, p5, p6)
    #endif

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
                                                PRINTF0 ("Assertion failed: " string);\
                                                for ( ; ; );}
    #else
        #define ASSERTMSG(expr,string)
    #endif
#endif




//---------------------------------------------------------------------------

#endif  // #ifndef _GLOBAL_H_

// EOF

