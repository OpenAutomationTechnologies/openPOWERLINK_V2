/**
********************************************************************************
\file   common/driver.h

\brief  Header file for openPOWERLINK drivers

This file contains the necessary definitions for using the openPOWERLINK
kernel driver modules.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, B&R Industrial Automation GmbH
Copyright (c) 2017, Kalycito Infotech Private Limited
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
#ifndef _INC_common_driver_H_
#define _INC_common_driver_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>
#include <common/dllcal.h>
#include <stddef.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------
typedef struct
{
    tDllCalQueue            queue;
    size_t                  size;
    void*                   pData;
} tIoctlDllCalAsync;

typedef struct
{
    void*                   pData;
    size_t                  size;
} tIoctlBufInfo;

typedef struct
{
    ptrdiff_t               offset;
    UINT32                  errVal;
} tErrHndIoctl;

/**
\brief IOCTL file chunk structure

The structure is used to forward a file chunk to the kernel stack. The descriptor
holds information about the current file chunk and the transfer progress.
*/
typedef struct
{
    tOplkApiFileChunkDesc   desc;           ///< File chunk descriptor
    void*                   pData;          ///< Pointer to file chunk
} tIoctlFileChunk;

/**
\brief PDO memory structure

The structure is used to retrieve the PDO memory allocated by openPOWERLINK
kernel stack and mapped into user virtual address space.
*/
typedef struct
{
    size_t                  memSize;        ///< Size of PDO to be allocated and mapped
    size_t                  pdoMemOffset;   ///< Offset of PDO memory returned by kernel
} tPdoMem;

/**
\brief Benchmark memory structure

The structure is used to retrieve the benchmark port(PIO in FPGA) memory
in the kernel stack. The memory is mapped by the driver in the user virtual
address space before sharing.
*/
typedef struct
{
    void*                   pBaseAddr;      ///< Pointer to the benchmark address returned by kernel
} tBenchmarkMem;

/**
\brief Memory parameters for a mapped memory

The structure contains parameters used to map openPOWERLINK kernel layer
memory into user layer.
*/
typedef struct
{
    void*                   pKernelAddr;    ///< Pointer to the Kernel address
    void*                   pUserAddr;      ///< Pointer to the User address
    size_t                  size;           ///< Size of the shared memory
} tMemStruc;

#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
/**
\brief SoC memory structure

The structure is used to retrieve the SoC memory allocated by openPOWERLINK
kernel stack and mapped into user virtual address space.
*/
typedef struct
{
    size_t                  socMemSize;     ///< Size of SoC memory to be allocated and mapped
    size_t                  socMemOffset;   ///< Offset of SoC memory returned by the kernel
} tSocMem;
#endif

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------
#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __cplusplus
}
#endif

//------------------------------------------------------------------------------
// include architecture specific definitions
//------------------------------------------------------------------------------
#if (TARGET_SYSTEM == _LINUX_)
#if defined (__LINUX_PCIE__) || defined(__LINUX_ZYNQ__)
#include <common/driver-linuxdpshm.h>
#else /* __LINUX_PCIE__ */
#include <common/driver-linux.h>
#endif /* __LINUX_PCIE__ */
#elif (TARGET_SYSTEM == _WIN32_)
#include <common/driver-windows.h>
#endif /* (TARGET_SYSTEM == _WIN32_) */

#endif /* _INC_common_driver_H_ */
