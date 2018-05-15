/**
********************************************************************************
\file   common/driver-windows.h

\brief  Header file for Windows openPOWERLINK drivers

This file contains the necessary definitions for using the openPOWERLINK
Windows kernel driver.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, Kalycito Infotech Private Limited
Copyright (c) 2016, B&R Industrial Automation GmbH
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
#ifndef _INC_common_driver_windows_H_
#define _INC_common_driver_windows_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#ifndef _KERNEL_MODE
#include <winioctl.h>
#endif /* _KERNEL_MODE */

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define PLK_DEV_FILE      "\\\\.\\plk"
#define PLK_DEV_STRING    L"\\Device\\plk"
#define PLK_LINK_NAME     L"\\DosDevices\\Global\\plk"

//------------------------------------------------------------------------------
//  Commands for <ioctl>
//------------------------------------------------------------------------------

#define PLK_IO_TYPE                    40001

#define PLK_CMD_CTRL_EXECUTE_CMD \
                                CTL_CODE(PLK_IO_TYPE, 0x901, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define PLK_CMD_CTRL_STORE_INITPARAM \
                                CTL_CODE(PLK_IO_TYPE, 0x902, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define PLK_CMD_CTRL_READ_INITPARAM \
                                CTL_CODE(PLK_IO_TYPE, 0x903, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define PLK_CMD_CTRL_GET_STATUS \
                                CTL_CODE(PLK_IO_TYPE, 0x904, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define PLK_CMD_CTRL_GET_HEARTBEAT \
                                CTL_CODE(PLK_IO_TYPE, 0x905, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define PLK_CMD_POST_EVENT      CTL_CODE(PLK_IO_TYPE, 0x906, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define PLK_CMD_GET_EVENT       CTL_CODE(PLK_IO_TYPE, 0x907, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define PLK_CMD_DLLCAL_ASYNCSEND \
                                CTL_CODE(PLK_IO_TYPE, 0x908, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define PLK_CMD_ERRHND_WRITE    CTL_CODE(PLK_IO_TYPE, 0x909, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define PLK_CMD_ERRHND_READ     CTL_CODE(PLK_IO_TYPE, 0x90A, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define PLK_CMD_TIMESYNC_SYNC   CTL_CODE(PLK_IO_TYPE, 0x90B, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define PLK_CMD_PDO_GET_MEM     CTL_CODE(PLK_IO_TYPE, 0x90C, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define PLK_CMD_PDO_FREE_MEM    CTL_CODE(PLK_IO_TYPE, 0x90D, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define PLK_CMD_CLEAN           CTL_CODE(PLK_IO_TYPE, 0x90E, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define PLK_GET_BENCHMARK_BASE  CTL_CODE(PLK_IO_TYPE, 0x90F, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define PLK_FREE_BENCHMARK_BASE CTL_CODE(PLK_IO_TYPE, 0x910, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define PLK_CMD_MAP_MEM         CTL_CODE(PLK_IO_TYPE, 0x911, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define PLK_CMD_UNMAP_MEM       CTL_CODE(PLK_IO_TYPE, 0x912, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define PLK_CMD_CTRL_WRITE_FILE_BUFFER \
                                CTL_CODE(PLK_IO_TYPE, 0x913, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define PLK_CMD_CTRL_GET_FILE_BUFFER_SIZE \
                                CTL_CODE(PLK_IO_TYPE, 0x914, METHOD_BUFFERED, FILE_ANY_ACCESS)
#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
#define PLK_CMD_SOC_GET_MEM     CTL_CODE(PLK_IO_TYPE, 0x915, METHOD_BUFFERED, FILE_ANY_ACCESS)
#endif

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

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

#endif /* _INC_common_driver_windows_H_ */
