/**
********************************************************************************
\file   firmwareinfodecode.h

\brief  Header file of the firmware info decode module

This header file contains the definitions of the firware info decoe module.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#ifndef _INC_firmwareinfodecode_H_
#define _INC_firmwareinfodecode_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <firmwaremanager/firmwaremanager.h>
#include <firmwaremanager/firmwarestore.h>

#include <oplk/oplk.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

/**
\brief Information structure representing a module
 */
typedef struct
{
    UINT8   nodeId;     ///< ID of the node
    UINT32  vendorId;   ///< Vendor ID of the module
    UINT32  productId;  ///< Product ID of the module
    UINT32  hwVariant;  ///< Hardware variant of the module
} tFirmwareModuleInfo;

/**
\brief Structure containing information about the configured firmware
 */
typedef struct
{
    tFirmwareModuleInfo     moduleInfo;         ///< Info of the associated module
    UINT32                  appSwDate;          ///< Software date of the configured firmware
    UINT32                  appSwTime;          ///< Software time of the configured firmware
    BOOL                    fFirmwareLocked;    ///< Flag for representation if the firmware
                                                ///< should be locked, this functionality is
                                                ///< currently not supported.
    tFirmwareStoreHandle    pFwImage;           ///< Handle of the firmware store instance for
                                                ///< accessing the firmware
} tFirmwareInfo;

/**
\brief List entry for a firmware info
 */
typedef struct tFirmwareInfoEntry
{
    tFirmwareInfo               fwInfo; ///< Firmware info structure
    struct tFirmwareInfoEntry*  pNext;  ///< Pointer to the next entry within list
} tFirmwareInfoEntry;

/**
\brief List of firmware info entries
 */
typedef tFirmwareInfoEntry* tFirmwareInfoList;
//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C"
{
#endif

tFirmwareRet firmwareinfodecode_decodeInfo(tFirmwareStoreHandle pStore_p,
                                           tFirmwareInfoList* ppInfoList_p);
tFirmwareRet firmwareinfodecode_freeInfo(tFirmwareInfoList pInfoList_p);

#ifdef __cplusplus
}
#endif

#endif /* _INC_firmwareinfodecode_H_ */
