/**
********************************************************************************
\file   oplk/obdal.h

\brief  Header file for object dictionary abstraction layer

This file provides the interface functions and constants for the abstracted
object dictionary access.
*******************************************************************************/

/*------------------------------------------------------------------------------
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
#ifndef _INC_oplk_obdal_H_
#define _INC_oplk_obdal_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <oplk/oplkinc.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------
/** \brief Data type for handle between SDO Command Layer and application */
typedef UINT tObdAlComConHdl;

/**
\brief Enumeration for OD abstraction layer call source modules

This enumeration lists all valid OD abstraction layer caller modules.
*/
typedef enum
{
    kObdAlOriginSdo             = 0x00,     ///< Caller of obdal is the SDO module
} eObdAlOrigin;

/**
\brief ObdAlOrigin data type

Data type for the enumerator \ref eObdAlOrigin.
*/
typedef UINT8 tObdAlOrigin;

/**
\brief Enumeration for OD abstraction layer access type

This enumeration lists all valid OD abstraction layer access types.
*/
typedef enum
{
    kObdAlAccessTypeRead        = 0x00,     ///< object read access
    kObdAlAccessTypeWrite       = 0x01,     ///< object write access
} eObdAlAccessType;

/**
\brief ObdAlAccessType data type

Data type for the enumerator \ref eObdAlAccessType.
*/
typedef UINT8 tObdAlAccessType;

/**
\brief Structure for OD abstraction layer connection to object dictionary

This structure is used to transfer data from the OD abstraction layer to the
user defined object dictionary and vice versa.
*/
typedef struct
{
    UINT                index;          ///< Index to read/write
    UINT                subIndex;       ///< Sub-index to read/write
    tObdAlAccessType    accessTyp;      ///< Object access type (read/write)
    void*               pSrcData;       ///< Pointer to data which should be transferred
    void*               pDstData;       /**< Pointer to storage destination for immediate
                                             read access response */
    size_t              totalPendSize;  /**< Total pending transfer size. For read access,
                                             this value needs to be set by user OD if
                                             dataOffset is 0. */
    size_t              dataSize;       /**< Write access: Current segment size, if segmented, otherwise 0.
                                             Read access[in]: Buffer size for pDstData
                                             Read access[out]: Size of copied data
                                             (or to be copied data for late response) */
    size_t              dataOffset;     /**< Segment offset in whole data block.
                                             If 0, its the initial segment. */
    tObdAlComConHdl     obdAlHdl;       /**< Handle to OD abstraction layer connection,
                                             generated by the OD abstraction layer */
    tOplkError          plkError;       ///< Error signaling to and from OD abstraction layer
    tObdAlOrigin        origin;         ///< Originating module of this OD access
} tObdAlConHdl;

#endif /* _INC_oplk_obdal_H_ */
