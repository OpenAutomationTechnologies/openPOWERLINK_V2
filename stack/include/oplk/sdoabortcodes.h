/**
********************************************************************************
\file   oplk/sdoabortcodes.h

\brief  Definition of SDO abort codes

This file contains the definitions of the SDO abort codes.
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
#ifndef _INC_oplk_sdoabortcodes_H_
#define _INC_oplk_sdoabortcodes_H_

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

// SDO abort codes
#define SDO_AC_TIME_OUT                             0x05040000L
#define SDO_AC_UNKNOWN_COMMAND_SPECIFIER            0x05040001L
#define SDO_AC_INVALID_BLOCK_SIZE                   0x05040002L
#define SDO_AC_INVALID_SEQUENCE_NUMBER              0x05040003L
#define SDO_AC_OUT_OF_MEMORY                        0x05040005L
#define SDO_AC_UNSUPPORTED_ACCESS                   0x06010000L
#define SDO_AC_READ_TO_WRITE_ONLY_OBJ               0x06010001L
#define SDO_AC_WRITE_TO_READ_ONLY_OBJ               0x06010002L
#define SDO_AC_OBJECT_NOT_EXIST                     0x06020000L
#define SDO_AC_OBJECT_NOT_MAPPABLE                  0x06040041L
#define SDO_AC_PDO_LENGTH_EXCEEDED                  0x06040042L
#define SDO_AC_GEN_PARAM_INCOMPATIBILITY            0x06040043L
#define SDO_AC_INVALID_HEARTBEAT_DEC                0x06040044L
#define SDO_AC_GEN_INTERNAL_INCOMPATIBILITY         0x06040047L
#define SDO_AC_ACCESS_FAILED_DUE_HW_ERROR           0x06060000L
#define SDO_AC_DATA_TYPE_LENGTH_NOT_MATCH           0x06070010L
#define SDO_AC_DATA_TYPE_LENGTH_TOO_HIGH            0x06070012L
#define SDO_AC_DATA_TYPE_LENGTH_TOO_LOW             0x06070013L
#define SDO_AC_SUB_INDEX_NOT_EXIST                  0x06090011L
#define SDO_AC_VALUE_RANGE_EXCEEDED                 0x06090030L
#define SDO_AC_VALUE_RANGE_TOO_HIGH                 0x06090031L
#define SDO_AC_VALUE_RANGE_TOO_LOW                  0x06090032L
#define SDO_AC_MAX_VALUE_LESS_MIN_VALUE             0x06090036L
#define SDO_AC_GENERAL_ERROR                        0x08000000L
#define SDO_AC_DATA_NOT_TRANSF_OR_STORED            0x08000020L
#define SDO_AC_DATA_NOT_TRANSF_DUE_LOCAL_CONTROL    0x08000021L
#define SDO_AC_DATA_NOT_TRANSF_DUE_DEVICE_STATE     0x08000022L
#define SDO_AC_OBJECT_DICTIONARY_NOT_EXIST          0x08000023L
#define SDO_AC_CONFIG_DATA_EMPTY                    0x08000024L

#endif /* _INC_oplk_sdoabortcodes_H_ */
