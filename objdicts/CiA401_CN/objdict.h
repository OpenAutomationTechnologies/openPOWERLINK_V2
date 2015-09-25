/**
********************************************************************************
\file   objdicts/CiA401_CN/objdict.h

\brief  Object dictionary according to CiA401

This file contains the object dictionary definition for the CANopen CiA401
device profile.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

#define OBD_DEFINE_MACRO
    #include <oplk/obdmacro.h>
#undef OBD_DEFINE_MACRO

OBD_BEGIN ()

    OBD_BEGIN_PART_GENERIC ()

        #include "../generic/objdict_1000-13ff.h"

        // Object 1400h: PDO_RxCommParam_00h_REC
        OBD_BEGIN_INDEX_RAM(0x1400, 0x03, oplk_cbGenericObdAccess)
            OBD_SUBINDEX_RAM_VAR(0x1400, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1400, 0x01, kObdTypeUInt8, kObdAccGRW, tObdUnsigned8, NodeID_U8, 0x00, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1400, 0x02, kObdTypeUInt8, kObdAccRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1400)

        // Object 1401h: PDO_RxCommParam_01h_REC
        OBD_BEGIN_INDEX_RAM(0x1401, 0x03, oplk_cbGenericObdAccess)
            OBD_SUBINDEX_RAM_VAR(0x1401, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1401, 0x01, kObdTypeUInt8, kObdAccGRW, tObdUnsigned8, NodeID_U8, 0x00, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1401, 0x02, kObdTypeUInt8, kObdAccRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1401)

        // Object 1402h: PDO_RxCommParam_02h_REC
        OBD_BEGIN_INDEX_RAM(0x1402, 0x03, oplk_cbGenericObdAccess)
            OBD_SUBINDEX_RAM_VAR(0x1402, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1402, 0x01, kObdTypeUInt8, kObdAccGRW, tObdUnsigned8, NodeID_U8, 0x00, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1402, 0x02, kObdTypeUInt8, kObdAccRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1402)

        // Object 1600h: PDO_RxMappParam_00h_AU64
        OBD_BEGIN_INDEX_RAM(0x1600, 0x1A, oplk_cbGenericObdAccess)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x00, kObdTypeUInt8, kObdAccRW, tObdUnsigned8, NumberOfEntries, 0x00)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x01, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x02, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x03, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x04, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x05, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x06, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x07, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x08, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x09, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x0A, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x0B, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x0C, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x0D, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x0E, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x0F, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x10, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x11, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x12, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x13, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x14, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x15, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x16, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x17, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x18, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x19, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
        OBD_END_INDEX(0x1600)

        // Object 1601h: PDO_RxMappParam_01h_AU64
        OBD_BEGIN_INDEX_RAM(0x1601, 0x1A, oplk_cbGenericObdAccess)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x00, kObdTypeUInt8, kObdAccRW, tObdUnsigned8, NumberOfEntries, 0x00)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x01, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x02, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x03, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x04, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x05, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x06, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x07, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x08, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x09, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x0A, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x0B, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x0C, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x0D, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x0E, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x0F, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x10, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x11, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x12, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x13, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x14, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x15, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x16, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x17, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x18, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x19, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
        OBD_END_INDEX(0x1601)

        // Object 1602h: PDO_RxMappParam_02h_AU64
        OBD_BEGIN_INDEX_RAM(0x1602, 0x1A, oplk_cbGenericObdAccess)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x00, kObdTypeUInt8, kObdAccRW, tObdUnsigned8, NumberOfEntries, 0x00)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x01, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x02, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x03, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x04, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x05, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x06, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x07, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x08, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x09, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x0A, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x0B, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x0C, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x0D, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x0E, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x0F, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x10, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x11, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x12, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x13, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x14, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x15, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x16, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x17, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x18, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x19, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
        OBD_END_INDEX(0x1602)

        // Object 1800h: PDO_TxCommParam_00h_REC
        OBD_BEGIN_INDEX_RAM(0x1800, 0x03, oplk_cbGenericObdAccess)
            OBD_SUBINDEX_RAM_VAR(0x1800, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1800, 0x01, kObdTypeUInt8, kObdAccGRW, tObdUnsigned8, NodeID_U8, 0x00, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1800, 0x02, kObdTypeUInt8, kObdAccRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1800)

        // Object 1A00h: PDO_TxMappParam_00h_AU64
        OBD_BEGIN_INDEX_RAM(0x1A00, 0x1A, oplk_cbGenericObdAccess)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x00, kObdTypeUInt8, kObdAccRW, tObdUnsigned8, NumberOfEntries, 0x0)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x01, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x02, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x03, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x04, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x05, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x06, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x07, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x08, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x09, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x0A, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x0B, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x0C, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x0D, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x0E, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x0F, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x10, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x11, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x12, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x13, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x14, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x15, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x16, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x17, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x18, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x19, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00LL)
        OBD_END_INDEX(0x1A00)

        #include "../generic/objdict_1b00-1fff.h"

    OBD_END_PART ()

    OBD_BEGIN_PART_MANUFACTURER ()

    OBD_END_PART ()

    OBD_BEGIN_PART_DEVICE ()

        // DigitalInput_00h_AU8
        OBD_BEGIN_INDEX_RAM(0x6000, 0x05, NULL)
            OBD_SUBINDEX_RAM_VAR(0x6000, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x04)
            OBD_SUBINDEX_RAM_USERDEF(0x6000, 0x01, kObdTypeUInt8, kObdAccVPR, tObdUnsigned8, DigitalInput, 0x00)
            OBD_SUBINDEX_RAM_USERDEF(0x6000, 0x02, kObdTypeUInt8, kObdAccVPR, tObdUnsigned8, DigitalInput, 0x00)
            OBD_SUBINDEX_RAM_USERDEF(0x6000, 0x03, kObdTypeUInt8, kObdAccVPR, tObdUnsigned8, DigitalInput, 0x00)
            OBD_SUBINDEX_RAM_USERDEF(0x6000, 0x04, kObdTypeUInt8, kObdAccVPR, tObdUnsigned8, DigitalInput, 0x00)
        OBD_END_INDEX(0x6000)

        // DigitalOutput_00h_AU8
        OBD_BEGIN_INDEX_RAM(0x6200, 0x05, NULL)
            OBD_SUBINDEX_RAM_VAR(0x6200, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x04)
            OBD_SUBINDEX_RAM_USERDEF(0x6200, 0x01, kObdTypeUInt8, kObdAccVPRW, tObdUnsigned8, DigitalOutput, 0x00)
            OBD_SUBINDEX_RAM_USERDEF(0x6200, 0x02, kObdTypeUInt8, kObdAccVPRW, tObdUnsigned8, DigitalOutput, 0x00)
            OBD_SUBINDEX_RAM_USERDEF(0x6200, 0x03, kObdTypeUInt8, kObdAccVPRW, tObdUnsigned8, DigitalOutput, 0x00)
            OBD_SUBINDEX_RAM_USERDEF(0x6200, 0x04, kObdTypeUInt8, kObdAccVPRW, tObdUnsigned8, DigitalOutput, 0x00)
        OBD_END_INDEX(0x6200)

        // AnalogueInput_00h_AI8
        OBD_BEGIN_INDEX_RAM(0x6400, 0x05, NULL)
            OBD_SUBINDEX_RAM_VAR(0x6400, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x04)
            OBD_SUBINDEX_RAM_USERDEF(0x6400, 0x01, kObdTypeInt8, kObdAccVPR, tObdInteger8, AnalogueInput, 0x00)
            OBD_SUBINDEX_RAM_USERDEF(0x6400, 0x02, kObdTypeInt8, kObdAccVPR, tObdInteger8, AnalogueInput, 0x00)
            OBD_SUBINDEX_RAM_USERDEF(0x6400, 0x03, kObdTypeInt8, kObdAccVPR, tObdInteger8, AnalogueInput, 0x00)
            OBD_SUBINDEX_RAM_USERDEF(0x6400, 0x04, kObdTypeInt8, kObdAccVPR, tObdInteger8, AnalogueInput, 0x00)
        OBD_END_INDEX(0x6400)

        // AnalogueInput_00h_AI16
        OBD_BEGIN_INDEX_RAM(0x6401, 0x03, NULL)
            OBD_SUBINDEX_RAM_VAR(0x6401, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_USERDEF(0x6401, 0x01, kObdTypeInt16, kObdAccVPR, tObdInteger16, AnalogueInput, 0x0000)
            OBD_SUBINDEX_RAM_USERDEF(0x6401, 0x02, kObdTypeInt16, kObdAccVPR, tObdInteger16, AnalogueInput, 0x0000)
        OBD_END_INDEX(0x6401)

        // AnalogueInput_00h_AI32
        OBD_BEGIN_INDEX_RAM(0x6402, 0x02, NULL)
            OBD_SUBINDEX_RAM_VAR(0x6402, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x01)
            OBD_SUBINDEX_RAM_USERDEF(0x6402, 0x01, kObdTypeInt32, kObdAccVPR, tObdInteger32, AnalogueInput, 0x00000000)
        OBD_END_INDEX(0x6402)

        // AnalogueOutput_00h_AI8
        OBD_BEGIN_INDEX_RAM(0x6410, 0x05, NULL)
            OBD_SUBINDEX_RAM_VAR(0x6410, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x04)
            OBD_SUBINDEX_RAM_USERDEF(0x6410, 0x01, kObdTypeInt8, kObdAccVPRW, tObdInteger8, AnalogueOutput, 0x00)
            OBD_SUBINDEX_RAM_USERDEF(0x6410, 0x02, kObdTypeInt8, kObdAccVPRW, tObdInteger8, AnalogueOutput, 0x00)
            OBD_SUBINDEX_RAM_USERDEF(0x6410, 0x03, kObdTypeInt8, kObdAccVPRW, tObdInteger8, AnalogueOutput, 0x00)
            OBD_SUBINDEX_RAM_USERDEF(0x6410, 0x04, kObdTypeInt8, kObdAccVPRW, tObdInteger8, AnalogueOutput, 0x00)
        OBD_END_INDEX(0x6410)

        // AnalogueOutput_00h_AI16
        OBD_BEGIN_INDEX_RAM(0x6411, 0x03, NULL)
            OBD_SUBINDEX_RAM_VAR(0x6411, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_USERDEF(0x6411, 0x01, kObdTypeInt16, kObdAccVPRW, tObdInteger16, AnalogueOutput, 0x0000)
            OBD_SUBINDEX_RAM_USERDEF(0x6411, 0x02, kObdTypeInt16, kObdAccVPRW, tObdInteger16, AnalogueOutput, 0x0000)
        OBD_END_INDEX(0x6411)

        // AnalogueOutput_00h_AI32
        OBD_BEGIN_INDEX_RAM(0x6412, 0x02, NULL)
            OBD_SUBINDEX_RAM_VAR(0x6412, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x01)
            OBD_SUBINDEX_RAM_USERDEF(0x6412, 0x01, kObdTypeInt32, kObdAccVPRW, tObdInteger32, AnalogueOutput, 0x00000000)
        OBD_END_INDEX(0x6412)

    OBD_END_PART ()

OBD_END ()


#define OBD_UNDEFINE_MACRO
    #include <oplk/obdmacro.h>
#undef OBD_UNDEFINE_MACRO

