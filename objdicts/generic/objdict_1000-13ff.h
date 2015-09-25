/**
********************************************************************************
\file   objdicts/generic/objdict_1000-13ff.h

\brief  OD definitions for generic communication area 0x1000 - 0x13FF

This file contains the object dictionary definition for the generic communication
area from 0x1000 - 0x13FF.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

        // Object 1000h: NMT_DeviceType_U32
        OBD_BEGIN_INDEX_RAM(0x1000, 0x01, NULL)
            OBD_SUBINDEX_RAM_VAR(0x1000, 0x00, kObdTypeUInt32, kObdAccR, tObdUnsigned32, NMT_DeviceType_U32, 0xF0191)
        OBD_END_INDEX(0x1000)

        // Object 1001h : ERR_ErrorRegister_U8
        OBD_BEGIN_INDEX_RAM(0x1001, 0x01, NULL)
            OBD_SUBINDEX_RAM_VAR(0x1001, 0x00, kObdTypeUInt8, kObdAccR, tObdUnsigned8, ERR_ErrorRegister_U8, 0x00)
        OBD_END_INDEX(0x1001)

/*
        // Object 1003h: ERR_History_ADOM
        OBD_RAM_INDEX_RAM_VARARRAY_NOINIT(0x1003, 10, NULL, tObdDomain, kObdAccR, tObdDomain, ERR_History_ADOM)
*/

        // Object 1006h: NMT_CycleLen_U32 in [us]
        OBD_BEGIN_INDEX_RAM(0x1006, 0x01, NULL)
            OBD_SUBINDEX_RAM_VAR(0x1006, 0x00, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, NMT_CycleLen_U32, 0x00)   // in [us]
        OBD_END_INDEX(0x1006)

        // Object 1008h: NMT_ManufactDevName_VS
        OBD_BEGIN_INDEX_RAM(0x1008, 0x01, NULL)
           OBD_SUBINDEX_RAM_VSTRING(0x1008, 0x00, kObdAccR, device_name, OBD_MAX_STRING_SIZE, "openPOWERLINK device")
        OBD_END_INDEX(0x1008)

        // Object 1009h: NMT_ManufactHwVers_VS
        OBD_BEGIN_INDEX_RAM(0x1009, 0x01, NULL)
           OBD_SUBINDEX_RAM_VSTRING(0x1009, 0x00, kObdAccR, hardware_version, OBD_MAX_STRING_SIZE, "1.00")
        OBD_END_INDEX(0x1009)

        // Object 100Ah: NMT_ManufactSwVers_VS
        OBD_BEGIN_INDEX_RAM(0x100A, 0x01, NULL)
           OBD_SUBINDEX_RAM_VSTRING(0x100A, 0x00, kObdAccR, software_version, OBD_MAX_STRING_SIZE, PLK_PRODUCT_NAME" "PLK_PRODUCT_VERSION)
        OBD_END_INDEX(0x100A)

#if (CONFIG_OBD_USE_STORE_RESTORE != FALSE)
        // Object 1010h: NMT_StoreParam_REC
        OBD_BEGIN_INDEX_RAM(0x1010, 0x05, ctrlu_cbObdAccess)
            OBD_SUBINDEX_RAM_VAR(0x1010, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x04)
            OBD_SUBINDEX_RAM_VAR_NOINIT(0x1010, 0x01, kObdTypeUInt32, kObdAccRW, tObdUnsigned32, AllParam_U32)
            OBD_SUBINDEX_RAM_VAR_NOINIT(0x1010, 0x02, kObdTypeUInt32, kObdAccRW, tObdUnsigned32, CommunicationParam_U32)
            OBD_SUBINDEX_RAM_VAR_NOINIT(0x1010, 0x03, kObdTypeUInt32, kObdAccRW, tObdUnsigned32, ApplicationParam_U32)
            OBD_SUBINDEX_RAM_VAR_NOINIT(0x1010, 0x04, kObdTypeUInt32, kObdAccRW, tObdUnsigned32, ManufacturerParam_U32)
        OBD_END_INDEX(0x1010)

        // Object 1011h: NMT_RestoreDefParam_REC
        OBD_BEGIN_INDEX_RAM(0x1011, 0x05, ctrlu_cbObdAccess)
            OBD_SUBINDEX_RAM_VAR(0x1011, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x04)
            OBD_SUBINDEX_RAM_VAR_NOINIT(0x1011, 0x01, kObdTypeUInt32, kObdAccRW, tObdUnsigned32, AllParam_U32)
            OBD_SUBINDEX_RAM_VAR_NOINIT(0x1011, 0x02, kObdTypeUInt32, kObdAccRW, tObdUnsigned32, CommunicationParam_U32)
            OBD_SUBINDEX_RAM_VAR_NOINIT(0x1011, 0x03, kObdTypeUInt32, kObdAccRW, tObdUnsigned32, ApplicationParam_U32)
            OBD_SUBINDEX_RAM_VAR_NOINIT(0x1011, 0x04, kObdTypeUInt32, kObdAccRW, tObdUnsigned32, ManufacturerParam_U32)
        OBD_END_INDEX(0x1011)
#endif

        // Object 1018h: NMT_IdentityObject_REC
        OBD_BEGIN_INDEX_RAM(0x1018, 0x05, NULL)
            OBD_SUBINDEX_RAM_VAR(0x1018, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x04)
            OBD_SUBINDEX_RAM_VAR(0x1018, 0x01, kObdTypeUInt32, kObdAccR, tObdUnsigned32, VendorId_U32, 0x00000000)
            OBD_SUBINDEX_RAM_VAR(0x1018, 0x02, kObdTypeUInt32, kObdAccR, tObdUnsigned32, ProductCode_U32, 0x00000000)
            OBD_SUBINDEX_RAM_VAR(0x1018, 0x03, kObdTypeUInt32, kObdAccR, tObdUnsigned32, RevisionNo_U32, PLK_DEFINED_OBJ1018_VERSION)
            OBD_SUBINDEX_RAM_VAR(0x1018, 0x04, kObdTypeUInt32, kObdAccR, tObdUnsigned32, SerialNo_U32, 0x00000000)
        OBD_END_INDEX(0x1018)

        // Object 1020h: CFM_VerifyConfiguration_REC
        OBD_BEGIN_INDEX_RAM(0x1020, 0x03, ctrlu_cbObdAccess)
            OBD_SUBINDEX_RAM_VAR(0x1020, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR(0x1020, 0x01, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, ConfDate_U32, 0)
            OBD_SUBINDEX_RAM_VAR(0x1020, 0x02, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, ConfTime_U32, 0)
//            OBD_SUBINDEX_RAM_VAR(0x1020, 0x03, kObdTypeUInt32, kObdAccRW, tObdUnsigned32, ConfId_U32, 0)
//            OBD_SUBINDEX_RAM_VAR(0x1020, 0x04, kObdTypeUInt32, kObdAccRW, tObdUnsigned32, VerifyConfInvalid_U32, 1)
        OBD_END_INDEX(0x1020)

        // Object 1030h: NMT_InterfaceGroup_Xh_REC
        OBD_BEGIN_INDEX_RAM(0x1030, 0x0A, NULL)
            OBD_SUBINDEX_RAM_VAR(0x1030, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x09)
            OBD_SUBINDEX_RAM_VAR(0x1030, 0x01, kObdTypeUInt16, kObdAccR, tObdUnsigned16, InterfaceIndex_U16, 0x01)
            OBD_SUBINDEX_RAM_VSTRING(0x1030, 0x02, kObdAccR, InterfaceDescription_VSTR, 0x20, "Interface 1")
            OBD_SUBINDEX_RAM_VAR(0x1030, 0x03, kObdTypeUInt8, kObdAccR, tObdUnsigned8, InterfaceType_U8, 0x06)
            OBD_SUBINDEX_RAM_VAR(0x1030, 0x04, kObdTypeUInt16, kObdAccR, tObdUnsigned16, InterfaceMtu_U16, 1500)
            OBD_SUBINDEX_RAM_OSTRING(0x1030, 0x05, kObdAccR,  InterfacePhysAddress_OSTR, 0x06)
            OBD_SUBINDEX_RAM_VSTRING(0x1030, 0x06, kObdAccR,  InterfaceName_VSTR, 0x20, "Interface 1")
            OBD_SUBINDEX_RAM_VAR(0x1030, 0x07, kObdTypeUInt8, kObdAccR, tObdUnsigned8, InterfaceOperStatus_U8, 0x1)
            OBD_SUBINDEX_RAM_VAR_RG(0x1030, 0x08, kObdTypeUInt8, kObdAccGRW, tObdUnsigned8, InterfaceAdminState_U8, 0x1, 0x0, 0x1)
            OBD_SUBINDEX_RAM_VAR(0x1030, 0x09, kObdTypeBool, kObdAccRW, tObdBoolean, Valid_BOOL, 0x1)
        OBD_END_INDEX(0x1030)

#if ((CONFIG_DLL_PRES_CHAINING_CN != FALSE) && (NMT_MAX_NODE_ID > 0))
        // Object 1050h: NMT_RelativeLatencyDiff_AU32
        OBD_RAM_INDEX_RAM_ARRAY(0x1050, NMT_MAX_NODE_ID, NULL, kObdTypeUInt32, kObdAccR, tObdUnsigned32, NMT_RelativeLatencyDiff_AU32, 0)
#endif

        // Object 1300h: SDO_SequLayerTimeout_U32 in [ms]
        OBD_BEGIN_INDEX_RAM(0x1300, 0x01, NULL)
            OBD_SUBINDEX_RAM_VAR_RG(0x1300, 0x00, kObdTypeUInt32, kObdAccSGRW, tObdUnsigned32, SDO_SequLayerTimeout_U32, 5000, 100, 0xFFFFFFFF)
        OBD_END_INDEX(0x1300)

