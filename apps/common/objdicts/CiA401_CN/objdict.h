/**
********************************************************************************
\file   objdicts/CiA401_CN/objdict.h

\brief  Object dictionary according to CiA401

This file contains the object dictionary definition for the CANopen CiA401
device profile.
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

#define OBD_DEFINE_MACRO
    #include <obdcreate/obdmacro.h>
#undef OBD_DEFINE_MACRO

OBD_BEGIN()
    /*************************************************************************
     * Communication Profile Area (0x1000 - 0x1FFF)
     *************************************************************************/
    OBD_BEGIN_PART_GENERIC()

        // Object 1000h: NMT_DeviceType_U32
        OBD_BEGIN_INDEX_RAM(0x1000, 0x01, FALSE)
            OBD_SUBINDEX_RAM_VAR(0x1000, 0x00, kObdTypeUInt32, kObdAccR, tObdUnsigned32, NMT_DeviceType_U32, 0x000F0191)
        OBD_END_INDEX(0x1000)

        // Object 1001h: ERR_ErrorRegister_U8
        OBD_BEGIN_INDEX_RAM(0x1001, 0x01, FALSE)
            OBD_SUBINDEX_RAM_VAR(0x1001, 0x00, kObdTypeUInt8, kObdAccR, tObdUnsigned8, ERR_ErrorRegister_U8, 0x00)
        OBD_END_INDEX(0x1001)

/*
        // Object 1003h: ERR_History_ADOM
        OBD_RAM_INDEX_RAM_VARARRAY_NOINIT(0x1003, 10, FALSE, tObdDomain, kObdAccR, tObdDomain, ERR_History_ADOM)
*/

        // Object 1006h: NMT_CycleLen_U32 in [us]
        OBD_BEGIN_INDEX_RAM(0x1006, 0x01, FALSE)
            OBD_SUBINDEX_RAM_VAR(0x1006, 0x00, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, NMT_CycleLen_U32, 0x00000000)   // in [us]
        OBD_END_INDEX(0x1006)

        // Object 1008h: NMT_ManufactDevName_VS
        OBD_BEGIN_INDEX_RAM(0x1008, 0x01, FALSE)
           OBD_SUBINDEX_RAM_VSTRING(0x1008, 0x00, kObdAccR, device_name, OBD_MAX_STRING_SIZE, "openPOWERLINK device")
        OBD_END_INDEX(0x1008)

        // Object 1009h: NMT_ManufactHwVers_VS
        OBD_BEGIN_INDEX_RAM(0x1009, 0x01, FALSE)
           OBD_SUBINDEX_RAM_VSTRING(0x1009, 0x00, kObdAccR, hardware_version, OBD_MAX_STRING_SIZE, "1.00")
        OBD_END_INDEX(0x1009)

        // Object 100Ah: NMT_ManufactSwVers_VS
        OBD_BEGIN_INDEX_RAM(0x100A, 0x01, FALSE)
           OBD_SUBINDEX_RAM_VSTRING(0x100A, 0x00, kObdAccR, software_version, OBD_MAX_STRING_SIZE, PLK_PRODUCT_NAME" "PLK_PRODUCT_VERSION)
        OBD_END_INDEX(0x100A)

#if defined(CONFIG_APP_STORE_RESTORE)
        // Object 1010h: NMT_StoreParam_REC
        OBD_BEGIN_INDEX_RAM(0x1010, 0x05, FALSE)
            OBD_SUBINDEX_RAM_VAR(0x1010, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x04)
            OBD_SUBINDEX_RAM_VAR_NOINIT(0x1010, 0x01, kObdTypeUInt32, kObdAccRW, tObdUnsigned32, AllParam_U32)
            OBD_SUBINDEX_RAM_VAR_NOINIT(0x1010, 0x02, kObdTypeUInt32, kObdAccRW, tObdUnsigned32, CommunicationParam_U32)
            OBD_SUBINDEX_RAM_VAR_NOINIT(0x1010, 0x03, kObdTypeUInt32, kObdAccRW, tObdUnsigned32, ApplicationParam_U32)
            OBD_SUBINDEX_RAM_VAR_NOINIT(0x1010, 0x04, kObdTypeUInt32, kObdAccRW, tObdUnsigned32, ManufacturerParam_04h_U32)
        OBD_END_INDEX(0x1010)

        // Object 1011h: NMT_RestoreDefParam_REC
        OBD_BEGIN_INDEX_RAM(0x1011, 0x05, FALSE)
            OBD_SUBINDEX_RAM_VAR(0x1011, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x04)
            OBD_SUBINDEX_RAM_VAR_NOINIT(0x1011, 0x01, kObdTypeUInt32, kObdAccRW, tObdUnsigned32, AllParam_U32)
            OBD_SUBINDEX_RAM_VAR_NOINIT(0x1011, 0x02, kObdTypeUInt32, kObdAccRW, tObdUnsigned32, CommunicationParam_U32)
            OBD_SUBINDEX_RAM_VAR_NOINIT(0x1011, 0x03, kObdTypeUInt32, kObdAccRW, tObdUnsigned32, ApplicationParam_U32)
            OBD_SUBINDEX_RAM_VAR_NOINIT(0x1011, 0x04, kObdTypeUInt32, kObdAccRW, tObdUnsigned32, ManufacturerParam_04h_U32)
        OBD_END_INDEX(0x1011)
#endif

        // Object 1018h: NMT_IdentityObject_REC
        OBD_BEGIN_INDEX_RAM(0x1018, 0x05, FALSE)
            OBD_SUBINDEX_RAM_VAR(0x1018, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x04)
            OBD_SUBINDEX_RAM_VAR(0x1018, 0x01, kObdTypeUInt32, kObdAccR, tObdUnsigned32, VendorId_U32, 0x00000000)
            OBD_SUBINDEX_RAM_VAR(0x1018, 0x02, kObdTypeUInt32, kObdAccR, tObdUnsigned32, ProductCode_U32, 0x00000000)
            OBD_SUBINDEX_RAM_VAR(0x1018, 0x03, kObdTypeUInt32, kObdAccR, tObdUnsigned32, RevisionNo_U32, PLK_DEFINED_OBJ1018_VERSION)
            OBD_SUBINDEX_RAM_VAR(0x1018, 0x04, kObdTypeUInt32, kObdAccR, tObdUnsigned32, SerialNo_U32, 0x00000000)
        OBD_END_INDEX(0x1018)

        // Object 1020h: CFM_VerifyConfiguration_REC
        OBD_BEGIN_INDEX_RAM(0x1020, 0x03, FALSE)
            OBD_SUBINDEX_RAM_VAR(0x1020, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR(0x1020, 0x01, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, ConfDate_U32, 0x00000000)
            OBD_SUBINDEX_RAM_VAR(0x1020, 0x02, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, ConfTime_U32, 0x00000000)
//            OBD_SUBINDEX_RAM_VAR(0x1020, 0x03, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, ConfId_U32, 0x00000000)
//            OBD_SUBINDEX_RAM_VAR(0x1020, 0x04, kObdTypeBool, kObdAccR, tObdBoolean, VerifyConfInvalid_BOOL, 0x01)
        OBD_END_INDEX(0x1020)

        // Object 1030h: NMT_InterfaceGroup_Xh_REC
        OBD_BEGIN_INDEX_RAM(0x1030, 0x0A, FALSE)
            OBD_SUBINDEX_RAM_VAR(0x1030, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x09)
            OBD_SUBINDEX_RAM_VAR_RG(0x1030, 0x01, kObdTypeUInt16, kObdAccR, tObdUnsigned16, InterfaceIndex_U16, 0x01, 0x00, 0x0A)
            OBD_SUBINDEX_RAM_VSTRING(0x1030, 0x02, kObdAccR, InterfaceDescription_VSTR, 0x20, "Interface 1")
            OBD_SUBINDEX_RAM_VAR(0x1030, 0x03, kObdTypeUInt8, kObdAccR, tObdUnsigned8, InterfaceType_U8, 0x06)
            OBD_SUBINDEX_RAM_VAR(0x1030, 0x04, kObdTypeUInt16, kObdAccR, tObdUnsigned16, InterfaceMtu_U16, 1518)
            OBD_SUBINDEX_RAM_OSTRING(0x1030, 0x05, kObdAccR, InterfacePhysAddress_OSTR, 0x06)
            OBD_SUBINDEX_RAM_VSTRING(0x1030, 0x06, kObdAccR, InterfaceName_VSTR, 0x20, "Interface 1")
            OBD_SUBINDEX_RAM_VAR_RG(0x1030, 0x07, kObdTypeUInt8, kObdAccR, tObdUnsigned8, InterfaceOperStatus_U8, 0x01, 0x00, 0x01)
            OBD_SUBINDEX_RAM_VAR_RG(0x1030, 0x08, kObdTypeUInt8, kObdAccGRW, tObdUnsigned8, InterfaceAdminState_U8, 0x01, 0x00, 0x01)
            OBD_SUBINDEX_RAM_VAR(0x1030, 0x09, kObdTypeBool, kObdAccRW, tObdBoolean, Valid_BOOL, 0x01)
        OBD_END_INDEX(0x1030)

#if (defined(CONFIG_DLL_PRES_CHAINING_CN) && (NMT_MAX_NODE_ID > 0))
        // Object 1050h: NMT_RelativeLatencyDiff_AU32
        OBD_RAM_INDEX_RAM_ARRAY(0x1050, NMT_MAX_NODE_ID, FALSE, kObdTypeUInt32, kObdAccR, tObdUnsigned32, NMT_RelativeLatencyDiff_AU32, 0)
#endif

        // Object 1300h: SDO_SequLayerTimeout_U32 in [ms]
        OBD_BEGIN_INDEX_RAM(0x1300, 0x01, FALSE)
            OBD_SUBINDEX_RAM_VAR_RG(0x1300, 0x00, kObdTypeUInt32, kObdAccSGRW, tObdUnsigned32, SDO_SequLayerTimeout_U32, 15000, 100, 0xFFFFFFFF)
        OBD_END_INDEX(0x1300)

        // Object 1400h: PDO_RxCommParam_00h_REC
        OBD_BEGIN_INDEX_RAM(0x1400, 0x03, FALSE)
            OBD_SUBINDEX_RAM_VAR(0x1400, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1400, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1400, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1400)

        // Object 1401h: PDO_RxCommParam_01h_REC
        OBD_BEGIN_INDEX_RAM(0x1401, 0x03, FALSE)
            OBD_SUBINDEX_RAM_VAR(0x1401, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1401, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1401, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1401)

        // Object 1402h: PDO_RxCommParam_02h_REC
        OBD_BEGIN_INDEX_RAM(0x1402, 0x03, FALSE)
            OBD_SUBINDEX_RAM_VAR(0x1402, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1402, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1402, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1402)

        // Object 1600h: PDO_RxMappParam_00h_AU64
        OBD_BEGIN_INDEX_RAM(0x1600, 0x1A, FALSE)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x00, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, NumberOfEntries, 0x00)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x01, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x02, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x03, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x04, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x05, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x06, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x07, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x08, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x09, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x0A, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x0B, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x0C, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x0D, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x0E, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x0F, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x10, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x11, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x12, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x13, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x14, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x15, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x16, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x17, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x18, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x19, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
        OBD_END_INDEX(0x1600)

        // Object 1601h: PDO_RxMappParam_01h_AU64
        OBD_BEGIN_INDEX_RAM(0x1601, 0x1A, FALSE)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x00, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, NumberOfEntries, 0x00)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x01, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x02, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x03, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x04, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x05, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x06, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x07, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x08, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x09, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x0A, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x0B, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x0C, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x0D, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x0E, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x0F, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x10, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x11, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x12, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x13, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x14, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x15, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x16, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x17, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x18, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x19, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
        OBD_END_INDEX(0x1601)

        // Object 1602h: PDO_RxMappParam_02h_AU64
        OBD_BEGIN_INDEX_RAM(0x1602, 0x1A, FALSE)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x00, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, NumberOfEntries, 0x00)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x01, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x02, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x03, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x04, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x05, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x06, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x07, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x08, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x09, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x0A, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x0B, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x0C, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x0D, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x0E, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x0F, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x10, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x11, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x12, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x13, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x14, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x15, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x16, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x17, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x18, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x19, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
        OBD_END_INDEX(0x1602)

        // Object 1800h: PDO_TxCommParam_00h_REC
        OBD_BEGIN_INDEX_RAM(0x1800, 0x03, FALSE)
            OBD_SUBINDEX_RAM_VAR(0x1800, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1800, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1800, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1800)

        // Object 1A00h: PDO_TxMappParam_00h_AU64
        OBD_BEGIN_INDEX_RAM(0x1A00, 0x1A, FALSE)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x00, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, NumberOfEntries, 0x00)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x01, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x02, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x03, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x04, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x05, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x06, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x07, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x08, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x09, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x0A, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x0B, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x0C, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x0D, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x0E, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x0F, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x10, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x11, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x12, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x13, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x14, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x15, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x16, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x17, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x18, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x19, kObdTypeUInt64, kObdAccSRW, tObdUnsigned64, ObjectMapping, 0x0000000000000000LL)
        OBD_END_INDEX(0x1A00)

        // Object 1C0Bh: DLL_CNLossSoC_REC
        OBD_BEGIN_INDEX_RAM(0x1C0B, 0x04, FALSE)
            OBD_SUBINDEX_RAM_VAR(0x1C0B, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 3)
            OBD_SUBINDEX_RAM_USERDEF_NOINIT(0x1C0B, 0x01, kObdTypeUInt32, kObdAccRW, tObdUnsigned32, CumulativeCnt_U32)
            OBD_SUBINDEX_RAM_USERDEF(0x1C0B, 0x02, kObdTypeUInt32, kObdAccR, tObdUnsigned32, ThresholdCnt_U32, 0)
            OBD_SUBINDEX_RAM_USERDEF(0x1C0B, 0x03, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, Threshold_U32, 15)
        OBD_END_INDEX(0x1C0B)

        // Object 1C0Dh: DLL_CNLossPReq_REC
        OBD_BEGIN_INDEX_RAM(0x1C0D, 0x04, FALSE)
            OBD_SUBINDEX_RAM_VAR(0x1C0D, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 3)
            OBD_SUBINDEX_RAM_USERDEF_NOINIT(0x1C0D, 0x01, kObdTypeUInt32, kObdAccRW, tObdUnsigned32, CumulativeCnt_U32)
            OBD_SUBINDEX_RAM_USERDEF(0x1C0D, 0x02, kObdTypeUInt32, kObdAccR, tObdUnsigned32, ThresholdCnt_U32, 0)
            OBD_SUBINDEX_RAM_USERDEF(0x1C0D, 0x03, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, Threshold_U32, 15)
        OBD_END_INDEX(0x1C0D)

        // Object 1C0Fh: DLL_CNCRCError_REC
        OBD_BEGIN_INDEX_RAM(0x1C0F, 0x04, FALSE)
            OBD_SUBINDEX_RAM_VAR(0x1C0F, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 3)
            OBD_SUBINDEX_RAM_USERDEF_NOINIT(0x1C0F, 0x01, kObdTypeUInt32, kObdAccRW, tObdUnsigned32, CumulativeCnt_U32)
            OBD_SUBINDEX_RAM_USERDEF(0x1C0F, 0x02, kObdTypeUInt32, kObdAccR, tObdUnsigned32, ThresholdCnt_U32, 0)
            OBD_SUBINDEX_RAM_USERDEF(0x1C0F, 0x03, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, Threshold_U32, 15)
        OBD_END_INDEX(0x1C0F)

        // Object 1C14h: DLL_LossOfSocTolerance_U32 in [ns]
        OBD_BEGIN_INDEX_RAM(0x1C14, 0x01, FALSE)
            OBD_SUBINDEX_RAM_VAR(0x1C14, 0x00, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, LossOfSocTolerance, 100000)
        OBD_END_INDEX(0x1C14)

#if defined(CONFIG_INCLUDE_IP)
        // Object 1E40h: NWL_IpAddrTable_0h_REC
        OBD_BEGIN_INDEX_RAM(0x1E40, 0x06, FALSE)
            OBD_SUBINDEX_RAM_VAR(0x1E40, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x05)
            OBD_SUBINDEX_RAM_VAR(0x1E40, 0x01, kObdTypeUInt16, kObdAccR, tObdUnsigned16, IfIndex_U16, 0x01)
            OBD_SUBINDEX_RAM_VAR(0x1E40, 0x02, kObdTypeUInt32, kObdAccR, tObdUnsigned32, Addr_IPAD, 0xC0A86401)
            OBD_SUBINDEX_RAM_VAR(0x1E40, 0x03, kObdTypeUInt32, kObdAccR, tObdUnsigned32, NetMask_IPAD, 0xFFFFFF00)
            OBD_SUBINDEX_RAM_VAR(0x1E40, 0x04, kObdTypeUInt16, kObdAccR, tObdUnsigned16, ReasmMaxSize_U16, 50000)
            OBD_SUBINDEX_RAM_VAR(0x1E40, 0x05, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, DefaultGateway_IPAD, 0xC0A864FE)
        OBD_END_INDEX(0x1E40)

        // Object 1E4Ah: NWL_IpGroup_REC
        OBD_BEGIN_INDEX_RAM(0x1E4A, 0x06, FALSE)
            OBD_SUBINDEX_RAM_VAR(0x1E4A, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x03)
            OBD_SUBINDEX_RAM_VAR(0x1E4A, 0x01, kObdTypeBool, kObdAccSRW, tObdBoolean, Forwarding_BOOL, 0x00)
            OBD_SUBINDEX_RAM_VAR(0x1E4A, 0x02, kObdTypeUInt16, kObdAccSRW, tObdUnsigned16, DefaultTTL_U16, 64)
            OBD_SUBINDEX_RAM_VAR(0x1E4A, 0x03, kObdTypeUInt32, kObdAccR, tObdUnsigned32, ForwardDatagrams_U32, 0x00000000)
        OBD_END_INDEX(0x1E4A)
#endif

#if NMT_MAX_NODE_ID > 0
        // Object 1F81h: NMT_NodeAssignment_AU32
        OBD_RAM_INDEX_RAM_ARRAY_ALT(0x1F81, NMT_MAX_NODE_ID, FALSE, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, NMT_NodeAssignment_AU32, 0)
#endif

        // Object 1F82h: NMT_FeatureFlags_U32
        OBD_BEGIN_INDEX_RAM(0x1F82, 0x01, FALSE)
            OBD_SUBINDEX_RAM_VAR(0x1F82, 0x00, kObdTypeUInt32, kObdAccR, tObdUnsigned32, NMT_FeatureFlags_U32, PLK_DEF_FEATURE_FLAGS)
        OBD_END_INDEX(0x1F82)

        // Object 1F83h: NMT_EPLVersion_U8
        OBD_BEGIN_INDEX_RAM(0x1F83, 0x01, FALSE)
            OBD_SUBINDEX_RAM_VAR(0x1F83, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NMT_EPLVersion_U8, 0x20)
        OBD_END_INDEX(0x1F83)

        // Object 1F8Ch: NMT_CurrNMTState_U8
        OBD_BEGIN_INDEX_RAM(0x1F8C, 0x01, FALSE)
            OBD_SUBINDEX_RAM_VAR(0x1F8C, 0x00, kObdTypeUInt8, kObdAccR, tObdUnsigned8, NMT_CurrNMTState_U8, 0x1C)
        OBD_END_INDEX(0x1F8C)

#if NMT_MAX_NODE_ID > 0
        // Object 1F8Dh: NMT_PResPayloadLimitList_AU16
        OBD_RAM_INDEX_RAM_ARRAY_ALT(0x1F8D, NMT_MAX_NODE_ID, FALSE, kObdTypeUInt16, kObdAccSRW, tObdUnsigned16, NMT_PResPayloadLimitList_AU16, 36)
#endif

        // Object 1F93h: NMT_EPLNodeID_REC
        OBD_BEGIN_INDEX_RAM(0x1F93, 0x03, FALSE)
            OBD_SUBINDEX_RAM_VAR(0x1F93, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR(0x1F93, 0x01, kObdTypeUInt8, kObdAccR, tObdUnsigned8, NodeID_U8, 0)
            OBD_SUBINDEX_RAM_VAR(0x1F93, 0x02, kObdTypeBool, kObdAccR, tObdBoolean, NodeIDByHW_BOOL, 0x00)
        OBD_END_INDEX(0x1F93)

        // Object 1F98h: NMT_CycleTiming_REC
#if !defined(CONFIG_DLL_PRES_CHAINING_CN)
        OBD_BEGIN_INDEX_RAM(0x1F98, 0x0A, FALSE)
            OBD_SUBINDEX_RAM_VAR(0x1F98, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x09)
#else
        OBD_BEGIN_INDEX_RAM(0x1F98, 0x0F, FALSE)
            OBD_SUBINDEX_RAM_VAR(0x1F98, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x0E)
#endif
            OBD_SUBINDEX_RAM_VAR(0x1F98, 0x01, kObdTypeUInt16, kObdAccR, tObdUnsigned16, IsochrTxMaxPayload_U16, 0)
            OBD_SUBINDEX_RAM_VAR(0x1F98, 0x02, kObdTypeUInt16, kObdAccR, tObdUnsigned16, IsochrRxMaxPayload_U16, 0)
            OBD_SUBINDEX_RAM_VAR(0x1F98, 0x03, kObdTypeUInt32, kObdAccR, tObdUnsigned32, PResMaxLatency_U32, 0)     // in [ns]
            OBD_SUBINDEX_RAM_VAR(0x1F98, 0x04, kObdTypeUInt16, kObdAccSRW, tObdUnsigned16, PReqActPayloadLimit_U16, 36)
            OBD_SUBINDEX_RAM_VAR(0x1F98, 0x05, kObdTypeUInt16, kObdAccSRW, tObdUnsigned16, PResActPayloadLimit_U16, 36)
            OBD_SUBINDEX_RAM_VAR(0x1F98, 0x06, kObdTypeUInt32, kObdAccR, tObdUnsigned32, ASndMaxLatency_U32, 0)     // in [ns]
            OBD_SUBINDEX_RAM_VAR(0x1F98, 0x07, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MultiplCycleCnt_U8, 0x00)
            OBD_SUBINDEX_RAM_VAR_RG(0x1F98, 0x08, kObdTypeUInt16, kObdAccSGRW, tObdUnsigned16, AsyncMTU_U16, C_DLL_MIN_ASYNC_MTU, C_DLL_MIN_ASYNC_MTU, C_DLL_MAX_ASYNC_MTU)
            OBD_SUBINDEX_RAM_VAR_RG(0x1F98, 0x09, kObdTypeUInt16, kObdAccSRW, tObdUnsigned16, Prescaler_U16, 2, 0, 1000)
#if defined(CONFIG_DLL_PRES_CHAINING_CN)
            OBD_SUBINDEX_RAM_VAR_RG(0x1F98, 0x0A, kObdTypeUInt8, kObdAccGR, tObdUnsigned8, PResMode_U8, 0x00, 0x00, 0x01)
            OBD_SUBINDEX_RAM_VAR(0x1F98, 0x0B, kObdTypeUInt32, kObdAccR, tObdUnsigned32, PResTimeFirst_U32, 0)      // in [ns]
            OBD_SUBINDEX_RAM_VAR(0x1F98, 0x0C, kObdTypeUInt32, kObdAccR, tObdUnsigned32, PResTimeSecond_U32, 0)     // in [ns]
            OBD_SUBINDEX_RAM_VAR(0x1F98, 0x0D, kObdTypeUInt32, kObdAccR, tObdUnsigned32, SyncMNDelayFirst_U32, 0)   // in [ns]
            OBD_SUBINDEX_RAM_VAR(0x1F98, 0x0E, kObdTypeUInt32, kObdAccR, tObdUnsigned32, SyncMNDelaySecond_U32, 0)  // in [ns]
#endif
        OBD_END_INDEX(0x1F98)

        // Object 1F99h: NMT_CNBasicEthernetTimeout_U32 in [us]
        OBD_BEGIN_INDEX_RAM(0x1F99, 0x01, FALSE)
            OBD_SUBINDEX_RAM_VAR(0x1F99, 0x00, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, NMT_CNBasicEthernetTimeout_U32, 5000000)  // in [us]
        OBD_END_INDEX(0x1F99)

#if defined(CONFIG_INCLUDE_IP)
        // Object 1F9Ah: NMT_HostName_VSTR
        OBD_BEGIN_INDEX_RAM(0x1F9A, 0x01, FALSE)
           OBD_SUBINDEX_RAM_VSTRING(0x1F9A, 0x00, kObdAccSRW, NMT_HostName_VSTR, 34, "")
        OBD_END_INDEX(0x1F9A)
#endif

#if NMT_MAX_NODE_ID > 0
        // Object 1F9Bh: NMT_MultiplCycleAssign_AU8
        OBD_RAM_INDEX_RAM_ARRAY_ALT(0x1F9B, NMT_MAX_NODE_ID, FALSE, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, NMT_MultiplCycleAssign_AU8, 0)
#endif

        // Object 1F9Eh: NMT_ResetCmd_U8
        OBD_BEGIN_INDEX_RAM(0x1F9E, 0x01, FALSE)
            OBD_SUBINDEX_RAM_VAR(0x1F9E, 0x00, kObdTypeUInt8, kObdAccRW, tObdUnsigned8, NMT_ResetCmd_U8, 0xFF)
        OBD_END_INDEX(0x1F9E)

    OBD_END_PART()

    /*************************************************************************
     * Manufacturer Specific Profile Area (0x2000 - 0x5FFF)
     *************************************************************************/
    OBD_BEGIN_PART_MANUFACTURER()

    OBD_END_PART()

    /*************************************************************************
     * Standardised Device Profile Area (0x6000 - 0x9FFF)
     *************************************************************************/
    OBD_BEGIN_PART_DEVICE()

        // DigitalInput_00h_AU8
        OBD_BEGIN_INDEX_RAM(0x6000, 0x05, FALSE)
            OBD_SUBINDEX_RAM_VAR(0x6000, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x04)
            OBD_SUBINDEX_RAM_USERDEF(0x6000, 0x01, kObdTypeUInt8, kObdAccVPR, tObdUnsigned8, DigitalInput, 0x00)
            OBD_SUBINDEX_RAM_USERDEF(0x6000, 0x02, kObdTypeUInt8, kObdAccVPR, tObdUnsigned8, DigitalInput, 0x00)
            OBD_SUBINDEX_RAM_USERDEF(0x6000, 0x03, kObdTypeUInt8, kObdAccVPR, tObdUnsigned8, DigitalInput, 0x00)
            OBD_SUBINDEX_RAM_USERDEF(0x6000, 0x04, kObdTypeUInt8, kObdAccVPR, tObdUnsigned8, DigitalInput, 0x00)
        OBD_END_INDEX(0x6000)

        // DigitalOutput_00h_AU8
        OBD_BEGIN_INDEX_RAM(0x6200, 0x05, FALSE)
            OBD_SUBINDEX_RAM_VAR(0x6200, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x04)
            OBD_SUBINDEX_RAM_USERDEF(0x6200, 0x01, kObdTypeUInt8, kObdAccVPRW, tObdUnsigned8, DigitalOutput, 0x00)
            OBD_SUBINDEX_RAM_USERDEF(0x6200, 0x02, kObdTypeUInt8, kObdAccVPRW, tObdUnsigned8, DigitalOutput, 0x00)
            OBD_SUBINDEX_RAM_USERDEF(0x6200, 0x03, kObdTypeUInt8, kObdAccVPRW, tObdUnsigned8, DigitalOutput, 0x00)
            OBD_SUBINDEX_RAM_USERDEF(0x6200, 0x04, kObdTypeUInt8, kObdAccVPRW, tObdUnsigned8, DigitalOutput, 0x00)
        OBD_END_INDEX(0x6200)

        // AnalogueInput_00h_AI8
        OBD_BEGIN_INDEX_RAM(0x6400, 0x05, FALSE)
            OBD_SUBINDEX_RAM_VAR(0x6400, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x04)
            OBD_SUBINDEX_RAM_USERDEF(0x6400, 0x01, kObdTypeInt8, kObdAccVPR, tObdInteger8, AnalogueInput, 0x00)
            OBD_SUBINDEX_RAM_USERDEF(0x6400, 0x02, kObdTypeInt8, kObdAccVPR, tObdInteger8, AnalogueInput, 0x00)
            OBD_SUBINDEX_RAM_USERDEF(0x6400, 0x03, kObdTypeInt8, kObdAccVPR, tObdInteger8, AnalogueInput, 0x00)
            OBD_SUBINDEX_RAM_USERDEF(0x6400, 0x04, kObdTypeInt8, kObdAccVPR, tObdInteger8, AnalogueInput, 0x00)
        OBD_END_INDEX(0x6400)

        // AnalogueInput_00h_AI16
        OBD_BEGIN_INDEX_RAM(0x6401, 0x03, FALSE)
            OBD_SUBINDEX_RAM_VAR(0x6401, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_USERDEF(0x6401, 0x01, kObdTypeInt16, kObdAccVPR, tObdInteger16, AnalogueInput, 0x0000)
            OBD_SUBINDEX_RAM_USERDEF(0x6401, 0x02, kObdTypeInt16, kObdAccVPR, tObdInteger16, AnalogueInput, 0x0000)
        OBD_END_INDEX(0x6401)

        // AnalogueInput_00h_AI32
        OBD_BEGIN_INDEX_RAM(0x6402, 0x02, FALSE)
            OBD_SUBINDEX_RAM_VAR(0x6402, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x01)
            OBD_SUBINDEX_RAM_USERDEF(0x6402, 0x01, kObdTypeInt32, kObdAccVPR, tObdInteger32, AnalogueInput, 0x00000000)
        OBD_END_INDEX(0x6402)

        // AnalogueOutput_00h_AI8
        OBD_BEGIN_INDEX_RAM(0x6410, 0x05, FALSE)
            OBD_SUBINDEX_RAM_VAR(0x6410, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x04)
            OBD_SUBINDEX_RAM_USERDEF(0x6410, 0x01, kObdTypeInt8, kObdAccVPRW, tObdInteger8, AnalogueOutput, 0x00)
            OBD_SUBINDEX_RAM_USERDEF(0x6410, 0x02, kObdTypeInt8, kObdAccVPRW, tObdInteger8, AnalogueOutput, 0x00)
            OBD_SUBINDEX_RAM_USERDEF(0x6410, 0x03, kObdTypeInt8, kObdAccVPRW, tObdInteger8, AnalogueOutput, 0x00)
            OBD_SUBINDEX_RAM_USERDEF(0x6410, 0x04, kObdTypeInt8, kObdAccVPRW, tObdInteger8, AnalogueOutput, 0x00)
        OBD_END_INDEX(0x6410)

        // AnalogueOutput_00h_AI16
        OBD_BEGIN_INDEX_RAM(0x6411, 0x03, FALSE)
            OBD_SUBINDEX_RAM_VAR(0x6411, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_USERDEF(0x6411, 0x01, kObdTypeInt16, kObdAccVPRW, tObdInteger16, AnalogueOutput, 0x0000)
            OBD_SUBINDEX_RAM_USERDEF(0x6411, 0x02, kObdTypeInt16, kObdAccVPRW, tObdInteger16, AnalogueOutput, 0x0000)
        OBD_END_INDEX(0x6411)

        // AnalogueOutput_00h_AI32
        OBD_BEGIN_INDEX_RAM(0x6412, 0x02, FALSE)
            OBD_SUBINDEX_RAM_VAR(0x6412, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x01)
            OBD_SUBINDEX_RAM_USERDEF(0x6412, 0x01, kObdTypeInt32, kObdAccVPRW, tObdInteger32, AnalogueOutput, 0x00000000)
        OBD_END_INDEX(0x6412)

    OBD_END_PART()

OBD_END()

#define OBD_UNDEFINE_MACRO
    #include <obdcreate/obdmacro.h>
#undef OBD_UNDEFINE_MACRO
