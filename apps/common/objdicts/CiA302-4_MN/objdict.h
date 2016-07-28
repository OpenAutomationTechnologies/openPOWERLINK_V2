/**
********************************************************************************
\file   objdicts/CiA302-4_MN/objdict.h

\brief  Object dictionary according to CiA302-4

This file contains the object dictionary definition for the CANopen CiA302-4
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
    #include <obdcreate/obdmacro.h>
#undef OBD_DEFINE_MACRO

OBD_BEGIN()
    /*************************************************************************
     * Communication Profile Area (0x1000 - 0x1FFF)
     *************************************************************************/
    OBD_BEGIN_PART_GENERIC()

        // Object 1000h: NMT_DeviceType_U32
        OBD_BEGIN_INDEX_RAM(0x1000, 0x01, FALSE)
            OBD_SUBINDEX_RAM_VAR(0x1000, 0x00, kObdTypeUInt32, kObdAccR, tObdUnsigned32, NMT_DeviceType_U32, 0x00000000)
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

#if defined(CONFIG_OBD_USE_STORE_RESTORE)
        // Object 1010h: NMT_StoreParam_REC
        OBD_BEGIN_INDEX_RAM(0x1010, 0x05, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1010, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x04)
            OBD_SUBINDEX_RAM_VAR_NOINIT(0x1010, 0x01, kObdTypeUInt32, kObdAccRW, tObdUnsigned32, AllParam_U32)
            OBD_SUBINDEX_RAM_VAR_NOINIT(0x1010, 0x02, kObdTypeUInt32, kObdAccRW, tObdUnsigned32, CommunicationParam_U32)
            OBD_SUBINDEX_RAM_VAR_NOINIT(0x1010, 0x03, kObdTypeUInt32, kObdAccRW, tObdUnsigned32, ApplicationParam_U32)
            OBD_SUBINDEX_RAM_VAR_NOINIT(0x1010, 0x04, kObdTypeUInt32, kObdAccRW, tObdUnsigned32, ManufacturerParam_U32)
        OBD_END_INDEX(0x1010)

        // Object 1011h: NMT_RestoreDefParam_REC
        OBD_BEGIN_INDEX_RAM(0x1011, 0x05, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1011, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x04)
            OBD_SUBINDEX_RAM_VAR_NOINIT(0x1011, 0x01, kObdTypeUInt32, kObdAccRW, tObdUnsigned32, AllParam_U32)
            OBD_SUBINDEX_RAM_VAR_NOINIT(0x1011, 0x02, kObdTypeUInt32, kObdAccRW, tObdUnsigned32, CommunicationParam_U32)
            OBD_SUBINDEX_RAM_VAR_NOINIT(0x1011, 0x03, kObdTypeUInt32, kObdAccRW, tObdUnsigned32, ApplicationParam_U32)
            OBD_SUBINDEX_RAM_VAR_NOINIT(0x1011, 0x04, kObdTypeUInt32, kObdAccRW, tObdUnsigned32, ManufacturerParam_U32)
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
        OBD_BEGIN_INDEX_RAM(0x1020, 0x03, TRUE)
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
        OBD_BEGIN_INDEX_RAM(0x1400, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1400, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1400, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1400, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1400)

        // Object 1401h: PDO_RxCommParam_01h_REC
        OBD_BEGIN_INDEX_RAM(0x1401, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1401, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1401, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1401, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1401)

        // Object 1402h: PDO_RxCommParam_02h_REC
        OBD_BEGIN_INDEX_RAM(0x1402, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1402, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1402, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1402, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1402)

        // Object 1403h: PDO_RxCommParam_03h_REC
        OBD_BEGIN_INDEX_RAM(0x1403, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1403, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1403, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1403, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1403)

        // Object 1404h: PDO_RxCommParam_04h_REC
        OBD_BEGIN_INDEX_RAM(0x1404, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1404, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1404, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1404, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1404)

        // Object 1405h: PDO_RxCommParam_05h_REC
        OBD_BEGIN_INDEX_RAM(0x1405, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1405, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1405, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1405, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1405)

        // Object 1406h: PDO_RxCommParam_06h_REC
        OBD_BEGIN_INDEX_RAM(0x1406, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1406, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1406, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1406, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1406)

        // Object 1407h: PDO_RxCommParam_07h_REC
        OBD_BEGIN_INDEX_RAM(0x1407, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1407, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1407, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1407, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1407)

        // Object 1408h: PDO_RxCommParam_08h_REC
        OBD_BEGIN_INDEX_RAM(0x1408, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1408, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1408, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1408, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1408)

        // Object 1409h: PDO_RxCommParam_09h_REC
        OBD_BEGIN_INDEX_RAM(0x1409, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1409, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1409, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1409, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1409)

        // Object 140Ah: PDO_RxCommParam_0Ah_REC
        OBD_BEGIN_INDEX_RAM(0x140A, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x140A, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x140A, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x140A, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x140A)

        // Object 140Bh: PDO_RxCommParam_0Bh_REC
        OBD_BEGIN_INDEX_RAM(0x140B, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x140B, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x140B, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x140B, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x140B)

        // Object 140Ch: PDO_RxCommParam_0Ch_REC
        OBD_BEGIN_INDEX_RAM(0x140C, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x140C, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x140C, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x140C, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x140C)

        // Object 140Dh: PDO_RxCommParam_0Dh_REC
        OBD_BEGIN_INDEX_RAM(0x140D, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x140D, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x140D, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x140D, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x140D)

        // Object 140Eh: PDO_RxCommParam_0Eh_REC
        OBD_BEGIN_INDEX_RAM(0x140E, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x140E, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x140E, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x140E, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x140E)

        // Object 140Fh: PDO_RxCommParam_0Fh_REC
        OBD_BEGIN_INDEX_RAM(0x140F, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x140F, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x140F, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x140F, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x140F)

        // Object 1410h: PDO_RxCommParam_10h_REC
        OBD_BEGIN_INDEX_RAM(0x1410, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1410, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1410, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1410, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1410)

        // Object 1411h: PDO_RxCommParam_11h_REC
        OBD_BEGIN_INDEX_RAM(0x1411, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1411, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1411, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1411, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1411)

        // Object 1412h: PDO_RxCommParam_12h_REC
        OBD_BEGIN_INDEX_RAM(0x1412, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1412, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1412, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1412, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1412)

        // Object 1413h: PDO_RxCommParam_13h_REC
        OBD_BEGIN_INDEX_RAM(0x1413, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1413, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1413, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1413, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1413)

        // Object 1414h: PDO_RxCommParam_14h_REC
        OBD_BEGIN_INDEX_RAM(0x1414, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1414, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1414, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1414, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1414)

        // Object 1415h: PDO_RxCommParam_15h_REC
        OBD_BEGIN_INDEX_RAM(0x1415, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1415, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1415, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1415, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1415)

        // Object 1416h: PDO_RxCommParam_16h_REC
        OBD_BEGIN_INDEX_RAM(0x1416, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1416, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1416, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1416, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1416)

        // Object 1417h: PDO_RxCommParam_17h_REC
        OBD_BEGIN_INDEX_RAM(0x1417, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1417, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1417, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1417, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1417)

        // Object 1418h: PDO_RxCommParam_18h_REC
        OBD_BEGIN_INDEX_RAM(0x1418, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1418, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1418, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1418, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1418)

        // Object 1419h: PDO_RxCommParam_19h_REC
        OBD_BEGIN_INDEX_RAM(0x1419, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1419, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1419, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1419, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1419)

        // Object 141Ah: PDO_RxCommParam_1Ah_REC
        OBD_BEGIN_INDEX_RAM(0x141A, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x141A, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x141A, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x141A, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x141A)

        // Object 141Bh: PDO_RxCommParam_1Bh_REC
        OBD_BEGIN_INDEX_RAM(0x141B, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x141B, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x141B, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x141B, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x141B)

        // Object 141Ch: PDO_RxCommParam_1Ch_REC
        OBD_BEGIN_INDEX_RAM(0x141C, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x141C, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x141C, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x141C, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x141C)

        // Object 141Dh: PDO_RxCommParam_1Dh_REC
        OBD_BEGIN_INDEX_RAM(0x141D, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x141D, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x141D, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x141D, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x141D)

        // Object 141Eh: PDO_RxCommParam_1Eh_REC
        OBD_BEGIN_INDEX_RAM(0x141E, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x141E, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x141E, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x141E, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x141E)

        // Object 141Fh: PDO_RxCommParam_1Fh_REC
        OBD_BEGIN_INDEX_RAM(0x141F, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x141F, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x141F, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x141F, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x141F)

        // Object 1420h: PDO_RxCommParam_20h_REC
        OBD_BEGIN_INDEX_RAM(0x1420, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1420, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1420, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1420, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1420)

        // Object 1421h: PDO_RxCommParam_21h_REC
        OBD_BEGIN_INDEX_RAM(0x1421, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1421, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1421, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1421, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1421)

        // Object 1422h: PDO_RxCommParam_22h_REC
        OBD_BEGIN_INDEX_RAM(0x1422, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1422, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1422, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1422, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1422)

        // Object 1423h: PDO_RxCommParam_23h_REC
        OBD_BEGIN_INDEX_RAM(0x1423, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1423, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1423, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1423, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1423)

        // Object 1424h: PDO_RxCommParam_24h_REC
        OBD_BEGIN_INDEX_RAM(0x1424, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1424, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1424, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1424, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1424)

        // Object 1425h: PDO_RxCommParam_25h_REC
        OBD_BEGIN_INDEX_RAM(0x1425, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1425, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1425, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1425, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1425)

        // Object 1426h: PDO_RxCommParam_26h_REC
        OBD_BEGIN_INDEX_RAM(0x1426, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1426, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1426, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1426, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1426)

        // Object 1427h: PDO_RxCommParam_27h_REC
        OBD_BEGIN_INDEX_RAM(0x1427, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1427, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1427, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1427, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1427)

        // Object 1600h: PDO_RxMappParam_00h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1600, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1601h: PDO_RxMappParam_01h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1601, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1602h: PDO_RxMappParam_02h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1602, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1603h: PDO_RxMappParam_03h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1603, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1604h: PDO_RxMappParam_04h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1604, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1605h: PDO_RxMappParam_05h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1605, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1606h: PDO_RxMappParam_06h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1606, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1607h: PDO_RxMappParam_07h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1607, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1608h: PDO_RxMappParam_08h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1608, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1609h: PDO_RxMappParam_09h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1609, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 160Ah: PDO_RxMappParam_0Ah_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x160A, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 160Bh: PDO_RxMappParam_0Bh_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x160B, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 160Ch: PDO_RxMappParam_0Ch_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x160C, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 160Dh: PDO_RxMappParam_0Dh_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x160D, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 160Eh: PDO_RxMappParam_0Eh_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x160E, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 160Fh: PDO_RxMappParam_0Fh_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x160F, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1610h: PDO_RxMappParam_10h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1610, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1611h: PDO_RxMappParam_11h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1611, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1612h: PDO_RxMappParam_12h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1612, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1613h: PDO_RxMappParam_13h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1613, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1614h: PDO_RxMappParam_14h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1614, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1615h: PDO_RxMappParam_15h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1615, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1616h: PDO_RxMappParam_16h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1616, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1617h: PDO_RxMappParam_17h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1617, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1618h: PDO_RxMappParam_18h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1618, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1619h: PDO_RxMappParam_19h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1619, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 161Ah: PDO_RxMappParam_1Ah_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x161A, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 161Bh: PDO_RxMappParam_1Bh_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x161B, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 161Ch: PDO_RxMappParam_1Ch_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x161C, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 161Dh: PDO_RxMappParam_1Dh_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x161D, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 161Eh: PDO_RxMappParam_1Eh_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x161E, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 161Fh: PDO_RxMappParam_1Fh_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x161F, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1620h: PDO_RxMappParam_20h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1620, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1621h: PDO_RxMappParam_21h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1621, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1622h: PDO_RxMappParam_22h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1622, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1623h: PDO_RxMappParam_23h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1623, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1624h: PDO_RxMappParam_24h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1624, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1625h: PDO_RxMappParam_25h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1625, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1626h: PDO_RxMappParam_26h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1626, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1627h: PDO_RxMappParam_27h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1627, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1800h: PDO_TxCommParam_00h_REC
        OBD_BEGIN_INDEX_RAM(0x1800, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1800, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1800, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1800, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1800)

        // Object 1801h: PDO_TxCommParam_01h_REC
        OBD_BEGIN_INDEX_RAM(0x1801, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1801, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1801, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1801, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1801)

        // Object 1802h: PDO_TxCommParam_02h_REC
        OBD_BEGIN_INDEX_RAM(0x1802, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1802, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1802, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1802, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1802)

        // Object 1803h: PDO_TxCommParam_03h_REC
        OBD_BEGIN_INDEX_RAM(0x1803, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1803, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1803, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1803, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1803)

        // Object 1804h: PDO_TxCommParam_04h_REC
        OBD_BEGIN_INDEX_RAM(0x1804, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1804, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1804, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1804, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1804)

        // Object 1805h: PDO_TxCommParam_05h_REC
        OBD_BEGIN_INDEX_RAM(0x1805, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1805, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1805, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1805, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1805)

        // Object 1806h: PDO_TxCommParam_06h_REC
        OBD_BEGIN_INDEX_RAM(0x1806, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1806, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1806, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1806, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1806)

        // Object 1807h: PDO_TxCommParam_07h_REC
        OBD_BEGIN_INDEX_RAM(0x1807, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1807, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1807, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1807, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1807)

        // Object 1808h: PDO_TxCommParam_08h_REC
        OBD_BEGIN_INDEX_RAM(0x1808, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1808, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1808, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1808, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1808)

        // Object 1809h: PDO_TxCommParam_09h_REC
        OBD_BEGIN_INDEX_RAM(0x1809, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1809, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1809, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1809, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1809)

        // Object 180Ah: PDO_TxCommParam_0Ah_REC
        OBD_BEGIN_INDEX_RAM(0x180A, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x180A, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x180A, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x180A, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x180A)

        // Object 180Bh: PDO_TxCommParam_0Bh_REC
        OBD_BEGIN_INDEX_RAM(0x180B, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x180B, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x180B, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x180B, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x180B)

        // Object 180Ch: PDO_TxCommParam_0Ch_REC
        OBD_BEGIN_INDEX_RAM(0x180C, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x180C, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x180C, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x180C, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x180C)

        // Object 180Dh: PDO_TxCommParam_0Dh_REC
        OBD_BEGIN_INDEX_RAM(0x180D, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x180D, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x180D, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x180D, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x180D)

        // Object 180Eh: PDO_TxCommParam_0Eh_REC
        OBD_BEGIN_INDEX_RAM(0x180E, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x180E, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x180E, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x180E, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x180E)

        // Object 180Fh: PDO_TxCommParam_0Fh_REC
        OBD_BEGIN_INDEX_RAM(0x180F, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x180F, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x180F, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x180F, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x180F)

        // Object 1810h: PDO_TxCommParam_10h_REC
        OBD_BEGIN_INDEX_RAM(0x1810, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1810, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1810, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1810, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1810)

        // Object 1811h: PDO_TxCommParam_11h_REC
        OBD_BEGIN_INDEX_RAM(0x1811, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1811, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1811, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1811, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1811)

        // Object 1812h: PDO_TxCommParam_12h_REC
        OBD_BEGIN_INDEX_RAM(0x1812, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1812, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1812, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1812, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1812)

        // Object 1813h: PDO_TxCommParam_13h_REC
        OBD_BEGIN_INDEX_RAM(0x1813, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1813, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1813, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1813, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1813)

        // Object 1814h: PDO_TxCommParam_14h_REC
        OBD_BEGIN_INDEX_RAM(0x1814, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1814, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1814, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1814, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1814)

        // Object 1815h: PDO_TxCommParam_15h_REC
        OBD_BEGIN_INDEX_RAM(0x1815, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1815, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1815, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1815, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1815)

        // Object 1816h: PDO_TxCommParam_16h_REC
        OBD_BEGIN_INDEX_RAM(0x1816, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1816, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1816, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1816, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1816)

        // Object 1817h: PDO_TxCommParam_17h_REC
        OBD_BEGIN_INDEX_RAM(0x1817, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1817, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1817, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1817, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1817)

        // Object 1818h: PDO_TxCommParam_18h_REC
        OBD_BEGIN_INDEX_RAM(0x1818, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1818, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1818, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1818, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1818)

        // Object 1819h: PDO_TxCommParam_19h_REC
        OBD_BEGIN_INDEX_RAM(0x1819, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1819, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1819, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1819, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1819)

        // Object 181Ah: PDO_TxCommParam_1Ah_REC
        OBD_BEGIN_INDEX_RAM(0x181A, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x181A, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x181A, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x181A, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x181A)

        // Object 181Bh: PDO_TxCommParam_1Bh_REC
        OBD_BEGIN_INDEX_RAM(0x181B, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x181B, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x181B, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x181B, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x181B)

        // Object 181Ch: PDO_TxCommParam_1Ch_REC
        OBD_BEGIN_INDEX_RAM(0x181C, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x181C, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x181C, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x181C, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x181C)

        // Object 181Dh: PDO_TxCommParam_1Dh_REC
        OBD_BEGIN_INDEX_RAM(0x181D, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x181D, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x181D, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x181D, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x181D)

        // Object 181Eh: PDO_TxCommParam_1Eh_REC
        OBD_BEGIN_INDEX_RAM(0x181E, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x181E, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x181E, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x181E, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x181E)

        // Object 181Fh: PDO_TxCommParam_1Fh_REC
        OBD_BEGIN_INDEX_RAM(0x181F, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x181F, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x181F, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x181F, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x181F)

        // Object 1820h: PDO_TxCommParam_20h_REC
        OBD_BEGIN_INDEX_RAM(0x1820, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1820, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1820, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1820, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1820)

        // Object 1821h: PDO_TxCommParam_21h_REC
        OBD_BEGIN_INDEX_RAM(0x1821, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1821, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1821, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1821, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1821)

        // Object 1822h: PDO_TxCommParam_22h_REC
        OBD_BEGIN_INDEX_RAM(0x1822, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1822, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1822, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1822, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1822)

        // Object 1823h: PDO_TxCommParam_23h_REC
        OBD_BEGIN_INDEX_RAM(0x1823, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1823, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1823, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1823, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1823)

        // Object 1824h: PDO_TxCommParam_24h_REC
        OBD_BEGIN_INDEX_RAM(0x1824, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1824, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1824, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1824, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1824)

        // Object 1825h: PDO_TxCommParam_25h_REC
        OBD_BEGIN_INDEX_RAM(0x1825, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1825, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1825, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1825, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1825)

        // Object 1826h: PDO_TxCommParam_26h_REC
        OBD_BEGIN_INDEX_RAM(0x1826, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1826, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1826, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1826, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1826)

        // Object 1827h: PDO_TxCommParam_27h_REC
        OBD_BEGIN_INDEX_RAM(0x1827, 0x03, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1827, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR_RG(0x1827, 0x01, kObdTypeUInt8, kObdAccSGRW, tObdUnsigned8, NodeID_U8, 0, 0, 254)
            OBD_SUBINDEX_RAM_VAR(0x1827, 0x02, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1827)

        // Object 1A00h: PDO_TxMappParam_00h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1A00, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1A01h: PDO_TxMappParam_01h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1A01, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1A02h: PDO_TxMappParam_02h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1A02, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1A03h: PDO_TxMappParam_03h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1A03, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1A04h: PDO_TxMappParam_04h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1A04, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1A05h: PDO_TxMappParam_05h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1A05, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1A06h: PDO_TxMappParam_06h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1A06, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1A07h: PDO_TxMappParam_07h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1A07, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1A08h: PDO_TxMappParam_08h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1A08, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1A09h: PDO_TxMappParam_09h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1A09, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1A0Ah: PDO_TxMappParam_0Ah_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1A0A, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1A0Bh: PDO_TxMappParam_0Bh_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1A0B, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1A0Ch: PDO_TxMappParam_0Ch_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1A0C, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1A0Dh: PDO_TxMappParam_0Dh_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1A0D, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1A0Eh: PDO_TxMappParam_0Eh_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1A0E, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1A0Fh: PDO_TxMappParam_0Fh_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1A0F, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1A10h: PDO_TxMappParam_10h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1A10, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1A11h: PDO_TxMappParam_11h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1A11, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1A12h: PDO_TxMappParam_12h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1A12, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1A13h: PDO_TxMappParam_13h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1A13, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1A14h: PDO_TxMappParam_14h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1A14, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1A15h: PDO_TxMappParam_15h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1A15, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1A16h: PDO_TxMappParam_16h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1A16, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1A17h: PDO_TxMappParam_17h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1A17, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1A18h: PDO_TxMappParam_18h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1A18, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1A19h: PDO_TxMappParam_19h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1A19, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1A1Ah: PDO_TxMappParam_1Ah_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1A1A, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1A1Bh: PDO_TxMappParam_1Bh_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1A1B, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1A1Ch: PDO_TxMappParam_1Ch_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1A1C, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1A1Dh: PDO_TxMappParam_1Dh_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1A1D, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1A1Eh: PDO_TxMappParam_1Eh_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1A1E, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1A1Fh: PDO_TxMappParam_1Fh_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1A1F, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1A20h: PDO_TxMappParam_20h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1A20, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1A21h: PDO_TxMappParam_21h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1A21, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1A22h: PDO_TxMappParam_22h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1A22, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1A23h: PDO_TxMappParam_23h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1A23, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1A24h: PDO_TxMappParam_24h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1A24, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1A25h: PDO_TxMappParam_25h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1A25, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1A26h: PDO_TxMappParam_26h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1A26, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1A27h: PDO_TxMappParam_27h_AU64
        OBD_RAM_INDEX_RAM_PDO_MAPPING(0x1A27, 0xFE, TRUE, kObdAccSRW, ObjectMapping, 0x0000000000000000LL)

        // Object 1C00h: DLL_MNCRCError_REC
        OBD_BEGIN_INDEX_RAM(0x1C00, 0x04, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1C00, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x03)
            OBD_SUBINDEX_RAM_USERDEF_NOINIT(0x1C00, 0x01, kObdTypeUInt32, kObdAccRW, tObdUnsigned32, CumulativeCnt_U32)
            OBD_SUBINDEX_RAM_USERDEF(0x1C00, 0x02, kObdTypeUInt32, kObdAccR, tObdUnsigned32, ThresholdCnt_U32, 0)
            OBD_SUBINDEX_RAM_USERDEF(0x1C00, 0x03, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, Threshold_U32, 15)
        OBD_END_INDEX(0x1C00)

        // Object 1C02h: DLL_MNCycTimeExceed_REC
        OBD_BEGIN_INDEX_RAM(0x1C02, 0x04, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1C02, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x03)
            OBD_SUBINDEX_RAM_USERDEF_NOINIT(0x1C02, 0x01, kObdTypeUInt32, kObdAccRW, tObdUnsigned32, CumulativeCnt_U32)
            OBD_SUBINDEX_RAM_USERDEF(0x1C02, 0x02, kObdTypeUInt32, kObdAccR, tObdUnsigned32, ThresholdCnt_U32, 0)
            OBD_SUBINDEX_RAM_USERDEF(0x1C02, 0x03, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, Threshold_U32, 15)
        OBD_END_INDEX(0x1C02)

        // Object 1C07h: DLL_MNCNLossPResCumCnt_AU32
        OBD_RAM_INDEX_RAM_VARARRAY_NOINIT(0x1C07, 254, TRUE, kObdTypeUInt32, kObdAccRW, tObdUnsigned32, DLL_MNCNLossPResCumCnt_AU32)

        // Object 1C08h: DLL_MNCNLossPResThrCnt_AU32
        OBD_RAM_INDEX_RAM_VARARRAY(0x1C08, 254, TRUE, kObdTypeUInt32, kObdAccR, tObdUnsigned32, DLL_MNCNLossPResThrCnt_AU32, 0)

        // Object 1C09h: DLL_MNCNLossPResThreshold_AU32
        OBD_RAM_INDEX_RAM_VARARRAY(0x1C09, 254, TRUE, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, DLL_MNCNLossPResThreshold_AU32, 15)

        // Object 1C0Bh: DLL_CNLossSoC_REC
        OBD_BEGIN_INDEX_RAM(0x1C0B, 0x04, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1C0B, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 3)
            OBD_SUBINDEX_RAM_USERDEF_NOINIT(0x1C0B, 0x01, kObdTypeUInt32, kObdAccRW, tObdUnsigned32, CumulativeCnt_U32)
            OBD_SUBINDEX_RAM_USERDEF(0x1C0B, 0x02, kObdTypeUInt32, kObdAccR, tObdUnsigned32, ThresholdCnt_U32, 0)
            OBD_SUBINDEX_RAM_USERDEF(0x1C0B, 0x03, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, Threshold_U32, 15)
        OBD_END_INDEX(0x1C0B)

        // Object 1C0Dh: DLL_CNLossPReq_REC
        OBD_BEGIN_INDEX_RAM(0x1C0D, 0x04, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1C0D, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 3)
            OBD_SUBINDEX_RAM_USERDEF_NOINIT(0x1C0D, 0x01, kObdTypeUInt32, kObdAccRW, tObdUnsigned32, CumulativeCnt_U32)
            OBD_SUBINDEX_RAM_USERDEF(0x1C0D, 0x02, kObdTypeUInt32, kObdAccR, tObdUnsigned32, ThresholdCnt_U32, 0)
            OBD_SUBINDEX_RAM_USERDEF(0x1C0D, 0x03, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, Threshold_U32, 15)
        OBD_END_INDEX(0x1C0D)

        // Object 1C0Fh: DLL_CNCRCError_REC
        OBD_BEGIN_INDEX_RAM(0x1C0F, 0x04, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1C0F, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 3)
            OBD_SUBINDEX_RAM_USERDEF_NOINIT(0x1C0F, 0x01, kObdTypeUInt32, kObdAccRW, tObdUnsigned32, CumulativeCnt_U32)
            OBD_SUBINDEX_RAM_USERDEF(0x1C0F, 0x02, kObdTypeUInt32, kObdAccR, tObdUnsigned32, ThresholdCnt_U32, 0)
            OBD_SUBINDEX_RAM_USERDEF(0x1C0F, 0x03, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, Threshold_U32, 15)
        OBD_END_INDEX(0x1C0F)

        // Object 1C12h: DLL_MNCycleSuspendNumber_U32
        OBD_BEGIN_INDEX_RAM(0x1C12, 0x01, FALSE)
            OBD_SUBINDEX_RAM_VAR(0x1C12, 0x00, kObdTypeUInt32, kObdAccRW, tObdUnsigned32, MNCycleSuspendNumber, 1)
        OBD_END_INDEX(0x1C12)

        // Object 1C14h: DLL_LossOfSocTolerance_U32 in [ns]
        OBD_BEGIN_INDEX_RAM(0x1C14, 0x01, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1C14, 0x00, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, LossOfSocTolerance, 100000)
        OBD_END_INDEX(0x1C14)

        // Object 1C16h: DLL_MNLossStatusResThrCnt_AU32
        OBD_RAM_INDEX_RAM_ARRAY(0x1C16, NMT_MAX_NODE_ID, FALSE, kObdTypeUInt32, kObdAccR, tObdUnsigned32, DLL_MNLossStatusResThrCnt_AU32, 0)

        // Object 1C17h: DLL_MNLossStatusResThreshold_AU32
        OBD_RAM_INDEX_RAM_ARRAY(0x1C17, NMT_MAX_NODE_ID, FALSE, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, DLL_MNLossStatusResThreshold_AU32, 15)

#if defined(CONFIG_INCLUDE_IP)
        // Object 1E40h: NWL_IpAddrTable_0h_REC
        OBD_BEGIN_INDEX_RAM(0x1E40, 0x06, TRUE)
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

#if defined(CONFIG_INCLUDE_CFM)
        OBD_RAM_INDEX_RAM_VARARRAY_NOINIT(0x1F22, NMT_MAX_NODE_ID, TRUE, kObdTypeDomain, kObdAccSRW, Domain, CFM_ConciseDcfList_ADOM)
        OBD_RAM_INDEX_RAM_ARRAY(0x1F26, NMT_MAX_NODE_ID, FALSE, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, CFM_ExpConfDateList_AU32, 0x00000000)
        OBD_RAM_INDEX_RAM_ARRAY(0x1F27, NMT_MAX_NODE_ID, FALSE, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, CFM_ExpConfTimeList_AU32, 0x00000000)
#endif

        // Object 1F80h: NMT_StartUp_U32
        OBD_BEGIN_INDEX_RAM(0x1F80, 0x01, FALSE)
            OBD_SUBINDEX_RAM_VAR(0x1F80, 0x00, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, NMT_StartUp_U32, 0x00000800)
        OBD_END_INDEX(0x1F80)

        // Object 1F81h: NMT_NodeAssignment_AU32
        OBD_RAM_INDEX_RAM_ARRAY_ALT(0x1F81, NMT_MAX_NODE_ID, FALSE, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, NMT_NodeAssignment_AU32, 0x00000000)

        // Object 1F82h: NMT_FeatureFlags_U32
        OBD_BEGIN_INDEX_RAM(0x1F82, 0x01, FALSE)
            OBD_SUBINDEX_RAM_VAR(0x1F82, 0x00, kObdTypeUInt32, kObdAccR, tObdUnsigned32, NMT_FeatureFlags_U32, PLK_DEF_FEATURE_FLAGS)
        OBD_END_INDEX(0x1F82)

        // Object 1F83h: NMT_EPLVersion_U8
        OBD_BEGIN_INDEX_RAM(0x1F83, 0x01, FALSE)
            OBD_SUBINDEX_RAM_VAR(0x1F83, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NMT_EPLVersion_U8, 0x20)
        OBD_END_INDEX(0x1F83)

        // Object 1F84h: NMT_MNDeviceTypeIdList_AU32
        OBD_RAM_INDEX_RAM_ARRAY_ALT(0x1F84, 254, FALSE, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, NMT_MNDeviceTypeIdList_AU32, 0x00000000)

        // Object 1F89h: NMT_BootTime_REC
#if defined(CONFIG_INCLUDE_NMT_RMN)
        OBD_BEGIN_INDEX_RAM(0x1F89, 0x09, FALSE)
        OBD_SUBINDEX_RAM_VAR(0x1F89, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x0C)
#else
        OBD_BEGIN_INDEX_RAM(0x1F89, 0x06, FALSE)
        OBD_SUBINDEX_RAM_VAR(0x1F89, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x05)
#endif
            OBD_SUBINDEX_RAM_VAR(0x1F89, 0x01, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, MNWaitNotAct_U32, 1000000)       // in [us]
            OBD_SUBINDEX_RAM_VAR(0x1F89, 0x02, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, MNTimeoutPreOp1_U32, 500000)     // in [us]
            OBD_SUBINDEX_RAM_VAR(0x1F89, 0x03, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, MNWaitPreOp1_U32, 500000)        // in [us]
            OBD_SUBINDEX_RAM_VAR(0x1F89, 0x04, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, MNTimeoutPreOp2_U32, 5000000)    // in [us]
            OBD_SUBINDEX_RAM_VAR(0x1F89, 0x05, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, MNTimeoutReadyToOp_U32, 500000)  // in [us]
#if defined(CONFIG_INCLUDE_NMT_RMN)
            OBD_SUBINDEX_RAM_VAR(0x1F89, 0x0A, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, MNSwitchOverPriority_U32, 10)
            OBD_SUBINDEX_RAM_VAR(0x1F89, 0x0B, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, MNSwitchOverDelay_U32, 10)
            OBD_SUBINDEX_RAM_VAR(0x1F89, 0x0C, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, MNSwitchOverCycleDivider_U32, 10)
#endif
        OBD_END_INDEX(0x1F89)

        // Object 1F8Ah: NMT_MNCycleTiming_REC
        OBD_BEGIN_INDEX_RAM(0x1F8A, 0x03, FALSE)
            OBD_SUBINDEX_RAM_VAR(0x1F8A, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR(0x1F8A, 0x01, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, WaitSoCPReq_U32, 1000)           // in [ns]
            OBD_SUBINDEX_RAM_VAR_RG(0x1F8A, 0x02, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, AsyncSlotTimeout_U32, 100000, 250, 0xFFFFFFFF) // in [ns]
        OBD_END_INDEX(0x1F8A)

        // Object 1F8Bh: NMT_MNPReqPayloadLimitList_AU16
        OBD_RAM_INDEX_RAM_ARRAY_ALT(0x1F8B, 254, FALSE, kObdTypeUInt16, kObdAccSRW, tObdUnsigned16, NMT_MNPReqPayloadLimitList_AU16, 36)

        // Object 1F8Ch: NMT_CurrNMTState_U8
        OBD_BEGIN_INDEX_RAM(0x1F8C, 0x01, FALSE)
            OBD_SUBINDEX_RAM_VAR(0x1F8C, 0x00, kObdTypeUInt8, (kObdAccR | kObdAccPdo), tObdUnsigned8, NMT_CurrNMTState_U8, 0x1C)
        OBD_END_INDEX(0x1F8C)

        // Object 1F8Dh: NMT_PResPayloadLimitList_AU16
        OBD_RAM_INDEX_RAM_ARRAY_ALT(0x1F8D, NMT_MAX_NODE_ID, FALSE, kObdTypeUInt16, kObdAccSRW, tObdUnsigned16, NMT_PResPayloadLimitList_AU16, 36)

        // Object 1F8Eh: NMT_MNNodeCurrState_AU8
        OBD_RAM_INDEX_RAM_ARRAY(0x1F8E, 254, FALSE, kObdTypeUInt8, kObdAccR, tObdUnsigned8, NMT_MNNodeCurrState_AU8, 0x1C)

        // Object 1F8Fh: NMT_MNNodeExpState_AU8
        OBD_RAM_INDEX_RAM_ARRAY(0x1F8F, 254, FALSE, kObdTypeUInt8, kObdAccR, tObdUnsigned8, NMT_MNNodeExpState_AU8, 0x1C)

        // Object 1F92h: NMT_MNCNPResTimeout_AU32 in [ns]
        OBD_RAM_INDEX_RAM_ARRAY_ALT(0x1F92, 254, FALSE, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, NMT_MNCNPResTimeout_AU32, 25000) // in [ns]

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
            OBD_SUBINDEX_RAM_VAR(0x1F98, 0x03, kObdTypeUInt32, kObdAccR, tObdUnsigned32, PResMaxLatency_U32, 0x00000000)     // in [ns]
            OBD_SUBINDEX_RAM_VAR(0x1F98, 0x04, kObdTypeUInt16, kObdAccSRW, tObdUnsigned16, PReqActPayloadLimit_U16, 36)
            OBD_SUBINDEX_RAM_VAR(0x1F98, 0x05, kObdTypeUInt16, kObdAccSRW, tObdUnsigned16, PResActPayloadLimit_U16, 36)
            OBD_SUBINDEX_RAM_VAR(0x1F98, 0x06, kObdTypeUInt32, kObdAccR, tObdUnsigned32, ASndMaxLatency_U32, 0x00000000)     // in [ns]
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

        // Object 1F9Bh: NMT_MultiplCycleAssign_AU8
        OBD_RAM_INDEX_RAM_ARRAY_ALT(0x1F9B, NMT_MAX_NODE_ID, FALSE, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, NMT_MultiplCycleAssign_AU8, 0)

        // Object 1F9Eh: NMT_ResetCmd_U8
        OBD_BEGIN_INDEX_RAM(0x1F9E, 0x01, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1F9E, 0x00, kObdTypeUInt8, kObdAccRW, tObdUnsigned8, NMT_ResetCmd_U8, 0xFF)
        OBD_END_INDEX(0x1F9E)

        // Object 1F9Fh: NMT_RequestCmd_REC
        OBD_BEGIN_INDEX_RAM(0x1F9F, 0x05, TRUE)
            OBD_SUBINDEX_RAM_VAR(0x1F9F, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x04)
            OBD_SUBINDEX_RAM_VAR(0x1F9F, 0x01, kObdTypeBool, kObdAccRW, tObdBoolean, Release_BOOL, FALSE)
            OBD_SUBINDEX_RAM_VAR(0x1F9F, 0x02, kObdTypeUInt8, kObdAccRW, tObdUnsigned8, CmdID_U8, 0xFF)
            OBD_SUBINDEX_RAM_VAR(0x1F9F, 0x03, kObdTypeUInt8, kObdAccRW, tObdUnsigned8, CmdTarget_U8, 0x00)
            OBD_SUBINDEX_RAM_DOMAIN(0x1F9F, 0x04, kObdAccVRW, CmdData_DOM)
        OBD_END_INDEX(0x1F9F)
    OBD_END_PART()

    /*************************************************************************
     * Manufacturer Specific Profile Area (0x2000 - 0x5FFF)
     *************************************************************************/
    OBD_BEGIN_PART_MANUFACTURER()

    OBD_END_PART()

    /*************************************************************************
     * Standardised Device Profile Area (0x6000 - 0x9FFF)
     * Standardised Interface Profile Area (0xA000 - 0xBFFF)
     *************************************************************************/
    OBD_BEGIN_PART_DEVICE()
        // static input process image (from network point of view)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA000, (252), FALSE, kObdTypeInt8, kObdAccVPR, tObdInteger8, PI_Input_I8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA001, (252), FALSE, kObdTypeInt8, kObdAccVPR, tObdInteger8, PI_Input_I8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA002, (252), FALSE, kObdTypeInt8, kObdAccVPR, tObdInteger8, PI_Input_I8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA003, (252), FALSE, kObdTypeInt8, kObdAccVPR, tObdInteger8, PI_Input_I8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA004, (252), FALSE, kObdTypeInt8, kObdAccVPR, tObdInteger8, PI_Input_I8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA005, (252), FALSE, kObdTypeInt8, kObdAccVPR, tObdInteger8, PI_Input_I8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA006, (252), FALSE, kObdTypeInt8, kObdAccVPR, tObdInteger8, PI_Input_I8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA007, (252), FALSE, kObdTypeInt8, kObdAccVPR, tObdInteger8, PI_Input_I8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA008, (252), FALSE, kObdTypeInt8, kObdAccVPR, tObdInteger8, PI_Input_I8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA009, (252), FALSE, kObdTypeInt8, kObdAccVPR, tObdInteger8, PI_Input_I8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA00A, (252), FALSE, kObdTypeInt8, kObdAccVPR, tObdInteger8, PI_Input_I8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA00B, (252), FALSE, kObdTypeInt8, kObdAccVPR, tObdInteger8, PI_Input_I8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA00C, (252), FALSE, kObdTypeInt8, kObdAccVPR, tObdInteger8, PI_Input_I8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA00D, (252), FALSE, kObdTypeInt8, kObdAccVPR, tObdInteger8, PI_Input_I8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA00E, (252), FALSE, kObdTypeInt8, kObdAccVPR, tObdInteger8, PI_Input_I8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA00F, (252), FALSE, kObdTypeInt8, kObdAccVPR, tObdInteger8, PI_Input_I8, 0x00)

        OBD_RAM_INDEX_RAM_VARARRAY(0xA040, (252), FALSE, kObdTypeUInt8, kObdAccVPR, tObdUnsigned8, PI_Input_U8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA041, (252), FALSE, kObdTypeUInt8, kObdAccVPR, tObdUnsigned8, PI_Input_U8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA042, (252), FALSE, kObdTypeUInt8, kObdAccVPR, tObdUnsigned8, PI_Input_U8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA043, (252), FALSE, kObdTypeUInt8, kObdAccVPR, tObdUnsigned8, PI_Input_U8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA044, (252), FALSE, kObdTypeUInt8, kObdAccVPR, tObdUnsigned8, PI_Input_U8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA045, (252), FALSE, kObdTypeUInt8, kObdAccVPR, tObdUnsigned8, PI_Input_U8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA046, (252), FALSE, kObdTypeUInt8, kObdAccVPR, tObdUnsigned8, PI_Input_U8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA047, (252), FALSE, kObdTypeUInt8, kObdAccVPR, tObdUnsigned8, PI_Input_U8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA048, (252), FALSE, kObdTypeUInt8, kObdAccVPR, tObdUnsigned8, PI_Input_U8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA049, (252), FALSE, kObdTypeUInt8, kObdAccVPR, tObdUnsigned8, PI_Input_U8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA04A, (252), FALSE, kObdTypeUInt8, kObdAccVPR, tObdUnsigned8, PI_Input_U8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA04B, (252), FALSE, kObdTypeUInt8, kObdAccVPR, tObdUnsigned8, PI_Input_U8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA04C, (252), FALSE, kObdTypeUInt8, kObdAccVPR, tObdUnsigned8, PI_Input_U8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA04D, (252), FALSE, kObdTypeUInt8, kObdAccVPR, tObdUnsigned8, PI_Input_U8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA04E, (252), FALSE, kObdTypeUInt8, kObdAccVPR, tObdUnsigned8, PI_Input_U8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA04F, (252), FALSE, kObdTypeUInt8, kObdAccVPR, tObdUnsigned8, PI_Input_U8, 0x00)

        OBD_RAM_INDEX_RAM_VARARRAY(0xA0C0, (252), FALSE, kObdTypeInt16, kObdAccVPR, tObdInteger16, PI_Input_I16, 0x0000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA0C1, (252), FALSE, kObdTypeInt16, kObdAccVPR, tObdInteger16, PI_Input_I16, 0x0000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA0C2, (252), FALSE, kObdTypeInt16, kObdAccVPR, tObdInteger16, PI_Input_I16, 0x0000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA0C3, (252), FALSE, kObdTypeInt16, kObdAccVPR, tObdInteger16, PI_Input_I16, 0x0000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA0C4, (252), FALSE, kObdTypeInt16, kObdAccVPR, tObdInteger16, PI_Input_I16, 0x0000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA0C5, (252), FALSE, kObdTypeInt16, kObdAccVPR, tObdInteger16, PI_Input_I16, 0x0000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA0C6, (252), FALSE, kObdTypeInt16, kObdAccVPR, tObdInteger16, PI_Input_I16, 0x0000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA0C7, (252), FALSE, kObdTypeInt16, kObdAccVPR, tObdInteger16, PI_Input_I16, 0x0000)

        OBD_RAM_INDEX_RAM_VARARRAY(0xA100, (252), FALSE, kObdTypeUInt16, kObdAccVPR, tObdUnsigned16, PI_Input_U16, 0x0000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA101, (252), FALSE, kObdTypeUInt16, kObdAccVPR, tObdUnsigned16, PI_Input_U16, 0x0000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA102, (252), FALSE, kObdTypeUInt16, kObdAccVPR, tObdUnsigned16, PI_Input_U16, 0x0000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA103, (252), FALSE, kObdTypeUInt16, kObdAccVPR, tObdUnsigned16, PI_Input_U16, 0x0000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA104, (252), FALSE, kObdTypeUInt16, kObdAccVPR, tObdUnsigned16, PI_Input_U16, 0x0000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA105, (252), FALSE, kObdTypeUInt16, kObdAccVPR, tObdUnsigned16, PI_Input_U16, 0x0000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA106, (252), FALSE, kObdTypeUInt16, kObdAccVPR, tObdUnsigned16, PI_Input_U16, 0x0000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA107, (252), FALSE, kObdTypeUInt16, kObdAccVPR, tObdUnsigned16, PI_Input_U16, 0x0000)

        OBD_RAM_INDEX_RAM_VARARRAY(0xA1C0, (252), FALSE, kObdTypeInt32, kObdAccVPR, tObdInteger32, PI_Input_I32, 0x00000000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA1C1, (252), FALSE, kObdTypeInt32, kObdAccVPR, tObdInteger32, PI_Input_I32, 0x00000000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA1C2, (252), FALSE, kObdTypeInt32, kObdAccVPR, tObdInteger32, PI_Input_I32, 0x00000000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA1C3, (252), FALSE, kObdTypeInt32, kObdAccVPR, tObdInteger32, PI_Input_I32, 0x00000000)

        OBD_RAM_INDEX_RAM_VARARRAY(0xA200, (252), FALSE, kObdTypeUInt32, kObdAccVPR, tObdUnsigned32, PI_Input_U32, 0x00000000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA201, (252), FALSE, kObdTypeUInt32, kObdAccVPR, tObdUnsigned32, PI_Input_U32, 0x00000000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA202, (252), FALSE, kObdTypeUInt32, kObdAccVPR, tObdUnsigned32, PI_Input_U32, 0x00000000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA203, (252), FALSE, kObdTypeUInt32, kObdAccVPR, tObdUnsigned32, PI_Input_U32, 0x00000000)

        OBD_RAM_INDEX_RAM_VARARRAY(0xA240, (252), FALSE, kObdTypeReal32, kObdAccVPR, tObdReal32, PI_Input_REAL32, 0x00000000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA241, (252), FALSE, kObdTypeReal32, kObdAccVPR, tObdReal32, PI_Input_REAL32, 0x00000000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA242, (252), FALSE, kObdTypeReal32, kObdAccVPR, tObdReal32, PI_Input_REAL32, 0x00000000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA243, (252), FALSE, kObdTypeReal32, kObdAccVPR, tObdReal32, PI_Input_REAL32, 0x00000000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA244, (252), FALSE, kObdTypeReal32, kObdAccVPR, tObdReal32, PI_Input_REAL32, 0x00000000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA245, (252), FALSE, kObdTypeReal32, kObdAccVPR, tObdReal32, PI_Input_REAL32, 0x00000000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA246, (252), FALSE, kObdTypeReal32, kObdAccVPR, tObdReal32, PI_Input_REAL32, 0x00000000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA247, (252), FALSE, kObdTypeReal32, kObdAccVPR, tObdReal32, PI_Input_REAL32, 0x00000000)

        OBD_RAM_INDEX_RAM_VARARRAY(0xA400, (252), FALSE, kObdTypeInt64, kObdAccVPR, tObdInteger64, PI_Input_I64, 0x0000000000000000LL)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA401, (252), FALSE, kObdTypeInt64, kObdAccVPR, tObdInteger64, PI_Input_I64, 0x0000000000000000LL)

        OBD_RAM_INDEX_RAM_VARARRAY(0xA440, (252), FALSE, kObdTypeUInt64, kObdAccVPR, tObdUnsigned64, PI_Input_U64, 0x0000000000000000LL)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA441, (252), FALSE, kObdTypeUInt64, kObdAccVPR, tObdUnsigned64, PI_Input_U64, 0x0000000000000000LL)


        // static output process image (from network point of view)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA480, (252), FALSE, kObdTypeInt8, kObdAccVPRW, tObdInteger8, PI_Output_I8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA481, (252), FALSE, kObdTypeInt8, kObdAccVPRW, tObdInteger8, PI_Output_I8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA482, (252), FALSE, kObdTypeInt8, kObdAccVPRW, tObdInteger8, PI_Output_I8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA483, (252), FALSE, kObdTypeInt8, kObdAccVPRW, tObdInteger8, PI_Output_I8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA484, (252), FALSE, kObdTypeInt8, kObdAccVPRW, tObdInteger8, PI_Output_I8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA485, (252), FALSE, kObdTypeInt8, kObdAccVPRW, tObdInteger8, PI_Output_I8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA486, (252), FALSE, kObdTypeInt8, kObdAccVPRW, tObdInteger8, PI_Output_I8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA487, (252), FALSE, kObdTypeInt8, kObdAccVPRW, tObdInteger8, PI_Output_I8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA488, (252), FALSE, kObdTypeInt8, kObdAccVPRW, tObdInteger8, PI_Output_I8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA489, (252), FALSE, kObdTypeInt8, kObdAccVPRW, tObdInteger8, PI_Output_I8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA48A, (252), FALSE, kObdTypeInt8, kObdAccVPRW, tObdInteger8, PI_Output_I8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA48B, (252), FALSE, kObdTypeInt8, kObdAccVPRW, tObdInteger8, PI_Output_I8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA48C, (252), FALSE, kObdTypeInt8, kObdAccVPRW, tObdInteger8, PI_Output_I8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA48D, (252), FALSE, kObdTypeInt8, kObdAccVPRW, tObdInteger8, PI_Output_I8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA48E, (252), FALSE, kObdTypeInt8, kObdAccVPRW, tObdInteger8, PI_Output_I8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA48F, (252), FALSE, kObdTypeInt8, kObdAccVPRW, tObdInteger8, PI_Output_I8, 0x00)

        OBD_RAM_INDEX_RAM_VARARRAY(0xA4C0, (252), FALSE, kObdTypeUInt8, kObdAccVPRW, tObdUnsigned8, PI_Output_U8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA4C1, (252), FALSE, kObdTypeUInt8, kObdAccVPRW, tObdUnsigned8, PI_Output_U8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA4C2, (252), FALSE, kObdTypeUInt8, kObdAccVPRW, tObdUnsigned8, PI_Output_U8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA4C3, (252), FALSE, kObdTypeUInt8, kObdAccVPRW, tObdUnsigned8, PI_Output_U8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA4C4, (252), FALSE, kObdTypeUInt8, kObdAccVPRW, tObdUnsigned8, PI_Output_U8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA4C5, (252), FALSE, kObdTypeUInt8, kObdAccVPRW, tObdUnsigned8, PI_Output_U8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA4C6, (252), FALSE, kObdTypeUInt8, kObdAccVPRW, tObdUnsigned8, PI_Output_U8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA4C7, (252), FALSE, kObdTypeUInt8, kObdAccVPRW, tObdUnsigned8, PI_Output_U8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA4C8, (252), FALSE, kObdTypeUInt8, kObdAccVPRW, tObdUnsigned8, PI_Output_U8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA4C9, (252), FALSE, kObdTypeUInt8, kObdAccVPRW, tObdUnsigned8, PI_Output_U8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA4CA, (252), FALSE, kObdTypeUInt8, kObdAccVPRW, tObdUnsigned8, PI_Output_U8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA4CB, (252), FALSE, kObdTypeUInt8, kObdAccVPRW, tObdUnsigned8, PI_Output_U8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA4CC, (252), FALSE, kObdTypeUInt8, kObdAccVPRW, tObdUnsigned8, PI_Output_U8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA4CD, (252), FALSE, kObdTypeUInt8, kObdAccVPRW, tObdUnsigned8, PI_Output_U8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA4CE, (252), FALSE, kObdTypeUInt8, kObdAccVPRW, tObdUnsigned8, PI_Output_U8, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA4CF, (252), FALSE, kObdTypeUInt8, kObdAccVPRW, tObdUnsigned8, PI_Output_U8, 0x00)

        OBD_RAM_INDEX_RAM_VARARRAY(0xA540, (252), FALSE, kObdTypeInt16, kObdAccVPRW, tObdInteger16, PI_Output_I16, 0x0000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA541, (252), FALSE, kObdTypeInt16, kObdAccVPRW, tObdInteger16, PI_Output_I16, 0x0000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA542, (252), FALSE, kObdTypeInt16, kObdAccVPRW, tObdInteger16, PI_Output_I16, 0x0000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA543, (252), FALSE, kObdTypeInt16, kObdAccVPRW, tObdInteger16, PI_Output_I16, 0x0000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA544, (252), FALSE, kObdTypeInt16, kObdAccVPRW, tObdInteger16, PI_Output_I16, 0x0000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA545, (252), FALSE, kObdTypeInt16, kObdAccVPRW, tObdInteger16, PI_Output_I16, 0x0000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA546, (252), FALSE, kObdTypeInt16, kObdAccVPRW, tObdInteger16, PI_Output_I16, 0x0000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA547, (252), FALSE, kObdTypeInt16, kObdAccVPRW, tObdInteger16, PI_Output_I16, 0x0000)

        OBD_RAM_INDEX_RAM_VARARRAY(0xA580, (252), FALSE, kObdTypeUInt16, kObdAccVPRW, tObdUnsigned16, PI_Output_U16, 0x0000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA581, (252), FALSE, kObdTypeUInt16, kObdAccVPRW, tObdUnsigned16, PI_Output_U16, 0x0000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA582, (252), FALSE, kObdTypeUInt16, kObdAccVPRW, tObdUnsigned16, PI_Output_U16, 0x0000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA583, (252), FALSE, kObdTypeUInt16, kObdAccVPRW, tObdUnsigned16, PI_Output_U16, 0x0000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA584, (252), FALSE, kObdTypeUInt16, kObdAccVPRW, tObdUnsigned16, PI_Output_U16, 0x0000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA585, (252), FALSE, kObdTypeUInt16, kObdAccVPRW, tObdUnsigned16, PI_Output_U16, 0x0000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA586, (252), FALSE, kObdTypeUInt16, kObdAccVPRW, tObdUnsigned16, PI_Output_U16, 0x0000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA587, (252), FALSE, kObdTypeUInt16, kObdAccVPRW, tObdUnsigned16, PI_Output_U16, 0x0000)

        OBD_RAM_INDEX_RAM_VARARRAY(0xA640, (252), FALSE, kObdTypeInt32, kObdAccVPRW, tObdInteger32, PI_Output_I32, 0x00000000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA641, (252), FALSE, kObdTypeInt32, kObdAccVPRW, tObdInteger32, PI_Output_I32, 0x00000000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA642, (252), FALSE, kObdTypeInt32, kObdAccVPRW, tObdInteger32, PI_Output_I32, 0x00000000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA643, (252), FALSE, kObdTypeInt32, kObdAccVPRW, tObdInteger32, PI_Output_I32, 0x00000000)

        OBD_RAM_INDEX_RAM_VARARRAY(0xA680, (252), FALSE, kObdTypeUInt32, kObdAccVPRW, tObdUnsigned32, PI_Output_U32, 0x00000000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA681, (252), FALSE, kObdTypeUInt32, kObdAccVPRW, tObdUnsigned32, PI_Output_U32, 0x00000000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA682, (252), FALSE, kObdTypeUInt32, kObdAccVPRW, tObdUnsigned32, PI_Output_U32, 0x00000000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA683, (252), FALSE, kObdTypeUInt32, kObdAccVPRW, tObdUnsigned32, PI_Output_U32, 0x00000000)

        OBD_RAM_INDEX_RAM_VARARRAY(0xA6C0, (252), FALSE, kObdTypeReal32, kObdAccVPRW, tObdReal32, PI_Output_REAL32, 0x00000000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA6C1, (252), FALSE, kObdTypeReal32, kObdAccVPRW, tObdReal32, PI_Output_REAL32, 0x00000000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA6C2, (252), FALSE, kObdTypeReal32, kObdAccVPRW, tObdReal32, PI_Output_REAL32, 0x00000000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA6C3, (252), FALSE, kObdTypeReal32, kObdAccVPRW, tObdReal32, PI_Output_REAL32, 0x00000000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA6C4, (252), FALSE, kObdTypeReal32, kObdAccVPRW, tObdReal32, PI_Output_REAL32, 0x00000000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA6C5, (252), FALSE, kObdTypeReal32, kObdAccVPRW, tObdReal32, PI_Output_REAL32, 0x00000000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA6C6, (252), FALSE, kObdTypeReal32, kObdAccVPRW, tObdReal32, PI_Output_REAL32, 0x00000000)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA6C7, (252), FALSE, kObdTypeReal32, kObdAccVPRW, tObdReal32, PI_Output_REAL32, 0x00000000)

        OBD_RAM_INDEX_RAM_VARARRAY(0xA880, (252), FALSE, kObdTypeInt64, kObdAccVPRW, tObdInteger64, PI_Output_I64, 0x0000000000000000LL)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA881, (252), FALSE, kObdTypeInt64, kObdAccVPRW, tObdInteger64, PI_Output_I64, 0x0000000000000000LL)

        OBD_RAM_INDEX_RAM_VARARRAY(0xA8C0, (252), FALSE, kObdTypeUInt64, kObdAccVPRW, tObdUnsigned64, PI_Output_U64, 0x0000000000000000LL)
        OBD_RAM_INDEX_RAM_VARARRAY(0xA8C1, (252), FALSE, kObdTypeUInt64, kObdAccVPRW, tObdUnsigned64, PI_Output_U64, 0x0000000000000000LL)

    OBD_END_PART()

OBD_END()

#define OBD_UNDEFINE_MACRO
    #include <obdcreate/obdmacro.h>
#undef OBD_UNDEFINE_MACRO
