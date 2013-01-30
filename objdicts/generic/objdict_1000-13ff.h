//-----------------------------------------------------------------
//  Generic Communication Profile Area 1000h - 13FFh
//-----------------------------------------------------------------

        // Object 1000h: NMT_DeviceType_U32
        EPL_OBD_BEGIN_INDEX_RAM(0x1000, 0x01, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1000, 0x00, kEplObdTypUInt32, kEplObdAccR, tEplObdUnsigned32, NMT_DeviceType_U32, 0xF0191)
        EPL_OBD_END_INDEX(0x1000)

        // Object 1001h : ERR_ErrorRegister_U8
        EPL_OBD_BEGIN_INDEX_RAM(0x1001, 0x01, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1001, 0x00, kEplObdTypUInt8, kEplObdAccR, tEplObdUnsigned8, ERR_ErrorRegister_U8, 0x00)
        EPL_OBD_END_INDEX(0x1001)

/*
        // Object 1003h: ERR_History_ADOM
        EPL_OBD_RAM_INDEX_RAM_VARARRAY_NOINIT(0x1003, 10, NULL, tEplObdDomain, kEplObdAccR, tEplObdDomain, ERR_History_ADOM)
*/

        // Object 1006h: NMT_CycleLen_U32 in [us]
        EPL_OBD_BEGIN_INDEX_RAM(0x1006, 0x01, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1006, 0x00, kEplObdTypUInt32, kEplObdAccSRW, tEplObdUnsigned32, NMT_CycleLen_U32, 0x00)   // in [us]
        EPL_OBD_END_INDEX(0x1006)

        // Object 1008h: NMT_ManufactDevName_VS
        EPL_OBD_BEGIN_INDEX_RAM(0x1008, 0x01, NULL)
           EPL_OBD_SUBINDEX_RAM_VSTRING(0x1008, 0x00, kEplObdAccR, device_name, EPL_OBD_MAX_STRING_SIZE, "openPOWERLINK device")
        EPL_OBD_END_INDEX(0x1008)

        // Object 1009h: NMT_ManufactHwVers_VS
        EPL_OBD_BEGIN_INDEX_RAM(0x1009, 0x01, NULL)
           EPL_OBD_SUBINDEX_RAM_VSTRING(0x1009, 0x00, kEplObdAccR, hardware_version, EPL_OBD_MAX_STRING_SIZE, "1.00")
        EPL_OBD_END_INDEX(0x1009)

        // Object 100Ah: NMT_ManufactSwVers_VS
        EPL_OBD_BEGIN_INDEX_RAM(0x100A, 0x01, NULL)
           EPL_OBD_SUBINDEX_RAM_VSTRING(0x100A, 0x00, kEplObdAccR, software_version, EPL_OBD_MAX_STRING_SIZE, EPL_PRODUCT_NAME" "EPL_PRODUCT_VERSION)
        EPL_OBD_END_INDEX(0x100A)

        // Object 1018h: NMT_IdentityObject_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1018, 0x05, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1018, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 0x04)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1018, 0x01, kEplObdTypUInt32, kEplObdAccR, tEplObdUnsigned32, VendorId_U32, 0x00000000)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1018, 0x02, kEplObdTypUInt32, kEplObdAccR, tEplObdUnsigned32, ProductCode_U32, 0x00000000)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1018, 0x03, kEplObdTypUInt32, kEplObdAccR, tEplObdUnsigned32, RevisionNo_U32, EPL_DEFINED_OBJ1018_VERSION)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1018, 0x04, kEplObdTypUInt32, kEplObdAccR, tEplObdUnsigned32, SerialNo_U32, 0x00000000)
        EPL_OBD_END_INDEX(0x1018)

        // Object 1020h: CFM_VerifyConfiguration_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1020, 0x03, EplApiCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1020, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1020, 0x01, kEplObdTypUInt32, kEplObdAccSRW, tEplObdUnsigned32, ConfDate_U32, 0)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1020, 0x02, kEplObdTypUInt32, kEplObdAccSRW, tEplObdUnsigned32, ConfTime_U32, 0)
//            EPL_OBD_SUBINDEX_RAM_VAR(0x1020, 0x03, kEplObdTypUInt32, kEplObdAccRW, tEplObdUnsigned32, ConfId_U32, 0)
//            EPL_OBD_SUBINDEX_RAM_VAR(0x1020, 0x04, kEplObdTypUInt32, kEplObdAccRW, tEplObdUnsigned32, VerifyConfInvalid_U32, 1)
        EPL_OBD_END_INDEX(0x1020)

        // Object 1030h: NMT_InterfaceGroup_Xh_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1030, 0x0A, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1030, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 0x09)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1030, 0x01, kEplObdTypUInt16, kEplObdAccR, tEplObdUnsigned16, InterfaceIndex_U16, 0x01)
            EPL_OBD_SUBINDEX_RAM_VSTRING(0x1030, 0x02, kEplObdAccR, InterfaceDescription_VSTR ,0x20, "Interface 1")
            EPL_OBD_SUBINDEX_RAM_VAR(0x1030, 0x03, kEplObdTypUInt8, kEplObdAccR, tEplObdUnsigned8, InterfaceType_U8, 0x06)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1030, 0x04, kEplObdTypUInt16, kEplObdAccR, tEplObdUnsigned16, InterfaceMtu_U16, 1500)
            EPL_OBD_SUBINDEX_RAM_OSTRING(0x1030, 0x05, kEplObdAccR,  InterfacePhysAddress_OSTR, 0x06)
            EPL_OBD_SUBINDEX_RAM_VSTRING(0x1030, 0x06, kEplObdAccR,  InterfaceName_VSTR, 0x20 ,"Interface 1")
            EPL_OBD_SUBINDEX_RAM_VAR(0x1030, 0x07, kEplObdTypUInt8, kEplObdAccR, tEplObdUnsigned8, InterfaceOperStatus_U8, 0x1)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1030, 0x08, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, InterfaceAdminState_U8, 0x1)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1030, 0x09, kEplObdTypBool, kEplObdAccRW, tEplObdBoolean, Valid_BOOL, 0x1)
        EPL_OBD_END_INDEX(0x1030)

#if ((EPL_DLL_PRES_CHAINING_CN != FALSE) && (EPL_NMT_MAX_NODE_ID > 0))
        // Object 1050h: NMT_RelativeLatencyDiff_AU32
        EPL_OBD_RAM_INDEX_RAM_ARRAY(0x1050, EPL_NMT_MAX_NODE_ID, NULL, kEplObdTypUInt32, kEplObdAccR, tEplObdUnsigned32, NMT_RelativeLatencyDiff_AU32, 0)
#endif

        // Object 1300h: SDO_SequLayerTimeout_U32 in [ms]
        EPL_OBD_BEGIN_INDEX_RAM(0x1300, 0x01, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1300, 0x00, kEplObdTypUInt32, kEplObdAccSRW, tEplObdUnsigned32, SDO_SequLayerTimeout_U32, 5000)
        EPL_OBD_END_INDEX(0x1300)

