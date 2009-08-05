//-----------------------------------------------------------------
//  OD for minimal EPL CN
//-----------------------------------------------------------------
#define EPL_OBD_DEFINE_MACRO
    #include "EplObdMacro.h"
#undef EPL_OBD_DEFINE_MACRO

EPL_OBD_BEGIN ()

    EPL_OBD_BEGIN_PART_GENERIC ()

        // Object 1000h: NMT_DeviceType_U32
        EPL_OBD_BEGIN_INDEX_RAM(0x1000, 0x01, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1000, 0x00, 0x07, 0x01, tEplObdUnsigned32, NMT_DeviceType_U32, 0xF0191)
        EPL_OBD_END_INDEX(0x1000)

        // Object 1001h : ERR_ErrorRegister_U8
        EPL_OBD_BEGIN_INDEX_RAM(0x1001, 0x01, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1001, 0x00, 0x05, 0x01, tEplObdUnsigned8, ERR_ErrorRegister_U8, 0x00)
        EPL_OBD_END_INDEX(0x1001)

/*
        // Object 1003h: ERR_History_ADOM
        EPL_OBD_RAM_INDEX_RAM_VARARRAY_NOINIT(0x1003, 10, NULL, 0x0F, 0x01, tEplObdDomain, ERR_History_ADOM)
*/

        // Object 1006h: NMT_CycleLen_U32 in [us]
        EPL_OBD_BEGIN_INDEX_RAM(0x1006, 0x01, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1006, 0x00, 0x07, 0x03, tEplObdUnsigned32, NMT_CycleLen_U32, 0x00)   // in [us]
        EPL_OBD_END_INDEX(0x1006)

        // Object 1008h: NMT_ManufactDevName_VS
        EPL_OBD_BEGIN_INDEX_RAM(0x1008, 0x01, NULL)
           EPL_OBD_SUBINDEX_RAM_VSTRING(0x1008, 0x00, 0x01, device_name, EPL_OBD_MAX_STRING_SIZE, "SYS TEC electronic EPL V2 Stack")
        EPL_OBD_END_INDEX(0x1008)

        // Object 1009h: NMT_ManufactHwVers_VS
        EPL_OBD_BEGIN_INDEX_RAM(0x1009, 0x01, NULL)
           EPL_OBD_SUBINDEX_RAM_VSTRING(0x1009, 0x00, 0x01, hardware_version, EPL_OBD_MAX_STRING_SIZE, "1.00")
        EPL_OBD_END_INDEX(0x1009)

        // Object 100Ah: NMT_ManufactSwVers_VS
        EPL_OBD_BEGIN_INDEX_RAM(0x100A, 0x01, NULL)
           EPL_OBD_SUBINDEX_RAM_VSTRING(0x100A, 0x00, 0x01, software_version, EPL_OBD_MAX_STRING_SIZE, EPL_PRODUCT_NAME" "EPL_PRODUCT_VERSION)
        EPL_OBD_END_INDEX(0x100A)

        // Object 1018h: NMT_IdentityObject_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1018, 0x05, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1018, 0x00, 0x05, 0x04, tEplObdUnsigned8, NumberOfEntries, 0x04)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1018, 0x01, 0x07, 0x01, tEplObdUnsigned32, VendorId_U32, 0x3f)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1018, 0x02, 0x07, 0x01, tEplObdUnsigned32, ProductCode_U32, 0x1067)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1018, 0x03, 0x07, 0x01, tEplObdUnsigned32, RevisionNo_U32, DEFINED_OBJ1018_VERSION)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1018, 0x04, 0x07, 0x01, tEplObdUnsigned32, SerialNo_U32, 0x12345678)
        EPL_OBD_END_INDEX(0x1018)

        // Object 1020h: CFM_VerifyConfiguration_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1020, 0x03, EplApiCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1020, 0x00, 0x05, 0x04, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1020, 0x01, 0x07, 0x03, tEplObdUnsigned32, ConfDate_U32, 0)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1020, 0x02, 0x07, 0x03, tEplObdUnsigned32, ConfTime_U32, 0)
//            EPL_OBD_SUBINDEX_RAM_VAR(0x1020, 0x03, 0x07, 0x03, tEplObdUnsigned32, ConfId_U32, 0)
//            EPL_OBD_SUBINDEX_RAM_VAR(0x1020, 0x04, 0x07, 0x03, tEplObdUnsigned32, VerifyConfInvalid_U32, 1)
        EPL_OBD_END_INDEX(0x1020)

        // Object 1030h: NMT_InterfaceGroup_Xh_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1030, 0x0A, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1030, 0x00, 0x05, 0x04, tEplObdUnsigned8, NumberOfEntries, 0x09)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1030, 0x01, 0x06, 0x01, tEplObdUnsigned16, InterfaceIndex_U16, 0x01)
            EPL_OBD_SUBINDEX_RAM_VSTRING(0x1030, 0x02, 0x04, InterfaceDescription_VSTR ,0x20, "Interface 1")
            EPL_OBD_SUBINDEX_RAM_VAR(0x1030, 0x03, 0x05, 0x04, tEplObdUnsigned8, InterfaceType_U8, 0x06)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1030, 0x04, 0x06, 0x04, tEplObdUnsigned16, InterfaceMtu_U16, 1500)
            EPL_OBD_SUBINDEX_RAM_OSTRING(0x1030, 0x05, 0x04,  InterfacePhysAddress_OSTR, 0x06)
            EPL_OBD_SUBINDEX_RAM_VSTRING(0x1030, 0x06, 0x04,  InterfaceName_VSTR, 0x20 ,"Interface 1")
            EPL_OBD_SUBINDEX_RAM_VAR(0x1030, 0x07, 0x05, 0x01, tEplObdUnsigned8, InterfaceOperStatus_U8, 0x1)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1030, 0x08, 0x05, 0x03, tEplObdUnsigned8, InterfaceAdminState_U8, 0x1)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1030, 0x09, 0x01, 0x03, tEplObdBoolean, Valid_BOOL, 0x1)
        EPL_OBD_END_INDEX(0x1030)

        // Object 1300h: SDO_SequLayerTimeout_U32 in [ms]
        EPL_OBD_BEGIN_INDEX_RAM(0x1300, 0x01, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1300, 0x00, 0x07, 0x03, tEplObdUnsigned32, SDO_SequLayerTimeout_U32, 5000)
        EPL_OBD_END_INDEX(0x1300)

        // Object 1400h: PDO_RxCommParam_00h_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1400, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1400, 0x00, 0x05, 0x04, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1400, 0x01, 0x05, 0x03, tEplObdUnsigned8, NodeID_U8, 0x00)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1400, 0x02, 0x05, 0x03, tEplObdUnsigned8, MappingVersion_U8, 0x00)
        EPL_OBD_END_INDEX(0x1400)

        // Object 1401h: PDO_RxCommParam_01h_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1401, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1401, 0x00, 0x05, 0x04, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1401, 0x01, 0x05, 0x03, tEplObdUnsigned8, NodeID_U8, 0x6E)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1401, 0x02, 0x05, 0x03, tEplObdUnsigned8, MappingVersion_U8, 0x00)
        EPL_OBD_END_INDEX(0x1401)

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
        // additional RxPDOs if master is enabled

        // Object 1402h: PDO_RxCommParam_02h_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1402, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1402, 0x00, 0x05, 0x04, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1402, 0x01, 0x05, 0x03, tEplObdUnsigned8, NodeID_U8, 0x01)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1402, 0x02, 0x05, 0x03, tEplObdUnsigned8, MappingVersion_U8, 0x00)
        EPL_OBD_END_INDEX(0x1402)

        // Object 1403h: PDO_RxCommParam_03h_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1403, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1403, 0x00, 0x05, 0x04, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1403, 0x01, 0x05, 0x03, tEplObdUnsigned8, NodeID_U8, 0x20)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1403, 0x02, 0x05, 0x03, tEplObdUnsigned8, MappingVersion_U8, 0x00)
        EPL_OBD_END_INDEX(0x1403)

#endif

        // Object 1600h: PDO_RxMappParam_00h_AU64
        EPL_OBD_BEGIN_INDEX_RAM(0x1600, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x00, 0x05, 0x03, tEplObdUnsigned8, NumberOfEntries, 0x01)
#if ((EPL_API_PROCESS_IMAGE_SIZE_IN > 0) || (EPL_API_PROCESS_IMAGE_SIZE_OUT > 0))
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x01, 0x1B, 0x03, tEplObdUnsigned64, ObjectMapping, 0x0008000000012000LL)
#else
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x01, 0x1B, 0x03, tEplObdUnsigned64, ObjectMapping, 0x0008000000016200LL)
#endif
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x02, 0x1B, 0x03, tEplObdUnsigned64, ObjectMapping, 0x00)
        EPL_OBD_END_INDEX(0x1600)

        // Object 1601h: PDO_RxMappParam_01h_AU64
        EPL_OBD_BEGIN_INDEX_RAM(0x1601, 0x03, EplPdouCbObdAccess)
#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
            // enable this RxPDO if master is enabled
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x00, 0x05, 0x03, tEplObdUnsigned8, NumberOfEntries, 0x01)
#else
            // otherwise the corresponding object does not exist
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x00, 0x05, 0x03, tEplObdUnsigned8, NumberOfEntries, 0x00)
#endif
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x01, 0x1B, 0x03, tEplObdUnsigned64, ObjectMapping, 0x0008000000012200LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x02, 0x1B, 0x03, tEplObdUnsigned64, ObjectMapping, 0x00)
        EPL_OBD_END_INDEX(0x1601)

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
        // additional RxPDOs if master is enabled

        // Object 1602h: PDO_RxMappParam_02h_AU64
        EPL_OBD_BEGIN_INDEX_RAM(0x1602, 0x02, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x00, 0x05, 0x03, tEplObdUnsigned8, NumberOfEntries, 0x01)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x01, 0x1B, 0x03, tEplObdUnsigned64, ObjectMapping, 0x0008000000022200LL)
        EPL_OBD_END_INDEX(0x1602)

        // Object 1603h: PDO_RxMappParam_03h_AU64
        EPL_OBD_BEGIN_INDEX_RAM(0x1603, 0x02, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x00, 0x05, 0x03, tEplObdUnsigned8, NumberOfEntries, 0x01)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x01, 0x1B, 0x03, tEplObdUnsigned64, ObjectMapping, 0x0008000000032200LL)
        EPL_OBD_END_INDEX(0x1603)

#endif

        // Object 1800h: PDO_TxCommParam_00h_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1800, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1800, 0x00, 0x05, 0x04, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1800, 0x01, 0x05, 0x03, tEplObdUnsigned8, NodeID_U8, 0x00)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1800, 0x02, 0x05, 0x03, tEplObdUnsigned8, MappingVersion_U8, 0x00)
        EPL_OBD_END_INDEX(0x1800)

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
        // additional TxPDOs if master is enabled

        // Object 1801h: PDO_TxCommParam_01h_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1801, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1801, 0x00, 0x05, 0x04, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1801, 0x01, 0x05, 0x03, tEplObdUnsigned8, NodeID_U8, 0x01)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1801, 0x02, 0x05, 0x03, tEplObdUnsigned8, MappingVersion_U8, 0x00)
        EPL_OBD_END_INDEX(0x1801)

        // Object 1802h: PDO_TxCommParam_02h_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1802, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1802, 0x00, 0x05, 0x04, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1802, 0x01, 0x05, 0x03, tEplObdUnsigned8, NodeID_U8, 0x20)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1802, 0x02, 0x05, 0x03, tEplObdUnsigned8, MappingVersion_U8, 0x00)
        EPL_OBD_END_INDEX(0x1802)

        // Object 1803h: PDO_TxCommParam_03h_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1803, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1803, 0x00, 0x05, 0x04, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1803, 0x01, 0x05, 0x03, tEplObdUnsigned8, NodeID_U8, 0x6E)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1803, 0x02, 0x05, 0x03, tEplObdUnsigned8, MappingVersion_U8, 0x00)
        EPL_OBD_END_INDEX(0x1803)

#endif

        // Object 1A00h: PDO_TxMappParam_00h_AU64
        EPL_OBD_BEGIN_INDEX_RAM(0x1A00, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x00, 0x05, 0x03, tEplObdUnsigned8, NumberOfEntries, 0x01)
#if ((EPL_API_PROCESS_IMAGE_SIZE_IN > 0) || (EPL_API_PROCESS_IMAGE_SIZE_OUT > 0))
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x01, 0x1B, 0x03, tEplObdUnsigned64, ObjectMapping, 0x0008000000012030LL)
#else
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x01, 0x1B, 0x03, tEplObdUnsigned64, ObjectMapping, 0x0008000000016000LL)
#endif
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x02, 0x1B, 0x03, tEplObdUnsigned64, ObjectMapping, 0x00)
        EPL_OBD_END_INDEX(0x1A00)

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
        // additional TxPDOs if master is enabled

        // Object 1A01h: PDO_TxMappParam_01h_AU64
        EPL_OBD_BEGIN_INDEX_RAM(0x1A01, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x00, 0x05, 0x03, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x01, 0x1B, 0x03, tEplObdUnsigned64, ObjectMapping, 0x0008000000022000LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x02, 0x1B, 0x03, tEplObdUnsigned64, ObjectMapping, 0x0008000800012000LL)
        EPL_OBD_END_INDEX(0x1A01)

        // Object 1A02h: PDO_TxMappParam_02h_AU64
        EPL_OBD_BEGIN_INDEX_RAM(0x1A02, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x00, 0x05, 0x03, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x01, 0x1B, 0x03, tEplObdUnsigned64, ObjectMapping, 0x0008000000022000LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x02, 0x1B, 0x03, tEplObdUnsigned64, ObjectMapping, 0x0008000800012000LL)
        EPL_OBD_END_INDEX(0x1A02)

        // Object 1A03h: PDO_TxMappParam_03h_AU64
        EPL_OBD_BEGIN_INDEX_RAM(0x1A03, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x00, 0x05, 0x03, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x01, 0x1B, 0x03, tEplObdUnsigned64, ObjectMapping, 0x0008000000012000LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x02, 0x1B, 0x03, tEplObdUnsigned64, ObjectMapping, 0x0008000800022000LL)
        EPL_OBD_END_INDEX(0x1A03)

#endif

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
        // Object 1C00h: DLL_MNCRCError_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1C00, 0x04, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1C00, 0x00, 0x05, 0x01, tEplObdUnsigned8, NumberOfEntries, 0x03)
            EPL_OBD_SUBINDEX_RAM_USERDEF_NOINIT(0x1C00, 0x01, 0x07, 0x03, tEplObdUnsigned32, CumulativeCnt_U32)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x1C00, 0x02, 0x07, 0x01, tEplObdUnsigned32, ThresholdCnt_U32, 0x0)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x1C00, 0x03, 0x07, 0x03, tEplObdUnsigned32, Threshold_U32, 0x1)
        EPL_OBD_END_INDEX(0x1C0F)

        // Object 1C02h: DLL_MNCycTimeExceed_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1C02, 0x04, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1C02, 0x00, 0x05, 0x01, tEplObdUnsigned8, NumberOfEntries, 0x03)
            EPL_OBD_SUBINDEX_RAM_USERDEF_NOINIT(0x1C02, 0x01, 0x07, 0x03, tEplObdUnsigned32, CumulativeCnt_U32)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x1C02, 0x02, 0x07, 0x01, tEplObdUnsigned32, ThresholdCnt_U32, 0x0)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x1C02, 0x03, 0x07, 0x03, tEplObdUnsigned32, Threshold_U32, 0x1)
        EPL_OBD_END_INDEX(0x1C0F)

        // Object 1C07h: DLL_MNCNLossPResCumCnt_AU32
        EPL_OBD_RAM_INDEX_RAM_VARARRAY_NOINIT(0x1C07, 254, NULL, 0x07, 0x03, tEplObdUnsigned32, DLL_MNCNLossPResCumCnt_AU32)

        // Object 1C08h: DLL_MNCNLossPResThrCnt_AU32
        EPL_OBD_RAM_INDEX_RAM_VARARRAY(0x1C08, 254, NULL, 0x07, 0x01, tEplObdUnsigned32, DLL_MNCNLossPResThrCnt_AU32, 0)

        // Object 1C09h: DLL_MNCNLossPResThreshold_AU32
        EPL_OBD_RAM_INDEX_RAM_VARARRAY(0x1C09, 254, NULL, 0x07, 0x03, tEplObdUnsigned32, DLL_MNCNLossPResThreshold_AU32, 15)
#endif

        // Object 1C0Bh: DLL_CNLossSoC_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1C0B, 0x04, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1C0B, 0x00, 0x05, 0x01, tEplObdUnsigned8, NumberOfEntries, 0x03)
            EPL_OBD_SUBINDEX_RAM_USERDEF_NOINIT(0x1C0B, 0x01, 0x07, 0x03, tEplObdUnsigned32, CumulativeCnt_U32)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x1C0B, 0x02, 0x07, 0x01, tEplObdUnsigned32, ThresholdCnt_U32, 0x0)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x1C0B, 0x03, 0x07, 0x03, tEplObdUnsigned32, Threshold_U32, 0x1)
        EPL_OBD_END_INDEX(0x1C0B)

        // Object 1C0Dh: DLL_CNLossPReq_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1C0D, 0x04, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1C0D, 0x00, 0x05, 0x01, tEplObdUnsigned8, NumberOfEntries, 0x03)
            EPL_OBD_SUBINDEX_RAM_USERDEF_NOINIT(0x1C0D, 0x01, 0x07, 0x03, tEplObdUnsigned32, CumulativeCnt_U32)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x1C0D, 0x02, 0x07, 0x01, tEplObdUnsigned32, ThresholdCnt_U32, 0x0)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x1C0D, 0x03, 0x07, 0x03, tEplObdUnsigned32, Threshold_U32, 0x1)
        EPL_OBD_END_INDEX(0x1C0D)

        // Object 1C0Fh: DLL_CNCRCError_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1C0F, 0x04, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1C0F, 0x00, 0x05, 0x01, tEplObdUnsigned8, NumberOfEntries, 0x03)
            EPL_OBD_SUBINDEX_RAM_USERDEF_NOINIT(0x1C0F, 0x01, 0x07, 0x03, tEplObdUnsigned32, CumulativeCnt_U32)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x1C0F, 0x02, 0x07, 0x01, tEplObdUnsigned32, ThresholdCnt_U32, 0x0)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x1C0F, 0x03, 0x07, 0x03, tEplObdUnsigned32, Threshold_U32, 0x1)
        EPL_OBD_END_INDEX(0x1C0F)

        // Object 1C14h: DLL_LossOfFrameTolerance_U32 in [ns]
        EPL_OBD_BEGIN_INDEX_RAM(0x1C14, 0x01, EplApiCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1C14, 0x00, 0x07, 0x03, tEplObdUnsigned32, LossOfFrameTolerance, 300000)
        EPL_OBD_END_INDEX(0x1C14)

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_VETH)) != 0)
        // Object 1E40h: NWL_IpAddrTable_0h_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1E40, 0x06, EplApiCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1E40, 0x00, 0x05, 0x01, tEplObdUnsigned8, NumberOfEntries, 0x05)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1E40, 0x01, 0x06, 0x01, tEplObdUnsigned16, IfIndex_U16, 0x01)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1E40, 0x02, 0x07, 0x01, tEplObdUnsigned32, Addr_IPAD, 0xC0A86401)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1E40, 0x03, 0x07, 0x01, tEplObdUnsigned32, NetMask_IPAD, 0xFFFFFF00)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1E40, 0x04, 0x06, 0x01, tEplObdUnsigned16, ReasmMaxSize_U16, 50000)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1E40, 0x05, 0x07, 0x03, tEplObdUnsigned32, DefaultGateway_IPAD, 0xC0A864FE)
        EPL_OBD_END_INDEX(0x1E40)
#endif

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
        // Object 1F80h: NMT_StartUp_U32
        EPL_OBD_BEGIN_INDEX_RAM(0x1F80, 0x01, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F80, 0x00, 0x07, 0x03, tEplObdUnsigned32, NMT_StartUp_U32, 0x00)
        EPL_OBD_END_INDEX(0x1F80)

        // Object 1F81h: NMT_NodeAssignment_AU32
        EPL_OBD_RAM_INDEX_RAM_ARRAY(0x1F81, 254, NULL, 0x07, 0x03, tEplObdUnsigned32, NMT_NodeAssignment_AU32, 0)
#endif

        // Object 1F82h: NMT_FeatureFlags_U32
        EPL_OBD_BEGIN_INDEX_RAM(0x1F82, 0x01, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F82, 0x00, 0x07, 0x01, tEplObdUnsigned32, NMT_FeatureFlags_U32, EPL_DEF_FEATURE_FLAGS)
        EPL_OBD_END_INDEX(0x1F82)

        // Object 1F83h: NMT_EPLVersion_U8
        EPL_OBD_BEGIN_INDEX_RAM(0x1F83, 0x01, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F83, 0x00, 0x05, 0x04, tEplObdUnsigned8, NMT_EPLVersion_U8, 0x20)
        EPL_OBD_END_INDEX(0x1F83)

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
        // Object 1F84h: NMT_MNDeviceTypeIdList_AU32
        EPL_OBD_RAM_INDEX_RAM_ARRAY(0x1F84, 254, NULL, 0x07, 0x03, tEplObdUnsigned32, NMT_MNDeviceTypeIdList_AU32, 0)

        // Object 1F89h: NMT_BootTime_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1F89, 0x06, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F89, 0x00, 0x05, 0x01, tEplObdUnsigned8, NumberOfEntries, 0x05)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F89, 0x01, 0x07, 0x03, tEplObdUnsigned32, MNWaitNotAct_U32, 1000000)        // in [us]
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F89, 0x02, 0x07, 0x03, tEplObdUnsigned32, MNTimeoutPreOp1_U32, 500000)      // in [us]
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F89, 0x03, 0x07, 0x03, tEplObdUnsigned32, MNWaitPreOp1_U32, 1000000)         // in [us]
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F89, 0x04, 0x07, 0x03, tEplObdUnsigned32, MNTimeoutPreOp2_U32, 500000)      // in [us]
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F89, 0x05, 0x07, 0x03, tEplObdUnsigned32, MNTimeoutReadyToOp_U32, 5000000)   // in [us]
        EPL_OBD_END_INDEX(0x1F89)

        // Object 1F8Ah: NMT_MNCycleTiming_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1F8A, 0x06, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F8A, 0x00, 0x05, 0x01, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F8A, 0x01, 0x07, 0x03, tEplObdUnsigned32, WaitSoCPReq_U32, 1000)    // in [ns]
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F8A, 0x02, 0x07, 0x03, tEplObdUnsigned32, AsyncSlotTimeout_U32, 100000) // in [ns]
        EPL_OBD_END_INDEX(0x1F8A)

        // Object 1F8Bh: NMT_MNPReqPayloadLimitList_AU16
        EPL_OBD_RAM_INDEX_RAM_ARRAY(0x1F8B, 254, NULL, 0x06, 0x03, tEplObdUnsigned16, NMT_MNPReqPayloadLimitList_AU16, 36)
#endif

        // Object 1F8Ch: NMT_CurrNMTState_U8
        EPL_OBD_BEGIN_INDEX_RAM(0x1F8C, 0x01, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F8C, 0x00, 0x05, 0x09, tEplObdUnsigned8, NMT_CurrNMTState_U8, 0x00)
        EPL_OBD_END_INDEX(0x1F8C)

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
        // Object 1F8Dh: NMT_PResPayloadLimitList_AU16
        EPL_OBD_RAM_INDEX_RAM_ARRAY(0x1F8D, 254, NULL, 0x06, 0x03, tEplObdUnsigned16, NMT_PResPayloadLimitList_AU16, 36)

        // Object 1F8Eh: NMT_MNNodeCurrState_AU8
        EPL_OBD_RAM_INDEX_RAM_ARRAY(0x1F8E, 254, NULL, 0x05, 0x01, tEplObdUnsigned8, NMT_MNNodeCurrState_AU8, 0x1C)

        // Object 1F8Fh: NMT_MNNodeExpState_AU8
        EPL_OBD_RAM_INDEX_RAM_ARRAY(0x1F8F, 254, NULL, 0x05, 0x01, tEplObdUnsigned8, NMT_MNNodeExpState_AU8, 0x1C)

        // Object 1F92h: NMT_MNCNPResTimeout_AU32 in [ns]
        EPL_OBD_RAM_INDEX_RAM_ARRAY(0x1F92, 254, NULL, 0x07, 0x03, tEplObdUnsigned32, NMT_MNCNPResTimeout_AU32, 140000) // in [ns]
#endif

        // Object 1F93h: NMT_EPLNodeID_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1F93, 0x03, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F93, 0x00, 0x05, 0x04, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F93, 0x01, 0x05, 0x01, tEplObdUnsigned8, NodeID_U8, 0x00)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F93, 0x02, 0x01, 0x01, tEplObdBoolean, NodeIDByHW_BOOL, 0x0)
        EPL_OBD_END_INDEX(0x1F93)

        // Object 1F98h: NMT_CycleTiming_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1F98, 0x09, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F98, 0x00, 0x05, 0x01, tEplObdUnsigned8, NumberOfEntries, 0x08)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F98, 0x01, 0x06, 0x01, tEplObdUnsigned16, IsochrTxMaxPayload_U16, 0x00)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F98, 0x02, 0x06, 0x01, tEplObdUnsigned16, IsochrRxMaxPayload_U16, 0x00)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F98, 0x03, 0x07, 0x01, tEplObdUnsigned32, PResMaxLatency_U32, 0x00)     // in [ns]
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F98, 0x04, 0x06, 0x03, tEplObdUnsigned16, PReqActPayloadLimit_U16, 36)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F98, 0x05, 0x06, 0x03, tEplObdUnsigned16, PResActPayloadLimit_U16, 36)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F98, 0x06, 0x07, 0x01, tEplObdUnsigned32, ASndMaxLatency_U32, 0x00)     // in [ns]
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F98, 0x07, 0x05, 0x03, tEplObdUnsigned8, MultiplCycleCnt_U8, 0x00)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F98, 0x08, 0x06, 0x03, tEplObdUnsigned16, AsyncMTU_U16, EPL_C_DLL_MIN_ASYNC_MTU)
//            EPL_OBD_SUBINDEX_RAM_VAR(0x1F98, 0x09, 0x06, 0x03, tEplObdUnsigned16, Prescaler_U16, 0x02)
        EPL_OBD_END_INDEX(0x1F98)

        // Object 1F99h: NMT_CNBasicEthernetTimeout_U32 in [us]
        EPL_OBD_BEGIN_INDEX_RAM(0x1F99, 0x01, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F99, 0x00, 0x07, 0x03, tEplObdUnsigned32, NMT_CNBasicEthernetTimeout_U32, 5000000)  // in [us]
        EPL_OBD_END_INDEX(0x1F99)

        // Object 1F9Ah: NMT_HostName_VS
        EPL_OBD_BEGIN_INDEX_RAM(0x1F9A, 0x01, NULL)
           EPL_OBD_SUBINDEX_RAM_VSTRING(0x1F9A, 0x00, 0x01, host_name, 33, "")
        EPL_OBD_END_INDEX(0x1F9A)

        // Object 1F9Eh: NMT_ResetCmd_U8
        EPL_OBD_BEGIN_INDEX_RAM(0x1F9E, 0x01, EplApiCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F9E, 0x00, 0x05, 0x03, tEplObdUnsigned8, NMT_ResetCmd_U8, 0xFF)
        EPL_OBD_END_INDEX(0x1F9E)

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
        // Object 1F9Fh: NMT_RequestCmd_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1F9F, 0x04, EplApiCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F9F, 0x00, 0x05, 0x01, tEplObdUnsigned8, NumberOfEntries, 0x03)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F9F, 0x01, 0x01, 0x03, tEplObdBoolean, Release_BOOL, FALSE)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F9F, 0x02, 0x05, 0x03, tEplObdUnsigned8, CmdID_U8, 0xFF)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F9F, 0x03, 0x05, 0x03, tEplObdUnsigned8, CmdTarget_U8, 0x00)
        EPL_OBD_END_INDEX(0x1F9F)
#endif

    EPL_OBD_END_PART ()

    EPL_OBD_BEGIN_PART_MANUFACTURER ()

#if EPL_API_PROCESS_IMAGE_SIZE_IN > 0
        // static input process image
        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0x2000, (EPL_API_PROCESS_IMAGE_SIZE_IN), NULL, kEplObdTypUInt8, kEplObdAccVPRW, tEplObdUnsigned8, BYTE_Merker, 0x00)
        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0x2001, (EPL_API_PROCESS_IMAGE_SIZE_IN), NULL, kEplObdTypInt8, kEplObdAccVPRW, tEplObdInteger8, SHORT_Merker, 0x00)
        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0x2010, (EPL_API_PROCESS_IMAGE_SIZE_IN / 2), NULL, kEplObdTypUInt16, kEplObdAccVPRW, tEplObdUnsigned16, WORD_Merker, 0)
        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0x2011, (EPL_API_PROCESS_IMAGE_SIZE_IN / 2), NULL, kEplObdTypInt16, kEplObdAccVPRW, tEplObdInteger16, INT_Merker, 0)
        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0x2020, (EPL_API_PROCESS_IMAGE_SIZE_IN / 4), NULL, kEplObdTypUInt32, kEplObdAccVPRW, tEplObdUnsigned32, DWORD_Merker, 0)
        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0x2021, (EPL_API_PROCESS_IMAGE_SIZE_IN / 4), NULL, kEplObdTypInt32, kEplObdAccVPRW, tEplObdInteger32, LINT_Merker, 0)
#endif

#if EPL_API_PROCESS_IMAGE_SIZE_OUT > 0
        // static output process image
        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0x2030, (EPL_API_PROCESS_IMAGE_SIZE_OUT), NULL, kEplObdTypUInt8, kEplObdAccVPR, tEplObdUnsigned8, BYTE_Merker, 0x00)
        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0x2031, (EPL_API_PROCESS_IMAGE_SIZE_OUT), NULL, kEplObdTypInt8, kEplObdAccVPR, tEplObdInteger8, SHORT_Merker, 0x00)
        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0x2040, (EPL_API_PROCESS_IMAGE_SIZE_OUT / 2), NULL, kEplObdTypUInt16, kEplObdAccVPR, tEplObdUnsigned16, WORD_Merker, 0)
        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0x2041, (EPL_API_PROCESS_IMAGE_SIZE_OUT / 2), NULL, kEplObdTypInt16, kEplObdAccVPR, tEplObdInteger16, INT_Merker, 0)
        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0x2050, (EPL_API_PROCESS_IMAGE_SIZE_OUT / 4), NULL, kEplObdTypUInt32, kEplObdAccVPR, tEplObdUnsigned32, DWORD_Merker, 0)
        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0x2051, (EPL_API_PROCESS_IMAGE_SIZE_OUT / 4), NULL, kEplObdTypInt32, kEplObdAccVPR, tEplObdInteger32, LINT_Merker, 0)
#endif

#if (!((EPL_API_PROCESS_IMAGE_SIZE_IN > 0) || (EPL_API_PROCESS_IMAGE_SIZE_OUT > 0))) \
    && (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
        // process image is not enabled but master is enabled

        // output variables of master
        EPL_OBD_BEGIN_INDEX_RAM(0x2000, 0x05, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x2000, 0x00, 0x05, 0x01, tEplObdUnsigned8, number_of_entries, 0x4)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x2000, 0x01, 0x05, 0x0B, tEplObdUnsigned8, Sendb1, 0x0)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x2000, 0x02, 0x05, 0x0B, tEplObdUnsigned8, Sendb2, 0x0)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x2000, 0x03, 0x05, 0x0B, tEplObdUnsigned8, Sendb3, 0x0)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x2000, 0x04, 0x05, 0x0B, tEplObdUnsigned8, Sendb4, 0x0)
        EPL_OBD_END_INDEX(0x2000)

        // input variables of master
        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0x2200, 3, NULL, kEplObdTypUInt8, kEplObdAccVPRW, tEplObdUnsigned8, Receivebx, 0x00)
#endif

    EPL_OBD_END_PART ()

    EPL_OBD_BEGIN_PART_DEVICE ()

#if !((EPL_API_PROCESS_IMAGE_SIZE_IN > 0) || (EPL_API_PROCESS_IMAGE_SIZE_OUT > 0))
        EPL_OBD_BEGIN_INDEX_RAM(0x6000, 0x02, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x6000, 0x00, 0x05, 0x01, tEplObdUnsigned8, number_of_entries, 0x1)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x6000, 0x01, 0x05, 0x09, tEplObdUnsigned8, Sendb1, 0x0)
        EPL_OBD_END_INDEX(0x6000)

        EPL_OBD_BEGIN_INDEX_RAM(0x6100, 0x01, NULL)
            EPL_OBD_SUBINDEX_RAM_DOMAIN(0x6100,0x00,0x03,TEST_DOMAIN)
        EPL_OBD_END_INDEX(0x6100)

        EPL_OBD_BEGIN_INDEX_RAM(0x6200, 0x02, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x6200, 0x00, 0x05, 0x01, tEplObdUnsigned8, number_of_entries, 0x1)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x6200, 0x01, 0x05, 0x0B, tEplObdUnsigned8, Recvb1, 0x0)
        EPL_OBD_END_INDEX(0x6200)
#endif

    EPL_OBD_END_PART ()

EPL_OBD_END ()

#define EPL_OBD_UNDEFINE_MACRO
    #include "EplObdMacro.h"
#undef EPL_OBD_UNDEFINE_MACRO
