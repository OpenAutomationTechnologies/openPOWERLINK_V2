//-----------------------------------------------------------------
//  OD for minimal EPL CN
//-----------------------------------------------------------------
#define EPL_OBD_DEFINE_MACRO
    #include "EplObdMacro.h"
#undef EPL_OBD_DEFINE_MACRO

EPL_OBD_BEGIN ()

    EPL_OBD_BEGIN_PART_GENERIC ()

        #include "../Generic/objdict_1000-13ff.h"


        // Object 1400h: PDO_RxCommParam_00h_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1400, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1400, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1400, 0x01, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NodeID_U8, 0x00)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1400, 0x02, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, MappingVersion_U8, 0x00)
        EPL_OBD_END_INDEX(0x1400)

        // Object 1401h: PDO_RxCommParam_01h_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1401, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1401, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1401, 0x01, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NodeID_U8, 0x6E)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1401, 0x02, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, MappingVersion_U8, 0x00)
        EPL_OBD_END_INDEX(0x1401)

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
        // additional RxPDOs if master is enabled

        // Object 1402h: PDO_RxCommParam_02h_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1402, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1402, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1402, 0x01, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NodeID_U8, 0x01)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1402, 0x02, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, MappingVersion_U8, 0x00)
        EPL_OBD_END_INDEX(0x1402)

        // Object 1403h: PDO_RxCommParam_03h_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1403, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1403, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1403, 0x01, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NodeID_U8, 0x20)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1403, 0x02, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, MappingVersion_U8, 0x00)
        EPL_OBD_END_INDEX(0x1403)

        // Object 1404h: PDO_RxCommParam_04h_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1404, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1404, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1404, 0x01, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NodeID_U8, 0x05)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1404, 0x02, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, MappingVersion_U8, 0x00)
        EPL_OBD_END_INDEX(0x1404)

        // Object 1405h: PDO_RxCommParam_05h_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1405, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1405, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1405, 0x01, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NodeID_U8, 0x06)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1405, 0x02, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, MappingVersion_U8, 0x00)
        EPL_OBD_END_INDEX(0x1405)

        // Object 1406h: PDO_RxCommParam_06h_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1406, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1406, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1406, 0x01, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NodeID_U8, 0x07)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1406, 0x02, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, MappingVersion_U8, 0x00)
        EPL_OBD_END_INDEX(0x1406)

        // Object 1407h: PDO_RxCommParam_07h_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1407, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1407, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1407, 0x01, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NodeID_U8, 0x08)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1407, 0x02, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, MappingVersion_U8, 0x00)
        EPL_OBD_END_INDEX(0x1407)

        // Object 1408h: PDO_RxCommParam_08h_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1408, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1408, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1408, 0x01, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NodeID_U8, 0x09)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1408, 0x02, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, MappingVersion_U8, 0x00)
        EPL_OBD_END_INDEX(0x1408)

        // Object 1409h: PDO_RxCommParam_09h_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1409, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1409, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1409, 0x01, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NodeID_U8, 0x0A)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1409, 0x02, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, MappingVersion_U8, 0x00)
        EPL_OBD_END_INDEX(0x1409)

        // Object 140Ah: PDO_RxCommParam_0Ah_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x140A, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x140A, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x140A, 0x01, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NodeID_U8, 0x0B)
            EPL_OBD_SUBINDEX_RAM_VAR(0x140A, 0x02, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, MappingVersion_U8, 0x00)
        EPL_OBD_END_INDEX(0x140A)

        // Object 140Bh: PDO_RxCommParam_0Bh_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x140B, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x140B, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x140B, 0x01, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NodeID_U8, 0x0C)
            EPL_OBD_SUBINDEX_RAM_VAR(0x140B, 0x02, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, MappingVersion_U8, 0x00)
        EPL_OBD_END_INDEX(0x140B)

#endif

        // Object 1600h: PDO_RxMappParam_00h_AU64
        EPL_OBD_BEGIN_INDEX_RAM(0x1600, 0x0D, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x00, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NumberOfEntries, 0x01)
//#if ((EPL_API_PROCESS_IMAGE_SIZE_IN > 0) || (EPL_API_PROCESS_IMAGE_SIZE_OUT > 0))
//            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x01, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0008000000012000LL)
//#else
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x01, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0008000000016200LL)
//#endif
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x02, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0008000800012700LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x03, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0008001000022700LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x04, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0008001800032700LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x05, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020002000012701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x06, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020004000022701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x07, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020006000032701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x08, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020008000042701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x09, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002000A000052701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x0A, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002000C000062701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x0B, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002000E000072701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x0C, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020010000082701LL)
        EPL_OBD_END_INDEX(0x1600)

        // Object 1601h: PDO_RxMappParam_01h_AU64
        EPL_OBD_BEGIN_INDEX_RAM(0x1601, 0x0D, EplPdouCbObdAccess)
#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
            // enable this RxPDO if master is enabled
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x00, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NumberOfEntries, 0x01)
#else
            // otherwise the corresponding object does not exist
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x00, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NumberOfEntries, 0x00)
#endif
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x01, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0008000000012200LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x02, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0008000800042700LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x03, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0008001000052700LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x04, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0008001800062700LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x05, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020002000092701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x06, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00200040000A2701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x07, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00200060000B2701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x08, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00200080000C2701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x09, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002000A0000D2701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x0A, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002000C0000E2701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x0B, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002000E0000F2701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x0C, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020010000102701LL)
        EPL_OBD_END_INDEX(0x1601)

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
        // additional RxPDOs if master is enabled

        // Object 1602h: PDO_RxMappParam_02h_AU64
        EPL_OBD_BEGIN_INDEX_RAM(0x1602, 0x0D, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x00, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NumberOfEntries, 0x01)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x01, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0008000000022200LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x02, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0008000800072700LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x03, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0008001000082700LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x04, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0008001800092700LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x05, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020002000112701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x06, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020004000122701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x07, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020006000132701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x08, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020008000142701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x09, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002000A000152701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x0A, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002000C000162701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x0B, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002000E000172701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x0C, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020010000182701LL)
        EPL_OBD_END_INDEX(0x1602)

        // Object 1603h: PDO_RxMappParam_03h_AU64
        EPL_OBD_BEGIN_INDEX_RAM(0x1603, 0x0D, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x00, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NumberOfEntries, 0x01)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x01, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0008000000032200LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x02, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00080008000A2700LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x03, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00080010000B2700LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x04, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00080018000C2700LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x05, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020002000192701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x06, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00200040001A2701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x07, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00200060001B2701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x08, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00200080001C2701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x09, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002000A0001D2701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x0A, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002000C0001E2701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x0B, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002000E0001F2701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x0C, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020010000202701LL)
        EPL_OBD_END_INDEX(0x1603)

        // Object 1604h: PDO_RxMappParam_04h_AU64
        EPL_OBD_BEGIN_INDEX_RAM(0x1604, 0x0A, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1604, 0x00, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NumberOfEntries, 0x00)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1604, 0x01, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020000000212701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1604, 0x02, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020002000222701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1604, 0x03, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020004000232701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1604, 0x04, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020006000242701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1604, 0x05, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020008000252701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1604, 0x06, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002000A000262701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1604, 0x07, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002000C000272701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1604, 0x08, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002000E000282701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1604, 0x09, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020010000292701LL)
        EPL_OBD_END_INDEX(0x1604)

        // Object 1605h: PDO_RxMappParam_05h_AU64
        EPL_OBD_BEGIN_INDEX_RAM(0x1605, 0x0A, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1605, 0x00, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NumberOfEntries, 0x00)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1605, 0x01, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00200000002A2701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1605, 0x02, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00200020002B2701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1605, 0x03, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00200040002C2701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1605, 0x04, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00200060002D2701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1605, 0x05, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00200080002E2701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1605, 0x06, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002000A0002F2701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1605, 0x07, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002000C000302701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1605, 0x08, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002000E000312701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1605, 0x09, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020010000322701LL)
        EPL_OBD_END_INDEX(0x1605)

        // Object 1606h: PDO_RxMappParam_06h_AU64
        EPL_OBD_BEGIN_INDEX_RAM(0x1606, 0x0A, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1606, 0x00, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NumberOfEntries, 0x00)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1606, 0x01, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020000000332701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1606, 0x02, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020002000342701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1606, 0x03, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020004000352701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1606, 0x04, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020006000362701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1606, 0x05, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020008000372701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1606, 0x06, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002000A000382701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1606, 0x07, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002000C000392701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1606, 0x08, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002000E0003A2701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1606, 0x09, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00200100003B2701LL)
        EPL_OBD_END_INDEX(0x1606)

        // Object 1607h: PDO_RxMappParam_07h_AU64
        EPL_OBD_BEGIN_INDEX_RAM(0x1607, 0x0A, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1607, 0x00, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NumberOfEntries, 0x00)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1607, 0x01, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00200000003C2701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1607, 0x02, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00200020003D2701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1607, 0x03, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00200040003E2701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1607, 0x04, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00200060003F2701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1607, 0x05, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020008000402701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1607, 0x06, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002000A000412701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1607, 0x07, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002000C000422701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1607, 0x08, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002000E000432701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1607, 0x09, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020010000442701LL)
        EPL_OBD_END_INDEX(0x1607)

        // Object 1608h: PDO_RxMappParam_08h_AU64
        EPL_OBD_BEGIN_INDEX_RAM(0x1608, 0x0A, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1608, 0x00, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NumberOfEntries, 0x00)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1608, 0x01, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020000000452701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1608, 0x02, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020002000462701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1608, 0x03, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020004000472701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1608, 0x04, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020006000482701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1608, 0x05, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020008000492701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1608, 0x06, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002000A0004A2701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1608, 0x07, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002000C0004B2701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1608, 0x08, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002000E0004C2701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1608, 0x09, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00200100004D2701LL)
        EPL_OBD_END_INDEX(0x1608)

        // Object 1609h: PDO_RxMappParam_09h_AU64
        EPL_OBD_BEGIN_INDEX_RAM(0x1609, 0x0A, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1609, 0x00, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NumberOfEntries, 0x00)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1609, 0x01, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00200000004E2701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1609, 0x02, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00200020004F2701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1609, 0x03, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020004000502701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1609, 0x04, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020006000512701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1609, 0x05, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020008000522701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1609, 0x06, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002000A000532701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1609, 0x07, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002000C000542701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1609, 0x08, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002000E000552701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1609, 0x09, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020010000562701LL)
        EPL_OBD_END_INDEX(0x1609)

        // Object 160Ah: PDO_RxMappParam_0Ah_AU64
        EPL_OBD_BEGIN_INDEX_RAM(0x160A, 0x0A, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x160A, 0x00, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NumberOfEntries, 0x00)
            EPL_OBD_SUBINDEX_RAM_VAR(0x160A, 0x01, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020000000572701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x160A, 0x02, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020002000582701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x160A, 0x03, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020004000592701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x160A, 0x04, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00200060005A2701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x160A, 0x05, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00200080005B2701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x160A, 0x06, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002000A0005C2701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x160A, 0x07, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002000C0005D2701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x160A, 0x08, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002000E0005E2701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x160A, 0x09, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00200100005F2701LL)
        EPL_OBD_END_INDEX(0x160A)

        // Object 160Bh: PDO_RxMappParam_0Bh_AU64
        EPL_OBD_BEGIN_INDEX_RAM(0x160B, 0x0A, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x160B, 0x00, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NumberOfEntries, 0x00)
            EPL_OBD_SUBINDEX_RAM_VAR(0x160B, 0x01, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020000000602701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x160B, 0x02, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020002000612701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x160B, 0x03, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020004000622701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x160B, 0x04, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020006000632701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x160B, 0x05, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020008000642701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x160B, 0x06, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002000A000652701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x160B, 0x07, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002000C000662701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x160B, 0x08, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002000E000672701LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x160B, 0x09, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020010000682701LL)
        EPL_OBD_END_INDEX(0x160B)

#endif

        // Object 1800h: PDO_TxCommParam_00h_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1800, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1800, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1800, 0x01, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NodeID_U8, 0x00)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1800, 0x02, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, MappingVersion_U8, 0x00)
        EPL_OBD_END_INDEX(0x1800)

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
        // additional TxPDOs if master is enabled

        // Object 1801h: PDO_TxCommParam_01h_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1801, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1801, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1801, 0x01, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NodeID_U8, 0x01)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1801, 0x02, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, MappingVersion_U8, 0x00)
        EPL_OBD_END_INDEX(0x1801)

        // Object 1802h: PDO_TxCommParam_02h_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1802, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1802, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1802, 0x01, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NodeID_U8, 0x20)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1802, 0x02, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, MappingVersion_U8, 0x00)
        EPL_OBD_END_INDEX(0x1802)

        // Object 1803h: PDO_TxCommParam_03h_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1803, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1803, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1803, 0x01, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NodeID_U8, 0x6E)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1803, 0x02, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, MappingVersion_U8, 0x00)
        EPL_OBD_END_INDEX(0x1803)

#endif

        // Object 1A00h: PDO_TxMappParam_00h_AU64
        EPL_OBD_BEGIN_INDEX_RAM(0x1A00, 0x5E, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x00, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NumberOfEntries, 0x01)
//#if ((EPL_API_PROCESS_IMAGE_SIZE_IN > 0) || (EPL_API_PROCESS_IMAGE_SIZE_OUT > 0))
//            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x01, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0008000000012030LL)
//#else
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x01, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0008000000016000LL)
//#endif
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x02, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0008000800012600LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x03, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0008001000022600LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x04, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0008001800032600LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x05, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020002000012601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x06, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020004000022601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x07, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020006000032601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x08, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020008000042601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x09, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002000A000052601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x0A, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002000C000062601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x0B, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002000E000072601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x0C, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020010000082601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x0D, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020012000092601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x0E, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00200140000A2601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x0F, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00200160000B2601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x10, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00200180000C2601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x11, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002001A0000D2601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x12, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002001C0000E2601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x13, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002001E0000F2601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x14, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020020000102601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x15, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020022000112601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x16, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020024000122601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x17, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020026000132601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x18, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020028000142601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x19, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002002A000152601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x1A, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002002C000162601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x1B, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002002E000172601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x1C, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020030000182601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x1D, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020032000192601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x1E, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00200340001A2601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x1F, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00200360001B2601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x20, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00200380001C2601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x21, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002003A0001D2601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x22, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002003C0001E2601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x23, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002003E0001F2601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x24, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020040000202601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x25, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020042000212601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x26, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020044000222601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x27, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020046000232601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x28, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020048000242601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x29, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002004A000252601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x2A, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002004C000262601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x2B, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002004E000272601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x2C, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020050000282601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x2D, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020052000292601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x2E, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00200540002A2601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x2F, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00200560002B2601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x30, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00200580002C2601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x31, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002005A0002D2601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x32, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002005C0002E2601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x33, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002005E0002F2601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x34, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020060000302601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x35, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020062000312601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x36, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020064000322601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x37, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020066000332601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x38, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020068000342601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x39, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002006A000352601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x3A, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002006C000362601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x3B, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002006E000372601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x3C, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020070000382601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x3D, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020072000392601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x3E, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00200740003A2601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x3F, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00200760003B2601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x40, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00200780003C2601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x41, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002007A0003D2601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x42, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002007C0003E2601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x43, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002007E0003F2601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x44, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020080000402601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x45, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020082000412601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x46, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020084000422601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x47, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020086000432601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x48, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020088000442601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x49, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002008A000452601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x4A, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002008C000462601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x4B, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002008E000472601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x4C, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020090000482601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x4D, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020092000492601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x4E, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00200940004A2601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x4F, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00200960004B2601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x50, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00200980004C2601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x51, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002009A0004D2601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x52, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002009C0004E2601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x53, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002009E0004F2601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x54, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00200A0000502601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x55, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00200A2000512601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x56, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00200A4000522601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x57, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00200A6000532601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x58, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00200A8000542601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x59, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00200AA000552601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x5A, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00200AC000562601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x5B, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00200AE000572601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x5C, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00200B0000582601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x5D, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00200B2000592601LL)
        EPL_OBD_END_INDEX(0x1A00)

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
        // additional TxPDOs if master is enabled

        // Object 1A01h: PDO_TxMappParam_01h_AU64
        EPL_OBD_BEGIN_INDEX_RAM(0x1A01, 0x0D, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x00, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x01, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0008000000022000LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x02, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0008000800012000LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x03, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0008001000042600LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x04, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0008001800052600LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x05, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00200020003F2601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x06, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020004000402601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x07, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020006000412601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x08, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020008000422601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x09, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002000A000432601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x0A, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002000C000442601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x0B, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002000E000452601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x0C, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020010000462601LL)
        EPL_OBD_END_INDEX(0x1A01)

        // Object 1A02h: PDO_TxMappParam_02h_AU64
        EPL_OBD_BEGIN_INDEX_RAM(0x1A02, 0x0D, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x00, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x01, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0008000000022000LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x02, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0008000800012000LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x03, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0008001000062600LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x04, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0008001800072600LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x05, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020002000472601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x06, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020004000482601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x07, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0020006000492601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x08, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00200080004A2601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x09, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002000A0004B2601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x0A, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002000C0004C2601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x0B, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x002000E0004D2601LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x0C, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00200100004E2601LL)
        EPL_OBD_END_INDEX(0x1A02)

        // Object 1A03h: PDO_TxMappParam_03h_AU64
        EPL_OBD_BEGIN_INDEX_RAM(0x1A03, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x00, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x01, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0008000000012000LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x02, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0008000800022000LL)
        EPL_OBD_END_INDEX(0x1A03)

#endif


        #include "../Generic/objdict_1b00-1fff.h"

    EPL_OBD_END_PART ()

    EPL_OBD_BEGIN_PART_MANUFACTURER ()

//#if EPL_API_PROCESS_IMAGE_SIZE_IN > 0
        // static input process image
//        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0x2000, (EPL_API_PROCESS_IMAGE_SIZE_IN), NULL, kEplObdTypUInt8, kEplObdAccVPRW, tEplObdUnsigned8, BYTE_Merker, 0x00)
//        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0x2001, (EPL_API_PROCESS_IMAGE_SIZE_IN), NULL, kEplObdTypInt8, kEplObdAccVPRW, tEplObdInteger8, SHORT_Merker, 0x00)
//        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0x2010, (EPL_API_PROCESS_IMAGE_SIZE_IN / 2), NULL, kEplObdTypUInt16, kEplObdAccVPRW, tEplObdUnsigned16, WORD_Merker, 0)
//        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0x2011, (EPL_API_PROCESS_IMAGE_SIZE_IN / 2), NULL, kEplObdTypInt16, kEplObdAccVPRW, tEplObdInteger16, INT_Merker, 0)
//        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0x2020, (EPL_API_PROCESS_IMAGE_SIZE_IN / 4), NULL, kEplObdTypUInt32, kEplObdAccVPRW, tEplObdUnsigned32, DWORD_Merker, 0)
//        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0x2021, (EPL_API_PROCESS_IMAGE_SIZE_IN / 4), NULL, kEplObdTypInt32, kEplObdAccVPRW, tEplObdInteger32, LINT_Merker, 0)
//#endif

//#if EPL_API_PROCESS_IMAGE_SIZE_OUT > 0
        // static output process image
//        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0x2030, (EPL_API_PROCESS_IMAGE_SIZE_OUT), NULL, kEplObdTypUInt8, kEplObdAccVPR, tEplObdUnsigned8, BYTE_Merker, 0x00)
//        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0x2031, (EPL_API_PROCESS_IMAGE_SIZE_OUT), NULL, kEplObdTypInt8, kEplObdAccVPR, tEplObdInteger8, SHORT_Merker, 0x00)
//        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0x2040, (EPL_API_PROCESS_IMAGE_SIZE_OUT / 2), NULL, kEplObdTypUInt16, kEplObdAccVPR, tEplObdUnsigned16, WORD_Merker, 0)
//        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0x2041, (EPL_API_PROCESS_IMAGE_SIZE_OUT / 2), NULL, kEplObdTypInt16, kEplObdAccVPR, tEplObdInteger16, INT_Merker, 0)
//        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0x2050, (EPL_API_PROCESS_IMAGE_SIZE_OUT / 4), NULL, kEplObdTypUInt32, kEplObdAccVPR, tEplObdUnsigned32, DWORD_Merker, 0)
//        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0x2051, (EPL_API_PROCESS_IMAGE_SIZE_OUT / 4), NULL, kEplObdTypInt32, kEplObdAccVPR, tEplObdInteger32, LINT_Merker, 0)
//#endif

//#if (!((EPL_API_PROCESS_IMAGE_SIZE_IN > 0) || (EPL_API_PROCESS_IMAGE_SIZE_OUT > 0)))
//    && (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
//#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
        // process image is not enabled but master is enabled

        // output variables of master
        EPL_OBD_BEGIN_INDEX_RAM(0x2000, 0x05, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x2000, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, number_of_entries, 0x4)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x2000, 0x01, kEplObdTypUInt8, kEplObdAccVPR, tEplObdUnsigned8, Sendb1, 0x0)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x2000, 0x02, kEplObdTypUInt8, kEplObdAccVPR, tEplObdUnsigned8, Sendb2, 0x0)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x2000, 0x03, kEplObdTypUInt8, kEplObdAccVPR, tEplObdUnsigned8, Sendb3, 0x0)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x2000, 0x04, kEplObdTypUInt8, kEplObdAccVPR, tEplObdUnsigned8, Sendb4, 0x0)
        EPL_OBD_END_INDEX(0x2000)

        // input variables of master
        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0x2200, 3, NULL, kEplObdTypUInt8, kEplObdAccVPRW, tEplObdUnsigned8, Receivebx, 0x00)

        // output variables of master (extended)
        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0x2600, 7, NULL, kEplObdTypUInt8, kEplObdAccVPRW, tEplObdUnsigned8, Receivebx, 0x00)
        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0x2601, 89, NULL, kEplObdTypUInt32, kEplObdAccVPRW, tEplObdUnsigned32, Receivedwx, 0x00)

        // input variables of master (extended)
        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0x2700, 12, NULL, kEplObdTypUInt8, kEplObdAccVPRW, tEplObdUnsigned8, Receivebx, 0x00)
        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0x2701, 104, NULL, kEplObdTypUInt32, kEplObdAccVPRW, tEplObdUnsigned32, Receivedwx, 0x00)
    
//#endif

    EPL_OBD_END_PART ()

    EPL_OBD_BEGIN_PART_DEVICE ()

//#if !((EPL_API_PROCESS_IMAGE_SIZE_IN > 0) || (EPL_API_PROCESS_IMAGE_SIZE_OUT > 0))
        EPL_OBD_BEGIN_INDEX_RAM(0x6000, 0x02, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x6000, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, number_of_entries, 0x1)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x6000, 0x01, kEplObdTypUInt8, kEplObdAccVPR, tEplObdUnsigned8, Sendb1, 0x0)
        EPL_OBD_END_INDEX(0x6000)

        EPL_OBD_BEGIN_INDEX_RAM(0x6100, 0x01, NULL)
            EPL_OBD_SUBINDEX_RAM_DOMAIN(0x6100,0x00,kEplObdAccRW,TEST_DOMAIN)
        EPL_OBD_END_INDEX(0x6100)

        EPL_OBD_BEGIN_INDEX_RAM(0x6200, 0x02, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x6200, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, number_of_entries, 0x1)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x6200, 0x01, kEplObdTypUInt8, kEplObdAccVPRW, tEplObdUnsigned8, Recvb1, 0x0)
        EPL_OBD_END_INDEX(0x6200)
//#endif

    EPL_OBD_END_PART ()

EPL_OBD_END ()

#define EPL_OBD_UNDEFINE_MACRO
    #include "EplObdMacro.h"
#undef EPL_OBD_UNDEFINE_MACRO
