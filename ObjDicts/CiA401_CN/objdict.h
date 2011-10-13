//-------------------------------
//  OD for a simple DS401 slave
//-------------------------------

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
            EPL_OBD_SUBINDEX_RAM_VAR(0x1401, 0x01, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NodeID_U8, 0x00)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1401, 0x02, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, MappingVersion_U8, 0x00)
        EPL_OBD_END_INDEX(0x1401)

        // Object 1402h: PDO_RxCommParam_02h_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1402, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1402, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1402, 0x01, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NodeID_U8, 0x00)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1402, 0x02, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, MappingVersion_U8, 0x00)
        EPL_OBD_END_INDEX(0x1402)

        // Object 1600h: PDO_RxMappParam_00h_AU64
        EPL_OBD_BEGIN_INDEX_RAM(0x1600, 0x1A, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x00, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NumberOfEntries, 0x00)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x01, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x02, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x03, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x04, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x05, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x06, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x07, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x08, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x09, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x0A, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x0B, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x0C, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x0D, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x0E, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x0F, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x10, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x11, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x12, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x13, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x14, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x15, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x16, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x17, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x18, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x19, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
        EPL_OBD_END_INDEX(0x1600)

        // Object 1601h: PDO_RxMappParam_01h_AU64
        EPL_OBD_BEGIN_INDEX_RAM(0x1601, 0x1A, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x00, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NumberOfEntries, 0x00)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x01, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x02, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x03, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x04, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x05, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x06, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x07, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x08, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x09, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x0A, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x0B, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x0C, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x0D, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x0E, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x0F, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x10, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x11, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x12, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x13, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x14, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x15, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x16, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x17, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x18, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x19, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
        EPL_OBD_END_INDEX(0x1601)

		// Object 1602h: PDO_RxMappParam_02h_AU64
        EPL_OBD_BEGIN_INDEX_RAM(0x1602, 0x1A, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x00, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NumberOfEntries, 0x00)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x01, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x02, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x03, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x04, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x05, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x06, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x07, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x08, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x09, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x0A, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x0B, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x0C, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x0D, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x0E, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x0F, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x10, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x11, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x12, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x13, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x14, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x15, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x16, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x17, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x18, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x19, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
        EPL_OBD_END_INDEX(0x1602)

        // Object 1800h: PDO_TxCommParam_00h_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1800, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1800, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1800, 0x01, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NodeID_U8, 0x00)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1800, 0x02, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, MappingVersion_U8, 0x00)
        EPL_OBD_END_INDEX(0x1800)

        // Object 1A00h: PDO_TxMappParam_00h_AU64
        EPL_OBD_BEGIN_INDEX_RAM(0x1A00, 0x1A, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x00, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NumberOfEntries, 0x0)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x01, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x02, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x03, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x04, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x05, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x06, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x07, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x08, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x09, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x0A, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x0B, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x0C, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x0D, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x0E, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x0F, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x10, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x11, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x12, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x13, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x14, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x15, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x16, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x17, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x18, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x19, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00LL)
        EPL_OBD_END_INDEX(0x1A00)

        #include "../Generic/objdict_1b00-1fff.h"

    EPL_OBD_END_PART ()

    EPL_OBD_BEGIN_PART_DEVICE ()

        // DigitalInput_00h_AU8
        EPL_OBD_BEGIN_INDEX_RAM(0x6000, 0x05, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x6000, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 0x04)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x6000, 0x01, kEplObdTypUInt8, kEplObdAccVPR, tEplObdUnsigned8, DigitalInput, 0x00)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x6000, 0x02, kEplObdTypUInt8, kEplObdAccVPR, tEplObdUnsigned8, DigitalInput, 0x00)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x6000, 0x03, kEplObdTypUInt8, kEplObdAccVPR, tEplObdUnsigned8, DigitalInput, 0x00)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x6000, 0x04, kEplObdTypUInt8, kEplObdAccVPR, tEplObdUnsigned8, DigitalInput, 0x00)
        EPL_OBD_END_INDEX(0x6000)

        // DigitalOutput_00h_AU8
        EPL_OBD_BEGIN_INDEX_RAM(0x6200, 0x05, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x6200, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 0x04)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x6200, 0x01, kEplObdTypUInt8, kEplObdAccVPRW, tEplObdUnsigned8, DigitalOutput, 0x00)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x6200, 0x02, kEplObdTypUInt8, kEplObdAccVPRW, tEplObdUnsigned8, DigitalOutput, 0x00)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x6200, 0x03, kEplObdTypUInt8, kEplObdAccVPRW, tEplObdUnsigned8, DigitalOutput, 0x00)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x6200, 0x04, kEplObdTypUInt8, kEplObdAccVPRW, tEplObdUnsigned8, DigitalOutput, 0x00)
        EPL_OBD_END_INDEX(0x6200)

        // AnalogueInput_00h_AI8
        EPL_OBD_BEGIN_INDEX_RAM(0x6400, 0x05, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x6400, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 0x04)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x6400, 0x01, kEplObdTypInt8, kEplObdAccVPR, tEplObdInteger8, AnalogueInput, 0x00)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x6400, 0x02, kEplObdTypInt8, kEplObdAccVPR, tEplObdInteger8, AnalogueInput, 0x00)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x6400, 0x03, kEplObdTypInt8, kEplObdAccVPR, tEplObdInteger8, AnalogueInput, 0x00)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x6400, 0x04, kEplObdTypInt8, kEplObdAccVPR, tEplObdInteger8, AnalogueInput, 0x00)
        EPL_OBD_END_INDEX(0x6400)

		// AnalogueInput_00h_AI16
        EPL_OBD_BEGIN_INDEX_RAM(0x6401, 0x03, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x6401, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x6401, 0x01, kEplObdTypInt16, kEplObdAccVPR, tEplObdInteger16, AnalogueInput, 0x0000)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x6401, 0x02, kEplObdTypInt16, kEplObdAccVPR, tEplObdInteger16, AnalogueInput, 0x0000)
        EPL_OBD_END_INDEX(0x6401)
		
		// AnalogueInput_00h_AI32
        EPL_OBD_BEGIN_INDEX_RAM(0x6402, 0x02, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x6402, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 0x01)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x6402, 0x01, kEplObdTypInt32, kEplObdAccVPR, tEplObdInteger32, AnalogueInput, 0x00000000)
        EPL_OBD_END_INDEX(0x6401)

		// AnalogueOutput_00h_AI8
        EPL_OBD_BEGIN_INDEX_RAM(0x6410, 0x05, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x6410, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 0x04)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x6410, 0x01, kEplObdTypInt8, kEplObdAccVPRW, tEplObdInteger8, AnalogueOutput, 0x00)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x6410, 0x02, kEplObdTypInt8, kEplObdAccVPRW, tEplObdInteger8, AnalogueOutput, 0x00)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x6410, 0x03, kEplObdTypInt8, kEplObdAccVPRW, tEplObdInteger8, AnalogueOutput, 0x00)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x6410, 0x04, kEplObdTypInt8, kEplObdAccVPRW, tEplObdInteger8, AnalogueOutput, 0x00)
        EPL_OBD_END_INDEX(0x6410)

		// AnalogueOutput_00h_AI16
        EPL_OBD_BEGIN_INDEX_RAM(0x6411, 0x03, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x6411, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x6411, 0x01, kEplObdTypInt16, kEplObdAccVPRW, tEplObdInteger16, AnalogueOutput, 0x0000)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x6411, 0x02, kEplObdTypInt16, kEplObdAccVPRW, tEplObdInteger16, AnalogueOutput, 0x0000)
        EPL_OBD_END_INDEX(0x6411)

		// AnalogueOutput_00h_AI32
        EPL_OBD_BEGIN_INDEX_RAM(0x6412, 0x02, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x6412, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 0x01)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x6412, 0x01, kEplObdTypInt32, kEplObdAccVPRW, tEplObdInteger32, AnalogueOutput, 0x00000000)
        EPL_OBD_END_INDEX(0x6412)

    EPL_OBD_END_PART ()

EPL_OBD_END ()


#define EPL_OBD_UNDEFINE_MACRO
    #include "EplObdMacro.h"
#undef EPL_OBD_UNDEFINE_MACRO

