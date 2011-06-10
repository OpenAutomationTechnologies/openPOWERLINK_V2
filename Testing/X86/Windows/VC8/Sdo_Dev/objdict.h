//-----------------------------------------------------------------
//  OD for minimal EPL CN
//-----------------------------------------------------------------
#define EPL_OBD_DEFINE_MACRO
    #include "EplObdMacro.h"
#undef EPL_OBD_DEFINE_MACRO

#define EplApiCbObdAccess NULL

EPL_OBD_BEGIN ()

    EPL_OBD_BEGIN_PART_GENERIC ()

        #include "../../../../../Objdicts/Generic/objdict_1000-13ff.h"


        // Object 1400h: PDO_RxCommParam_00h_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1400, 0x03, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1400, 0x00, 0x05, 0x04, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1400, 0x01, 0x05, 0x03, tEplObdUnsigned8, NodeID_U8, 0x00)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1400, 0x02, 0x05, 0x03, tEplObdUnsigned8, MappingVersion_U8, 0x00)
        EPL_OBD_END_INDEX(0x1400)

        // Object 1401h: PDO_RxCommParam_01h_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1401, 0x03, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1401, 0x00, 0x05, 0x04, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1401, 0x01, 0x05, 0x03, tEplObdUnsigned8, NodeID_U8, 0x6E)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1401, 0x02, 0x05, 0x03, tEplObdUnsigned8, MappingVersion_U8, 0x00)
        EPL_OBD_END_INDEX(0x1401)

        // Object 1600h: PDO_RxMappParam_00h_AU64
        EPL_OBD_BEGIN_INDEX_RAM(0x1600, 0x03, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x00, 0x05, 0x03, tEplObdUnsigned8, NumberOfEntries, 0x01)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x01, 0x1B, 0x03, tEplObdUnsigned64, ObjectMapping, 0x0008000000016200LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x02, 0x1B, 0x03, tEplObdUnsigned64, ObjectMapping, 0x00)
        EPL_OBD_END_INDEX(0x1600)

        // Object 1601h: PDO_RxMappParam_00h_AU64
        EPL_OBD_BEGIN_INDEX_RAM(0x1601, 0x03, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x00, 0x05, 0x03, tEplObdUnsigned8, NumberOfEntries, 0x00)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x01, 0x1B, 0x03, tEplObdUnsigned64, ObjectMapping, 0x0008000000016200LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x02, 0x1B, 0x03, tEplObdUnsigned64, ObjectMapping, 0x00)
        EPL_OBD_END_INDEX(0x1601)

        // Object 1800h: PDO_TxCommParam_00h_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1800, 0x03, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1800, 0x00, 0x05, 0x04, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1800, 0x01, 0x05, 0x03, tEplObdUnsigned8, NodeID_U8, 0x00)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1800, 0x02, 0x05, 0x03, tEplObdUnsigned8, MappingVersion_U8, 0x00)
        EPL_OBD_END_INDEX(0x1800)

        // Object 1A00h: PDO_TxMappParam_00h_AU64
        EPL_OBD_BEGIN_INDEX_RAM(0x1A00, 0x03, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x00, 0x05, 0x03, tEplObdUnsigned8, NumberOfEntries, 0x01)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x01, 0x1B, 0x03, tEplObdUnsigned64, ObjectMapping, 0x0008000000016000LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x02, 0x1B, 0x03, tEplObdUnsigned64, ObjectMapping, 0x00)
        EPL_OBD_END_INDEX(0x1A00)


        #include "../../../../../Objdicts/Generic/objdict_1b00-1fff.h"

    EPL_OBD_END_PART ()

    EPL_OBD_BEGIN_PART_MANUFACTURER ()

        EPL_OBD_BEGIN_INDEX_RAM(0x2000, 0x04, EplAppCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x2000, 0x00, kEplObdTypUInt8, kEplObdAccCR, tEplObdUnsigned8, number_of_entries, 0x1)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x2000, 0x01, kEplObdTypUInt8, kEplObdAccVPRW, tEplObdUnsigned8, Sendb1, 0x0)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x2000, 0x02, kEplObdTypUInt8, kEplObdAccVPRW, tEplObdUnsigned8, Sendb1, 0x0)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x2000, 0x03, kEplObdTypUInt8, kEplObdAccVPRW, tEplObdUnsigned8, Sendb1, 0x0)
        EPL_OBD_END_INDEX(0x2000)

        EPL_OBD_BEGIN_INDEX_RAM(0x2100, 0x01, EplAppCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_DOMAIN(0x2100,0x00,0x03,TEST_DOMAIN)
        EPL_OBD_END_INDEX(0x2100)

        EPL_OBD_BEGIN_INDEX_RAM(0x2200, 0x04, EplAppCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x2200, 0x00, kEplObdTypUInt8, kEplObdAccCR, tEplObdUnsigned8, number_of_entries, 0x1)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x2200, 0x01, kEplObdTypUInt16, kEplObdAccVPRW, tEplObdUnsigned16, Recvb1, 0x0)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x2200, 0x02, kEplObdTypUInt16, kEplObdAccVPRW, tEplObdUnsigned16, Recvb1, 0x0)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x2200, 0x03, kEplObdTypUInt16, kEplObdAccVPRW, tEplObdUnsigned16, Recvb1, 0x0)
        EPL_OBD_END_INDEX(0x2200)

    EPL_OBD_END_PART ()

    EPL_OBD_BEGIN_PART_DEVICE ()

    EPL_OBD_END_PART ()

EPL_OBD_END ()

#define EPL_OBD_UNDEFINE_MACRO
    #include "EplObdMacro.h"
#undef EPL_OBD_UNDEFINE_MACRO
