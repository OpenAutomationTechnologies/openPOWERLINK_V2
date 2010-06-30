//-----------------------------------------------------------------
//  OD for programmable device according to CiA 302-4
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
            EPL_OBD_SUBINDEX_RAM_VAR(0x1401, 0x01, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NodeID_U8, 0x00)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1401, 0x02, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, MappingVersion_U8, 0x00)
        EPL_OBD_END_INDEX(0x1401)

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
        // additional RxPDOs if master is enabled

        // Object 1402h: PDO_RxCommParam_02h_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1402, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1402, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1402, 0x01, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NodeID_U8, 0x00)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1402, 0x02, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, MappingVersion_U8, 0x00)
        EPL_OBD_END_INDEX(0x1402)

        // Object 1403h: PDO_RxCommParam_03h_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1403, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1403, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1403, 0x01, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NodeID_U8, 0x00)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1403, 0x02, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, MappingVersion_U8, 0x00)
        EPL_OBD_END_INDEX(0x1403)

#endif

        // Object 1600h: PDO_RxMappParam_00h_AU64
         EPL_OBD_BEGIN_INDEX_RAM(0x1600, 0x41, EplPdouCbObdAccess)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x00, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NumberOfEntries, 0x40)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x01, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x02, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x03, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x04, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x05, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x06, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x07, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x08, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x09, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x0A, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x0B, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x0C, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x0D, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x0E, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x0F, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x10, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x11, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x12, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x13, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x14, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x15, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x16, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x17, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x18, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x19, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x1A, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x1B, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x1C, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x1D, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x1E, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x1F, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x20, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x21, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x22, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x23, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x24, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x25, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x26, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x27, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x28, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x29, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x2A, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x2B, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x2C, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x2D, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x2E, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x2F, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x30, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x31, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x32, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x33, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x34, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x35, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x36, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x37, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x38, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x39, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x3A, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x3B, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x3C, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x3D, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x3E, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x3F, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x40, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
         EPL_OBD_END_INDEX(0x1600)

        // Object 1601h: PDO_RxMappParam_01h_AU64
        EPL_OBD_BEGIN_INDEX_RAM(0x1601, 0x41, EplPdouCbObdAccess)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x00, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NumberOfEntries, 0x40)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x01, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x02, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x03, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x04, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x05, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x06, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x07, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x08, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x09, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x0A, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x0B, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x0C, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x0D, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x0E, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x0F, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x10, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x11, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x12, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x13, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x14, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x15, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x16, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x17, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x18, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x19, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x1A, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x1B, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x1C, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x1D, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x1E, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x1F, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x20, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x21, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x22, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x23, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x24, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x25, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x26, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x27, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x28, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x29, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x2A, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x2B, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x2C, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x2D, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x2E, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x2F, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x30, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x31, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x32, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x33, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x34, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x35, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x36, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x37, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x38, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x39, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x3A, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x3B, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x3C, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x3D, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x3E, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x3F, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x40, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
        EPL_OBD_END_INDEX(0x1601)

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
        // additional RxPDOs if master is enabled

        // Object 1602h: PDO_RxMappParam_02h_AU64
        EPL_OBD_BEGIN_INDEX_RAM(0x1602, 0x41, EplPdouCbObdAccess)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x00, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NumberOfEntries, 0x40)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x01, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x02, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x03, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x04, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x05, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x06, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x07, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x08, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x09, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x0A, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x0B, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x0C, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x0D, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x0E, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x0F, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x10, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x11, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x12, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x13, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x14, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x15, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x16, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x17, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x18, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x19, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x1A, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x1B, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x1C, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x1D, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x1E, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x1F, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x20, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x21, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x22, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x23, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x24, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x25, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x26, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x27, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x28, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x29, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x2A, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x2B, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x2C, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x2D, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x2E, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x2F, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x30, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x31, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x32, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x33, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x34, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x35, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x36, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x37, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x38, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x39, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x3A, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x3B, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x3C, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x3D, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x3E, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x3F, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x40, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
        EPL_OBD_END_INDEX(0x1602)

        // Object 1603h: PDO_RxMappParam_03h_AU64
        EPL_OBD_BEGIN_INDEX_RAM(0x1603, 0x41, EplPdouCbObdAccess)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x00, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NumberOfEntries, 0x40)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x01, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x02, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x03, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x04, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x05, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x06, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x07, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x08, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x09, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x0A, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x0B, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x0C, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x0D, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x0E, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x0F, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x10, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x11, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x12, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x13, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x14, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x15, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x16, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x17, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x18, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x19, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x1A, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x1B, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x1C, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x1D, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x1E, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x1F, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x20, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x21, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x22, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x23, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x24, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x25, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x26, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x27, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x28, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x29, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x2A, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x2B, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x2C, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x2D, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x2E, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x2F, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x30, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x31, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x32, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x33, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x34, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x35, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x36, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x37, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x38, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x39, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x3A, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x3B, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x3C, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x3D, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x3E, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x3F, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x40, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
        EPL_OBD_END_INDEX(0x1603)

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
            EPL_OBD_SUBINDEX_RAM_VAR(0x1801, 0x01, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NodeID_U8, 0x00)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1801, 0x02, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, MappingVersion_U8, 0x00)
        EPL_OBD_END_INDEX(0x1801)

        // Object 1802h: PDO_TxCommParam_02h_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1802, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1802, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1802, 0x01, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NodeID_U8, 0x00)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1802, 0x02, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, MappingVersion_U8, 0x00)
        EPL_OBD_END_INDEX(0x1802)

        // Object 1803h: PDO_TxCommParam_03h_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1803, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1803, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1803, 0x01, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NodeID_U8, 0x00)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1803, 0x02, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, MappingVersion_U8, 0x00)
        EPL_OBD_END_INDEX(0x1803)

#endif

        // Object 1A00h: PDO_TxMappParam_00h_AU64
         EPL_OBD_BEGIN_INDEX_RAM(0x1A00, 0x41, EplPdouCbObdAccess)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x00, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NumberOfEntries, 0x40)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x01, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x02, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x03, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x04, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x05, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x06, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x07, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x08, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x09, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x0A, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x0B, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x0C, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x0D, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x0E, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x0F, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x10, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x11, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x12, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x13, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x14, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x15, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x16, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x17, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x18, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x19, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x1A, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x1B, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x1C, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x1D, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x1E, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x1F, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x20, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x21, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x22, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x23, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x24, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x25, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x26, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x27, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x28, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x29, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x2A, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x2B, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x2C, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x2D, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x2E, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x2F, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x30, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x31, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x32, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x33, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x34, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x35, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x36, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x37, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x38, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x39, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x3A, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x3B, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x3C, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x3D, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x3E, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x3F, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x40, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
         EPL_OBD_END_INDEX(0x1A00)

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
        // additional TxPDOs if master is enabled
        // Object 1A01h: PDO_TxMappParam_01h_AU64
        EPL_OBD_BEGIN_INDEX_RAM(0x1A01, 0x41, EplPdouCbObdAccess)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x00, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NumberOfEntries, 0x40)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x01, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x02, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x03, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x04, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x05, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x06, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x07, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x08, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x09, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x0A, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x0B, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x0C, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x0D, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x0E, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x0F, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x10, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x11, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x12, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x13, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x14, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x15, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x16, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x17, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x18, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x19, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x1A, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x1B, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x1C, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x1D, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x1E, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x1F, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x20, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x21, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x22, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x23, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x24, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x25, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x26, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x27, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x28, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x29, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x2A, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x2B, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x2C, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x2D, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x2E, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x2F, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x30, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x31, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x32, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x33, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x34, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x35, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x36, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x37, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x38, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x39, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x3A, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x3B, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x3C, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x3D, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x3E, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x3F, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x40, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
        EPL_OBD_END_INDEX(0x1A01)


        // Object 1A02h: PDO_TxMappParam_02h_AU64
        EPL_OBD_BEGIN_INDEX_RAM(0x1A02, 0x41, EplPdouCbObdAccess)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x00, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NumberOfEntries, 0x40)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x01, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x02, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x03, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x04, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x05, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x06, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x07, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x08, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x09, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x0A, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x0B, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x0C, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x0D, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x0E, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x0F, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x10, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x11, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x12, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x13, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x14, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x15, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x16, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x17, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x18, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x19, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x1A, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x1B, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x1C, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x1D, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x1E, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x1F, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x20, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x21, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x22, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x23, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x24, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x25, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x26, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x27, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x28, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x29, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x2A, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x2B, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x2C, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x2D, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x2E, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x2F, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x30, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x31, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x32, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x33, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x34, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x35, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x36, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x37, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x38, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x39, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x3A, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x3B, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x3C, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x3D, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x3E, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x3F, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x40, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
        EPL_OBD_END_INDEX(0x1A02)

        // Object 1A03h: PDO_TxMappParam_03h_AU64
        EPL_OBD_BEGIN_INDEX_RAM(0x1A03, 0x41, EplPdouCbObdAccess)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x00, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NumberOfEntries, 0x40)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x01, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x02, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x03, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x04, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x05, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x06, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x07, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x08, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x09, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x0A, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x0B, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x0C, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x0D, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x0E, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x0F, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x10, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x11, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x12, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x13, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x14, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x15, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x16, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x17, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x18, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x19, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x1A, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x1B, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x1C, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x1D, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x1E, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x1F, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x20, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x21, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x22, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x23, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x24, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x25, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x26, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x27, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x28, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x29, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x2A, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x2B, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x2C, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x2D, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x2E, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x2F, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x30, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x31, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x32, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x33, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x34, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x35, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x36, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x37, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x38, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x39, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x3A, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x3B, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x3C, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x3D, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x3E, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x3F, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
             EPL_OBD_SUBINDEX_RAM_VAR(0x1A03, 0x40, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
        EPL_OBD_END_INDEX(0x1A03)

#endif


        #include "../Generic/objdict_1b00-1fff.h"

    EPL_OBD_END_PART ()

    EPL_OBD_BEGIN_PART_MANUFACTURER ()

    EPL_OBD_END_PART ()

    EPL_OBD_BEGIN_PART_DEVICE ()

        // static input process image

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA000, (252), NULL, kEplObdTypInt8, kEplObdAccVPRW, tEplObdInteger8, SHORT_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA001, (252), NULL, kEplObdTypInt8, kEplObdAccVPRW, tEplObdInteger8, SHORT_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA002, (252), NULL, kEplObdTypInt8, kEplObdAccVPRW, tEplObdInteger8, SHORT_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA003, (252), NULL, kEplObdTypInt8, kEplObdAccVPRW, tEplObdInteger8, SHORT_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA004, (252), NULL, kEplObdTypInt8, kEplObdAccVPRW, tEplObdInteger8, SHORT_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA005, (252), NULL, kEplObdTypInt8, kEplObdAccVPRW, tEplObdInteger8, SHORT_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA006, (252), NULL, kEplObdTypInt8, kEplObdAccVPRW, tEplObdInteger8, SHORT_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA007, (252), NULL, kEplObdTypInt8, kEplObdAccVPRW, tEplObdInteger8, SHORT_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA008, (252), NULL, kEplObdTypInt8, kEplObdAccVPRW, tEplObdInteger8, SHORT_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA009, (252), NULL, kEplObdTypInt8, kEplObdAccVPRW, tEplObdInteger8, SHORT_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA00A, (252), NULL, kEplObdTypInt8, kEplObdAccVPRW, tEplObdInteger8, SHORT_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA00B, (252), NULL, kEplObdTypInt8, kEplObdAccVPRW, tEplObdInteger8, SHORT_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA00C, (252), NULL, kEplObdTypInt8, kEplObdAccVPRW, tEplObdInteger8, SHORT_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA00D, (252), NULL, kEplObdTypInt8, kEplObdAccVPRW, tEplObdInteger8, SHORT_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA00E, (252), NULL, kEplObdTypInt8, kEplObdAccVPRW, tEplObdInteger8, SHORT_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA00F, (252), NULL, kEplObdTypInt8, kEplObdAccVPRW, tEplObdInteger8, SHORT_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA040, (252), NULL, kEplObdTypUInt8, kEplObdAccVPRW, tEplObdUnsigned8, BYTE_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA041, (252), NULL, kEplObdTypUInt8, kEplObdAccVPRW, tEplObdUnsigned8, BYTE_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA042, (252), NULL, kEplObdTypUInt8, kEplObdAccVPRW, tEplObdUnsigned8, BYTE_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA043, (252), NULL, kEplObdTypUInt8, kEplObdAccVPRW, tEplObdUnsigned8, BYTE_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA044, (252), NULL, kEplObdTypUInt8, kEplObdAccVPRW, tEplObdUnsigned8, BYTE_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA045, (252), NULL, kEplObdTypUInt8, kEplObdAccVPRW, tEplObdUnsigned8, BYTE_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA046, (252), NULL, kEplObdTypUInt8, kEplObdAccVPRW, tEplObdUnsigned8, BYTE_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA047, (252), NULL, kEplObdTypUInt8, kEplObdAccVPRW, tEplObdUnsigned8, BYTE_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA048, (252), NULL, kEplObdTypUInt8, kEplObdAccVPRW, tEplObdUnsigned8, BYTE_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA049, (252), NULL, kEplObdTypUInt8, kEplObdAccVPRW, tEplObdUnsigned8, BYTE_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA04A, (252), NULL, kEplObdTypUInt8, kEplObdAccVPRW, tEplObdUnsigned8, BYTE_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA04B, (252), NULL, kEplObdTypUInt8, kEplObdAccVPRW, tEplObdUnsigned8, BYTE_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA04C, (252), NULL, kEplObdTypUInt8, kEplObdAccVPRW, tEplObdUnsigned8, BYTE_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA04D, (252), NULL, kEplObdTypUInt8, kEplObdAccVPRW, tEplObdUnsigned8, BYTE_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA04E, (252), NULL, kEplObdTypUInt8, kEplObdAccVPRW, tEplObdUnsigned8, BYTE_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA04F, (252), NULL, kEplObdTypUInt8, kEplObdAccVPRW, tEplObdUnsigned8, BYTE_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA0C0, (252), NULL, kEplObdTypInt16, kEplObdAccVPRW, tEplObdInteger16, INT_Merker, 0)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA0C1, (252), NULL, kEplObdTypInt16, kEplObdAccVPRW, tEplObdInteger16, INT_Merker, 0)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA0C2, (252), NULL, kEplObdTypInt16, kEplObdAccVPRW, tEplObdInteger16, INT_Merker, 0)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA0C3, (252), NULL, kEplObdTypInt16, kEplObdAccVPRW, tEplObdInteger16, INT_Merker, 0)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA0C4, (252), NULL, kEplObdTypInt16, kEplObdAccVPRW, tEplObdInteger16, INT_Merker, 0)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA0C5, (252), NULL, kEplObdTypInt16, kEplObdAccVPRW, tEplObdInteger16, INT_Merker, 0)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA0C6, (252), NULL, kEplObdTypInt16, kEplObdAccVPRW, tEplObdInteger16, INT_Merker, 0)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA0C7, (252), NULL, kEplObdTypInt16, kEplObdAccVPRW, tEplObdInteger16, INT_Merker, 0)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA100, (252), NULL, kEplObdTypUInt16, kEplObdAccVPRW, tEplObdUnsigned16, WORD_Merker, 0)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA101, (252), NULL, kEplObdTypUInt16, kEplObdAccVPRW, tEplObdUnsigned16, WORD_Merker, 0)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA102, (252), NULL, kEplObdTypUInt16, kEplObdAccVPRW, tEplObdUnsigned16, WORD_Merker, 0)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA103, (252), NULL, kEplObdTypUInt16, kEplObdAccVPRW, tEplObdUnsigned16, WORD_Merker, 0)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA104, (252), NULL, kEplObdTypUInt16, kEplObdAccVPRW, tEplObdUnsigned16, WORD_Merker, 0)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA105, (252), NULL, kEplObdTypUInt16, kEplObdAccVPRW, tEplObdUnsigned16, WORD_Merker, 0)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA106, (252), NULL, kEplObdTypUInt16, kEplObdAccVPRW, tEplObdUnsigned16, WORD_Merker, 0)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA107, (252), NULL, kEplObdTypUInt16, kEplObdAccVPRW, tEplObdUnsigned16, WORD_Merker, 0)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA1C0, (252), NULL, kEplObdTypUInt32, kEplObdAccVPRW, tEplObdUnsigned32, DWORD_Merker, 0)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA1C1, (252), NULL, kEplObdTypUInt32, kEplObdAccVPRW, tEplObdUnsigned32, DWORD_Merker, 0)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA1C2, (252), NULL, kEplObdTypUInt32, kEplObdAccVPRW, tEplObdUnsigned32, DWORD_Merker, 0)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA1C3, (252), NULL, kEplObdTypUInt32, kEplObdAccVPRW, tEplObdUnsigned32, DWORD_Merker, 0)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA200, (252), NULL, kEplObdTypUInt32, kEplObdAccVPRW, tEplObdUnsigned32, DWORD_Merker, 0)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA201, (252), NULL, kEplObdTypUInt32, kEplObdAccVPRW, tEplObdUnsigned32, DWORD_Merker, 0)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA202, (252), NULL, kEplObdTypUInt32, kEplObdAccVPRW, tEplObdUnsigned32, DWORD_Merker, 0)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA203, (252), NULL, kEplObdTypUInt32, kEplObdAccVPRW, tEplObdUnsigned32, DWORD_Merker, 0)


        // static output process image

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA480, (252), NULL, kEplObdTypInt8, kEplObdAccVPR, tEplObdInteger8, SHORT_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA481, (252), NULL, kEplObdTypInt8, kEplObdAccVPR, tEplObdInteger8, SHORT_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA482, (252), NULL, kEplObdTypInt8, kEplObdAccVPR, tEplObdInteger8, SHORT_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA483, (252), NULL, kEplObdTypInt8, kEplObdAccVPR, tEplObdInteger8, SHORT_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA484, (252), NULL, kEplObdTypInt8, kEplObdAccVPR, tEplObdInteger8, SHORT_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA485, (252), NULL, kEplObdTypInt8, kEplObdAccVPR, tEplObdInteger8, SHORT_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA486, (252), NULL, kEplObdTypInt8, kEplObdAccVPR, tEplObdInteger8, SHORT_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA487, (252), NULL, kEplObdTypInt8, kEplObdAccVPR, tEplObdInteger8, SHORT_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA488, (252), NULL, kEplObdTypInt8, kEplObdAccVPR, tEplObdInteger8, SHORT_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA489, (252), NULL, kEplObdTypInt8, kEplObdAccVPR, tEplObdInteger8, SHORT_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA48A, (252), NULL, kEplObdTypInt8, kEplObdAccVPR, tEplObdInteger8, SHORT_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA48B, (252), NULL, kEplObdTypInt8, kEplObdAccVPR, tEplObdInteger8, SHORT_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA48C, (252), NULL, kEplObdTypInt8, kEplObdAccVPR, tEplObdInteger8, SHORT_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA48D, (252), NULL, kEplObdTypInt8, kEplObdAccVPR, tEplObdInteger8, SHORT_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA48E, (252), NULL, kEplObdTypInt8, kEplObdAccVPR, tEplObdInteger8, SHORT_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA48F, (252), NULL, kEplObdTypInt8, kEplObdAccVPR, tEplObdInteger8, SHORT_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA4C0, (252), NULL, kEplObdTypUInt8, kEplObdAccVPR, tEplObdUnsigned8, BYTE_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA4C1, (252), NULL, kEplObdTypUInt8, kEplObdAccVPR, tEplObdUnsigned8, BYTE_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA4C2, (252), NULL, kEplObdTypUInt8, kEplObdAccVPR, tEplObdUnsigned8, BYTE_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA4C3, (252), NULL, kEplObdTypUInt8, kEplObdAccVPR, tEplObdUnsigned8, BYTE_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA4C4, (252), NULL, kEplObdTypUInt8, kEplObdAccVPR, tEplObdUnsigned8, BYTE_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA4C5, (252), NULL, kEplObdTypUInt8, kEplObdAccVPR, tEplObdUnsigned8, BYTE_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA4C6, (252), NULL, kEplObdTypUInt8, kEplObdAccVPR, tEplObdUnsigned8, BYTE_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA4C7, (252), NULL, kEplObdTypUInt8, kEplObdAccVPR, tEplObdUnsigned8, BYTE_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA4C8, (252), NULL, kEplObdTypUInt8, kEplObdAccVPR, tEplObdUnsigned8, BYTE_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA4C9, (252), NULL, kEplObdTypUInt8, kEplObdAccVPR, tEplObdUnsigned8, BYTE_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA4CA, (252), NULL, kEplObdTypUInt8, kEplObdAccVPR, tEplObdUnsigned8, BYTE_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA4CB, (252), NULL, kEplObdTypUInt8, kEplObdAccVPR, tEplObdUnsigned8, BYTE_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA4CC, (252), NULL, kEplObdTypUInt8, kEplObdAccVPR, tEplObdUnsigned8, BYTE_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA4CD, (252), NULL, kEplObdTypUInt8, kEplObdAccVPR, tEplObdUnsigned8, BYTE_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA4CE, (252), NULL, kEplObdTypUInt8, kEplObdAccVPR, tEplObdUnsigned8, BYTE_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA4CF, (252), NULL, kEplObdTypUInt8, kEplObdAccVPR, tEplObdUnsigned8, BYTE_Merker, 0x00)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA540, (252), NULL, kEplObdTypInt16, kEplObdAccVPR, tEplObdInteger16, INT_Merker, 0)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA541, (252), NULL, kEplObdTypInt16, kEplObdAccVPR, tEplObdInteger16, INT_Merker, 0)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA542, (252), NULL, kEplObdTypInt16, kEplObdAccVPR, tEplObdInteger16, INT_Merker, 0)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA543, (252), NULL, kEplObdTypInt16, kEplObdAccVPR, tEplObdInteger16, INT_Merker, 0)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA544, (252), NULL, kEplObdTypInt16, kEplObdAccVPR, tEplObdInteger16, INT_Merker, 0)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA545, (252), NULL, kEplObdTypInt16, kEplObdAccVPR, tEplObdInteger16, INT_Merker, 0)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA546, (252), NULL, kEplObdTypInt16, kEplObdAccVPR, tEplObdInteger16, INT_Merker, 0)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA547, (252), NULL, kEplObdTypInt16, kEplObdAccVPR, tEplObdInteger16, INT_Merker, 0)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA580, (252), NULL, kEplObdTypUInt16, kEplObdAccVPR, tEplObdUnsigned16, WORD_Merker, 0)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA581, (252), NULL, kEplObdTypUInt16, kEplObdAccVPR, tEplObdUnsigned16, WORD_Merker, 0)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA582, (252), NULL, kEplObdTypUInt16, kEplObdAccVPR, tEplObdUnsigned16, WORD_Merker, 0)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA583, (252), NULL, kEplObdTypUInt16, kEplObdAccVPR, tEplObdUnsigned16, WORD_Merker, 0)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA584, (252), NULL, kEplObdTypUInt16, kEplObdAccVPR, tEplObdUnsigned16, WORD_Merker, 0)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA585, (252), NULL, kEplObdTypUInt16, kEplObdAccVPR, tEplObdUnsigned16, WORD_Merker, 0)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA586, (252), NULL, kEplObdTypUInt16, kEplObdAccVPR, tEplObdUnsigned16, WORD_Merker, 0)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA587, (252), NULL, kEplObdTypUInt16, kEplObdAccVPR, tEplObdUnsigned16, WORD_Merker, 0)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA640, (252), NULL, kEplObdTypUInt32, kEplObdAccVPR, tEplObdUnsigned32, DWORD_Merker, 0)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA641, (252), NULL, kEplObdTypUInt32, kEplObdAccVPR, tEplObdUnsigned32, DWORD_Merker, 0)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA642, (252), NULL, kEplObdTypUInt32, kEplObdAccVPR, tEplObdUnsigned32, DWORD_Merker, 0)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA643, (252), NULL, kEplObdTypUInt32, kEplObdAccVPR, tEplObdUnsigned32, DWORD_Merker, 0)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA680, (252), NULL, kEplObdTypUInt32, kEplObdAccVPR, tEplObdUnsigned32, DWORD_Merker, 0)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA681, (252), NULL, kEplObdTypUInt32, kEplObdAccVPR, tEplObdUnsigned32, DWORD_Merker, 0)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA682, (252), NULL, kEplObdTypUInt32, kEplObdAccVPR, tEplObdUnsigned32, DWORD_Merker, 0)

        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0xA683, (252), NULL, kEplObdTypUInt32, kEplObdAccVPR, tEplObdUnsigned32, DWORD_Merker, 0)

    EPL_OBD_END_PART ()

EPL_OBD_END ()

#define EPL_OBD_UNDEFINE_MACRO
    #include "EplObdMacro.h"
#undef EPL_OBD_UNDEFINE_MACRO
