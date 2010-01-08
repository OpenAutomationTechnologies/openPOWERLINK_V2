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

#endif

        // Object 1600h: PDO_RxMappParam_00h_AU64
        EPL_OBD_BEGIN_INDEX_RAM(0x1600, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x00, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NumberOfEntries, 0x01)
#if ((EPL_API_PROCESS_IMAGE_SIZE_IN > 0) || (EPL_API_PROCESS_IMAGE_SIZE_OUT > 0))
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x01, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0008000000012000LL)
#else
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x01, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0008000000016200LL)
#endif
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x02, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
        EPL_OBD_END_INDEX(0x1600)

        // Object 1601h: PDO_RxMappParam_01h_AU64
        EPL_OBD_BEGIN_INDEX_RAM(0x1601, 0x03, EplPdouCbObdAccess)
#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
            // enable this RxPDO if master is enabled
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x00, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NumberOfEntries, 0x01)
#else
            // otherwise the corresponding object does not exist
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x00, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NumberOfEntries, 0x00)
#endif
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x01, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0008000000012200LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x02, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
        EPL_OBD_END_INDEX(0x1601)

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
        // additional RxPDOs if master is enabled

        // Object 1602h: PDO_RxMappParam_02h_AU64
        EPL_OBD_BEGIN_INDEX_RAM(0x1602, 0x02, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x00, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NumberOfEntries, 0x01)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1602, 0x01, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0008000000022200LL)
        EPL_OBD_END_INDEX(0x1602)

        // Object 1603h: PDO_RxMappParam_03h_AU64
        EPL_OBD_BEGIN_INDEX_RAM(0x1603, 0x02, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x00, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NumberOfEntries, 0x01)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1603, 0x01, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0008000000032200LL)
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
        EPL_OBD_BEGIN_INDEX_RAM(0x1A00, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x00, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NumberOfEntries, 0x01)
#if ((EPL_API_PROCESS_IMAGE_SIZE_IN > 0) || (EPL_API_PROCESS_IMAGE_SIZE_OUT > 0))
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x01, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0008000000012030LL)
#else
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x01, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0008000000016000LL)
#endif
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x02, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
        EPL_OBD_END_INDEX(0x1A00)

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
        // additional TxPDOs if master is enabled

        // Object 1A01h: PDO_TxMappParam_01h_AU64
        EPL_OBD_BEGIN_INDEX_RAM(0x1A01, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x00, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x01, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0008000000022000LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x02, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0008000800012000LL)
        EPL_OBD_END_INDEX(0x1A01)

        // Object 1A02h: PDO_TxMappParam_02h_AU64
        EPL_OBD_BEGIN_INDEX_RAM(0x1A02, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x00, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x01, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0008000000022000LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A02, 0x02, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0008000800012000LL)
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
            EPL_OBD_SUBINDEX_RAM_VAR(0x2000, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, number_of_entries, 0x4)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x2000, 0x01, kEplObdTypUInt8, kEplObdAccVPR, tEplObdUnsigned8, Sendb1, 0x0)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x2000, 0x02, kEplObdTypUInt8, kEplObdAccVPR, tEplObdUnsigned8, Sendb2, 0x0)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x2000, 0x03, kEplObdTypUInt8, kEplObdAccVPR, tEplObdUnsigned8, Sendb3, 0x0)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x2000, 0x04, kEplObdTypUInt8, kEplObdAccVPR, tEplObdUnsigned8, Sendb4, 0x0)
        EPL_OBD_END_INDEX(0x2000)

        // input variables of master
        EPL_OBD_RAM_INDEX_RAM_VARARRAY (0x2200, 3, NULL, kEplObdTypUInt8, kEplObdAccVPRW, tEplObdUnsigned8, Receivebx, 0x00)
#endif

    EPL_OBD_END_PART ()

    EPL_OBD_BEGIN_PART_DEVICE ()

#if !((EPL_API_PROCESS_IMAGE_SIZE_IN > 0) || (EPL_API_PROCESS_IMAGE_SIZE_OUT > 0))
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
#endif

    EPL_OBD_END_PART ()

EPL_OBD_END ()

#define EPL_OBD_UNDEFINE_MACRO
    #include "EplObdMacro.h"
#undef EPL_OBD_UNDEFINE_MACRO
