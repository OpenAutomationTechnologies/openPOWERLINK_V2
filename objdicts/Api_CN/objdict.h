//-----------------------------------------------------------------
//  OD for minimal EPL CN
//-----------------------------------------------------------------
#define OBD_DEFINE_MACRO
    #include <oplk/obdmacro.h>
#undef OBD_DEFINE_MACRO

OBD_BEGIN ()

    OBD_BEGIN_PART_GENERIC ()

        #include "../Generic/objdict_1000-13ff.h"


        // Object 1400h: PDO_RxCommParam_00h_REC
        OBD_BEGIN_INDEX_RAM(0x1400, 0x03, pdou_CbObdAccess)
            OBD_SUBINDEX_RAM_VAR(0x1400, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR(0x1400, 0x01, kObdTypeUInt8, kObdAccRW, tObdUnsigned8, NodeID_U8, 0x00)
            OBD_SUBINDEX_RAM_VAR(0x1400, 0x02, kObdTypeUInt8, kObdAccRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1400)

        // Object 1401h: PDO_RxCommParam_01h_REC
        OBD_BEGIN_INDEX_RAM(0x1401, 0x03, pdou_CbObdAccess)
            OBD_SUBINDEX_RAM_VAR(0x1401, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR(0x1401, 0x01, kObdTypeUInt8, kObdAccRW, tObdUnsigned8, NodeID_U8, 0x6E)
            OBD_SUBINDEX_RAM_VAR(0x1401, 0x02, kObdTypeUInt8, kObdAccRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1401)

#if defined(CONFIG_INCLUDE_NMT_MN)
        // additional RxPDOs if master is enabled

        // Object 1402h: PDO_RxCommParam_02h_REC
        OBD_BEGIN_INDEX_RAM(0x1402, 0x03, pdou_CbObdAccess)
            OBD_SUBINDEX_RAM_VAR(0x1402, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR(0x1402, 0x01, kObdTypeUInt8, kObdAccRW, tObdUnsigned8, NodeID_U8, 0x01)
            OBD_SUBINDEX_RAM_VAR(0x1402, 0x02, kObdTypeUInt8, kObdAccRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1402)

        // Object 1403h: PDO_RxCommParam_03h_REC
        OBD_BEGIN_INDEX_RAM(0x1403, 0x03, pdou_CbObdAccess)
            OBD_SUBINDEX_RAM_VAR(0x1403, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR(0x1403, 0x01, kObdTypeUInt8, kObdAccRW, tObdUnsigned8, NodeID_U8, 0x20)
            OBD_SUBINDEX_RAM_VAR(0x1403, 0x02, kObdTypeUInt8, kObdAccRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1403)

#endif

        // Object 1600h: PDO_RxMappParam_00h_AU64
        OBD_BEGIN_INDEX_RAM(0x1600, 0x03, pdou_CbObdAccess)
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x00, kObdTypeUInt8, kObdAccRW, tObdUnsigned8, NumberOfEntries, 0x01)
#if ((EPL_API_PROCESS_IMAGE_SIZE_IN > 0) || (EPL_API_PROCESS_IMAGE_SIZE_OUT > 0))
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x01, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x0008000000012000LL)
#else
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x01, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x0008000000016200LL)
#endif
            OBD_SUBINDEX_RAM_VAR(0x1600, 0x02, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00)
        OBD_END_INDEX(0x1600)

        // Object 1601h: PDO_RxMappParam_01h_AU64
        OBD_BEGIN_INDEX_RAM(0x1601, 0x03, pdou_CbObdAccess)
#if defined(CONFIG_INCLUDE_NMT_MN)
            // enable this RxPDO if master is enabled
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x00, kObdTypeUInt8, kObdAccRW, tObdUnsigned8, NumberOfEntries, 0x01)
#else
            // otherwise the corresponding object does not exist
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x00, kObdTypeUInt8, kObdAccRW, tObdUnsigned8, NumberOfEntries, 0x00)
#endif
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x01, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x0008000000012200LL)
            OBD_SUBINDEX_RAM_VAR(0x1601, 0x02, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00)
        OBD_END_INDEX(0x1601)

#if defined(CONFIG_INCLUDE_NMT_MN)
        // additional RxPDOs if master is enabled

        // Object 1602h: PDO_RxMappParam_02h_AU64
        OBD_BEGIN_INDEX_RAM(0x1602, 0x02, pdou_CbObdAccess)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x00, kObdTypeUInt8, kObdAccRW, tObdUnsigned8, NumberOfEntries, 0x01)
            OBD_SUBINDEX_RAM_VAR(0x1602, 0x01, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x0008000000022200LL)
        OBD_END_INDEX(0x1602)

        // Object 1603h: PDO_RxMappParam_03h_AU64
        OBD_BEGIN_INDEX_RAM(0x1603, 0x02, pdou_CbObdAccess)
            OBD_SUBINDEX_RAM_VAR(0x1603, 0x00, kObdTypeUInt8, kObdAccRW, tObdUnsigned8, NumberOfEntries, 0x01)
            OBD_SUBINDEX_RAM_VAR(0x1603, 0x01, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x0008000000032200LL)
        OBD_END_INDEX(0x1603)

#endif

        // Object 1800h: PDO_TxCommParam_00h_REC
        OBD_BEGIN_INDEX_RAM(0x1800, 0x03, pdou_CbObdAccess)
            OBD_SUBINDEX_RAM_VAR(0x1800, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR(0x1800, 0x01, kObdTypeUInt8, kObdAccRW, tObdUnsigned8, NodeID_U8, 0x00)
            OBD_SUBINDEX_RAM_VAR(0x1800, 0x02, kObdTypeUInt8, kObdAccRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1800)

#if defined(CONFIG_INCLUDE_NMT_MN)
        // additional TxPDOs if master is enabled

        // Object 1801h: PDO_TxCommParam_01h_REC
        OBD_BEGIN_INDEX_RAM(0x1801, 0x03, pdou_CbObdAccess)
            OBD_SUBINDEX_RAM_VAR(0x1801, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR(0x1801, 0x01, kObdTypeUInt8, kObdAccRW, tObdUnsigned8, NodeID_U8, 0x01)
            OBD_SUBINDEX_RAM_VAR(0x1801, 0x02, kObdTypeUInt8, kObdAccRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1801)

        // Object 1802h: PDO_TxCommParam_02h_REC
        OBD_BEGIN_INDEX_RAM(0x1802, 0x03, pdou_CbObdAccess)
            OBD_SUBINDEX_RAM_VAR(0x1802, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR(0x1802, 0x01, kObdTypeUInt8, kObdAccRW, tObdUnsigned8, NodeID_U8, 0x20)
            OBD_SUBINDEX_RAM_VAR(0x1802, 0x02, kObdTypeUInt8, kObdAccRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1802)

        // Object 1803h: PDO_TxCommParam_03h_REC
        OBD_BEGIN_INDEX_RAM(0x1803, 0x03, pdou_CbObdAccess)
            OBD_SUBINDEX_RAM_VAR(0x1803, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR(0x1803, 0x01, kObdTypeUInt8, kObdAccRW, tObdUnsigned8, NodeID_U8, 0x6E)
            OBD_SUBINDEX_RAM_VAR(0x1803, 0x02, kObdTypeUInt8, kObdAccRW, tObdUnsigned8, MappingVersion_U8, 0x00)
        OBD_END_INDEX(0x1803)

#endif

        // Object 1A00h: PDO_TxMappParam_00h_AU64
        OBD_BEGIN_INDEX_RAM(0x1A00, 0x03, pdou_CbObdAccess)
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x00, kObdTypeUInt8, kObdAccRW, tObdUnsigned8, NumberOfEntries, 0x01)
#if ((EPL_API_PROCESS_IMAGE_SIZE_IN > 0) || (EPL_API_PROCESS_IMAGE_SIZE_OUT > 0))
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x01, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x0008000000012030LL)
#else
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x01, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x0008000000016000LL)
#endif
            OBD_SUBINDEX_RAM_VAR(0x1A00, 0x02, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x00)
        OBD_END_INDEX(0x1A00)

#if defined(CONFIG_INCLUDE_NMT_MN)
        // additional TxPDOs if master is enabled

        // Object 1A01h: PDO_TxMappParam_01h_AU64
        OBD_BEGIN_INDEX_RAM(0x1A01, 0x03, pdou_CbObdAccess)
            OBD_SUBINDEX_RAM_VAR(0x1A01, 0x00, kObdTypeUInt8, kObdAccRW, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR(0x1A01, 0x01, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x0008000000022000LL)
            OBD_SUBINDEX_RAM_VAR(0x1A01, 0x02, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x0008000800012000LL)
        OBD_END_INDEX(0x1A01)

        // Object 1A02h: PDO_TxMappParam_02h_AU64
        OBD_BEGIN_INDEX_RAM(0x1A02, 0x03, pdou_CbObdAccess)
            OBD_SUBINDEX_RAM_VAR(0x1A02, 0x00, kObdTypeUInt8, kObdAccRW, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR(0x1A02, 0x01, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x0008000000022000LL)
            OBD_SUBINDEX_RAM_VAR(0x1A02, 0x02, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x0008000800012000LL)
        OBD_END_INDEX(0x1A02)

        // Object 1A03h: PDO_TxMappParam_03h_AU64
        OBD_BEGIN_INDEX_RAM(0x1A03, 0x03, pdou_CbObdAccess)
            OBD_SUBINDEX_RAM_VAR(0x1A03, 0x00, kObdTypeUInt8, kObdAccRW, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR(0x1A03, 0x01, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x0008000000012000LL)
            OBD_SUBINDEX_RAM_VAR(0x1A03, 0x02, kObdTypeUInt64, kObdAccRW, tObdUnsigned64, ObjectMapping, 0x0008000800022000LL)
        OBD_END_INDEX(0x1A03)

#endif


        #include "../Generic/objdict_1b00-1fff.h"

    OBD_END_PART ()

    OBD_BEGIN_PART_MANUFACTURER ()

#if EPL_API_PROCESS_IMAGE_SIZE_IN > 0
        // static input process image
        OBD_RAM_INDEX_RAM_VARARRAY (0x2000, (EPL_API_PROCESS_IMAGE_SIZE_IN), NULL, kObdTypeUInt8, kObdAccVPRW, tObdUnsigned8, BYTE_Merker, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY (0x2001, (EPL_API_PROCESS_IMAGE_SIZE_IN), NULL, kObdTypeInt8, kObdAccVPRW, tObdInteger8, SHORT_Merker, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY (0x2010, (EPL_API_PROCESS_IMAGE_SIZE_IN / 2), NULL, kObdTypeUInt16, kObdAccVPRW, tObdUnsigned16, WORD_Merker, 0)
        OBD_RAM_INDEX_RAM_VARARRAY (0x2011, (EPL_API_PROCESS_IMAGE_SIZE_IN / 2), NULL, kObdTypeInt16, kObdAccVPRW, tObdInteger16, INT_Merker, 0)
        OBD_RAM_INDEX_RAM_VARARRAY (0x2020, (EPL_API_PROCESS_IMAGE_SIZE_IN / 4), NULL, kObdTypeUInt32, kObdAccVPRW, tObdUnsigned32, DWORD_Merker, 0)
        OBD_RAM_INDEX_RAM_VARARRAY (0x2021, (EPL_API_PROCESS_IMAGE_SIZE_IN / 4), NULL, kObdTypeInt32, kObdAccVPRW, tObdInteger32, LINT_Merker, 0)
#endif

#if EPL_API_PROCESS_IMAGE_SIZE_OUT > 0
        // static output process image
        OBD_RAM_INDEX_RAM_VARARRAY (0x2030, (EPL_API_PROCESS_IMAGE_SIZE_OUT), NULL, kObdTypeUInt8, kObdAccVPR, tObdUnsigned8, BYTE_Merker, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY (0x2031, (EPL_API_PROCESS_IMAGE_SIZE_OUT), NULL, kObdTypeInt8, kObdAccVPR, tObdInteger8, SHORT_Merker, 0x00)
        OBD_RAM_INDEX_RAM_VARARRAY (0x2040, (EPL_API_PROCESS_IMAGE_SIZE_OUT / 2), NULL, kObdTypeUInt16, kObdAccVPR, tObdUnsigned16, WORD_Merker, 0)
        OBD_RAM_INDEX_RAM_VARARRAY (0x2041, (EPL_API_PROCESS_IMAGE_SIZE_OUT / 2), NULL, kObdTypeInt16, kObdAccVPR, tObdInteger16, INT_Merker, 0)
        OBD_RAM_INDEX_RAM_VARARRAY (0x2050, (EPL_API_PROCESS_IMAGE_SIZE_OUT / 4), NULL, kObdTypeUInt32, kObdAccVPR, tObdUnsigned32, DWORD_Merker, 0)
        OBD_RAM_INDEX_RAM_VARARRAY (0x2051, (EPL_API_PROCESS_IMAGE_SIZE_OUT / 4), NULL, kObdTypeInt32, kObdAccVPR, tObdInteger32, LINT_Merker, 0)
#endif

#if (!((EPL_API_PROCESS_IMAGE_SIZE_IN > 0) || (EPL_API_PROCESS_IMAGE_SIZE_OUT > 0))) \
    && defined(CONFIG_INCLUDE_NMT_MN)
        // process image is not enabled but master is enabled

        // output variables of master
        OBD_BEGIN_INDEX_RAM(0x2000, 0x05, NULL)
            OBD_SUBINDEX_RAM_VAR(0x2000, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, number_of_entries, 0x4)
            OBD_SUBINDEX_RAM_USERDEF(0x2000, 0x01, kObdTypeUInt8, kObdAccVPR, tObdUnsigned8, Sendb1, 0x0)
            OBD_SUBINDEX_RAM_USERDEF(0x2000, 0x02, kObdTypeUInt8, kObdAccVPR, tObdUnsigned8, Sendb2, 0x0)
            OBD_SUBINDEX_RAM_USERDEF(0x2000, 0x03, kObdTypeUInt8, kObdAccVPR, tObdUnsigned8, Sendb3, 0x0)
            OBD_SUBINDEX_RAM_USERDEF(0x2000, 0x04, kObdTypeUInt8, kObdAccVPR, tObdUnsigned8, Sendb4, 0x0)
        OBD_END_INDEX(0x2000)

        // input variables of master
        OBD_RAM_INDEX_RAM_VARARRAY (0x2200, 3, NULL, kObdTypeUInt8, kObdAccVPRW, tObdUnsigned8, Receivebx, 0x00)
#endif

    OBD_END_PART ()

    OBD_BEGIN_PART_DEVICE ()

#if !((EPL_API_PROCESS_IMAGE_SIZE_IN > 0) || (EPL_API_PROCESS_IMAGE_SIZE_OUT > 0))
        OBD_BEGIN_INDEX_RAM(0x6000, 0x02, NULL)
            OBD_SUBINDEX_RAM_VAR(0x6000, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, number_of_entries, 0x1)
            OBD_SUBINDEX_RAM_USERDEF(0x6000, 0x01, kObdTypeUInt8, kObdAccVPR, tObdUnsigned8, Sendb1, 0x0)
        OBD_END_INDEX(0x6000)

        OBD_BEGIN_INDEX_RAM(0x6100, 0x01, NULL)
            OBD_SUBINDEX_RAM_DOMAIN(0x6100,0x00,kObdAccRW,TEST_DOMAIN)
        OBD_END_INDEX(0x6100)

        OBD_BEGIN_INDEX_RAM(0x6200, 0x02, NULL)
            OBD_SUBINDEX_RAM_VAR(0x6200, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, number_of_entries, 0x1)
            OBD_SUBINDEX_RAM_USERDEF(0x6200, 0x01, kObdTypeUInt8, kObdAccVPRW, tObdUnsigned8, Recvb1, 0x0)
        OBD_END_INDEX(0x6200)
#endif

    OBD_END_PART ()

OBD_END ()

#define OBD_UNDEFINE_MACRO
    #include <oplk/obdmacro.h>
#undef OBD_UNDEFINE_MACRO
