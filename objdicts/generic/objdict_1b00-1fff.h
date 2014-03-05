/**
********************************************************************************
\file   objdicts/generic/objdict_1b00-1fff.h

\brief  OD definitions for generic communication area 0x1B00 - 0x1FFF

This file contains the object dictionary definition for the generic communication
area from 0x1B00 - 0x1FFF.
*******************************************************************************/

#if defined(CONFIG_INCLUDE_NMT_MN)
        // Object 1C00h: DLL_MNCRCError_REC
        OBD_BEGIN_INDEX_RAM(0x1C00, 0x04, errhndu_cbObdAccess)
            OBD_SUBINDEX_RAM_VAR(0x1C00, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x03)
            OBD_SUBINDEX_RAM_USERDEF_NOINIT(0x1C00, 0x01, kObdTypeUInt32, kObdAccRW, tObdUnsigned32, CumulativeCnt_U32)
            OBD_SUBINDEX_RAM_USERDEF(0x1C00, 0x02, kObdTypeUInt32, kObdAccR, tObdUnsigned32, ThresholdCnt_U32, 0x0)
            OBD_SUBINDEX_RAM_USERDEF(0x1C00, 0x03, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, Threshold_U32, 15)
        OBD_END_INDEX(0x1C00)

        // Object 1C02h: DLL_MNCycTimeExceed_REC
        OBD_BEGIN_INDEX_RAM(0x1C02, 0x04, errhndu_cbObdAccess)
            OBD_SUBINDEX_RAM_VAR(0x1C02, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x03)
            OBD_SUBINDEX_RAM_USERDEF_NOINIT(0x1C02, 0x01, kObdTypeUInt32, kObdAccRW, tObdUnsigned32, CumulativeCnt_U32)
            OBD_SUBINDEX_RAM_USERDEF(0x1C02, 0x02, kObdTypeUInt32, kObdAccR, tObdUnsigned32, ThresholdCnt_U32, 0x0)
            OBD_SUBINDEX_RAM_USERDEF(0x1C02, 0x03, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, Threshold_U32, 15)
        OBD_END_INDEX(0x1C02)

        // Object 1C07h: DLL_MNCNLossPResCumCnt_AU32
        OBD_RAM_INDEX_RAM_VARARRAY_NOINIT(0x1C07, 254, errhndu_mnCnLossPresCbObdAccess, kObdTypeUInt32, kObdAccRW, tObdUnsigned32, DLL_MNCNLossPResCumCnt_AU32)

        // Object 1C08h: DLL_MNCNLossPResThrCnt_AU32
        OBD_RAM_INDEX_RAM_VARARRAY(0x1C08, 254, errhndu_mnCnLossPresCbObdAccess, kObdTypeUInt32, kObdAccR, tObdUnsigned32, DLL_MNCNLossPResThrCnt_AU32, 0)

        // Object 1C09h: DLL_MNCNLossPResThreshold_AU32
        OBD_RAM_INDEX_RAM_VARARRAY(0x1C09, 254, errhndu_mnCnLossPresCbObdAccess, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, DLL_MNCNLossPResThreshold_AU32, 15)
#endif

        // Object 1C0Bh: DLL_CNLossSoC_REC
        OBD_BEGIN_INDEX_RAM(0x1C0B, 0x04, errhndu_cbObdAccess)
            OBD_SUBINDEX_RAM_VAR(0x1C0B, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 3)
            OBD_SUBINDEX_RAM_USERDEF_NOINIT(0x1C0B, 0x01, kObdTypeUInt32, kObdAccRW, tObdUnsigned32, CumulativeCnt_U32)
            OBD_SUBINDEX_RAM_USERDEF(0x1C0B, 0x02, kObdTypeUInt32, kObdAccR, tObdUnsigned32, ThresholdCnt_U32, 0)
            OBD_SUBINDEX_RAM_USERDEF(0x1C0B, 0x03, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, Threshold_U32, 15)
        OBD_END_INDEX(0x1C0B)

        // Object 1C0Dh: DLL_CNLossPReq_REC
        OBD_BEGIN_INDEX_RAM(0x1C0D, 0x04, errhndu_cbObdAccess)
            OBD_SUBINDEX_RAM_VAR(0x1C0D, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 3)
            OBD_SUBINDEX_RAM_USERDEF_NOINIT(0x1C0D, 0x01, kObdTypeUInt32, kObdAccRW, tObdUnsigned32, CumulativeCnt_U32)
            OBD_SUBINDEX_RAM_USERDEF(0x1C0D, 0x02, kObdTypeUInt32, kObdAccR, tObdUnsigned32, ThresholdCnt_U32, 0)
            OBD_SUBINDEX_RAM_USERDEF(0x1C0D, 0x03, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, Threshold_U32, 15)
        OBD_END_INDEX(0x1C0D)

        // Object 1C0Fh: DLL_CNCRCError_REC
        OBD_BEGIN_INDEX_RAM(0x1C0F, 0x04, errhndu_cbObdAccess)
            OBD_SUBINDEX_RAM_VAR(0x1C0F, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 3)
            OBD_SUBINDEX_RAM_USERDEF_NOINIT(0x1C0F, 0x01, kObdTypeUInt32, kObdAccRW, tObdUnsigned32, CumulativeCnt_U32)
            OBD_SUBINDEX_RAM_USERDEF(0x1C0F, 0x02, kObdTypeUInt32, kObdAccR, tObdUnsigned32, ThresholdCnt_U32, 0)
            OBD_SUBINDEX_RAM_USERDEF(0x1C0F, 0x03, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, Threshold_U32, 15)
        OBD_END_INDEX(0x1C0F)

        // Object 1C14h: DLL_LossOfSocTolerance_U32 in [ns]
        OBD_BEGIN_INDEX_RAM(0x1C14, 0x01, ctrlu_cbObdAccess)
            OBD_SUBINDEX_RAM_VAR(0x1C14, 0x00, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, LossOfSocTolerance, 100000)
        OBD_END_INDEX(0x1C14)

#if defined(CONFIG_INCLUDE_VETH)
        // Object 1E40h: NWL_IpAddrTable_0h_REC
        OBD_BEGIN_INDEX_RAM(0x1E40, 0x06, ctrlu_cbObdAccess)
            OBD_SUBINDEX_RAM_VAR(0x1E40, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x05)
            OBD_SUBINDEX_RAM_VAR(0x1E40, 0x01, kObdTypeUInt16, kObdAccR, tObdUnsigned16, IfIndex_U16, 0x01)
            OBD_SUBINDEX_RAM_VAR(0x1E40, 0x02, kObdTypeUInt32, kObdAccR, tObdUnsigned32, Addr_IPAD, 0xC0A86401)
            OBD_SUBINDEX_RAM_VAR(0x1E40, 0x03, kObdTypeUInt32, kObdAccR, tObdUnsigned32, NetMask_IPAD, 0xFFFFFF00)
            OBD_SUBINDEX_RAM_VAR(0x1E40, 0x04, kObdTypeUInt16, kObdAccR, tObdUnsigned16, ReasmMaxSize_U16, 50000)
            OBD_SUBINDEX_RAM_VAR(0x1E40, 0x05, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, DefaultGateway_IPAD, 0xC0A864FE)
        OBD_END_INDEX(0x1E40)
#endif

#if defined(CONFIG_INCLUDE_CFM)
        OBD_RAM_INDEX_RAM_VARARRAY_NOINIT(0x1F22, NMT_MAX_NODE_ID, cfmu_cbObdAccess, kObdTypeDomain, kObdAccSRW, Domain, CFM_ConciseDcfList_ADOM)
        OBD_RAM_INDEX_RAM_ARRAY(0x1F26, NMT_MAX_NODE_ID, NULL, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, CFM_ExpConfDateList_AU32, 0)
        OBD_RAM_INDEX_RAM_ARRAY(0x1F27, NMT_MAX_NODE_ID, NULL, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, CFM_ExpConfTimeList_AU32, 0)
#endif

#if defined(CONFIG_INCLUDE_NMT_MN)
        // Object 1F80h: NMT_StartUp_U32
        OBD_BEGIN_INDEX_RAM(0x1F80, 0x01, NULL)
            OBD_SUBINDEX_RAM_VAR(0x1F80, 0x00, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, NMT_StartUp_U32, 0x00)
        OBD_END_INDEX(0x1F80)
#endif

#if NMT_MAX_NODE_ID > 0
        // Object 1F81h: NMT_NodeAssignment_AU32
        OBD_RAM_INDEX_RAM_ARRAY_ALT(0x1F81, NMT_MAX_NODE_ID, NULL, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, NMT_NodeAssignment_AU32, 0)
#endif

        // Object 1F82h: NMT_FeatureFlags_U32
        OBD_BEGIN_INDEX_RAM(0x1F82, 0x01, NULL)
            OBD_SUBINDEX_RAM_VAR(0x1F82, 0x00, kObdTypeUInt32, kObdAccR, tObdUnsigned32, NMT_FeatureFlags_U32, PLK_DEF_FEATURE_FLAGS)
        OBD_END_INDEX(0x1F82)

        // Object 1F83h: NMT_EPLVersion_U8
        OBD_BEGIN_INDEX_RAM(0x1F83, 0x01, NULL)
            OBD_SUBINDEX_RAM_VAR(0x1F83, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NMT_EPLVersion_U8, 0x20)
        OBD_END_INDEX(0x1F83)

#if defined(CONFIG_INCLUDE_NMT_MN)
        // Object 1F84h: NMT_MNDeviceTypeIdList_AU32
        OBD_RAM_INDEX_RAM_ARRAY_ALT(0x1F84, 254, NULL, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, NMT_MNDeviceTypeIdList_AU32, 0)

        // Object 1F89h: NMT_BootTime_REC
        OBD_BEGIN_INDEX_RAM(0x1F89, 0x06, NULL)
            OBD_SUBINDEX_RAM_VAR(0x1F89, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x05)
            OBD_SUBINDEX_RAM_VAR(0x1F89, 0x01, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, MNWaitNotAct_U32, 1000000)        // in [us]
            OBD_SUBINDEX_RAM_VAR(0x1F89, 0x02, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, MNTimeoutPreOp1_U32, 500000)      // in [us]
            OBD_SUBINDEX_RAM_VAR(0x1F89, 0x03, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, MNWaitPreOp1_U32, 500000)         // in [us]
            OBD_SUBINDEX_RAM_VAR(0x1F89, 0x04, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, MNTimeoutPreOp2_U32, 5000000)      // in [us]
            OBD_SUBINDEX_RAM_VAR(0x1F89, 0x05, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, MNTimeoutReadyToOp_U32, 500000)   // in [us]
        OBD_END_INDEX(0x1F89)

        // Object 1F8Ah: NMT_MNCycleTiming_REC
        OBD_BEGIN_INDEX_RAM(0x1F8A, 0x03, NULL)
            OBD_SUBINDEX_RAM_VAR(0x1F8A, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR(0x1F8A, 0x01, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, WaitSoCPReq_U32, 1000)    // in [ns]
            OBD_SUBINDEX_RAM_VAR(0x1F8A, 0x02, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, AsyncSlotTimeout_U32, 100000) // in [ns]
        OBD_END_INDEX(0x1F8A)

        // Object 1F8Bh: NMT_MNPReqPayloadLimitList_AU16
        OBD_RAM_INDEX_RAM_ARRAY_ALT(0x1F8B, 254, NULL, kObdTypeUInt16, kObdAccSRW, tObdUnsigned16, NMT_MNPReqPayloadLimitList_AU16, 36)
#endif

        // Object 1F8Ch: NMT_CurrNMTState_U8
        OBD_BEGIN_INDEX_RAM(0x1F8C, 0x01, NULL)
            OBD_SUBINDEX_RAM_VAR(0x1F8C, 0x00, kObdTypeUInt8, (kObdAccR | kObdAccPdo), tObdUnsigned8, NMT_CurrNMTState_U8, 0x1C)
        OBD_END_INDEX(0x1F8C)

#if NMT_MAX_NODE_ID > 0
        // Object 1F8Dh: NMT_PResPayloadLimitList_AU16
        OBD_RAM_INDEX_RAM_ARRAY_ALT(0x1F8D, NMT_MAX_NODE_ID, NULL, kObdTypeUInt16, kObdAccSRW, tObdUnsigned16, NMT_PResPayloadLimitList_AU16, 36)
        #endif

#if defined(CONFIG_INCLUDE_NMT_MN)
        // Object 1F8Eh: NMT_MNNodeCurrState_AU8
        OBD_RAM_INDEX_RAM_ARRAY(0x1F8E, 254, NULL, kObdTypeUInt8, kObdAccR, tObdUnsigned8, NMT_MNNodeCurrState_AU8, 0x1C)

        // Object 1F8Fh: NMT_MNNodeExpState_AU8
        OBD_RAM_INDEX_RAM_ARRAY(0x1F8F, 254, NULL, kObdTypeUInt8, kObdAccR, tObdUnsigned8, NMT_MNNodeExpState_AU8, 0x1C)

        // Object 1F92h: NMT_MNCNPResTimeout_AU32 in [ns]
        OBD_RAM_INDEX_RAM_ARRAY_ALT(0x1F92, 254, NULL, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, NMT_MNCNPResTimeout_AU32, 140000) // in [ns]
#endif

        // Object 1F93h: NMT_EPLNodeID_REC
        OBD_BEGIN_INDEX_RAM(0x1F93, 0x03, NULL)
            OBD_SUBINDEX_RAM_VAR(0x1F93, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x02)
            OBD_SUBINDEX_RAM_VAR(0x1F93, 0x01, kObdTypeUInt8, kObdAccR, tObdUnsigned8, NodeID_U8, 0x00)
            OBD_SUBINDEX_RAM_VAR(0x1F93, 0x02, kObdTypeBool, kObdAccR, tObdBoolean, NodeIDByHW_BOOL, 0x0)
        OBD_END_INDEX(0x1F93)

        // Object 1F98h: NMT_CycleTiming_REC
#if CONFIG_DLL_PRES_CHAINING_CN == FALSE
        OBD_BEGIN_INDEX_RAM(0x1F98, 0x0A, NULL)
            OBD_SUBINDEX_RAM_VAR(0x1F98, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x09)
#else
        OBD_BEGIN_INDEX_RAM(0x1F98, 0x0F, NULL)
            OBD_SUBINDEX_RAM_VAR(0x1F98, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x0E)
#endif
            OBD_SUBINDEX_RAM_VAR(0x1F98, 0x01, kObdTypeUInt16, kObdAccR, tObdUnsigned16, IsochrTxMaxPayload_U16, 0x00)
            OBD_SUBINDEX_RAM_VAR(0x1F98, 0x02, kObdTypeUInt16, kObdAccR, tObdUnsigned16, IsochrRxMaxPayload_U16, 0x00)
            OBD_SUBINDEX_RAM_VAR(0x1F98, 0x03, kObdTypeUInt32, kObdAccR, tObdUnsigned32, PResMaxLatency_U32, 0x00)     // in [ns]
            OBD_SUBINDEX_RAM_VAR(0x1F98, 0x04, kObdTypeUInt16, kObdAccSRW, tObdUnsigned16, PReqActPayloadLimit_U16, 36)
            OBD_SUBINDEX_RAM_VAR(0x1F98, 0x05, kObdTypeUInt16, kObdAccSRW, tObdUnsigned16, PResActPayloadLimit_U16, 36)
            OBD_SUBINDEX_RAM_VAR(0x1F98, 0x06, kObdTypeUInt32, kObdAccR, tObdUnsigned32, ASndMaxLatency_U32, 0x00)     // in [ns]
            OBD_SUBINDEX_RAM_VAR(0x1F98, 0x07, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, MultiplCycleCnt_U8, 0x00)
            OBD_SUBINDEX_RAM_VAR(0x1F98, 0x08, kObdTypeUInt16, kObdAccSRW, tObdUnsigned16, AsyncMTU_U16, C_DLL_MIN_ASYNC_MTU)
            OBD_SUBINDEX_RAM_VAR_RG(0x1F98, 0x09, kObdTypeUInt16, kObdAccSRW, tObdUnsigned16, Prescaler_U16, 0x02, 0, 1000)
#if CONFIG_DLL_PRES_CHAINING_CN != FALSE
            OBD_SUBINDEX_RAM_VAR(0x1F98, 0x0A, kObdTypeUInt8, kObdAccR, tObdUnsigned8, PResMode_U8, 0x00)
            OBD_SUBINDEX_RAM_VAR(0x1F98, 0x0B, kObdTypeUInt32, kObdAccR, tObdUnsigned32, PResTimeFirst_U32, 0x00)      // in [ns]
            OBD_SUBINDEX_RAM_VAR(0x1F98, 0x0C, kObdTypeUInt32, kObdAccR, tObdUnsigned32, PResTimeSecond_U32, 0x00)     // in [ns]
            OBD_SUBINDEX_RAM_VAR(0x1F98, 0x0D, kObdTypeUInt32, kObdAccR, tObdUnsigned32, SyncMNDelayFirst_U32, 0x00)   // in [ns]
            OBD_SUBINDEX_RAM_VAR(0x1F98, 0x0E, kObdTypeUInt32, kObdAccR, tObdUnsigned32, SyncMNDelaySecond_U32, 0x00)  // in [ns]
#endif
        OBD_END_INDEX(0x1F98)

        // Object 1F99h: NMT_CNBasicEthernetTimeout_U32 in [us]
        OBD_BEGIN_INDEX_RAM(0x1F99, 0x01, NULL)
            OBD_SUBINDEX_RAM_VAR(0x1F99, 0x00, kObdTypeUInt32, kObdAccSRW, tObdUnsigned32, NMT_CNBasicEthernetTimeout_U32, 5000000)  // in [us]
        OBD_END_INDEX(0x1F99)

#if defined(CONFIG_INCLUDE_VETH)
        // Object 1F9Ah: NMT_HostName_VSTR
        OBD_BEGIN_INDEX_RAM(0x1F9A, 0x01, NULL)
           OBD_SUBINDEX_RAM_VSTRING(0x1F9A, 0x00, kObdAccSRW, NMT_HostName_VSTR, 34, "")
        OBD_END_INDEX(0x1F9A)
#endif

#if NMT_MAX_NODE_ID > 0
        // Object 1F9Bh: NMT_MultiplCycleAssign_AU8
        OBD_RAM_INDEX_RAM_ARRAY_ALT(0x1F9B, NMT_MAX_NODE_ID, NULL, kObdTypeUInt8, kObdAccSRW, tObdUnsigned8, NMT_MultiplCycleAssign_AU8, 0)
#endif

        // Object 1F9Eh: NMT_ResetCmd_U8
        OBD_BEGIN_INDEX_RAM(0x1F9E, 0x01, ctrlu_cbObdAccess)
            OBD_SUBINDEX_RAM_VAR(0x1F9E, 0x00, kObdTypeUInt8, kObdAccRW, tObdUnsigned8, NMT_ResetCmd_U8, 0xFF)
        OBD_END_INDEX(0x1F9E)

#if defined(CONFIG_INCLUDE_NMT_MN)
        // Object 1F9Fh: NMT_RequestCmd_REC
        OBD_BEGIN_INDEX_RAM(0x1F9F, 0x04, ctrlu_cbObdAccess)
            OBD_SUBINDEX_RAM_VAR(0x1F9F, 0x00, kObdTypeUInt8, kObdAccConst, tObdUnsigned8, NumberOfEntries, 0x03)
            OBD_SUBINDEX_RAM_VAR(0x1F9F, 0x01, kObdTypeBool, kObdAccRW, tObdBoolean, Release_BOOL, FALSE)
            OBD_SUBINDEX_RAM_VAR(0x1F9F, 0x02, kObdTypeUInt8, kObdAccRW, tObdUnsigned8, CmdID_U8, 0xFF)
            OBD_SUBINDEX_RAM_VAR(0x1F9F, 0x03, kObdTypeUInt8, kObdAccRW, tObdUnsigned8, CmdTarget_U8, 0x00)
        OBD_END_INDEX(0x1F9F)
#endif

