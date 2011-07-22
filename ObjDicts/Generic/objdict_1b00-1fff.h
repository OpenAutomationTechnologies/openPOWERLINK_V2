//-----------------------------------------------------------------
//  Generic Communication Profile Area 1B00h - 1FFFh
//-----------------------------------------------------------------

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
        // Object 1C00h: DLL_MNCRCError_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1C00, 0x04, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1C00, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 0x03)
            EPL_OBD_SUBINDEX_RAM_USERDEF_NOINIT(0x1C00, 0x01, kEplObdTypUInt32, kEplObdAccRW, tEplObdUnsigned32, CumulativeCnt_U32)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x1C00, 0x02, kEplObdTypUInt32, kEplObdAccR, tEplObdUnsigned32, ThresholdCnt_U32, 0x0)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x1C00, 0x03, kEplObdTypUInt32, kEplObdAccSRW, tEplObdUnsigned32, Threshold_U32, 15)
        EPL_OBD_END_INDEX(0x1C0F)

        // Object 1C02h: DLL_MNCycTimeExceed_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1C02, 0x04, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1C02, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 0x03)
            EPL_OBD_SUBINDEX_RAM_USERDEF_NOINIT(0x1C02, 0x01, kEplObdTypUInt32, kEplObdAccRW, tEplObdUnsigned32, CumulativeCnt_U32)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x1C02, 0x02, kEplObdTypUInt32, kEplObdAccR, tEplObdUnsigned32, ThresholdCnt_U32, 0x0)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x1C02, 0x03, kEplObdTypUInt32, kEplObdAccSRW, tEplObdUnsigned32, Threshold_U32, 15)
        EPL_OBD_END_INDEX(0x1C0F)

        // Object 1C07h: DLL_MNCNLossPResCumCnt_AU32
        EPL_OBD_RAM_INDEX_RAM_VARARRAY_NOINIT(0x1C07, 254, NULL, kEplObdTypUInt32, kEplObdAccRW, tEplObdUnsigned32, DLL_MNCNLossPResCumCnt_AU32)

        // Object 1C08h: DLL_MNCNLossPResThrCnt_AU32
        EPL_OBD_RAM_INDEX_RAM_VARARRAY(0x1C08, 254, NULL, kEplObdTypUInt32, kEplObdAccR, tEplObdUnsigned32, DLL_MNCNLossPResThrCnt_AU32, 0)

        // Object 1C09h: DLL_MNCNLossPResThreshold_AU32
        EPL_OBD_RAM_INDEX_RAM_VARARRAY(0x1C09, 254, NULL, kEplObdTypUInt32, kEplObdAccSRW, tEplObdUnsigned32, DLL_MNCNLossPResThreshold_AU32, 15)
#endif

        // Object 1C0Bh: DLL_CNLossSoC_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1C0B, 0x04, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1C0B, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 3)
            EPL_OBD_SUBINDEX_RAM_USERDEF_NOINIT(0x1C0B, 0x01, kEplObdTypUInt32, kEplObdAccRW, tEplObdUnsigned32, CumulativeCnt_U32)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x1C0B, 0x02, kEplObdTypUInt32, kEplObdAccR, tEplObdUnsigned32, ThresholdCnt_U32, 0)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x1C0B, 0x03, kEplObdTypUInt32, kEplObdAccSRW, tEplObdUnsigned32, Threshold_U32, 15)
        EPL_OBD_END_INDEX(0x1C0B)

        // Object 1C0Dh: DLL_CNLossPReq_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1C0D, 0x04, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1C0D, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 3)
            EPL_OBD_SUBINDEX_RAM_USERDEF_NOINIT(0x1C0D, 0x01, kEplObdTypUInt32, kEplObdAccRW, tEplObdUnsigned32, CumulativeCnt_U32)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x1C0D, 0x02, kEplObdTypUInt32, kEplObdAccR, tEplObdUnsigned32, ThresholdCnt_U32, 0)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x1C0D, 0x03, kEplObdTypUInt32, kEplObdAccSRW, tEplObdUnsigned32, Threshold_U32, 15)
        EPL_OBD_END_INDEX(0x1C0D)

        // Object 1C0Fh: DLL_CNCRCError_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1C0F, 0x04, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1C0F, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 0x03)
            EPL_OBD_SUBINDEX_RAM_USERDEF_NOINIT(0x1C0F, 0x01, kEplObdTypUInt32, kEplObdAccRW, tEplObdUnsigned32, CumulativeCnt_U32)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x1C0F, 0x02, kEplObdTypUInt32, kEplObdAccR, tEplObdUnsigned32, ThresholdCnt_U32, 0x0)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x1C0F, 0x03, kEplObdTypUInt32, kEplObdAccSRW, tEplObdUnsigned32, Threshold_U32, 0x1)
        EPL_OBD_END_INDEX(0x1C0F)

        // Object 1C14h: DLL_LossOfFrameTolerance_U32 in [ns]
        EPL_OBD_BEGIN_INDEX_RAM(0x1C14, 0x01, EplApiCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1C14, 0x00, kEplObdTypUInt32, kEplObdAccSRW, tEplObdUnsigned32, LossOfFrameTolerance, 300000)
        EPL_OBD_END_INDEX(0x1C14)

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_VETH)) != 0)
        // Object 1E40h: NWL_IpAddrTable_0h_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1E40, 0x06, EplApiCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1E40, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 0x05)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1E40, 0x01, kEplObdTypUInt16, kEplObdAccR, tEplObdUnsigned16, IfIndex_U16, 0x01)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1E40, 0x02, kEplObdTypUInt32, kEplObdAccR, tEplObdUnsigned32, Addr_IPAD, 0xC0A86401)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1E40, 0x03, kEplObdTypUInt32, kEplObdAccR, tEplObdUnsigned32, NetMask_IPAD, 0xFFFFFF00)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1E40, 0x04, kEplObdTypUInt16, kEplObdAccR, tEplObdUnsigned16, ReasmMaxSize_U16, 50000)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1E40, 0x05, kEplObdTypUInt32, kEplObdAccSRW, tEplObdUnsigned32, DefaultGateway_IPAD, 0xC0A864FE)
        EPL_OBD_END_INDEX(0x1E40)
#endif

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_CFM)) != 0)
        EPL_OBD_RAM_INDEX_RAM_VARARRAY_NOINIT(0x1F22, EPL_NMT_MAX_NODE_ID, EplCfmuCbObdAccess, kEplObdTypDomain, kEplObdAccSRW, Domain, CFM_ConciseDcfList_ADOM)
        EPL_OBD_RAM_INDEX_RAM_ARRAY(0x1F26, EPL_NMT_MAX_NODE_ID, NULL, kEplObdTypUInt32, kEplObdAccSRW, tEplObdUnsigned32, CFM_ExpConfDateList_AU32, 0)
        EPL_OBD_RAM_INDEX_RAM_ARRAY(0x1F27, EPL_NMT_MAX_NODE_ID, NULL, kEplObdTypUInt32, kEplObdAccSRW, tEplObdUnsigned32, CFM_ExpConfTimeList_AU32, 0)
#endif

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
        // Object 1F80h: NMT_StartUp_U32
        EPL_OBD_BEGIN_INDEX_RAM(0x1F80, 0x01, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F80, 0x00, kEplObdTypUInt32, kEplObdAccSRW, tEplObdUnsigned32, NMT_StartUp_U32, 0x00)
        EPL_OBD_END_INDEX(0x1F80)
#endif

#if EPL_NMT_MAX_NODE_ID > 0
        // Object 1F81h: NMT_NodeAssignment_AU32
        EPL_OBD_RAM_INDEX_RAM_ARRAY(0x1F81, EPL_NMT_MAX_NODE_ID, NULL, kEplObdTypUInt32, kEplObdAccSRW, tEplObdUnsigned32, NMT_NodeAssignment_AU32, 0)
#endif

        // Object 1F82h: NMT_FeatureFlags_U32
        EPL_OBD_BEGIN_INDEX_RAM(0x1F82, 0x01, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F82, 0x00, kEplObdTypUInt32, kEplObdAccR, tEplObdUnsigned32, NMT_FeatureFlags_U32, EPL_DEF_FEATURE_FLAGS)
        EPL_OBD_END_INDEX(0x1F82)

        // Object 1F83h: NMT_EPLVersion_U8
        EPL_OBD_BEGIN_INDEX_RAM(0x1F83, 0x01, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F83, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NMT_EPLVersion_U8, 0x20)
        EPL_OBD_END_INDEX(0x1F83)

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
        // Object 1F84h: NMT_MNDeviceTypeIdList_AU32
        EPL_OBD_RAM_INDEX_RAM_ARRAY(0x1F84, 254, NULL, kEplObdTypUInt32, kEplObdAccSRW, tEplObdUnsigned32, NMT_MNDeviceTypeIdList_AU32, 0)

        // Object 1F89h: NMT_BootTime_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1F89, 0x06, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F89, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 0x05)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F89, 0x01, kEplObdTypUInt32, kEplObdAccSRW, tEplObdUnsigned32, MNWaitNotAct_U32, 1000000)        // in [us]
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F89, 0x02, kEplObdTypUInt32, kEplObdAccSRW, tEplObdUnsigned32, MNTimeoutPreOp1_U32, 500000)      // in [us]
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F89, 0x03, kEplObdTypUInt32, kEplObdAccSRW, tEplObdUnsigned32, MNWaitPreOp1_U32, 1000000)         // in [us]
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F89, 0x04, kEplObdTypUInt32, kEplObdAccSRW, tEplObdUnsigned32, MNTimeoutPreOp2_U32, 5000000)      // in [us]
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F89, 0x05, kEplObdTypUInt32, kEplObdAccSRW, tEplObdUnsigned32, MNTimeoutReadyToOp_U32, 500000)   // in [us]
        EPL_OBD_END_INDEX(0x1F89)

        // Object 1F8Ah: NMT_MNCycleTiming_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1F8A, 0x03, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F8A, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F8A, 0x01, kEplObdTypUInt32, kEplObdAccSRW, tEplObdUnsigned32, WaitSoCPReq_U32, 1000)    // in [ns]
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F8A, 0x02, kEplObdTypUInt32, kEplObdAccSRW, tEplObdUnsigned32, AsyncSlotTimeout_U32, 100000) // in [ns]
        EPL_OBD_END_INDEX(0x1F8A)

        // Object 1F8Bh: NMT_MNPReqPayloadLimitList_AU16
        EPL_OBD_RAM_INDEX_RAM_ARRAY(0x1F8B, 254, NULL, kEplObdTypUInt16, kEplObdAccSRW, tEplObdUnsigned16, NMT_MNPReqPayloadLimitList_AU16, 36)
#endif

        // Object 1F8Ch: NMT_CurrNMTState_U8
        EPL_OBD_BEGIN_INDEX_RAM(0x1F8C, 0x01, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F8C, 0x00, kEplObdTypUInt8, (kEplObdAccR | kEplObdAccPdo), tEplObdUnsigned8, NMT_CurrNMTState_U8, 0x1C)
        EPL_OBD_END_INDEX(0x1F8C)

#if EPL_NMT_MAX_NODE_ID > 0
        // Object 1F8Dh: NMT_PResPayloadLimitList_AU16
        EPL_OBD_RAM_INDEX_RAM_ARRAY(0x1F8D, EPL_NMT_MAX_NODE_ID, NULL, kEplObdTypUInt16, kEplObdAccSRW, tEplObdUnsigned16, NMT_PResPayloadLimitList_AU16, 36)
#endif

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
        // Object 1F8Eh: NMT_MNNodeCurrState_AU8
        EPL_OBD_RAM_INDEX_RAM_ARRAY(0x1F8E, 254, NULL, kEplObdTypUInt8, kEplObdAccR, tEplObdUnsigned8, NMT_MNNodeCurrState_AU8, 0x1C)

        // Object 1F8Fh: NMT_MNNodeExpState_AU8
        EPL_OBD_RAM_INDEX_RAM_ARRAY(0x1F8F, 254, NULL, kEplObdTypUInt8, kEplObdAccR, tEplObdUnsigned8, NMT_MNNodeExpState_AU8, 0x1C)

        // Object 1F92h: NMT_MNCNPResTimeout_AU32 in [ns]
        EPL_OBD_RAM_INDEX_RAM_ARRAY(0x1F92, 254, NULL, kEplObdTypUInt32, kEplObdAccSRW, tEplObdUnsigned32, NMT_MNCNPResTimeout_AU32, 140000) // in [ns]
#endif

        // Object 1F93h: NMT_EPLNodeID_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1F93, 0x03, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F93, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F93, 0x01, kEplObdTypUInt8, kEplObdAccR, tEplObdUnsigned8, NodeID_U8, 0x00)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F93, 0x02, kEplObdTypBool, kEplObdAccR, tEplObdBoolean, NodeIDByHW_BOOL, 0x0)
        EPL_OBD_END_INDEX(0x1F93)

        // Object 1F98h: NMT_CycleTiming_REC
#if EPL_DLL_PRES_CHAINING_CN == FALSE
        EPL_OBD_BEGIN_INDEX_RAM(0x1F98, 0x09, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F98, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 0x08)
#else
        EPL_OBD_BEGIN_INDEX_RAM(0x1F98, 0x0E, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F98, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 0x0E)
#endif
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F98, 0x01, kEplObdTypUInt16, kEplObdAccR, tEplObdUnsigned16, IsochrTxMaxPayload_U16, 0x00)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F98, 0x02, kEplObdTypUInt16, kEplObdAccR, tEplObdUnsigned16, IsochrRxMaxPayload_U16, 0x00)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F98, 0x03, kEplObdTypUInt32, kEplObdAccR, tEplObdUnsigned32, PResMaxLatency_U32, 0x00)     // in [ns]
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F98, 0x04, kEplObdTypUInt16, kEplObdAccSRW, tEplObdUnsigned16, PReqActPayloadLimit_U16, 36)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F98, 0x05, kEplObdTypUInt16, kEplObdAccSRW, tEplObdUnsigned16, PResActPayloadLimit_U16, 36)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F98, 0x06, kEplObdTypUInt32, kEplObdAccR, tEplObdUnsigned32, ASndMaxLatency_U32, 0x00)     // in [ns]
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F98, 0x07, kEplObdTypUInt8, kEplObdAccSRW, tEplObdUnsigned8, MultiplCycleCnt_U8, 0x00)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F98, 0x08, kEplObdTypUInt16, kEplObdAccSRW, tEplObdUnsigned16, AsyncMTU_U16, EPL_C_DLL_MIN_ASYNC_MTU)
//            EPL_OBD_SUBINDEX_RAM_VAR(0x1F98, 0x09, kEplObdTypUInt16, kEplObdAccRW, tEplObdUnsigned16, Prescaler_U16, 0x02)
#if EPL_DLL_PRES_CHAINING_CN != FALSE
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F98, 0x0A, kEplObdTypUInt8, kEplObdAccR, tEplObdUnsigned8, PResMode_U8, 0x00)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F98, 0x0B, kEplObdTypUInt32, kEplObdAccR, tEplObdUnsigned32, PResTimeFirst_U32, 0x00)      // in [ns]
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F98, 0x0C, kEplObdTypUInt32, kEplObdAccR, tEplObdUnsigned32, PResTimeSecond_U32, 0x00)     // in [ns]
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F98, 0x0D, kEplObdTypUInt32, kEplObdAccR, tEplObdUnsigned32, SyncMNDelayFirst_U32, 0x00)   // in [ns]
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F98, 0x0E, kEplObdTypUInt32, kEplObdAccR, tEplObdUnsigned32, SyncMNDelaySecond_U32, 0x00)  // in [ns]
#endif
        EPL_OBD_END_INDEX(0x1F98)

        // Object 1F99h: NMT_CNBasicEthernetTimeout_U32 in [us]
        EPL_OBD_BEGIN_INDEX_RAM(0x1F99, 0x01, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F99, 0x00, kEplObdTypUInt32, kEplObdAccSRW, tEplObdUnsigned32, NMT_CNBasicEthernetTimeout_U32, 5000000)  // in [us]
        EPL_OBD_END_INDEX(0x1F99)

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_VETH)) != 0)
        // Object 1F9Ah: NMT_HostName_VS
        EPL_OBD_BEGIN_INDEX_RAM(0x1F9A, 0x01, NULL)
           EPL_OBD_SUBINDEX_RAM_VSTRING(0x1F9A, 0x00, kEplObdAccR, host_name, 33, "")
        EPL_OBD_END_INDEX(0x1F9A)
#endif

        // Object 1F9Eh: NMT_ResetCmd_U8
        EPL_OBD_BEGIN_INDEX_RAM(0x1F9E, 0x01, EplApiCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F9E, 0x00, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NMT_ResetCmd_U8, 0xFF)
        EPL_OBD_END_INDEX(0x1F9E)

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
        // Object 1F9Fh: NMT_RequestCmd_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1F9F, 0x04, EplApiCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F9F, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 0x03)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F9F, 0x01, kEplObdTypBool, kEplObdAccRW, tEplObdBoolean, Release_BOOL, FALSE)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F9F, 0x02, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, CmdID_U8, 0xFF)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1F9F, 0x03, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, CmdTarget_U8, 0x00)
        EPL_OBD_END_INDEX(0x1F9F)
#endif

