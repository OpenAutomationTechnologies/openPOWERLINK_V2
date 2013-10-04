-------------------------------------------------------------------------------
-- Process Data Interface (PDI) for
--    POWERLINK Communication Processor (PCP): Avalon
--    Application Processor (AP): Avalon
--
--       Copyright (C) 2010 B&R
--
--    Redistribution and use in source and binary forms, with or without
--    modification, are permitted provided that the following conditions
--    are met:
--
--    1. Redistributions of source code must retain the above copyright
--       notice, this list of conditions and the following disclaimer.
--
--    2. Redistributions in binary form must reproduce the above copyright
--       notice, this list of conditions and the following disclaimer in the
--       documentation and/or other materials provided with the distribution.
--
--    3. Neither the name of B&R nor the names of its
--       contributors may be used to endorse or promote products derived
--       from this software without prior written permission. For written
--       permission, please contact office@br-automation.com
--
--    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
--    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
--    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
--    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
--    COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
--    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
--    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
--    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
--    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
--    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
--    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
--    POSSIBILITY OF SUCH DAMAGE.
--
-------------------------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.std_logic_arith.all;
USE ieee.std_logic_unsigned.all;
USE ieee.math_real.log2;
USE ieee.math_real.ceil;
USE work.memMap.all; --used for memory mapping (alignment, ...)

entity pdi is
    generic (
            genOnePdiClkDomain_g        :        boolean := false;
            iPdiRev_g                    :        integer := 0; --for HW/SW match verification (0..65535)
            pcpSysId                    :       integer := 0; --for HW/SW match verification (0..65535)
            iRpdos_g                    :        integer := 3;
            iTpdos_g                    :        integer := 1;
            genABuf1_g                    :        boolean := true; --if false iABuf1_g must be set to 0!
            genABuf2_g                    :        boolean := true; --if false iABuf2_g must be set to 0!
            genLedGadget_g                :        boolean := false;
            genTimeSync_g                :        boolean := false;
            genEvent_g                    :        boolean := false;
            --PDO buffer size *3
            iTpdoBufSize_g                :        integer := 100;
            iRpdo0BufSize_g                :        integer := 116; --includes header
            iRpdo1BufSize_g                :        integer := 116; --includes header
            iRpdo2BufSize_g                :        integer := 116; --includes header
            --asynchronous buffer size
            iABuf1_g                    :        integer := 512; --includes header
            iABuf2_g                    :        integer := 512  --includes header
    );

    port (
            pcp_reset                    : in    std_logic;
            pcp_clk                      : in    std_logic;
            ap_reset                    : in    std_logic;
            ap_clk                        : in    std_logic;
        -- Avalon Slave Interface for PCP
            pcp_chipselect                : in    std_logic;
            pcp_read                    : in    std_logic;
            pcp_write                    : in    std_logic;
            pcp_byteenable                : in    std_logic_vector(3 DOWNTO 0);
            pcp_address                   : in    std_logic_vector(12 DOWNTO 0);
            pcp_writedata                 : in    std_logic_vector(31 DOWNTO 0);
            pcp_readdata                  : out   std_logic_vector(31 DOWNTO 0);
            pcp_waitrequest                : out    std_logic;
            pcp_irq                        : in    std_logic; --should be connected to the Time Cmp Toggle of openMAC!
        -- Avalon Slave Interface for AP
            ap_chipselect               : in    std_logic;
            ap_read                        : in    std_logic;
            ap_write                    : in    std_logic;
            ap_byteenable                 : in    std_logic_vector(3 DOWNTO 0);
            ap_address                  : in    std_logic_vector(12 DOWNTO 0);
            ap_writedata                : in    std_logic_vector(31 DOWNTO 0);
            ap_readdata                 : out   std_logic_vector(31 DOWNTO 0);
            ap_waitrequest                : out    std_logic;
            ap_irq                        : out    std_logic; --Sync Irq to the AP
        -- async interrupt
            ap_asyncIrq                    : out    std_logic; --Async Irq to the Ap
        -- LED
            ledsOut                        : out    std_logic_vector(15 downto 0) := (others => '0'); --LEDs: GPO7, ..., GPO0, O1, O0, PA1, PL1, PA0, PL0, E, S
            phyLink                        : in    std_logic_vector(1 downto 0); --link: phy1, phy0
            phyAct                        : in    std_logic_vector(1 downto 0); --acti: phy1, phy0
        --PDI change buffer triggers
            rpdo_change_tog                : in    std_logic_vector(2 downto 0);
            tpdo_change_tog                : in    std_logic
    );
end entity pdi;

architecture rtl of pdi is
------------------------------------------------------------------------------------------------------------------------
--types
---for pcp and ap side
type pdiSel_t is
    record
            pcp                         : std_logic;
            ap                             : std_logic;
    end record;
type pdiTrig_t is
    record
            pcp                         : std_logic_vector(3 downto 0);
            ap                             : std_logic_vector(3 downto 0);
    end record;
type pdi32Bit_t is
    record
            pcp                         : std_logic_vector(31 downto 0);
            ap                             : std_logic_vector(31 downto 0);
    end record;
------------------------------------------------------------------------------------------------------------------------
--constants
---memory mapping from outside (e.g. Avalon or SPI)
----max memory span of one space
constant    extMaxOneSpan                : integer := 2 * 1024; --2kB
constant    extLog2MaxOneSpan            : integer := integer(ceil(log2(real(extMaxOneSpan))));
----control / status register
constant    extCntStReg_c                : memoryMapping_t := (16#0000#, 16#98#);
----asynchronous buffers
constant    extABuf1Tx_c                : memoryMapping_t := (16#0800#, iABuf1_g); --header is included in generic value!
constant    extABuf1Rx_c                : memoryMapping_t := (16#1000#, iABuf1_g); --header is included in generic value!
constant    extABuf2Tx_c                : memoryMapping_t := (16#1800#, iABuf2_g); --header is included in generic value!
constant    extABuf2Rx_c                : memoryMapping_t := (16#2000#, iABuf2_g); --header is included in generic value!
----pdo buffer
constant    extTpdoBuf_c                : memoryMapping_t := (16#2800#, iTpdoBufSize_g); --header is included in generic value!
constant    extRpdo0Buf_c                : memoryMapping_t := (16#3000#, iRpdo0BufSize_g); --header is included in generic value!
constant    extRpdo1Buf_c                : memoryMapping_t := (16#3800#, iRpdo1BufSize_g); --header is included in generic value!
constant    extRpdo2Buf_c                : memoryMapping_t := (16#4000#, iRpdo2BufSize_g); --header is included in generic value!
---memory mapping inside the PDI's DPR
----control / status register
constant    intCntStReg_c                : memoryMapping_t := (16#0000#, 22 * 4); --bytes mapped to dpr (dword alignment!!!), note: 4 times a double buffer!
----asynchronous buffers
constant    intABuf1Tx_c                : memoryMapping_t := (intCntStReg_c.base + intCntStReg_c.span, align32(extABuf1Tx_c.span));
constant    intABuf1Rx_c                : memoryMapping_t := (intABuf1Tx_c.base + intABuf1Tx_c.span, align32(extABuf1Rx_c.span));
constant    intABuf2Tx_c                : memoryMapping_t := (intABuf1Rx_c.base + intABuf1Rx_c.span, align32(extABuf2Tx_c.span));
constant    intABuf2Rx_c                : memoryMapping_t := (intABuf2Tx_c.base + intABuf2Tx_c.span, align32(extABuf2Rx_c.span));
----pdo buffers (triple buffers considered!)
constant    intTpdoBuf_c                : memoryMapping_t := (intABuf2Rx_c.base + intABuf2Rx_c.span, align32(extTpdoBuf_c.span) *3);
constant    intRpdo0Buf_c                : memoryMapping_t := (intTpdoBuf_c.base  + intTpdoBuf_c.span,  align32(extRpdo0Buf_c.span)*3);
constant    intRpdo1Buf_c                : memoryMapping_t := (intRpdo0Buf_c.base + intRpdo0Buf_c.span, align32(extRpdo1Buf_c.span)*3);
constant    intRpdo2Buf_c                : memoryMapping_t := (intRpdo1Buf_c.base + intRpdo1Buf_c.span, align32(extRpdo2Buf_c.span)*3);
----obtain dpr size of different configurations
constant    dprSize_c                    : integer := (    intCntStReg_c.span +
                                                        intABuf1Tx_c.span +
                                                        intABuf1Rx_c.span +
                                                        intABuf2Tx_c.span +
                                                        intABuf2Rx_c.span +
                                                        intTpdoBuf_c.span  +
                                                        intRpdo0Buf_c.span +
                                                        intRpdo1Buf_c.span +
                                                        intRpdo2Buf_c.span );
constant    dprAddrWidth_c                : integer := integer(ceil(log2(real(dprSize_c))));
---other constants
constant    magicNumber_c                : integer := 16#50435000#;
constant    pdiRev_c                    : integer := iPdiRev_g;
constant    pcpSysId_c                  : integer := pcpSysId;

------------------------------------------------------------------------------------------------------------------------
--signals
---dpr
type dprSig_t is
    record
            addr                        : std_logic_vector(dprAddrWidth_c-2-1 downto 0); --double word address!
            addrOff                        : std_logic_vector(dprAddrWidth_c-2 downto 0); --double word address!
            be                            : std_logic_vector(3 downto 0);
            din                            : std_logic_vector(31 downto 0);
            wr                            : std_logic;
    end record;
type dprPdi_t is
    record
            pcp                            : dprSig_t;
            ap                            : dprSig_t;
    end record;
----signals to the DPR
signal        dpr                            : dprPdi_t;
signal        dprOut                        : pdi32Bit_t;
----control / status register
signal        dprCntStReg_s                : dprPdi_t;
----asynchronous buffers
signal        dprABuf1Tx_s                : dprPdi_t;
signal        dprABuf1Rx_s                : dprPdi_t;
signal        dprABuf2Tx_s                : dprPdi_t;
signal        dprABuf2Rx_s                : dprPdi_t;
----pdo buffers (triple buffers considered!)
signal        dprTpdoBuf_s                : dprPdi_t := (((others => '0'), (others => '0'), (others => '0'), (others => '0'), '0'),
                                                        ((others => '0'), (others => '0'), (others => '0'), (others => '0'), '0'));
signal        dprRpdo0Buf_s                : dprPdi_t := (((others => '0'), (others => '0'), (others => '0'), (others => '0'), '0'),
                                                        ((others => '0'), (others => '0'), (others => '0'), (others => '0'), '0'));
signal        dprRpdo1Buf_s                : dprPdi_t := (((others => '0'), (others => '0'), (others => '0'), (others => '0'), '0'),
                                                        ((others => '0'), (others => '0'), (others => '0'), (others => '0'), '0'));
signal        dprRpdo2Buf_s                : dprPdi_t := (((others => '0'), (others => '0'), (others => '0'), (others => '0'), '0'),
                                                        ((others => '0'), (others => '0'), (others => '0'), (others => '0'), '0'));
---chip select
----control / status register
signal        selCntStReg_s                : pdiSel_t;
----asynchronous buffers
signal        selABuf1Tx_s                : pdiSel_t;
signal        selABuf1Rx_s                : pdiSel_t;
signal        selABuf2Tx_s                : pdiSel_t;
signal        selABuf2Rx_s                : pdiSel_t;
----pdo buffers (triple buffers considered!)
signal        selTpdoBuf_s                : pdiSel_t;
signal        selRpdo0Buf_s                : pdiSel_t;
signal        selRpdo1Buf_s                : pdiSel_t;
signal        selRpdo2Buf_s                : pdiSel_t;
---data output
----control / status register
signal        outCntStReg_s                : pdi32Bit_t;
----asynchronous buffers
signal        outABuf1Tx_s                : pdi32Bit_t;
signal        outABuf1Rx_s                : pdi32Bit_t;
signal        outABuf2Tx_s                : pdi32Bit_t;
signal        outABuf2Rx_s                : pdi32Bit_t;
----pdo buffers (triple buffers considered!)
signal        outTpdoBuf_s                : pdi32Bit_t := ((others => '0'), (others => '0'));
signal        outRpdo0Buf_s                : pdi32Bit_t := ((others => '0'), (others => '0'));
signal        outRpdo1Buf_s                : pdi32Bit_t := ((others => '0'), (others => '0'));
signal        outRpdo2Buf_s                : pdi32Bit_t := ((others => '0'), (others => '0'));
---virtual buffer control/state
signal        vBufTriggerPdo_s            : pdiTrig_t; --tpdo, rpdo2, rpdo1, rpdo0
signal        vBufSel_s                    : pdi32Bit_t := ((others => '1'), (others => '1')); --TXPDO_ACK | RXPDO2_ACK | RXPDO1_ACK | RXPDO0_ACK
---ap irq generation
signal        apIrqValue                    : std_logic_vector(31 downto 0);
signal        apIrqControlPcp,
            apIrqControlPcp2,
            apIrqControlApOut,
            apIrqControlApIn            : std_logic_vector(15 downto 0);
signal        ap_irq_s                    : std_logic;
---address calulation result
signal        pcp_addrRes                    : std_logic_vector(dprAddrWidth_c-2 downto 0);
signal        ap_addrRes                    : std_logic_vector(dprAddrWidth_c-2 downto 0);

---EVENT stuff
signal        pcp_eventSet_s, --pulse to set event
            pcp_eventRead                : std_logic_vector(15 downto 0);
signal        ap_eventAck_p, --pulse to ack event
            ap_eventAck                    : std_logic_vector(15 downto 0);
signal        asyncIrqCtrlOut_s,
            asyncIrqCtrlIn_s            : std_logic_vector(15 downto 0);
signal        ap_asyncIrq_s                : std_logic; --Async Irq to the Ap
signal        phyLink_s,
            phyLinkEvent                : std_logic_vector(phyLink'range);

--LED stuff
signal        pcp_ledForce_s,
            pcp_ledSet_s                : std_logic_vector(15 downto 0) := (others => '0');
signal        ap_ledForce_s,
            ap_ledSet_s                    : std_logic_vector(15 downto 0) := (others => '0');
signal        hw_ledForce_s,
            hw_ledSet_s                    : std_logic_vector(15 downto 0) := (others => '0');

--TIME SYNCHRONIZATION
signal        pcp_timeSyncDBufSel            : std_logic;
signal        ap_timeSyncDBufSel            : std_logic;
begin

    ASSERT NOT(iRpdos_g < 1 or iRpdos_g > 3)
        REPORT "Only 1, 2 or 3 Rpdos are supported!"
        severity failure;

    ASSERT NOT(iTpdos_g /= 1)
        REPORT "Only 1 Tpdo is supported!"
            severity failure;

------------------------------------------------------------------------------------------------------------------------
-- merge data to pcp/ap
    theMerger : block
    begin
        pcp_readdata    <=    outCntStReg_s.pcp    when     selCntStReg_s.pcp = '1' else
                            outABuf1Tx_s.pcp    when    selABuf1Tx_s.pcp  = '1' else
                            outABuf1Rx_s.pcp    when    selABuf1Rx_s.pcp  = '1' else
                            outABuf2Tx_s.pcp    when    selABuf2Tx_s.pcp  = '1' else
                            outABuf2Rx_s.pcp    when    selABuf2Rx_s.pcp  = '1' else
                            outTpdoBuf_s.pcp    when    selTpdoBuf_s.pcp  = '1' else
                            outRpdo0Buf_s.pcp    when    selRpdo0Buf_s.pcp = '1' else
                            outRpdo1Buf_s.pcp    when    selRpdo1Buf_s.pcp = '1'    else
                            outRpdo2Buf_s.pcp    when    selRpdo2Buf_s.pcp = '1' else
                            (others => '0');

        ap_readdata    <=        outCntStReg_s.ap    when     selCntStReg_s.ap = '1' else
                            outABuf1Tx_s.ap     when    selABuf1Tx_s.ap  = '1' else
                            outABuf1Rx_s.ap     when    selABuf1Rx_s.ap  = '1' else
                            outABuf2Tx_s.ap     when    selABuf2Tx_s.ap  = '1' else
                            outABuf2Rx_s.ap     when    selABuf2Rx_s.ap  = '1' else
                            outTpdoBuf_s.ap        when    selTpdoBuf_s.ap  = '1' else
                            outRpdo0Buf_s.ap    when    selRpdo0Buf_s.ap = '1' else
                            outRpdo1Buf_s.ap    when    selRpdo1Buf_s.ap = '1' else
                            outRpdo2Buf_s.ap    when    selRpdo2Buf_s.ap = '1' else
                            (others => '0');
    end block;
--
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- dual ported RAM
    theDpr : entity work.pdi_dpr
        generic map (
        NUM_WORDS        =>        (dprSize_c/4),
        LOG2_NUM_WORDS    =>        dprAddrWidth_c-2
        )
        port map (
        address_a        =>        pcp_addrRes(dprAddrWidth_c-2-1 downto 0),
        address_b        =>        ap_addrRes(dprAddrWidth_c-2-1 downto 0),
        byteena_a        =>        dpr.pcp.be,
        byteena_b        =>        dpr.ap.be,
        clock_a            =>        pcp_clk,
        clock_b            =>        ap_clk,
        data_a            =>        dpr.pcp.din,
        data_b            =>        dpr.ap.din,
        wren_a            =>        dpr.pcp.wr,
        wren_b            =>        dpr.ap.wr,
        q_a                =>        dprOut.pcp,
        q_b                =>        dprOut.ap
        );

    pcp_addrRes <= EXT('0' & pcp_address(extLog2MaxOneSpan-2-1 downto 0) + dpr.pcp.addrOff, pcp_addrRes'length);

    dpr.pcp    <=    dprCntStReg_s.pcp    when    selCntStReg_s.pcp = '1'    else
                dprABuf1Tx_s.pcp    when    selABuf1Tx_s.pcp  = '1' else
                dprABuf1Rx_s.pcp    when    selABuf1Rx_s.pcp  = '1' else
                dprABuf2Tx_s.pcp    when    selABuf2Tx_s.pcp  = '1' else
                dprABuf2Rx_s.pcp    when    selABuf2Rx_s.pcp  = '1' else
                dprTpdoBuf_s.pcp    when    selTpdoBuf_s.pcp = '1'    else
                dprRpdo0Buf_s.pcp    when    selRpdo0Buf_s.pcp = '1' and iRpdos_g >= 1 else
                dprRpdo1Buf_s.pcp    when    selRpdo1Buf_s.pcp = '1' and iRpdos_g >= 2 else
                dprRpdo2Buf_s.pcp    when    selRpdo2Buf_s.pcp = '1' and iRpdos_g >= 3 else
                ((others => '0'), (others => '0'), (others => '0'), (others => '0'), '0');



    ap_addrRes <= EXT('0' & ap_address(extLog2MaxOneSpan-2-1 downto 0) + dpr.ap.addrOff, ap_addrRes'length);

    dpr.ap    <=    dprCntStReg_s.ap    when    selCntStReg_s.ap = '1'    else
                dprABuf1Tx_s.ap     when    selABuf1Tx_s.ap   = '1' else
                dprABuf1Rx_s.ap     when    selABuf1Rx_s.ap   = '1' else
                dprABuf2Tx_s.ap     when    selABuf2Tx_s.ap   = '1' else
                dprABuf2Rx_s.ap     when    selABuf2Rx_s.ap   = '1' else
                dprTpdoBuf_s.ap        when    selTpdoBuf_s.ap = '1'    else
                dprRpdo0Buf_s.ap    when    selRpdo0Buf_s.ap = '1'     and iRpdos_g >= 1 else
                dprRpdo1Buf_s.ap    when    selRpdo1Buf_s.ap = '1'     and iRpdos_g >= 2 else
                dprRpdo2Buf_s.ap    when    selRpdo2Buf_s.ap = '1'     and iRpdos_g >= 3 else
                ((others => '0'), (others => '0'), (others => '0'), (others => '0'), '0');

------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- address decoder to generate select signals for different memory ranges
    theAddressDecoder : block
    begin
        --pcp side
        ---control / status register
        selCntStReg_s.pcp    <=    pcp_chipselect        when    (conv_integer(pcp_address)*4 >= extCntStReg_c.base and
                                                            (conv_integer(pcp_address)*4 < extCntStReg_c.base + extCntStReg_c.span))
                                                    else    '0';
        ---asynchronous buffers
        selABuf1Tx_s.pcp    <=    pcp_chipselect        when    (conv_integer(pcp_address)*4 >= extABuf1Tx_c.base and
                                                             (conv_integer(pcp_address)*4 < extABuf1Tx_c.base + extABuf1Tx_c.span))
                                                    else    '0';
        selABuf1Rx_s.pcp    <=    pcp_chipselect        when    (conv_integer(pcp_address)*4 >= extABuf1Rx_c.base and
                                                             (conv_integer(pcp_address)*4 < extABuf1Rx_c.base + extABuf1Rx_c.span))
                                                    else    '0';
        selABuf2Tx_s.pcp    <=    pcp_chipselect        when    (conv_integer(pcp_address)*4 >= extABuf2Tx_c.base and
                                                             (conv_integer(pcp_address)*4 < extABuf2Tx_c.base + extABuf2Tx_c.span))
                                                    else    '0';
        selABuf2Rx_s.pcp    <=    pcp_chipselect        when    (conv_integer(pcp_address)*4 >= extABuf2Rx_c.base and
                                                             (conv_integer(pcp_address)*4 < extABuf2Rx_c.base + extABuf2Rx_c.span))
                                                    else    '0';

        ---pdo buffers (triple buffers considered!)
        selTpdoBuf_s.pcp    <=    pcp_chipselect        when    (conv_integer(pcp_address)*4 >= extTpdoBuf_c.base and
                                                             (conv_integer(pcp_address)*4 < extTpdoBuf_c.base + extTpdoBuf_c.span))
                                                    else    '0';
        selRpdo0Buf_s.pcp    <=    pcp_chipselect        when    (conv_integer(pcp_address)*4 >= extRpdo0Buf_c.base and
                                                             (conv_integer(pcp_address)*4 < extRpdo0Buf_c.base + extRpdo0Buf_c.span))
                                                    else    '0';
        selRpdo1Buf_s.pcp    <=    pcp_chipselect        when    (conv_integer(pcp_address)*4 >= extRpdo1Buf_c.base and
                                                             (conv_integer(pcp_address)*4 < extRpdo1Buf_c.base + extRpdo1Buf_c.span))
                                                    else    '0';
        selRpdo2Buf_s.pcp    <=    pcp_chipselect        when    (conv_integer(pcp_address)*4 >= extRpdo2Buf_c.base and
                                                             (conv_integer(pcp_address)*4 < extRpdo2Buf_c.base + extRpdo2Buf_c.span))
                                                    else    '0';

        --ap side
        ---control / status register
        selCntStReg_s.ap    <=    ap_chipselect    when    (conv_integer(ap_address)*4 >= extCntStReg_c.base and
                                                         (conv_integer(ap_address)*4 < extCntStReg_c.base + extCntStReg_c.span))
                                                else    '0';
        ---asynchronous buffers
        selABuf1Tx_s.ap     <=    ap_chipselect    when    (conv_integer(ap_address)*4 >= extABuf1Tx_c.base and
                                                         (conv_integer(ap_address)*4 < extABuf1Tx_c.base + extABuf1Tx_c.span))
                                                else    '0';
        selABuf1Rx_s.ap     <=    ap_chipselect    when    (conv_integer(ap_address)*4 >= extABuf1Rx_c.base and
                                                         (conv_integer(ap_address)*4 < extABuf1Rx_c.base + extABuf1Rx_c.span))
                                                else    '0';
        selABuf2Tx_s.ap     <=    ap_chipselect    when    (conv_integer(ap_address)*4 >= extABuf2Tx_c.base and
                                                         (conv_integer(ap_address)*4 < extABuf2Tx_c.base + extABuf2Tx_c.span))
                                                else    '0';
        selABuf2Rx_s.ap     <=    ap_chipselect    when    (conv_integer(ap_address)*4 >= extABuf2Rx_c.base and
                                                         (conv_integer(ap_address)*4 < extABuf2Rx_c.base + extABuf2Rx_c.span))
                                                else    '0';
        ---pdo buffers (triple buffers considered!)
        selTpdoBuf_s.ap        <=    ap_chipselect    when    (conv_integer(ap_address)*4 >= extTpdoBuf_c.base and
                                                         (conv_integer(ap_address)*4 < extTpdoBuf_c.base + extTpdoBuf_c.span))
                                                else    '0';
        selRpdo0Buf_s.ap    <=    ap_chipselect    when    (conv_integer(ap_address)*4 >= extRpdo0Buf_c.base and
                                                         (conv_integer(ap_address)*4 < extRpdo0Buf_c.base + extRpdo0Buf_c.span))
                                                else    '0';
        selRpdo1Buf_s.ap    <=    ap_chipselect    when    (conv_integer(ap_address)*4 >= extRpdo1Buf_c.base and
                                                         (conv_integer(ap_address)*4 < extRpdo1Buf_c.base + extRpdo1Buf_c.span))
                                                else    '0';
        selRpdo2Buf_s.ap    <=    ap_chipselect    when    (conv_integer(ap_address)*4 >= extRpdo2Buf_c.base and
                                                         (conv_integer(ap_address)*4 < extRpdo2Buf_c.base + extRpdo2Buf_c.span))
                                                else    '0';
    end block theAddressDecoder;
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- control / status register
    theCntrlStatReg4Pcp : entity work.pdiControlStatusReg
    generic map (
            bIsPcp                        => true,
            iAddrWidth_g                => extLog2MaxOneSpan-2,
            iBaseDpr_g                    => 16#8#/4, --base address of content to be mapped to dpr
            iSpanDpr_g                    => intCntStReg_c.span/4, --size of content to be mapped to dpr
            iBaseMap2_g                    => intCntStReg_c.base/4, --base address in dpr
            iDprAddrWidth_g                => dprCntStReg_s.pcp.addr'length,
            iRpdos_g                    => iRpdos_g,
            genLedGadget_g                => genLedGadget_g,
            genTimeSync_g                => genTimeSync_g,
            genEvent_g                    => genEvent_g,
            --register content
            ---constant values
            magicNumber                    => conv_std_logic_vector(magicNumber_c, 32),
            pdiRev                        => conv_std_logic_vector(pdiRev_c, 16),
            pcpSysId                    => conv_std_logic_vector(pcpSysId_c, 16),
            tPdoBuffer                    => conv_std_logic_vector(extTpdoBuf_c.base, 16) &
                                            conv_std_logic_vector(extTpdoBuf_c.span, 16),
            rPdo0Buffer                    => conv_std_logic_vector(extRpdo0Buf_c.base, 16) &
                                            conv_std_logic_vector(extRpdo0Buf_c.span, 16),
            rPdo1Buffer                    => conv_std_logic_vector(extRpdo1Buf_c.base, 16) &
                                            conv_std_logic_vector(extRpdo1Buf_c.span, 16),
            rPdo2Buffer                    => conv_std_logic_vector(extRpdo2Buf_c.base, 16) &
                                            conv_std_logic_vector(extRpdo2Buf_c.span, 16),
            asyncBuffer1Tx                => conv_std_logic_vector(extABuf1Tx_c.base, 16) &
                                            conv_std_logic_vector(extABuf1Tx_c.span, 16),
            asyncBuffer1Rx                => conv_std_logic_vector(extABuf1Rx_c.base, 16) &
                                            conv_std_logic_vector(extABuf1Rx_c.span, 16),
            asyncBuffer2Tx                => conv_std_logic_vector(extABuf2Tx_c.base, 16) &
                                            conv_std_logic_vector(extABuf2Tx_c.span, 16),
            asyncBuffer2Rx                => conv_std_logic_vector(extABuf2Rx_c.base, 16) &
                                            conv_std_logic_vector(extABuf2Rx_c.span, 16)
    )

    port map (
            --memory mapped interface
            clk                            => pcp_clk,
            rst                            => pcp_reset,
            sel                            => selCntStReg_s.pcp,
            wr                            => pcp_write,
            rd                            => pcp_read,
            addr                        => pcp_address(extLog2MaxOneSpan-1-2 downto 0),
            be                            => pcp_byteenable,
            din                            => pcp_writedata,
            dout                        => outCntStReg_s.pcp,
            --register content
            ---virtual buffer control signals
            pdoVirtualBufferSel            => vBufSel_s.pcp,
            tPdoTrigger                    => vBufTriggerPdo_s.pcp(3),
            rPdoTrigger                    => vBufTriggerPdo_s.pcp(2 downto 0),
            ---event registers
            eventAckIn                     => pcp_eventRead,
            eventAckOut                    => pcp_eventSet_s,
            ---async irq (by event)
            asyncIrqCtrlIn                => (others => '0'), --not for pcp
            asyncIrqCtrlOut                => open, --not for pcp
            ---led stuff
            ledCnfgIn                     => pcp_ledForce_s,
            ledCnfgOut                     => pcp_ledForce_s,
            ledCtrlIn                     => pcp_ledSet_s,
            ledCtrlOut                     => pcp_ledSet_s,
            ---time synchronization
            doubleBufSel_out            => open, --PCP is the sink
            doubleBufSel_in                => pcp_timeSyncDBufSel,
            timeSyncIrq                    => '0', --pcp is not interested
            --dpr interface (from PCP/AP to DPR)
            dprAddrOff                    => dprCntStReg_s.pcp.addrOff,
            dprDin                        => dprCntStReg_s.pcp.din,
            dprDout                        => dprOut.pcp,
            dprBe                        => dprCntStReg_s.pcp.be,
            dprWr                        => dprCntStReg_s.pcp.wr,
            --ap irq generation
            apIrqControlOut                => apIrqControlPcp,
            --SW is blind, thus, use the transferred enable signal from AP!
            apIrqControlIn                => apIrqControlPcp2,
            --hw acc triggering
            rpdo_change_tog                => rpdo_change_tog,
            tpdo_change_tog                => tpdo_change_tog
    );

    --only read 15 bits of the written, the msbit is read from transferred AP bit
    apIrqControlPcp2(14 downto 0) <= apIrqControlPcp(14 downto 0);

    --transfer the AP's enable signal to PCP, since SW is blind... :)
    syncApEnable2Pcp : entity work.sync
        generic map (
            doSync_g => not genOnePdiClkDomain_g
        )
        port map (
            din => apIrqControlApOut(15),
            dout => apIrqControlPcp2(15),
            clk => pcp_clk,
            rst => pcp_reset
        );

    --sync double buffer select for time sync to AP if the feature is enabled
    -- note: signal toggles on PCP side when NETTIME [seconds] is written
    syncDBuf_TimeSync : entity work.sync
        generic map (
            doSync_g => not genOnePdiClkDomain_g
        )
        port map (
            dout => pcp_timeSyncDBufSel,
            din => ap_timeSyncDBufSel,
            clk => pcp_clk,
            rst => pcp_reset
        );

    theCntrlStatReg4Ap : entity work.pdiControlStatusReg
    generic map (
            bIsPcp                        => false,
            iAddrWidth_g                => extLog2MaxOneSpan-2,
            iBaseDpr_g                    => 16#8#/4, --base address of content to be mapped to dpr
            iSpanDpr_g                    => intCntStReg_c.span/4, --size of content to be mapped to dpr
            iBaseMap2_g                    => intCntStReg_c.base/4, --base address in dpr
            iDprAddrWidth_g                => dprCntStReg_s.ap.addr'length,
            iRpdos_g                    => iRpdos_g,
            genLedGadget_g                => genLedGadget_g,
            genTimeSync_g                => genTimeSync_g,
            genEvent_g                    => genEvent_g,
            --register content
            ---constant values
            magicNumber                    => conv_std_logic_vector(magicNumber_c, 32),
            pdiRev                        => conv_std_logic_vector(pdiRev_c, 16),
            pcpSysId                    => conv_std_logic_vector(pcpSysId_c, 16),
            tPdoBuffer                    => conv_std_logic_vector(extTpdoBuf_c.base, 16) &
                                            conv_std_logic_vector(extTpdoBuf_c.span, 16),
            rPdo0Buffer                    => conv_std_logic_vector(extRpdo0Buf_c.base, 16) &
                                            conv_std_logic_vector(extRpdo0Buf_c.span, 16),
            rPdo1Buffer                    => conv_std_logic_vector(extRpdo1Buf_c.base, 16) &
                                            conv_std_logic_vector(extRpdo1Buf_c.span, 16),
            rPdo2Buffer                    => conv_std_logic_vector(extRpdo2Buf_c.base, 16) &
                                            conv_std_logic_vector(extRpdo2Buf_c.span, 16),
            asyncBuffer1Tx                => conv_std_logic_vector(extABuf1Tx_c.base, 16) &
                                            conv_std_logic_vector(extABuf1Tx_c.span, 16),
            asyncBuffer1Rx                => conv_std_logic_vector(extABuf1Rx_c.base, 16) &
                                            conv_std_logic_vector(extABuf1Rx_c.span, 16),
            asyncBuffer2Tx                => conv_std_logic_vector(extABuf2Tx_c.base, 16) &
                                            conv_std_logic_vector(extABuf2Tx_c.span, 16),
            asyncBuffer2Rx                => conv_std_logic_vector(extABuf2Rx_c.base, 16) &
                                            conv_std_logic_vector(extABuf2Rx_c.span, 16)
    )

    port map (
            --memory mapped interface
            clk                            => ap_clk,
            rst                            => ap_reset,
            sel                            => selCntStReg_s.ap,
            wr                            => ap_write,
            rd                            => ap_read,
            addr                        => ap_address(extLog2MaxOneSpan-1-2 downto 0),
            be                            => ap_byteenable,
            din                            => ap_writedata,
            dout                        => outCntStReg_s.ap,
            --register content
            ---virtual buffer control signals
            pdoVirtualBufferSel            => vBufSel_s.ap,
            tPdoTrigger                    => vBufTriggerPdo_s.ap(3),
            rPdoTrigger                    => vBufTriggerPdo_s.ap(2 downto 0),
            ---event registers
            eventAckIn                     => ap_eventAck,
            eventAckOut                    => ap_eventAck_p,
            ---async irq (by event)
            asyncIrqCtrlIn                => asyncIrqCtrlIn_s,
            asyncIrqCtrlOut                => asyncIrqCtrlOut_s,
            ---led stuff
            ledCnfgIn                     => ap_ledForce_s,
            ledCnfgOut                     => ap_ledForce_s,
            ledCtrlIn                     => ap_ledSet_s,
            ledCtrlOut                     => ap_ledSet_s,
            ---time synchronization
            doubleBufSel_out            => ap_timeSyncDBufSel,
            doubleBufSel_in                => '0', --AP is the source
            timeSyncIrq                    => ap_irq_s,
            --dpr interface (from PCP/AP to DPR)
            dprAddrOff                    => dprCntStReg_s.ap.addrOff,
            dprDin                        => dprCntStReg_s.ap.din,
            dprDout                        => dprOut.ap,
            dprBe                        => dprCntStReg_s.ap.be,
            dprWr                        => dprCntStReg_s.ap.wr,
            --ap irq generation
            --apIrqValue                    =>
            apIrqControlOut                => apIrqControlApOut,
            apIrqControlIn                => apIrqControlApIn,
            rpdo_change_tog                => (others => '0'),
            tpdo_change_tog                => '0'
    );

    theApIrqGenerator : entity work.apIrqGen
        generic map (
            genOnePdiClkDomain_g => genOnePdiClkDomain_g
        )
        port map (
            --CLOCK DOMAIN PCP
            clkA => pcp_clk,
            rstA => pcp_reset,
            irqA => pcp_irq,
            --preValA => apIrqValue,
            enableA => apIrqControlPcp(7),
            modeA => apIrqControlPcp(6),
            setA => apIrqControlPcp(0),
            --CLOCK DOMAIN AP
            clkB => ap_clk,
            rstB => ap_reset,
            ackB => apIrqControlApOut(0),
            irqB => ap_irq_s
        );

    --irq enabled by apIrqControlApOut(15)
    ap_irq <= ap_irq_s and apIrqControlApOut(15);
    apIrqControlApIn <= apIrqControlApOut(15) & "000" & x"00" & "000" & ap_irq_s;

    --the LED stuff
    genLedGadget : if genLedGadget_g generate
        --first set the hw leds
        hw_ledForce_s <= x"00" & "00111100"; --phy1 and 0 act and link
        hw_ledSet_s <= x"00" & "00" & (phyAct(1) and phyLink(1)) & phyLink(1) & (phyAct(0) and phyLink(0)) & phyLink(0) & "00";

        theLedGadget : entity work.pdiLed
        generic map (
            iLedWidth_g                        => ledsOut'length
        )
        port map (
            --src A (lowest priority)
            srcAled                            => hw_ledSet_s(ledsOut'range),
            srcAforce                        => hw_ledForce_s(ledsOut'range),
            --src B
            srcBled                            => pcp_ledSet_s(ledsOut'range),
            srcBforce                        => pcp_ledForce_s(ledsOut'range),
            --src C (highest priority)
            srcCled                            => ap_ledSet_s(ledsOut'range),
            srcCforce                        => ap_ledForce_s(ledsOut'range),
            --led output
            ledOut                            => ledsOut
        );
    end generate;

    genEventComp : if genEvent_g generate
    begin
        theEventBlock : block
            --set here the number of events
            constant iSwEvent_c : integer := 1;
            constant iHwEvent_c : integer := 2;

            signal eventSetA    : std_logic_vector(iSwEvent_c-1 downto 0);
            signal eventReadA    : std_logic_vector(iSwEvent_c+iHwEvent_c-1 downto 0);

            signal eventAckB    : std_logic_vector(iSwEvent_c+iHwEvent_c-1 downto 0);
            signal eventReadB    : std_logic_vector(iSwEvent_c+iHwEvent_c-1 downto 0);
        begin

            --event mapping: 15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
            -- in register    x  x  x  x  x  x  x  x hw hw  x  x  x  x  x sw
            -- in pdiEvent                                          hw hw sw
            eventSetA <= pcp_eventSet_s(0 downto 0); --pcp sets sw event (I know, its called generic event, bla...)
            pcp_eventRead <= x"00" & eventReadA(iSwEvent_c+iHwEvent_c-1 downto iSwEvent_c) &
                            "00000" & eventReadA(iSwEvent_c-1 downto 0);

            eventAckB <= ap_eventAck_p(7 downto 6) & ap_eventAck_p(0); --ap acks events
            ap_eventAck <= x"00" & eventReadB(iSwEvent_c+iHwEvent_c-1 downto iSwEvent_c) &
                            "00000" & eventReadB(iSwEvent_c-1 downto 0);

            theEventStuff : entity work.pdiEvent
            --16 bit
            -- sw is at bit 0
            -- hw is at bit 6 and 7
            generic map (
                    genOnePdiClkDomain_g => genOnePdiClkDomain_g,
                    iSwEvent_g => 1,
                    iHwEvent_g => 2
            )

            port map (
                    --port A -> PCP
                    clkA                        => pcp_clk,
                    rstA                        => pcp_reset,
                    eventSetA                    => eventSetA,
                    eventReadA                    => eventReadA,
                    --port B -> AP
                    clkB                        => ap_clk,
                    rstB                        => ap_reset,
                    eventAckB                    => eventAckB,
                    eventReadB                    => eventReadB,
                    --hw event set pulse (must be synchronous to clkB!)
                    hwEventSetPulseB            => phyLinkEvent
            );

            --generate async interrupt
            asyncIrq : process(ap_eventAck)
            variable tmp : std_logic;
            begin
                tmp := '0';
                for i in ap_eventAck'range loop
                    tmp := tmp or ap_eventAck(i);
                end loop;
                ap_asyncIrq_s <= tmp;
            end process;
            --IRQ is asserted if enabled by AP
            ap_asyncIrq <= ap_asyncIrq_s and asyncIrqCtrlOut_s(15);

            asyncIrqCtrlIn_s(15) <= asyncIrqCtrlOut_s(15);
            asyncIrqCtrlIn_s(14 downto 1) <= (others => '0'); --ignoring the rest
            asyncIrqCtrlIn_s(0) <= ap_asyncIrq_s; --AP may poll IRQ level

            syncPhyLinkGen : for i in phyLink'range generate
                syncPhyLink : entity work.sync
                    generic map (
                        doSync_g => not genOnePdiClkDomain_g
                    )
                    port map (
                        din => phyLink(i),
                        dout => phyLink_s(i),
                        clk => ap_clk,
                        rst => ap_reset
                    );

                detPhyLinkEdge : entity work.edgeDet
                    port map (
                        din => phyLink_s(i),
                        rising => open,
                        falling => phyLinkEvent(i), --if phy link deasserts - EVENT!!!
                        any => open,
                        clk => ap_clk,
                        rst => ap_reset
                    );
            end generate;
        end block;
    end generate;
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- asynchronous Buffer 1 Tx
    genABuf1Tx : if genABuf1_g generate
        theAsyncBuf1Tx4Pcp : entity work.pdiSimpleReg
        generic map (
                iAddrWidth_g                => extLog2MaxOneSpan-2,
                iBaseMap2_g                    => intABuf1Tx_c.base/4,
                iDprAddrWidth_g                => dprABuf1Tx_s.pcp.addr'length
        )

        port map (
                --memory mapped interface
                sel                            => selABuf1Tx_s.pcp,
                wr                            => pcp_write,
                rd                            => pcp_read,
                addr                        => pcp_address(extLog2MaxOneSpan-1-2 downto 0),
                be                            => pcp_byteenable,
                din                            => pcp_writedata,
                dout                        => outABuf1Tx_s.pcp,
                --dpr interface (from PCP/AP to DPR)
                dprAddrOff                    => dprABuf1Tx_s.pcp.addrOff,
                dprDin                        => dprABuf1Tx_s.pcp.din,
                dprDout                        => dprOut.pcp,
                dprBe                        => dprABuf1Tx_s.pcp.be,
                dprWr                        => dprABuf1Tx_s.pcp.wr
        );

        theAsyncBuf1Tx4Ap : entity work.pdiSimpleReg
        generic map (
                iAddrWidth_g                => extLog2MaxOneSpan-2,
                iBaseMap2_g                    => intABuf1Tx_c.base/4,
                iDprAddrWidth_g                => dprABuf1Tx_s.ap.addr'length
        )

        port map (
                --memory mapped interface
                sel                            => selABuf1Tx_s.ap,
                wr                            => ap_write,
                rd                            => ap_read,
                addr                        => ap_address(extLog2MaxOneSpan-1-2 downto 0),
                be                            => ap_byteenable,
                din                            => ap_writedata,
                dout                        => outABuf1Tx_s.ap,
                --dpr interface (from PCP/AP to DPR)
                dprAddrOff                    => dprABuf1Tx_s.ap.addrOff,
                dprDin                        => dprABuf1Tx_s.ap.din,
                dprDout                        => dprOut.ap,
                dprBe                        => dprABuf1Tx_s.ap.be,
                dprWr                        => dprABuf1Tx_s.ap.wr
        );
    end generate;
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- asynchronous Buffer 1 Rx
    genABuf1Rx : if genABuf1_g generate
        theAsyncBuf1Rx4Pcp : entity work.pdiSimpleReg
        generic map (
                iAddrWidth_g                => extLog2MaxOneSpan-2,
                iBaseMap2_g                    => intABuf1Rx_c.base/4,
                iDprAddrWidth_g                => dprABuf1Rx_s.pcp.addr'length
        )

        port map (
                --memory mapped interface
                sel                            => selABuf1Rx_s.pcp,
                wr                            => pcp_write,
                rd                            => pcp_read,
                addr                        => pcp_address(extLog2MaxOneSpan-1-2 downto 0),
                be                            => pcp_byteenable,
                din                            => pcp_writedata,
                dout                        => outABuf1Rx_s.pcp,
                --dpr interface (from PCP/AP to DPR)
                dprAddrOff                    => dprABuf1Rx_s.pcp.addrOff,
                dprDin                        => dprABuf1Rx_s.pcp.din,
                dprDout                        => dprOut.pcp,
                dprBe                        => dprABuf1Rx_s.pcp.be,
                dprWr                        => dprABuf1Rx_s.pcp.wr
        );

        theAsyncBuf1Rx4Ap : entity work.pdiSimpleReg
        generic map (
                iAddrWidth_g                => extLog2MaxOneSpan-2,
                iBaseMap2_g                    => intABuf1Rx_c.base/4,
                iDprAddrWidth_g                => dprABuf1Rx_s.ap.addr'length
        )

        port map (
                --memory mapped interface
                sel                            => selABuf1Rx_s.ap,
                wr                            => ap_write,
                rd                            => ap_read,
                addr                        => ap_address(extLog2MaxOneSpan-1-2 downto 0),
                be                            => ap_byteenable,
                din                            => ap_writedata,
                dout                        => outABuf1Rx_s.ap,
                --dpr interface (from PCP/AP to DPR)
                dprAddrOff                    => dprABuf1Rx_s.ap.addrOff,
                dprDin                        => dprABuf1Rx_s.ap.din,
                dprDout                        => dprOut.ap,
                dprBe                        => dprABuf1Rx_s.ap.be,
                dprWr                        => dprABuf1Rx_s.ap.wr
        );
    end generate;
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- asynchronous Buffer 2 Tx
    genABuf2Tx : if genABuf2_g generate
        theAsyncBuf2Tx4Pcp : entity work.pdiSimpleReg
        generic map (
                iAddrWidth_g                => extLog2MaxOneSpan-2,
                iBaseMap2_g                    => intABuf2Tx_c.base/4,
                iDprAddrWidth_g                => dprABuf2Tx_s.pcp.addr'length
        )

        port map (
                --memory mapped interface
                sel                            => selABuf2Tx_s.pcp,
                wr                            => pcp_write,
                rd                            => pcp_read,
                addr                        => pcp_address(extLog2MaxOneSpan-1-2 downto 0),
                be                            => pcp_byteenable,
                din                            => pcp_writedata,
                dout                        => outABuf2Tx_s.pcp,
                --dpr interface (from PCP/AP to DPR)
                dprAddrOff                    => dprABuf2Tx_s.pcp.addrOff,
                dprDin                        => dprABuf2Tx_s.pcp.din,
                dprDout                        => dprOut.pcp,
                dprBe                        => dprABuf2Tx_s.pcp.be,
                dprWr                        => dprABuf2Tx_s.pcp.wr
        );

        theAsyncBuf2Tx4Ap : entity work.pdiSimpleReg
        generic map (
                iAddrWidth_g                => extLog2MaxOneSpan-2,
                iBaseMap2_g                    => intABuf2Tx_c.base/4,
                iDprAddrWidth_g                => dprABuf2Tx_s.ap.addr'length
        )

        port map (
                --memory mapped interface
                sel                            => selABuf2Tx_s.ap,
                wr                            => ap_write,
                rd                            => ap_read,
                addr                        => ap_address(extLog2MaxOneSpan-1-2 downto 0),
                be                            => ap_byteenable,
                din                            => ap_writedata,
                dout                        => outABuf2Tx_s.ap,
                --dpr interface (from PCP/AP to DPR)
                dprAddrOff                    => dprABuf2Tx_s.ap.addrOff,
                dprDin                        => dprABuf2Tx_s.ap.din,
                dprDout                        => dprOut.ap,
                dprBe                        => dprABuf2Tx_s.ap.be,
                dprWr                        => dprABuf2Tx_s.ap.wr
        );
    end generate;
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- asynchronous Buffer 2 Rx
    genABuf2Rx : if genABuf2_g generate
        theAsyncBuf2Rx4Pcp : entity work.pdiSimpleReg
        generic map (
                iAddrWidth_g                => extLog2MaxOneSpan-2,
                iBaseMap2_g                    => intABuf2Rx_c.base/4,
                iDprAddrWidth_g                => dprABuf2Rx_s.pcp.addr'length
        )

        port map (
                --memory mapped interface
                sel                            => selABuf2Rx_s.pcp,
                wr                            => pcp_write,
                rd                            => pcp_read,
                addr                        => pcp_address(extLog2MaxOneSpan-1-2 downto 0),
                be                            => pcp_byteenable,
                din                            => pcp_writedata,
                dout                        => outABuf2Rx_s.pcp,
                --dpr interface (from PCP/AP to DPR)
                dprAddrOff                    => dprABuf2Rx_s.pcp.addrOff,
                dprDin                        => dprABuf2Rx_s.pcp.din,
                dprDout                        => dprOut.pcp,
                dprBe                        => dprABuf2Rx_s.pcp.be,
                dprWr                        => dprABuf2Rx_s.pcp.wr
        );

        theAsyncBuf2Rx4Ap : entity work.pdiSimpleReg
        generic map (
                iAddrWidth_g                => extLog2MaxOneSpan-2,
                iBaseMap2_g                    => intABuf2Rx_c.base/4,
                iDprAddrWidth_g                => dprABuf2Rx_s.ap.addr'length
        )

        port map (
                --memory mapped interface
                sel                            => selABuf2Rx_s.ap,
                wr                            => ap_write,
                rd                            => ap_read,
                addr                        => ap_address(extLog2MaxOneSpan-1-2 downto 0),
                be                            => ap_byteenable,
                din                            => ap_writedata,
                dout                        => outABuf2Rx_s.ap,
                --dpr interface (from PCP/AP to DPR)
                dprAddrOff                    => dprABuf2Rx_s.ap.addrOff,
                dprDin                        => dprABuf2Rx_s.ap.din,
                dprDout                        => dprOut.ap,
                dprBe                        => dprABuf2Rx_s.ap.be,
                dprWr                        => dprABuf2Rx_s.ap.wr
        );
    end generate;
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
--TPDO buffer
    theTpdoTrippleBuffer : block
    signal selVBufPcpOneHot                : std_logic_vector(2 downto 0);
    signal selVBufApOneHot                : std_logic_vector(2 downto 0);
    begin

        vBufSel_s.pcp(31 downto 24) <=     x"00" when selVBufPcpOneHot = "001" else
                                        x"11" when selVBufPcpOneHot = "010" else
                                        x"22" when selVBufPcpOneHot = "100" else
                                        x"FF";

        vBufSel_s.ap(31 downto 24) <=     x"00" when selVBufApOneHot = "001" else
                                        x"11" when selVBufApOneHot = "010" else
                                        x"22" when selVBufApOneHot = "100" else
                                        x"FF";

        dprTpdoBuf_s.pcp.din <= pcp_writedata;
        outTpdoBuf_s.pcp <= dprOut.pcp;
        dprTpdoBuf_s.pcp.be <= pcp_byteenable;
        dprTpdoBuf_s.pcp.wr <= pcp_write;

        dprTpdoBuf_s.ap.din <= ap_writedata;
        outTpdoBuf_s.ap <= dprOut.ap;
        dprTpdoBuf_s.ap.be <= ap_byteenable;
        dprTpdoBuf_s.ap.wr <= ap_write;

        theTrippleMechanism : entity work.tripleVBufLogic
        generic map (
            genOnePdiClkDomain_g         => genOnePdiClkDomain_g,
            --base address of virtual buffers in DPR
            iVirtualBufferBase_g        => intTpdoBuf_c.base/4, --double word!
            --size of one virtual buffer in DPR (must be aligned!!!)
            iVirtualBufferSize_g        => intTpdoBuf_c.span/3/4, --double word!
            --out address width
            iOutAddrWidth_g                => dprTpdoBuf_s.pcp.addr'length,
            --in address width
            iInAddrWidth_g                => extLog2MaxOneSpan-2,
            --ap is producer
            bApIsProducer                => true
        )

        port map (
            pcpClk                        => pcp_clk,
            pcpReset                    => pcp_reset,
            pcpTrigger                    => vBufTriggerPdo_s.pcp(3),
            pcpOutAddrOff                => dprTpdoBuf_s.pcp.addrOff,
            pcpOutSelVBuf                => selVBufPcpOneHot,
            apClk                        => ap_clk,
            apReset                        => ap_reset,
            apTrigger                    => vBufTriggerPdo_s.ap(3),
            apOutAddrOff                => dprTpdoBuf_s.ap.addrOff,
            apOutSelVBuf                => selVBufApOneHot
        );

    end block;
--
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
--RPDO0 buffer
    theRpdo0TrippleBuffer : block
    signal selVBufPcpOneHot                : std_logic_vector(2 downto 0);
    signal selVBufApOneHot                : std_logic_vector(2 downto 0);
    begin

        vBufSel_s.pcp(7 downto 0) <=     x"00" when selVBufPcpOneHot = "001" else
                                        x"11" when selVBufPcpOneHot = "010" else
                                        x"22" when selVBufPcpOneHot = "100" else
                                        x"FF";

        vBufSel_s.ap(7 downto 0) <=     x"00" when selVBufApOneHot = "001" else
                                        x"11" when selVBufApOneHot = "010" else
                                        x"22" when selVBufApOneHot = "100" else
                                        x"FF";

        dprRpdo0Buf_s.pcp.din <= pcp_writedata;
        outRpdo0Buf_s.pcp <= dprOut.pcp;
        dprRpdo0Buf_s.pcp.be <= pcp_byteenable;
        dprRpdo0Buf_s.pcp.wr <= pcp_write;

        dprRpdo0Buf_s.ap.din <= ap_writedata;
        outRpdo0Buf_s.ap <= dprOut.ap;
        dprRpdo0Buf_s.ap.be <= ap_byteenable;
        dprRpdo0Buf_s.ap.wr <= ap_write;

        theTrippleMechanism : entity work.tripleVBufLogic
        generic map (
            genOnePdiClkDomain_g         => genOnePdiClkDomain_g,
            --base address of virtual buffers in DPR
            iVirtualBufferBase_g        => intRpdo0Buf_c.base/4, --double word!
            --size of one virtual buffer in DPR (must be aligned!!!)
            iVirtualBufferSize_g        => intRpdo0Buf_c.span/3/4, --double word!
            --out address width
            iOutAddrWidth_g                => dprRpdo0Buf_s.pcp.addr'length,
            --in address width
            iInAddrWidth_g                => extLog2MaxOneSpan-2,
            --ap is NOT producer
            bApIsProducer                => false
        )

        port map (
            pcpClk                        => pcp_clk,
            pcpReset                    => pcp_reset,
            pcpTrigger                    => vBufTriggerPdo_s.pcp(0),
            pcpOutAddrOff                => dprRpdo0Buf_s.pcp.addrOff,
            pcpOutSelVBuf                => selVBufPcpOneHot,
            apClk                        => ap_clk,
            apReset                    => ap_reset,
            apTrigger                    => vBufTriggerPdo_s.ap(0),
            apOutAddrOff                => dprRpdo0Buf_s.ap.addrOff,
            apOutSelVBuf                => selVBufApOneHot
        );

    end block;
--
------------------------------------------------------------------------------------------------------------------------
genRpdo1 : if iRpdos_g >= 2 generate
------------------------------------------------------------------------------------------------------------------------
--RPDO1 buffer
    theRpdo1TrippleBuffer : block
    signal selVBufPcpOneHot                : std_logic_vector(2 downto 0);
    signal selVBufApOneHot                : std_logic_vector(2 downto 0);
    begin

        vBufSel_s.pcp(15 downto 8) <=     x"00" when selVBufPcpOneHot = "001" else
                                        x"11" when selVBufPcpOneHot = "010" else
                                        x"22" when selVBufPcpOneHot = "100" else
                                        x"FF";

        vBufSel_s.ap(15 downto 8) <=     x"00" when selVBufApOneHot = "001" else
                                        x"11" when selVBufApOneHot = "010" else
                                        x"22" when selVBufApOneHot = "100" else
                                        x"FF";

        dprRpdo1Buf_s.pcp.din <= pcp_writedata;
        outRpdo1Buf_s.pcp <= dprOut.pcp;
        dprRpdo1Buf_s.pcp.be <= pcp_byteenable;
        dprRpdo1Buf_s.pcp.wr <= pcp_write;

        dprRpdo1Buf_s.ap.din <= ap_writedata;
        outRpdo1Buf_s.ap <= dprOut.ap;
        dprRpdo1Buf_s.ap.be <= ap_byteenable;
        dprRpdo1Buf_s.ap.wr <= ap_write;

        theTrippleMechanism : entity work.tripleVBufLogic
        generic map (
            genOnePdiClkDomain_g         => genOnePdiClkDomain_g,
            --base address of virtual buffers in DPR
            iVirtualBufferBase_g        => intRpdo1Buf_c.base/4, --double word!
            --size of one virtual buffer in DPR (must be aligned!!!)
            iVirtualBufferSize_g        => intRpdo1Buf_c.span/3/4, --double word!
            --out address width
            iOutAddrWidth_g                => dprRpdo1Buf_s.pcp.addr'length,
            --in address width
            iInAddrWidth_g                => extLog2MaxOneSpan-2,
            --ap is NOT producer
            bApIsProducer                => false
        )

        port map (
            pcpClk                        => pcp_clk,
            pcpReset                    => pcp_reset,
            pcpTrigger                    => vBufTriggerPdo_s.pcp(1),
            pcpOutAddrOff                => dprRpdo1Buf_s.pcp.addrOff,
            pcpOutSelVBuf                => selVBufPcpOneHot,
            apClk                        => ap_clk,
            apReset                        => ap_reset,
            apTrigger                    => vBufTriggerPdo_s.ap(1),
            apOutAddrOff                => dprRpdo1Buf_s.ap.addrOff,
            apOutSelVBuf                => selVBufApOneHot
        );

    end block;
--
------------------------------------------------------------------------------------------------------------------------
end generate;

genRpdo2 : if iRpdos_g >= 3 generate
------------------------------------------------------------------------------------------------------------------------
--RPDO2 buffer
    theRpdo2TrippleBuffer : block
    signal selVBufPcpOneHot                : std_logic_vector(2 downto 0);
    signal selVBufApOneHot                : std_logic_vector(2 downto 0);
    begin

        vBufSel_s.pcp(23 downto 16) <=     x"00" when selVBufPcpOneHot = "001" else
                                        x"11" when selVBufPcpOneHot = "010" else
                                        x"22" when selVBufPcpOneHot = "100" else
                                        x"FF";

        vBufSel_s.ap(23 downto 16) <=     x"00" when selVBufApOneHot = "001" else
                                        x"11" when selVBufApOneHot = "010" else
                                        x"22" when selVBufApOneHot = "100" else
                                        x"FF";

        dprRpdo2Buf_s.pcp.din <= pcp_writedata;
        outRpdo2Buf_s.pcp <= dprOut.pcp;
        dprRpdo2Buf_s.pcp.be <= pcp_byteenable;
        dprRpdo2Buf_s.pcp.wr <= pcp_write;

        dprRpdo2Buf_s.ap.din <= ap_writedata;
        outRpdo2Buf_s.ap <= dprOut.ap;
        dprRpdo2Buf_s.ap.be <= ap_byteenable;
        dprRpdo2Buf_s.ap.wr <= ap_write;

        theTrippleMechanism : entity work.tripleVBufLogic
        generic map (
            genOnePdiClkDomain_g         => genOnePdiClkDomain_g,
            --base address of virtual buffers in DPR
            iVirtualBufferBase_g        => intRpdo2Buf_c.base/4, --double word!
            --size of one virtual buffer in DPR (must be aligned!!!)
            iVirtualBufferSize_g        => intRpdo2Buf_c.span/3/4, --double word!
            --out address width
            iOutAddrWidth_g                => dprRpdo2Buf_s.pcp.addr'length,
            --in address width
            iInAddrWidth_g                => extLog2MaxOneSpan-2,
            --ap is NOT producer
            bApIsProducer                => false
        )

        port map (
            pcpClk                        => pcp_clk,
            pcpReset                    => pcp_reset,
            pcpTrigger                    => vBufTriggerPdo_s.pcp(2),
            pcpOutAddrOff                => dprRpdo2Buf_s.pcp.addrOff,
            pcpOutSelVBuf                => selVBufPcpOneHot,
            apClk                        => ap_clk,
            apReset                        => ap_reset,
            apTrigger                    => vBufTriggerPdo_s.ap(2),
            apOutAddrOff                => dprRpdo2Buf_s.ap.addrOff,
            apOutSelVBuf                => selVBufApOneHot
        );

    end block;
--
------------------------------------------------------------------------------------------------------------------------
end generate;

------------------------------------------------------------------------------------------------------------------------
-- waitrequest signals
    theWaitrequestGenerators : block
        signal pcp_wr, pcp_rd, pcp_rd_ack, pcp_wr_ack : std_logic;
        signal ap_wr, ap_rd, ap_rd_ack, ap_wr_ack : std_logic;
    begin

        -- PCP
        thePcpWrWaitReqAckGen : entity work.req_ack
        generic map (
            zero_delay_g => true
        )
        port map (
            clk => pcp_clk,
            rst => pcp_reset,
            enable => pcp_wr,
            ack => pcp_wr_ack
        );

        thePcpRdWaitReqAckGen : entity work.req_ack
        generic map (
            ack_delay_g => 2,
            zero_delay_g => false
        )
        port map (
            clk => pcp_clk,
            rst => pcp_reset,
            enable => pcp_rd,
            ack => pcp_rd_ack
        );

        pcp_wr <= pcp_chipselect and pcp_write;
        pcp_rd <= pcp_chipselect and pcp_read;
        pcp_waitrequest <= not(pcp_rd_ack or pcp_wr_ack);

        -- AP
        theApWrWaitReqAckGen : entity work.req_ack
        generic map (
            zero_delay_g => true
        )
        port map (
            clk => ap_clk,
            rst => ap_reset,
            enable => ap_wr,
            ack => ap_wr_ack
        );

        theApRdWaitReqAckGen : entity work.req_ack
        generic map (
            ack_delay_g => 2,
            zero_delay_g => false
        )
        port map (
            clk => ap_clk,
            rst => ap_reset,
            enable => ap_rd,
            ack => ap_rd_ack
        );

        ap_wr <= ap_chipselect and ap_write;
        ap_rd <= ap_chipselect and ap_read;
        ap_waitrequest <= not(ap_rd_ack or ap_wr_ack);

    end block;

--
------------------------------------------------------------------------------------------------------------------------

end architecture rtl;
