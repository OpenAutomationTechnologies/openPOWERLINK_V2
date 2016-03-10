-------------------------------------------------------------------------------
--! @file openmac-rtl-ea.vhd
--
--! @brief openMAC core
--
--! @details This is the openMAC core file implementing the MAC functionality.
-------------------------------------------------------------------------------
--
--    (c) B&R, 2009
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

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;

--! Common library
library libcommon;
--! Use common library global package
use libcommon.global.all;

entity openmac is
    generic (
        --! Dma address high bit
        gDmaHighAddr    : in    integer := 16;
        --! Enable MAC timer
        gTimerEnable    : in    boolean := false;
        --! Enable Timer triggered Tx
        gTimerTrigTx    : in    boolean := false;
        --! Enable Auto-response delay
        gAutoTxDel      : in    boolean := false
    );
    port (
        --! Reset
        iRst            : in    std_logic;
        --! Clock (RMII, 50 MHz)
        iClk            : in    std_logic;
        --! Write to RAM or content (low-active)
        inWrite         : in    std_logic;
        --! Select RAM (descriptor and Rx filter)
        iSelectRam      : in    std_logic;
        --! Select content (Tx/Rx status/control registers)
        iSelectCont     : in    std_logic;
        --! Byteenable (low-active)
        inByteenable    : in    std_logic_vector(1 downto 0);
        --! Address for RAM or content
        iAddress        : in    std_logic_vector(10 downto 1);
        --! Writedata to RAM or content
        iWritedata      : in    std_logic_vector(15 downto 0);
        --! Readdata from RAM or content
        oReaddata       : out   std_logic_vector(15 downto 0);
        --! Tx interrupt request (low-active)
        onTxIrq         : out   std_logic;
        --! Rx interrupt request (low-active)
        onRxIrq         : out   std_logic;
        --! Tx begin interrupt request (low-active)
        onTxBegIrq      : out   std_logic;
        --! DMA read transfer for frame done
        oDmaReadDone    : out   std_logic;
        --! DMA write transfer for frame done
        oDmaWriteDone   : out   std_logic;
        --! DMA request strobe
        oDmaReq         : out   std_logic;
        --! DMA write strobe (low-active)
        onDmaWrite      : out   std_logic;
        --! DMA acknowledge input
        iDmaAck         : in    std_logic;
        --! DMA request overflow flag
        oDmaReqOverflow : out   std_logic;
        --! DMA read request length
        oDmaReadLength  : out   std_logic_vector(11 downto 0);
        --! DMA address
        oDmaAddress     : out   std_logic_vector(gDmaHighAddr downto 1);
        --! DMA writedata
        oDmaWritedata   : out   std_logic_vector(15 downto 0);
        --! DMA readdata
        iDmaReaddata    : in    std_logic_vector(15 downto 0);
        --! RMII Rx data
        iRxData         : in    std_logic_vector(1 downto 0);
        --! RMII Rx data valid
        iRxDv           : in    std_logic;
        --! RMII Tx data
        oTxData         : out   std_logic_vector(1 downto 0);
        --! RMII Tx enable
        oTxEn           : out   std_logic;
        --! Hub Rx port (connect to openHUB)
        iHubRxPort      : in    std_logic_vector(1 downto 0);
        --! MAC Time
        oMacTime        : out   std_logic_vector(31 downto 0)
    );
end entity openmac;

architecture struct OF openmac is
    signal Rx_Dv                        : std_logic;
    signal R_Req                        : std_logic;
    signal Auto_Desc                    : std_logic_vector( 4 downto 0);
    signal Zeit                         : std_logic_vector(31 downto 0);
    signal Tx_Dma_Req                   : std_logic;
    signal Rx_Dma_Req                   : std_logic;
    signal Tx_Dma_Ack                   : std_logic;
    signal Rx_Dma_Ack                   : std_logic;
    signal Tx_Ram_Dat                   : std_logic_vector(15 downto 0);
    signal Rx_Ram_Dat                   : std_logic_vector(15 downto 0);
    signal Tx_Dma_Len                   : std_logic_vector(11 downto 0);
    signal Tx_Reg                       : std_logic_vector(15 downto 0);
    signal Rx_Reg                       : std_logic_vector(15 downto 0);
    signal Dma_Tx_Addr                  : std_logic_vector(oDmaAddress'range);
    signal Dma_Rx_Addr                  : std_logic_vector(oDmaAddress'range);
    signal Dma_Req_s                    : std_logic;
    signal Dma_Rw_s                     : std_logic;
    signal halfDuplex                   : std_logic; -- cActivated ... MAC in half-duplex mode
    signal Tx_Active                    : std_logic; -- cActivated ... TX = Data or CRC
    signal Tx_Dma_Very1stOverflow       : std_logic; -- cActivated ... very first TX DMA overflow
    signal Tx_Col                       : std_logic;
    signal Sel_Tx_Ram                   : std_logic;
    signal Sel_Tx_Reg                   : std_logic;
    signal Tx_LatchH                    : std_logic_vector(7 downto 0);
    signal Tx_LatchL                    : std_logic_vector(7 downto 0);
begin

    oReaddata <=    Tx_Ram_Dat    when    iSelectRam  = '1' and Sel_Tx_Ram = '1'        else
                Rx_Ram_Dat    when    iSelectRam  = '1'                            else
                Tx_Reg        when    iSelectCont = '1' and Sel_Tx_Reg = '1'        else
                Rx_Reg;

    oMacTime <= Zeit;

    oDmaReadLength <= Tx_Dma_Len + 4;

b_DmaObserver : block
    signal dmaObserverCounter, dmaObserverCounterNext : std_logic_vector(2 downto 0);
    constant cDmaObserverCounterHalf : std_logic_vector(dmaObserverCounter'range) := "110"; --every 8th cycle
    constant cDmaObserverCounterFull : std_logic_vector(dmaObserverCounter'range) := "010"; --every 4th cycle
begin

    process(iClk, iRst)
    begin
        if iRst = '1' then
            dmaObserverCounter <= (others => cInactivated);
        elsif rising_edge(iClk) then
            dmaObserverCounter <= dmaObserverCounterNext;
        end if;
    end process;

    oDmaReqOverflow <= --very first TX Dma transfer
                        Dma_Req_s when Tx_Dma_Very1stOverflow = cActivated and Tx_Active = cInactivated else
                        --RX Dma transfers and TX Dma transfers without the very first
                        Dma_Req_s when dmaObserverCounterNext = cDmaObserverCounterHalf and halfDuplex = cActivated else
                        Dma_Req_s when dmaObserverCounterNext = cDmaObserverCounterFull and halfDuplex = cInactivated else
                        cInactivated;

    dmaObserverCounterNext <= --increment counter if DMA Read req (TX) during data and crc
                              dmaObserverCounter + 1 when Dma_Req_s = cActivated and Dma_Rw_s = cActivated
                                                                                 and Tx_Active = cActivated else
                              --increment counter if DMA Write req (RX)
                              dmaObserverCounter + 1 when Dma_Req_s = cActivated and Dma_Rw_s = cInactivated else
                              (others => cInactivated); --reset DmaObserverCounter if no oDmaReq

end block;

b_Dma:    block
    signal Rx_Dma   : std_logic;
    signal Tx_Dma   : std_logic;
begin
    oDmaReq <= Dma_Req_s;
    Dma_Req_s  <= '1'    when    (Tx_Dma_Req = '1' and Tx_Dma_Ack = '0') or Rx_Dma_Req = '1'        else '0';

    onDmaWrite <= Dma_Rw_s;
    Dma_Rw_s   <= '1'            when   (Rx_Dma = '0' and Tx_Dma_Req = '1' and Tx_Dma_Ack = '0') or Tx_Dma = '1'     else '0';
    oDmaAddress   <= Dma_Tx_Addr    when   (Rx_Dma = '0' and Tx_Dma_Req = '1' and Tx_Dma_Ack = '0') or Tx_Dma = '1'     else Dma_Rx_Addr;

    Rx_Dma_Ack <= '1'    when    Rx_Dma = '1' and iDmaAck = '1'    else '0';

pDmaArb: process( iClk, iRst )     is
begin

    if iRst = '1'    then
        Rx_Dma <= '0'; Tx_Dma <= '0'; Tx_Dma_Ack <= '0';
        Tx_LatchH <= (others => '0'); Tx_LatchL <= (others => '0');
        Zeit <= (others => '0');
    elsif rising_edge( iClk )    then

        if gTimerEnable then
            Zeit <= Zeit + 1;
        end if;

        Sel_Tx_Ram  <= iAddress(9);
        Sel_Tx_Reg  <= not iAddress(3);

        if        iDmaAck = '0'    then
            if        Rx_Dma = '0' and Tx_Dma_Req = '1' and Tx_Dma_Ack = '0'    then    Tx_Dma <= '1';
            elsif    Tx_Dma = '0' and Rx_Dma_Req = '1'                        then    Rx_Dma <= '1';
            end if;
        else
            if        Rx_Dma = '1' and Tx_Dma_Req = '1' and Tx_Dma_Ack = '0'     then    Tx_Dma <= '1';    Rx_Dma <= '0';
            elsif    Tx_Dma = '1' and Rx_Dma_Req = '1'                        then    Tx_Dma <= '0';    Rx_Dma <= '1';
            else                                                                    Tx_Dma <= '0';    Rx_Dma <= '0';
            end if;
        end if;

        if        Tx_Dma = '1' and iDmaAck = '1'    then    Tx_Dma_Ack <= '1';
        else                                            Tx_Dma_Ack <= '0';
        end if;

        if    Tx_Dma_Ack = '1'        then    Tx_LatchL <= iDmaReaddata(15 downto 8);
                                            Tx_LatchH <= iDmaReaddata( 7 downto 0);
        end if;

    end if;

end process pDmaArb;

end block b_Dma;

b_Full_Tx : block
    type tTxState is (
        sIdle,
        sBop,
        sPre,
        sTxd,
        sCrc,
        sCol,
        sJam
    );

    signal Sm_Tx        : tTxState;
    signal Start_Tx     : std_logic;
    signal ClrCol       : std_logic;
    signal Tx_On        : std_logic;
    signal Dibl_Cnt     : std_logic_vector(1 downto 0);
    signal F_End        : std_logic;
    signal Was_Col      : std_logic;
    signal Block_Col    : std_logic;
    signal Ipg_Cnt      : std_logic_vector(7 downto 0);
    signal Tx_Timer     : std_logic_vector(7 downto 0);
    alias  Ipg          : std_logic is Ipg_Cnt(7);
    alias  Tx_Time      : std_logic is Tx_Timer(7);
    signal Tx_Ipg       : std_logic_vector(5 downto 0);
    signal Tx_Count     : std_logic_vector(11 downto 0);
    signal Tx_En        : std_logic;
    signal F_Val        : std_logic;
    signal Tx_Half      : std_logic;
    signal Tx_Sr        : std_logic_vector(7 downto 0);
    signal F_TxB        : std_logic_vector(7 downto 0);
    signal Crc          : std_logic_vector(31 downto 0);
    signal CrcDin       : std_logic_vector(1 downto 0);
    signal Tx_Dat       : std_logic_vector(1 downto 0);
    signal Col_Cnt      : std_logic_vector(3 downto 0);
    signal Auto_Coll    : std_logic;
    signal Rnd_Num      : std_logic_vector(9 downto 0);
    signal Retry_Cnt    : std_logic_vector(9 downto 0);
    signal Max_Retry    : std_logic_vector(3 downto 0);
begin

    oTxEn  <= Tx_En;
    oTxData <= Tx_Dat;

    halfDuplex <= Tx_Half;

    Tx_Active <= cActivated when Sm_Tx = sTxd or Sm_Tx = sCrc else cInactivated;

pTxSm: process ( iClk, iRst )     is
begin

    if iRst = '1'    then
        Sm_Tx <= sIdle;
    elsif rising_edge( iClk ) then
        if    Sm_Tx = sIdle or Sm_Tx = sBop or Dibl_Cnt = "11"     then
            case Sm_Tx    is
                when    sIdle =>    if    Start_Tx = '1'
                                    and (Tx_Half = '0' or Rx_Dv = '0')
                                    and Ipg = '0'                        then    Sm_Tx <= sBop;    end if;
                when    sBop =>                                                Sm_Tx <= sPre;
                when    sPre =>    if        Tx_Time = '1'                then    Sm_Tx <= sTxd;    end if;
                when    sTxd =>    if        Was_Col = '1'                then    Sm_Tx <= sCol;
                                    elsif    Tx_Count = 0                then    Sm_Tx <= sCrc;    end if;
                when    sCol =>                                                Sm_Tx <= sJam;
                when    sJam =>    if        Tx_Time = '1'                then    Sm_Tx <= sIdle;
                                    end if;
                when    sCrc =>    if        Was_Col = '1'                then    Sm_Tx <= sCol;
                                    elsif    Tx_Time = '1'                then    Sm_Tx <= sIdle;    end if;
                when    others  =>    NULL;
            end case;
        end if;
    end if;

end process pTxSm;


pTxCtl: process ( iClk, iRst )     is
    variable vPreload   : std_logic_vector(Tx_Timer'range);
    variable vLoad      : std_logic;
begin

    if iRst = '1'    then
        Tx_Dat <= "00"; Tx_En <= '0'; Dibl_Cnt <= "00"; F_End <= '0'; F_Val <= '0'; Tx_Col  <= '0'; Was_Col <= '0'; Block_Col <= '0';
        Ipg_Cnt <= (others => '0'); Tx_Timer <= (others => '0'); Tx_Sr <= (others => '0');
    elsif rising_edge( iClk )     then

        if        Sm_Tx = sBop        then    Dibl_Cnt <= "00";
        else                                Dibl_Cnt <= Dibl_Cnt + 1;
        end if;

        if        Tx_En = '1'                        then    Ipg_Cnt <= "1"  & conv_std_logic_vector( 44, 7);
        elsif    Rx_Dv = '1' and Tx_Half = '1'    then    Ipg_Cnt <= "10" & Tx_Ipg;
        elsif    Ipg = '1'                         then    Ipg_Cnt <= Ipg_Cnt - 1;
        end if;

        if        Dibl_Cnt = "11" and Sm_Tx = sCrc and Tx_Time = '1'        then    F_End  <= '1';
        elsif    Dibl_Cnt = "11" and Sm_Tx = sCol then
            if        Col_Cnt = (Max_Retry - 1)                            then    F_End  <= '1';
            elsif    Col_Cnt < x"E"                                        then    Tx_Col <= '1';
            else                                                                F_End  <= '1';
            end if;
        else                                                                    F_End  <= '0';
                                                                                Tx_Col <= '0';
        end if;

        if         Tx_Half = '1' and Rx_Dv = '1'
            and (Sm_Tx = sPre or Sm_Tx = sTxd)    then    Was_Col <= '1';
        elsif     Sm_Tx = sCol                        then    Was_Col <= '0';
        end if;

        if         Sm_Tx = sCol                    then    Block_Col <= '1';
        elsif     Auto_Coll = '1'                then    Block_Col <= '0';
        elsif     Retry_Cnt = 0                    then    Block_Col <= '0';
        end if;

        if        Dibl_Cnt = "10" and Sm_Tx = sPre and Tx_Time = '1'    then    F_Val <= '1';
        elsif   Dibl_Cnt = "10" and Sm_Tx = sTxd                    then    F_Val <= '1';
        else                                                                F_Val <= '0';
        end if;

        vLoad := '0';
        if        Sm_Tx = sBop        then    vPreload := x"06";    vLoad := '1';
        elsif    Sm_Tx = sTxd        then    vPreload := x"02";    vLoad := '1';
        elsif    Sm_Tx = sCol        then    vPreload := x"01";    vLoad := '1';
        elsif    Tx_Time = '1'        then    vPreload := x"3e";    vLoad := '1';
        end if;

        if        Dibl_Cnt = "11"    or Sm_Tx = sBop     then
            if        vLoad = '1'    then    Tx_Timer <= vPreload;
            else                        Tx_Timer <= Tx_Timer - 1;
            end if;
        end if;

        if        F_Val = '1'        then    Tx_Sr <= F_TxB;
        else                            Tx_Sr <= "00" & Tx_Sr(7 downto 2);
        end if;

        if        Sm_Tx = sPre                                        then    Tx_En <= '1';
        elsif    Sm_Tx = sIdle or (Sm_Tx = sJam and Tx_Time = '1')    then    Tx_En <= '0';
        end if;

        if        Sm_Tx = sPre and Tx_Time = '1' and Dibl_Cnt = "11"    then    Tx_Dat <= "11";
        elsif    Sm_Tx = sPre                                        then    Tx_Dat <= "01";
        elsif    Sm_Tx = sTxd                                        then    Tx_Dat <= CrcDin;
        elsif    Sm_Tx = sCrc                                        then    Tx_Dat <= not Crc(30) & not Crc(31);
        elsif    Sm_Tx = sCol or Sm_Tx = sJam                        then    Tx_Dat <= "11";
        else                                                                Tx_Dat <= "00";
        end if;

    end if;

end process pTxCtl;

pBackDel: process ( iClk, iRst )    is
begin
    if iRst = '1'    then
        Rnd_Num   <= (others => '0');
        Col_Cnt   <= (others => '0');
        Retry_Cnt <= (others => '0');
    elsif rising_edge( iClk )     then

        Rnd_Num <= Rnd_Num(8 downto 0) & (Rnd_Num(9) xor not Rnd_Num(2));

        if        ClrCol = '1'                        then    Col_Cnt <= x"0";
        elsif    Dibl_Cnt = "11"    and Sm_Tx = sCol    then    Col_Cnt <= Col_Cnt + 1;
        end if;

        if    Dibl_Cnt = "11"    then
            if        Tx_On = '0'    or Auto_Coll = '1'        then    Retry_Cnt <= (others => '0');
            elsif    Sm_Tx = sCol  then
                for i in 0 to 9 loop
                    if     Col_Cnt >= i    then    Retry_Cnt(i) <= Rnd_Num(i);
                    else                        Retry_Cnt(i) <= '0';
                    end if;
                end loop;
            elsif Sm_Tx /= sJam and Tx_Time = '1' and Retry_Cnt /= 0    then    Retry_Cnt <= Retry_Cnt - 1;
            end if;
        end if;
    end if;
end process pBackDel;


    CrcDin <= Tx_Sr(1 downto 0);

Calc: process ( iClk, Crc, CrcDin, Sm_Tx )     is
    variable H : std_logic_vector(1 downto 0);
begin

    H(0) := Crc(31) xor CrcDin(0);
    H(1) := Crc(30) xor CrcDin(1);

    if rising_edge( iClk )  then
        if        Sm_Tx = sPre        then    Crc <= x"FFFFFFFF";
        elsif     Sm_Tx = sCrc        then    Crc <= Crc(29 downto 0) & "00";
        else
            Crc( 0) <=                        H(1);
            Crc( 1) <=             H(0) xor H(1);
            Crc( 2) <= Crc( 0) xor H(0) xor H(1);
            Crc( 3) <= Crc( 1) xor H(0)         ;
            Crc( 4) <= Crc( 2)          xor H(1);
            Crc( 5) <= Crc( 3) xor H(0) xor H(1);
            Crc( 6) <= Crc( 4) xor H(0)         ;
            Crc( 7) <= Crc( 5)          xor H(1);
            Crc( 8) <= Crc( 6) xor H(0) xor H(1);
            Crc( 9) <= Crc( 7) xor H(0)         ;
            Crc(10) <= Crc( 8)          xor H(1);
            Crc(11) <= Crc( 9) xor H(0) xor H(1);
            Crc(12) <= Crc(10) xor H(0) xor H(1);
            Crc(13) <= Crc(11) xor H(0)         ;
            Crc(14) <= Crc(12)                  ;
            Crc(15) <= Crc(13)                  ;
            Crc(16) <= Crc(14)          xor H(1);
            Crc(17) <= Crc(15) xor H(0)         ;
            Crc(18) <= Crc(16)                  ;
            Crc(19) <= Crc(17)                  ;
            Crc(20) <= Crc(18)                  ;
            Crc(21) <= Crc(19)                  ;
            Crc(22) <= Crc(20)          xor H(1);
            Crc(23) <= Crc(21) xor H(0) xor H(1);
            Crc(24) <= Crc(22) xor H(0)         ;
            Crc(25) <= Crc(23)                  ;
            Crc(26) <= Crc(24)          xor H(1);
            Crc(27) <= Crc(25) xor H(0)         ;
            Crc(28) <= Crc(26)                  ;
            Crc(29) <= Crc(27)                  ;
            Crc(30) <= Crc(28)                  ;
            Crc(31) <= Crc(29)                  ;
        end if;
    end if;
end process Calc;

bTxDesc:    block
    type tDescState is (
        sIdle,
        sLen,
        sTimL,
        sTimH,
        sAdrH,
        sAdrL,
        sReq,
        sBegL,
        sBegH,
        sDel,
        sData,
        sStat,
        sColl
    );

    signal Dsm          : tDescState;
    signal Tx_Dsm_Next  : tDescState;
    signal DescRam_Out  : std_logic_vector(15 downto 0);
    signal DescRam_In   : std_logic_vector(15 downto 0);
    alias  TX_LEN       : std_logic_vector(11 downto 0) is DescRam_Out(11 downto 0);
    alias  TX_OWN       : std_logic                     is DescRam_Out(8);
    alias  TX_LAST      : std_logic                     is DescRam_Out(9);
    alias  TX_READY     : std_logic                     is DescRam_Out(10);
    alias  TX_BEGDEL    : std_logic                     is DescRam_Out(12);
    alias  TX_BEGON     : std_logic                     is DescRam_Out(13);
    alias  TX_TIME      : std_logic                     is DescRam_Out(14);
    alias  TX_RETRY     : std_logic_vector( 3 downto 0) is DescRam_Out(3 downto 0);
    signal Ram_Be       : std_logic_vector( 1 downto 0);
    signal Ram_Wr       : std_logic;
    signal Desc_We      : std_logic;
    signal Last_Desc    : std_logic;
    signal ZeitL        : std_logic_vector(15 downto 0);
    signal Tx_Ie        : std_logic;
    signal Tx_Wait      : std_logic;
    signal Tx_BegInt    : std_logic;
    signal Tx_BegSet    : std_logic;
    signal Tx_Early     : std_logic;
    signal Tx_Del       : std_logic;
    signal Ext_Tx       : std_logic;
    signal Ext_Ack      : std_logic;
    signal Tx_Desc      : std_logic_vector(Auto_Desc'range);
    signal Tx_Desc_One  : std_logic_vector(Tx_Desc'range);
    signal Ext_Desc     : std_logic_vector(Tx_Desc'range);
    signal DescIdx      : std_logic_vector( 2 downto 0);
    signal Desc_Addr    : std_logic_vector(Tx_Desc'length + DescIdx'length - 1 downto 0);
    signal Tx_Icnt      : std_logic_vector( 4 downto 0);
    signal Tx_SoftInt   : std_logic;
    signal Sel_TxH      : std_logic;
    signal Sel_TxL      : std_logic;
    signal H_Byte       : std_logic;
    signal Tx_Buf       : std_logic_vector( 7 downto 0);
    signal Tx_Idle      : std_logic;
    signal TxInt        : std_logic;
    signal Tx_Beg       : std_logic;
    signal Tx_Sync      : std_logic;
    signal Tx_Ident     : std_logic_vector( 1 downto 0);
    signal Tx_Cmp_High  : std_logic_vector(15 downto 0);
    signal Start_TxS    : std_logic;
    signal Tx_Dma_Out   : std_logic;
    signal Tx_Del_Cnt   : std_logic_vector(32 downto 0);
    alias  Tx_Del_End   : std_logic is Tx_Del_Cnt(Tx_Del_Cnt'high);
    signal Tx_Del_Run   : std_logic;
    signal Tx_Done      : std_logic;

begin

    oDmaReadDone <= Tx_Done;

    Tx_Done <= '1' when Dsm = sStat or Dsm = sColl else '0';

    Tx_Dma_Very1stOverflow <= cActivated when Dibl_Cnt = "01" and Sm_Tx = sPre and Tx_Timer(7) = '1' else cInactivated;

    Ram_Wr    <= '1' when    inWrite = '0' and iSelectRam = '1' and iAddress(10 downto 9) = "11"    else '0';
    Ram_Be(1) <= '1' when    inWrite = '1' or inByteenable(1) = '0'                        else '0';
    Ram_Be(0) <= '1' when    inWrite = '1' or inByteenable(0) = '0'                        else '0';

    DescIdx <=    "000"    when    Desc_We = '0' and Tx_Dsm_Next = sIdle    else
                "000"    when    Desc_We = '1' and Dsm = sIdle            else
                "001"    when    Desc_We = '0' and Tx_Dsm_Next = sLen    else
                "001"    when    Desc_We = '1' and Dsm = sLen            else
                "010"    when    Desc_We = '0' and Tx_Dsm_Next = sAdrH    else
                "010"    when    Desc_We = '1' and Dsm = sAdrH            else
                "011"    when    Desc_We = '0' and Tx_Dsm_Next = sAdrL    else
                "011"    when    Desc_We = '1' and Dsm = sAdrL            else
                "100"    when    Desc_We = '0' and Tx_Dsm_Next = sBegH    else
                "100"    when    Desc_We = '1' and Dsm = sBegH            else
                "101"    when    Desc_We = '0' and Tx_Dsm_Next = sBegL    else
                "101"    when    Desc_We = '1' and Dsm = sBegL            else
                "110"    when    Desc_We = '0' and Tx_Dsm_Next = sTimH    else
                "110"    when    Desc_We = '1' and Dsm = sTimH            else
                "111"    when    Desc_We = '0' and Tx_Dsm_Next = sTimL    else
                "111"    when    Desc_We = '1' and Dsm = sTimL            else
                "111"    when    Desc_We = '0' and Tx_Dsm_Next = sData    else
                "111"    when    Desc_We = '1' and Dsm = sData            else
                "000";

    Desc_We <= '1' when  Dsm = sTimL or Dsm = sTimH or Dsm = sStat    else   '0';

    Desc_Addr <= Tx_Desc  & DescIdx    when    Ext_Tx = '0'    else
                 Ext_Desc & DescIdx;

gTxTime:    if gTimerEnable generate
    DescRam_In <= Zeit(15 downto 0)            when    Dsm  = sTimH    else
                  ZeitL                        when    Dsm  = sTimL    else
                  x"000" & "01" & Tx_Ident    when    Dsm  = sBegL    else
                  Tx_Dma_Out & Tx_Sync & "00" & "0100" & "00" & "0" & "0" & Col_Cnt;
end generate;

gnTxTime:    if not gTimerEnable generate
    DescRam_In <= x"000" & "01" & Tx_Ident    when    Dsm  = sBegL    else
                  Tx_Dma_Out & Tx_Sync & "00" & "0100" & "00" & "0" & "0" & Col_Cnt;
end generate;

    --! This DPRAM holds the Tx descriptor accessible by the host and the DMA.
    TXRAM : entity work.dpRamOpenmac
        generic map (
            gWordWidth      => iWritedata'length,
            gNumberOfWords  => 256,
            gInitFile       => "UNUSED"
        )
        port map (
            iClk_A          => iClk,
            iEnable_A       => cActivated,
            iWriteEnable_A  => Ram_Wr,
            iAddress_A      => iAddress(8 downto 1),
            iByteenable_A   => Ram_Be,
            iWritedata_A    => iWritedata,
            oReaddata_A     => Tx_Ram_Dat,
            iClk_B          => iClk,
            iEnable_B       => cActivated,
            iWriteEnable_B  => Desc_We,
            iByteenable_B   => (others => cActivated),
            iAddress_B      => Desc_Addr,
            iWritedata_B    => DescRam_In,
            oReaddata_B     => DescRam_Out
        );

    assert not( gTimerTrigTx and not gTimerEnable )
        report "Time Triggered Tx needs Timer!"
            severity failure;

pTxSm: process( Dsm,
                Tx_On, TX_OWN, Retry_Cnt, Ext_Tx, Tx_Wait,
                Tx_Sync, Sm_Tx, F_End, Tx_Col, Ext_Ack, Tx_Del, Tx_Beg, Tx_Half, Tx_Del_End,
                iRxDv )
begin


        Tx_Dsm_Next <= Dsm;
        case    Dsm is
            when sIdle     =>    if    Tx_On = '1' and TX_OWN = '1' and Retry_Cnt = 0    then
                                if    (Ext_Tx = '1' and Ext_Ack = '0') or Tx_Wait  = '0'      then
                                                        Tx_Dsm_Next <= sAdrH; --sLen;
                                end if;
                            end if;
            when sLen     =>    if    Tx_Sync = '0'    then    Tx_Dsm_Next <= sReq; --sAdrH;
                            else                        Tx_Dsm_Next <= sBegH;
                            end if;
            when sBegH     =>                                Tx_Dsm_Next <= sBegL;
            when sBegL     =>    if       Tx_On  = '0'    then    Tx_Dsm_Next <= sIdle;
                            elsif Tx_Sync = '0'    then
                                if      Tx_Del = '1' then    Tx_Dsm_Next <= sDel;
                                elsif Sm_Tx = sPre    then
                                                        Tx_Dsm_Next <= sTimH;
                                end if;
                            elsif Tx_Sync = '1' and Tx_Beg = '1' and Tx_Half = '1' and iRxDv = '1' then
                                                        Tx_Dsm_Next <= sColl;
                            elsif Tx_Beg = '1'    then    Tx_Dsm_Next <= sReq;
                            end if;
            when sDel     =>    if Tx_On = '0'      then    Tx_Dsm_Next <= sIdle; --avoid FSM hang
                            elsif Tx_Del_End = '1' then Tx_Dsm_Next <= sTimH;
                            end if;
            when sAdrH     =>                                Tx_Dsm_Next <= sAdrL;
            when sAdrL   =>                             Tx_Dsm_Next <= sLen; --sReq;
            --leaving sAdrL and entering sReq leads to the very first Tx_Dma_Req
            -- this enables early dma req at the beginning of IPG (auto-resp)
            when sReq     =>    if    Tx_On  = '0'    then    Tx_Dsm_Next <= sIdle;
                            elsif Tx_Del = '1'    then    Tx_Dsm_Next <= sBegH;
                            elsif Tx_Sync = '0'    then    Tx_Dsm_Next <= sBegL;
                            elsif Sm_Tx = sBop    then    Tx_Dsm_Next <= sTimH;
                            end if;
            when sTimH     =>                                Tx_Dsm_Next <= sTimL;
            when sTimL     =>                                Tx_Dsm_Next <= sData;
            when sData     =>    if      F_End = '1'    then    Tx_Dsm_Next <= sStat;
                            elsif Tx_Col = '1'    then    Tx_Dsm_Next <= sColl;
                            end if;
            when sStat     =>                                Tx_Dsm_Next <= sIdle;
            when sColl     =>    if    sm_tx = sIdle then
                                if    Tx_Sync = '1' then    Tx_Dsm_Next <= sStat;
                                else                    Tx_Dsm_Next <= sIdle;
                                end if;
                            end if;
            when others     =>
        end case;
end process pTxSm;

    pTxSmClk : process(iRst, iClk)
    begin
        if iRst = cActivated then
            Dsm <= sIdle;
        elsif rising_edge(iClk) then
            Dsm <= Tx_Dsm_Next;
        end if;
    end process pTxSmClk;

pTxControl: process( iRst, iClk )
begin

    if    iRst = '1'    then
        Last_Desc <= '0'; Start_TxS <= '0'; Tx_Dma_Req  <= '0'; H_Byte <= '0';
        Tx_Beg <= '0'; Tx_BegSet <= '0'; Tx_Early <= '0'; Auto_Coll <= '0'; Tx_Dma_Out <= '0';
        Ext_Tx <= '0'; Ext_Ack <= '0'; ClrCol <= '0'; Ext_Desc <= (others => '0'); Tx_Sync <= '0'; Max_Retry <= (others => '0');
        ZeitL <= (others => '0'); Tx_Count <= (others => '0'); Tx_Ident <= "00";
        Dma_Tx_Addr <= (others => '0'); Tx_Cmp_High <= (others => '0');
        Tx_Del_Run <= '0';
        Tx_Del <= '0'; Tx_Del_Cnt <= (others => '0'); Tx_Dma_Len <= (others => '0');
    elsif    rising_edge( iClk )     then

        if    gTimerTrigTx  = true    then
            if        Tx_Sync = '1' and Dsm = sBegL and (DescRam_Out & Tx_Cmp_High ) = Zeit    then    Tx_Beg <= '1';
            else                                                                                    Tx_Beg <= '0';
            end if;
        end if;

        if    Dsm = sStat and Desc_We = '1'    then    ClrCol  <= '1';
        else                                        ClrCol  <= '0';
        end if;

        if gTimerEnable then
            if    Dsm  = sTimH    then    ZeitL <= Zeit(31 downto 16);
            end if;
        end if;

        if        Ext_Ack = '0' and R_Req = '1'                    then    Ext_Desc <= Auto_Desc;
                                                                        Ext_Ack  <= '1';
        elsif    Ext_Tx = '1' or    Tx_On = '0'                        then    Ext_Ack  <= '0';
        end if;

        if        Dsm = sIdle and Ext_Ack = '1'                    then    Ext_Tx  <= '1';
        elsif    Dsm = sStat or     Tx_Col = '1' or    Tx_On = '0'        then    Ext_Tx  <= '0';
        end if;

        if       (F_End = '1' or Tx_On = '0'
            or (Tx_Col = '1' and Ext_Tx = '1' )
            or dsm = sColl     )                            then    Start_TxS <= '0';
                                                                Auto_Coll <= Auto_Coll or (Tx_Col and Ext_Tx);
        elsif    Dsm = sReq and Tx_Del = '0'            then    Start_TxS <= '1';
        elsif    Dsm = sDel and Tx_Del_End = '1'            then    Start_TxS <= '1';
        elsif    Sm_Tx = sIdle                            then    Auto_Coll <= '0';
        end if;

        if        Dsm = sIdle        then    Last_Desc <= TX_LAST;
        end if;

        if        Dsm = sLen        then    Tx_Count  <= TX_LEN; Tx_Dma_Len <= TX_LEN; --add CRC
        elsif   F_Val = '1'        then    Tx_Count  <= Tx_Count - 1;
        end if;

        if        Dsm = sBegH        then    Tx_Cmp_High <= DescRam_Out;
        end if;

        if    Dsm = sIdle and Tx_On = '1' and TX_OWN = '1' and Retry_Cnt = 0    then
            if    Ext_Tx = '1' or Tx_Wait  = '0'      then
                if    gTimerTrigTx then                    Tx_Sync <= TX_TIME;
                else                                Tx_Sync <= '0';
                end if;
                                                    Max_Retry <= TX_RETRY;
                                                    Tx_Early  <= TX_BEGON;

                if    gAutoTxDel = true            then    Tx_Del      <= TX_BEGDEL;
                end if;
            end if;
        elsif Dsm = sTimH                    then    Tx_BegSet <= Tx_Early;
        elsif Dsm = sTimL                    then    Tx_BegSet <= '0';
        elsif Dsm = sIdle                    then    Tx_Del <= '0';
        end if;

        if    gAutoTxDel = true and Tx_Del = '1' then
            if    Dsm = sBegH                            then    Tx_Del_Cnt(Tx_Del_Cnt'high)    <= '0';
                                                            Tx_Del_Cnt(15 downto 0)    <= DescRam_Out;
            elsif    Dsm = sBegL                        then    Tx_Del_Cnt(31 downto 16) <= DescRam_Out;
            elsif    Dsm = sDel and Tx_Del_Run = '1'    then    Tx_Del_Cnt <= Tx_Del_Cnt - 1;
            end if;

            if        Tx_Del_Run = '0' and  Dsm = sDel                then    Tx_Del_Run <= '1'; --don't consider Ipg
            elsif    Tx_Del_End = '1'                                then    Tx_Del_Run <= '0';
            end if;

        end if;

        if        Dsm = sAdrL         then    --Dma_Tx_Addr(15 downto 1)    <= DescRam_Out(15 downto 1);
                Dma_Tx_Addr(oDmaAddress'high downto 16) <= DescRam_Out(oDmaAddress'high-16 downto 0);
                Tx_Ident <= DescRam_Out(15 downto 14);
        elsif    Tx_Dma_Ack = '1' then    Dma_Tx_Addr(15 downto 1)    <= Dma_Tx_Addr(15 downto 1) + 1;
        end if;

        if        Dsm = sAdrH         then    Dma_Tx_Addr(15 downto 1)    <= DescRam_Out(15 downto 1);
--                Dma_Tx_Addr(oDmaAddress'high downto 16) <= DescRam_Out(oDmaAddress'high-16 downto 0);
--                Tx_Ident <= DescRam_Out(15 downto 14);
        elsif    Tx_Dma_Ack = '1' and Dma_Tx_Addr(15 downto 1) = x"FFF" & "111"    then
                Dma_Tx_Addr(oDmaAddress'high downto 16) <= Dma_Tx_Addr(oDmaAddress'high downto 16) + 1;
        end if;

        if        DSM = sAdrL
            or (F_Val = '1' and H_Byte = '0')    then    Tx_Dma_Req  <= '1';
        elsif    Tx_Dma_Ack = '1'                then    Tx_Dma_Req  <= '0';
        end if;

        if        Sm_Tx = sBop                    then    H_Byte <= '0';
        elsif    F_Val = '1'                        then    H_Byte <= not H_Byte;
        end if;

        if    F_Val = '1'            then    Tx_Buf <= Tx_LatchL;
        end if;

        if        H_Byte = '0' and F_Val = '1' and Tx_Dma_Req = '1'    then    Tx_Dma_Out <= '1';
        elsif    Sm_Tx = sBop                                        then    Tx_Dma_Out <= '0';
        end if;

    end if;

end process pTxControl;

    Start_Tx <= '1'    when    Start_TxS = '1' and Block_Col = '0'            else
                '1'    when    not gAutoTxDel and not gTimerTrigTx and R_Req = '1'    else
                '0';
    F_TxB <=    Tx_LatchH    when    H_Byte = '0'    else
                Tx_Buf;

    onTxIrq <= '1'    when    (Tx_Icnt = 0 and Tx_SoftInt = '0') or Tx_Ie = '0'    else    '0';

    Tx_Idle <= '1'    when    Sm_Tx = sIdle and Dsm = sIdle else '0';

    Tx_Reg <= Tx_Ie & Tx_SoftInt & Tx_Half & Tx_Wait & (Tx_Icnt(4) or Tx_Icnt(3)) & Tx_Icnt(2 downto 0) &
              Tx_On &  Tx_BegInt & Tx_Idle & Tx_Desc;

    Sel_TxH <= '1'    when inWrite = '0' and iSelectCont = '1' and iAddress(3) = '0' and    Ram_Be(1) = '1'    else    '0';
    Sel_TxL <= '1'    when inWrite = '0' and iSelectCont = '1' and iAddress(3) = '0' and    Ram_Be(0) = '1'    else    '0';

    Tx_Desc <= Tx_Desc_One;

    Tx_SoftInt <= '0';

pTxRegs: process( iRst, iClk )
begin

    if    iRst = '1'    then
        Tx_On   <= '0'; Tx_Ie <= '0'; Tx_Half  <= '0'; Tx_Wait  <= '0'; onTxBegIrq <= '0';
        Tx_Desc_One <= (others => '0');
        Tx_Icnt <= (others => '0'); TxInt <= '0'; Tx_BegInt  <= '0';
        Tx_Ipg  <= conv_std_logic_vector( 42, 6);
    elsif    rising_edge( iClk )    then

        if    Sel_TxL = '1'    then
            if        iAddress(2 downto 1) = "00"                        then    Tx_On  <= iWritedata( 7);
            elsif    iAddress(2 downto 1) = "01" and iWritedata( 7) = '1'    then    Tx_On  <= '1';
            elsif    iAddress(2 downto 1) = "10" and iWritedata( 7) = '1'    then    Tx_On  <= '0';
            end if;
        end if;

        if        Tx_BegSet = '1'    and Tx_Ie = '1'                                 then    Tx_BegInt  <= '1';
        elsif    Sel_TxL = '1' and iAddress(2 downto 1) = "01" and iWritedata( 6) = '1'    then    Tx_BegInt  <= '1';
        elsif    Sel_TxL = '1' and iAddress(2 downto 1) = "10" and iWritedata( 6) = '1'    then    Tx_BegInt  <= '0';
        end if;

        onTxBegIrq <= not Tx_BegInt;

        if    Sel_TxL = '1' and iAddress(2 downto 1) = "11"                then    Tx_Desc_One <= iWritedata(Tx_Desc_One'range);
        elsif    Dsm = sStat and Ext_Tx = '0'     then
            if        Last_Desc = '1'                                    then    Tx_Desc_One <= (others => '0');
            else                                                            Tx_Desc_One <= Tx_Desc + 1;
            end if;
        end if;

        if        Sel_TxH = '1'    then
            if        iAddress(2 downto 1) = "00"                        then    Tx_Ie  <= iWritedata(15);
            elsif    iAddress(2 downto 1) = "01" and iWritedata(15) = '1'    then    Tx_Ie  <= '1';
            elsif    iAddress(2 downto 1) = "10" and iWritedata(15) = '1'    then    Tx_Ie  <= '0';
            end if;
        end if;

        if        Sel_TxH = '1'    then
            if        iAddress(2 downto 1) = "00"                        then    Tx_Half  <= iWritedata(13);
            elsif    iAddress(2 downto 1) = "01" and iWritedata(13) = '1'    then    Tx_Half  <= '1';
            elsif    iAddress(2 downto 1) = "10" and iWritedata(13) = '1'    then    Tx_Half  <= '0';
            end if;
        end if;

        if        Sel_TxH = '1'    then
            if        iAddress(2 downto 1) = "00"                        then    Tx_Wait  <= iWritedata(12);
            elsif    iAddress(2 downto 1) = "01" and iWritedata(12) = '1'    then    Tx_Wait  <= '1';
            elsif    iAddress(2 downto 1) = "10" and iWritedata(12) = '1'    then    Tx_Wait  <= '0';
            end if;
        end if;

        if        Sel_TxH = '1'    then
            if        iAddress(2 downto 1) = "11" and iWritedata(14) = '1'    then    Tx_Ipg        <= iWritedata(13 downto 8);
            end if;
        end if;

        if        Tx_Ie = '1' and Dsm = sStat and Desc_We = '1'        then    TxInt <= '1';
        else                                                                TxInt <= '0';
        end if;

        if        Sel_TxH = '1' and iAddress(2 downto 1) = "10" and iWritedata(8) = '1'
            and    Tx_Icnt /= 0                                        then    Tx_Icnt <= Tx_Icnt - not TxInt;
        elsif    TxInt = '1'    and Tx_Icnt /= "11111"                    then    Tx_Icnt <= Tx_Icnt + 1;
        end if;

    end if;

end process pTxRegs;


end block bTxDesc;

end block b_Full_Tx;


b_Full_Rx:    block
    type tRxState is (
        sIdle,
        sSof,
        sRxd
    );

    signal Sm_Rx        : tRxState;
    signal Rx_Dat       : std_logic_vector(1 downto 0);
    signal Rx_DatL      : std_logic_vector(1 downto 0);
    signal Tx_Timer     : std_logic_vector(7 downto 0);
    signal Dibl_Cnt     : std_logic_vector(1 downto 0);
    signal Crc          : std_logic_vector(31 downto 0);
    signal nCrc         : std_logic_vector(31 downto 0);
    signal CrcDin       : std_logic_vector(1 downto 0);
    signal F_Err        : std_logic;
    signal P_Err        : std_logic;
    signal N_Err        : std_logic;
    signal A_Err        : std_logic;
    signal F_End        : std_logic;
    signal F_Val        : std_logic;
    signal Rx_Beg       : std_logic;
    signal Rx_Sr        : std_logic_vector(7 downto 0);
    signal nCrc_Ok      : std_logic;
    signal Crc_Ok       : std_logic;
    signal WrDescStat   : std_logic;
    signal PreCount     : std_logic_vector(4 downto 0);
    signal PreBeg       : std_logic;
    signal PreErr       : std_logic;
    signal Rx_DvL       : std_logic;
    signal Diag         : std_logic;

begin

    Rx_Beg <= '1' when    Rx_Dv = '1' and Sm_Rx = sSof and Rx_Dat = "11"    else '0';

    nCrc_Ok <= '1' when nCrc = x"C704DD7B"    else '0';

rxsm: process ( iClk, iRst )     is
begin

    if iRst = '1'    then
        Sm_Rx <= sIdle;
    elsif rising_edge( iClk )     then
        if    Sm_Rx = sIdle or Sm_Rx = sRxd or Sm_Rx = sSof or Dibl_Cnt = "11"    then
            case Sm_Rx    is
                when    sIdle =>    if        Rx_Dv = '1'        then    Sm_Rx    <= sSof;    end if;
                when    sSof =>    if        Rx_Dat = "11"    then    Sm_Rx    <= sRxd;
                                    elsif    Rx_Dv = '0'        then    Sm_Rx    <= sIdle;    end if;
                when    sRxd =>    if        Rx_Dv = '0'        then    Sm_Rx    <= sIdle;    end if;
                when  others  =>    NULL;
            end case;
        end if;
    end if;

end process rxsm;

pRxCtl: process ( iClk, iRst )     is
    variable vPreload   :    std_logic_vector(Tx_Timer'range);
    variable vLoad      :    std_logic;
begin

    if iRst = '1'    then
        Rx_DatL <= "00"; Rx_Dat <= "00"; Rx_Dv <= '0'; Dibl_Cnt <= "00"; PreCount <= (others => '0');
        F_End <= '0'; F_Err <= '0';  F_Val <= '0'; Crc_Ok <= '0';
        A_Err <= '0'; N_Err <= '0'; P_Err <= '0'; PreBeg <= '0'; PreErr <= '0';
    elsif rising_edge( iClk )     then

        Rx_DatL <= iRxData;

        Rx_Dat <= Rx_DatL;

        if        Rx_Dv = '0' and iRxDv = '1'                        then    Rx_Dv <= '1';
        elsif    Rx_Dv = '1'    and iRxDv = '0' and Dibl_Cnt(0) = '1'    then    Rx_Dv <= '0';
        end  if;

        if        Rx_Beg = '1'    then    Dibl_Cnt <= "00";
        else                            Dibl_Cnt <= Dibl_Cnt + 1;
        end if;

        Crc_Ok <= nCrc_Ok;

        if        (Sm_Rx = sRxd and Rx_Dv = '0')            then    F_End <= '1';
                                                                F_Err <= not Crc_Ok;
        else                                                    F_End <= '0';
        end if;

        if        Dibl_Cnt = "11" and Sm_Rx = sRxd        then    F_Val <= '1';
        else                                                    F_Val <= '0';
        end if;

        if        WrDescStat = '1'                        then    A_Err <= '0';
        elsif    F_End = '1' and Dibl_Cnt /= 1            then    A_Err <= '1';
        end if;

        if        Rx_Dv = '0' or Rx_Dat(0) = '0'            then    PreCount <= (others => '1');
        else                                                    PreCount <= PreCount - 1;
        end if;

        if        Rx_Dv  = '0'    then    PreBeg <= '0';
        elsif    Rx_Dat = "01"    then    PreBeg <= '1';
        end if;

        if        WrDescStat = '1'                                    then    N_Err <= '0';
        elsif    Sm_Rx = sSof and Rx_Dv  = '0'                        then    N_Err <= '1';
        end if;

        if        Rx_DvL = '0'                                        then    PreErr <= '0';
        elsif    PreBeg = '0' and (Rx_Dat = "10" or Rx_Dat = "11")    then    PreErr <= '1';
        elsif    PreBeg = '1' and (Rx_Dat = "10" or Rx_Dat = "00")    then    PreErr <= '1';
        end if;

        if        WrDescStat = '1'                then    P_Err <= '0';
        elsif    Rx_Beg = '1' and PreErr = '1'    then    P_Err <= '1';
        elsif    Rx_Beg = '1' and PreCount /= 0    then    P_Err <= '1';
        end if;

        Rx_Sr <= Rx_Dat(1) & Rx_Dat(0) & Rx_Sr(7 downto 2);

        Rx_DvL  <= Rx_Dv;

    end if;

end process pRxCtl;

    CrcDin <= Rx_Dat;

Calc: process ( iClk, Crc, nCrc, CrcDin, Sm_Rx )     is
    variable H : std_logic_vector(1 downto 0);
begin

    H(0) := Crc(31) xor CrcDin(0);
    H(1) := Crc(30) xor CrcDin(1);

    if        Sm_Rx = sSof     then    nCrc <= x"FFFFFFFF";
    else
        nCrc( 0) <=                         H(1);
        nCrc( 1) <=             H(0) xor H(1);
        nCrc( 2) <= Crc( 0) xor H(0) xor H(1);
        nCrc( 3) <= Crc( 1) xor H(0)         ;
        nCrc( 4) <= Crc( 2)          xor H(1);
        nCrc( 5) <= Crc( 3) xor H(0) xor H(1);
        nCrc( 6) <= Crc( 4) xor H(0)         ;
        nCrc( 7) <= Crc( 5)          xor H(1);
        nCrc( 8) <= Crc( 6) xor H(0) xor H(1);
        nCrc( 9) <= Crc( 7) xor H(0)         ;
        nCrc(10) <= Crc( 8)          xor H(1);
        nCrc(11) <= Crc( 9) xor H(0) xor H(1);
        nCrc(12) <= Crc(10) xor H(0) xor H(1);
        nCrc(13) <= Crc(11) xor H(0)         ;
        nCrc(14) <= Crc(12)                  ;
        nCrc(15) <= Crc(13)                  ;
        nCrc(16) <= Crc(14)          xor H(1);
        nCrc(17) <= Crc(15) xor H(0)         ;
        nCrc(18) <= Crc(16)                  ;
        nCrc(19) <= Crc(17)                  ;
        nCrc(20) <= Crc(18)                  ;
        nCrc(21) <= Crc(19)                  ;
        nCrc(22) <= Crc(20)          xor H(1);
        nCrc(23) <= Crc(21) xor H(0) xor H(1);
        nCrc(24) <= Crc(22) xor H(0)         ;
        nCrc(25) <= Crc(23)                  ;
        nCrc(26) <= Crc(24)          xor H(1);
        nCrc(27) <= Crc(25) xor H(0)         ;
        nCrc(28) <= Crc(26)                  ;
        nCrc(29) <= Crc(27)                  ;
        nCrc(30) <= Crc(28)                  ;
        nCrc(31) <= Crc(29)                   ;
    end if;

    if rising_edge( iClk )  then
        Crc <= nCrc;
    end if;

end process Calc;

bRxDesc:    block
    type tDescState is (
        sIdle,
        sLen,
        sTimL,
        sTimH,
        sAdrH,
        sAdrL,
        sData,
        sOdd,
        sStat,
        sLenW
    );

    signal Dsm          : tDescState;
    signal Rx_Dsm_Next  : tDescState;
    signal Rx_Buf       : std_logic_vector(7 downto 0);
    signal Rx_LatchH    : std_logic_vector(7 downto 0);
    signal Rx_LatchL    : std_logic_vector(7 downto 0);
    signal Rx_Ovr       : std_logic;
    signal DescRam_Out  : std_logic_vector(15 downto 0);
    signal DescRam_In   : std_logic_vector(15 downto 0);
    alias  RX_LEN       : std_logic_vector(11 downto 0) is DescRam_Out(11 downto 0);
    alias  RX_OWN       : std_logic                     is DescRam_Out(8);
    alias  RX_LAST      : std_logic                     is DescRam_Out(9);
    signal Ram_Be       : std_logic_vector(1 downto 0);
    signal Ram_Wr       : std_logic;
    signal Desc_We      : std_logic;
    signal ZeitL        : std_logic_vector(15 downto 0);
    signal Rx_On        : std_logic;
    signal Rx_Ie        : std_logic;
    signal Sel_RxH      : std_logic;
    signal Sel_RxL      : std_logic;
    signal Rx_Desc      : std_logic_vector(4 downto 0);
    signal DescIdx      : std_logic_vector(2 downto 0);
    signal Desc_Addr    : std_logic_vector(Rx_Desc'length + DescIdx'length - 1 downto 0);
    signal Match_Desc   : std_logic_vector(3 downto 0); -- The matching filter index
    signal Rx_Icnt      : std_logic_vector(4 downto 0);
    signal Rx_Lost      : std_logic;
    signal Last_Desc    : std_logic;
    signal Answer_Tx    : std_logic;
    signal Rx_Count     : std_logic_vector(11 downto 0);
    signal Rx_Limit     : std_logic_vector(11 downto 0);
    signal Match        : std_logic;
    signal Filt_Cmp     : std_logic;
    signal Rx_Idle      : std_logic;
    signal RxInt        : std_logic;
    signal Hub_Rx_L     : std_logic_vector(1 downto 0);
    signal Rx_Dma_Out   : std_logic;
    signal Rx_Done      : std_logic;

begin

    process(iRst, iClk)
        variable doPulse : std_logic;
    begin
        if iRst = cActivated then
            Rx_Done <= cInactivated;
            doPulse := cInactivated;
        elsif rising_edge(iClk) then
            Rx_Done <= cInactivated;

            if Dsm /= sIdle and Rx_Dsm_Next = sIdle then
                -- RX is done
                doPulse := cActivated;
            end if;

            if doPulse = cActivated and Rx_Dma_Req = cInactivated and Rx_Count = 0 then
                -- RX is done and there is no dma request
                Rx_Done <= cActivated;
                doPulse := cInactivated;
            end if;

        end if;
    end process;

    oDmaWriteDone <= Rx_Done;

    WrDescStat <= '1' when Dsm = sStat    else '0';

    Ram_Wr    <= '1' when    inWrite = '0' and iSelectRam = '1' and iAddress(10 downto 9) = "10"    else '0';
    Ram_Be(1) <= '1' when    inWrite = '1' or inByteenable(1) = '0'                        else '0';
    Ram_Be(0) <= '1' when    inWrite = '1' or inByteenable(0) = '0'                        else '0';

    DescIdx <=    "001"    when    Desc_We = '0' and (Rx_Dsm_Next = sLen or Rx_Dsm_Next = sLenW)    else
                "001"    when    Desc_We = '1' and (Dsm         = sLen or Dsm         = sLenW)    else
                "010"    when    Desc_We = '0' and Rx_Dsm_Next = sAdrH                            else
                "010"    when    Desc_We = '1' and Dsm         = sAdrH                            else
                "011"    when    Desc_We = '0' and Rx_Dsm_Next = sAdrL                            else
                "011"    when    Desc_We = '1' and Dsm         = sAdrL                            else
                "110"    when    Desc_We = '0' and Rx_Dsm_Next = sTimH                            else
                "110"    when    Desc_We = '1' and Dsm         = sTimH                            else
                "111"    when    Desc_We = '0' and Rx_Dsm_Next = sTimL                            else
                "111"    when    Desc_We = '1' and Dsm         = sTimL                            else
                "000";

    Desc_We <= '1'    when   Dsm = sTimL or Dsm = sTimH                    else
               '1'    when  (Dsm = sLenW or Dsm = sStat) and Match = '1'  else    '0';

    Desc_Addr <= Rx_Desc & DescIdx;

gRxTime:    if gTimerEnable generate
    DescRam_In <= Zeit(15 downto 0)                when    Dsm = sTimH        else
                  ZeitL                            when    Dsm = sTimL        else
                  x"0"  & Rx_Count                when    Dsm = sLenW        else
                  Rx_Dma_Out & '0' & "0" & A_Err & Hub_Rx_L & "00" & Match_Desc & N_Err & P_Err & Rx_Ovr & F_Err;
end generate;

ngRxTime:    if not gTimerEnable generate
    DescRam_In <= x"0"  & Rx_Count                when    Dsm = sLenW                    else
                  Rx_Dma_Out & '0' & "0" & A_Err & Hub_Rx_L & "00" & Match_Desc & N_Err & P_Err & Rx_Ovr & F_Err;
end generate;

    --! This DPRAM holds the Rx descriptor accessible by the host and the DMA.
    RXRAM : entity work.dpRamOpenmac
        generic map (
            gWordWidth      => iWritedata'length,
            gNumberOfWords  => 256,
            gInitFile       => "UNUSED"
        )
        port map (
            iClk_A          => iClk,
            iEnable_A       => cActivated,
            iWriteEnable_A  => Ram_Wr,
            iAddress_A      => iAddress(8 downto 1),
            iByteenable_A   => Ram_Be,
            iWritedata_A    => iWritedata,
            oReaddata_A     => Rx_Ram_Dat,
            iClk_B          => iClk,
            iEnable_B       => cActivated,
            iWriteEnable_B  => Desc_We,
            iByteenable_B   => (others => cActivated),
            iAddress_B      => Desc_Addr,
            iWritedata_B    => DescRam_In,
            oReaddata_B     => DescRam_Out
        );

pRxSm: process( Dsm,
                Rx_Beg, Rx_On, RX_OWN, F_End, F_Err, Diag, Rx_Count )
begin

        Rx_Dsm_Next <= Dsm;
        case    Dsm is
            when sIdle     =>    if    Rx_Beg = '1' and Rx_On = '1' and RX_OWN = '1' then
                                                        Rx_Dsm_Next <= sLen;
                            end if;
            when sLen     =>                                Rx_Dsm_Next <= sAdrH;
            when sAdrH     =>                                Rx_Dsm_Next <= sAdrL;
            when sAdrL     =>                                Rx_Dsm_Next <= sTimH;
            when sTimH     =>                                Rx_Dsm_Next <= sTimL;
            when sTimL     =>                                Rx_Dsm_Next <= sData;
            when sData     =>    if    F_End = '1'    then
                                if    F_Err = '0'
                                 or Diag  = '1'    then    Rx_Dsm_Next <= sStat;
                                else                    Rx_Dsm_Next <= sIdle;
                                end if;
                            end if;
            when sStat     =>                                Rx_Dsm_Next <= sLenW;
            when sLenW   =>    if    Rx_Count(0) = '0' then
                                                        Rx_Dsm_Next <= sIdle;
                            else                        Rx_Dsm_Next <= sOdd;
                            end if;
            when sOdd   =>                                Rx_Dsm_Next <= sIdle;
            when others     =>
        end case;
end process pRxSm;

    pRxSmClk : process(iRst, iClk)
    begin
        if iRst = cActivated then
            Dsm <= sIdle;
        elsif rising_edge(iClk) then
            Dsm <= Rx_Dsm_Next;
        end if;
    end process pRxSmClk;

pRxControl: process( iRst, iClk )
begin

    if    iRst = '1'    then
        Rx_Ovr <= '0'; Rx_Dma_Req  <= '0'; Last_Desc <= '0'; Rx_Dma_Out <= '0';
        Rx_Count <= (others => '0');
        Rx_Buf <= (others => '0'); Rx_LatchL <= (others => '0'); Rx_LatchH <= (others => '0');
        Dma_Rx_Addr <= (others => '0');
    elsif    rising_edge( iClk )     then

        if    gTimerEnable    then
            if        Dsm  = sTimH    then    ZeitL <= Zeit(31 downto 16);
            end if;
        end if;

        if        Dsm = sIdle        then    Rx_Count  <= (others => '0');
                                        Last_Desc <= RX_LAST;
        elsif   F_Val = '1'        then    Rx_Count  <= Rx_Count + 1;
        end if;

        if        Dsm = sLen        then    Rx_Limit      <= RX_LEN;
                                        Hub_Rx_L      <= iHubRxPort;
        end if;

        if    F_Val = '1'        then    Rx_Buf <= Rx_Sr;
        end if;

        if    (F_Val = '1' and Rx_Count(0) = '1') or    Dsm = sStat     then    Rx_LatchH <= Rx_Buf;
                                                                        Rx_LatchL <= Rx_Sr;
            if        Rx_Dma_Req = '1' and Sm_Rx /= sIdle            then    Rx_Dma_Out <= '1';
            end if;
        elsif    Dsm = sLen                                        then    Rx_Dma_Out <= '0';
        end if;

        if        Dsm = sLen                                then    Rx_Ovr <= '0';
        elsif    F_Val = '1' and Rx_Limit = Rx_Count        then    Rx_Ovr <= '1';
        end if;


        if        Dsm = sAdrL         then    --Dma_Rx_Addr(15 downto 1) <= DescRam_Out(15 downto 1);
            Dma_Rx_Addr(oDmaAddress'high downto 16) <= DescRam_Out(oDmaAddress'high-16 downto 0);
        elsif    Rx_Dma_Ack = '1' then    Dma_Rx_Addr(15 downto 1) <= Dma_Rx_Addr(15 downto 1) + 1;
        end if;

        if        Dsm = sAdrH         then    Dma_Rx_Addr(15 downto 1) <= DescRam_Out(15 downto 1);
                --Dma_Rx_Addr(oDmaAddress'high downto 16) <= DescRam_Out(oDmaAddress'high-16 downto 0);
        elsif    Rx_Dma_Ack = '1' and Dma_Rx_Addr(15 downto 1) = x"FFF" & "111"    then
                Dma_Rx_Addr(oDmaAddress'high downto 16) <= Dma_Rx_Addr(oDmaAddress'high downto 16) + 1;
        end if;

        if        Filt_Cmp = '0' and Match ='0'                                        then    Rx_Dma_Req  <= '0';

        elsif  (Dsm = sOdd  and Rx_Ovr = '0')
            or (Dsm = sData and Rx_Ovr = '0' and F_Val = '1' and Rx_Count(0) = '1')    then    Rx_Dma_Req  <= '1';
        elsif    Rx_Dma_Ack = '1'                                                    then    Rx_Dma_Req  <= '0';
        end if;

    end if;

end process pRxControl;

    oDmaWritedata <= Rx_LatchL & Rx_LatchH; --Rx_LatchH & Rx_LatchL;

    onRxIrq <= '1'    when    Rx_Icnt = 0 or Rx_Ie = '0'         else    '0';

    Rx_Idle <= '1'    when    Sm_Rx = sIdle else '0';

    Rx_Reg <= Rx_Ie & "000" & (Rx_Icnt(4) or Rx_Icnt(3)) & Rx_Icnt(2 downto 0) &
              Rx_On & Rx_Lost & Rx_Idle & Rx_Desc;

bFilter: block
    signal Ram_Addr     : std_logic_vector(7 downto 0);
    signal Ram_BeH      : std_logic_vector(1 downto 0);
    signal Ram_BeL      : std_logic_vector(1 downto 0);
    signal Ram_Wr       : std_logic;
    signal Filter_Addr  : std_logic_vector(6 downto 0);
    signal Filter_Out_H : std_logic_vector(31 downto 0);
    signal Filter_Out_L : std_logic_vector(31 downto 0);
    alias  DIRON_0      : std_logic is Filter_Out_H(11);
    alias  DIRON_1      : std_logic is Filter_Out_H(27);
    alias  DIRON_2      : std_logic is Filter_Out_L(11);
    alias  DIRON_3      : std_logic is Filter_Out_L(27);
    alias  TX_0         : std_logic is Filter_Out_H(7);
    alias  TX_1         : std_logic is Filter_Out_H(23);
    alias  TX_2         : std_logic is Filter_Out_L(7);
    alias  TX_3         : std_logic is Filter_Out_L(23);
    alias  ON_0         : std_logic is Filter_Out_H(6);
    alias  ON_1         : std_logic is Filter_Out_H(22);
    alias  ON_2         : std_logic is Filter_Out_L(6);
    alias  ON_3         : std_logic is Filter_Out_L(22);
    alias  DESC_0       : std_logic_vector(Auto_Desc'range) is Filter_Out_H(Auto_Desc'left downto Auto_Desc'right);
    alias  DESC_1       : std_logic_vector(Auto_Desc'range) is Filter_Out_H(Auto_Desc'left+16 downto Auto_Desc'right+16); --(19 downto 16);
    alias  DESC_2       : std_logic_vector(Auto_Desc'range) is Filter_Out_L(Auto_Desc'left downto Auto_Desc'right);
    alias  DESC_3       : std_logic_vector(Auto_Desc'range) is Filter_Out_L(Auto_Desc'left+16 downto Auto_Desc'right+16); --(19 downto 16);
    signal Byte_Cnt     : std_logic_vector(4 downto 0) := (others => '0');
    signal Erg0         : std_logic_vector(7 downto 0);
    signal Erg1         : std_logic_vector(7 downto 0);
    signal Erg2         : std_logic_vector(7 downto 0);
    signal Erg3         : std_logic_vector(7 downto 0);
    signal Mat_Reg      : std_logic_vector(15 downto 0);
    signal Filt_Idx     : std_logic_vector(1 downto 0);
    signal Mat_Sel      : std_logic_vector(3 downto 0);
    signal M_Prio       : std_logic_vector(2 downto 0);
    alias  Found        : std_logic is M_Prio(2);
begin
    Ram_Addr   <= iAddress(9 downto 8) & iAddress(5 downto 1) & iAddress(6);

    Ram_Wr     <= '1' when inWrite = '0' and iSelectRam = '1'  and iAddress(10) = '0' else '0';
    Ram_BeH(1) <= '1' when inWrite = '1' or (inByteenable(1) = '0' and iAddress(7) = '0') else '0';
    Ram_BeH(0) <= '1' when inWrite = '1' or (inByteenable(0) = '0' and iAddress(7) = '0') else '0';
    Ram_BeL(1) <= '1' when inWrite = '1' or (inByteenable(1) = '0' and iAddress(7) = '1') else '0';
    Ram_BeL(0) <= '1' when inWrite = '1' or (inByteenable(0) = '0' and iAddress(7) = '1') else '0';

    Filter_Addr <= Dibl_Cnt & Byte_Cnt;

    --! This simplex DPRAM holds the higher dword for the Rx packet filters.
    FILTERRAMHIGH : entity work.dpRamSplx
        generic map (
            gWordWidthA         => iWritedata'length,
            gByteenableWidthA   => Ram_BeH'length,
            gNumberOfWordsA     => 256,
            gWordWidthB         => Filter_Out_H'length,
            gNumberOfWordsB     => 128,
            gInitFile           => "UNUSED"
        )
        port map (
            iClk_A          => iClk,
            iEnable_A       => cActivated,
            iWriteEnable_A  => Ram_Wr,
            iAddress_A      => Ram_Addr,
            iByteenable_A   => Ram_BeH,
            iWritedata_A    => iWritedata,
            iClk_B          => iClk,
            iEnable_B       => cActivated,
            iAddress_B      => Filter_Addr,
            oReaddata_B     => Filter_Out_H
        );

    --! This simplex DPRAM holds the lower dword for the Rx packet filters.
    FILTERRAMLOW : entity work.dpRamSplx
        generic map (
            gWordWidthA         => iWritedata'length,
            gByteenableWidthA   => Ram_BeL'length,
            gNumberOfWordsA     => 256,
            gWordWidthB         => Filter_Out_H'length,
            gNumberOfWordsB     => 128,
            gInitFile           => "UNUSED"
        )
        port map (
            iClk_A          => iClk,
            iEnable_A       => cActivated,
            iWriteEnable_A  => Ram_Wr,
            iAddress_A      => Ram_Addr,
            iByteenable_A   => Ram_BeL,
            iWritedata_A    => iWritedata,
            iClk_B          => iClk,
            iEnable_B       => cActivated,
            iAddress_B      => Filter_Addr,
            oReaddata_B     => Filter_Out_L
        );

    Erg0 <= (Rx_Buf xor Filter_Out_H( 7 downto  0)) and Filter_Out_H(15 downto  8);
     Erg1 <= (Rx_Buf xor Filter_Out_H(23 downto 16)) and Filter_Out_H(31 downto 24);
    Erg2 <= (Rx_Buf xor Filter_Out_L( 7 downto  0)) and Filter_Out_L(15 downto  8);
    Erg3 <= (Rx_Buf xor Filter_Out_L(23 downto 16)) and Filter_Out_L(31 downto 24);

genMatSel:    for i in 0 to 3 generate
    Mat_Sel(i) <=    Mat_Reg( 0 + i)    when  Filt_Idx = "00"    else
                    Mat_Reg( 4 + i)    when  Filt_Idx = "01"    else
                    Mat_Reg( 8 + i)    when  Filt_Idx = "10"    else
                    Mat_Reg(12 + i); --    when  Filt_Idx = "11";
end generate;

    M_Prio <= "000" when    Filt_Cmp = '0' or Match = '1'                            else
              "100"    when    Mat_Sel(0) = '1'  and On_0 = '1' and (DIRON_0 = '0')    else
              "101"    when    Mat_Sel(1) = '1'  and On_1 = '1' and (DIRON_1 = '0')    else
              "110"    when    Mat_Sel(2) = '1'  and On_2 = '1' and (DIRON_2 = '0')    else
              "111"    when    Mat_Sel(3) = '1'  and On_3 = '1' and (DIRON_3 = '0')    else
              "000";

pFilter: process( iRst, iClk )
begin

    if    iRst = '1'    then
        Filt_Idx <= "00"; Match <= '0';
        Filt_Cmp <= '0'; Mat_Reg <= (others => '0'); Byte_Cnt <= (others =>'0');
        Match_Desc <= (others => '0');Auto_Desc <= (others =>'0'); Answer_Tx <= '0';
    elsif    rising_edge( iClk )     then

        Filt_Idx  <= Dibl_Cnt;

        if        Dibl_Cnt = "11"    and Rx_Count(5) = '0'    then    Byte_Cnt <= Rx_Count(Byte_Cnt'range);
        end if;

        if        Dsm = sTiml                                            then    Filt_Cmp  <= '1';
        elsif    Rx_Dv = '0'    or (F_Val = '1' and Rx_Count(5) = '1')    then    Filt_Cmp  <= '0';
        end if;

        if        Dsm = sTimL        then    Mat_Reg <= (others => '1');
        else
            for i in 0 to 3 loop
                if    Erg0 /= 0 and conv_integer(Filt_Idx) = i    then    Mat_Reg(4*i + 0) <= '0';    end if;
                if    Erg1 /= 0 and conv_integer(Filt_Idx) = i    then    Mat_Reg(4*i + 1) <= '0';    end if;
                if    Erg2 /= 0 and conv_integer(Filt_Idx) = i    then    Mat_Reg(4*i + 2) <= '0';    end if;
                if    Erg3 /= 0 and conv_integer(Filt_Idx) = i    then    Mat_Reg(4*i + 3) <= '0';    end if;
            end loop;
        end if;

        if        Dsm = sTimL                        then    Match <= '0';
        elsif    Found = '1'                        then    Match <= '1';        Match_Desc <= Filt_Idx & M_Prio(1 downto 0);
            if       M_Prio(1 downto 0) = "00"    then    Answer_Tx <= TX_0;    Auto_Desc  <= DESC_0;
            elsif    M_Prio(1 downto 0) = "01"    then    Answer_Tx <= TX_1;    Auto_Desc  <= DESC_1;
            elsif    M_Prio(1 downto 0) = "10"    then    Answer_Tx <= TX_2;    Auto_Desc  <= DESC_2;
            elsif    M_Prio(1 downto 0) = "11"    then    Answer_Tx <= TX_3;    Auto_Desc  <= DESC_3;
            end if;
        elsif    F_End = '1'                        then    Answer_Tx <= '0';
        end if;

    end if;

end process pFilter;

    R_Req  <= Answer_Tx when F_End = '1' and F_Err = '0'      else '0';

end block  bFilter;


    Sel_RxH <= '1'    when inWrite = '0' and iSelectCont = '1' and iAddress(3) = '1' and    inByteenable(1) = '0'    else    '0';
    Sel_RxL <= '1'    when inWrite = '0' and iSelectCont = '1' and iAddress(3) = '1' and    inByteenable(0) = '0'    else    '0';

pRxRegs: process( iRst, iClk )
begin

    if    iRst = '1'    then
        Rx_Desc <= (others => '0');     Rx_On  <= '0';
        Rx_Ie   <= '0';    Rx_Lost <= '0';    Rx_Icnt <= (others => '0'); RxInt <= '0'; Diag  <= '0';
    elsif    rising_edge( iClk )    then

        if    Sel_RxH = '1'    then
            if        iAddress(2 downto 1) = "00"                        then    Rx_Ie  <= iWritedata(15);
            elsif    iAddress(2 downto 1) = "01" and iWritedata(15) = '1'    then    Rx_Ie  <= '1';
            elsif    iAddress(2 downto 1) = "10" and iWritedata(15) = '1'    then    Rx_Ie  <= '0';
            end if;
        end if;

        if    Sel_RxH = '1'    then
            if        iAddress(2 downto 1) = "00"                        then    Diag  <= iWritedata(12);
            elsif    iAddress(2 downto 1) = "01" and iWritedata(12) = '1'    then    Diag  <= '1';
            elsif    iAddress(2 downto 1) = "10" and iWritedata(12) = '1'    then    Diag  <= '0';
            end if;
        end if;

        if    Sel_RxL = '1'    then
            if        iAddress(2 downto 1) = "00"                        then    Rx_On  <= iWritedata( 7);
            elsif    iAddress(2 downto 1) = "01" and iWritedata( 7) = '1'    then    Rx_On  <= '1';
            elsif    iAddress(2 downto 1) = "10" and iWritedata( 7) = '1'    then    Rx_On  <= '0';
            end if;
        end if;

        if        Rx_Beg  = '1' and (RX_OWN = '0' or Rx_On = '0')                    then    Rx_Lost  <= '1';
        elsif    Sel_RxL = '1' and iAddress(2 downto 1) = "10" and iWritedata( 6) = '1'    then    Rx_Lost  <= '0';
        end if;

        if        Sel_RxL = '1' and iAddress(2 downto 1) = "11"            then    Rx_Desc <= iWritedata(Rx_Desc'range);
        elsif    Dsm = sLenW and Desc_We = '1'  then
            if        Last_Desc = '1'                                    then    Rx_Desc <= (others => '0');
            else                                                            Rx_Desc <= Rx_Desc + 1;
            end if;
        end if;

        if        Rx_Ie = '1' and Desc_We = '1' and Dsm = sStat        then    RxInt <= '1';
        else                                                                RxInt <= '0';
        end if;

        if        Sel_RxH = '1' and iAddress(2 downto 1) = "10" and iWritedata(8) = '1'
            and    Rx_Icnt /= 0                                        then    Rx_Icnt <= Rx_Icnt - not RxInt;
        elsif    RxInt = '1'    and Rx_Icnt /= "11111"                    then    Rx_Icnt <= Rx_Icnt + 1;
        end if;

    end if;

end process pRxRegs;

end block bRxDesc;

end block b_Full_Rx;

end architecture struct;