-------------------------------------------------------------------------------
-- OpenMAC
--
--       Copyright (C) 2009 B&R
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
-- Note: Used DPR is specific to Altera/Xilinx. Use one of the following files:
--       OpenMAC_DPR_Altera.vhd
--       OpenMAC_DPR_Xilinx.vhd
--
-------------------------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.std_logic_arith.ALL;
USE ieee.std_logic_unsigned.ALL;

ENTITY OpenMAC IS
    GENERIC( HighAdr                : IN    integer := 16;
             Timer                  : IN    boolean := false;
             TxSyncOn                : IN    boolean := false;
             TxDel                    : IN    boolean := false;
             Simulate               : IN    boolean := false
           );
    PORT ( Rst, Clk                    : IN    std_logic;
           -- Processor
           s_nWr, Sel_Ram, Sel_Cont    : IN    std_logic := '0';
           S_nBe                    : IN    std_logic_vector( 1 DOWNTO 0);
           S_Adr                    : IN    std_logic_vector(10 DOWNTO 1);
           S_Din                    : IN    std_logic_vector(15 DOWNTO 0);
           S_Dout                   : OUT   std_logic_vector(15 DOWNTO 0);
           nTx_Int, nRx_Int            : OUT   std_logic;
           nTx_BegInt               : OUT   std_logic;
           -- DMA
           Dma_Rd_Done                : OUT    std_logic;
           Dma_Wr_Done                : OUT    std_logic;
           Dma_Req, Dma_Rw          : OUT   std_logic;
           Dma_Ack                  : IN    std_logic;
           Dma_Req_Overflow            : OUT    std_logic;
           Dma_Rd_Len               : OUT   std_logic_vector(11 downto 0);
           Dma_Addr                 : OUT   std_logic_vector(HighAdr DOWNTO 1);
           Dma_Dout                 : OUT   std_logic_vector(15 DOWNTO 0);
           Dma_Din                  : IN    std_logic_vector(15 DOWNTO 0);
           -- RMII
           rRx_Dat                  : IN    std_logic_vector( 1 DOWNTO 0);
           rCrs_Dv                  : IN    std_logic;
           rTx_Dat                  : OUT   std_logic_vector( 1 DOWNTO 0);
           rTx_En                   : OUT   std_logic;
           Hub_Rx                   : IN    std_logic_vector( 1 DOWNTO 0) := "00";
           Mac_Zeit                    : OUT   std_logic_vector(31 DOWNTO 0)
         );
END ENTITY OpenMAC;

ARCHITECTURE struct OF OpenMAC IS
    CONSTANT    cInactivated            : std_logic := '0';
    CONSTANT    cActivated                : std_logic := '1';
    SIGNAL    Rx_Dv                        : std_logic;
    SIGNAL    R_Req                        : std_logic;
    SIGNAL    Auto_Desc                    : std_logic_vector( 3 DOWNTO 0);
    SIGNAL    Zeit                        : std_logic_vector(31 DOWNTO 0);
    SIGNAL    Tx_Dma_Req,  Rx_Dma_Req        : std_logic;
    SIGNAL    Tx_Dma_Ack,  Rx_Dma_Ack        : std_logic;
    SIGNAL    Tx_Ram_Dat,  Rx_Ram_Dat        : std_logic_vector(15 DOWNTO 0);
    SIGNAL  Tx_Dma_Len                  : std_logic_vector(11 DOWNTO 0);
    SIGNAL    Tx_Reg,         Rx_Reg            : std_logic_vector(15 DOWNTO 0);
    SIGNAL    Dma_Tx_Addr, Dma_Rx_Addr    : std_logic_vector(Dma_Addr'RANGE);
    SIGNAL  Dma_Req_s, Dma_Rw_s         : std_logic;
    SIGNAL  halfDuplex                  : std_logic; -- cActivated ... MAC in half-duplex mode
    SIGNAL  Tx_Active                   : std_logic; -- cActivated ... TX = Data or CRC
    SIGNAL  Tx_Dma_Very1stOverflow      : std_logic; -- cActivated ... very first TX DMA overflow
    SIGNAL    Tx_Col                        : std_logic;
    SIGNAL    Sel_Tx_Ram, Sel_Tx_Reg        : std_logic;
    SIGNAL    Tx_LatchH, Tx_LatchL        : std_logic_vector( 7 DOWNTO 0);

BEGIN

    S_Dout <=    Tx_Ram_Dat    WHEN    Sel_Ram  = '1' AND Sel_Tx_Ram = '1'        ELSE
                Rx_Ram_Dat    WHEN    Sel_Ram  = '1'                            ELSE
                Tx_Reg        WHEN    Sel_Cont = '1' AND Sel_Tx_Reg = '1'        ELSE
                Rx_Reg;

    Mac_Zeit <= Zeit;

    Dma_Rd_Len <= Tx_Dma_Len + 4;

b_DmaObserver : block
    signal dmaObserverCounter, dmaObserverCounterNext : std_logic_vector(2 downto 0);
    constant cDmaObserverCounterHalf : std_logic_vector(dmaObserverCounter'range) := "110"; --every 8th cycle
    constant cDmaObserverCounterFull : std_logic_vector(dmaObserverCounter'range) := "010"; --every 4th cycle
begin

    process(Clk, Rst)
    begin
        if Rst = '1' then
            dmaObserverCounter <= (others => cInactivated);
        elsif rising_edge(Clk) then
            dmaObserverCounter <= dmaObserverCounterNext;
        end if;
    end process;

    Dma_Req_Overflow <= --very first TX Dma transfer
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
                              (others => cInactivated); --reset DmaObserverCounter if no Dma_Req

end block;

b_Dma:    BLOCK
    SIGNAL    Rx_Dma, Tx_Dma    : std_logic;
BEGIN
    Dma_Req <= Dma_Req_s;
    Dma_Req_s  <= '1'    WHEN    (Tx_Dma_Req = '1' AND Tx_Dma_Ack = '0') OR Rx_Dma_Req = '1'        ELSE '0';

    Dma_Rw <= Dma_Rw_s;
    Dma_Rw_s   <= '1'            WHEN   (Rx_Dma = '0' AND Tx_Dma_Req = '1' AND Tx_Dma_Ack = '0') OR Tx_Dma = '1'     ELSE '0';
    Dma_Addr   <= Dma_Tx_Addr    WHEN   (Rx_Dma = '0' AND Tx_Dma_Req = '1' AND Tx_Dma_Ack = '0') OR Tx_Dma = '1'     ELSE Dma_Rx_Addr;

    Rx_Dma_Ack <= '1'    WHEN    Rx_Dma = '1' AND Dma_Ack = '1'    ELSE '0';

pDmaArb: PROCESS( Clk, Rst )     IS
BEGIN

    IF Rst = '1'    THEN
        Rx_Dma <= '0'; Tx_Dma <= '0'; Tx_Dma_Ack <= '0';
        Tx_LatchH <= (OTHERS => '0'); Tx_LatchL <= (OTHERS => '0');
        Zeit <= (OTHERS => '0');
    ELSIF rising_edge( Clk )    THEN

        IF Timer THEN
            Zeit <= Zeit + 1;
        END IF;

        Sel_Tx_Ram  <= s_Adr(8);
        Sel_Tx_Reg  <= NOT s_Adr(3);

        IF        Dma_Ack = '0'    THEN
            IF        Rx_Dma = '0' AND Tx_Dma_Req = '1' AND Tx_Dma_Ack = '0'    THEN    Tx_Dma <= '1';
            ELSIF    Tx_Dma = '0' AND Rx_Dma_Req = '1'                        THEN    Rx_Dma <= '1';
            END IF;
        ELSE
            IF        Rx_Dma = '1' AND Tx_Dma_Req = '1' AND Tx_Dma_Ack = '0'     THEN    Tx_Dma <= '1';    Rx_Dma <= '0';
            ELSIF    Tx_Dma = '1' AND Rx_Dma_Req = '1'                        THEN    Tx_Dma <= '0';    Rx_Dma <= '1';
            ELSE                                                                    Tx_Dma <= '0';    Rx_Dma <= '0';
            END IF;
        END IF;

        IF        Tx_Dma = '1' AND Dma_Ack = '1'    THEN    Tx_Dma_Ack <= '1';
        ELSE                                            Tx_Dma_Ack <= '0';
        END IF;

        IF    Tx_Dma_Ack = '1'        THEN    Tx_LatchL <= Dma_Din(15 DOWNTO 8);
                                            Tx_LatchH <= Dma_Din( 7 DOWNTO 0);
        END IF;

    END IF;

END PROCESS pDmaArb;

END BLOCK b_Dma;

b_Full_Tx:    BLOCK
    TYPE    MACTX_TYPE IS ( R_Idl, R_Bop, R_Pre, R_Txd, R_Crc, R_Col, R_Jam );
    SIGNAL    Sm_Tx                        : MACTX_TYPE;
    SIGNAL    Start_Tx, ClrCol, Tx_On        : std_logic;
    SIGNAL    Dibl_Cnt                    : std_logic_vector( 1 DOWNTO 0);
    SIGNAL    F_End, Was_Col, Block_Col    : std_logic;
    SIGNAL    Ipg_Cnt, Tx_Timer            : std_logic_vector( 7 DOWNTO 0);
     ALIAS    Ipg                            : std_logic IS Ipg_Cnt(7);
     ALIAS    Tx_Time                        : std_logic IS Tx_Timer(7);
    SIGNAL    Tx_Ipg                        : std_logic_vector( 5 DOWNTO 0);
    SIGNAL    Tx_Count                    : std_logic_vector(11 DOWNTO 0);
    SIGNAL    Tx_En, F_Val, Tx_Half        : std_logic;
    SIGNAL    Tx_Sr, F_TxB                : std_logic_vector( 7 DOWNTO 0);
    SIGNAL    Crc                            : std_logic_vector(31 DOWNTO 0);
    SIGNAL    CrcDin, Tx_Dat                : std_logic_vector( 1 DOWNTO 0);
     SIGNAL    Col_Cnt                        : std_logic_vector( 3 DOWNTO 0);
    SIGNAL    Auto_Coll                    : std_logic;
     SIGNAL    Rnd_Num                        : std_logic_vector( 9 DOWNTO 0);
     SIGNAL    Retry_Cnt                    : std_logic_vector( 9 DOWNTO 0);
    SIGNAL    Max_Retry                    : std_logic_vector( 3 DOWNTO 0);

BEGIN

    rTx_En  <= Tx_En;
    rTx_Dat <= Tx_Dat;

    halfDuplex <= Tx_Half;

    Tx_Active <= cActivated when Sm_Tx = R_Txd or Sm_Tx = R_Crc else cInactivated;

pTxSm: PROCESS ( Clk, Rst )     IS
BEGIN

    IF Rst = '1'    THEN
        Sm_Tx <= R_Idl;
    ELSIF rising_edge( Clk ) THEN
        IF    Sm_Tx = R_Idl OR Sm_Tx = R_Bop OR Dibl_Cnt = "11"     THEN
            CASE Sm_Tx    IS
                WHEN    R_Idl =>    IF    Start_Tx = '1'
                                    AND (Tx_Half = '0' OR Rx_Dv = '0')
                                    AND Ipg = '0'                        THEN    Sm_Tx <= R_Bop;    END IF;
                WHEN    R_Bop =>                                                Sm_Tx <= R_Pre;
                WHEN    R_Pre =>    IF        Tx_Time = '1'                THEN    Sm_Tx <= R_Txd;    END IF;
                WHEN    R_Txd =>    IF        Was_Col = '1'                THEN    Sm_Tx <= R_Col;
                                    ELSIF    Tx_Count = 0                THEN    Sm_Tx <= R_Crc;    END IF;
                WHEN    R_Col =>                                                Sm_Tx <= R_Jam;
                WHEN    R_Jam =>    IF        Tx_Time = '1'                THEN    Sm_Tx <= R_Idl;
                                    END IF;
                WHEN    R_Crc =>    IF        Was_Col = '1'                THEN    Sm_Tx <= R_Col;
                                    ELSIF    Tx_Time = '1'                THEN    Sm_Tx <= R_Idl;    END IF;
                WHEN    OTHERS  =>    NULL;
            END CASE;
        END IF;
    END IF;

END PROCESS pTxSm;


pTxCtl: PROCESS ( Clk, Rst )     IS
    VARIABLE    Preload        :    std_logic_vector(Tx_Timer'RANGE);
    VARIABLE    Load        :    std_logic;
BEGIN

    IF Rst = '1'    THEN
        Tx_Dat <= "00"; Tx_En <= '0'; Dibl_Cnt <= "00"; F_End <= '0'; F_Val <= '0'; Tx_Col  <= '0'; Was_Col <= '0'; Block_Col <= '0';
        Ipg_Cnt <= (OTHERS => '0'); Tx_Timer <= (OTHERS => '0'); Tx_Sr <= (OTHERS => '0');
    ELSIF rising_edge( Clk )     THEN

        IF        Sm_Tx = R_Bop        THEN    Dibl_Cnt <= "00";
        ELSE                                Dibl_Cnt <= Dibl_Cnt + 1;
        END IF;

        IF        Tx_En = '1'                        THEN    Ipg_Cnt <= "1"  & conv_std_logic_vector( 44, 7);
        ELSIF    Rx_Dv = '1' AND Tx_Half = '1'    THEN    Ipg_Cnt <= "10" & Tx_Ipg;
        ELSIF    Ipg = '1'                         THEN    Ipg_Cnt <= Ipg_Cnt - 1;
        END IF;

        IF        Dibl_Cnt = "11" AND Sm_Tx = R_Crc AND Tx_Time = '1'        THEN    F_End  <= '1';
        ELSIF    Dibl_Cnt = "11" AND Sm_Tx = R_Col THEN
            IF        Col_Cnt = (Max_Retry - 1)                            THEN    F_End  <= '1';
            ELSIF    Col_Cnt < x"E"                                        THEN    Tx_Col <= '1';
            ELSE                                                                F_End  <= '1';
            END IF;
        ELSE                                                                    F_End  <= '0';
                                                                                Tx_Col <= '0';
        END IF;

        IF         Tx_Half = '1' AND Rx_Dv = '1'
            AND (Sm_Tx = R_Pre OR Sm_Tx = R_Txd)    THEN    Was_Col <= '1';
        ELSIF     Sm_Tx = R_Col                        THEN    Was_Col <= '0';
        END IF;

        IF         Sm_Tx = R_Col                    THEN    Block_Col <= '1';
        ELSIF     Auto_Coll = '1'                THEN    Block_Col <= '0';
        ELSIF     Retry_Cnt = 0                    THEN    Block_Col <= '0';
        END IF;

        IF        Dibl_Cnt = "10" AND Sm_Tx = R_Pre AND Tx_Time = '1'    THEN    F_Val <= '1';
        ELSIF   Dibl_Cnt = "10" AND Sm_Tx = R_Txd                    THEN    F_Val <= '1';
        ELSE                                                                F_Val <= '0';
        END IF;

        Load := '0';
        IF        Sm_Tx = R_Bop        THEN    Preload := x"06";    Load := '1';
        ELSIF    Sm_Tx = R_Txd        THEN    Preload := x"02";    Load := '1';
        ELSIF    Sm_Tx = R_Col        THEN    Preload := x"01";    Load := '1';
        ELSIF    Tx_Time = '1'        THEN    Preload := x"3e";    Load := '1';
        END IF;

        IF        Dibl_Cnt = "11"    OR Sm_Tx = R_Bop     THEN
            IF        Load = '1'    THEN    Tx_Timer <= Preload;
            ELSE                        Tx_Timer <= Tx_Timer - 1;
            END IF;
        END IF;

        IF        F_Val = '1'        THEN    Tx_Sr <= F_TxB;
        ELSE                            Tx_Sr <= "00" & Tx_Sr(7 DOWNTO 2);
        END IF;

        IF        Sm_Tx = R_Pre                                        THEN    Tx_En <= '1';
        ELSIF    Sm_Tx = R_Idl OR (Sm_Tx = R_Jam AND Tx_Time = '1')    THEN    Tx_En <= '0';
        END IF;

        IF        Sm_Tx = R_Pre AND Tx_Time = '1' AND Dibl_Cnt = "11"    THEN    Tx_Dat <= "11";
        ELSIF    Sm_Tx = R_Pre                                        THEN    Tx_Dat <= "01";
        ELSIF    Sm_Tx = R_Txd                                        THEN    Tx_Dat <= CrcDin;
        ELSIF    Sm_Tx = R_Crc                                        THEN    Tx_Dat <= NOT Crc(30) & NOT Crc(31);
        ELSIF    Sm_Tx = R_Col OR Sm_Tx = R_Jam                        THEN    Tx_Dat <= "11";
        ELSE                                                                Tx_Dat <= "00";
        END IF;

    END IF;

END PROCESS pTxCtl;

pBackDel: PROCESS ( Clk, Rst )    IS
BEGIN
    IF Rst = '1'    THEN
        Rnd_Num   <= (OTHERS => '0');
        Col_Cnt   <= (OTHERS => '0');
        Retry_Cnt <= (OTHERS => '0');
    ELSIF rising_edge( Clk )     THEN

        Rnd_Num <= Rnd_Num(8 DOWNTO 0) & (Rnd_Num(9) XOR NOT Rnd_Num(2));

        IF        ClrCol = '1'                        THEN    Col_Cnt <= x"0";
        ELSIF    Dibl_Cnt = "11"    AND Sm_Tx = R_Col    THEN    Col_Cnt <= Col_Cnt + 1;
        END IF;

        IF    Dibl_Cnt = "11"    THEN
            IF        Tx_On = '0'    OR Auto_Coll = '1'        THEN    Retry_Cnt <= (OTHERS => '0');
            ELSIF    Sm_Tx = R_Col  THEN
                FOR i IN 0 TO 9 LOOP
                    IF     Col_Cnt >= i    THEN    Retry_Cnt(i) <= Rnd_Num(i);
                    ELSE                        Retry_Cnt(i) <= '0';
                    END IF;
                END LOOP;
            ELSIF Sm_Tx /= R_Jam AND Tx_Time = '1' AND Retry_Cnt /= 0    THEN    Retry_Cnt <= Retry_Cnt - 1;
            END IF;
        END IF;
    END IF;
END PROCESS pBackDel;


    CrcDin <= Tx_Sr(1 DOWNTO 0);

Calc: PROCESS ( Clk, Crc, CrcDin )     IS
    VARIABLE    H         : std_logic_vector(1 DOWNTO 0);
BEGIN

    H(0) := Crc(31) XOR CrcDin(0);
    H(1) := Crc(30) XOR CrcDin(1);

    IF rising_edge( Clk )  THEN
        IF        Sm_Tx = R_Pre        THEN    Crc <= x"FFFFFFFF";
        ELSIF     Sm_Tx = R_Crc        THEN    Crc <= Crc(29 DOWNTO 0) & "00";
        ELSE
            Crc( 0) <=                        H(1);
            Crc( 1) <=             H(0) XOR H(1);
            Crc( 2) <= Crc( 0) XOR H(0) XOR H(1);
            Crc( 3) <= Crc( 1) XOR H(0)         ;
            Crc( 4) <= Crc( 2)          XOR H(1);
            Crc( 5) <= Crc( 3) XOR H(0) XOR H(1);
            Crc( 6) <= Crc( 4) XOR H(0)         ;
            Crc( 7) <= Crc( 5)          XOR H(1);
            Crc( 8) <= Crc( 6) XOR H(0) XOR H(1);
            Crc( 9) <= Crc( 7) XOR H(0)         ;
            Crc(10) <= Crc( 8)          XOR H(1);
            Crc(11) <= Crc( 9) XOR H(0) XOR H(1);
            Crc(12) <= Crc(10) XOR H(0) XOR H(1);
            Crc(13) <= Crc(11) XOR H(0)         ;
            Crc(14) <= Crc(12)                  ;
            Crc(15) <= Crc(13)                  ;
            Crc(16) <= Crc(14)          XOR H(1);
            Crc(17) <= Crc(15) XOR H(0)         ;
            Crc(18) <= Crc(16)                  ;
            Crc(19) <= Crc(17)                  ;
            Crc(20) <= Crc(18)                  ;
            Crc(21) <= Crc(19)                  ;
            Crc(22) <= Crc(20)          XOR H(1);
            Crc(23) <= Crc(21) XOR H(0) XOR H(1);
            Crc(24) <= Crc(22) XOR H(0)         ;
            Crc(25) <= Crc(23)                  ;
            Crc(26) <= Crc(24)          XOR H(1);
            Crc(27) <= Crc(25) XOR H(0)         ;
            Crc(28) <= Crc(26)                  ;
            Crc(29) <= Crc(27)                  ;
            Crc(30) <= Crc(28)                  ;
            Crc(31) <= Crc(29)                  ;
        END IF;
    END IF;
END PROCESS Calc;

bTxDesc:    BLOCK
    TYPE    sDESC    IS    (sIdle, sLen, sTimL, sTimH, sAdrH, sAdrL, sReq, sBegL, sBegH, sDel, sData, sStat, sColl );
    SIGNAL    Dsm, Tx_Dsm_Next                    : sDESC;
    SIGNAL    DescRam_Out, DescRam_In                : std_logic_vector(15 DOWNTO 0);
     ALIAS    TX_LEN                                : std_logic_vector(11 DOWNTO 0)    IS    DescRam_Out(11 DOWNTO 0);
     ALIAS    TX_OWN                                : std_logic                        IS    DescRam_Out( 8);
     ALIAS    TX_LAST                                : std_logic                        IS    DescRam_Out( 9);
     ALIAS    TX_READY                            : std_logic                        IS    DescRam_Out(10);
     ALIAS    TX_BEGDEL                            : std_logic                        IS    DescRam_Out(12);
     ALIAS    TX_BEGON                            : std_logic                        IS    DescRam_Out(13);
     ALIAS    TX_TIME                                : std_logic                        IS    DescRam_Out(14);
     ALIAS    TX_RETRY                            : std_logic_vector( 3 DOWNTO 0)    IS    DescRam_Out(3 DOWNTO 0);
    SIGNAL    Ram_Be                                : std_logic_vector( 1 DOWNTO 0);
    SIGNAL    Ram_Wr, Desc_We                        : std_logic;
    SIGNAL    Desc_Addr                            : std_logic_vector( 7 DOWNTO 0);
    SIGNAL    DescIdx                                : std_logic_vector( 2 DOWNTO 0);
    SIGNAL    Last_Desc                            : std_logic;
    SIGNAL    ZeitL                                : std_logic_vector(15 DOWNTO 0);
    SIGNAL    Tx_Ie, Tx_Wait                        : std_logic;
    SIGNAL    Tx_BegInt, Tx_BegSet, Tx_Early        : std_logic;
    SIGNAL    Tx_Del                                : std_logic;
    SIGNAL    Ext_Tx, Ext_Ack                        : std_logic;
    SIGNAL    Tx_Desc, Tx_Desc_One, Ext_Desc        : std_logic_vector( 3 DOWNTO 0);
    SIGNAL    Tx_Icnt                                : std_logic_vector( 4 DOWNTO 0);
    SIGNAL    Tx_SoftInt                            : std_logic;
    SIGNAL    Sel_TxH, Sel_TxL, H_Byte            : std_logic;
    SIGNAL    Tx_Buf                                : std_logic_vector( 7 DOWNTO 0);
    SIGNAL    Tx_Idle, TxInt, Tx_Beg, Tx_Sync        : std_logic;
    SIGNAL    Tx_Ident                            : std_logic_vector( 1 DOWNTO 0);
    SIGNAL    Tx_Cmp_High                            : std_logic_vector(15 downto 0);
    SIGNAL    Start_TxS                            : std_logic;
    SIGNAL    Tx_Dma_Out                            : std_logic;
    SIGNAL    Tx_Del_Cnt                            : std_logic_vector(32 downto 0);
     ALIAS    Tx_Del_End                            : std_logic is Tx_Del_Cnt(Tx_Del_Cnt'high);
    SIGNAL    Tx_Del_Run                            : std_logic;
    signal    Tx_Done                                : std_logic;

BEGIN

    Dma_Rd_Done <= Tx_Done;

    Tx_Done <= '1' when Dsm = sStat or Dsm = sColl else '0';

    Tx_Dma_Very1stOverflow <= cActivated when Dibl_Cnt = "01" and Sm_Tx = R_Pre and Tx_Timer(7) = '1' else cInactivated;

    Ram_Wr    <= '1' WHEN    s_nWr = '0' AND Sel_Ram = '1' AND s_Adr(10) = '1'    ELSE '0';
    Ram_Be(1) <= '1' WHEN    s_nWr = '1' OR s_nBE(1) = '0'                        ELSE '0';
    Ram_Be(0) <= '1' WHEN    s_nWr = '1' OR s_nBE(0) = '0'                        ELSE '0';

    DescIdx <=    "000"    WHEN    Desc_We = '0' AND Tx_Dsm_Next = sIdle    ELSE
                "000"    WHEN    Desc_We = '1' AND Dsm = sIdle            ELSE
                "001"    WHEN    Desc_We = '0' AND Tx_Dsm_Next = sLen    ELSE
                "001"    WHEN    Desc_We = '1' AND Dsm = sLen            ELSE
                "010"    WHEN    Desc_We = '0' AND Tx_Dsm_Next = sAdrH    ELSE
                "010"    WHEN    Desc_We = '1' AND Dsm = sAdrH            ELSE
                "011"    WHEN    Desc_We = '0' AND Tx_Dsm_Next = sAdrL    ELSE
                "011"    WHEN    Desc_We = '1' AND Dsm = sAdrL            ELSE
                "100"    WHEN    Desc_We = '0' AND Tx_Dsm_Next = sBegH    ELSE
                "100"    WHEN    Desc_We = '1' AND Dsm = sBegH            ELSE
                "101"    WHEN    Desc_We = '0' AND Tx_Dsm_Next = sBegL    ELSE
                "101"    WHEN    Desc_We = '1' AND Dsm = sBegL            ELSE
                "110"    WHEN    Desc_We = '0' AND Tx_Dsm_Next = sTimH    ELSE
                "110"    WHEN    Desc_We = '1' AND Dsm = sTimH            ELSE
                "111"    WHEN    Desc_We = '0' AND Tx_Dsm_Next = sTimL    ELSE
                "111"    WHEN    Desc_We = '1' AND Dsm = sTimL            ELSE
                "111"    WHEN    Desc_We = '0' AND Tx_Dsm_Next = sData    ELSE
                "111"    WHEN    Desc_We = '1' AND Dsm = sData            ELSE
                "000";

    Desc_We <= '1' WHEN  Dsm = sTimL OR Dsm = sTimH OR Dsm = sStat    ELSE   '0';

    Desc_Addr <= '1' & Tx_Desc  & DescIdx    WHEN    Ext_Tx = '0'    ELSE
                 '1' & Ext_Desc & DescIdx;

gTxTime:    IF Timer GENERATE
    DescRam_In <= Zeit(15 DOWNTO 0)            WHEN    Dsm  = sTimH    ELSE
                  ZeitL                        WHEN    Dsm  = sTimL    ELSE
                  x"000" & "01" & Tx_Ident    WHEN    Dsm  = sBegL    ELSE
                  Tx_Dma_Out & Tx_Sync & "00" & "0100" & "00" & "0" & "0" & Col_Cnt;
END GENERATE;

gnTxTime:    IF NOT Timer GENERATE
    DescRam_In <= x"000" & "01" & Tx_Ident    WHEN    Dsm  = sBegL    ELSE
                  Tx_Dma_Out & Tx_Sync & "00" & "0100" & "00" & "0" & "0" & Col_Cnt;
END GENERATE;


RamH:    ENTITY    work.Dpr_16_16
    GENERIC MAP(Simulate => Simulate)
    PORT MAP (    CLKA    => Clk,                    CLKB    => Clk,
                EnA        => cActivated,            Enb        => cActivated,
                BEA        => Ram_Be,
                WEA        => Ram_Wr,                WEB        => Desc_We,
                ADDRA    => s_Adr(8 DOWNTO 1),    ADDRB    => Desc_Addr,
                  DIA        => s_Din,                DIB        => DescRam_In,
                DOA        => Tx_Ram_Dat,            DOB        => DescRam_Out
            );

    ASSERT NOT( TxSyncOn AND NOT Timer )
        REPORT "TxSyncOn needs Timer!"
            severity failure;

pTxSm: PROCESS( Rst, Clk, Dsm,
                Tx_On, TX_OWN, Retry_Cnt, Ext_Tx, Tx_Wait,
                Tx_Sync, Sm_Tx, F_End, Tx_Col, Ext_Ack, Tx_Del, Tx_Beg, Tx_Half, Tx_Del_End )
BEGIN


        Tx_Dsm_Next <= Dsm;
        CASE    Dsm IS
            WHEN sIdle     =>    IF    Tx_On = '1' AND TX_OWN = '1' AND Retry_Cnt = 0    THEN
                                IF    (Ext_Tx = '1' AND Ext_Ack = '0') OR Tx_Wait  = '0'      THEN
                                                        Tx_Dsm_Next <= sAdrH; --sLen;
                                END IF;
                            END IF;
            WHEN sLen     =>    IF    Tx_Sync = '0'    THEN    Tx_Dsm_Next <= sReq; --sAdrH;
                            ELSE                        Tx_Dsm_Next <= sBegH;
                            END IF;
            WHEN sBegH     =>                                Tx_Dsm_Next <= sBegL;
            WHEN sBegL     =>    IF       Tx_On  = '0'    THEN    Tx_Dsm_Next <= sIdle;
                            ELSIF Tx_Sync = '0'    THEN
                                if      Tx_Del = '1' then    Tx_Dsm_Next <= sDel;
                                elsIF Sm_Tx = R_Pre    THEN
                                                        Tx_Dsm_Next <= sTimH;
                                END IF;
                            ELSIF Tx_Sync = '1' and Tx_Beg = '1' and Tx_Half = '1' and rCrs_Dv = '1' THEN
                                                        Tx_Dsm_Next <= sColl;
                            ELSIF Tx_Beg = '1'    THEN    Tx_Dsm_Next <= sReq;
                            END IF;
            WHEN sDel     =>    IF Tx_On = '0'      THEN    Tx_Dsm_Next <= sIdle; --avoid FSM hang
                            ELSIF Tx_Del_End = '1' THEN Tx_Dsm_Next <= sTimH;
                            END IF;
            WHEN sAdrH     =>                                Tx_Dsm_Next <= sAdrL;
            WHEN sAdrL   =>                             Tx_Dsm_Next <= sLen; --sReq;
            --leaving sAdrL and entering sReq leads to the very first Tx_Dma_Req
            -- this enables early dma req at the beginning of IPG (auto-resp)
            WHEN sReq     =>    IF    Tx_On  = '0'    THEN    Tx_Dsm_Next <= sIdle;
                            elsif Tx_Del = '1'    then    Tx_Dsm_Next <= sBegH;
                            ELSIF Tx_Sync = '0'    THEN    Tx_Dsm_Next <= sBegL;
                            ELSIF Sm_Tx = R_Bop    THEN    Tx_Dsm_Next <= sTimH;
                            END IF;
            WHEN sTimH     =>                                Tx_Dsm_Next <= sTimL;
            WHEN sTimL     =>                                Tx_Dsm_Next <= sData;
            WHEN sData     =>    IF      F_End = '1'    THEN    Tx_Dsm_Next <= sStat;
                            ELSIF Tx_Col = '1'    THEN    Tx_Dsm_Next <= sColl;
                            END IF;
            WHEN sStat     =>                                Tx_Dsm_Next <= sIdle;
            WHEN sColl     =>    if    sm_tx = r_idl then
                                if    Tx_Sync = '1' then    Tx_Dsm_Next <= sStat;
                                else                    Tx_Dsm_Next <= sIdle;
                                end if;
                            end if;
            WHEN OTHERS     =>
        END CASE;

    IF    Rst = '1'                    THEN    Dsm <= sIdle;
    ELSIF    rising_edge( Clk )        THEN    Dsm <= Tx_Dsm_Next;
    END IF;

END PROCESS pTxSm;
pTxControl: PROCESS( Rst, Clk )
BEGIN

    IF    Rst = '1'    THEN
        Last_Desc <= '0'; Start_TxS <= '0'; Tx_Dma_Req  <= '0'; H_Byte <= '0';
        Tx_Beg <= '0'; Tx_BegSet <= '0'; Tx_Early <= '0'; Auto_Coll <= '0'; Tx_Dma_Out <= '0';
        Ext_Tx <= '0'; Ext_Ack <= '0'; ClrCol <= '0'; Ext_Desc <= (OTHERS => '0'); Tx_Sync <= '0'; Max_Retry <= (others => '0');
        ZeitL <= (OTHERS => '0'); Tx_Count <= (OTHERS => '0'); Tx_Ident <= "00";
        Dma_Tx_Addr <= (OTHERS => '0'); Tx_Cmp_High <= (others => '0');
        Tx_Del_Run <= '0';
        Tx_Del <= '0'; Tx_Del_Cnt <= (others => '0'); Tx_Dma_Len <= (others => '0');
    ELSIF    rising_edge( Clk )     THEN

        IF    TxSyncOn  = true    THEN
            IF        Tx_Sync = '1' AND Dsm = sBegL AND (DescRam_Out & Tx_Cmp_High ) = Zeit    THEN    Tx_Beg <= '1';
            ELSE                                                                                    Tx_Beg <= '0';
            END IF;
        END IF;

        IF    Dsm = sStat AND Desc_We = '1'    THEN    ClrCol  <= '1';
        ELSE                                        ClrCol  <= '0';
        END IF;

        IF Timer THEN
            IF    Dsm  = sTimH    THEN    ZeitL <= Zeit(31 DOWNTO 16);
            END IF;
        END IF;

        IF        Ext_Ack = '0' AND R_Req = '1'                    THEN    Ext_Desc <= Auto_Desc;
                                                                        Ext_Ack  <= '1';
        ELSIF    Ext_Tx = '1' OR    Tx_On = '0'                        THEN    Ext_Ack  <= '0';
        END IF;

        IF        Dsm = sIdle AND Ext_Ack = '1'                    THEN    Ext_Tx  <= '1';
        ELSIF    Dsm = sStat OR     Tx_Col = '1' OR    Tx_On = '0'        THEN    Ext_Tx  <= '0';
        END IF;

        IF       (F_End = '1' OR Tx_On = '0'
            OR (Tx_Col = '1' AND Ext_Tx = '1' )
            OR dsm = sColl     )                            THEN    Start_TxS <= '0';
                                                                Auto_Coll <= Auto_Coll OR (Tx_Col AND Ext_Tx);
        ELSIF    Dsm = sReq and Tx_Del = '0'            THEN    Start_TxS <= '1';
        ELSIF    Dsm = sDel and Tx_Del_End = '1'            THEN    Start_TxS <= '1';
        ELSIF    Sm_Tx = R_Idl                            THEN    Auto_Coll <= '0';
        END IF;

        IF        Dsm = sIdle        THEN    Last_Desc <= TX_LAST;
        END IF;

        IF        Dsm = sLen        THEN    Tx_Count  <= TX_LEN; Tx_Dma_Len <= TX_LEN; --add CRC
        ELSIF   F_Val = '1'        THEN    Tx_Count  <= Tx_Count - 1;
        END IF;

        IF        Dsm = sBegH        THEN    Tx_Cmp_High <= DescRam_Out;
        END IF;

        IF    Dsm = sIdle AND Tx_On = '1' AND TX_OWN = '1' AND Retry_Cnt = 0    THEN
            IF    Ext_Tx = '1' OR Tx_Wait  = '0'      THEN
                IF    TxSyncOn THEN                    Tx_Sync <= TX_TIME;
                ELSE                                Tx_Sync <= '0';
                END IF;
                                                    Max_Retry <= TX_RETRY;
                                                    Tx_Early  <= TX_BEGON;

                IF    TxDel = true            THEN    Tx_Del      <= TX_BEGDEL;
                END IF;
            END IF;
        ELSIF Dsm = sTimH                    THEN    Tx_BegSet <= Tx_Early;
        ELSIF Dsm = sTimL                    THEN    Tx_BegSet <= '0';
        ELSIF Dsm = sIdle                    THEN    Tx_Del <= '0';
        END IF;

        if    TxDel = true and Tx_Del = '1' then
            if    Dsm = sBegH                            then    Tx_Del_Cnt(Tx_Del_Cnt'high)    <= '0';
                                                            Tx_Del_Cnt(15 downto 0)    <= DescRam_Out;
            elsif    Dsm = sBegL                        then    Tx_Del_Cnt(31 downto 16) <= DescRam_Out;
            elsif    Dsm = sDel and Tx_Del_Run = '1'    then    Tx_Del_Cnt <= Tx_Del_Cnt - 1;
            end if;

            if        Tx_Del_Run = '0' and  Dsm = sDel                then    Tx_Del_Run <= '1'; --don't consider Ipg
            elsif    Tx_Del_End = '1'                                then    Tx_Del_Run <= '0';
            end if;

        end if;

        IF        Dsm = sAdrL         THEN    --Dma_Tx_Addr(15 DOWNTO 1)    <= DescRam_Out(15 DOWNTO 1);
                Dma_Tx_Addr(Dma_Addr'high DOWNTO 16) <= DescRam_Out(Dma_Addr'high-16 DOWNTO 0);
                Tx_Ident <= DescRam_Out(15 DOWNTO 14);
        ELSIF    Tx_Dma_Ack = '1' THEN    Dma_Tx_Addr(15 DOWNTO 1)    <= Dma_Tx_Addr(15 DOWNTO 1) + 1;
        END IF;

        IF        Dsm = sAdrH         THEN    Dma_Tx_Addr(15 DOWNTO 1)    <= DescRam_Out(15 DOWNTO 1);
--                Dma_Tx_Addr(Dma_Addr'high DOWNTO 16) <= DescRam_Out(Dma_Addr'high-16 DOWNTO 0);
--                Tx_Ident <= DescRam_Out(15 DOWNTO 14);
        ELSIF    Tx_Dma_Ack = '1' AND Dma_Tx_Addr(15 DOWNTO 1) = x"FFF" & "111"    THEN
                Dma_Tx_Addr(Dma_Addr'high DOWNTO 16) <= Dma_Tx_Addr(Dma_Addr'high DOWNTO 16) + 1;
        END IF;

        IF        DSM = sAdrL
            OR (F_Val = '1' AND H_Byte = '0')    THEN    Tx_Dma_Req  <= '1';
        ELSIF    Tx_Dma_Ack = '1'                THEN    Tx_Dma_Req  <= '0';
        END IF;

        IF        Sm_Tx = R_Bop                    THEN    H_Byte <= '0';
        ELSIF    F_Val = '1'                        THEN    H_Byte <= NOT H_Byte;
        END IF;

        IF    F_Val = '1'            THEN    Tx_Buf <= Tx_LatchL;
        END IF;

        if        H_Byte = '0' and F_Val = '1' and Tx_Dma_Req = '1'    then    Tx_Dma_Out <= '1';
        elsif    Sm_Tx = R_Bop                                        then    Tx_Dma_Out <= '0';
        end if;

    END IF;

END PROCESS pTxControl;

    Start_Tx <= '1'    WHEN    Start_TxS = '1' AND Block_Col = '0'            ELSE
                '1'    WHEN    not TxDel and not TxSyncOn and R_Req = '1'    ELSE
                '0';
    F_TxB <=    Tx_LatchH    WHEN    H_Byte = '0'    ELSE
                Tx_Buf;

    nTx_Int <= '1'    WHEN    (Tx_Icnt = 0 AND Tx_SoftInt = '0') OR Tx_Ie = '0'    ELSE    '0';

    Tx_Idle <= '1'    WHEN    Sm_Tx = R_Idl AND Dsm = sIdle ELSE '0';

    Tx_Reg(15 DOWNTO 4) <= Tx_Ie & Tx_SoftInt & Tx_Half & Tx_Wait & (Tx_Icnt(4) OR Tx_Icnt(3)) & Tx_Icnt(2 DOWNTO 0)
                         & Tx_On &  Tx_BegInt & Tx_Idle & "0" ;

    Tx_Reg( 3 DOWNTO 0) <=  Tx_Desc;


    Sel_TxH <= '1'    WHEN s_nWr = '0' AND Sel_Cont = '1' AND s_Adr(3) = '0' AND    Ram_Be(1) = '1'    ELSE    '0';
    Sel_TxL <= '1'    WHEN s_nWr = '0' AND Sel_Cont = '1' AND s_Adr(3) = '0' AND    Ram_Be(0) = '1'    ELSE    '0';

    Tx_Desc <= Tx_Desc_One;

    Tx_SoftInt <= '0';

pTxRegs: PROCESS( Rst, Clk )
BEGIN

    IF    Rst = '1'    THEN
        Tx_On   <= '0'; Tx_Ie <= '0'; Tx_Half  <= '0'; Tx_Wait  <= '0'; nTx_BegInt <= '0';
        Tx_Desc_One <= (OTHERS => '0');
        Tx_Icnt <= (OTHERS => '0'); TxInt <= '0'; Tx_BegInt  <= '0';
        Tx_Ipg  <= conv_std_logic_vector( 42, 6);
    ELSIF    rising_edge( Clk )    THEN

        IF    Sel_TxL = '1'    THEN
            IF        s_Adr(2 DOWNTO 1) = "00"                        THEN    Tx_On  <= S_Din( 7);
            ELSIF    s_Adr(2 DOWNTO 1) = "01" AND S_Din( 7) = '1'    THEN    Tx_On  <= '1';
            ELSIF    s_Adr(2 DOWNTO 1) = "10" AND S_Din( 7) = '1'    THEN    Tx_On  <= '0';
            END IF;
        END IF;

        IF        Tx_BegSet = '1'    AND Tx_Ie = '1'                                 THEN    Tx_BegInt  <= '1';
        ELSIF    Sel_TxL = '1' AND s_Adr(2 DOWNTO 1) = "01" AND S_Din( 6) = '1'    THEN    Tx_BegInt  <= '1';
        ELSIF    Sel_TxL = '1' AND s_Adr(2 DOWNTO 1) = "10" AND S_Din( 6) = '1'    THEN    Tx_BegInt  <= '0';
        END IF;

        nTx_BegInt <= NOT Tx_BegInt;

        IF    Sel_TxL = '1' AND s_Adr(2 DOWNTO 1) = "11"                THEN    Tx_Desc_One <= S_Din( 3 DOWNTO 0);
        ELSIF    Dsm = sStat AND Ext_Tx = '0'     THEN
            IF        Last_Desc = '1'                                    THEN    Tx_Desc_One <= x"0";
            ELSE                                                            Tx_Desc_One <= Tx_Desc + 1;
            END IF;
        END IF;

        IF        Sel_TxH = '1'    THEN
            IF        s_Adr(2 DOWNTO 1) = "00"                        THEN    Tx_Ie  <= S_Din(15);
            ELSIF    s_Adr(2 DOWNTO 1) = "01" AND S_Din(15) = '1'    THEN    Tx_Ie  <= '1';
            ELSIF    s_Adr(2 DOWNTO 1) = "10" AND S_Din(15) = '1'    THEN    Tx_Ie  <= '0';
            END IF;
        END IF;

        IF        Sel_TxH = '1'    THEN
            IF        s_Adr(2 DOWNTO 1) = "00"                        THEN    Tx_Half  <= S_Din(13);
            ELSIF    s_Adr(2 DOWNTO 1) = "01" AND S_Din(13) = '1'    THEN    Tx_Half  <= '1';
            ELSIF    s_Adr(2 DOWNTO 1) = "10" AND S_Din(13) = '1'    THEN    Tx_Half  <= '0';
            END IF;
        END IF;

        IF        Sel_TxH = '1'    THEN
            IF        s_Adr(2 DOWNTO 1) = "00"                        THEN    Tx_Wait  <= S_Din(12);
            ELSIF    s_Adr(2 DOWNTO 1) = "01" AND S_Din(12) = '1'    THEN    Tx_Wait  <= '1';
            ELSIF    s_Adr(2 DOWNTO 1) = "10" AND S_Din(12) = '1'    THEN    Tx_Wait  <= '0';
            END IF;
        END IF;

        IF        Sel_TxH = '1'    THEN
            IF        s_Adr(2 DOWNTO 1) = "11" AND S_Din(14) = '1'    THEN    Tx_Ipg        <= S_Din(13 DOWNTO 8);
            END IF;
        END IF;

        IF        Tx_Ie = '1' AND Dsm = sStat AND Desc_We = '1'        THEN    TxInt <= '1';
        ELSE                                                                TxInt <= '0';
        END IF;

        IF        Sel_TxH = '1' AND s_Adr(2 DOWNTO 1) = "10" AND S_Din(8) = '1'
            AND    Tx_Icnt /= 0                                        THEN    Tx_Icnt <= Tx_Icnt - NOT TxInt;
        ELSIF    TxInt = '1'    AND Tx_Icnt /= "11111"                    THEN    Tx_Icnt <= Tx_Icnt + 1;
        END IF;

    END IF;

END PROCESS pTxRegs;


END BLOCK bTxDesc;

END BLOCK b_Full_Tx;


b_Full_Rx:    BLOCK

    TYPE    MACRX_TYPE IS ( R_Idl, R_Sof, R_Rxd );
    SIGNAL    Sm_Rx                        : MACRX_TYPE;
    SIGNAL    Rx_Dat, Rx_DatL                : std_logic_vector( 1 DOWNTO 0);
    SIGNAL    Tx_Timer                    : std_logic_vector( 7 DOWNTO 0);
    SIGNAL    Dibl_Cnt                    : std_logic_vector( 1 DOWNTO 0);
    SIGNAL    Crc, nCrc                    : std_logic_vector(31 DOWNTO 0);
    SIGNAL    CrcDin                        : std_logic_vector( 1 DOWNTO 0);
    SIGNAL    F_Err, P_Err, N_Err, A_Err    : std_logic;
    SIGNAL    F_End, F_Val, Rx_Beg        : std_logic;
    SIGNAL    Rx_Sr                        : std_logic_vector( 7 DOWNTO 0);
    SIGNAL    nCrc_Ok, Crc_Ok                : std_logic;
    SIGNAL    WrDescStat                    : std_logic;
    SIGNAL    PreCount                    : std_logic_vector( 4 DOWNTO 0);
    SIGNAL    PreBeg, PreErr                : std_logic;
    SIGNAL    Rx_DvL                        : std_logic;
    SIGNAL    Diag                        : std_logic;

BEGIN

    Rx_Beg <= '1' WHEN    Rx_Dv = '1' AND Sm_Rx = R_SOF AND Rx_Dat = "11"    ELSE '0';

    nCrc_Ok <= '1' WHEN nCrc = x"C704DD7B"    ELSE '0';

rxsm: PROCESS ( Clk, Rst )     IS
BEGIN

    IF Rst = '1'    THEN
        Sm_Rx <= R_Idl;
    ELSIF rising_edge( Clk )     THEN
        IF    Sm_Rx = R_Idl OR Sm_Rx = R_Rxd OR Sm_Rx = R_Sof OR Dibl_Cnt = "11"    THEN
            CASE Sm_Rx    IS
                WHEN    R_Idl =>    IF        Rx_Dv = '1'        THEN    Sm_Rx    <= R_Sof;    END IF;
                WHEN    R_Sof =>    IF        Rx_Dat = "11"    THEN    Sm_Rx    <= R_Rxd;
                                    ELSIF    Rx_Dv = '0'        THEN    Sm_Rx    <= R_Idl;    END IF;
                WHEN    R_Rxd =>    IF        Rx_Dv = '0'        THEN    Sm_Rx    <= R_Idl;    END IF;
                WHEN  OTHERS  =>    NULL;
            END CASE;
        END IF;
    END IF;

END PROCESS rxsm;

pRxCtl: PROCESS ( Clk, Rst )     IS
    VARIABLE    Preload        :    std_logic_vector(Tx_Timer'RANGE);
    VARIABLE    Load        :    std_logic;
BEGIN

    IF Rst = '1'    THEN
        Rx_DatL <= "00"; Rx_Dat <= "00"; Rx_Dv <= '0'; Dibl_Cnt <= "00"; PreCount <= (OTHERS => '0');
        F_End <= '0'; F_Err <= '0';  F_Val <= '0'; Crc_Ok <= '0';
        A_Err <= '0'; N_Err <= '0'; P_Err <= '0'; PreBeg <= '0'; PreErr <= '0';
    ELSIF rising_edge( Clk )     THEN

        Rx_DatL <= rRx_Dat;

        Rx_Dat <= Rx_DatL;

        IF        Rx_Dv = '0' AND rCrs_Dv = '1'                        THEN    Rx_Dv <= '1';
        ELSIF    Rx_Dv = '1'    AND rCrs_Dv = '0' AND Dibl_Cnt(0) = '1'    THEN    Rx_Dv <= '0';
        END  IF;

        IF        Rx_Beg = '1'    THEN    Dibl_Cnt <= "00";
        ELSE                            Dibl_Cnt <= Dibl_Cnt + 1;
        END IF;

        Crc_Ok <= nCrc_Ok;

        IF        (Sm_Rx = R_Rxd AND Rx_Dv = '0')            THEN    F_End <= '1';
                                                                F_Err <= NOT Crc_Ok;
        ELSE                                                    F_End <= '0';
        END IF;

        IF        Dibl_Cnt = "11" AND Sm_Rx = R_Rxd        THEN    F_Val <= '1';
        ELSE                                                    F_Val <= '0';
        END IF;

        IF        WrDescStat = '1'                        THEN    A_Err <= '0';
        ELSIF    F_End = '1' AND Dibl_Cnt /= 1            THEN    A_Err <= '1';
        END IF;

        IF        Rx_Dv = '0' OR Rx_Dat(0) = '0'            THEN    PreCount <= (OTHERS => '1');
        ELSE                                                    PreCount <= PreCount - 1;
        END IF;

        IF        Rx_Dv  = '0'    THEN    PreBeg <= '0';
        ELSIF    Rx_Dat = "01"    THEN    PreBeg <= '1';
        END IF;

        IF        WrDescStat = '1'                                    THEN    N_Err <= '0';
        ELSIF    Sm_Rx = R_Sof AND Rx_Dv  = '0'                        THEN    N_Err <= '1';
        END IF;

        IF        Rx_DvL = '0'                                        THEN    PreErr <= '0';
        ELSIF    PreBeg = '0' AND (Rx_Dat = "10" OR Rx_Dat = "11")    THEN    PreErr <= '1';
        ELSIF    PreBeg = '1' AND (Rx_Dat = "10" OR Rx_Dat = "00")    THEN    PreErr <= '1';
        END IF;

        IF        WrDescStat = '1'                THEN    P_Err <= '0';
        ELSIF    Rx_Beg = '1' AND PreErr = '1'    THEN    P_Err <= '1';
        ELSIF    Rx_Beg = '1' AND PreCount /= 0    THEN    P_Err <= '1';
        END IF;

        Rx_Sr <= Rx_Dat(1) & Rx_Dat(0) & Rx_Sr(7 DOWNTO 2);

        Rx_DvL  <= Rx_Dv;

    END IF;

END PROCESS pRxCtl;

    CrcDin <= Rx_Dat;

Calc: PROCESS ( Clk, Crc, nCrc, CrcDin, Sm_Rx )     IS
    VARIABLE    H         : std_logic_vector(1 DOWNTO 0);
BEGIN

    H(0) := Crc(31) XOR CrcDin(0);
    H(1) := Crc(30) XOR CrcDin(1);

    IF        Sm_Rx = R_Sof     THEN    nCrc <= x"FFFFFFFF";
    ELSE
        nCrc( 0) <=                         H(1);
        nCrc( 1) <=             H(0) XOR H(1);
        nCrc( 2) <= Crc( 0) XOR H(0) XOR H(1);
        nCrc( 3) <= Crc( 1) XOR H(0)         ;
        nCrc( 4) <= Crc( 2)          XOR H(1);
        nCrc( 5) <= Crc( 3) XOR H(0) XOR H(1);
        nCrc( 6) <= Crc( 4) XOR H(0)         ;
        nCrc( 7) <= Crc( 5)          XOR H(1);
        nCrc( 8) <= Crc( 6) XOR H(0) XOR H(1);
        nCrc( 9) <= Crc( 7) XOR H(0)         ;
        nCrc(10) <= Crc( 8)          XOR H(1);
        nCrc(11) <= Crc( 9) XOR H(0) XOR H(1);
        nCrc(12) <= Crc(10) XOR H(0) XOR H(1);
        nCrc(13) <= Crc(11) XOR H(0)         ;
        nCrc(14) <= Crc(12)                  ;
        nCrc(15) <= Crc(13)                  ;
        nCrc(16) <= Crc(14)          XOR H(1);
        nCrc(17) <= Crc(15) XOR H(0)         ;
        nCrc(18) <= Crc(16)                  ;
        nCrc(19) <= Crc(17)                  ;
        nCrc(20) <= Crc(18)                  ;
        nCrc(21) <= Crc(19)                  ;
        nCrc(22) <= Crc(20)          XOR H(1);
        nCrc(23) <= Crc(21) XOR H(0) XOR H(1);
        nCrc(24) <= Crc(22) XOR H(0)         ;
        nCrc(25) <= Crc(23)                  ;
        nCrc(26) <= Crc(24)          XOR H(1);
        nCrc(27) <= Crc(25) XOR H(0)         ;
        nCrc(28) <= Crc(26)                  ;
        nCrc(29) <= Crc(27)                  ;
        nCrc(30) <= Crc(28)                  ;
        nCrc(31) <= Crc(29)                   ;
    END IF;

    IF rising_edge( Clk )  THEN
        Crc <= nCrc;
    END IF;

END PROCESS Calc;

bRxDesc:    BLOCK
    TYPE    sDESC    IS    (sIdle, sLen, sTimL, sTimH, sAdrH, sAdrL, sData, sOdd, sStat, sLenW );
    SIGNAL    Dsm, Rx_Dsm_Next                        : sDESC;
    SIGNAL    Rx_Buf, Rx_LatchH, Rx_LatchL            : std_logic_vector( 7 DOWNTO 0);
    SIGNAL    Rx_Ovr                                    : std_logic;
    SIGNAL    DescRam_Out, DescRam_In                    : std_logic_vector(15 DOWNTO 0);
     ALIAS    RX_LEN                                    : std_logic_vector(11 DOWNTO 0)    IS    DescRam_Out(11 DOWNTO  0);
     ALIAS    RX_OWN                                    : std_logic                        IS    DescRam_Out( 8);
     ALIAS    RX_LAST                                    : std_logic                        IS    DescRam_Out( 9);
    SIGNAL    Ram_Be                                    : std_logic_vector( 1 DOWNTO 0);
    SIGNAL    Ram_Wr, Desc_We                            : std_logic;
    SIGNAL    Desc_Addr                                : std_logic_vector( 7 DOWNTO 0);
    SIGNAL    ZeitL                                    : std_logic_vector(15 DOWNTO 0);
    SIGNAL    Rx_On, Rx_Ie, Sel_RxH, Sel_RxL            : std_logic;
    SIGNAL    Rx_Desc, Match_Desc                        : std_logic_vector( 3 DOWNTO 0);
    SIGNAL    Rx_Icnt                                    : std_logic_vector( 4 DOWNTO 0);
    SIGNAL    Rx_Lost, Last_Desc, Answer_Tx            : std_logic;
    SIGNAL    DescIdx                                    : std_logic_vector( 2 DOWNTO 0);
    SIGNAL    Rx_Count, Rx_Limit                        : std_logic_vector(11 DOWNTO 0);
    SIGNAL    Match, Filt_Cmp                            : std_logic;
    SIGNAL    Rx_Idle, RxInt                             : std_logic;
    SIGNAL    Hub_Rx_L                                : std_logic_vector( 1 DOWNTO 0);
    SIGNAL    Rx_Dma_Out                                : std_logic;
    signal  Rx_Done                                 : std_logic;

BEGIN

    process(rst, clk)
        variable doPulse : std_logic;
    begin
        if rst = cActivated then
            Rx_Done <= cInactivated;
            doPulse := cInactivated;
        elsif rising_edge(clk) then
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

    Dma_Wr_Done <= Rx_Done;

    WrDescStat <= '1' WHEN Dsm = sStat    ELSE '0';

    Ram_Wr    <= '1' WHEN    s_nWr = '0' AND Sel_Ram = '1' AND s_Adr(10) = '1'    ELSE '0';
    Ram_Be(1) <= '1' WHEN    s_nWr = '1' OR s_nBE(1) = '0'                        ELSE '0';
    Ram_Be(0) <= '1' WHEN    s_nWr = '1' OR s_nBE(0) = '0'                        ELSE '0';

    DescIdx <=    "001"    WHEN    Desc_We = '0' AND (Rx_Dsm_Next = sLen OR Rx_Dsm_Next = sLenW)    ELSE
                "001"    WHEN    Desc_We = '1' AND (Dsm         = sLen OR Dsm         = sLenW)    ELSE
                "010"    WHEN    Desc_We = '0' AND Rx_Dsm_Next = sAdrH                            ELSE
                "010"    WHEN    Desc_We = '1' AND Dsm         = sAdrH                            ELSE
                "011"    WHEN    Desc_We = '0' AND Rx_Dsm_Next = sAdrL                            ELSE
                "011"    WHEN    Desc_We = '1' AND Dsm         = sAdrL                            ELSE
                "110"    WHEN    Desc_We = '0' AND Rx_Dsm_Next = sTimH                            ELSE
                "110"    WHEN    Desc_We = '1' AND Dsm         = sTimH                            ELSE
                "111"    WHEN    Desc_We = '0' AND Rx_Dsm_Next = sTimL                            ELSE
                "111"    WHEN    Desc_We = '1' AND Dsm         = sTimL                            ELSE
                "000";

    Desc_We <= '1'    WHEN   Dsm = sTimL OR Dsm = sTimH                    ELSE
               '1'    WHEN  (Dsm = sLenW OR Dsm = sStat) AND Match = '1'  ELSE    '0';

    Desc_Addr <= "0" & Rx_Desc & DescIdx;

gRxTime:    IF timer GENERATE
    DescRam_In <= Zeit(15 DOWNTO 0)                WHEN    Dsm = sTimH        ELSE
                  ZeitL                            WHEN    Dsm = sTimL        ELSE
                  x"0"  & Rx_Count                WHEN    Dsm = sLenW        ELSE
                  Rx_Dma_Out & '0' & "0" & A_Err & Hub_Rx_L & "00" & Match_Desc & N_Err & P_Err & Rx_Ovr & F_Err;
END GENERATE;

ngRxTime:    IF NOT timer GENERATE
    DescRam_In <= x"0"  & Rx_Count                WHEN    Dsm = sLenW                    ELSE
                  Rx_Dma_Out & '0' & "0" & A_Err & Hub_Rx_L & "00" & Match_Desc & N_Err & P_Err & Rx_Ovr & F_Err;
END GENERATE;

RxRam:    ENTITY    work.Dpr_16_16
    GENERIC MAP(Simulate => Simulate)
    PORT MAP (    CLKA    => Clk,                    CLKB    => Clk,
                EnA        => cActivated,            Enb        => cActivated,
                BEA        => Ram_Be,
                WEA        => Ram_Wr,                WEB        => Desc_We,
                ADDRA    => s_Adr(8 DOWNTO 1),    ADDRB    => Desc_Addr,
                  DIA        => s_Din,                DIB        => DescRam_In,
                DOA        => Rx_Ram_Dat,            DOB        => DescRam_Out
            );


pRxSm: PROCESS( Rst, Clk, Dsm,
                Rx_Beg, Rx_On, RX_OWN, F_End, F_Err, Diag, Rx_Count )
BEGIN

        Rx_Dsm_Next <= Dsm;
        CASE    Dsm IS
            WHEN sIdle     =>    IF    Rx_Beg = '1' AND Rx_On = '1' AND RX_OWN = '1' THEN
                                                        Rx_Dsm_Next <= sLen;
                            END IF;
            WHEN sLen     =>                                Rx_Dsm_Next <= sAdrH;
            WHEN sAdrH     =>                                Rx_Dsm_Next <= sAdrL;
            WHEN sAdrL     =>                                Rx_Dsm_Next <= sTimH;
            WHEN sTimH     =>                                Rx_Dsm_Next <= sTimL;
            WHEN sTimL     =>                                Rx_Dsm_Next <= sData;
            WHEN sData     =>    IF    F_End = '1'    THEN
                                IF    F_Err = '0'
                                 OR Diag  = '1'    THEN    Rx_Dsm_Next <= sStat;
                                ELSE                    Rx_Dsm_Next <= sIdle;
                                END IF;
                            END IF;
            WHEN sStat     =>                                Rx_Dsm_Next <= sLenW;
            WHEN sLenW   =>    IF    Rx_Count(0) = '0' THEN
                                                        Rx_Dsm_Next <= sIdle;
                            ELSE                        Rx_Dsm_Next <= sOdd;
                            END IF;
            WHEN sOdd   =>                                Rx_Dsm_Next <= sIdle;
            WHEN OTHERS     =>
        END CASE;

    IF        Rst = '1'                THEN    Dsm <= sIdle;
    ELSIF    rising_edge( Clk )        THEN    Dsm <= Rx_Dsm_Next;
    END IF;

END PROCESS pRxSm;

pRxControl: PROCESS( Rst, Clk )
BEGIN

    IF    Rst = '1'    THEN
        Rx_Ovr <= '0'; Rx_Dma_Req  <= '0'; Last_Desc <= '0'; Rx_Dma_Out <= '0';
        Rx_Count <= (OTHERS => '0');
        Rx_Buf <= (OTHERS => '0'); Rx_LatchL <= (OTHERS => '0'); Rx_LatchH <= (OTHERS => '0');
        Dma_Rx_Addr <= (OTHERS => '0');
    ELSIF    rising_edge( Clk )     THEN

        IF    Timer    THEN
            IF        Dsm  = sTimH    THEN    ZeitL <= Zeit(31 DOWNTO 16);
            END IF;
        END IF;

        IF        Dsm = sIdle        THEN    Rx_Count  <= (OTHERS => '0');
                                        Last_Desc <= RX_LAST;
        ELSIF   F_Val = '1'        THEN    Rx_Count  <= Rx_Count + 1;
        END IF;

        IF        Dsm = sLen        THEN    Rx_Limit      <= RX_LEN;
                                        Hub_Rx_L      <= Hub_Rx;
        END IF;

        IF    F_Val = '1'        THEN    Rx_Buf <= Rx_Sr;
        END IF;

        IF    (F_Val = '1' AND Rx_Count(0) = '1') OR    Dsm = sStat     THEN    Rx_LatchH <= Rx_Buf;
                                                                        Rx_LatchL <= Rx_Sr;
            IF        Rx_Dma_Req = '1' AND Sm_Rx /= R_Idl            THEN    Rx_Dma_Out <= '1';
            END IF;
        ELSIF    Dsm = sLen                                        THEN    Rx_Dma_Out <= '0';
        END IF;

        IF        Dsm = sLen                                THEN    Rx_Ovr <= '0';
        ELSIF    F_Val = '1' AND Rx_Limit = Rx_Count        THEN    Rx_Ovr <= '1';
        END IF;


        IF        Dsm = sAdrL         THEN    --Dma_Rx_Addr(15 DOWNTO 1) <= DescRam_Out(15 DOWNTO 1);
            Dma_Rx_Addr(Dma_Addr'high DOWNTO 16) <= DescRam_Out(Dma_Addr'high-16 DOWNTO 0);
        ELSIF    Rx_Dma_Ack = '1' THEN    Dma_Rx_Addr(15 DOWNTO 1) <= Dma_Rx_Addr(15 DOWNTO 1) + 1;
        END IF;

        IF        Dsm = sAdrH         THEN    Dma_Rx_Addr(15 DOWNTO 1) <= DescRam_Out(15 DOWNTO 1);
                --Dma_Rx_Addr(Dma_Addr'high DOWNTO 16) <= DescRam_Out(Dma_Addr'high-16 DOWNTO 0);
        ELSIF    Rx_Dma_Ack = '1' AND Dma_Rx_Addr(15 DOWNTO 1) = x"FFF" & "111"    THEN
                Dma_Rx_Addr(Dma_Addr'high DOWNTO 16) <= Dma_Rx_Addr(Dma_Addr'high DOWNTO 16) + 1;
        END IF;

        IF        Filt_Cmp = '0' AND Match ='0'                                        THEN    Rx_Dma_Req  <= '0';

        ELSIF  (Dsm = sOdd  AND Rx_Ovr = '0')
            OR (Dsm = sData AND Rx_Ovr = '0' AND F_Val = '1' AND Rx_Count(0) = '1')    THEN    Rx_Dma_Req  <= '1';
        ELSIF    Rx_Dma_Ack = '1'                                                    THEN    Rx_Dma_Req  <= '0';
        END IF;

    END IF;

END PROCESS pRxControl;

    Dma_Dout <= Rx_LatchL & Rx_LatchH; --Rx_LatchH & Rx_LatchL;

    nRx_Int <= '1'    WHEN    Rx_Icnt = 0 OR Rx_Ie = '0'         ELSE    '0';

    Rx_Idle <= '1'    WHEN    Sm_Rx = R_Idl ELSE '0';

    Rx_Reg(15 DOWNTO 4) <= Rx_Ie & '0' & "0"      & '0'     & (Rx_Icnt(4) OR Rx_Icnt(3)) & Rx_Icnt(2 DOWNTO 0)
                         & Rx_On & "0" & Rx_Idle & Rx_Lost;

    Rx_Reg( 3 DOWNTO 0) <= Rx_Desc;

bFilter: BLOCK
    SIGNAL    Ram_Addr                    : std_logic_vector( 7 DOWNTO 0);
    SIGNAL    Ram_BeH, Ram_BeL            : std_logic_vector( 1 DOWNTO 0);
    SIGNAL    Ram_Wr                        : std_logic;
    SIGNAL    Filter_Addr                    : std_logic_vector( 6 DOWNTO 0);
     SIGNAL    Filter_Out_H, Filter_Out_L    : std_logic_vector(31 DOWNTO 0);
     ALIAS    DIRON_0                        : std_logic    IS    Filter_Out_H( 11);
     ALIAS    DIRON_1                        : std_logic    IS    Filter_Out_H( 27);
     ALIAS    DIRON_2                        : std_logic    IS    Filter_Out_L( 11);
     ALIAS    DIRON_3                        : std_logic    IS    Filter_Out_L( 27);
     ALIAS    TX_0                        : std_logic    IS    Filter_Out_H( 7);
     ALIAS    TX_1                        : std_logic    IS    Filter_Out_H(23);
     ALIAS    TX_2                        : std_logic    IS    Filter_Out_L( 7);
     ALIAS    TX_3                        : std_logic    IS    Filter_Out_L(23);
      ALIAS    ON_0                        : std_logic    IS    Filter_Out_H( 6);
     ALIAS    ON_1                        : std_logic    IS    Filter_Out_H(22);
      ALIAS    ON_2                        : std_logic    IS    Filter_Out_L( 6);
     ALIAS    ON_3                        : std_logic    IS    Filter_Out_L(22);
     ALIAS    DESC_0                        : std_logic_vector( 3 DOWNTO 0)    IS    Filter_Out_H( 3 DOWNTO  0);
     ALIAS    DESC_1                        : std_logic_vector( 3 DOWNTO 0)    IS    Filter_Out_H(19 DOWNTO 16);
     ALIAS    DESC_2                        : std_logic_vector( 3 DOWNTO 0)    IS    Filter_Out_L( 3 DOWNTO  0);
     ALIAS    DESC_3                        : std_logic_vector( 3 DOWNTO 0)    IS    Filter_Out_L(19 DOWNTO 16);

    SIGNAL    Byte_Cnt                    : std_logic_vector( 4 DOWNTO 0) := (OTHERS => '0');
    SIGNAL    Erg0, Erg1, Erg2, Erg3        : std_logic_vector( 7 DOWNTO 0);
    SIGNAL    Mat_Reg                        : std_logic_vector(15 DOWNTO 0);
     SIGNAL    Filt_Idx                    : std_logic_vector( 1 DOWNTO 0);
     SIGNAL    Mat_Sel                        : std_logic_vector( 3 DOWNTO 0);
     SIGNAL    M_Prio                        : std_logic_vector( 2 DOWNTO 0);
     ALIAS    Found                        : std_logic IS M_Prio(2);

BEGIN
    Ram_Addr   <= s_Adr(9 DOWNTO 8) & s_Adr(5 DOWNTO 1) & s_Adr(6);

    Ram_Wr     <= '1' WHEN    s_nWr = '0' AND Sel_Ram = '1'  AND s_Adr(10) = '0'    ELSE '0';
    Ram_BeH(1) <= '1' WHEN    s_nWr = '1' OR (s_nBE(1) = '0' AND s_Adr( 7) = '0')    ELSE '0';
    Ram_BeH(0) <= '1' WHEN    s_nWr = '1' OR (s_nBE(0) = '0' AND s_Adr( 7) = '0')    ELSE '0';
    Ram_BeL(1) <= '1' WHEN    s_nWr = '1' OR (s_nBE(1) = '0' AND s_Adr( 7) = '1')    ELSE '0';
    Ram_BeL(0) <= '1' WHEN    s_nWr = '1' OR (s_nBE(0) = '0' AND s_Adr( 7) = '1')    ELSE '0';

    Filter_Addr <= Dibl_Cnt & Byte_Cnt;

FiltRamH:    ENTITY    work.Dpr_16_32
    GENERIC MAP(Simulate => Simulate)
    PORT MAP (    CLKA    => Clk,            CLKB    => Clk,
                EnA        => cActivated,    EnB        => cActivated,
                BEA        => Ram_BeH,
                WEA        => Ram_Wr,
                ADDRA    => Ram_Addr,    ADDRB    => Filter_Addr,
                  DIA        => s_Din,        DOB        => Filter_Out_H
                );

FiltRamL:    ENTITY    work.Dpr_16_32
    GENERIC MAP(Simulate => Simulate)
    PORT MAP (    CLKA    => Clk,            CLKB    => Clk,
                EnA        => cActivated,    EnB        => cActivated,
                BEA        => Ram_BeL,
                WEA        => Ram_Wr,
                ADDRA    => Ram_Addr,    ADDRB    => Filter_Addr,
                DIA        => s_Din,        DOB        => Filter_Out_L
            );

    Erg0 <= (Rx_Buf XOR Filter_Out_H( 7 DOWNTO  0)) AND Filter_Out_H(15 DOWNTO  8);
     Erg1 <= (Rx_Buf XOR Filter_Out_H(23 DOWNTO 16)) AND Filter_Out_H(31 DOWNTO 24);
    Erg2 <= (Rx_Buf XOR Filter_Out_L( 7 DOWNTO  0)) AND Filter_Out_L(15 DOWNTO  8);
    Erg3 <= (Rx_Buf XOR Filter_Out_L(23 DOWNTO 16)) AND Filter_Out_L(31 DOWNTO 24);

genMatSel:    FOR i IN 0 TO 3 GENERATE
    Mat_Sel(i) <=    Mat_Reg( 0 + i)    WHEN  Filt_Idx = "00"    ELSE
                    Mat_Reg( 4 + i)    WHEN  Filt_Idx = "01"    ELSE
                    Mat_Reg( 8 + i)    WHEN  Filt_Idx = "10"    ELSE
                    Mat_Reg(12 + i); --    WHEN  Filt_Idx = "11";
END GENERATE;

    M_Prio <= "000" WHEN    Filt_Cmp = '0' OR Match = '1'                            ELSE
              "100"    WHEN    Mat_Sel(0) = '1'  AND On_0 = '1' AND (DIRON_0 = '0')    ELSE
              "101"    WHEN    Mat_Sel(1) = '1'  AND On_1 = '1' AND (DIRON_1 = '0')    ELSE
              "110"    WHEN    Mat_Sel(2) = '1'  AND On_2 = '1' AND (DIRON_2 = '0')    ELSE
              "111"    WHEN    Mat_Sel(3) = '1'  AND On_3 = '1' AND (DIRON_3 = '0')    ELSE
              "000";

pFilter: PROCESS( Rst, Clk )
BEGIN

    IF    Rst = '1'    THEN
        Filt_Idx <= "00"; Match <= '0';
        Filt_Cmp <= '0'; Mat_Reg <= (OTHERS => '0'); Byte_Cnt <= (OTHERS =>'0');
        Match_Desc <= (OTHERS => '0');Auto_Desc <= (OTHERS =>'0'); Answer_Tx <= '0';
    ELSIF    rising_edge( Clk )     THEN

        Filt_Idx  <= Dibl_Cnt;

        IF        Dibl_Cnt = "11"    AND Rx_Count(5) = '0'    THEN    Byte_Cnt <= Rx_Count(Byte_Cnt'RANGE);
        END IF;

        IF        Dsm = sTiml                                            THEN    Filt_Cmp  <= '1';
        ELSIF    Rx_Dv = '0'    OR (F_Val = '1' AND Rx_Count(5) = '1')    THEN    Filt_Cmp  <= '0';
        END IF;

        IF        Dsm = sTimL        THEN    Mat_Reg <= (OTHERS => '1');
        ELSE
            FOR i IN 0 TO 3 LOOP
                IF    Erg0 /= 0 AND conv_integer(Filt_Idx) = i    THEN    Mat_Reg(4*i + 0) <= '0';    END IF;
                IF    Erg1 /= 0 AND conv_integer(Filt_Idx) = i    THEN    Mat_Reg(4*i + 1) <= '0';    END IF;
                IF    Erg2 /= 0 AND conv_integer(Filt_Idx) = i    THEN    Mat_Reg(4*i + 2) <= '0';    END IF;
                IF    Erg3 /= 0 AND conv_integer(Filt_Idx) = i    THEN    Mat_Reg(4*i + 3) <= '0';    END IF;
            END LOOP;
        END IF;

        IF        Dsm = sTimL                        THEN    Match <= '0';
        ELSIF    Found = '1'                        THEN    Match <= '1';        Match_Desc <= Filt_Idx & M_Prio(1 DOWNTO 0);
            IF        M_Prio(1 DOWNTO 0) = "00"    THEN    Answer_Tx <= TX_0;    Auto_Desc  <= DESC_0;
            ELSIF    M_Prio(1 DOWNTO 0) = "01"    THEN    Answer_Tx <= TX_1;    Auto_Desc  <= DESC_1;
            ELSIF    M_Prio(1 DOWNTO 0) = "10"    THEN    Answer_Tx <= TX_2;    Auto_Desc  <= DESC_2;
            ELSIF    M_Prio(1 DOWNTO 0) = "11"    THEN    Answer_Tx <= TX_3;    Auto_Desc  <= DESC_3;
            END IF;
        ELSIF    F_End = '1'                        THEN    Answer_Tx <= '0';
        END IF;

    END IF;

END PROCESS pFilter;

    R_Req  <= Answer_Tx WHEN F_End = '1' AND F_Err = '0'      ELSE '0';

END BLOCK  bFilter;


    Sel_RxH <= '1'    WHEN s_nWr = '0' AND Sel_Cont = '1' AND s_Adr(3) = '1' AND    s_nBe(1) = '0'    ELSE    '0';
    Sel_RxL <= '1'    WHEN s_nWr = '0' AND Sel_Cont = '1' AND s_Adr(3) = '1' AND    s_nBe(0) = '0'    ELSE    '0';

pRxRegs: PROCESS( Rst, Clk )
BEGIN

    IF    Rst = '1'    THEN
        Rx_Desc <= (OTHERS => '0');     Rx_On  <= '0';
        Rx_Ie   <= '0';    Rx_Lost <= '0';    Rx_Icnt <= (OTHERS => '0'); RxInt <= '0'; Diag  <= '0';
    ELSIF    rising_edge( Clk )    THEN

        IF    Sel_RxH = '1'    THEN
            IF        s_Adr(2 DOWNTO 1) = "00"                        THEN    Rx_Ie  <= S_Din(15);
            ELSIF    s_Adr(2 DOWNTO 1) = "01" AND S_Din(15) = '1'    THEN    Rx_Ie  <= '1';
            ELSIF    s_Adr(2 DOWNTO 1) = "10" AND S_Din(15) = '1'    THEN    Rx_Ie  <= '0';
            END IF;
        END IF;

        IF    Sel_RxH = '1'    THEN
            IF        s_Adr(2 DOWNTO 1) = "00"                        THEN    Diag  <= S_Din(12);
            ELSIF    s_Adr(2 DOWNTO 1) = "01" AND S_Din(12) = '1'    THEN    Diag  <= '1';
            ELSIF    s_Adr(2 DOWNTO 1) = "10" AND S_Din(12) = '1'    THEN    Diag  <= '0';
            END IF;
        END IF;

        IF    Sel_RxL = '1'    THEN
            IF        s_Adr(2 DOWNTO 1) = "00"                        THEN    Rx_On  <= S_Din( 7);
            ELSIF    s_Adr(2 DOWNTO 1) = "01" AND S_Din( 7) = '1'    THEN    Rx_On  <= '1';
            ELSIF    s_Adr(2 DOWNTO 1) = "10" AND S_Din( 7) = '1'    THEN    Rx_On  <= '0';
            END IF;
        END IF;

        IF        Rx_Beg  = '1' AND (RX_OWN = '0' OR Rx_On = '0')                    THEN    Rx_Lost  <= '1';
        ELSIF    Sel_RxL = '1' AND s_Adr(2 DOWNTO 1) = "10" AND S_Din( 4) = '1'    THEN    Rx_Lost  <= '0';
        END IF;

        IF        Sel_RxL = '1' AND s_Adr(2 DOWNTO 1) = "11"            THEN    Rx_Desc <= S_Din( 3 DOWNTO 0);
        ELSIF    Dsm = sLenW AND Desc_We = '1'  THEN
            IF        Last_Desc = '1'                                    THEN    Rx_Desc <= x"0";
            ELSE                                                            Rx_Desc <= Rx_Desc + 1;
            END IF;
        END IF;

        IF        Rx_Ie = '1' AND Desc_We = '1' AND Dsm = sStat        THEN    RxInt <= '1';
        ELSE                                                                RxInt <= '0';
        END IF;

        IF        Sel_RxH = '1' AND s_Adr(2 DOWNTO 1) = "10" AND S_Din(8) = '1'
            AND    Rx_Icnt /= 0                                        THEN    Rx_Icnt <= Rx_Icnt - NOT RxInt;
        ELSIF    RxInt = '1'    AND Rx_Icnt /= "11111"                    THEN    Rx_Icnt <= Rx_Icnt + 1;
        END IF;

    END IF;

END PROCESS pRxRegs;

END BLOCK bRxDesc;

END BLOCK b_Full_Rx;

END ARCHITECTURE struct;