------------------------------------------------------------------------------------------------------------------------
-- OpenFILTER
--
-- 	  Copyright (C) 2009 B&R
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
-- Note: RxDv and RxDat have to be synchron to Clk
--	     The following Conditions are checked:
--         RxDV >163.64탎ec HIGH -> invalid
--         RxDV <0.64탎ec LOW -> invalid
--         RxDV 4x <5.12탎ec HIGH -> invalid
--         RxDV >5.12탎ec HIGH -> valid
--		   RxErr HIGH -> invalid
--			if invalid deactivation of port, until RxDv and RxErr > 10.24탎ec low
--
------------------------------------------------------------------------------------------------------------------------
-- Version History
------------------------------------------------------------------------------------------------------------------------
-- 2009-08-07  	V0.01   			Converted from V1.1 to first official version.
-- 2011-07-23  	V0.10	zelenkaj	Consideration of RX Error signal and jitter (converted from V2.3)
-- 2011-08-03  	V0.11	zelenkaj	translated comments
-- 2011-11-18  	V0.12	zelenkaj	bypass filter by generic
-- 2011-11-28	V0.13	zelenkaj	Changed reset level to high-active
-- 2012-04-19   V0.20   zelenkaj    Redesign with fsm, Preamble-check improvement
-- 2012-05-21   V0.21   muelhausens changed timeout of fs_FRMnopre to 660ns
------------------------------------------------------------------------------------------------------------------------

library ieee;                                                                                 
use ieee.std_logic_unsigned.all;                                                             
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;


ENTITY openFILTER is
	Generic (
				bypassFilter		:		boolean := false
			);
    Port    (   Rst					: in    std_logic;
                Clk                 : in    std_logic;
                nCheckShortFrames   : in    std_logic := '0';   -- Rx Port von Hub;
                RxDvIn              : in    std_logic;
                RxDatIn             : in    std_logic_vector(1 downto 0);
                RxDvOut             : out   std_logic;
                RxDatOut            : out   std_logic_vector(1 downto 0);
                TxEnIn              : in    std_logic;
                TxDatIn             : in    std_logic_vector(1 downto 0);
                TxEnOut             : out   std_logic;
                TxDatOut            : out   std_logic_vector(1 downto 0);
                RxErr               : in    std_logic := '0'
            );
END ENTITY openFILTER;

ARCHITECTURE rtl OF openFILTER IS

    type aRxSet is record
      RxDv : std_logic;
      RxDat: std_logic_vector(1 downto 0);
    end record;
    
    type aRxSetArr is array (3 downto 0) of aRxSet;
    type aFiltState is (fs_init, fs_GAP2short, fs_GAPext, fs_GAPok, fs_FRMnopre,
                        fs_FRMpre2short, fs_FRMpreOk, fs_FRM2short, fs_FRMok, fs_FRM2long, fs_BlockAll);
     
    signal FiltState     : aFiltState;
    signal RxDel         : aRxSetArr;
    signal FrameShift    : std_logic;
    signal LastFrameNOK  : std_logic;
    signal StCnt         : std_logic_vector(13 downto 0);
    signal BlockRxPort   : std_logic;

BEGIN

disFilter : if bypassFilter generate
begin
	RxDvOut <= RxDvIn;
	RxDatOut <= RxDatIn;
	TxEnOut <= TxEnIn;
	TxDatOut <= TxDatIn;
end generate;

enFilter : if not bypassFilter generate
begin
    
    -- IN --
    RxDel(0).RxDv  <= RxDvIn;
    RxDel(0).RxDat <= RxDatIn;

    BlockRxPort <= '1' when FiltState = fs_FRMnopre or FiltState = fs_BlockAll or LastFrameNOK = '1' else '0';


  -- OUTPUT MUX --
    RxDvOut  <= '0'              when BlockRxPort = '1' else
                RxDel(3).RxDv    when FrameShift = '1'  else
                RxDel(1).RxDv;

    RxDatOut <= "00"             when BlockRxPort = '1' else
                RxDel(3).RxDat   when FrameShift = '1'  else
                RxDel(1).RxDat;

    TxEnOut  <= TxEnIn;

    TxDatOut <= TxDatIn;



fsm: PROCESS(Rst, Clk)
  VARIABLE RstStCnt : std_logic;
begin 
  if Rst = '1' then
    StCnt               <= (others => '0');
    FiltState           <= fs_init;
    FrameShift          <= '0';
    RxDel(3 downto 1)   <= (others => ('0',"00"));
    LastFrameNOK        <= '0';
  elsif rising_edge(Clk) then
   RxDel(3 downto 1) <= RxDel(2 downto 0);


  -- DEFAULT --
   RstStCnt := '0';

   case FiltState is
-------------------------------- INIT ---------------------------------------
     when fs_init =>
       FiltState <= fs_GAP2short; RstStCnt  := '1'; 



-------------------------------- GAP 2 SHORT --------------------------------
     when fs_GAP2short =>
       FrameShift <= '0';
       IF StCnt(4) = '1'              then FiltState <= fs_GAPext;                      END IF;   -- 360ns 
       IF RxDel(0).RxDv = '1'         then FiltState <= fs_BlockAll;   RstStCnt := '1'; END IF;   -- Gap < 360 ns -> too short -> Block Filter


-------------------------------- GAP EXTEND ---------------------------------
     when fs_GAPext =>
       IF StCnt(5 downto 0) = "101110"   then FiltState <= fs_GAPok;                     END IF;
       IF RxDel(0).RxDv = '1' then                                                                -- GAP [360ns .. 960ns] -> short, but ok -> Start Frame
            RstStCnt := '1';
            FrameShift <= '1';
            IF RxDel(0).RxDat = "01" then FiltState <= fs_FRMpre2short;                            -- GAP > 960ns -> OK -> Start Frame (preamble already beginning)
            ELSE                          FiltState <= fs_FRMnopre;                                -- GAP > 960ns -> OK -> Start Frame and wait of preamble
            END IF;
       END IF;


-------------------------------- GAP OK -------------------------------------
     when fs_GAPok =>
        IF RxDel(0).RxDv = '1' then
            RstStCnt := '1';
            IF RxDel(0).RxDat = "01" then FiltState <= fs_FRMpre2short;                            -- GAP > 960ns -> OK -> Start Frame (preamble already beginning)
            ELSE                          FiltState <= fs_FRMnopre;                                -- GAP > 960ns -> OK -> Start Frame and wait of preamble
            END IF;
        END IF;   



-------------------------------- FRAME, BUT STILL NO PREAMBLE ---------------
     when fs_FRMnopre =>
       IF StCnt(5) = '1' or                                                                       -- no preamble for >=660 ns     -> Block Filter
          RxDel(0).RxDat = "11" or  RxDel(0).RxDat = "10" or                                      -- preamble wrong               -> Block Filter
          (RxDel(0).RxDv = '0'  and RxDel(1).RxDv = '0')
                                    then FiltState <= fs_BlockAll;     RstStCnt := '1';
       elsif RxDel(0).RxDat = "01"  then FiltState <= fs_FRMpre2short; RstStCnt := '1';           -- preamble starts              -> Check Preamble
       END IF;

-------------------------------- FRAME CHECK PREAMBLE TOO SHORT --------------
     when fs_FRMpre2short =>
       IF RxDel(0).RxDat /= "01" or                                                               -- preamble wrong               -> Block Filter
          (RxDel(0).RxDv = '0'   and RxDel(1).RxDv = '0')
                                    then FiltState <= fs_BlockAll;     RstStCnt := '1'; 
       ELSIF StCnt(3) = '1'         then FiltState <= fs_FRMpreOk;                      END IF;   -- preamble ok for 180 ns       -> Preamble OK


-------------------------------- FRAME CHECK PREAMBLE OK ---------------
     when fs_FRMpreOk => 
       IF RxDel(0).RxDat /= "01"                  then FiltState <= fs_FRMok;                     END IF;   -- preamble done                -> Start Frame
       IF (StCnt(5) = '1' and StCnt(2) = '1') or                                                            -- preamble to long for 740 ns  -> Block Filter
          (RxDel(0).RxDv = '0' and RxDel(1).RxDv = '0')
                                                  then FiltState <= fs_BlockAll; RstStCnt := '1'; END IF;
       LastFrameNOK <= '0';                                                                                 -- preamble is OK


-------------------------------- FRAME OK -----------------------------------
     when fs_FRMok     =>
       IF StCnt(13) = '1'           then FiltState <= fs_BlockAll;     RstStCnt := '1'; END IF;   -- FRAME > 163,842 us -> too long      -> Block Filter
       IF RxDel(0).RxDv = '0' and
          RxDel(1).RxDv = '0'       then FiltState <= fs_GAP2short;    RstStCnt := '1'; END IF;   -- FRAME [163,842 us] -> OK -> Start GAP


-------------------------------- Block Filter -------------------------------
     when fs_BlockAll   =>
       IF StCnt(2) = '1'            then FiltState <= fs_GAP2short;    RstStCnt := '1'; END IF;   -- Block for 100 nsec
       IF RxDel(0).RxDv = '1'       then                               RstStCnt := '1'; END IF;   -- RxDv != '0' -> Reset Wait Period
       LastFrameNOK <= '1';                                                                       -- block next rx frame (until receive a valid preamble)

     when others =>
       FiltState <= fs_init;
   end case;


   IF RxErr = '1'                   then FiltState <= fs_BlockAll;     RstStCnt := '1'; END IF;   -- RxErr -> Block Filter



  -- State Counter --
   StCnt <= StCnt + 1;
   if RstStCnt = '1' then StCnt <= (others => '0'); end if;

  end if;
end process;
    
end generate;
END rtl;
