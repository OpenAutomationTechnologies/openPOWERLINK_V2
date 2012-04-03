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
    
    
    signal Cnt_Rx_high                 : std_logic_vector(13 downto 0);
    signal Cnt_RxHigh_ToShort          : std_logic_vector(1 downto 0);
    signal RxHigh_ToShort              : std_logic;
    signal RxLow_ToShort               : std_logic;
    signal RxHigh_ToShort_temp         : std_logic;
    signal RxLow_ToShort_temp          : std_logic;
    signal RxLowGap_ToShort            : std_logic;
    signal RxLowGap_ToShort_temp       : std_logic; 
    signal RxDataValidLatch            : std_logic;  
    signal PortIsEnable                : std_logic;
    signal RxErrOccur                  : std_logic;
    signal RxAnyError                  : std_logic;      
    signal RxTxNotActive               : std_logic;
    signal DisablePort                 : std_logic;
    
    signal RxDel : aRxSetArr;
    
                                        

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
  


    
    RxDvOut         <= RxDel(3).RxDv  and PortIsEnable                    when RxLowGap_ToShort = '1' else
                       RxDel(1).RxDv  and PortIsEnable;  
                       
    RxDatOut        <= RxDel(3).RxDat and (PortIsEnable & PortIsEnable)   when RxLowGap_ToShort = '1' else
                       RxDel(1).RxDat and (PortIsEnable & PortIsEnable);                         
                                                   
                                                         
    
    TxEnOut         <= TxEnIn           and PortIsEnable;
    TxDatOut        <= TxDatIn          and (PortIsEnable & PortIsEnable);
     
                                                                       
                                                                      
    
                                                                                  
    
    RxAnyError      <=  '1'     when (Cnt_Rx_high(Cnt_Rx_high'high) = '1') or (RxHigh_ToShort = '1') or (RxLow_ToShort = '1') or (RxErr = '1') or (RxErrOccur = '1') else
                        '0';
    RxTxNotActive   <=  '1'     when (RxDvIn = '0') and (RxDel(1).RxDv = '0') and (RxDel(2).RxDv = '0') and (TxEnIn = '0') and (TxDatIn = "00") else
                        '0';
    
    -- Port is allowed to be active if RX_DV is not active
    PortIsEnable    <= '1'  when Rst = '1' else
                       '0'  when (RxAnyError = '1') or (DisablePort = '1') else
                       '1';                  

               
do: PROCESS (Rst, Clk)

BEGIN
    if Rst = '1' then                                                                     
        Cnt_RxHigh_ToShort  <= (others => '0');
        RxHigh_ToShort      <= '0';                    RxHigh_ToShort_temp   <= '0';
        RxLow_ToShort       <= '0';                    RxLow_ToShort_temp    <= '0';
        RxLowGap_ToShort    <= '0';                    RxLowGap_ToShort_temp <= '0'; 
        RxDel(3 downto 1)   <= (others => ('0',"00"));
        RxDataValidLatch    <= '0';                          
        Cnt_Rx_high         <= (others => '0');
        RxErrOccur          <= '0';          
        DisablePort         <= '0';
        
    elsif rising_edge(Clk) then
        RxDel(1) <= RxDel(0);
        RxDel(2) <= RxDel(1);
        RxDel(3) <= RxDel(2);
        RxDataValidLatch    <= RxDel(1).RxDv or RxDel(2).RxDv;
        
        
                   
        if    (DisablePort = '0') and (RxAnyError = '1')                            then    DisablePort <= '1';
        elsif (DisablePort = '1') and (RxAnyError = '0') and (RxTxNotActive = '1')  then    DisablePort <= '0';
        else                                                                                DisablePort <= DisablePort;
        end if;
        
        
        
                                                        
 ----------------------------------------------- Pending Error: Block Port for at least 10.24 usec -----------------------------------------------        
        if RxErrOccur = '1' then  
            if RxErr = '1' then                                                                  -- phy error
                Cnt_Rx_high <= (others => '0');                                                  
            else                                                                                 -- other error
                if Cnt_Rx_high(13) = '0' then Cnt_Rx_high <= Cnt_Rx_high + 1;                      -- wait for 163.84 usec
                else                          RxErrOccur  <= '0';
                                              Cnt_Rx_high <= (others => '0');
                end if;                               
            end if;                                   
                                         
                   
                                                 
 ----------------------------------------------- Phy Error -----------------------------------------------   
        elsif RxErr = '1' then                                                                   
             Cnt_Rx_high    <= (others => '0');                                                  -- -> block
             RxErrOccur     <= '1';        
                    
                                                                                             
                                                                                                 
 ----------------------------------------------- RxDv = 1 -----------------------------------------------            
        elsif RxDel(1).RxDv = '1' or RxDel(2).RxDv = '1' then                                        
            if RxLow_ToShort_temp  = '1' then                                                               -- if previous Low Phase too short
                RxLow_ToShort_temp  <= '0';                                                                   --> reset temp error
                RxLow_ToShort       <= '1';                                                                   --> set RxLow_ToShort Error !!!
            end if;                                                                    
                             
            if RxDataValidLatch = '0' then                                                                  -- rising_edge of RxDv    
                Cnt_Rx_high         <= (others => '0');                                                       --> reset counter
                RxHigh_ToShort_temp <= '1';                                                                   --> set temp error
            else                                                                                           
                if Cnt_Rx_high(13) = '0' then Cnt_Rx_high <= Cnt_Rx_high + 1;  end if;                      -- 163.84 usec (maximum size of frames)
                if Cnt_Rx_high(8)  = '1' then RxHigh_ToShort_temp <= '0';      end if;                      --   5.12 usec (minimum size of frames)
            end if;                                                                                           --> reset temp error
                                                                                                                  
                                                                                                                        
            
 ----------------------------------------------- RxDv = 0 -----------------------------------------------    
        elsif RxDel(1).RxDv = '0' or RxDel(2).RxDv = '0' then                                                      
            if RxDataValidLatch = '1' then                                                                  -- falling_edge of RxDv
                if RxHigh_ToShort_temp = '1' then                                                             -- if previous High Phase too short
                    if Cnt_RxHigh_ToShort /= "11"   then    Cnt_RxHigh_ToShort  <= Cnt_RxHigh_ToShort + 1;      --> count Occations
                                                            RxHigh_ToShort      <= '0';                         -- if less than 4 short Frames in a Row -> no error
                    else                                    RxHigh_ToShort      <= '1';                         -- else                                 -> RxHigh_ToShort Error !! 
                    end if;                                                                                      
                else                                                                                          -- if no error  
                    Cnt_RxHigh_ToShort  <= (others => '0');                                                     --> reset Short Frame Counter
                    RxHigh_ToShort      <= '0';                      
                end if;                                                                                           
                RxLow_ToShort_temp  <= '1';                                                                   -- set temp error                 
                RxHigh_ToShort_temp <= '0';                                                                   -- reset previous temp error
                RxLowGap_ToShort    <= '0';
                RxLowGap_ToShort_temp <= '1';
                Cnt_Rx_high         <= "00000000000001";                                                      -- reset Counter (=Low Counter)
            else                                                                                            -- no edge 
                if Cnt_Rx_high(5) = '1'  then RxLow_ToShort_temp <= '0'; end if;                              --  0.64 usec (minimum size of inter frame gap) -> reset tmp error                                                                                                       
                if Cnt_Rx_high(9) = '0'  then    Cnt_Rx_high     <= Cnt_Rx_high + 1;                          --  For 10.24 usec no Frame  
                else                             RxHigh_ToShort  <= '0';                                      --> Reset All Errors
                                                 RxLow_ToShort   <= '0';                                   
                end if;                                          
                
                if Cnt_Rx_high(5 downto 1) = "10111" then RxLowGap_ToShort_temp <= '0'; end if;              -- 920 ns  
                if RxLowGap_ToShort_temp = '1' and RxDvIn = '1' then RxLowGap_ToShort <= '1'; end if;        -- FrameGap > 940 ns -> Insert 2 Clks Delay to Rx                      
                                               
                                                    
                
            end if;
        end if;                
    end if;

END PROCESS do;
end generate;
END rtl;
