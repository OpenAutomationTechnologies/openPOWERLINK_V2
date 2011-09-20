------------------------------------------------------------------------------------------------------------------------
-- Avalon Interface of OpenMAC to use with NIOSII
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
------------------------------------------------------------------------------------------------------------------------
-- Version History
------------------------------------------------------------------------------------------------------------------------
-- 2009-05-20  V0.01        First version
-- 2009-06-04  V0.10        Implemented FIFO for Data-Queing form/to DMA
-- 2009-06-15  V0.11        Increased performance of FIFO
-- 2009-06-20  V0.20        New FIFO concept. (FIFO IP of Altera used)
-- 2009-06-26  V0.21        Little Bugfix of DMA -> Reset was handled wrong
-- 2009-08-07  V0.30        Converted to official version
-- 2009-08-21  V0.40		TX DMA run if fifo is not empty. Interface for Timer Cmp + IRQ
-- 2009-09-03  V0.50		RX FIFO is definitely empty when a new frame arrives (Fifo sclr is set for 1 cycle)
-- 2009-09-07  V0.60		Added openFilter and openHub. Some changes in Mii core map. Added 2nd RMii Port.
-- 2009-09-15  V0.61		Added ability to read the Mac Time over Time Cmp Slave Interface (32 bits).
-- 2009-09-18  V0.62		Deleted in Phy Mii core NodeNr port. 
-- 2010-04-01  V0.63		Added Timer triggered transmission ability
--							RXFifo Clr is done at end of RxFrame (not beginning! refer to V0.50)
--							Added "CrsDv Filter" (deletes CrsDv toggle)
-- 2010-04-26  V0.70		reduced to two Avalon Slave and one Avalon Master Interface
-- 2010-05-03  V0.71		omit Avalon Master Interface / use internal DPR
-- 2010-08-02  V0.72		Enabled Timer triggered TX functionality (just adding generic TxSyncOn)
-- 2010-08-19  V0.73		Filter for phy ports
--							100MHz Clk for RMII ports (better for timing)
-- 2010-09-07  V0.74		Bugfix: Rx packets are not stored to DPRAM (Dma_Dout/Dma_Dout_s mixed up)
-- 2010-09-13  V0.75		added selection Rmii / Mii
------------------------------------------------------------------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.std_logic_arith.all;
USE ieee.std_logic_unsigned.all;

ENTITY Openmac_Xilinx_IF IS
   GENERIC(             Simulate                    : 		boolean := false;
   			iBufSize_g                  : 		integer := 1024;
   			iBufSizeLOG2_g		    : 		integer := 10;
			useRmii_g		    : 		boolean := false);
   PORT (               Reset_n		            : IN    STD_LOGIC;
			Clk50                  	    : IN    STD_LOGIC;
			ClkFaster		    : IN    STD_LOGIC;
			ClkEth			    : IN    STD_LOGIC;

                       --Mii Core
	 ---SMI
	 Mii_D_I                   : IN     std_logic_vector(1 downto 0);
	 Mii_D_O                   : OUT    std_logic_vector(1 downto 0);
	 Mii_D_T                   : OUT    std_logic;
	 Mii_Clk                   : OUT    std_logic;
         
            ---Phy Reset
         nResetOut                 : OUT    std_logic;
         TX_IR_n                   : OUT    std_logic;
	 RX_IR_n                   : OUT    std_logic;

        wr_n            	    : in   std_logic                      ;
	mac_addr        	    : in   std_logic_vector(10 downto 1)  ;
	mac_be_n        	    : in   std_logic_vector(1 downto 0)   ;
	mac_indata16    	    : in   std_logic_vector(15 downto 0)  ;
	mac_outdata16   	    : out  std_logic_vector(15 downto 0)  ;
	mac_selCont     	    : in   std_logic                      ;
	mac_selRam      	    : in   std_logic                      ;
	mii_addr        	    : in   std_logic_vector(2 downto 0)   ;
	mii_be_n        	    : in   std_logic_vector(1 downto 0)   ;
	mii_indata16    	    : in   std_logic_vector(15 downto 0)  ;
	mii_outdata16   	    : out  std_logic_vector(15 downto 0)  ;
	mii_sel         	    : in   std_logic                      ;

         s_Dma_Req                  : out  std_logic ;
	 s_Dma_Rw    	            : out  std_logic ;
	 s_Dma_Ack                  : in   std_logic ;
	 s_Dma_Addr                 : out  std_logic_vector(31 DOWNTO 1) ;
	 s_Dma_Dout                 : out  std_logic_vector(15 DOWNTO 0) ;
	 s_Dma_Din                  : in   std_logic_vector(15 DOWNTO 0) ;
	  Mac_Zeit		    : out  std_logic_vector(31 downto 0) ;
	
		-- RMII Port 0
            rRx_Dat_0                   : IN    STD_LOGIC_VECTOR(1 DOWNTO 0);  -- RMII Rx Daten
            rCrs_Dv_0                   : IN    STD_LOGIC;                     -- RMII Carrier Sense / Data Valid
            rTx_Dat_0                   : OUT   STD_LOGIC_VECTOR(1 DOWNTO 0);  -- RMII Tx Daten
            rTx_En_0                    : OUT   STD_LOGIC;                     -- RMII Tx_Enable
		-- RMII Port 1
            rRx_Dat_1                   : IN    STD_LOGIC_VECTOR(1 DOWNTO 0);  -- RMII Rx Daten
            rCrs_Dv_1                   : IN    STD_LOGIC;                     -- RMII Carrier Sense / Data Valid
            rTx_Dat_1                   : OUT   STD_LOGIC_VECTOR(1 DOWNTO 0);  -- RMII Tx Daten
            rTx_En_1                    : OUT   STD_LOGIC;                     -- RMII Tx_Enable
		--- MII PORTS
			phyMii0_RxClk		    : in    std_logic	;
			phyMii0_RxDat           : in    std_logic_vector(3 downto 0);
			phyMii0_RxDv            : in    std_logic	;
			phyMii0_TxClk		    : in    std_logic	;
			phyMii0_TxDat           : out   std_logic_vector(3 downto 0);
			phyMii0_TxEn            : out   std_logic	;
			phyMii0_Crs				: in	std_logic	;
			
			phyMii0_TxEr            : out   std_logic	;
			phyMii0_RxErr			: in	std_logic	;
			phyMii0_Col				: in	std_logic	;
			
			phyMii1_RxClk		    : in    std_logic;
			phyMii1_RxDat           : in    std_logic_vector(3 downto 0);
			phyMii1_RxDv            : in    std_logic;
			phyMii1_TxClk		    : in    std_logic;
			phyMii1_TxDat           : out   std_logic_vector(3 downto 0);
			phyMii1_TxEn            : out   std_logic;
			phyMii1_Crs             : in   std_logic;
			
			phyMii1_TxEr            : out   std_logic	;
			phyMii1_RxErr			: in	std_logic	;
			phyMii1_Col				: in	std_logic	;

			MacRxDv                 : out std_logic        ;
			MacTxEn                 : out std_logic        


        );
END ENTITY Openmac_Xilinx_IF;

ARCHITECTURE struct OF Openmac_Xilinx_IF IS

component OpenFILTER IS
    PORT    (   nRst                : IN	std_logic;
                Clk                 : IN    std_logic;
                nCheckShortFrames   : IN    std_logic := '0';
                RxDvIn              : IN    std_logic;
                RxDatIn             : IN    std_logic_vector(1 DOWNTO 0);
                RxDvOut             : OUT   std_logic;
                RxDatOut            : OUT   std_logic_vector(1 DOWNTO 0);
                TxEnIn              : IN    std_logic;
                TxDatIn             : IN    std_logic_vector(1 DOWNTO 0);
                TxEnOut             : OUT   std_logic;
                TxDatOut            : OUT   std_logic_vector(1 DOWNTO 0)
			);
end component OpenFILTER ;

component OpenHUB IS
	GENERIC	( Ports				:		integer := 3 );
	PORT	( nRst				: IN	std_logic;
			  Clk				: IN	std_logic;
			  RxDv				: IN	std_logic_vector(Ports DOWNTO 1);
			  RxDat0, RxDat1	: IN	std_logic_vector(Ports DOWNTO 1);
			  TxEn				: OUT	std_logic_vector(Ports DOWNTO 1);
			  TxDat0, TxDat1	: OUT	std_logic_vector(Ports DOWNTO 1);
			  internPort		: IN	integer RANGE 1 TO Ports := 1;
			  TransmitMask		: IN	std_logic_vector(Ports DOWNTO 1) := (OTHERS => '1');
			  ReceivePort		: OUT	integer RANGE 0 TO Ports
			);
END component OpenHUB;


component rmii_2_mii is
	port (
		---testports---
		test_rx_fifo_ae : out std_logic;
		test_rx_fifo_full : out std_logic;
		test_rx_fifo_empty : out std_logic;
		test_rx_fifo_valid : out std_logic;
		test_rx_fifo_wr : out std_logic;
		test_rx_fifo_rd : out std_logic;
		test_rx_fifo_din : out std_logic_vector(3 downto 0);
		test_rx_fifo_dout : out std_logic_vector(1 downto 0);
		
		test_tx_fifo_ae : out std_logic;
		test_tx_fifo_full : out std_logic;
		test_tx_fifo_empty : out std_logic;
		test_tx_fifo_valid : out std_logic;
		test_tx_fifo_wr : out std_logic;
		test_tx_fifo_rd : out std_logic;
		test_tx_fifo_din : out std_logic_vector(1 downto 0);
		test_tx_fifo_dout : out std_logic_vector(3 downto 0);
		---testports---
		clk50 		: in std_logic;
		rst 		: in std_logic;
		--RMII MAC
		rCrsDv		: out std_logic := '0';
		rRxErr		: out std_logic := '0';
		rRxD		: out std_logic_vector(1 downto 0) := (others => '0');
		rTxEn		: in  std_logic;
		rTxD		: in  std_logic_vector(1 downto 0);
		--MII PHY
		mDv			: in  std_logic;
		mRxD		: in  std_logic_vector(3 downto 0);
		mRxErr		: in  std_logic;
		mRxClk		: in  std_logic;
		mCrs		: in  std_logic;
		mCol		: in  std_logic;
		mTxEn		: out std_logic := '0';
		mTxD		: out std_logic_vector(3 downto 0) := (others => '0');
		mTxErr		: out std_logic := '0';
		mTxClk		: in  std_logic
	);
end component rmii_2_mii;

component OpenMAC IS
	GENERIC( HighAdr                : IN    integer := 16;
			 Timer                  : IN    boolean := false;
			 TxSyncOn				: IN	boolean := false;
			 TxDel					: IN	boolean := false;
             Simulate               : IN    boolean := false
           );
	PORT ( nRes, Clk            : IN    std_logic;
           -- Processor
           s_nWr, Sel_Ram, Sel_Cont : IN    std_logic := '0';
           S_nBe                    : IN    std_logic_vector( 1 DOWNTO 0);
           S_Adr                    : IN    std_logic_vector(10 DOWNTO 1);
           S_Din                    : IN    std_logic_vector(15 DOWNTO 0);
           S_Dout                   : OUT   std_logic_vector(15 DOWNTO 0);
           nTx_Int, nRx_Int			: OUT   std_logic;
           nTx_BegInt               : OUT   std_logic;
           -- DMA
           Dma_Req, Dma_Rw          : OUT   std_logic;
           Dma_Ack                  : IN    std_logic;
           Dma_Addr                 : OUT   std_logic_vector(HighAdr DOWNTO 1);
           Dma_Dout                 : OUT   std_logic_vector(15 DOWNTO 0);
           Dma_Din                  : IN    std_logic_vector(15 DOWNTO 0);
           -- RMII
           rRx_Dat                  : IN    std_logic_vector( 1 DOWNTO 0);
           rCrs_Dv                  : IN    std_logic;
           rTx_Dat                  : OUT   std_logic_vector( 1 DOWNTO 0);
           rTx_En                   : OUT   std_logic;
           Hub_Rx                   : IN    std_logic_vector( 1 DOWNTO 0) := "00";
           Mac_Zeit		    : OUT   std_logic_vector(31 DOWNTO 0)
         );
END component;

component OpenMAC_MII IS
	PORT(	Clk			: IN	std_logic;						
			nRst		: IN	std_logic;						
			Addr		: IN	std_logic_vector( 2 DOWNTO 0);	
			Sel			: IN	std_logic;						
			nBe			: IN	std_logic_vector( 1 DOWNTO 0);	
			nWr			: IN	std_logic;						
			Data_In		: IN	std_logic_vector(15 DOWNTO 0);	
			Data_Out	: OUT	std_logic_vector(15 DOWNTO 0);	
			Mii_Clk		: OUT	std_logic;
--			Mii_Dio		: InOut	std_logic;
			Mii_Di : IN std_logic;
			Mii_Do : out std_logic;
			Mii_Doe : out std_logic;
			nResetOut	: OUT	std_logic;
			NodeNr		: IN	std_logic_vector(7 downto 0) := (others => '0')
		);
END component;


	signal rst		        	: std_logic;
--Hub Signals
	SIGNAL  HubTxEn			 	: STD_LOGIC_VECTOR(3 DOWNTO 1);
	SIGNAL  HubTxDat0		        : STD_LOGIC_VECTOR(3 DOWNTO 1);
	SIGNAL  HubTxDat1			: STD_LOGIC_VECTOR(3 DOWNTO 1);
	SIGNAL  HubRxDv				: STD_LOGIC_VECTOR(3 DOWNTO 1);
	SIGNAL  HubRxDat0			: STD_LOGIC_VECTOR(3 DOWNTO 1);
	SIGNAL  HubRxDat1			: STD_LOGIC_VECTOR(3 DOWNTO 1);
	SIGNAL RxPortInt			: integer RANGE 0 TO 3; --0 is idle
	SIGNAL RxPort				: STD_LOGIC_VECTOR(1 downto 0);
--Filter0 Signals
	SIGNAL Flt0TxEn				: STD_LOGIC;
	SIGNAL Flt0TxDat			: STD_LOGIC_VECTOR(1 downto 0);
	SIGNAL Flt0RxDv				: STD_LOGIC;
	SIGNAL Flt0RxDat			: STD_LOGIC_VECTOR(1 downto 0);
--Filter1 Signals
	SIGNAL Flt1TxEn				: STD_LOGIC;
	SIGNAL Flt1TxDat			: STD_LOGIC_VECTOR(1 downto 0);
	SIGNAL Flt1RxDv				: STD_LOGIC;
	SIGNAL Flt1RxDat			: STD_LOGIC_VECTOR(1 downto 0);
--Phy0 Signals
	SIGNAL Phy0TxEn				: STD_LOGIC;
	SIGNAL Phy0TxDat			: STD_LOGIC_VECTOR(1 downto 0);
	SIGNAL Phy0RxDv				: STD_LOGIC;
	SIGNAL Phy0RxDat			: STD_LOGIC_VECTOR(1 downto 0);
--Phy1 Signals
	SIGNAL Phy1TxEn				: STD_LOGIC;
	SIGNAL Phy1TxDat			: STD_LOGIC_VECTOR(1 downto 0);
	SIGNAL Phy1RxDv				: STD_LOGIC;
	SIGNAL Phy1RxDat			: STD_LOGIC_VECTOR(1 downto 0);
-- Mii Signals
	--SIGNAL mii_Doei				: STD_LOGIC;

--Added for new porting
	SIGNAL 	s_MacRxDv			: STD_LOGIC;
	SIGNAL  s_MacRxDat                      : STD_LOGIC_VECTOR(1 downto 0);
	SIGNAL	s_MacTxEn			: STD_LOGIC;
	SIGNAL	s_MacTxDat			: STD_LOGIC_VECTOR(1 downto 0);
    SIGNAL  s_phyMii0_RxClk		        : std_logic;
	SIGNAL	s_phyMii0_RxDat                 : std_logic_vector(3 downto 0);
    SIGNAL 	s_phyMii0_RxDv                  : std_logic;
	
	SIGNAL	s_phyMii0_TxClk		        : std_logic;
	SIGNAL	s_phyMii0_TxDat                 : std_logic_vector(3 downto 0);
	SIGNAL	s_phyMii0_TxEn                  : std_logic;
	
	SIGNAL	s_phyMii1_RxClk		        : std_logic;
	SIGNAL	s_phyMii1_RxDat                 : std_logic_vector(3 downto 0);
	SIGNAL	s_phyMii1_RxDv                  : std_logic;
	SIGNAL	s_phyMii1_TxClk		        : std_logic;
	SIGNAL	s_phyMii1_TxDat                 : std_logic_vector(3 downto 0);
	SIGNAL	s_phyMii1_TxEn                  : std_logic;
	
	SIGNAL	s_phyMii1_RxErr					: std_logic ;
	SIGNAL	s_phyMii1_TxEr                  : std_logic;
	SIGNAL	s_phyMii1_Crs					: std_logic ;
	SIGNAL	s_phyMii1_Col					: std_logic ;
	
	SIGNAL	s_phyMii0_RxErr					: std_logic ;
	SIGNAL	s_phyMii0_TxEr                  : std_logic;
	SIGNAL	s_phyMii0_Crs					: std_logic ;
	SIGNAL	s_phyMii0_Col					: std_logic ;
        
	signal Mac_Mii_D_I      	     : std_logic;
	signal Mac_Mii_D_O      	     : std_logic;
	signal Mac_Mii_D_T      	     : std_logic;

BEGIN
	
	rst          <=   not Reset_n;

	--Concurrent Assignments
        MacRxDv      <=   s_MacRxDv;
--	MacRxDat     <=   s_MacRxDat;

	MacTxEn    <=   s_MacTxEn;
	--s_MacTxDat   <=   MacTxDat;

        	
        s_phyMii0_RxClk	 <=    phyMii0_RxClk  ;
        s_phyMii0_RxDat  <=    phyMii0_RxDat  ;
        s_phyMii0_RxDv   <=    phyMii0_RxDv   ;
        s_phyMii0_TxClk	 <=    phyMii0_TxClk  ; 
        phyMii0_TxDat    <=    s_phyMii0_TxDat; 
        phyMii0_TxEn     <=    s_phyMii0_TxEn ; 
        phyMii0_TxEr     <=    s_phyMii0_TxEr ; 
        s_phyMii1_RxClk  <=    phyMii1_RxClk  ;
        s_phyMii1_RxDat  <=    phyMii1_RxDat  ;
        s_phyMii1_RxDv   <=    phyMii1_RxDv   ;
        s_phyMii1_TxClk	 <=    phyMii1_TxClk  ;
        phyMii1_TxDat    <=    s_phyMii1_TxDat; 
        phyMii1_TxEn     <=    s_phyMii1_TxEn ; 
        phyMii1_TxEr     <=    s_phyMii1_TxEr ;
		
		s_phyMii0_RxErr	 <=	   phyMii0_RxErr  ;
		s_phyMii0_Crs	 <=    phyMii0_Crs	  ;
		s_phyMii0_Col	 <=	   phyMii0_Col    ;

		s_phyMii1_RxErr	 <=	   phyMii1_RxErr  ;
		s_phyMii1_Crs	 <=    phyMii1_Crs	  ;
		s_phyMii1_Col	 <=	   phyMii1_Col    ;
	
	the_Hub : OpenHub
		generic map (	Ports => 3
		)
		port map 	(
						nRst 			=> 	Reset_n,
						Clk 			=> 	Clk50,
						RxDv 			=> 	HubRxDv,
						RxDat0 			=> 	HubRxDat0,
						RxDat1 			=> 	HubRxDat1,
						TxEn 			=> 	HubTxEn,
						TxDat0 			=> 	HubTxDat0,
						TxDat1 			=> 	HubTxDat1,
						internPort 		=> 	1,
						TransmitMask 		=> 	(others => '1'),
						ReceivePort 		=> 	RxPortInt
		);
	RxPort <= conv_std_logic_vector(RxPortInt, RxPort'length);
	HubRxDv <= Flt1RxDv & Flt0RxDv & s_MacTxEn;
	HubRxDat1 <= Flt1RxDat(1) & Flt0RxDat(1) & s_MacTxDat(1);
	HubRxDat0 <= Flt1RxDat(0) & Flt0RxDat(0) & s_MacTxDat(0);
	Flt1TxEn <= HubTxEn(3);
	Flt0TxEn <= HubTxEn(2);
	s_MacRxDv <= HubTxEn(1);
	Flt1TxDat(1) <= HubTxDat1(3);
	Flt0TxDat(1) <= HubTxDat1(2);
	s_MacRxDat(1) <= HubTxDat1(1);
	Flt1TxDat(0) <= HubTxDat0(3);
	Flt0TxDat(0) <= HubTxDat0(2);
	s_MacRxDat(0) <= HubTxDat0(1);
	
	the_Filter4Phy0 : OpenFILTER
		port map	(
						nRst              => Reset_n,
						Clk               => Clk50,
						nCheckShortFrames => '0',
						RxDvIn		  => Phy0RxDv,
						RxDatIn 	  => Phy0RxDat,
						RxDvOut 	  => Flt0RxDv,
						RxDatOut 	  => Flt0RxDat,
						TxEnIn 		  => Flt0TxEn,
						TxDatIn		  => Flt0TxDat,
						TxEnOut 	  => Phy0TxEn,
						TxDatOut 	  => Phy0TxDat
				);
	
	the_Filter4Phy1 :  OpenFILTER
		port map	(
						nRst 		  => Reset_n,
						Clk 		  => Clk50,
						nCheckShortFrames => '0',
						RxDvIn 		  => Phy1RxDv,
						RxDatIn 	  => Phy1RxDat,
						RxDvOut 	  => Flt1RxDv,
						RxDatOut	  => Flt1RxDat,
						TxEnIn 	  	  => Flt1TxEn,
						TxDatIn 	  => Flt1TxDat,
						TxEnOut 	  => Phy1TxEn,
						TxDatOut 	  => Phy1TxDat
		);

	genRmii : if useRmii_g generate
		regPhy100Meg : process(ClkEth, Reset_n)
		--latches tx signals to phy with falling edge of 100MHz clk
		begin
			if Reset_n = '0' then
				rTx_En_0 <= '0';
				rTx_Dat_0 <= (others => '0');
				rTx_En_1 <= '0';
				rTx_Dat_1 <= (others => '0');
			elsif ClkEth = '0' and ClkEth'event then
				rTx_En_0 <= Phy0TxEn;
				rTx_Dat_0 <= Phy0TxDat;
				rTx_En_1 <= Phy1TxEn;
				rTx_Dat_1 <= Phy1TxDat;
			end if;
		end process;
		
		regPhy50Meg : process(clk50, Reset_n)
		--latches rx signals from phy with rising edge of 100MHz clk
		begin
			if Reset_n = '0' then
				Phy0RxDv <= '0';
				Phy0RxDat <= (others => '0');
				Phy1RxDv <= '0';
				Phy1RxDat <= (others => '0');
			elsif clk50 = '1' and clk50'event then
				Phy0RxDv <= rCrs_Dv_0;
				Phy0RxDat <= rRx_Dat_0;
				Phy1RxDv <= rCrs_Dv_1;
				Phy1RxDat <= rRx_Dat_1;
			end if;
		end process;
	end generate;	
    		
	geMii : if not useRmii_g generate	
		--s_phyMii0_TxEr <= '0';
		theRmii2MiiCnv0 :  rmii_2_mii
			port map (
				clk50				=> Clk50,
				rst					=> rst,
				--RMII (MAC)
				rCrsDv				=> Phy0RxDv, 
				rRxErr				=> open,
				rRxD				=> Phy0RxDat ,
				
				rTxEn				=> Phy0TxEn ,
				rTxD				=> Phy0TxDat ,
				--MII (PHY)
				mTxEn				=> s_phyMii0_TxEn,
				mTxD				=> s_phyMii0_TxDat,
				mTxErr				=> s_phyMii0_TxEr, 
				mTxClk				=> s_phyMii0_TxClk,
				
				mDv				=> s_phyMii0_RxDv,
				mRxD				=> s_phyMii0_RxDat,
				mRxErr				=> s_phyMii0_RxErr,
				mRxClk				=> s_phyMii0_RxClk,
				mCrs				=> s_phyMii0_Crs,
				mCol				=> s_phyMii0_Col
				
			);
		
		--s_phyMii1_TxEr <= '0';
		theRmii2MiiCnv1 :  rmii_2_mii
			port map (
				clk50				=> Clk50,
				rst				=> rst,
				--RMII (MAC)
				rCrsDv				=> Phy1RxDv, 
				rRxErr				=> open,
				rRxD				=> Phy1RxDat ,
				
				rTxEn				=> Phy1TxEn ,
				rTxD				=> Phy1TxDat ,

				--MII (PHY)
				mTxEn				=> s_phyMii1_TxEn,
				mTxD				=> s_phyMii1_TxDat,
				mTxErr				=> s_phyMii1_TxEr,
				mTxClk				=> s_phyMii1_TxClk,
				
				mDv				=> s_phyMii1_RxDv,
				mRxD				=> s_phyMii1_RxDat,
				mRxErr				=> s_phyMii1_RxErr,
				mRxClk				=> s_phyMii1_RxClk,
				mCrs				=> s_phyMii1_Crs,
				mCol				=> s_phyMii1_Col
			);
	end generate;


      core_eplmac : OpenMAC
      generic map (
         HighAdr  => 31,
         Timer    => true,
		 TxSyncOn => false,
		 TxDel	  => false,
         Simulate => false
      )
      port map (
		nRes 		=> 	not rst		,
		Clk 		=> 	Clk50		,
		-- Processor
		s_nWr 		=> 	wr_n		,
		Sel_Ram 	=> 	mac_selRam	,
		Sel_Cont 	=> 	mac_selCont	,
		S_nBe 		=> 	mac_be_n	,
		S_Adr 		=> 	mac_addr	,
		S_Din 		=> 	mac_indata16,
		S_Dout 		=> 	mac_outdata16,
		nTx_Int 	=> 	TX_IR_n		,
		nRx_Int 	=> 	RX_IR_n		,
		nTx_BegInt	=> 	open		,
		-- DMA
		Dma_Req 	=> 	s_Dma_Req	,	
		Dma_Rw 		=> 	s_Dma_Rw	,
		Dma_Ack 	=> 	s_Dma_Ack	,
		Dma_Addr 	=> 	s_Dma_Addr(31 downto 1),
		Dma_Dout 	=> 	s_Dma_Dout	,
		Dma_Din 	=> 	s_Dma_Din	,
		-- RMII
		rRx_Dat 	=> 	s_MacRxDat	,
		rCrs_Dv 	=> 	s_MacRxDv	,
		rTx_Dat 	=> 	s_MacTxDat	,
		rTx_En 		=> 	s_MacTxEn	,
		Hub_Rx 		=> 	(others => '0') ,
		Mac_Zeit 	=> 	Mac_Zeit
		);
	
	core_eplmii : OpenMAC_MII
        port map (
			Clk 		=> 	clk50	        ,
			nRst 		=> 	not rst		,
			Addr 		=> 	mii_addr	,
			Sel 		=> 	mii_sel		,
			nBe 		=> 	mii_be_n	,
			nWr 		=> 	wr_n		,
			Data_In 	=> 	mii_indata16    ,
			Data_Out	=> 	mii_outdata16   ,
			Mii_Clk 	=> 	Mii_Clk		,
			--			Mii_Dio		: InOut	std_logic;
			Mii_Di 		=> 	Mac_Mii_D_I	,
			Mii_Do 		=> 	Mac_Mii_D_O	,
			Mii_Doe 	=> 	Mac_Mii_D_T	,
			nResetOut 	=> 	nResetOut	,
			NodeNr 		=> 	open
        );
	
	Mac_Mii_D_I     <= Mii_D_I(1) and Mii_D_I(0);
	Mii_D_O(1) 	<= Mac_Mii_D_O;
	Mii_D_O(0) 	<= Mac_Mii_D_O;
	Mii_D_T 	<= Mac_Mii_D_T;
		
END ARCHITECTURE struct;
