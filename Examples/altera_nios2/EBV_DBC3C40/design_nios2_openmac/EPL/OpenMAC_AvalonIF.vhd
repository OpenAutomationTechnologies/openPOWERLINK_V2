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
------------------------------------------------------------------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.std_logic_arith.all;
USE ieee.std_logic_unsigned.all;

ENTITY AlteraOpenMACIF IS
   GENERIC( MacTimer                    : IN    boolean := true;
            Simulate                    : IN    boolean := false);
   PORT (   Reset_n						: IN    STD_LOGIC;
			Clk                  		: IN    STD_LOGIC;
		-- Avalon Slave Interface 
            chipselect                  : IN    STD_LOGIC;
            read_n						: IN    STD_LOGIC;
            write_n						: IN    STD_LOGIC;
            byteenable                  : IN    STD_LOGIC_VECTOR(1 DOWNTO 0);
            address                     : IN    STD_LOGIC_VECTOR(10 DOWNTO 0);
            writedata                   : IN    STD_LOGIC_VECTOR(15 DOWNTO 0);
            readdata                    : OUT   STD_LOGIC_VECTOR(15 DOWNTO 0);
            nTx_Int						: OUT   STD_LOGIC;
		-- Avalon Master Interface (the_DMAAvalonIF)
            m_read_n					: OUT   STD_LOGIC;
            m_write_n					: OUT   STD_LOGIC;
            m_byteenable_n              : OUT   STD_LOGIC_VECTOR(1 DOWNTO 0);
            m_address                   : OUT   STD_LOGIC_VECTOR(31 DOWNTO 0);
            m_writedata                 : OUT   STD_LOGIC_VECTOR(15 DOWNTO 0);
            m_readdata                  : IN    STD_LOGIC_VECTOR(15 DOWNTO 0);
            m_waitrequest               : IN    STD_LOGIC;
            m_arbiterlock				: OUT   STD_LOGIC;
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
		-- Serial Management Interface (the_Mii)
			mii_Addr					: IN	STD_LOGIC_VECTOR(2 DOWNTO 0);	
			mii_Sel						: IN	STD_LOGIC;						
			mii_nBe						: IN	STD_LOGIC_VECTOR(1 DOWNTO 0);	
			mii_nWr						: IN	STD_LOGIC;						
			mii_Data_In					: IN	STD_LOGIC_VECTOR(15 DOWNTO 0);	
			mii_Data_Out				: OUT	STD_LOGIC_VECTOR(15 DOWNTO 0);	
			mii_Clk						: OUT	STD_LOGIC;
			mii_Di						: IN	STD_LOGIC;
			mii_Do						: OUT	STD_LOGIC;
			mii_Doe						: OUT	STD_LOGIC;
			mii_nResetOut				: OUT	STD_LOGIC;
		-- Dummy Interface (for TX IRQ) for newer SOPC builders
			d_chipselect                : IN    STD_LOGIC;
            d_read_n					: IN    STD_LOGIC;
            d_write_n					: IN    STD_LOGIC;
            d_byteenable                : IN    STD_LOGIC_VECTOR(1 DOWNTO 0);
            d_address                   : IN    STD_LOGIC_VECTOR(0 DOWNTO 0);
            d_writedata                 : IN    STD_LOGIC_VECTOR(15 DOWNTO 0);
            d_readdata                  : OUT   STD_LOGIC_VECTOR(15 DOWNTO 0);
			nRx_Int						: OUT   STD_LOGIC;
		-- MAC Timer Cmp Interface
			t_chipselect                : IN    STD_LOGIC;
            t_read_n					: IN    STD_LOGIC;
            t_write_n					: IN    STD_LOGIC;
            t_byteenable                : IN    STD_LOGIC_VECTOR(3 DOWNTO 0);
            t_address                   : IN    STD_LOGIC_VECTOR(0 DOWNTO 0);
            t_writedata                 : IN    STD_LOGIC_VECTOR(31 DOWNTO 0);
            t_readdata                  : OUT   STD_LOGIC_VECTOR(31 DOWNTO 0);
			nCmp_Int					: OUT   STD_LOGIC
        );
END ENTITY AlteraOpenMACIF;

ARCHITECTURE struct OF AlteraOpenMACIF IS

-- Core Interface 
    SIGNAL  s_nWr						: STD_LOGIC;
    SIGNAL  Sel_Ram						: STD_LOGIC;
    SIGNAL  Sel_Cont					: STD_LOGIC;
    SIGNAL  S_nBe                       : STD_LOGIC_VECTOR( 1 DOWNTO 0);
    SIGNAL  S_Addr                      : STD_LOGIC_VECTOR(31 DOWNTO 1);
    SIGNAL  S_Din                       : STD_LOGIC_VECTOR(15 DOWNTO 0);
    SIGNAL  S_Dout                      : STD_LOGIC_VECTOR(15 DOWNTO 0);
    SIGNAL  SelShadow                   : STD_LOGIC;
    SIGNAL  ShadowRead                  : STD_LOGIC_VECTOR(15 DOWNTO 0);
-- DMA Interface  
    SIGNAL  Dma_Req						: STD_LOGIC;
    SIGNAL  Dma_Rw						: STD_LOGIC;
    SIGNAL  Dma_Ack                     : STD_LOGIC;
    SIGNAL  Dma_Addr                    : STD_LOGIC_VECTOR(31 DOWNTO 1);
    SIGNAL  Dma_Dout                    : STD_LOGIC_VECTOR(15 DOWNTO 0);
    SIGNAL  Dma_Din                     : STD_LOGIC_VECTOR(15 DOWNTO 0);
-- Timer Interface
    SIGNAL  CsMacCmp                    : STD_LOGIC;
    SIGNAL  Mac_Zeit                    : STD_LOGIC_VECTOR(31 DOWNTO 0);
    SIGNAL  Mac_Cmp_Wert                : STD_LOGIC_VECTOR(31 DOWNTO 0);
    SIGNAL  Mac_Cmp_On					: STD_LOGIC;
    SIGNAL  Mac_Cmp_Int					: STD_LOGIC;

-- Mac RMii Signals to Filter
    SIGNAL  rTx_Eni						: STD_LOGIC;
	SIGNAL  rRx_Dati                   	: STD_LOGIC_VECTOR(1 DOWNTO 0);
	SIGNAL  rCrs_Dvi                   	: STD_LOGIC;
	SIGNAL  rTx_Dati                   	: STD_LOGIC_VECTOR(1 DOWNTO 0);

-- Filter RMii Signals to Hub
    SIGNAL  rTx_Enii					: STD_LOGIC;
	SIGNAL  rRx_Datii                   : STD_LOGIC_VECTOR(1 DOWNTO 0);
	SIGNAL  rCrs_Dvii                   : STD_LOGIC;
	SIGNAL  rTx_Datii                   : STD_LOGIC_VECTOR(1 DOWNTO 0);

-- Hub Signals
	SIGNAL hubTxEn						: STD_LOGIC_VECTOR(3 downto 1);
	SIGNAL hubTxDat0					: STD_LOGIC_VECTOR(3 downto 1);
	SIGNAL hubTxDat1					: STD_LOGIC_VECTOR(3 downto 1);
	SIGNAL RxPortInt					: integer RANGE 0 TO 3; --0 is idle
	SIGNAL RxPort						: STD_LOGIC_VECTOR(1 downto 0);

-- Mii Signals
	SIGNAL mii_Doei						: STD_LOGIC;
BEGIN

	d_readdata <= (OTHERS => '0');

	the_Mii : ENTITY work.OpenMAC_MII
		PORT MAP (  nRst => Reset_n,
					Clk  => Clk,
				    --Slave IF
					Addr     => mii_Addr,
					Sel      => mii_Sel,
					nBe      => mii_nBe,
					nWr      => mii_nWr,
					Data_In  => mii_Data_In,
					Data_Out => mii_Data_Out,
					--Export
					Mii_Clk   => mii_Clk,
					Mii_Di    => mii_Di,
					Mii_Do	  => mii_Do,
					Mii_Doe   => mii_Doei, -- '1' ... Input / '0' ... Output
					nResetOut => mii_nResetOut
				   );	
	mii_Doe <= not mii_Doei;
	
	the_Mac : ENTITY work.OpenMAC
		GENERIC MAP (	HighAdr  => s_Addr'HIGH,
						Simulate => Simulate,
						Timer    => MacTimer
					)
		PORT MAP	(	nRes => Reset_n,
						Clk  => Clk,
						--Export
						rRx_Dat  => rRx_Dati,
						rCrs_Dv  => rCrs_Dvi,
						rTx_Dat  => rTx_Dati,
						rTx_En   => rTx_Eni,
						Mac_Zeit => Mac_Zeit,
						--ir
						nTx_Int => nTx_Int,
						nRx_Int => nRx_Int,
						-- Slave Interface
						S_nBe    => s_nBe,
						s_nWr    => s_nWr,
						Sel_Ram  => Sel_Ram,
						Sel_Cont => Sel_Cont,
						S_Adr    => s_addr(10 DOWNTO 1),
						S_Din    => S_Din,
						S_Dout   => S_Dout,
						-- Master Interface
						Dma_Req  => Dma_Req,
						Dma_Rw   => Dma_Rw,
						Dma_Ack  => Dma_Ack,
						Dma_Addr => Dma_Addr,
						Dma_Dout => Dma_Dout,
						Dma_Din  => Dma_Din,
						-- Hub
						Hub_Rx => RxPort
					);

	the_Filter : entity work.OpenFILTER
		port map	(
						nRst => Reset_n,
						Clk => Clk,
						nCheckShortFrames => '0',
						RxDvIn => rCrs_Dvii,
						RxDatIn => rRx_Datii,
						RxDvOut => rCrs_Dvi,
						RxDatOut => rRx_Dati,
						TxEnIn => rTx_Eni,
						TxDatIn => rTx_Dati,
						TxEnOut => rTx_Enii,
						TxDatOut => rTx_Datii
		);
	
	the_Hub : entity work.OpenHub
		generic map (	Ports => 3
		)
		port map 	(
						nRst 			=> 	Reset_n,
						Clk 			=> 	Clk,
						RxDv 			=> 	rCrs_Dv_1 & rCrs_Dv_0 & rTx_Enii,
						RxDat0 			=> 	rRx_Dat_1(0) & rRx_Dat_0(0) & rTx_Datii(0),
						RxDat1 			=> 	rRx_Dat_1(1) & rRx_Dat_0(1) & rTx_Datii(1),
						TxEn 			=> 	hubTxEn,
						TxDat0 			=> 	hubTxDat0,
						TxDat1 			=> 	hubTxDat1,
						internPort 		=> 	1,
						TransmitMask 	=> 	(others => '1'),
						ReceivePort 	=> 	RxPortInt
		);
	RxPort <= conv_std_logic_vector(RxPortInt, RxPort'length);
	
	rTx_En_1 <= hubTxEn(3);
	rTx_En_0 <= hubTxEn(2);
	rCrs_Dvii <= hubTxEn(1);
	
	rTx_Dat_1(0) <= hubTxDat0(3);
	rTx_Dat_0(0) <= hubTxDat0(2);
	rRx_Datii(0) <= hubTxDat0(1);
	
	rTx_Dat_1(1) <= hubTxDat1(3);
	rTx_Dat_0(1) <= hubTxDat1(2);
	rRx_Datii(1) <= hubTxDat1(1);
	-----------------------------------------------------------------------
	-- Avalon Slave Interface <-> openMac
	-----------------------------------------------------------------------
	
	S_nBe    <= byteenable(0) & byteenable(1);
	s_nWr    <= write_n;
	
	Sel_Cont <= '1' WHEN ( Chipselect = '1' AND ( read_n = '0' OR write_n = '0' ) AND address(10 DOWNTO 9) = "00" ) ELSE '0';
	Sel_Ram  <= '1' WHEN ( Chipselect = '1' AND ( read_n = '0' OR write_n = '0' ) AND address(10 DOWNTO 10) = "1" ) ELSE '0';

	s_addr(11 DOWNTO 1) <= address(10 DOWNTO 1) &     address(0) WHEN ( Sel_Ram = '1' AND address(9) = '0' ) ELSE 
						   address(10 DOWNTO 1) & NOT address(0);

	s_Din    <= writedata(15 DOWNTO 8)  & writedata(7 DOWNTO 0) WHEN ( byteenable = "00" ) ELSE
				writedata(7 DOWNTO 0)   & writedata(15 DOWNTO 8);
	
	readdata <= ShadowRead(15 DOWNTO 8) & ShadowRead(7 DOWNTO 0)  WHEN ( SelShadow = '1' AND byteenable = "00" ) ELSE
				ShadowRead(7 DOWNTO 0)  & ShadowRead(15 DOWNTO 8) WHEN ( SelShadow = '1' ) ELSE
				s_Dout(15 DOWNTO 8)     & s_Dout(7 DOWNTO 0)      WHEN ( ( Sel_Ram = '1' OR Sel_Cont = '1') AND byteenable = "00" ) ELSE
				s_Dout(7 DOWNTO 0)      & s_Dout(15 DOWNTO 8)     WHEN ( Sel_Ram = '1' OR Sel_Cont = '1') ELSE
				(others => '0');

	SelShadow <= '1' WHEN ( Sel_Ram = '1' AND address(9) = '0' ) ELSE '0';

	the_ShadowRam : ENTITY work.Shadow_Ram 
		PORT MAP (  clock => clk,
					clken => SelShadow,
					wren => read_n,
					byteena => byteenable,
					address => address( 8 DOWNTO 0 ),
					data => writedata,
					q => ShadowRead
				);
	
	-----------------------------------------------------------------------
	-- Avalon Master Interface <-> openMac (new)
	-----------------------------------------------------------------------

	the_DMAAvalonIF: BLOCK
		-- Control
		TYPE	SM_Active	IS	(sIdle, sAct);
		SIGNAL	SM_Act      	:  SM_Active;
		SIGNAL  Tx_Act          :  STD_LOGIC;
		SIGNAL  Rx_Act, Rx_ActL :  STD_LOGIC;
		SIGNAL  Rx_ActLL        :  STD_LOGIC;
		SIGNAL  Fifo_AClr       :  STD_LOGIC;
		-- Avalon
		SIGNAL  m_rd_n          :  STD_LOGIC;
		SIGNAL  m_rd_cnt        :  STD_LOGIC_VECTOR(1 DOWNTO 0);
		SIGNAL  m_wr_n          :  STD_LOGIC;
		SIGNAL  m_wr_cnt        :  STD_LOGIC_VECTOR(1 DOWNTO 0);
		-- TX
		SIGNAL  TXFifo_Clr		:  STD_LOGIC;
		SIGNAL  TXFifo_Rd		:  STD_LOGIC;
		SIGNAL  TXFifo_Wr		:  STD_LOGIC;
		SIGNAL  TXFifo_AE		:  STD_LOGIC;
		SIGNAL  TXFifo_AF		:  STD_LOGIC;
		SIGNAL  TXFifo_E		:  STD_LOGIC;
		SIGNAL  TXFifo_F		:  STD_LOGIC;
		SIGNAL 	TXFifo_Addr		:  STD_LOGIC_VECTOR(31 DOWNTO 0);
		-- RX
		SIGNAL  RXFifo_Clr		:  STD_LOGIC;
		SIGNAL  RXFifo_Rd		:  STD_LOGIC;
		SIGNAL  RXFifo_Wr		:  STD_LOGIC;
		SIGNAL  RXFifo_AE		:  STD_LOGIC;
		SIGNAL  RXFifo_AF		:  STD_LOGIC;
		SIGNAL  RXFifo_E		:  STD_LOGIC;
		SIGNAL  RXFifo_F		:  STD_LOGIC;
		SIGNAL 	RXFifo_Addr		:  STD_LOGIC_VECTOR(31 DOWNTO 0);
		SIGNAL  RXTimeout       :  STD_LOGIC_VECTOR( 5 DOWNTO 0);
		SIGNAL  RXFirstRead		:  STD_LOGIC;
		SIGNAL  RXLastWrite 	:  STD_LOGIC;
		-- DMA
		SIGNAL  Dma_Acki        :  STD_LOGIC;
		-- MAC
		SIGNAL  rTx_EnL			:  STD_LOGIC;
		SIGNAL  rCrs_DvL		:  STD_LOGIC;
		signal	monitorCrsDv	:  std_logic_vector(3 downto 0); --monitor Crs Dv signal

	BEGIN
		Fifo_AClr <= NOT Reset_n;

		the_TxFifo : ENTITY work.OpenMAC_DMAFifo
		PORT MAP	(	aclr     	 => Fifo_AClr,
						clock	 	 => Clk,
						data	 	 => m_readdata(7 downto 0) & m_readdata(15 downto 8),
						rdreq	 	 => TXFifo_Rd,
						sclr	 	 => TXFifo_Clr,
						wrreq	 	 => TXFifo_Wr,
						almost_empty => TXFifo_AE,
						almost_full	 => TXFifo_AF,
						empty		 => TXFifo_E,
						full		 => TXFifo_F,
						q			 => Dma_Din
					);
		
		the_RxFifo : ENTITY work.OpenMAC_DMAFifo
		PORT MAP	(	aclr     	 => Fifo_AClr,
						clock	 	 => Clk,
						data	 	 => Dma_Dout(7 downto 0) & Dma_Dout(15 downto 8),
						rdreq	 	 => RXFifo_Rd,
						sclr	 	 => RXFifo_Clr,
						wrreq	 	 => RXFifo_Wr,
						almost_empty => RXFifo_AE,
						almost_full	 => RXFifo_AF,
						empty		 => RXFifo_E,
						full		 => RXFifo_F,
						q			 => m_writedata
					);
		
		-- Avalon Control
		m_byteenable_n <= "00";
		m_read_n       <= m_rd_n;
		m_write_n      <= m_wr_n;
		m_address      <= TXFifo_Addr WHEN m_rd_n = '0' ELSE
						  RXFifo_Addr;
		-- MAC Control
		Dma_Ack <= Dma_Acki;

		-- TXFifo Control 
		TXFifo_Rd <= Dma_Req AND Dma_Acki AND Dma_Rw;
		TXFifo_Wr <= NOT m_rd_n AND NOT m_waitrequest;

		-- RXFifo Control
		RXFifo_Wr <= Dma_Req AND Dma_Acki AND NOT Dma_Rw;
		RXFifo_Rd <= ( NOT m_wr_n AND NOT m_waitrequest AND NOT RXFifo_E ) OR RXFirstRead;

		p_DMAFifo : PROCESS ( Reset_n, Clk )	
		BEGIN
		
			IF ( Reset_n = '0' ) THEN
				rTx_EnL       <= '0';
				TXFifo_Clr    <= '0';
				TXFifo_Addr   <= (others => '0');
				Tx_Act        <= '0';
				
				rCrs_DvL      <= '0';
				RXFifo_Clr    <= '0';
				RXFifo_Addr   <= (others => '0');
				RXTimeout     <= (others => '0');
				Rx_Act        <= '0';
				RXFirstRead   <= '0';
				RXLastWrite   <= '0';

				SM_Act        <= sIdle;

				Dma_Acki      <= '0';

				m_rd_n        <= '1';
				m_rd_cnt      <= (others => '0');
				m_wr_n        <= '1';
				m_wr_cnt      <= (others => '0');
				m_arbiterlock <= '0';
				
			ELSIF rising_edge( Clk ) THEN
				rTx_EnL  <= rTx_Eni;
				rCrs_DvL <= rCrs_Dvi;
				Rx_ActL  <= Rx_Act;
				Rx_ActLL <= Rx_ActL;

				--monitor Crs Dv to find out beginning of rx frame
				monitorCrsDv <= monitorCrsDv(2 downto 0) & rCrs_DvL; --shift into monitor vector
				-- find two cycles 0 and two 1 => avoids rst when Crs Dv toggls at the end
				if monitorCrsDv = "0011" then
					RXFifo_Clr <= '1';
				else
					RXFifo_Clr <= '0';
				end if;
				
				IF Dma_Req = '1' AND Dma_Acki = '0' THEN
					IF    Dma_Rw = '1' AND TXFifo_E = '0' -- start when there is one entry / start when more entries are available ... AND TXFifo_AE = '0' 
						THEN Dma_Acki <= '1'; 
					ELSIF Dma_Rw = '0' AND RXFifo_F = '0'                     THEN Dma_Acki <= '1'; 
					END IF;
				ELSE                                                               Dma_Acki <= '0';
				END IF;

			-- Tx Control
				IF    Dma_Req = '1' AND Dma_Rw = '1'  THEN Tx_Act <= '1';
				ELSIF rTx_EnL = '1' AND rTx_Eni = '0' THEN Tx_Act <= '0';
				END IF;

				IF    Dma_Req = '1' AND Dma_Rw = '1' AND Tx_Act = '0' THEN TXFifo_Addr <= Dma_Addr & '0';
				ELSIF m_rd_n = '0'  AND m_waitrequest = '0'           THEN TXFifo_Addr <= TXFifo_Addr + 2;
				END IF;

				IF    m_rd_n = '0'  AND m_waitrequest = '0'           THEN m_rd_cnt <= m_rd_cnt + 1;
				END IF;

				IF Tx_Act = '1' THEN
					IF TXFifo_AF = '0' AND m_rd_cnt(m_rd_cnt'HIGH) = '0' AND m_wr_n = '1' THEN 
						m_arbiterlock <= '1';
						m_rd_n        <= '0';
					ELSIF m_waitrequest = '0' AND m_rd_n = '0' THEN 
						m_arbiterlock <= '0';
						m_rd_n        <= '1';
						m_rd_cnt      <= (others => '0');
					END IF;
				ELSE
					IF m_rd_n = '0' THEN
						IF m_waitrequest = '0' THEN
							m_arbiterlock <= '0';
							m_rd_n        <= '1';
							m_rd_cnt      <= (others => '0');
						END IF;
					ELSIF TXFifo_E = '0' THEN 
						TXFifo_Clr <= '1';
					ELSE
						TXFifo_Clr <= '0';
					END IF;
				END IF;

			-- Rx Control
				IF    Dma_Req = '1' AND Dma_Rw = '0'                    THEN Rx_Act <= '1';
				ELSIF rCrs_Dvi = '0' AND RXTimeout(RXTimeout'HIGH) = '1' THEN Rx_Act <= '0';
				END IF;

				IF    Dma_Req = '1' AND Dma_Rw = '0' AND Rx_Act = '0'   THEN RXFifo_Addr <= Dma_Addr & '0';
				ELSIF m_wr_n = '0'  AND m_waitrequest = '0'             THEN RXFifo_Addr <= RXFifo_Addr + 2;
				END IF;

				IF    m_wr_n = '0'  AND m_waitrequest = '0'             THEN m_wr_cnt <= m_wr_cnt + 1;
				END IF;

				IF Rx_Act = '1' OR m_wr_n = '0' OR RXLastWrite = '1' THEN
					IF   (RXFifo_AE = '0' OR (RXFifo_E = '0' AND rCrs_DvL = '0') OR RXLastWrite = '1') AND 
						  m_wr_cnt(m_wr_cnt'HIGH) = '0' AND m_rd_n = '1'                               AND
						 (Tx_Act = '0' OR TXFifo_AF = '1' OR m_rd_cnt(m_rd_cnt'HIGH) = '1')            THEN
						m_arbiterlock <= '1';
						m_wr_n        <= '0';
						RXLastWrite   <= '0';
					ELSIF m_waitrequest = '0' AND m_wr_n = '0' THEN 
						m_arbiterlock <= '0';
						m_wr_n        <= '1';
						m_wr_cnt      <= (others => '0');
						IF m_wr_cnt(m_wr_cnt'HIGH) = '1' AND RXFifo_E = '0' AND rCrs_DvL = '0' THEN RXLastWrite <= '1';
						END IF;
					END IF;
				END IF;

				IF    rCrs_DvL = '1' AND rCrs_Dvi = '1' THEN RXTimeout <= (others => '0');
				ELSIF RXTimeout(RXTimeout'HIGH) = '0'  THEN RXTimeout <= RXTimeout + 1;
				END IF;
				
				IF Rx_ActLL = '0' AND Rx_ActL = '1'    THEN RXFirstRead <= '1';
				ELSE                                        RXFirstRead <= '0';
				END IF;

			END IF;
			
		END PROCESS p_DMAFifo;
		
	END BLOCK the_DMAAvalonIF;

	-----------------------------------------------------------------------
	-- MAC-Time compare
	-- Mac Time output
	-----------------------------------------------------------------------

    nCmp_Int <= not Mac_Cmp_Int;
	CsMacCmp <= t_chipselect;

	p_MacCmp : PROCESS ( Reset_n, Clk )
	BEGIN
		IF ( Reset_n = '0' ) THEN
			Mac_Cmp_Int  <= '0';
			Mac_Cmp_On   <= '0';
			Mac_Cmp_Wert <= (OTHERS => '0'); 
			t_readdata <= (others => '0');
		ELSIF rising_edge( Clk ) THEN
		
			IF ( CsMacCmp = '1' AND t_write_n = '0' ) THEN
				case t_address(0 downto 0) is
					when "0" => --0
						Mac_Cmp_Wert(31 DOWNTO 0) <= t_writedata;
						Mac_Cmp_Int <= '0';
					when "1" => --4
						Mac_Cmp_On <= t_writedata(0);
					when others =>
						-- do nothing
				end case;
			END IF;

			IF ( Mac_Cmp_On = '1' AND Mac_Cmp_Wert( Mac_Zeit'RANGE ) = Mac_Zeit ) THEN
				Mac_Cmp_Int <= '1';
			END IF;
			
			if t_chipselect = '1' and t_read_n = '0' then
				case t_address(0 downto 0) is
					when "0" => --0
						t_readdata <= Mac_Zeit(31 DOWNTO 0);
					when "1" => --4
						t_readdata <= x"0000000" & "00" & Mac_Cmp_Int & Mac_Cmp_On;
					when others =>
						t_readdata <= (others => '0');
				end case;
			end if;

		END IF;
	END PROCESS p_MacCmp;

END ARCHITECTURE struct;
