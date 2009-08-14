-----------------------------------------------------------
--
--		Altera Avalon Interface for EPL MAC AND Mii Core
--			for use with Nios II soft-core
--
-- Copyright (C) 2009  B&R 
--     
-- This program is free software; you can redistribute it AND/OR modify
-- it under the terms of the GNU General Public License as published by
-- the Free Software Foundation; either version 2 of the License, OR
-- (at your option) any later version.
--  
--	Needs :	EPL_Mac.vhd (MAC)
--			EPL_Mii.vhd (serial management interface for phy)
--			EPL_Mac_DPR_altera.vhd (dual PORT ram)
--			EPL_Mac_AvalonIF_hw.tcl (for SOPC builder)
--
-----------------------------------------------------------------------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.std_logic_arith.all;
USE ieee.std_logic_unsigned.all;


ENTITY EPL_openMAC IS
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
            nRx_Int						: OUT   STD_LOGIC;
            nCmp_Int					: OUT   STD_LOGIC;
		-- Avalon Master Interface (DMA)
            m_read_n					: OUT   STD_LOGIC;
            m_write_n					: OUT   STD_LOGIC;
            m_byteenable_n              : OUT   STD_LOGIC_VECTOR(1 DOWNTO 0);
            m_address                   : OUT   STD_LOGIC_VECTOR(31 DOWNTO 0);
            m_writedata                 : OUT   STD_LOGIC_VECTOR(15 DOWNTO 0);
            m_readdata                  : IN    STD_LOGIC_VECTOR(15 DOWNTO 0);
            m_waitrequest               : IN    STD_LOGIC;
            m_arbiterlock				: OUT   STD_LOGIC;
		-- RMII
            rRx_Dat                     : IN    STD_LOGIC_VECTOR(1 DOWNTO 0);  -- RMII Rx Daten
            rCrs_Dv                     : IN    STD_LOGIC;                     -- RMII Carrier Sense / Data Valid
            rTx_Dat                     : OUT   STD_LOGIC_VECTOR(1 DOWNTO 0);  -- RMII Tx Daten
            rTx_En                      : OUT   STD_LOGIC;                     -- RMII Tx_Enable
		-- Serial Management Interface (Mii Core)
			mii_Addr					: IN	STD_LOGIC_VECTOR(2 DOWNTO 0);	
			mii_Sel						: IN	STD_LOGIC;						
			mii_nBe						: IN	STD_LOGIC_VECTOR(1 DOWNTO 0);	
			mii_nWr						: IN	STD_LOGIC;						
			mii_Data_In					: IN	STD_LOGIC_VECTOR(15 DOWNTO 0);	
			mii_Data_Out				: OUT	STD_LOGIC_VECTOR(15 DOWNTO 0);	
			mii_Clk						: OUT	STD_LOGIC;
			mii_Dio						: INOUT	STD_LOGIC;
			mii_nResetOut				: OUT	STD_LOGIC;
			mii_NodeNr					: IN	STD_LOGIC_VECTOR(7 DOWNTO 0) := (OTHERS => '0');
		-- Dummy Interface for newer SOPC builders
			d_chipselect                : IN    STD_LOGIC;
            d_read_n					: IN    STD_LOGIC;
            d_write_n					: IN    STD_LOGIC;
            d_byteenable                : IN    STD_LOGIC_VECTOR(1 DOWNTO 0);
            d_address                   : IN    STD_LOGIC_VECTOR(0 DOWNTO 0);
            d_writedata                 : IN    STD_LOGIC_VECTOR(15 DOWNTO 0);
            d_readdata                  : OUT   STD_LOGIC_VECTOR(15 DOWNTO 0)
        );
END ENTITY EPL_openMAC;

ARCHITECTURE struct OF EPL_openMAC IS

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
    
    SIGNAL  rTx_Eni						: STD_LOGIC;
    SIGNAL  ByteCnt                     : STD_LOGIC_VECTOR(4 DOWNTO 0);
BEGIN

	d_readdata <= (OTHERS => '0');

	Core_Mii : ENTITY work.EPL_Mii
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
					Mii_Dio   => mii_Dio,
					nResetOut => mii_nResetOut,
					NodeNr    => mii_NodeNr
				   );


	the_EthMac : ENTITY work.EPL_Mac
		GENERIC MAP (	HighAdr  => s_Addr'HIGH,
						Simulate => Simulate,
						Timer    => MacTimer
					)
		PORT MAP	(	nRes => Reset_n,
						Clk  => Clk,
						--Export
						rRx_Dat  => rRx_Dat,
						rCrs_Dv  => rCrs_Dv,
						rTx_Dat  => rTx_Dat,
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
						Dma_Din  => Dma_Din
					);

	rTx_En <= rTx_Eni;

	-----------------------------------------------------------------------
-- Avalon Slave Interface <-> openMac
	-----------------------------------------------------------------------
	
	S_nBe    <= byteenable(0) & byteenable(1);
	s_nWr    <= write_n;
	
	Sel_Cont <= '1' WHEN ( Chipselect = '1' AND ( read_n = '0' OR write_n = '0' ) AND address(10 DOWNTO 9) = "00" ) ELSE '0';
	CsMacCmp <= '1' WHEN ( Chipselect = '1' AND ( read_n = '0' OR write_n = '0' ) AND address(10 DOWNTO 9) = "01" ) ELSE '0';
	Sel_Ram  <= '1' WHEN ( Chipselect = '1' AND ( read_n = '0' OR write_n = '0' ) AND address(10 DOWNTO 10) = "1" ) ELSE '0';

	s_addr(11 DOWNTO 1) <= address(10 DOWNTO 1) &     address(0) WHEN ( Sel_Ram = '1' AND address(9) = '0' ) ELSE 
						   address(10 DOWNTO 1) & NOT address(0);

	s_Din    <= writedata(15 DOWNTO 8)  & writedata(7 DOWNTO 0) WHEN ( byteenable = "00" ) ELSE
				writedata(7 DOWNTO 0)   & writedata(15 DOWNTO 8);
	
	readdata <= ShadowRead(15 DOWNTO 8) & ShadowRead(7 DOWNTO 0)  WHEN ( SelShadow = '1' AND byteenable = "00" ) ELSE
				ShadowRead(7 DOWNTO 0)  & ShadowRead(15 DOWNTO 8) WHEN ( SelShadow = '1' ) ELSE
				s_Dout(15 DOWNTO 8)     & s_Dout(7 DOWNTO 0)      WHEN ( ( Sel_Ram = '1' OR Sel_Cont = '1') AND byteenable = "00" ) ELSE
				s_Dout(7 DOWNTO 0)      & s_Dout(15 DOWNTO 8)     WHEN ( Sel_Ram = '1' OR Sel_Cont = '1') ELSE
				x"0" & "00" & Mac_Cmp_Int & Mac_Cmp_On & x"00";

	SelShadow <= '1' WHEN ( Sel_Ram = '1' AND address(9) = '0' ) ELSE '0';

	theShadowRam : ENTITY work.Shadow_Ram 
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

	DMA_Avalon_Interface: BLOCK
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

	BEGIN
		Fifo_AClr <= NOT Reset_n;

		the_TxFifo : ENTITY work.EPL_Fifo
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
		
		the_RxFifo : ENTITY work.EPL_Fifo
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

		p_MacFifo : PROCESS ( Reset_n, Clk )	
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
				rCrs_DvL <= rCrs_Dv;
				Rx_ActL  <= Rx_Act;
				Rx_ActLL <= Rx_ActL;

				IF Dma_Req = '1' AND Dma_Acki = '0' THEN
					IF    Dma_Rw = '1' AND TXFifo_E = '0' AND TXFifo_AE = '0' THEN Dma_Acki <= '1'; 
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
				ELSIF rCrs_Dv = '0' AND RXTimeout(RXTimeout'HIGH) = '1' THEN Rx_Act <= '0';
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

				IF    rCrs_DvL = '1' AND rCrs_Dv = '1' THEN RXTimeout <= (others => '0');
				ELSIF RXTimeout(RXTimeout'HIGH) = '0'  THEN RXTimeout <= RXTimeout + 1;
				END IF;
				
				IF Rx_ActLL = '0' AND Rx_ActL = '1'    THEN RXFirstRead <= '1';
				ELSE                                        RXFirstRead <= '0';
				END IF;

			END IF;
			
		END PROCESS p_MacFifo;
		
	END BLOCK DMA_Avalon_Interface;

	-----------------------------------------------------------------------
-- MAC-Time compare
	--  Address 0 : Cmp_H
	--          2 : Cmp_L
	--          4 : Enable int IF D0 = 1, Disable int IF D0 = 0 
	-----------------------------------------------------------------------

    nCmp_Int <= Mac_Cmp_Int;

	p_MacCmp : PROCESS ( Reset_n, Clk )
	BEGIN
		IF ( Reset_n = '0' ) THEN
			Mac_Cmp_Int  <= '0';
			Mac_Cmp_On   <= '0';
			Mac_Cmp_Wert <= (OTHERS => '0');
		ELSIF rising_edge( Clk ) THEN
		
			IF ( CsMacCmp = '1' AND write_n = '0' ) THEN
				IF ( address(2) = '0' AND address(0) = '0' ) THEN
					Mac_Cmp_Wert(31 DOWNTO 16) <= writedata;
				ELSIF ( address(2) = '0' AND address(0) = '1' ) THEN
					Mac_Cmp_Wert(15 DOWNTO 0) <= writedata;
					Mac_Cmp_Int <= '0';
				ELSIF ( address(2) = '1' ) THEN
					Mac_Cmp_On <= writedata(0);
				END IF;
			END IF;

			IF ( Mac_Cmp_On = '1' AND Mac_Cmp_Wert( Mac_Zeit'RANGE ) = Mac_Zeit ) THEN
				Mac_Cmp_Int <= '1';
			END IF;

		END IF;
	END PROCESS p_MacCmp;

END ARCHITECTURE struct;
