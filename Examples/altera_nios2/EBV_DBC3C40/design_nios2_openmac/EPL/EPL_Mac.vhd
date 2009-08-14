-----------------------------------------------------------
--
--		EPL  MAC for Controlled Node  MAC
--
-- Copyright (C) 2009  B&R 
--     
-- This program is free software; you can redistribute it and/or modify
-- it under the terms of the GNU General Public License as published by
-- the Free Software Foundation; either version 2 of the License, or
-- (at your option) any later version.
--  
--	Needs : EPL_Mac_DPR_v.vhd
--
-----------------------------------------------------------------------------------------------------------------------------
--
--
LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.std_logic_arith.all;
USE ieee.std_logic_unsigned.all;

--	V 00.00-00.20					Aus Standard Mac generiert
--	V 00.03			30.03.2009	EP	Fehler bei Rx-Dam :		Rx_Dma_Inh  und gleich auch Tx_DmaAckl
--															Ist von 10 MBaud übergeblieben.
--


ENTITY EPL_Mac	IS
	Generic(HighAdr		: IN		integer := 16;
			Timer		: IN		boolean := true;
			Simulate	: IN		boolean := false  );
	PORT (	nRes, Clk					: IN    	std_logic;
-- Processor 
			s_nWr, Sel_Ram, Sel_Cont	: IN    	std_logic := '0';
			S_nBe						: IN    	std_logic_vector( 1 downto 0);
			S_Adr						: IN    	std_logic_vector(10 downto 1);
			S_Din						: IN    	std_logic_vector(15 downto 0);
			S_Dout						: OUT    	std_logic_vector(15 downto 0);
			nTx_Int, nRx_Int			: OUT    	std_logic;
			nTx_BegInt					: OUT    	std_logic;
-- DMA	
			Dma_Req, Dma_Rw				: OUT    	std_logic;	
			Dma_Ack						: In    	std_logic;	
			Dma_Addr					: OUT    	std_logic_vector(HighAdr downto 1);	
			Dma_Dout					: OUT    	std_logic_vector(15 downto 0);	
			Dma_Din						: IN    	std_logic_vector(15 downto 0);
-- RMII
			rRx_Dat	 					: IN    	std_logic_vector( 1 downto 0);			
			rCrs_Dv						: IN    	std_logic;								
			rTx_Dat	 					: OUT    	std_logic_vector( 1 downto 0);			
			rTx_En						: OUT    	std_logic;								
			Hub_Rx						: IN    	std_logic_vector( 1 downto 0) := "00";	
			Mac_Zeit					: OUT		std_logic_vector(31 downto 0)
   		);
END ENTITY EPL_Mac;

ARCHITECTURE struct OF EPL_Mac IS
	CONSTANT	log0					: std_logic := '0';
	CONSTANT	log1					: std_logic := '1';
	SIGNAL	Rx_Dv						: std_logic;						
	SIGNAL	R_Req						: std_logic;
	SIGNAL	Auto_Desc					: std_logic_vector( 3 downto 0);
	SIGNAL	Zeit						: std_logic_vector(31 downto 0);	
	SIGNAL	Tx_Dma_Req,  Rx_Dma_Req		: std_logic;
	SIGNAL	Tx_Dma_Ack,  Rx_Dma_Ack		: std_logic;
	SIGNAL	Tx_Ram_Dat,  Rx_Ram_Dat		: std_logic_vector(15 downto 0);
	SIGNAL	Tx_Reg,		 Rx_Reg			: std_logic_vector(15 downto 0);
	SIGNAL	Dma_Tx_Addr, Dma_Rx_Addr	: std_logic_vector(Dma_Addr'range);	
	SIGNAL	Tx_Col						: std_logic;
	SIGNAL	Sel_Tx_Ram, Sel_Tx_Reg		: std_logic;
	SIGNAL	Tx_LatchH, Tx_LatchL		: std_logic_vector( 7 downto 0);

BEGIN 

	S_Dout <=	Tx_Ram_Dat	WHEN	Sel_Ram  = '1' AND Sel_Tx_Ram = '1'		ELSE
				Rx_Ram_Dat	WHEN	Sel_Ram  = '1'							ELSE
				Tx_Reg		WHEN	Sel_Cont = '1' AND Sel_Tx_Reg = '1'		ELSE
				Rx_Reg;

	Mac_Zeit <= Zeit;

b_Dma:	BLOCK
	SIGNAL	Rx_Dma, Tx_Dma	: std_logic;
BEGIN
	Dma_Req    <= '1'	WHEN    (Tx_Dma_Req = '1' AND Tx_Dma_Ack = '0') OR Rx_Dma_Req = '1'		ELSE '0';

	Dma_Rw     <= '1'			WHEN   (Rx_Dma = '0' AND Tx_Dma_Req = '1' AND Tx_Dma_Ack = '0') OR Tx_Dma = '1' 	ELSE '0';	
	Dma_Addr   <= Dma_Tx_Addr	WHEN   (Rx_Dma = '0' AND Tx_Dma_Req = '1' AND Tx_Dma_Ack = '0') OR Tx_Dma = '1' 	ELSE Dma_Rx_Addr;	

	Rx_Dma_Ack <= '1'	WHEN	Rx_Dma = '1' AND Dma_Ack = '1'	ELSE '0';

pDmaArb: PROCESS( Clk, nRes )	 IS
BEGIN

	IF nRes = '0'	THEN
		Rx_Dma <= '0'; Tx_Dma <= '0'; Tx_Dma_Ack <= '0'; 
		Tx_LatchH <= (others => '0'); Tx_LatchL <= (others => '0');
		Zeit <= (others => '0'); 
	ELSIF rising_edge( Clk )	THEN

		IF Timer Then
			Zeit <= Zeit + 1;
		end if;

		Sel_Tx_Ram  <= s_Adr(8);				
		Sel_Tx_Reg  <= NOT s_Adr(3);			

		IF		Dma_Ack = '0'	THEN
			IF		Rx_Dma = '0' AND Tx_Dma_Req = '1' AND Tx_Dma_Ack = '0'	THEN	Tx_Dma <= '1';
			ELSIF	Tx_Dma = '0' AND Rx_Dma_Req = '1'						THEN	Rx_Dma <= '1';
			END IF;
		ELSE
			IF		Rx_Dma = '1' AND Tx_Dma_Req = '1' AND Tx_Dma_Ack = '0' 	THEN	Tx_Dma <= '1';	Rx_Dma <= '0';
			ELSIF	Tx_Dma = '1' AND Rx_Dma_Req = '1'						THEN	Tx_Dma <= '0';	Rx_Dma <= '1';
			ELSE																	Tx_Dma <= '0';	Rx_Dma <= '0';
			END IF;
		END IF;

		IF		Tx_Dma = '1' AND Dma_Ack = '1'	THEN	Tx_Dma_Ack <= '1';
		else											Tx_Dma_Ack <= '0';
		END IF;
		
		IF	Tx_Dma_Ack = '1'		THEN	Tx_LatchH <= Dma_Din(15 downto 8);
											Tx_LatchL <= Dma_Din( 7 downto 0);
		END IF;

	END IF; 

END PROCESS pDmaArb;

END BLOCK b_Dma;

b_Full_Tx:	BLOCK
	TYPE	MACTX_TYPE IS ( R_Idl, R_Bop, R_Pre, R_Txd, R_Crc, R_Col, R_Jam );
	SIGNAL	Sm_Tx						: MACTX_TYPE;
	SIGNAL	Start_Tx, ClrCol, Tx_On		: std_logic;
	SIGNAL	Dibl_Cnt					: std_logic_vector( 1 downto 0);	
	SIGNAL	F_End, Was_Col, Block_Col	: std_logic;						
	SIGNAL	Ipg_Cnt, Tx_Timer			: std_logic_vector( 7 downto 0);	
	 ALIAS	Ipg							: std_logic IS Ipg_Cnt(7);
	 ALIAS	Tx_Time						: std_logic IS Tx_Timer(7);
	SIGNAL	Tx_Ipg						: std_logic_vector( 5 downto 0);	
	SIGNAL	Tx_Count					: std_logic_vector(11 downto 0);	
	SIGNAL	Tx_En, F_Val, Tx_Half		: std_logic;
	SIGNAL	Tx_Sr, F_TxB				: std_logic_vector( 7 downto 0);
	SIGNAL	Crc							: std_logic_vector(31 downto 0);
	SIGNAL	CrcDin, Tx_Dat				: std_logic_vector( 1 downto 0);	
 	SIGNAL	Col_Cnt						: std_logic_vector( 3 downto 0);	
	SIGNAL	Auto_Coll					: std_logic;
 	SIGNAL	Rnd_Num						: std_logic_vector( 9 downto 0);	
 	SIGNAL	Retry_Cnt					: std_logic_vector( 9 downto 0);	
	SIGNAL	Max_Retry					: std_logic_vector( 3 downto 0);	

BEGIN

	rTx_En  <= Tx_En;
	rTx_Dat <= Tx_Dat;

pTxSm: PROCESS ( Clk, nRes )	 IS
BEGIN

	IF nRes = '0'	THEN
		Sm_Tx <= R_Idl;
	ELSIF rising_edge( Clk ) THEN
		IF	Sm_Tx = R_Idl OR Sm_Tx = R_Bop OR Dibl_Cnt = "11" 	THEN
			CASE Sm_Tx	IS
				WHEN	R_Idl =>	IF	Start_Tx = '1' 
									AND (Tx_Half = '0' OR Rx_Dv = '0')
									AND Ipg = '0'						THEN	Sm_Tx <= R_Bop;	END IF;
				WHEN	R_Bop =>												Sm_Tx <= R_Pre;
				WHEN	R_Pre =>	IF		Tx_Time = '1'				THEN	Sm_Tx <= R_Txd;	END IF;
				WHEN	R_Txd =>	IF		Was_Col = '1'				THEN	Sm_Tx <= R_Col;
									ELSIF	Tx_Count = 0				THEN	Sm_Tx <= R_Crc;	END IF;								
				WHEN	R_Col =>												Sm_Tx <= R_Jam;
				WHEN	R_Jam =>	IF		Tx_Time = '1'				THEN	Sm_Tx <= R_Idl;
									END IF;
				WHEN	R_Crc =>	IF		Was_Col = '1'				THEN	Sm_Tx <= R_Col;
									ELSIF	Tx_Time = '1'				THEN	Sm_Tx <= R_Idl;	END IF;
				WHEN	others  =>	NULL;
			END CASE;
		END IF;
	END IF; 

END PROCESS pTxSm;


pTxCtl: PROCESS ( Clk, nRes )	 IS
	VARIABLE	Preload		:	std_logic_vector(Tx_Timer'range);
	VARIABLE	Load		:	std_logic;
BEGIN

	IF nRes = '0'	THEN
		Tx_Dat <= "00"; Tx_En <= '0'; Dibl_Cnt <= "00"; F_End <= '0'; F_Val <= '0'; Tx_Col  <= '0'; Was_Col <= '0'; Block_Col <= '0';
		Ipg_Cnt <= (others => '0'); Tx_Timer <= (others => '0'); Tx_Sr <= (others => '0'); 
	ELSIF rising_edge( Clk ) 	THEN

		IF		Sm_Tx = R_Bop		THEN	Dibl_Cnt <= "00";			
		ELSE								Dibl_Cnt <= Dibl_Cnt + 1;
		END IF;

		IF		Tx_En = '1'						then	Ipg_Cnt <= "1"  & conv_std_logic_vector( 44, 7);	
		elsif	Rx_Dv = '1' AND Tx_Half = '1'	then	Ipg_Cnt <= "10" & Tx_Ipg;
		ELSIF	Ipg = '1'	 					THEN	Ipg_Cnt <= Ipg_Cnt - 1;
		END IF;

		IF		Dibl_Cnt = "11" AND Sm_Tx = R_Crc AND Tx_Time = '1'		THEN	F_End  <= '1';
		ELSIF	Dibl_Cnt = "11" AND Sm_Tx = R_Col THEN
			if		Col_Cnt = (Max_Retry - 1)							THEN	F_End  <= '1';	
			elsif	Col_Cnt < x"E"										THEN	Tx_Col <= '1';	
			ELSE																F_End  <= '1';
			END IF;
		ELSE																	F_End  <= '0';	
																				Tx_Col <= '0';
		END IF;
	
		IF		 Tx_Half = '1' AND Rx_Dv = '1'
			AND (Sm_Tx = R_Pre OR Sm_Tx = R_Txd)	THEN	Was_Col <= '1';	
		ELSIF	 Sm_Tx = R_Col						THEN	Was_Col <= '0';
		END IF;

		IF		 Sm_Tx = R_Col					THEN	Block_Col <= '1';	
		elsif	 Auto_Coll = '1'				then	Block_Col <= '0';	
		ELSIF	 Retry_Cnt = 0					THEN	Block_Col <= '0';
		END IF;

		IF		Dibl_Cnt = "10" AND Sm_Tx = R_Pre AND Tx_Time = '1'	THEN	F_Val <= '1';	
		ELSIF   Dibl_Cnt = "10" AND Sm_Tx = R_Txd					THEN	F_Val <= '1';	
		ELSE																F_Val <= '0';
		END IF;

		Load := '0';
		IF		Sm_Tx = R_Bop		THEN	Preload := x"06";	Load := '1';
		ELSIF	Sm_Tx = R_Txd		THEN	Preload := x"02";	Load := '1';
		ELSIF	Sm_Tx = R_Col		THEN	Preload := x"01";	Load := '1';
		ELSIF	Tx_Time = '1'		THEN	Preload := x"3e";	Load := '1';
		END IF;

		IF		Dibl_Cnt = "11"	OR Sm_Tx = R_Bop 	THEN
			IF		Load = '1'	THEN	Tx_Timer <= Preload;
			ELSE						Tx_Timer <= Tx_Timer - 1;
			END IF;
		END IF;

		IF		F_Val = '1'		THEN	Tx_Sr <= F_TxB;						
		ELSE							Tx_Sr <= "00" & Tx_Sr(7 downto 2);	
		END IF;

		IF		Sm_Tx = R_Pre										THEN	Tx_En <= '1';
		ELSIF	Sm_Tx = R_Idl OR (Sm_Tx = R_Jam AND Tx_Time = '1')	THEN	Tx_En <= '0';
		END IF;

		IF		Sm_Tx = R_Pre AND Tx_Time = '1' AND Dibl_Cnt = "11"	THEN	Tx_Dat <= "11";	
		ELSIF	Sm_Tx = R_Pre										THEN	Tx_Dat <= "01";	
		ELSIF	Sm_Tx = R_Txd										THEN	Tx_Dat <= CrcDin;	
		ELSIF	Sm_Tx = R_Crc										THEN	Tx_Dat <= NOT Crc(30) & NOT Crc(31);	
		ELSIF	Sm_Tx = R_Col OR Sm_Tx = R_Jam						THEN	Tx_Dat <= "11";
		ELSE																Tx_Dat <= "00";
		END IF;

	END IF; 

END PROCESS pTxCtl;

pBackDel: PROCESS ( Clk, nRes )	IS
BEGIN
	IF nRes = '0'	THEN
		Rnd_Num   <= (others => '0');		
		Col_Cnt   <= (others => '0');		
		Retry_Cnt <= (others => '0');
	ELSIF rising_edge( Clk ) 	THEN

		Rnd_Num <= Rnd_Num(8 downto 0) & (Rnd_Num(9) XOR NOT Rnd_Num(2));

		IF		ClrCol = '1'						THEN	Col_Cnt <= x"0";
		ELSIF	Dibl_Cnt = "11"	AND Sm_Tx = R_Col	THEN	Col_Cnt <= Col_Cnt + 1;		
		END IF;

		IF	Dibl_Cnt = "11"	THEN
			IF		Tx_On = '0'	or Auto_Coll = '1'		THEN	Retry_Cnt <= (others => '0');		
			ELSIF	Sm_Tx = R_Col  THEN
				FOR i in 0 to 9 LOOP
					IF	 Col_Cnt >= i	THEN	Retry_Cnt(i) <= Rnd_Num(i);
					ELSE						Retry_Cnt(i) <= '0';
					END IF;	
				END LOOP; 
			ELSIF Sm_Tx /= R_Jam AND Tx_Time = '1' AND Retry_Cnt /= 0	THEN	Retry_Cnt <= Retry_Cnt - 1;
			END IF;
		END IF;
	END IF; 
END PROCESS pBackDel;


	CrcDin <= Tx_Sr(1 downto 0);

Calc: PROCESS ( Clk, Crc, CrcDin )	 IS
	VARIABLE	H 		: std_logic_vector(1 downto 0);
BEGIN

	H(0) := Crc(31) XOR CrcDin(0);
	H(1) := Crc(30) XOR CrcDin(1);

	IF rising_edge( Clk )  THEN	 
		IF		Sm_Tx = R_Pre		THEN	Crc <= x"FFFFFFFF";
		ELSIF 	Sm_Tx = R_Crc		THEN	Crc <= Crc(29 downto 0) & "00";
		ELSE
			Crc( 0) <=		                H(1);		
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

bTxDesc:	BLOCK
	TYPE	sDESC	IS	(sIdle, sLen, sTimL, sTimH, sAdrH, sAdrL, sBegL, sBegH, sData, sStat, sColl );
	SIGNAL	Dsm, Tx_Dsm_Next					: sDESC;
	SIGNAL	DescRam_Out, DescRam_In				: std_logic_vector(15 downto 0);
	 ALIAS	TX_LEN								: std_logic_vector(11 downto 0)	IS	DescRam_Out(11 downto 0);
	 ALIAS	TX_OWN								: std_logic						IS	DescRam_Out( 8);
	 ALIAS	TX_LAST								: std_logic						IS	DescRam_Out( 9);
	 ALIAS	TX_READY							: std_logic						IS	DescRam_Out(10);
	 ALIAS	TX_BEGDEL							: std_logic						IS	DescRam_Out(12);
	 ALIAS	TX_BEGON							: std_logic						IS	DescRam_Out(13);
	 ALIAS	TX_TIME								: std_logic						IS	DescRam_Out(14);
	 ALIAS	TX_RETRY							: std_logic_vector( 3 downto 0)	IS	DescRam_Out(3 downto 0);
	SIGNAL	Ram_Be								: std_logic_vector( 1 downto 0);
	SIGNAL	Ram_Wr, Desc_We						: std_logic;
	SIGNAL	Desc_Addr							: std_logic_vector( 7 downto 0);
	SIGNAL	DescIdx								: std_logic_vector( 2 downto 0);	
	SIGNAL	Last_Desc							: std_logic;
	SIGNAL	ZeitL								: std_logic_vector(15 downto 0);	
	SIGNAL	Tx_Ie, Tx_Wait						: std_logic;
	SIGNAL	Tx_BegInt, Tx_BegSet, Tx_Early		: std_logic;
	SIGNAL	Ext_Tx, Ext_Ack						: std_logic;
	SIGNAL	Tx_Desc, Tx_Desc_One, Ext_Desc		: std_logic_vector( 3 downto 0);
	SIGNAL	Tx_Icnt								: std_logic_vector( 4 downto 0);
	SIGNAL	Tx_SoftInt							: std_logic;
	SIGNAL	Sel_TxH, Sel_TxL, H_Byte			: std_logic;
	SIGNAL	Tx_Buf								: std_logic_vector( 7 downto 0);
	SIGNAL	Tx_Idle, TxInt						: std_logic;
	SIGNAL	Tx_Ident							: std_logic_vector( 1 downto 0);
	SIGNAL	Start_TxS							: std_logic;
	
BEGIN

	Ram_Wr    <= '1' WHEN	s_nWr = '0' AND Sel_Ram = '1' AND s_Adr(10) = '1'	ELSE '0';
	Ram_Be(1) <= '1' WHEN	s_nWr = '1' OR s_nBE(1) = '0'						ELSE '0';
	Ram_Be(0) <= '1' WHEN	s_nWr = '1' OR s_nBE(0) = '0'						ELSE '0';

	DescIdx <=	"000"	WHEN	Desc_We = '0' and Tx_Dsm_Next = sIdle	ELSE
				"000"	WHEN	Desc_We = '1' and Dsm = sIdle			ELSE
				"001"	WHEN	Desc_We = '0' and Tx_Dsm_Next = sLen	ELSE
				"001"	WHEN	Desc_We = '1' and Dsm = sLen			ELSE
				"010"	WHEN	Desc_We = '0' and Tx_Dsm_Next = sAdrH	ELSE
				"010"	WHEN	Desc_We = '1' and Dsm = sAdrH			ELSE
				"011"	WHEN	Desc_We = '0' and Tx_Dsm_Next = sAdrL	ELSE
				"011"	WHEN	Desc_We = '1' and Dsm = sAdrL			ELSE
				"100"	WHEN	Desc_We = '0' and Tx_Dsm_Next = sBegH	ELSE
				"100"	WHEN	Desc_We = '1' and Dsm = sBegH			ELSE
				"101"	WHEN	Desc_We = '0' and Tx_Dsm_Next = sBegL	ELSE
				"101"	WHEN	Desc_We = '1' and Dsm = sBegL			ELSE
				"110"	WHEN	Desc_We = '0' and Tx_Dsm_Next = sTimH	ELSE
				"110"	WHEN	Desc_We = '1' and Dsm = sTimH			ELSE
				"111"	WHEN	Desc_We = '0' and Tx_Dsm_Next = sTimL	ELSE
				"111"	WHEN	Desc_We = '1' and Dsm = sTimL			ELSE
				"111"	WHEN	Desc_We = '0' and Tx_Dsm_Next = sData	ELSE
				"111"	WHEN	Desc_We = '1' and Dsm = sData			ELSE
				"000";
	
	Desc_We <= '1' WHEN  Dsm = sTimL OR Dsm = sTimH OR Dsm = sStat	ELSE   '0';

	Desc_Addr <= '1' & Tx_Desc  & DescIdx	WHEN	Ext_Tx = '0'	ELSE	
				 '1' & Ext_Desc & DescIdx;

gTxTime:	if Timer generate
	DescRam_In <= Zeit(31 downto 16)		WHEN	Dsm  = sTimH	ELSE
				  ZeitL						WHEN	Dsm  = sTimL	ELSE
				  x"000" & "01" & Tx_Ident	WHEN	Dsm  = sBegL	ELSE
				  "0" & "0" & "00" & "0100" & "00" & "0" & "0" & Col_Cnt;
end generate;

gnTxTime:	if NOT Timer generate
	DescRam_In <= x"000" & "01" & Tx_Ident	WHEN	Dsm  = sBegL	ELSE
				  "0" & "0" & "00" & "0100" & "00" & "0" & "0" & Col_Cnt;
end generate;


RamH:	entity	work.Dpr_16_16
	generic map(Simulate => Simulate)
	port map (	CLKA	=> Clk,					CLKB	=> Clk,
				EnA		=> log1,				Enb		=> log1,
				BEA		=> Ram_Be,				
				WEA		=> Ram_Wr,				WEB		=> Desc_We,
				ADDRA	=> s_Adr(8 downto 1),	ADDRB	=> Desc_Addr,
			  	DIA		=> s_Din,				DIB		=> DescRam_In,
				DOA		=> Tx_Ram_Dat,			DOB		=> DescRam_Out
			);

pTxSm: Process( nRes, Clk, Dsm, 
				Tx_On, TX_OWN, Retry_Cnt, Ext_Tx, Tx_Wait, Sm_Tx, F_End, Tx_Col, Ext_Ack )
Begin

		
		Tx_Dsm_Next <= Dsm;
		CASE	Dsm IS
			WHEN sIdle	 =>	IF	Tx_On = '1' AND TX_OWN = '1' AND Retry_Cnt = 0	THEN
								IF	(Ext_Tx = '1' and Ext_Ack = '0') OR Tx_Wait  = '0' 	 THEN	
														Tx_Dsm_Next <= sLen;
								END IF;
							END IF;
			WHEN sLen	 =>								Tx_Dsm_Next <= sAdrH; 
			WHEN sBegH	 =>								Tx_Dsm_Next <= sBegL;
			WHEN sBegL	 =>	IF	   Tx_On  = '0'	THEN	Tx_Dsm_Next <= sIdle;
							ELSIF Sm_Tx = R_Pre THEN	Tx_Dsm_Next <= sTimH; 
							END IF;
			WHEN sAdrH	 =>								Tx_Dsm_Next <= sAdrL;
			WHEN sAdrL	 =>	IF    Tx_On  = '0'	THEN	Tx_Dsm_Next <= sIdle;
							ELSE						Tx_Dsm_Next <= sBegL;	
							END IF;
			WHEN sTimH	 =>								Tx_Dsm_Next <= sTimL;
			WHEN sTimL	 =>								Tx_Dsm_Next <= sData;			
			WHEN sData	 =>	IF	  F_End = '1'	THEN	Tx_Dsm_Next <= sStat;	
							ELSIF Tx_Col = '1'	THEN	Tx_Dsm_Next <= sColl;	
							END IF;
			WHEN sStat	 =>								Tx_Dsm_Next <= sIdle;	
			when sColl	 =>	if	sm_tx = r_idl then		Tx_Dsm_Next <= sIdle;	
							end if;
			WHEN others	 =>
		END CASE;

	IF	nRes = '0'					THEN	Dsm <= sIdle;
	ELSIF	rising_edge( Clk )		THEN	Dsm <= Tx_Dsm_Next;
	END IF;

end process pTxSm;
pTxControl: Process( nRes, Clk )
Begin

	IF	nRes = '0'	THEN
		Last_Desc <= '0'; Start_TxS <= '0'; Tx_Dma_Req  <= '0'; H_Byte <= '0'; 
		Tx_BegSet <= '0'; Tx_Early <= '0'; Auto_Coll <= '0'; 
		Ext_Tx <= '0'; Ext_Ack <= '0'; ClrCol <= '0'; Ext_Desc <= (others => '0'); Max_Retry <= (others => '0');
		ZeitL <= (others => '0'); Tx_Count <= (others => '0'); Tx_Ident <= "00"; 
		Dma_Tx_Addr <= (others => '0');
	ELSIF	rising_edge( Clk ) 	THEN

		IF	Dsm = sStat AND Desc_We = '1'	THEN	ClrCol  <= '1';	
		ELSE										ClrCol  <= '0';
		END IF;

		IF Timer Then
			IF	Dsm  = sTimH	THEN	ZeitL <= Zeit(15 downto 0);
			END IF;
		END IF;

		IF		Ext_Ack = '0' AND R_Req = '1'					THEN	Ext_Desc <= Auto_Desc;		
																		Ext_Ack  <= '1';			
		ELSIF	Ext_Tx = '1' OR	Tx_On = '0'						THEN	Ext_Ack  <= '0';			
		END IF;

		IF		Dsm = sIdle AND Ext_Ack = '1'					THEN	Ext_Tx  <= '1';			
		ELSIF	Dsm = sStat OR 	Tx_Col = '1' OR	Tx_On = '0'		THEN	Ext_Tx  <= '0';			
		END IF;
		
		IF	   (F_End = '1' OR Tx_On = '0'									
			or (Tx_Col = '1' and Ext_Tx = '1' )				
			or dsm = sColl	 )							THEN	Start_TxS <= '0';		
																Auto_Coll <= Auto_Coll or (Tx_Col and Ext_Tx); 
		ELSIF	Dsm = sAdrH								THEN	Start_TxS <= '1';		
		elsif	Sm_Tx = R_Idl							then	Auto_Coll <= '0';
		END IF;

		IF		Dsm = sIdle		THEN	Last_Desc <= TX_LAST;	
		END IF;

		IF		Dsm = sLen		THEN	Tx_Count  <= TX_LEN;
		ELSIF   F_Val = '1'		THEN	Tx_Count  <= Tx_Count - 1; 
		END IF;

		IF	Dsm = sIdle and Tx_On = '1' AND TX_OWN = '1' AND Retry_Cnt = 0	THEN
			IF	Ext_Tx = '1' OR Tx_Wait  = '0' 	 THEN	
													Max_Retry <= TX_RETRY;
													Tx_Early  <= TX_BEGON;
			END IF;
		ELSIF Dsm = sTimH					THEN	Tx_BegSet <= Tx_Early;
		ELSIF Dsm = sTimL					THEN	Tx_BegSet <= '0';		
		END IF;

		
		IF		Dsm = sAdrL		 THEN	Dma_Tx_Addr(15 downto 1)	<= DescRam_Out(15 downto 1);	
		ELSIF	Tx_Dma_Ack = '1' THEN	Dma_Tx_Addr(15 downto 1)	<= Dma_Tx_Addr(15 downto 1) + 1;	
		END IF;

		IF		Dsm = sAdrH		 THEN	
				Dma_Tx_Addr(Dma_Addr'high downto 16) <= DescRam_Out(Dma_Addr'high-16 downto 0);
				Tx_Ident <= DescRam_Out(15 downto 14);
		ELSIF	Tx_Dma_Ack = '1' AND Dma_Tx_Addr(15 downto 1) = x"FFF" & "111"	THEN	
				Dma_Tx_Addr(Dma_Addr'high downto 16) <= Dma_Tx_Addr(Dma_Addr'high downto 16) + 1;	
		END IF;

		IF		DSM = sAdrL 
			OR (F_Val = '1' AND H_Byte = '0')	THEN	Tx_Dma_Req  <= '1' after 0 nS;
		ELSIF	Tx_Dma_Ack = '1'				THEN	Tx_Dma_Req  <= '0';
		END IF;

		IF		Sm_Tx = R_Bop					THEN	H_Byte <= '0';
		ELSIF	F_Val = '1'						THEN	H_Byte <= NOT H_Byte;
		END IF;

		IF	F_Val = '1'			THEN	Tx_Buf <= Tx_LatchL;	
		END IF;

	END IF;	

End Process pTxControl;

	Start_Tx <= '1'	when	Start_TxS = '1' and Block_Col = '0'		else
				'1'	when	R_Req = '1'								else	
				'0';	
	F_TxB <=	Tx_LatchH	WHEN	H_Byte = '0'	ELSE
				Tx_Buf;

	nTx_Int <= '1'	WHEN	(Tx_Icnt = 0 AND Tx_SoftInt = '0') OR Tx_Ie = '0'	ELSE	'0';

	Tx_Idle <= '1'	WHEN	Sm_Tx = R_Idl AND Dsm = sIdle ELSE '0';

	Tx_Reg(15 downto 4) <= Tx_Ie & Tx_SoftInt & Tx_Half & Tx_Wait & (Tx_Icnt(4) or Tx_Icnt(3)) & Tx_Icnt(2 downto 0)
						 & Tx_On &  Tx_BegInt & Tx_Idle & "0" ;

	Tx_Reg( 3 downto 0) <=  Tx_Desc;
	

	Sel_TxH <= '1'	WHEN s_nWr = '0' AND Sel_Cont = '1' AND s_Adr(3) = '0' AND	Ram_Be(1) = '1'	ELSE	'0';
	Sel_TxL <= '1'	WHEN s_nWr = '0' AND Sel_Cont = '1' AND s_Adr(3) = '0' AND	Ram_Be(0) = '1'	ELSE	'0';

	Tx_Desc <= Tx_Desc_One;

pTxRegs: Process( nRes, Clk )
Begin

	IF	nRes = '0'	THEN
		Tx_On   <= '0'; Tx_Ie <= '0'; Tx_Half  <= '0'; Tx_Wait  <= '0'; nTx_BegInt <= '0';
		Tx_Desc_One <= (others => '0');
		Tx_Icnt <= (others => '0'); TxInt <= '0'; Tx_SoftInt <= '0'; Tx_BegInt  <= '0';
		Tx_Ipg  <= conv_std_logic_vector( 42, 6);	
	ELSIF	rising_edge( Clk )	THEN

		IF	Sel_TxL = '1'	THEN
			IF		s_Adr(2 downto 1) = "00"						THEN	Tx_On  <= S_Din( 7);
			ELSIF	s_Adr(2 downto 1) = "01" AND S_Din( 7) = '1'	THEN	Tx_On  <= '1';
			ELSIF	s_Adr(2 downto 1) = "10" AND S_Din( 7) = '1'	THEN	Tx_On  <= '0';
			END IF;
		END IF;

		if		Tx_BegSet = '1'	and Tx_Ie = '1' 								then	Tx_BegInt  <= '1';	
		elsif	Sel_TxL = '1' and s_Adr(2 downto 1) = "01" AND S_Din( 6) = '1'	then	Tx_BegInt  <= '1';
		elsif	Sel_TxL = '1' and s_Adr(2 downto 1) = "10" AND S_Din( 6) = '1'	then	Tx_BegInt  <= '0';
		end if;

		nTx_BegInt <= not Tx_BegInt;

		IF	Sel_TxL = '1' AND s_Adr(2 downto 1) = "11"				THEN	Tx_Desc_One <= S_Din( 3 downto 0);
		ELSIF	Dsm = sStat AND Ext_Tx = '0'	 THEN
			IF		Last_Desc = '1'									THEN	Tx_Desc_One <= x"0";
			ELSE															Tx_Desc_One <= Tx_Desc + 1;
			END IF;
		END IF;
				
		IF		Sel_TxH = '1'	THEN
			IF		s_Adr(2 downto 1) = "00"						THEN	Tx_Ie  <= S_Din(15);
			ELSIF	s_Adr(2 downto 1) = "01" AND S_Din(15) = '1'	THEN	Tx_Ie  <= '1';
			ELSIF	s_Adr(2 downto 1) = "10" AND S_Din(15) = '1'	THEN	Tx_Ie  <= '0';
			END IF;
		END IF;

		IF		Sel_TxH = '1'	THEN
			IF		s_Adr(2 downto 1) = "00"						THEN	Tx_Half  <= S_Din(13);
			ELSIF	s_Adr(2 downto 1) = "01" AND S_Din(13) = '1'	THEN	Tx_Half  <= '1';
			ELSIF	s_Adr(2 downto 1) = "10" AND S_Din(13) = '1'	THEN	Tx_Half  <= '0';
			END IF;
		END IF;

		IF		Sel_TxH = '1'	THEN
			IF		s_Adr(2 downto 1) = "00"						THEN	Tx_Wait  <= S_Din(12);
			ELSIF	s_Adr(2 downto 1) = "01" AND S_Din(12) = '1'	THEN	Tx_Wait  <= '1';
			ELSIF	s_Adr(2 downto 1) = "10" AND S_Din(12) = '1'	THEN	Tx_Wait  <= '0';
			END IF;
		END IF;

		IF		Sel_TxH = '1'	THEN
			IF		s_Adr(2 downto 1) = "11" AND S_Din(14) = '1'	THEN	Tx_Ipg	    <= S_Din(13 downto 8);
			END IF;
		END IF;

		IF		Tx_Ie = '1' AND Dsm = sStat AND Desc_We = '1'		THEN	TxInt <= '1';
		ELSE																TxInt <= '0';
		END IF;	

		IF		Sel_TxH = '1' AND s_Adr(2 downto 1) = "10" AND S_Din(8) = '1'	
			AND	Tx_Icnt /= 0										THEN	Tx_Icnt <= Tx_Icnt - NOT TxInt;
		ELSIF	TxInt = '1'	AND Tx_Icnt /= "11111"					THEN	Tx_Icnt <= Tx_Icnt + 1;
		END IF;

	END IF;	

End Process pTxRegs;


END BLOCK bTxDesc;

END BLOCK b_Full_Tx;


b_Full_Rx:	BLOCK

	TYPE	MACRX_TYPE IS ( R_Idl, R_Sof, R_Rxd );
	SIGNAL	Sm_Rx						: MACRX_TYPE;
	SIGNAL	Rx_Dat, Rx_DatL				: std_logic_vector( 1 downto 0);	
	SIGNAL	Tx_Timer					: std_logic_vector( 7 downto 0);	
	SIGNAL	Dibl_Cnt					: std_logic_vector( 1 downto 0);	
	SIGNAL	Crc, nCrc					: std_logic_vector(31 downto 0);
	SIGNAL	CrcDin						: std_logic_vector( 1 downto 0);	
	SIGNAL	F_Err, P_Err, N_Err, A_Err	: std_logic;						
	SIGNAL	F_End, F_Val, Rx_Beg		: std_logic;						
	SIGNAL	Rx_Sr						: std_logic_vector( 7 downto 0);
	SIGNAL	nCrc_Ok, Crc_Ok				: std_logic;						
	SIGNAL	WrDescStat					: std_logic;
	SIGNAL	PreCount					: std_logic_vector( 4 downto 0);
	SIGNAL	PreBeg, PreErr				: std_logic;
	SIGNAL	Rx_DvL						: std_logic;
	SIGNAL	Diag						: std_logic;

BEGIN

	Rx_Beg <= '1' WHEN	Rx_Dv = '1' and Sm_Rx = R_SOF AND Rx_Dat = "11"	ELSE '0';

	nCrc_Ok <= '1' WHEN nCrc = x"C704DD7B"	ELSE '0';	

rxsm: PROCESS ( Clk, nRes )	 IS
BEGIN

	IF nRes = '0'	THEN
		Sm_Rx <= R_Idl;
	ELSIF rising_edge( Clk ) 	THEN
		IF	Sm_Rx = R_Idl OR Sm_Rx = R_Rxd OR Sm_Rx = R_Sof OR Dibl_Cnt = "11"	THEN
			CASE Sm_Rx	IS
				WHEN	R_Idl =>	IF		Rx_Dv = '1'		THEN	Sm_Rx	<= R_Sof;	END IF;
				WHEN	R_Sof =>	IF		Rx_Dat = "11"	THEN	Sm_Rx	<= R_Rxd;
									ELSIF	Rx_Dv = '0'		THEN	Sm_Rx	<= R_Idl;	END IF;
				WHEN	R_Rxd =>	IF		Rx_Dv = '0'		THEN	Sm_Rx	<= R_Idl;	END IF;
				WHEN  others  =>	NULL;
			END CASE;
		END IF;
	END IF; 

END PROCESS rxsm;

pRxCtl: PROCESS ( Clk, nRes )	 IS
	VARIABLE	Preload		:	std_logic_vector(Tx_Timer'range);
	VARIABLE	Load		:	std_logic;
BEGIN

	IF nRes = '0'	THEN
		Rx_DatL <= "00"; Rx_Dat <= "00"; Rx_Dv <= '0'; Dibl_Cnt <= "00"; PreCount <= (others => '0');
		F_End <= '0'; F_Err <= '0';  F_Val <= '0'; Crc_Ok <= '0'; 
		A_Err <= '0'; N_Err <= '0'; P_Err <= '0'; PreBeg <= '0'; PreErr <= '0';
	ELSIF rising_edge( Clk ) 	THEN

		Rx_DatL <= rRx_Dat;		

		Rx_Dat <= Rx_DatL;	
		
		IF		Rx_Dv = '0' AND rCrs_Dv = '1'						THEN	Rx_Dv <= '1';	
		ELSIF	Rx_Dv = '1'	AND rCrs_Dv = '0' AND Dibl_Cnt(0) = '1'	THEN	Rx_Dv <= '0';	
		END  IF;	

		IF		Rx_Beg = '1'	THEN	Dibl_Cnt <= "00";			
		ELSE							Dibl_Cnt <= Dibl_Cnt + 1;
		END IF;

		Crc_Ok <= nCrc_Ok;

		IF		(Sm_Rx = R_Rxd AND Rx_Dv = '0')			THEN	F_End <= '1';	
																F_Err <= NOT Crc_Ok;
		ELSE													F_End <= '0';
		END IF;

		IF		Dibl_Cnt = "11" AND Sm_Rx = R_Rxd		THEN	F_Val <= '1';	
		ELSE													F_Val <= '0';
		END IF;

		IF		WrDescStat = '1'						THEN	A_Err <= '0';	
		ELSIF	F_End = '1' and Dibl_Cnt /= 1			THEN	A_Err <= '1';	
		END IF;		

		IF		Rx_Dv = '0' OR Rx_Dat(0) = '0'			THEN	PreCount <= (others => '1');
		ELSE													PreCount <= PreCount - 1;
		END IF;

		IF		Rx_Dv  = '0'	then	PreBeg <= '0';		
		ELSIF	Rx_Dat = "01"	then	PreBeg <= '1';
		END IF;

		IF		WrDescStat = '1'									then	N_Err <= '0';
		ELSIF	Sm_Rx = R_Sof AND Rx_Dv  = '0'						then	N_Err <= '1';	
		END IF;	

		IF		Rx_DvL = '0'										then	PreErr <= '0'; 
		ELSIF	PreBeg = '0' AND (Rx_Dat = "10" or Rx_Dat = "11")	then	PreErr <= '1'; 
		ELSIF	PreBeg = '1' AND (Rx_Dat = "10" or Rx_Dat = "00")	then	PreErr <= '1';
		END IF;

		IF		WrDescStat = '1'				then	P_Err <= '0';
		ELSIF	Rx_Beg = '1' AND PreErr = '1'	then	P_Err <= '1';
		ELSIF	Rx_Beg = '1' AND PreCount /= 0	then	P_Err <= '1';
		END IF;	

		Rx_Sr <= Rx_Dat(1) & Rx_Dat(0) & Rx_Sr(7 downto 2);

		Rx_DvL  <= Rx_Dv;

	END IF; 

END PROCESS pRxCtl;

	CrcDin <= Rx_Dat;

Calc: PROCESS ( Clk, Crc, nCrc, CrcDin, Sm_Rx )	 IS
	VARIABLE	H 		: std_logic_vector(1 downto 0);
BEGIN

	H(0) := Crc(31) XOR CrcDin(0);
	H(1) := Crc(30) XOR CrcDin(1);

	IF		Sm_Rx = R_Sof 	THEN	nCrc <= x"FFFFFFFF";
	ELSE
		nCrc( 0) <=		                 H(1);		
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

bRxDesc:	BLOCK
	TYPE	sDESC	IS	(sIdle, sLen, sTimL, sTimH, sAdrH, sAdrL, sData, sOdd, sStat, sLenW );
	SIGNAL	Dsm, Rx_Dsm_Next						: sDESC;
	SIGNAL	Rx_Buf, Rx_LatchH, Rx_LatchL			: std_logic_vector( 7 downto 0);
	SIGNAL	Rx_Ovr									: std_logic;
	SIGNAL	DescRam_Out, DescRam_In					: std_logic_vector(15 downto 0);
	 ALIAS	RX_LEN									: std_logic_vector(11 downto 0)	IS	DescRam_Out(11 downto  0);
	 ALIAS	RX_OWN									: std_logic						IS	DescRam_Out( 8);
	 ALIAS	RX_LAST									: std_logic						IS	DescRam_Out( 9);
	SIGNAL	Ram_Be									: std_logic_vector( 1 downto 0);
	SIGNAL	Ram_Wr, Desc_We							: std_logic;
	SIGNAL	Desc_Addr								: std_logic_vector( 7 downto 0);
	SIGNAL	ZeitL									: std_logic_vector(15 downto 0);	
	SIGNAL	Rx_On, Rx_Ie, Sel_RxH, Sel_RxL			: std_logic;
	SIGNAL	Rx_Desc, Match_Desc						: std_logic_vector( 3 downto 0);
	SIGNAL	Rx_Icnt									: std_logic_vector( 4 downto 0);
	SIGNAL	Rx_Lost, Last_Desc, Answer_Tx			: std_logic;
	SIGNAL	DescIdx									: std_logic_vector( 2 downto 0);	
	SIGNAL	Rx_Count, Rx_Limit						: std_logic_vector(11 downto 0);	
	SIGNAL	Match, Filt_Cmp							: std_logic;
	SIGNAL	Rx_Idle, RxInt			 				: std_logic;
	SIGNAL	Hub_Rx_L								: std_logic_vector( 1 downto 0);
	SIGNAL	Rx_Dma_Out								: std_logic;

BEGIN
	WrDescStat <= '1' WHEN Dsm = sStat	ELSE '0';	

	Ram_Wr    <= '1' WHEN	s_nWr = '0' AND Sel_Ram = '1' AND s_Adr(10) = '1'	ELSE '0';
	Ram_Be(1) <= '1' WHEN	s_nWr = '1' OR s_nBE(1) = '0'						ELSE '0';
	Ram_Be(0) <= '1' WHEN	s_nWr = '1' OR s_nBE(0) = '0'						ELSE '0';

	DescIdx <=	"001"	WHEN	Desc_We = '0' and (Rx_Dsm_Next = sLen OR Rx_Dsm_Next = sLenW)	ELSE
				"001"	WHEN	Desc_We = '1' and (Dsm         = sLen OR Dsm         = sLenW)	ELSE
				"010"	WHEN	Desc_We = '0' and Rx_Dsm_Next = sAdrH							ELSE
				"010"	WHEN	Desc_We = '1' and Dsm         = sAdrH							ELSE
				"011"	WHEN	Desc_We = '0' and Rx_Dsm_Next = sAdrL							ELSE
				"011"	WHEN	Desc_We = '1' and Dsm         = sAdrL							ELSE
				"110"	WHEN	Desc_We = '0' and Rx_Dsm_Next = sTimH							ELSE
				"110"	WHEN	Desc_We = '1' and Dsm         = sTimH							ELSE
				"111"	WHEN	Desc_We = '0' and Rx_Dsm_Next = sTimL							ELSE
				"111"	WHEN	Desc_We = '1' and Dsm         = sTimL							ELSE
				"000";
	
	Desc_We <= '1'	WHEN   Dsm = sTimL OR Dsm = sTimH					ELSE
			   '1'	WHEN  (Dsm = sLenW OR Dsm = sStat) AND Match = '1'  ELSE	'0';

	Desc_Addr <= "0" & Rx_Desc & DescIdx;	

gRxTime:	if timer generate
	DescRam_In <= Zeit(31 downto 16)			WHEN	Dsm = sTimH		ELSE
				  ZeitL							WHEN	Dsm = sTimL		ELSE
				  x"0"  & Rx_Count				WHEN	Dsm = sLenW		ELSE
				  Rx_Dma_Out & '0' & "0" & A_Err & Hub_Rx_L & "00" & Match_Desc & N_Err & P_Err & Rx_Ovr & F_Err;
end generate;

ngRxTime:	if NOT timer generate
	DescRam_In <= x"0"  & Rx_Count				WHEN	Dsm = sLenW					ELSE
				  Rx_Dma_Out & '0' & "0" & A_Err & Hub_Rx_L & "00" & Match_Desc & N_Err & P_Err & Rx_Ovr & F_Err;
end generate;

RxRam:	entity	work.Dpr_16_16
	generic map(Simulate => Simulate)
	port map (	CLKA	=> Clk,					CLKB	=> Clk,
				EnA		=> log1,				Enb		=> log1,
				BEA		=> Ram_Be,				
				WEA		=> Ram_Wr,				WEB		=> Desc_We,
				ADDRA	=> s_Adr(8 downto 1),	ADDRB	=> Desc_Addr,
			  	DIA		=> s_Din,				DIB		=> DescRam_In,
				DOA		=> Rx_Ram_Dat,			DOB		=> DescRam_Out
			);


pRxSm: Process( nRes, Clk, Dsm,
				Rx_Beg, Rx_On, RX_OWN, F_End, F_Err, Diag, Rx_Count )
Begin

		Rx_Dsm_Next <= Dsm;
		CASE	Dsm IS
			WHEN sIdle	 =>	IF	Rx_Beg = '1' AND Rx_On = '1' AND RX_OWN = '1' THEN	
														Rx_Dsm_Next <= sLen;	
							END IF;
			WHEN sLen	 =>								Rx_Dsm_Next <= sAdrH;
			WHEN sAdrH	 =>								Rx_Dsm_Next <= sAdrL;
			WHEN sAdrL	 =>								Rx_Dsm_Next <= sTimH;
			WHEN sTimH	 =>								Rx_Dsm_Next <= sTimL;			
			WHEN sTimL	 =>								Rx_Dsm_Next <= sData;			
			WHEN sData	 =>	IF	F_End = '1'	THEN
								if	F_Err = '0'										
								 or Diag  = '1'	then	Rx_Dsm_Next <= sStat;		
								else					Rx_Dsm_Next <= sIdle;
								end if;
							END IF; 
			WHEN sStat	 =>								Rx_Dsm_Next <= sLenW;
			WHEN sLenW   =>	IF	Rx_Count(0) = '0' THEN	
														Rx_Dsm_Next <= sIdle;
							ELSE						Rx_Dsm_Next <= sOdd;	
							END IF;
			WHEN sOdd   =>								Rx_Dsm_Next <= sIdle;
			WHEN others	 =>
		END CASE;

	IF		nRes = '0'				THEN	Dsm <= sIdle;
	ELSIF	rising_edge( Clk )		THEN	Dsm <= Rx_Dsm_Next;
	END IF;

end process pRxSm;

pRxControl: Process( nRes, Clk )
Begin

	IF	nRes = '0'	THEN
		Rx_Ovr <= '0'; Rx_Dma_Req  <= '0'; Last_Desc <= '0'; Rx_Dma_Out <= '0';
		Rx_Count <= (others => '0');
		Rx_Buf <= (others => '0'); Rx_LatchL <= (others => '0'); Rx_LatchH <= (others => '0');
		Dma_Rx_Addr <= (others => '0');
	ELSIF	rising_edge( Clk ) 	THEN

		IF	Timer	THEN
			IF		Dsm  = sTimH	THEN	ZeitL <= Zeit(15 downto 0);
			END IF;
		END IF;

		IF		Dsm = sIdle		THEN	Rx_Count  <= (others => '0');
										Last_Desc <= RX_LAST;
		ELSIF   F_Val = '1'		THEN	Rx_Count  <= Rx_Count + 1;
		END IF;

		IF		Dsm = sLen		THEN	Rx_Limit      <= RX_LEN;
										Hub_Rx_L      <= Hub_Rx;
		END IF;

		IF	F_Val = '1'		THEN	Rx_Buf <= Rx_Sr;
		END IF;

		IF	(F_Val = '1' AND Rx_Count(0) = '1') OR	Dsm = sStat	 THEN	Rx_LatchH <= Rx_Buf;
																		Rx_LatchL <= Rx_Sr;
			if		Rx_Dma_Req = '1' and Sm_Rx /= R_Idl			then	Rx_Dma_Out <= '1';
			end if;
		elsif	Dsm = sLen										then	Rx_Dma_Out <= '0';
		END IF;

		IF		Dsm = sLen								THEN	Rx_Ovr <= '0';
		ELSIF	F_Val = '1' AND Rx_Limit = Rx_Count		THEN	Rx_Ovr <= '1';
		END IF;

		
		IF		Dsm = sAdrL		 THEN	Dma_Rx_Addr(15 downto 1) <= DescRam_Out(15 downto 1);	
		ELSIF	Rx_Dma_Ack = '1' THEN	Dma_Rx_Addr(15 downto 1) <= Dma_Rx_Addr(15 downto 1) + 1;	
		END IF;

		IF		Dsm = sAdrH		 THEN	
				Dma_Rx_Addr(Dma_Addr'high downto 16) <= DescRam_Out(Dma_Addr'high-16 downto 0);
		ELSIF	Rx_Dma_Ack = '1' AND Dma_Rx_Addr(15 downto 1) = x"FFF" & "111"	THEN	
				Dma_Rx_Addr(Dma_Addr'high downto 16) <= Dma_Rx_Addr(Dma_Addr'high downto 16) + 1;	
		END IF;

		IF		Filt_Cmp = '0' AND Match ='0'										THEN	Rx_Dma_Req  <= '0';
			
		ELSIF  (Dsm = sOdd  AND Rx_Ovr = '0')	
			OR (Dsm = sData AND Rx_Ovr = '0' AND F_Val = '1' AND Rx_Count(0) = '1')	THEN	Rx_Dma_Req  <= '1' after 101 nS;
		ELSIF	Rx_Dma_Ack = '1'													THEN	Rx_Dma_Req  <= '0';
		END IF;

	END IF;	

End Process pRxControl;

	Dma_Dout <= Rx_LatchH & Rx_LatchL;

	nRx_Int <= '1'	WHEN	Rx_Icnt = 0 OR Rx_Ie = '0'	 	ELSE	'0';

	Rx_Idle <= '1'	WHEN	Sm_Rx = R_Idl ELSE '0';

	Rx_Reg(15 downto 4) <= Rx_Ie & '0' & "0"      & '0'     & (Rx_Icnt(4) or Rx_Icnt(3)) & Rx_Icnt(2 downto 0)
						 & Rx_On & "0" & Rx_Idle & Rx_Lost;	

	Rx_Reg( 3 downto 0) <= Rx_Desc;

bFilter: Block
	SIGNAL	Ram_Addr					: std_logic_vector( 7 downto 0);
	SIGNAL	Ram_BeH, Ram_BeL			: std_logic_vector( 1 downto 0);
	SIGNAL	Ram_Wr						: std_logic;
	SIGNAL	Filter_Addr					: std_logic_vector( 6 downto 0);
 	SIGNAL	Filter_Out_H, Filter_Out_L	: std_logic_vector(31 downto 0);
	 ALIAS	DIRON_0						: std_logic	IS	Filter_Out_H( 11);
	 ALIAS	DIRON_1						: std_logic	IS	Filter_Out_H( 27);
	 ALIAS	DIRON_2						: std_logic	IS	Filter_Out_L( 11);
	 ALIAS	DIRON_3						: std_logic	IS	Filter_Out_L( 27);
	 ALIAS	TX_0						: std_logic	IS	Filter_Out_H( 7);
	 ALIAS	TX_1						: std_logic	IS	Filter_Out_H(23);
	 ALIAS	TX_2						: std_logic	IS	Filter_Out_L( 7);
	 ALIAS	TX_3						: std_logic	IS	Filter_Out_L(23);
 	 ALIAS	ON_0						: std_logic	IS	Filter_Out_H( 6);
	 ALIAS	ON_1						: std_logic	IS	Filter_Out_H(22);
 	 ALIAS	ON_2						: std_logic	IS	Filter_Out_L( 6);
	 ALIAS	ON_3						: std_logic	IS	Filter_Out_L(22);
	 ALIAS	DESC_0						: std_logic_vector( 3 downto 0)	IS	Filter_Out_H( 3 downto  0);
	 ALIAS	DESC_1						: std_logic_vector( 3 downto 0)	IS	Filter_Out_H(19 downto 16);
	 ALIAS	DESC_2						: std_logic_vector( 3 downto 0)	IS	Filter_Out_L( 3 downto  0);
	 ALIAS	DESC_3						: std_logic_vector( 3 downto 0)	IS	Filter_Out_L(19 downto 16);

	SIGNAL	Byte_Cnt					: std_logic_vector( 4 downto 0) := (others => '0'); 
	SIGNAL	Erg0, Erg1, Erg2, Erg3		: std_logic_vector( 7 downto 0);						
	SIGNAL	Mat_Reg						: std_logic_vector(15 downto 0);
 	SIGNAL	Filt_Idx					: std_logic_vector( 1 downto 0);
 	SIGNAL	Mat_Sel						: std_logic_vector( 3 downto 0);
 	SIGNAL	M_Prio						: std_logic_vector( 2 downto 0);
	 ALIAS	Found						: std_logic IS M_Prio(2);

BEGIN
	Ram_Addr   <= s_Adr(9 downto 8) & s_Adr(5 downto 1) & s_Adr(6);	

	Ram_Wr     <= '1' WHEN	s_nWr = '0' AND Sel_Ram = '1'  AND s_Adr(10) = '0'	ELSE '0';
	Ram_BeH(1) <= '1' WHEN	s_nWr = '1' OR (s_nBE(1) = '0' AND s_Adr( 7) = '0')	ELSE '0';
	Ram_BeH(0) <= '1' WHEN	s_nWr = '1' OR (s_nBE(0) = '0' AND s_Adr( 7) = '0')	ELSE '0';
	Ram_BeL(1) <= '1' WHEN	s_nWr = '1' OR (s_nBE(1) = '0' AND s_Adr( 7) = '1')	ELSE '0';
	Ram_BeL(0) <= '1' WHEN	s_nWr = '1' OR (s_nBE(0) = '0' AND s_Adr( 7) = '1')	ELSE '0';

	Filter_Addr <= Dibl_Cnt & Byte_Cnt;

FiltRamH:	entity	work.Dpr_16_32
	generic map(Simulate => Simulate)
	port map (	CLKA	=> Clk,			CLKB	=> Clk,
				EnA		=> log1,		EnB		=> log1,
				BEA		=> Ram_BeH,		
				WEA		=> Ram_Wr,
				ADDRA	=> Ram_Addr,	ADDRB	=> Filter_Addr,
			  	DIA		=> s_Din,		DOB		=> Filter_Out_H
				);

FiltRamL:	entity	work.Dpr_16_32
	generic map(Simulate => Simulate)
	port map (	CLKA	=> Clk,			CLKB	=> Clk,
				EnA		=> log1,		EnB		=> log1,
				BEA		=> Ram_BeL,		
				WEA		=> Ram_Wr,
				ADDRA	=> Ram_Addr,	ADDRB	=> Filter_Addr,
				DIA		=> s_Din,		DOB		=> Filter_Out_L
			);

	Erg0 <= (Rx_Buf XOR Filter_Out_H( 7 downto  0)) AND Filter_Out_H(15 downto  8);	
 	Erg1 <= (Rx_Buf XOR Filter_Out_H(23 downto 16)) AND Filter_Out_H(31 downto 24);
	Erg2 <= (Rx_Buf XOR Filter_Out_L( 7 downto  0)) AND Filter_Out_L(15 downto  8);
	Erg3 <= (Rx_Buf XOR Filter_Out_L(23 downto 16)) AND Filter_Out_L(31 downto 24);

genMatSel:	for i in 0 to 3 generate					
	Mat_Sel(i) <=	Mat_Reg( 0 + i)	WHEN  Filt_Idx = "00"	ELSE
					Mat_Reg( 4 + i)	WHEN  Filt_Idx = "01"	ELSE
					Mat_Reg( 8 + i)	WHEN  Filt_Idx = "10"	ELSE
					Mat_Reg(12 + i)	WHEN  Filt_Idx = "11";
end generate;
	
	M_Prio <= "000" WHEN    Filt_Cmp = '0' OR Match = '1'													ELSE		
			  "100"	WHEN	Mat_Sel(0) = '1'  AND On_0 = '1' and (DIRON_0 = '0')	ELSE	
			  "101"	WHEN	Mat_Sel(1) = '1'  AND On_1 = '1' and (DIRON_1 = '0')	ELSE	
			  "110"	WHEN	Mat_Sel(2) = '1'  AND On_2 = '1' and (DIRON_2 = '0')	ELSE	
			  "111"	WHEN	Mat_Sel(3) = '1'  AND On_3 = '1' and (DIRON_3 = '0')	ELSE
			  "000";

pFilter: Process( nRes, Clk )
Begin

	IF	nRes = '0'	THEN
		Filt_Idx <= "00"; Match <= '0'; 
		Filt_Cmp <= '0'; Mat_Reg <= (others => '0'); Byte_Cnt <= (others =>'0');
		Match_Desc <= (others => '0');Auto_Desc <= (others =>'0'); Answer_Tx <= '0';
	ELSIF	rising_edge( Clk ) 	THEN

		Filt_Idx  <= Dibl_Cnt;

		IF		Dibl_Cnt = "11"	AND Rx_Count(5) = '0'	THEN	Byte_Cnt <= Rx_Count(Byte_Cnt'range);
		END IF;

		IF		Dsm = sTiml											THEN	Filt_Cmp  <= '1';	
		ELSIF	Rx_Dv = '0'	OR (F_Val = '1' AND Rx_Count(5) = '1')	THEN	Filt_Cmp  <= '0';	
		END IF;

		IF		Dsm = sTimL		THEN	Mat_Reg <= (others => '1');		
		ELSE
			for i in 0 to 3 loop										
				IF	Erg0 /= 0 AND conv_integer(Filt_Idx) = i	THEN	Mat_Reg(4*i + 0) <= '0';	END IF;
				IF	Erg1 /= 0 AND conv_integer(Filt_Idx) = i	THEN	Mat_Reg(4*i + 1) <= '0';	END IF;
				IF	Erg2 /= 0 AND conv_integer(Filt_Idx) = i	THEN	Mat_Reg(4*i + 2) <= '0';	END IF;
				IF	Erg3 /= 0 AND conv_integer(Filt_Idx) = i	THEN	Mat_Reg(4*i + 3) <= '0';	END IF;
			end loop;
		END IF;

		IF		Dsm = sTimL						THEN	Match <= '0';
		ELSIF	Found = '1'						THEN	Match <= '1';		Match_Desc <= Filt_Idx & M_Prio(1 downto 0);		
			IF		M_Prio(1 downto 0) = "00"	THEN	Answer_Tx <= TX_0;	Auto_Desc  <= DESC_0;  
			ELSIF	M_Prio(1 downto 0) = "01"	THEN	Answer_Tx <= TX_1;	Auto_Desc  <= DESC_1;  
			ELSIF	M_Prio(1 downto 0) = "10"	THEN	Answer_Tx <= TX_2;	Auto_Desc  <= DESC_2;  
			ELSIF	M_Prio(1 downto 0) = "11"	THEN	Answer_Tx <= TX_3;	Auto_Desc  <= DESC_3;  
			END IF;
		ELSIF	F_End = '1'						THEN	Answer_Tx <= '0';						   							
		END IF;

	END IF;	

End Process pFilter;

	R_Req  <= Answer_Tx when F_End = '1' AND F_Err = '0'  	else '0';	

END BLOCK  bFilter;


	Sel_RxH <= '1'	WHEN s_nWr = '0' AND Sel_Cont = '1' AND s_Adr(3) = '1' AND	s_nBe(1) = '0'	ELSE	'0';
	Sel_RxL <= '1'	WHEN s_nWr = '0' AND Sel_Cont = '1' AND s_Adr(3) = '1' AND	s_nBe(0) = '0'	ELSE	'0';

pRxRegs: Process( nRes, Clk )
Begin

	IF	nRes = '0'	THEN
		Rx_Desc <= (others => '0');	 Rx_On  <= '0';
		Rx_Ie   <= '0';	Rx_Lost <= '0';	Rx_Icnt <= (others => '0'); RxInt <= '0'; Diag  <= '0';
	ELSIF	rising_edge( Clk )	THEN

		IF	Sel_RxH = '1'	THEN
			IF		s_Adr(2 downto 1) = "00"						THEN	Rx_Ie  <= S_Din(15);
			ELSIF	s_Adr(2 downto 1) = "01" AND S_Din(15) = '1'	THEN	Rx_Ie  <= '1';
			ELSIF	s_Adr(2 downto 1) = "10" AND S_Din(15) = '1'	THEN	Rx_Ie  <= '0';
			END IF;
		END IF;

		IF	Sel_RxH = '1'	THEN
			IF		s_Adr(2 downto 1) = "00"						THEN	Diag  <= S_Din(12);
			ELSIF	s_Adr(2 downto 1) = "01" AND S_Din(12) = '1'	THEN	Diag  <= '1';
			ELSIF	s_Adr(2 downto 1) = "10" AND S_Din(12) = '1'	THEN	Diag  <= '0';
			END IF;
		END IF;

		IF	Sel_RxL = '1'	THEN
			IF		s_Adr(2 downto 1) = "00"						THEN	Rx_On  <= S_Din( 7);
			ELSIF	s_Adr(2 downto 1) = "01" AND S_Din( 7) = '1'	THEN	Rx_On  <= '1';
			ELSIF	s_Adr(2 downto 1) = "10" AND S_Din( 7) = '1'	THEN	Rx_On  <= '0';
			END IF;
		END IF;

		IF		Rx_Beg  = '1' AND (RX_OWN = '0' OR Rx_On = '0')					THEN	Rx_Lost  <= '1';		
		ELSIF	Sel_RxL = '1' AND s_Adr(2 downto 1) = "10" AND S_Din( 4) = '1'	THEN	Rx_Lost  <= '0';		
		END IF;

		IF		Sel_RxL = '1' AND s_Adr(2 downto 1) = "11"			THEN	Rx_Desc <= S_Din( 3 downto 0);
		ELSIF	Dsm = sLenW AND Desc_We = '1'  THEN
			IF		Last_Desc = '1'									THEN	Rx_Desc <= x"0";
			ELSE															Rx_Desc <= Rx_Desc + 1;
			END IF;
		END IF;

		IF		Rx_Ie = '1' AND Desc_We = '1' and Dsm = sStat		then	RxInt <= '1';
		else																RxInt <= '0';
		END IF;	

		IF		Sel_RxH = '1' AND s_Adr(2 downto 1) = "10" AND S_Din(8) = '1'	
			AND	Rx_Icnt /= 0										THEN	Rx_Icnt <= Rx_Icnt - NOT RxInt;
		ELSIF	RxInt = '1'	AND Rx_Icnt /= "11111"					THEN	Rx_Icnt <= Rx_Icnt + 1;
		END IF;

	END IF;	

End Process pRxRegs;

END BLOCK bRxDesc;

END BLOCK b_Full_Rx;

END ARCHITECTURE struct;




