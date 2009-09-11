------------------------------------------------------------------------------------------------------------------------
-- Phy Management Interface for OpenMAC
--
-- Copyright (C) 2009 B&R
--
-- This program is free software; you can redistribute it and/or modify
-- it under the terms of the GNU General Public License as published by
-- the Free Software Foundation; either version 2 of the License, or
-- (at your option) any later version.
--
------------------------------------------------------------------------------------------------------------------------
-- Version History
------------------------------------------------------------------------------------------------------------------------      
-- 2009-08-07  V0.01        Converted to official version.
-- 2009-09-07  V0.02		changed tristate port to In/Out and enable (Xilinx XPS doesn't like IO Ports...)
------------------------------------------------------------------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_unsigned.ALL;
USE ieee.std_logic_1164.ALL;
USE ieee.std_logic_arith.ALL;

ENTITY OpenMAC_MII IS
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
			Mii_Doe : out std_logic; --'1' ... Input / '0' ... Output!!!
			nResetOut	: OUT	std_logic;
			NodeNr		: IN	std_logic_vector(7 downto 0) := (others => '0')
		);
END ENTITY OpenMAC_MII;

ARCHITECTURE struct OF OpenMAC_MII IS
    SIGNAL	ShiftReg				: std_logic_vector (31 DOWNTO 0);	
    SIGNAL	iMiiClk					: std_logic;						
    SIGNAL	ClkDiv					: std_logic_vector (4 DOWNTO 0);		
	 ALIAS	Shift					: std_logic	IS ClkDiv(ClkDiv'high);
    SIGNAL	BitCnt					: std_logic_vector (2 DOWNTO 0);		
    SIGNAL	BytCnt					: std_logic_vector (2 DOWNTO 0);		
    SIGNAL	Run, SrBusy, nReset		: std_logic;
    SIGNAL	M_Dout, M_Oe			: std_logic;
BEGIN

	Data_Out	<= nodenr & nReset & "000000" & SrBusy		WHEN Addr(0) = '0'	ELSE
				   ShiftReg(15 DOWNTO 0);

	Mii_Clk		<= iMiiClk;
--	Mii_Dio		<= M_Dout	WHEN M_Oe = '1'	ELSE 'Z';
	Mii_Do		<= M_Dout	WHEN M_Oe = '1'	ELSE 'Z';
	Mii_Doe 		<= not M_Oe;
	
	nresetout	<= nReset;	

p_Mii: PROCESS (Clk, nRst)
BEGIN

	IF nRst = '0'	THEN
		iMiiClk <= '0'; Run <= '0'; SrBusy <= '0'; M_Oe <= '1'; M_Dout <= '1'; nReset <= '0';
		BitCnt <= (OTHERS => '0');	 BytCnt <= (OTHERS => '0');
		ShiftReg <= x"0000ABCD"; ClkDiv <= (OTHERS => '0');
	ELSIF rising_edge( Clk ) THEN

		IF	Shift = '1'		THEN	ClkDiv  <= conv_std_logic_vector( 8, ClkDiv'high + 1);
									iMiiClk <= NOT iMiiClk;
		ELSE						ClkDiv  <= ClkDiv - 1;			
		END IF;

		IF	Sel = '1' AND nWr = '0' AND SrBusy = '0' AND Addr(1) = '1' AND nBE(0) = '0'	THEN	nReset <= Data_In(7);
		END IF;

		IF	Sel = '1' AND nWr = '0' AND SrBusy = '0' AND Addr(1) = '0'	THEN	
			IF	Addr(0) = '0'	THEN
				IF	nBE(1) = '0' THEN	ShiftReg(31 DOWNTO 24) <= Data_In(15 DOWNTO 8);
				END IF;
				IF	nBE(0) = '0' THEN	ShiftReg(23 DOWNTO 16) <= Data_In( 7 DOWNTO 0);
										SrBusy <= '1';
				END IF;
			ELSE
				IF	nBE(1) = '0' THEN	ShiftReg(15 DOWNTO  8) <= Data_In(15 DOWNTO 8);
				END IF;
				IF	nBE(0) = '0' THEN	ShiftReg( 7 DOWNTO  0) <= Data_In( 7 DOWNTO 0);
				END IF;
			END IF;
		ELSE
			IF	Shift = '1' AND iMiiClk = '1' 	THEN
				IF	Run = '0' AND SrBusy = '1'	THEN
					Run    <= '1';
					BytCnt <= "111";				
					BitCnt <= "111";				
				ELSE
					IF	BytCnt(2) = '0' AND SrBusy = '1'	THEN				
						M_Dout <= ShiftReg(31);	
						ShiftReg <= ShiftReg(30 DOWNTO 0) & Mii_Di; -- & Mii_Dio;	
					END IF;		
					BitCnt <= BitCnt - 1;
					IF BitCnt = 0	THEN										
						BytCnt <= BytCnt - 1;									
						IF BytCnt = 0 	THEN	
							SrBusy <= '0';										
							Run    <= '0';
						END IF;
					END IF;
					IF	BytCnt = 2 AND BitCnt = 1 AND ShiftReg(31) = '0' THEN
						M_Oe   <= '0';		
					END IF;
				END IF;
				IF	SrBusy = '0' OR Run = '0'	THEN
					M_Dout <= '1';									
					M_Oe   <= '1';					
				END IF;
			END IF;
		END IF;
	END IF;

END PROCESS p_Mii;

END struct;