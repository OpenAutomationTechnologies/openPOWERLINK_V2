------------------------------------------------------------------------------------------------------------------------
-- Phy Management Interface for OpenMAC
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
-- 2009-08-07  	V0.01   			Converted to official version.
-- 2009-09-07  	V0.02	zelenkaj	Changed tristate port to In/Out and enable (Xilinx XPS doesn't like IO Ports...)
-- 2009-09-18  	V0.03	zelenkaj	Deleted NodeNr - isn't used by anyone...
-- 2011-11-28	V0.04	zelenkaj	Changed reset level to high-active
------------------------------------------------------------------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_unsigned.ALL;
USE ieee.std_logic_1164.ALL;
USE ieee.std_logic_arith.ALL;

ENTITY OpenMAC_MII IS
	PORT(	Clk			: IN	std_logic;						
			Rst			: IN	std_logic;						
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
			nResetOut	: OUT	std_logic
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

	Data_Out	<= x"00" & nReset & x"0" & "00" & SrBusy		WHEN Addr(0) = '0'	ELSE
				   ShiftReg(15 DOWNTO 0);

	Mii_Clk		<= iMiiClk;
--	Mii_Dio		<= M_Dout	WHEN M_Oe = '1'	ELSE 'Z';
	Mii_Do		<= M_Dout	WHEN M_Oe = '1'	ELSE 'Z';
	Mii_Doe 		<= not M_Oe;
	
	nresetout	<= nReset;	

p_Mii: PROCESS (Clk, Rst)
BEGIN

	IF Rst = '1'	THEN
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