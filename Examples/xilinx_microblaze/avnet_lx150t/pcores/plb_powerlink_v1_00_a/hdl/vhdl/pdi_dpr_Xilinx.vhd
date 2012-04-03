------------------------------------------------------------------------------------------------------------------------
-- Process Data Interface (PDI) DPR for Xilinx
--
-- 	  Copyright (C) 2011 B&R
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
-- 2011-11-17  	V0.01	zelenkaj	First version
-- 2011-12-06	V0.02	zelenkaj	Uses openMAC DPR implementation
------------------------------------------------------------------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_1164.all;

ENTITY pdi_dpr IS
	GENERIC
	(
		NUM_WORDS		: INTEGER := 1024;
		LOG2_NUM_WORDS	: INTEGER := 10
	);
	PORT
	(
		address_a		: IN STD_LOGIC_VECTOR (LOG2_NUM_WORDS-1 DOWNTO 0);
		address_b		: IN STD_LOGIC_VECTOR (LOG2_NUM_WORDS-1 DOWNTO 0);
		byteena_a		: IN STD_LOGIC_VECTOR (3 DOWNTO 0) :=  (OTHERS => '1');
		byteena_b		: IN STD_LOGIC_VECTOR (3 DOWNTO 0) :=  (OTHERS => '1');
		clock_a		: IN STD_LOGIC  := '1';
		clock_b		: IN STD_LOGIC ;
		data_a		: IN STD_LOGIC_VECTOR (31 DOWNTO 0);
		data_b		: IN STD_LOGIC_VECTOR (31 DOWNTO 0);
		wren_a		: IN STD_LOGIC  := '0';
		wren_b		: IN STD_LOGIC  := '0';
		q_a		: OUT STD_LOGIC_VECTOR (31 DOWNTO 0);
		q_b		: OUT STD_LOGIC_VECTOR (31 DOWNTO 0)
	);
END pdi_dpr;

architecture struct of pdi_dpr is
	constant cActivated : std_logic := '1';
begin
	
	abuseMacDpr : entity work.dc_dpr_be
	generic map (
		WIDTH => data_a'length,
		SIZE => NUM_WORDS,
		ADDRWIDTH => LOG2_NUM_WORDS
	)
	port map (
		clkA => clock_a,		clkB => clock_b,
		enA => cActivated,	enB => cActivated,
		addrA => address_a, 	addrB => address_b,
		diA => data_a,			diB => data_b,
		doA => q_a, 			doB => q_b,
		weA => wren_a, 		weB => wren_b,
		beA => byteena_a,		beB => byteena_b
	);
	
end architecture struct;
