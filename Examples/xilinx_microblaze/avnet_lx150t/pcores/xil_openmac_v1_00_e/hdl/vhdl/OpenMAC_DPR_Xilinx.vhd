------------------------------------------------------------------------------------------------------------------------
-- OpenMAC Memory (Xilinx)
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
--	           V0.00-0.30   First generation.
-- 2009-08-07  V0.31        Converted to official version.
------------------------------------------------------------------------------------------------------------------------

LIBRARY ieee;                   
USE ieee.std_logic_1164.all;    
USE ieee.std_logic_arith.all;   
USE ieee.std_logic_unsigned.all;

entity Dpr_16_16 is
  generic(Simulate	:  in	boolean);
  port (
	 ClkA,  ClkB		:  in  std_logic;
	 WeA,   WeB			:  in  std_logic := '0';
	 EnA,   EnB			:  in  std_logic := '1';
	 BeA				:  in  std_logic_vector( 1 downto 0) := "11";
	 AddrA				:  in  std_logic_vector;
	 DiA				:  in  std_logic_vector(15 downto 0) := (others => '0');
	 DoA				:  out std_logic_vector(15 downto 0); 
	 BeB				:  in  std_logic_vector( 1 downto 0) := "11";
	 AddrB 				:  in  std_logic_vector;
	 DiB				:  in  std_logic_vector(15 downto 0) := (others => '0');
	 DoB				:  out std_logic_vector(15 downto 0) 
	 );
end Dpr_16_16;

architecture struct of Dpr_16_16 is

	component dual_port_ram_16_16
		port (
		clka: IN std_logic;
		dina: IN std_logic_VECTOR(15 downto 0);
		addra: IN std_logic_VECTOR(7 downto 0);
		ena: IN std_logic;
		wea: IN std_logic_VECTOR(1 downto 0);
		douta: OUT std_logic_VECTOR(15 downto 0);
		clkb: IN std_logic;
		dinb: IN std_logic_VECTOR(15 downto 0);
		addrb: IN std_logic_VECTOR(7 downto 0);
		enb: IN std_logic;
		web: IN std_logic_VECTOR(1 downto 0);
		doutb: OUT std_logic_VECTOR(15 downto 0));
	end component;

	signal s_WeA, s_WeB : std_logic_vector(1 downto 0);
	
begin
	
	s_WeA	<=	(BeA(1) and WeA) & (BeA(0) and WeA);
	s_WeB	<=	(BeB(1) and WeB) & (BeB(0) and WeB);
	
	DPR1616 : dual_port_ram_16_16
			port map (
				clka => ClkA,
				dina => DiA,
				addra => AddrA,
				ena => EnA,
				wea => s_WeA,
				douta => DoA,
				clkb => ClkB,
				dinb => DiB,
				addrb => AddrB,
				enb => EnB,
				web => s_WeB,
				doutb => DoB);
	
end struct;

LIBRARY ieee;                   
USE ieee.std_logic_1164.all;    
USE ieee.std_logic_arith.all;   
USE ieee.std_logic_unsigned.all;

entity Dpr_16_32 is
  generic(Simulate	:  in	boolean);
  port (
	 ClkA,  ClkB		:  in  std_logic;
	 WeA				:  in  std_logic := '0';
	 EnA,   EnB			:  in  std_logic := '1';
	 AddrA				:  in  std_logic_vector;
	 DiA				:  in  std_logic_vector (15 downto 0) := (others => '0');
	 BeA				:  in  std_logic_vector ( 1 downto 0) := "11";
	 AddrB 				:  in  std_logic_vector;								
	 DoB				:  out  std_logic_vector(31 downto 0) 
	 );
end Dpr_16_32;

architecture struct of Dpr_16_32 is

	component dual_port_ram_16_32
		port (
			clka: IN std_logic;
			dina: IN std_logic_VECTOR(15 downto 0);
			addra: IN std_logic_VECTOR(7 downto 0);
			ena: IN std_logic;
			wea: IN std_logic_VECTOR(1 downto 0);
			clkb: IN std_logic;
			addrb: IN std_logic_VECTOR(6 downto 0);
			enb: IN std_logic;
			doutb: OUT std_logic_VECTOR(31 downto 0));
	end component;
	
	signal s_WeA : std_logic_vector(1 downto 0);
	
begin
	
	s_WeA	<=	(BeA(1) and WeA) & (BeA(0) and WeA);
	
	DPR1632 : dual_port_ram_16_32
			port map (
				clka => ClkA,
				dina => DiA,
				addra => AddrA,
				ena => EnA,
				wea => s_WeA,
				clkb => ClkB,
				addrb => AddrB,
				enb => EnB,
				doutb => DoB);
	
end struct;
