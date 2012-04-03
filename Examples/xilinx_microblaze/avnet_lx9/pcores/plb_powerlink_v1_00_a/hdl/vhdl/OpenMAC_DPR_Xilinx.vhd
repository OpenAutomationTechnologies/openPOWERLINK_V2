------------------------------------------------------------------------------------------------------------------------
-- OpenMAC - DPR for Xilinx FPGA
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
--  2009-08-07  V0.01	zelenkaj	Converted to official version.
--	2011-10-12	V0.10	zelenkaj	Implementation is based on UG687 (v13.2)
------------------------------------------------------------------------------------------------------------------------
--

-- dual clocked DPRAM for XILINX SPARTAN 6 --
library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;
use ieee.std_logic_arith.all;

entity dc_dpr is
	generic (
		WIDTH      	: integer := 16;
		SIZE       	: integer := 128;
		ADDRWIDTH  	: integer := 7
	);
	
	port (
		clkA   	: in  std_logic;
		clkB   	: in  std_logic;
		enA		: in  std_logic;
		enB		: in  std_logic;
		weA		: in std_logic;
		weB		: in std_logic;
		addrA  	: in  std_logic_vector(ADDRWIDTH-1 downto 0);
		addrB  	: in  std_logic_vector(ADDRWIDTH-1 downto 0);
		diA    	: in  std_logic_vector(WIDTH-1 downto 0);
		diB    	: in  std_logic_vector(WIDTH-1 downto 0);
		doA    	: out std_logic_vector(WIDTH-1 downto 0);
		doB    	: out std_logic_vector(WIDTH-1 downto 0)
	);
end dc_dpr;

architecture xilinx of dc_dpr is

  function log2 (val: INTEGER) return natural is
    variable res : natural;
  begin
        for i in 0 to 31 loop
            if (val <= (2**i)) then
                res := i;
                exit;
            end if;
        end loop;
        return res;
  end function Log2;

  type ramType is array (0 to SIZE-1) of std_logic_vector(WIDTH-1 downto 0);
  shared variable ram : ramType := (others => (others => '0'));
  
  signal readA : std_logic_vector(WIDTH-1 downto 0):= (others => '0');
  signal readB : std_logic_vector(WIDTH-1 downto 0):= (others => '0');
begin
	
	process (clkA)
	begin
		if rising_edge(clkA) then
			if enA = '1' then
				if weA = '1' then
					ram(conv_integer(addrA)) := diA;
				end if;
				readA <= ram(conv_integer(addrA));
			end if;
		end if;
	end process;
	
	doA <= readA;
	
	process (clkB)
	begin
		if rising_edge(clkB) then
			if enB = '1' then
				if weB = '1' then
					ram(conv_integer(addrB)) := diB;
				end if;
				readB <= ram(conv_integer(addrB));
			end if;
		end if;
	end process;
	
	doB <= readB;
	
end xilinx;

-- dual clocked DPRAM with byte enables for XILINX SPARTAN 6 --
library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;
use ieee.std_logic_arith.all;

entity dc_dpr_be is
	generic (
		WIDTH      	: integer := 16;
		SIZE       	: integer := 128;
		ADDRWIDTH  	: integer := 7
	);
	
	port (
		clkA   	: in  std_logic;
		clkB   	: in  std_logic;
		enA		: in  std_logic;
		enB		: in  std_logic;
		weA		: in std_logic;
		weB		: in std_logic;
		beA		: in std_logic_vector(WIDTH/8-1 downto 0);
		beB		: in std_logic_vector(WIDTH/8-1 downto 0);
		addrA  	: in  std_logic_vector(ADDRWIDTH-1 downto 0);
		addrB  	: in  std_logic_vector(ADDRWIDTH-1 downto 0);
		diA    	: in  std_logic_vector(WIDTH-1 downto 0);
		diB    	: in  std_logic_vector(WIDTH-1 downto 0);
		doA    	: out std_logic_vector(WIDTH-1 downto 0);
		doB    	: out std_logic_vector(WIDTH-1 downto 0)
	);
end dc_dpr_be;

architecture xilinx of dc_dpr_be is

  function log2 (val: INTEGER) return natural is
    variable res : natural;
  begin
        for i in 0 to 31 loop
            if (val <= (2**i)) then
                res := i;
                exit;
            end if;
        end loop;
        return res;
  end function Log2;

  type ramType is array (0 to SIZE-1) of std_logic_vector(WIDTH-1 downto 0);
  shared variable ram : ramType := (others => (others => '0'));
  
  constant BYTE : integer := 8;
  
  signal readA : std_logic_vector(WIDTH-1 downto 0):= (others => '0');
  signal readB : std_logic_vector(WIDTH-1 downto 0):= (others => '0');
begin
	
	process (clkA)
	begin
		if rising_edge(clkA) then
			if enA = '1' then
				if weA = '1' then
					for i in beA'range loop
						if beA(i) = '1' then
							ram(conv_integer(addrA))((i+1)*BYTE-1 downto i*BYTE) := diA((i+1)*BYTE-1 downto i*BYTE);
						end if;
					end loop;
				end if;
				readA <= ram(conv_integer(addrA));
			end if;
		end if;
	end process;
	
	doA <= readA;
	
	process (clkB)
	begin
		if rising_edge(clkB) then
			if enB = '1' then
				if weB = '1' then
					for i in beB'range loop
						if beB(i) = '1' then
							ram(conv_integer(addrB))((i+1)*BYTE-1 downto i*BYTE) := diB((i+1)*BYTE-1 downto i*BYTE);
						end if;
					end loop;
				end if;
				readB <= ram(conv_integer(addrB));
			end if;
		end if;
	end process;
	
	doB <= readB;
	
end xilinx;

-- dual clocked DPRAM with 16x16 --
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
	 BeA				:  in  std_logic_vector ( 1 downto 0) := "11";
	 AddrA				:  in  std_logic_vector ( 7 downto 0);
	 DiA				:  in  std_logic_vector (15 downto 0) := (others => '0');
	 DoA				:  out  std_logic_vector(15 downto 0); 
	 BeB				:  in  std_logic_vector ( 1 downto 0) := "11";
	 AddrB 				:  in  std_logic_vector ( 7 downto 0);
	 DiB				:  in  std_logic_vector (15 downto 0) := (others => '0');
	 DoB				:  out  std_logic_vector(15 downto 0) 
	 );
end Dpr_16_16;

architecture struct of Dpr_16_16 is
begin
	
	dpr_packet:		entity work.dc_dpr_be
	generic map (
		WIDTH => 16,
		SIZE => 2**AddrA'length,
		ADDRWIDTH => AddrA'length
	)
	port map (
		clkA => ClkA, 			clkB => ClkB,
		enA => EnA,				enB => EnB,
		addrA => AddrA, 		addrB => AddrB,
		diA => DiA,				diB => DiB,
		doA => DoA, 			doB => DoB,
		weA => WeA, 			weB => WeB,
		beA => BeA,				beB => BeB
	);
	
end struct;

-- dual clocked DPRAM with 16x32 --
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
	 AddrA				:  in  std_logic_vector ( 7 downto 0);
	 DiA				:  in  std_logic_vector (15 downto 0) := (others => '0');
	 BeA				:  in  std_logic_vector ( 1 downto 0) := "11";
	 AddrB 				:  in  std_logic_vector ( 6 downto 0);
	 DoB				:  out  std_logic_vector(31 downto 0) 
	 );
end Dpr_16_32;

architecture struct of Dpr_16_32 is
	signal addra_s : std_logic_vector(AddrB'range);
	signal dia_s : std_logic_vector(DoB'range);
	signal bea_s : std_logic_vector(DoB'length/8-1 downto 0);
begin
	
	dpr_packet:		entity work.dc_dpr_be
	generic map (
		WIDTH => 32,
		SIZE => 2**AddrB'length,
		ADDRWIDTH => AddrB'length
	)
	port map (
		clkA => ClkA, 			clkB => ClkB,
		enA => EnA,				enB => EnB,
		addrA => addra_s, 		addrB => AddrB,
		diA => dia_s,			diB => (others => '0'),
		doA => open, 			doB => DoB,
		weA => weA, 			weB => '0',
		beA => bea_s,			beB => (others => '1')
	);
	
	addra_s <= AddrA(AddrA'left downto 1);
	
	dia_s <= DiA & DiA;
	
	bea_s(3) <= BeA(1) and AddrA(0);
	bea_s(2) <= BeA(0) and AddrA(0);
	bea_s(1) <= BeA(1) and not AddrA(0);
	bea_s(0) <= BeA(0) and not AddrA(0);
	
end struct;

-- dual clocked DPRAM with 32x32 for packets --
LIBRARY ieee;                   
USE ieee.std_logic_1164.all;    
USE ieee.std_logic_arith.all;   
USE ieee.std_logic_unsigned.all;

ENTITY OpenMAC_DPRpackets IS
	GENERIC
	(
		memSizeLOG2_g : integer := 10;
		memSize_g : integer := 1024
	);
	PORT
	(
		address_a		: IN STD_LOGIC_VECTOR (memSizeLOG2_g-2 DOWNTO 0);
		address_b		: IN STD_LOGIC_VECTOR (memSizeLOG2_g-3 DOWNTO 0);
		byteena_a		: IN STD_LOGIC_VECTOR (1 DOWNTO 0) :=  (OTHERS => '1');
		byteena_b		: IN STD_LOGIC_VECTOR (3 DOWNTO 0) :=  (OTHERS => '1');
		clock_a		: IN STD_LOGIC  := '1';
		clock_b		: IN STD_LOGIC ;
		data_a		: IN STD_LOGIC_VECTOR (15 DOWNTO 0);
		data_b		: IN STD_LOGIC_VECTOR (31 DOWNTO 0);
		rden_a		: IN STD_LOGIC  := '1';
		rden_b		: IN STD_LOGIC  := '1';
		wren_a		: IN STD_LOGIC  := '0';
		wren_b		: IN STD_LOGIC  := '0';
		q_a		: OUT STD_LOGIC_VECTOR (15 DOWNTO 0);
		q_b		: OUT STD_LOGIC_VECTOR (31 DOWNTO 0)
	);
END OpenMAC_DPRpackets;

architecture struct of OpenMAC_DPRpackets is
signal address_a_s : std_logic_vector(address_b'range);
signal bea : std_logic_vector(byteena_b'range);
signal q_a_s, q_b_s, data_a_s : std_logic_vector(q_b'range);
signal q_a_s1 : std_logic_vector(q_a'range);
begin
	
	dpr_packet:		entity work.dc_dpr_be
	generic map (
		WIDTH => 32,
		SIZE => memSize_g/4,
		ADDRWIDTH => memSizeLOG2_g-2
	)
	port map (
		clkA => clock_a, 		clkB => clock_b,
		enA => '1',				enB => '1',
		addrA => address_a_s, 	addrB => address_b,
		diA => data_a_s,		diB => data_b,
		doA => q_a_s, 			doB => q_b_s,
		weA => wren_a, 			weB => wren_b,
		beA => bea,				beB => byteena_b
	);
	
	address_a_s <= address_a(address_a'left downto 1);
	
	bea(3) <= byteena_a(1) and address_a(0);
	bea(2) <= byteena_a(0) and address_a(0);
	bea(1) <= byteena_a(1) and not address_a(0);
	bea(0) <= byteena_a(0) and not address_a(0);
	
	data_a_s <= data_a & data_a;
	
	q_a_s1 <=	q_a_s(q_a'length*2-1 downto q_a'length) when address_a(0) = '1' else
				q_a_s(q_a'range);
	
--sync outputs
	process(clock_a)
	begin
		if rising_edge(clock_a) then
			q_a <= q_a_s1;
		end if;
	end process;
	
	process(clock_b)
	begin
		if rising_edge(clock_b) then
			q_b <= q_b_s;
		end if;
	end process;
	
end struct;
