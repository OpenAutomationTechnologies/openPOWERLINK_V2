------------------------------------------------------------------------------------------------------------------------
-- OpenMAC Shadow RAM (Xilinx)
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
-- 2009-08-07  V0.00        First generation.
------------------------------------------------------------------------------------------------------------------------

LIBRARY ieee;                   
USE ieee.std_logic_1164.all;    
USE ieee.std_logic_arith.all;   
USE ieee.std_logic_unsigned.all;

ENTITY shadowram IS
  PORT (
  clk : in std_logic;
  addr : in std_logic_vector;
  be : in std_logic_vector(1 downto 0);
  indata : in std_logic_vector;
  outdata : out std_logic_vector;
  wr, sel : in std_logic
  );
END ENTITY shadowram;

ARCHITECTURE struct OF shadowram IS
  type ram_type is array ((2**addr'length)-1 downto 0) of std_logic_vector(indata'range);
  signal shd_ram : ram_type;
  signal rd_addr : std_logic_vector(addr'range);
  signal s_indata : std_logic_vector(indata'range);
BEGIN
	
	process(be)
	begin
		if be = "01" then
			s_indata <= shd_ram(conv_integer(addr))(indata'left downto indata'left/2+1) & indata(indata'left/2 downto 0);
		elsif be = "10" then
			s_indata <= indata(indata'left downto indata'left/2+1) & shd_ram(conv_integer(addr))(indata'left/2 downto 0);
		else
			s_indata <= indata;
		end if;
	end process;
	
	process(clk)
	begin
		if clk = '1' and clk'event then
			if wr = '1' and sel = '1' then
				shd_ram(conv_integer(addr)) <= s_indata;
			end if;
		end if;
	end process;
	
	outdata <= shd_ram(conv_integer(addr));
	
END ARCHITECTURE struct;

