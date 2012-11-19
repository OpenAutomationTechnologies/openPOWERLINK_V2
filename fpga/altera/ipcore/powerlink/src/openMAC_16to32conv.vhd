-------------------------------------------------------------------------------
--
-- Title       : openMAC_16to32conv
-- Design      : POWERLINK
--
-------------------------------------------------------------------------------
--
-- File        : openMAC_16to32conv.vhd
-- Generated   : Mon Sep 12 15:35:37 2011
-- From        : interface description file
-- By          : Itf2Vhdl ver. 1.22
--
-------------------------------------------------------------------------------
--
--    (c) B&R, 2011
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
-------------------------------------------------------------------------------
--
-- This is a 32-to-16 bit converter which is necessary for e.g. Xilinx PLB.
-- The component has to be connected to openMAC_Ethernet or powerlink.
-- NOT use this directly with openMAC!
--
-------------------------------------------------------------------------------
-- 2011-09-12	V0.01	zelenkaj	Initial creation
-- 2011-10-10	V0.02	zelenkaj	Split bus ack into wr/rd and bug fix
-- 2012-03-21   V0.03   zelenkaj    Added endian generic
-------------------------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_unsigned.ALL;
USE ieee.std_logic_1164.ALL;
USE ieee.std_logic_arith.ALL;

entity openMAC_16to32conv is
	generic(
        gEndian : string := "little";
        bus_address_width : integer := 10
	);
	port(
		clk : in std_logic;
		rst : in std_logic;
		--port from 32bit bus
		bus_select : in std_logic;
		bus_write : in std_logic;
		bus_read : in std_logic;
		bus_byteenable : in std_logic_vector(3 downto 0);
		bus_writedata : in std_logic_vector(31 downto 0);
		bus_readdata : out std_logic_vector(31 downto 0);
		bus_address : in std_logic_vector(bus_address_width-1 downto 0);
		bus_ack_wr : out std_logic;
		bus_ack_rd : out std_logic;
		--port to openMAC_Ethernet
		s_chipselect : out std_logic;
		s_write : out std_logic;
		s_read : out std_logic;
		s_address : out std_logic_vector(bus_address_width-1 downto 0);
		s_byteenable : out std_logic_vector(1 downto 0);
		s_waitrequest : in std_logic;
		s_readdata : in std_logic_vector(15 downto 0);
		s_writedata : out std_logic_vector(15 downto 0)
	);
end openMAC_16to32conv;

architecture rtl of openMAC_16to32conv is
-- types
type fsm_t is (idle, doAccess);
type bus_access_t is (none, dword, word);

-- fsm
signal fsm, fsm_next : fsm_t;
signal bus_access : bus_access_t;

-- cnt
signal cnt, cnt_next, cnt_load_val : std_logic_vector(1 downto 0);
signal cnt_load, cnt_dec, cnt_zero : std_logic;
signal bus_ack : std_logic;

-- word register
signal word_reg, word_reg_next : std_logic_vector(15 downto 0);

begin
	
	process(clk, rst)
	begin
		if rst = '1' then
			cnt <= (others => '0');
			fsm <= idle;
			word_reg <= (others => '0');
		elsif clk = '1' and clk'event then
			cnt <= cnt_next;
			fsm <= fsm_next;
			word_reg <= word_reg_next;
		end if;
	end process;
	
	word_reg_next <= 	s_readdata when bus_access = dword and cnt = 2 and s_waitrequest = '0' else
						word_reg;
	
	s_chipselect <= bus_select; --not cnt_zero;
	s_write <= bus_write and bus_select;
	s_read <= bus_read and bus_select;
	cnt_dec <= (not s_waitrequest) and bus_select;
	
	bus_readdata <= s_readdata & word_reg when bus_access = dword else
					s_readdata & s_readdata;
	
	bus_ack <= 	'1' when cnt = 1 and s_waitrequest = '0' and bus_access = dword else
				'1' when s_waitrequest = '0' and bus_access = word else
				'0';
	
	bus_ack_wr <= bus_ack and bus_write;
	bus_ack_rd <= bus_ack and bus_read;
	
	s_address(bus_address_width-1 downto 1) <= '0' & bus_address(bus_address_width-1 downto 2);
	--word address set to +0 (little) when first dword access or word access with selected word/byte
	s_address(0) <= '0' when bus_access = dword and (cnt = 2 or cnt = 0) and gEndian = "little" else --first word of dword access
                    '1' when bus_access = dword and cnt = 1 and gEndian = "little" else
                    '1' when bus_access = dword and (cnt = 2 or cnt = 0) and gEndian = "big" else
                    '0' when bus_access = dword and cnt = 1 and gEndian = "big" else --first word of dword access
					bus_address(1);
	
	s_byteenable <= "11" when bus_access = dword else
					bus_byteenable(3 downto 2) or bus_byteenable(1 downto 0);
	
	s_writedata <= 	bus_writedata(15 downto 0) when bus_access = dword and (cnt = 2 or cnt = 0) else
					bus_writedata(31 downto 16) when bus_access = dword and cnt = 1 else
					bus_writedata(15 downto 0) when bus_address(1) = '0' else
					bus_writedata(31 downto 16); --when bus_address(1) = '1' else
	
	--fsm
	bus_access <= 	none when bus_select /= '1' else
					dword when bus_byteenable = "1111" else
					word;
	
	fsm_next <= doAccess when fsm = idle and cnt_zero = '1' and bus_access = dword else
				idle when fsm = doAccess and cnt_zero = '1' and bus_access = none else
				fsm;
	
	--if dword, access twice, otherwise (byte, word) access once
	cnt_load_val <= "10" when bus_byteenable = "1111" and bus_read = '1' else "01";
	cnt_load <= '1' when fsm_next = doAccess and fsm = idle else '0';
	
	--counter
	cnt_next <= cnt_load_val when cnt_load = '1' else
				cnt - 1 when cnt_dec = '1' and bus_access = dword else
				cnt;
	
	cnt_zero <= '1' when cnt = 0 else '0';

end rtl;
