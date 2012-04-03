-------------------------------------------------------------------------------
--
-- Title       : openMAC_cmp
-- Design      : plk_mn
--
-------------------------------------------------------------------------------
--
-- File        : OpenMAC_cmp.vhd
-- Generated   : Wed Jul 27 10:52:27 2011
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
-- 2011-07-26  	V0.01	zelenkaj    First version						 				  
-- 2012-01-11	V0.02	mairt		moved registers to seperate cmp int and tog int
--
-------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;

entity openMAC_cmp is
	generic(
		mac_time_width_g : integer := 32;
		gen2ndCmpTimer_g : boolean := false
	);
	port(
		clk : in std_logic;
		rst : in std_logic;
		wr : in std_logic;
		addr : in std_logic_vector(1 downto 0);
		din : in std_logic_vector(31 downto 0);
		dout : out std_logic_vector(31 downto 0);
		mac_time : in std_logic_vector(mac_time_width_g-1 downto 0);
		irq : out std_logic;
		toggle : out std_logic
	);
end openMAC_cmp;

architecture rtl of openMAC_cmp is
signal cmp_enable, tog_enable : std_logic;
signal cmp_value, tog_value : std_logic_vector(mac_time'range);
signal irq_s, toggle_s : std_logic;
begin
	irq <= irq_s;
	toggle <= toggle_s;
	
	process(clk, rst)
	begin
		if rst = '1' then
			cmp_enable <= '0'; cmp_value <= (others => '0'); irq_s <= '0';
			
			if gen2ndCmpTimer_g = TRUE then
				tog_enable <= '0'; tog_value <= (others => '0'); toggle_s <= '0';
			end if;
			
		elsif clk = '1' and clk'event then
			
			--cmp
			if cmp_enable = '1' and mac_time = cmp_value then
				irq_s <= '1';
			end if;
						
			--tog
			if tog_enable = '1' and mac_time = tog_value and gen2ndCmpTimer_g = TRUE then
				toggle_s <= not toggle_s;
			end if;
			
			--memory mapping
			if wr = '1' then
				case addr is
					when "00" =>
						cmp_value <= din;
						irq_s <= '0';
					when "01" =>
						cmp_enable <= din(0);
					when "10" =>
						if gen2ndCmpTimer_g = TRUE then
							tog_value <= din;
						end if;	   
					when "11" =>
						if gen2ndCmpTimer_g = TRUE then
							tog_enable <= din(0);
						end if;						
					when others =>
						--go and get a coffee...
				end case;
			end if;
			
		end if;
	end process;
	
	dout <= 
		mac_time 																when addr = "00" else
		x"000000" & "00" & "00" & "00" & irq_s & cmp_enable 					when addr = "01" else
		tog_value 																when addr = "10" and gen2ndCmpTimer_g = TRUE else	  
		x"000000" & "00" & "00" & "00" & toggle_s & tog_enable					when addr = "11" and gen2ndCmpTimer_g = TRUE else			
		mac_time; --otherwise give me the current time...
	
end rtl;
