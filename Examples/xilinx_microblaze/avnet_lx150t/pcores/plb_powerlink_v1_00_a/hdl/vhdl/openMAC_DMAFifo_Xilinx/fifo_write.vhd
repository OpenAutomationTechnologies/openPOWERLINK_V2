------------------------------------------------------------------------------------------------------------------------
-- write controller of the fifo
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
-- Note: A general implementation of a asynchronous fifo which is
--			using a dual port ram. This file is the write controler.
--
------------------------------------------------------------------------------------------------------------------------
-- Version History
------------------------------------------------------------------------------------------------------------------------
-- 2011-09-22	V0.01		mairt		first version
-- 2011-10-14	V0.02		zelenkaj	element calculation buggy
------------------------------------------------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all; 

entity fifo_write_ctrl is
   generic(N: natural:=4);
   port(
      clkw, resetw: in std_logic;
      wr: in std_logic;
      r_ptr_in: in std_logic_vector(N downto 0);
      w_full: out std_logic; 
	  w_empty: out std_logic;
      w_ptr_out: out std_logic_vector(N downto 0);
      w_addr: out std_logic_vector(N-1 downto 0);
	  w_elements: out std_logic_vector(N-1 downto 0)
   );
end fifo_write_ctrl;

architecture gray_arch of fifo_write_ctrl is
   signal w_ptr_reg, w_ptr_next: std_logic_vector(N downto 0);
   signal r_ptr_reg, r_ptr_next : std_logic_vector(N downto 0) := (others => '0');
   signal gray1, bin, bin1: std_logic_vector(N downto 0);
   signal waddr_all: std_logic_vector(N-1 downto 0);
   signal waddr_msb, raddr_msb: std_logic;
   signal full_flag, empty_flag: std_logic;
   signal w_elements_wr, w_elements_rd, w_elements_diff : std_logic_vector(N downto 0);
   signal w_elements_reg, w_elements_next : std_logic_vector(N-1 downto 0);
begin
   -- register
   process(clkw,resetw)
   begin
      if (resetw='1') then
          w_ptr_reg <= (others=>'0');
		  --r_ptr_reg <= (others => '0');
		  w_elements_reg <= (others => '0');
      elsif (clkw'event and clkw='1') then
         w_ptr_reg <= w_ptr_next;
		 --r_ptr_reg <= r_ptr_next;
		 w_elements_reg <= w_elements_next;
      end if;
   end process;
   -- (N+1)-bit Gray counter
   bin <= w_ptr_reg xor ('0' & bin(N downto 1));
   bin1 <= std_logic_vector(unsigned(bin) + 1);
   gray1 <= bin1 xor ('0' & bin1(N downto 1));
   -- update write pointer
   w_ptr_next <= gray1 when wr='1' and full_flag='0' else
	   w_ptr_reg;  
   -- save read pointer
   r_ptr_next <= r_ptr_in;
   -- N-bit Gray counter
   waddr_msb <=  w_ptr_reg(N) xor w_ptr_reg(N-1);
   waddr_all <= waddr_msb & w_ptr_reg(N-2 downto 0);
   
   -- check for FIFO full and empty
   raddr_msb <= r_ptr_in(N) xor r_ptr_in(N-1);
   full_flag <=
     '1' when r_ptr_in(N) /=w_ptr_reg(N) and
         r_ptr_in(N-2 downto 0)=w_ptr_reg(N-2 downto 0) and
         raddr_msb = waddr_msb else
     '0';				
   empty_flag <=
     '1' when r_ptr_in(N) =w_ptr_reg(N) and
         r_ptr_in(N-2 downto 0)=w_ptr_reg(N-2 downto 0) and
         raddr_msb = waddr_msb else
     '0';
	 
   -- convert gray value to bin and obtain difference
   w_elements_wr <= bin;
   w_elements_rd <= r_ptr_in xor ('0' & w_elements_rd(N downto 1));
   w_elements_diff <= std_logic_vector(unsigned(w_elements_wr) - unsigned(w_elements_rd));
   w_elements_next <= w_elements_diff(w_elements_next'range);  
   
   -- output
   w_addr <= waddr_all;
   w_ptr_out <= w_ptr_reg;
   w_elements <= w_elements_reg;
   w_full <= full_flag;
   w_empty <= empty_flag; 
   
end gray_arch;