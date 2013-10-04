-------------------------------------------------------------------------------
-- read controller of the fifo
--
--       Copyright (C) 2009 B&R
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
-- using a dual port ram. This file is the read controler.
--
-------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
entity fifo_read_ctrl is
   generic(N: natural:=4);
   port(
      clkr, resetr: in std_logic;
      w_ptr_in: in std_logic_vector(N downto 0);
      rd: in std_logic;
      r_empty: out std_logic;
      r_full: out std_logic;
      r_ptr_out: out std_logic_vector(N downto 0);
      r_addr: out std_logic_vector(N-1 downto 0);
      r_elements: out std_logic_vector(N-1 downto 0)
   );
end fifo_read_ctrl;

architecture gray_arch of fifo_read_ctrl is
   signal r_ptr_reg, r_ptr_next: std_logic_vector(N downto 0);
   signal w_ptr_reg, w_ptr_next : std_logic_vector(N downto 0) := (others => '0');
   signal gray1, bin, bin1: std_logic_vector(N downto 0);
   signal raddr_all: std_logic_vector(N-1 downto 0);
   signal raddr_msb,waddr_msb: std_logic;
   signal empty_flag, full_flag: std_logic;
   signal r_elements_wr, r_elements_rd, r_elements_diff : std_logic_vector(N downto 0);
   signal r_elements_reg, r_elements_next : std_logic_vector(N-1 downto 0);
begin
   -- register
   process(clkr,resetr)
   begin
      if (resetr='1') then
         r_ptr_reg <= (others=>'0');
         --w_ptr_reg <= (others => '0');
         r_elements_reg <= (others => '0');
      elsif (clkr'event and clkr='1') then
         r_ptr_reg <= r_ptr_next;
         --w_ptr_reg <= w_ptr_next;
         r_elements_reg <= r_elements_next;
      end if;
   end process;
   -- (N+1)-bit Gray counter
   bin <= r_ptr_reg xor ('0' & bin(N downto 1));
   bin1 <= std_logic_vector(unsigned(bin) + 1);
   gray1 <= bin1 xor ('0' & bin1(N downto 1));
   -- update read pointer
   r_ptr_next <= gray1 when rd='1' and empty_flag='0' else
       r_ptr_reg;
   -- save write pointer
   w_ptr_next <= w_ptr_in;
   -- N-bit Gray counter
   raddr_msb <= r_ptr_reg(N) xor r_ptr_reg(N-1);
   raddr_all <= raddr_msb & r_ptr_reg(N-2 downto 0);
   waddr_msb <= w_ptr_in(N) xor w_ptr_in(N-1);
   -- check for FIFO read empty
   empty_flag <=
      '1' when w_ptr_in(N)=r_ptr_reg(N) and
          w_ptr_in(N-2 downto 0)=r_ptr_reg(N-2 downto 0) and
          raddr_msb = waddr_msb else
      '0';
    -- check for FIFO read full
    full_flag <=
      '1' when w_ptr_in(N)/=r_ptr_reg(N) and
          w_ptr_in(N-2 downto 0)=r_ptr_reg(N-2 downto 0) and
          raddr_msb = waddr_msb else
      '0';

       -- convert gray value to bin and obtain difference
    r_elements_wr <= bin;
    r_elements_rd <= w_ptr_in xor ('0' & r_elements_rd(N downto 1));
    r_elements_diff <= std_logic_vector(unsigned(r_elements_rd) - unsigned(r_elements_wr));
    r_elements_next <= r_elements_diff(r_elements_next'range);

    -- output
    r_addr <= raddr_all;
    r_ptr_out <= r_ptr_reg;
    r_elements <= r_elements_reg;
    r_empty <= empty_flag;
    r_full <= full_flag;
end gray_arch;