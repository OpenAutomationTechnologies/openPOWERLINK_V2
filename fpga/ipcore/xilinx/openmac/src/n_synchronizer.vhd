-------------------------------------------------------------------------------
-- n sychronizer of the async fifo
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
--            using a dual port ram. This file is the n sychronizer.
--
-------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
entity synchronizer_g is
   generic(N: natural);
   port(
      clk, reset: in std_logic;
      in_async: in std_logic_vector(N-1 downto 0);
      out_sync: out std_logic_vector(N-1 downto 0)
   );
end synchronizer_g;

architecture two_ff_arch of synchronizer_g is
   signal meta_reg, sync_reg, sync_reg1 : std_logic_vector(N-1 downto 0) := (others => '0');
   signal meta_next, sync_next, sync_next1 : std_logic_vector(N-1 downto 0) := (others => '0');
begin
   -- two registers
   process(clk)--,reset)
   begin
--      if (reset='1') then
--         meta_reg <= (others=>'0');
--         sync_reg <= (others=>'0');
--         sync_reg1 <= (others => '0');
      if (clk'event and clk='1') then
         meta_reg <= meta_next;
         sync_reg <= sync_next;
         sync_reg1 <= sync_next1;
      end if;
   end process;
   -- next-state logic
   meta_next <= in_async;
   sync_next <= meta_reg;
   sync_next1 <= sync_reg;
   -- output
   out_sync <= sync_reg1;
end two_ff_arch;