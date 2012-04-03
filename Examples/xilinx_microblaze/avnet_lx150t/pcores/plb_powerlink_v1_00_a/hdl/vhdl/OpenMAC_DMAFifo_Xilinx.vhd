-------------------------------------------------------------------------------
--
-- Title       : OpenMAC_DMAFifo_Xilinx
-- Design      : POWERLINK
--
-------------------------------------------------------------------------------
--
-- File        : C:\git\VHDL_IP-Cores\active_hdl\compile\OpenMAC_DMAFifo_Xilinx.vhd
-- Generated   : Thu Nov 24 15:08:50 2011
-- From        : C:\git\VHDL_IP-Cores\active_hdl\src\OpenMAC_DMAFifo_Xilinx.bde
-- By          : Bde2Vhdl ver. 2.6
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
-- Design unit header --
--
-- This is the toplevel file of the dual clocked DMA FIFO
-- for Xilinx FPGAs.
--
-------------------------------------------------------------------------------
--
-- 2011-10-13	V0.01	mairt		First version
--
-------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;

entity openMAC_DMAfifo is
  generic(
       fifo_data_width_g : NATURAL := 16;
       fifo_word_size_g : NATURAL := 32;
       fifo_word_size_log2_g : NATURAL := 5
  );
  port(
       aclr : in std_logic;
       rd_clk : in std_logic;
       rd_req : in std_logic;
       wr_clk : in std_logic;
       wr_req : in std_logic;
       wr_data : in std_logic_vector(fifo_data_width_g - 1 downto 0);
       rd_empty : out std_logic;
       rd_full : out std_logic;
       wr_empty : out std_logic;
       wr_full : out std_logic;
       rd_data : out std_logic_vector(fifo_data_width_g - 1 downto 0);
       rd_usedw : out std_logic_vector(fifo_word_size_log2_g - 1 downto 0);
       wr_usedw : out std_logic_vector(fifo_word_size_log2_g - 1 downto 0)
  );
end openMAC_DMAfifo;

architecture struct of openMAC_DMAfifo is

---- Component declarations -----

component async_fifo_ctrl
  generic(
       ADDR_WIDTH : natural := 5
  );
  port (
       clkr : in std_logic;
       clkw : in std_logic;
       rd : in std_logic;
       resetr : in std_logic;
       resetw : in std_logic;
       wr : in std_logic;
       r_addr : out std_logic_vector(ADDR_WIDTH-1 downto 0);
       r_empty : out std_logic;
       r_full : out std_logic;
       rd_used_w : out std_logic_vector(ADDR_WIDTH-1 downto 0);
       w_addr : out std_logic_vector(ADDR_WIDTH-1 downto 0);
       w_empty : out std_logic;
       w_full : out std_logic;
       wd_used_w : out std_logic_vector(ADDR_WIDTH-1 downto 0)
  );
end component;
component dc_dpr
  generic(
       ADDRWIDTH : integer := 7;
       SIZE : integer := 128;
       WIDTH : integer := 16
  );
  port (
       addrA : in std_logic_vector(ADDRWIDTH-1 downto 0);
       addrB : in std_logic_vector(ADDRWIDTH-1 downto 0);
       clkA : in std_logic;
       clkB : in std_logic;
       diA : in std_logic_vector(WIDTH-1 downto 0);
       diB : in std_logic_vector(WIDTH-1 downto 0);
       enA : in std_logic;
       enB : in std_logic;
       weA : in std_logic;
       weB : in std_logic;
       doA : out std_logic_vector(WIDTH-1 downto 0);
       doB : out std_logic_vector(WIDTH-1 downto 0)
  );
end component;

---- Signal declarations used on the diagram ----

signal enA : std_logic;
signal enB : std_logic;
signal wea : std_logic;
signal weB : std_logic;
signal wr_full_s : std_logic;
signal diB : std_logic_vector (fifo_data_width_g-1 downto 0);
signal rd_addr : std_logic_vector (fifo_word_size_log2_g-1 downto 0);
signal wr_addr : std_logic_vector (fifo_word_size_log2_g-1 downto 0);

begin

---- User Signal Assignments ----
--assignments
---port a writes only
enA <= wea;
---port b reads only
enB <= rd_req;
weB <= '0';
diB <= (others => '0');

----  Component instantiations  ----

THE_FIFO_CONTROL : async_fifo_ctrl
  generic map (
       ADDR_WIDTH => fifo_word_size_log2_g
  )
  port map(
       clkr => rd_clk,
       clkw => wr_clk,
       r_addr => rd_addr( fifo_word_size_log2_g-1 downto 0 ),
       r_empty => rd_empty,
       r_full => rd_full,
       rd => rd_req,
       rd_used_w => rd_usedw( fifo_word_size_log2_g - 1 downto 0 ),
       resetr => aclr,
       resetw => aclr,
       w_addr => wr_addr( fifo_word_size_log2_g-1 downto 0 ),
       w_empty => wr_empty,
       w_full => wr_full_s,
       wd_used_w => wr_usedw( fifo_word_size_log2_g - 1 downto 0 ),
       wr => wr_req
  );

THE_FIFO_DPR : dc_dpr
  generic map (
       ADDRWIDTH => fifo_word_size_log2_g,
       SIZE => fifo_word_size_g,
       WIDTH => fifo_data_width_g
  )
  port map(
       addrA => wr_addr( fifo_word_size_log2_g-1 downto 0 ),
       addrB => rd_addr( fifo_word_size_log2_g-1 downto 0 ),
       clkA => wr_clk,
       clkB => rd_clk,
       diA => wr_data( fifo_data_width_g - 1 downto 0 ),
       diB => diB( fifo_data_width_g-1 downto 0 ),
       doB => rd_data( fifo_data_width_g - 1 downto 0 ),
       enA => enA,
       enB => enB,
       weA => wea,
       weB => weB
  );

wea <= not(wr_full_s) and wr_req;


---- Terminal assignment ----

    -- Output\buffer terminals
	wr_full <= wr_full_s;


end struct;
