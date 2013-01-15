-------------------------------------------------------------------------------
--! @file tbLutFileBhv.vhd
--
--! @brief Testbench for Look-up table file
--
-------------------------------------------------------------------------------
--
--    (c) B&R, 2012
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
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.global.all;


entity tbLutFile is
end tbLutFile;

architecture Bhv of tbLutFile is

----- Architecture header declarations -----
constant cDataWidth : natural := 32;
constant cLutCounts : natural := 8;
constant cLutInitA : std_logic_vector(cDataWidth-1 downto 0) := x"1234_1234";
constant cLutInitB : std_logic_vector(cDataWidth-1 downto 0) := x"ABCD_ABCD";
constant cLutInit : std_logic_vector(cDataWidth*cLutCounts-1 downto 0) :=
cLutInitA & cLutInitB &cLutInitA & cLutInitB &cLutInitA & cLutInitB &cLutInitA & cLutInitB;


---- Component declarations -----

component clkgen
  generic(
       gPeriod : time := 20 ns
  );
  port (
       iDone : in std_logic;
       oClk : out std_logic
  );
end component;
component lutFile
  generic(
       gLutCount : natural := 4;
       gLutDataWidth : natural := 32;
       gLutInitValues : std_logic_vector := x"1111_1111"&x"2222_2222"&x"3333_3333"&x"4444_4444"
  );
  port (
       iAddrRead : in std_logic_vector(LogDualis(gLutCount)-1 downto 0);
       oData : out std_logic_vector(gLutDataWidth-1 downto 0)
  );
end component;

---- Signal declarations used on the diagram ----

signal clk : std_logic;
signal done : std_logic;
signal valid : std_logic;
signal address : std_logic_vector (LogDualis(cLutCounts)-1 downto 0);
signal data : std_logic_vector (cDataWidth-1 downto 0);

begin

---- User Signal Assignments ----
process(clk)
    variable vCnt : std_logic_vector(LogDualis(cLutCounts) downto 0) := (others => '0');
begin
    if rising_edge(clk) then
        assert (valid = '1') report "LUT value incorrect address=" & integer'IMAGE(to_integer(unsigned(vCnt))) severity error;
        vCnt := std_logic_vector(unsigned(vCnt) + 1);
    end if;
    address <= vCnt(address'range);
    done <= vCnt(vCnt'left);
end process;

valid <= '1' when address(0) = '0' and data = cLutInitA else
            '1' when address(0) = '1' and data = cLutInitB else
            '0';

----  Component instantiations  ----

DUT : lutFile
  generic map (
       gLutCount => cLutCounts,
       gLutDataWidth => cDataWidth,
       gLutInitValues => cLutInit
  )
  port map(
       iAddrRead => address( LogDualis(cLutCounts)-1 downto 0 ),
       oData => data( cDataWidth-1 downto 0 )
  );

U1 : clkgen
  port map(
       iDone => done,
       oClk => clk
  );


end Bhv;
