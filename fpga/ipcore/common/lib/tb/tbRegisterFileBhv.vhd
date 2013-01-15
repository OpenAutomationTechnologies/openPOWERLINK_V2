-------------------------------------------------------------------------------
--! @file tbRegisterFileBhv.vhd
--
--! @brief Testbench for Register file
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


entity tbRegisterFile is
end tbRegisterFile;

architecture Bhv of tbRegisterFile is

---- Component declarations -----

component busMaster
  generic(
       gAddrWidth : integer := 32;
       gDataWidth : integer := 32;
       gStimuliFile : string := "name_TB_stim.txt"
  );
  port (
       iAck : in std_logic;
       iClk : in std_logic;
       iEnable : in std_logic;
       iReaddata : in std_logic_vector(gDataWidth-1 downto 0);
       iRst : in std_logic;
       oAddress : out std_logic_vector(gAddrWidth-1 downto 0);
       oByteenable : out std_logic_vector(gDataWidth/8-1 downto 0);
       oDone : out std_logic;
       oRead : out std_logic;
       oSelect : out std_logic;
       oWrite : out std_logic;
       oWritedata : out std_logic_vector(gDataWidth-1 downto 0)
  );
end component;
component clkgen
  generic(
       gPeriod : time := 20 ns
  );
  port (
       iDone : in std_logic;
       oClk : out std_logic
  );
end component;
component enableGen
  generic(
       gEnableDelay : time := 100 ns
  );
  port (
       iReset : in std_logic;
       oEnable : out std_logic;
       onEnable : out std_logic
  );
end component;
component registerFile
  generic(
       gRegCount : natural := 8;
       gRegDataWidth : natural := 32
  );
  port (
       iAddrA : in std_logic_vector(LogDualis(gRegCount)-1 downto 0);
       iAddrB : in std_logic_vector(LogDualis(gRegCount)-1 downto 0);
       iClk : in std_logic;
       iRst : in std_logic;
       iWriteA : in std_logic;
       iWriteB : in std_logic;
       iWritedataA : in std_logic_vector(gRegDataWidth-1 downto 0);
       iWritedataB : in std_logic_vector(gRegDataWidth-1 downto 0);
       oReaddataA : out std_logic_vector(gRegDataWidth-1 downto 0);
       oReaddataB : out std_logic_vector(gRegDataWidth-1 downto 0)
  );
end component;

----     Constants     -----
constant VCC_CONSTANT   : std_logic := '1';
constant GND_CONSTANT   : std_logic := '0';

---- Signal declarations used on the diagram ----

signal clk : std_logic;
signal done : std_logic;
signal GND : std_logic;
signal rst : std_logic;
signal VCC : std_logic;
signal wr : std_logic;
signal addr : std_logic_vector (2 downto 0);
signal readData : std_logic_vector (31 downto 0);
signal writeData : std_logic_vector (31 downto 0);

begin

----  Component instantiations  ----

DUT : registerFile
  port map(
       iAddrA => addr( 2 downto 0 ),
       iAddrB => addr( 2 downto 0 ),
       iClk => clk,
       iRst => rst,
       iWriteA => wr,
       iWriteB => GND,
       iWritedataA => writeData( 31 downto 0 ),
       iWritedataB => writeData( 31 downto 0 ),
       oReaddataB => readData( 31 downto 0 )
  );

U1 : enableGen
  generic map (
       gEnableDelay => 100 ns
  )
  port map(
       iReset => GND,
       onEnable => rst
  );

U2 : clkgen
  generic map (
       gPeriod => 20 ns
  )
  port map(
       iDone => done,
       oClk => clk
  );

U3 : busMaster
  generic map (
       gAddrWidth => 3,
       gDataWidth => 32,
       gStimuliFile => "lib/tb/tbRegisterFile_stim.txt"
  )
  port map(
       iAck => VCC,
       iClk => clk,
       iEnable => VCC,
       iReaddata => readData( 31 downto 0 ),
       iRst => rst,
       oAddress => addr( 2 downto 0 ),
       oDone => done,
       oWrite => wr,
       oWritedata => writeData( 31 downto 0 )
  );


---- Power , ground assignment ----

GND <= GND_CONSTANT;
VCC <= VCC_CONSTANT;

end Bhv;
