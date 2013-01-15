-------------------------------------------------------------------------------
--! @file tbMagicBridgeBhv.vhd
--
--! @brief
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
use work.hostInterfacePkg.all;

entity tbMagicBridge is
end tbMagicBridge;

architecture Bhv of tbMagicBridge is

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
component magicBridge
  generic(
       gAddressSpaceCount : natural := 2;
       gBaseAddressArray : tarraystd32 := x"0000_1000"&x"0000_2000"&x"0000_3000";
       gBridgeInAddressWidth : natural := 16;
       gBridgeOutAddressWidth : natural := 32
  );
  port (
       iBaseSetAddress : in std_logic_vector(LogDualis(gAddressSpaceCount)-1 downto 0);
       iBaseSetData : in std_logic_vector(gBridgeOutAddressWidth-1 downto 0);
       iBaseSetWrite : in std_logic;
       iBridgeAddress : in std_logic_vector(gBridgeInAddressWidth-1 downto 0);
       iBridgeSelect : in std_logic;
       iClk : in std_logic;
       iRst : in std_logic;
       oBaseSetData : out std_logic_vector(gBridgeOutAddressWidth-1 downto 0);
       oBridgeAddress : out std_logic_vector(gBridgeOutAddressWidth-1 downto 0);
       oBridgeSelect : out std_logic_vector(gAddressSpaceCount-1 downto 0);
       oBridgeSelectAny : out std_logic
  );
end component;

---- Architecture declarations -----
constant cInAddressWidth : natural := 16;
constant cOutAddressWidth : natural := 30;

constant cAddressArray : tArrayStd32 :=
x"0000_1000" & x"0000_2000" & x"0000_3000" & x"0000_3100";

constant cAddressSpaceCount : natural := 3;
constant cAddressSpaceCount_log2 : natural := LogDualis(cAddressSpaceCount);


----     Constants     -----
constant VCC_CONSTANT   : std_logic := '1';
constant GND_CONSTANT   : std_logic := '0';

---- Signal declarations used on the diagram ----

signal baseSetWrite : std_logic;
signal clk : std_logic;
signal done : std_logic;
signal GND : std_logic;
signal inputSelect : std_logic;
signal outputSelectAny : std_logic;
signal rst : std_logic;
signal VCC : std_logic;
signal baseSetAddress : std_logic_vector (cAddressSpaceCount_log2-1 downto 0);
signal baseSetData : std_logic_vector (cOutAddressWidth-1 downto 0);
signal inputAddress : std_logic_vector (cInAddressWidth-1 downto 0);
signal outputAddress : std_logic_vector (cOutAddressWidth-1 downto 0);
signal outputSelect : std_logic_vector (cAddressSpaceCount-1 downto 0);
signal readAddress : std_logic_vector (31 downto 0) := (others => '0');

begin

---- User Signal Assignments ----
--set base sets

process(clk)
variable vAddr : natural;
begin
    if rising_edge(clk) then
        if rst = cActivated then
            baseSetWrite <= cInactivated;
            baseSetAddress <= (others => '0');
            baseSetData <= std_logic_vector(to_unsigned(16#2000_0000#, baseSetData'length));
            vAddr := 0;
        else
            baseSetWrite <= cInactivated;
            baseSetAddress <= (others => '0');
            baseSetData <= (others => '0');

            if vAddr = cAddressSpaceCount then
                --stop here...
            else
                baseSetWrite <= cActivated;
                baseSetAddress <= std_logic_vector(to_unsigned(vAddr, baseSetAddress'length));
                baseSetData <= std_logic_vector(unsigned(baseSetData) + to_unsigned(16#0100_0000#, baseSetData'length));
                vAddr := vAddr + 1;
            end if;
        end if;
    end if;

end process;

----  Component instantiations  ----

DUT : magicBridge
  generic map (
       gAddressSpaceCount => cAddressSpaceCount,
       gBaseAddressArray => cAddressArray,
       gBridgeInAddressWidth => cInAddressWidth,
       gBridgeOutAddressWidth => cOutAddressWidth
  )
  port map(
       iBaseSetAddress => baseSetAddress( cAddressSpaceCount_log2-1 downto 0 ),
       iBaseSetData => baseSetData( cOutAddressWidth-1 downto 0 ),
       iBaseSetWrite => baseSetWrite,
       iBridgeAddress => inputAddress( cInAddressWidth-1 downto 0 ),
       iBridgeSelect => inputSelect,
       iClk => clk,
       iRst => rst,
       oBaseSetData => readAddress( cOutAddressWidth-1 downto 0 ),
       oBridgeAddress => outputAddress( cOutAddressWidth-1 downto 0 ),
       oBridgeSelect => outputSelect( cAddressSpaceCount-1 downto 0 ),
       oBridgeSelectAny => outputSelectAny
  );

U1 : clkgen
  port map(
       iDone => done,
       oClk => clk
  );

U2 : enableGen
  port map(
       iReset => GND,
       onEnable => rst
  );

U3 : busMaster
  generic map (
       gAddrWidth => cInAddressWidth,
       gDataWidth => 32,
       gStimuliFile => "HOST_INTERFACE/tb/tbMagicBridge_stim.txt"
  )
  port map(
       iAck => VCC,
       iClk => clk,
       iEnable => VCC,
       iReaddata => readAddress( 31 downto 0 ),
       iRst => rst,
       oAddress => inputAddress( cInAddressWidth-1 downto 0 ),
       oDone => done,
       oSelect => inputSelect
  );


---- Power , ground assignment ----

GND <= GND_CONSTANT;
VCC <= VCC_CONSTANT;

end Bhv;
