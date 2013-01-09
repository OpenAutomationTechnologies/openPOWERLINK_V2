-------------------------------------------------------------------------------
-- Entity : No Title
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
library IEEE;
use IEEE.std_logic_1164.all;

library work;
use global.all;

entity tbOpenMAC_16to32conv is 
end tbOpenMAC_16to32conv;

architecture bhv of tbOpenMAC_16to32conv is

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
component Dpr_16_16
  generic(
       Simulate : boolean
  );
  port (
       AddrA : in std_logic_vector(7 downto 0);
       AddrB : in std_logic_vector(7 downto 0);
       BeA : in std_logic_vector(1 downto 0) := "11";
       BeB : in std_logic_vector(1 downto 0) := "11";
       ClkA : in std_logic;
       ClkB : in std_logic;
       DiA : in std_logic_vector(15 downto 0) := (others => '0');
       DiB : in std_logic_vector(15 downto 0) := (others => '0');
       EnA : in std_logic := '1';
       EnB : in std_logic := '1';
       WeA : in std_logic := '0';
       WeB : in std_logic := '0';
       DoA : out std_logic_vector(15 downto 0);
       DoB : out std_logic_vector(15 downto 0)
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
component openMAC_16to32conv
  generic(
       bus_address_width : integer := 10
  );
  port (
       bus_address : in std_logic_vector(bus_address_width-1 downto 0);
       bus_byteenable : in std_logic_vector(3 downto 0);
       bus_read : in std_logic;
       bus_select : in std_logic;
       bus_write : in std_logic;
       bus_writedata : in std_logic_vector(31 downto 0);
       clk : in std_logic;
       rst : in std_logic;
       s_readdata : in std_logic_vector(15 downto 0);
       s_waitrequest : in std_logic;
       bus_ack_rd : out std_logic;
       bus_ack_wr : out std_logic;
       bus_readdata : out std_logic_vector(31 downto 0);
       s_address : out std_logic_vector(bus_address_width-1 downto 0);
       s_byteenable : out std_logic_vector(1 downto 0);
       s_chipselect : out std_logic;
       s_read : out std_logic;
       s_write : out std_logic;
       s_writedata : out std_logic_vector(15 downto 0)
  );
end component;
component req_ack
  generic(
       ack_delay_g : integer := 1;
       zero_delay_g : boolean := false
  );
  port (
       clk : in std_logic;
       enable : in std_logic;
       rst : in std_logic;
       ack : out std_logic
  );
end component;

---- Architecture declarations -----
-- Click here to add additional declarations --
constant cAddrwidth : integer := 32;
constant cDatawidth : integer := 32;


----     Constants     -----
constant DANGLING_INPUT_CONSTANT : std_logic := 'Z';
constant GND_CONSTANT   : std_logic := '0';

---- Signal declarations used on the diagram ----

signal ack : std_logic;
signal busMasterDone : std_logic;
signal clk50 : std_logic;
signal done : std_logic := '1';
signal enable : std_logic;
signal GND : std_logic;
signal NET1416 : std_logic;
signal NET1425 : std_logic;
signal NET928 : std_logic;
signal NET932 : std_logic;
signal read : std_logic;
signal reset : std_logic;
signal sel : std_logic;
signal s_chipselect : std_logic;
signal s_read : std_logic;
signal s_waitrequest : std_logic;
signal s_write : std_logic;
signal write : std_logic;
signal address : std_logic_vector (cAddrwidth-1 downto 0);
signal byteenable : std_logic_vector (cDatawidth/8-1 downto 0);
signal readdata : std_logic_vector (cDatawidth-1 downto 0);
signal s_address : std_logic_vector (cAddrwidth-1 downto 0);
signal s_byteenable : std_logic_vector (1 downto 0);
signal s_readdata : std_logic_vector (15 downto 0);
signal s_writedata : std_logic_vector (15 downto 0);
signal writedata : std_logic_vector (cDatawidth-1 downto 0);

---- Declaration for Dangling input ----
signal Dangling_Input_Signal : STD_LOGIC;

begin

---- User Signal Assignments ----
--generate done signal
done <= busMasterDone;

----  Component instantiations  ----

DUT : openMAC_16to32conv
  generic map (
       bus_address_width => cAddrwidth
  )
  port map(
       bus_byteenable(0) => byteenable(0),
       bus_byteenable(1) => byteenable(1),
       bus_byteenable(2) => byteenable(2),
       bus_byteenable(3) => byteenable(3),
       bus_ack_rd => NET928,
       bus_ack_wr => NET932,
       bus_address => address( cAddrwidth-1 downto 0 ),
       bus_read => read,
       bus_readdata => readdata( cDatawidth-1 downto 0 ),
       bus_select => sel,
       bus_write => write,
       bus_writedata => writedata( cDatawidth-1 downto 0 ),
       clk => clk50,
       rst => reset,
       s_address => s_address( cAddrwidth-1 downto 0 ),
       s_byteenable => s_byteenable,
       s_chipselect => s_chipselect,
       s_read => s_read,
       s_readdata => s_readdata,
       s_waitrequest => s_waitrequest,
       s_write => s_write,
       s_writedata => s_writedata
  );

U1 : clkgen
  generic map (
       gPeriod => 20 ns
  )
  port map(
       iDone => done,
       oClk => clk50
  );

U2 : enableGen
  generic map (
       gEnableDelay => 50 ns
  )
  port map(
       iReset => GND,
       onEnable => reset
  );

U3 : enableGen
  generic map (
       gEnableDelay => 50 ns
  )
  port map(
       iReset => reset,
       oEnable => enable
  );

U4 : busMaster
  generic map (
       gAddrWidth => cAddrwidth,
       gDataWidth => cDatawidth,
       gStimuliFile => "openMAC/tb/tbOpenMAC_16to32conv_stim.txt"
  )
  port map(
       iAck => ack,
       iClk => clk50,
       iEnable => enable,
       iReaddata => readdata( cDatawidth-1 downto 0 ),
       iRst => reset,
       oAddress => address( cAddrwidth-1 downto 0 ),
       oByteenable => byteenable( cDatawidth/8-1 downto 0 ),
       oDone => busMasterDone,
       oRead => read,
       oSelect => sel,
       oWrite => write,
       oWritedata => writedata( cDatawidth-1 downto 0 )
  );

U5 : req_ack
  generic map (
       ack_delay_g => 1,
       zero_delay_g => true
  )
  port map(
       ack => NET1425,
       clk => clk50,
       enable => s_write,
       rst => reset
  );

U6 : req_ack
  generic map (
       ack_delay_g => 1,
       zero_delay_g => false
  )
  port map(
       ack => NET1416,
       clk => clk50,
       enable => s_read,
       rst => reset
  );

ack <= NET928 or NET932;

s_waitrequest <= not(NET1416 or NET1425);

U9 : Dpr_16_16
  generic map (
       Simulate => false
  )
  port map(
       AddrA(0) => s_address(0),
       AddrA(1) => s_address(1),
       AddrA(2) => s_address(2),
       AddrA(3) => s_address(3),
       AddrA(4) => s_address(4),
       AddrA(5) => s_address(5),
       AddrA(6) => s_address(6),
       AddrA(7) => s_address(7),
       AddrB(0) => Dangling_Input_Signal,
       AddrB(1) => Dangling_Input_Signal,
       AddrB(2) => Dangling_Input_Signal,
       AddrB(3) => Dangling_Input_Signal,
       AddrB(4) => Dangling_Input_Signal,
       AddrB(5) => Dangling_Input_Signal,
       AddrB(6) => Dangling_Input_Signal,
       AddrB(7) => Dangling_Input_Signal,
       BeA => s_byteenable,
       ClkA => clk50,
       ClkB => clk50,
       DiA => s_writedata,
       DoA => s_readdata,
       EnA => s_chipselect,
       EnB => GND,
       WeA => s_write,
       WeB => GND
  );


---- Power , ground assignment ----

GND <= GND_CONSTANT;

---- Dangling input signal assignment ----

Dangling_Input_Signal <= DANGLING_INPUT_CONSTANT;

end bhv;
