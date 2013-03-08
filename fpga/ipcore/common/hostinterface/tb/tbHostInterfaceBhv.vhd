-------------------------------------------------------------------------------
--! @file tbHostInterfaceBhv.vhd
--
--! @brief Testbench for Hostinterface ipcore
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


entity tbHostInterface is
end tbHostInterface;

architecture Bhv of tbHostInterface is

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
component hostInterface
  generic(
       gBaseDynBuf0 : natural := 2048;
       gBaseDynBuf1 : natural := 4096;
       gBaseErrCntr : natural := 6144;
       gBaseK2UQ : natural := 28672;
       gBaseRes : natural := 81920;
       gBaseRpdo : natural := 57344;
       gBaseRxVetQ : natural := 26624;
       gBaseTpdo : natural := 45056;
       gBaseTxGenQ : natural := 14336;
       gBaseTxNmtQ : natural := 10240;
       gBaseTxSynQ : natural := 18432;
       gBaseTxVetQ : natural := 22528;
       gBaseU2KQ : natural := 36864;
       gVersionCount : natural := 0;
       gVersionMajor : natural := 255;
       gVersionMinor : natural := 255;
       gVersionRevision : natural := 255
  );
  port (
       iHostBridgeReaddata : in std_logic_vector(31 downto 0);
       iHostBridgeWaitrequest : in std_logic;
       iHostAddress : in std_logic_vector(16 downto 2);
       iHostByteenable : in std_logic_vector(3 downto 0);
       iHostRead : in std_logic;
       iHostWrite : in std_logic;
       iHostWritedata : in std_logic_vector(31 downto 0);
       iPcpAddress : in std_logic_vector(10 downto 2);
       iPcpByteenable : in std_logic_vector(3 downto 0);
       iPcpRead : in std_logic;
       iPcpWrite : in std_logic;
       iPcpWritedata : in std_logic_vector(31 downto 0);
       oHostBridgeAddress : out std_logic_vector(29 downto 0);
       oHostBridgeByteenable : out std_logic_vector(3 downto 0);
       oHostBridgeRead : out std_logic;
       oHostBridgeWrite : out std_logic;
       oHostBridgeWritedata : out std_logic_vector(31 downto 0);
       oHostReaddata : out std_logic_vector(31 downto 0);
       oHostWaitrequest : out std_logic;
       oPcpReaddata : out std_logic_vector(31 downto 0);
       oPcpWaitrequest : out std_logic;
       
       iIrqExtSync : in std_logic;
       iNodeId : in std_logic_vector(7 downto 0);
       iClk : in std_logic;
       iIrqIntSync : in std_logic;
       iRst : in std_logic;
       oPlkLedError : out std_logic;
       oPlkLedStatus : out std_logic;
       oIrq : out std_logic
  );
  
end component;
component pdi_dpr
  generic(
       LOG2_NUM_WORDS : integer := 10;
       NUM_WORDS : integer := 1024
  );
  port (
       address_a : in std_logic_vector(LOG2_NUM_WORDS-1 downto 0);
       address_b : in std_logic_vector(LOG2_NUM_WORDS-1 downto 0);
       byteena_a : in std_logic_vector(3 downto 0) := (others => '1');
       byteena_b : in std_logic_vector(3 downto 0) := (others => '1');
       clock_a : in std_logic := '1';
       clock_b : in std_logic;
       data_a : in std_logic_vector(31 downto 0);
       data_b : in std_logic_vector(31 downto 0);
       wren_a : in std_logic := '0';
       wren_b : in std_logic := '0';
       q_a : out std_logic_vector(31 downto 0);
       q_b : out std_logic_vector(31 downto 0)
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
-- base addresses
constant cVersionMajor : natural := 16#01#;
constant cVersionMinor : natural := 16#02#;
constant cVersionRevision : natural := 16#03#;
constant cVersionCount : natural := 16#FF#;
constant cBaseDynBuf0 : natural := 16#00800#;
constant cBaseDynBuf1 : natural := 16#01000#;
constant cBaseErrCntr : natural := 16#01800#;
constant cBaseTxNmtQ : natural := 16#02800#;
constant cBaseTxGenQ : natural := 16#03800#;
constant cBaseTxSynQ : natural := 16#04800#;
constant cBaseTxVetQ : natural := 16#05800#;
constant cBaseRxVetQ : natural := 16#06800#;
constant cBaseK2UQ : natural := 16#07000#;
constant cBaseU2KQ : natural := 16#09000#;
constant cBaseTpdo : natural := 16#0B000#;
constant cBaseRpdo : natural := 16#0E000#;
constant cBaseRes : natural := 16#14000#;


----     Constants     -----
constant VCC_CONSTANT   : std_logic := '1';
constant GND_CONSTANT   : std_logic := '0';

---- Signal declarations used on the diagram ----

signal oHostBridgeRead : std_logic;
signal avm_hostBridge_ready : std_logic;
signal avm_hostBridge_sel : std_logic;
signal iHostBridgeWaitrequest : std_logic;
signal oHostBridgeWrite : std_logic;
signal iHostRead : std_logic;
signal oHostWaitrequest : std_logic;
signal iHostWrite : std_logic;
signal iPcpRead : std_logic;
signal oPcpWaitrequest : std_logic;
signal iPcpWrite : std_logic;
signal clk : std_logic;
signal iIrqExtSync : std_logic;
signal oPlkLedError : std_logic;
signal oPlkLedStatus : std_logic;
signal done : std_logic;
signal GND : std_logic;
signal hostAck : std_logic;
signal hostDone : std_logic;
signal iIrqIntSync : std_logic;
signal oIrq : std_logic;
signal pcpAck : std_logic;
signal pcpDone : std_logic;
signal rst : std_logic;
signal VCC : std_logic;
signal oHostBridgeAddress : std_logic_vector (29 downto 0);
signal oHostBridgeByteenable : std_logic_vector (3 downto 0);
signal iHostBridgeReaddata : std_logic_vector (31 downto 0);
signal oHostBridgeWritedata : std_logic_vector (31 downto 0);
signal iHostAddress : std_logic_vector (16 downto 0);
signal iHostByteenable : std_logic_vector (3 downto 0);
signal oHostReaddata : std_logic_vector (31 downto 0);
signal iHostWritedata : std_logic_vector (31 downto 0);
signal iPcpAddress : std_logic_vector (10 downto 0);
signal iPcpByteenable : std_logic_vector (3 downto 0);
signal oPcpReaddata : std_logic_vector (31 downto 0);
signal iPcpWritedata : std_logic_vector (31 downto 0);
signal iNodeId : std_logic_vector (7 downto 0);
signal counter : std_logic_vector (7 downto 0);

begin

---- User Signal Assignments ----
process(clk)
begin

    if rising_edge(clk) then

        if rst = cActivated then
            counter <= (others => cInactivated);
        else
            counter <= std_logic_vector(
                unsigned(counter) + 1);
        end if;

    end if;

end process;

iIrqIntSync <= '1' when unsigned(counter) = 10 else '0';

iIrqExtSync <= '0';

iNodeId <= x"F0";

----  Component instantiations  ----

DUT : hostInterface
  generic map (
       gBaseDynBuf0 => cBaseDynBuf0,
       gBaseDynBuf1 => cBaseDynBuf1,
       gBaseErrCntr => cBaseErrCntr,
       gBaseK2UQ => cBaseK2UQ,
       gBaseRes => cBaseRes,
       gBaseRpdo => cBaseRpdo,
       gBaseRxVetQ => cBaseRxVetQ,
       gBaseTpdo => cBaseTpdo,
       gBaseTxGenQ => cBaseTxGenQ,
       gBaseTxNmtQ => cBaseTxNmtQ,
       gBaseTxSynQ => cBaseTxSynQ,
       gBaseTxVetQ => cBaseTxVetQ,
       gBaseU2KQ => cBaseU2KQ,
       gVersionCount => cVersionCount,
       gVersionMajor => cVersionMajor,
       gVersionMinor => cVersionMinor,
       gVersionRevision => cVersionRevision
  )
  port map(
       iHostAddress => iHostAddress,
       oHostBridgeAddress => oHostBridgeAddress,
       oHostBridgeByteenable => oHostBridgeByteenable,
       oHostBridgeRead => oHostBridgeRead,
       iHostBridgeReaddata => iHostBridgeReaddata,
       iHostBridgeWaitrequest => iHostBridgeWaitrequest,
       oHostBridgeWrite => oHostBridgeWrite,
       oHostBridgeWritedata => oHostBridgeWritedata,
       iHostByteenable => iHostByteenable,
       iHostRead => iHostRead,
       oHostReaddata => oHostReaddata,
       oHostWaitrequest => oHostWaitrequest,
       iHostWrite => iHostWrite,
       iHostWritedata => iHostWritedata,
       iPcpByteenable => iPcpByteenable,
       iPcpRead => iPcpRead,
       oPcpReaddata => oPcpReaddata,
       oPcpWaitrequest => oPcpWaitrequest,
       iPcpWrite => iPcpWrite,
       iPcpWritedata => iPcpWritedata,
       iIrqExtSync => iIrqExtSync,
       iNodeId => iNodeId,
       oPlkLedError => oPlkLedError,
       oPlkLedStatus => oPlkLedStatus,
       iClk => clk,
       iIrqIntSync => iIrqIntSync,
       oIrq => oIrq,
       iRst => rst
  );

RAM : pdi_dpr
  generic map (
       LOG2_NUM_WORDS => 21,
       NUM_WORDS => 2**21
  )
  port map(
       address_a => oHostBridgeAddress( 22 downto 2 ),
       address_b => oHostBridgeAddress( 22 downto 2 ),
       byteena_a => oHostBridgeByteenable,
       byteena_b => oHostBridgeByteenable,
       clock_a => clk,
       clock_b => clk,
       data_a => oHostBridgeWritedata,
       data_b => oHostBridgeWritedata,
       q_a => iHostBridgeReaddata,
       wren_a => oHostBridgeWrite,
       wren_b => GND
  );

RAM_ACK : req_ack
  generic map (
       ack_delay_g => 2,
       zero_delay_g => false
  )
  port map(
       ack => avm_hostBridge_ready,
       clk => clk,
       enable => avm_hostBridge_sel,
       rst => rst
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

hostAck <= not(oHostWaitrequest);

done <= hostDone and pcpDone;

pcpAck <= not(oPcpWaitrequest);

avm_hostBridge_sel <= oHostBridgeWrite or oHostBridgeRead;

iHostBridgeWaitrequest <= not(avm_hostBridge_ready);

host : busMaster
  generic map (
       gAddrWidth => 17,
       gDataWidth => 32,
       gStimuliFile => "HOST_INTERFACE/tb/tbHostInterface_ap_stim.txt"
  )
  port map(
       iAck => hostAck,
       iClk => clk,
       iEnable => VCC,
       iReaddata => oHostReaddata( 31 downto 0 ),
       iRst => rst,
       oAddress => iHostAddress( 16 downto 0 ),
       oByteenable => iHostByteenable( 3 downto 0 ),
       oDone => hostDone,
       oRead => iHostRead,
       oWrite => iHostWrite,
       oWritedata => iHostWritedata( 31 downto 0 )
  );

pcp : busMaster
  generic map (
       gAddrWidth => 11,
       gDataWidth => 32,
       gStimuliFile => "HOST_INTERFACE/tb/tbHostInterface_pcp_stim.txt"
  )
  port map(
       iAck => pcpAck,
       iClk => clk,
       iEnable => VCC,
       iReaddata => oPcpReaddata( 31 downto 0 ),
       iRst => rst,
       oAddress => iPcpAddress( 10 downto 0 ),
       oByteenable => iPcpByteenable( 3 downto 0 ),
       oDone => pcpDone,
       oRead => iPcpRead,
       oWrite => iPcpWrite,
       oWritedata => iPcpWritedata( 31 downto 0 )
  );


---- Power , ground assignment ----

GND <= GND_CONSTANT;
VCC <= VCC_CONSTANT;

end Bhv;
