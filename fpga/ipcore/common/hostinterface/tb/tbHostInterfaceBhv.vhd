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
       avm_hostBridge_readdata : in std_logic_vector(31 downto 0);
       avm_hostBridge_waitrequest : in std_logic;
       avs_host_address : in std_logic_vector(16 downto 2);
       avs_host_byteenable : in std_logic_vector(3 downto 0);
       avs_host_read : in std_logic;
       avs_host_write : in std_logic;
       avs_host_writedata : in std_logic_vector(31 downto 0);
       avs_pcp_address : in std_logic_vector(10 downto 2);
       avs_pcp_byteenable : in std_logic_vector(3 downto 0);
       avs_pcp_read : in std_logic;
       avs_pcp_write : in std_logic;
       avs_pcp_writedata : in std_logic_vector(31 downto 0);
       coe_ExtSync_exsync : in std_logic;
       coe_Gpio_gpin : in std_logic_vector(15 downto 0);
       coe_NodeId_nodeid : in std_logic_vector(7 downto 0);
       csi_c0_clock : in std_logic;
       inr_irqSync_irq : in std_logic;
       rsi_r0_reset : in std_logic;
       avm_hostBridge_address : out std_logic_vector(29 downto 0);
       avm_hostBridge_byteenable : out std_logic_vector(3 downto 0);
       avm_hostBridge_read : out std_logic;
       avm_hostBridge_write : out std_logic;
       avm_hostBridge_writedata : out std_logic_vector(31 downto 0);
       avs_host_readdata : out std_logic_vector(31 downto 0);
       avs_host_waitrequest : out std_logic;
       avs_pcp_readdata : out std_logic_vector(31 downto 0);
       avs_pcp_waitrequest : out std_logic;
       coe_Gpio_gpout : out std_logic_vector(15 downto 0);
       coe_PlkLed_lederr : out std_logic;
       coe_PlkLed_ledst : out std_logic;
       ins_irqOut_irq : out std_logic
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

signal avm_hostBridge_read : std_logic;
signal avm_hostBridge_ready : std_logic;
signal avm_hostBridge_sel : std_logic;
signal avm_hostBridge_waitrequest : std_logic;
signal avm_hostBridge_write : std_logic;
signal avs_host_read : std_logic;
signal avs_host_waitrequest : std_logic;
signal avs_host_write : std_logic;
signal avs_pcp_read : std_logic;
signal avs_pcp_waitrequest : std_logic;
signal avs_pcp_write : std_logic;
signal clk : std_logic;
signal coe_ExtSync_exsync : std_logic;
signal coe_PlkLed_lederr : std_logic;
signal coe_PlkLed_ledst : std_logic;
signal done : std_logic;
signal GND : std_logic;
signal hostAck : std_logic;
signal hostDone : std_logic;
signal inr_irqSync_irq : std_logic;
signal ins_irqOut_irq : std_logic;
signal pcpAck : std_logic;
signal pcpDone : std_logic;
signal rst : std_logic;
signal VCC : std_logic;
signal avm_hostBridge_address : std_logic_vector (29 downto 0);
signal avm_hostBridge_byteenable : std_logic_vector (3 downto 0);
signal avm_hostBridge_readdata : std_logic_vector (31 downto 0);
signal avm_hostBridge_writedata : std_logic_vector (31 downto 0);
signal avs_host_address : std_logic_vector (16 downto 0);
signal avs_host_byteenable : std_logic_vector (3 downto 0);
signal avs_host_readdata : std_logic_vector (31 downto 0);
signal avs_host_writedata : std_logic_vector (31 downto 0);
signal avs_pcp_address : std_logic_vector (10 downto 0);
signal avs_pcp_byteenable : std_logic_vector (3 downto 0);
signal avs_pcp_readdata : std_logic_vector (31 downto 0);
signal avs_pcp_writedata : std_logic_vector (31 downto 0);
signal coe_Gpio_gpin : std_logic_vector (15 downto 0);
signal coe_Gpio_gpout : std_logic_vector (15 downto 0);
signal coe_NodeId_nodeid : std_logic_vector (7 downto 0);
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

inr_irqSync_irq <= '1' when unsigned(counter) = 10 else '0';

coe_ExtSync_exsync <= '0';

coe_NodeId_nodeid <= x"F0";

coe_Gpio_gpin <= x"ABCD";

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
       avs_host_address(2) => avs_host_address(2),
       avs_host_address(3) => avs_host_address(3),
       avs_host_address(4) => avs_host_address(4),
       avs_host_address(5) => avs_host_address(5),
       avs_host_address(6) => avs_host_address(6),
       avs_host_address(7) => avs_host_address(7),
       avs_host_address(8) => avs_host_address(8),
       avs_host_address(9) => avs_host_address(9),
       avs_host_address(10) => avs_host_address(10),
       avs_host_address(11) => avs_host_address(11),
       avs_host_address(12) => avs_host_address(12),
       avs_host_address(13) => avs_host_address(13),
       avs_host_address(14) => avs_host_address(14),
       avs_host_address(15) => avs_host_address(15),
       avs_host_address(16) => avs_host_address(16),
       avs_pcp_address(2) => avs_pcp_address(2),
       avs_pcp_address(3) => avs_pcp_address(3),
       avs_pcp_address(4) => avs_pcp_address(4),
       avs_pcp_address(5) => avs_pcp_address(5),
       avs_pcp_address(6) => avs_pcp_address(6),
       avs_pcp_address(7) => avs_pcp_address(7),
       avs_pcp_address(8) => avs_pcp_address(8),
       avs_pcp_address(9) => avs_pcp_address(9),
       avs_pcp_address(10) => avs_pcp_address(10),
       avm_hostBridge_address => avm_hostBridge_address,
       avm_hostBridge_byteenable => avm_hostBridge_byteenable,
       avm_hostBridge_read => avm_hostBridge_read,
       avm_hostBridge_readdata => avm_hostBridge_readdata,
       avm_hostBridge_waitrequest => avm_hostBridge_waitrequest,
       avm_hostBridge_write => avm_hostBridge_write,
       avm_hostBridge_writedata => avm_hostBridge_writedata,
       avs_host_byteenable => avs_host_byteenable,
       avs_host_read => avs_host_read,
       avs_host_readdata => avs_host_readdata,
       avs_host_waitrequest => avs_host_waitrequest,
       avs_host_write => avs_host_write,
       avs_host_writedata => avs_host_writedata,
       avs_pcp_byteenable => avs_pcp_byteenable,
       avs_pcp_read => avs_pcp_read,
       avs_pcp_readdata => avs_pcp_readdata,
       avs_pcp_waitrequest => avs_pcp_waitrequest,
       avs_pcp_write => avs_pcp_write,
       avs_pcp_writedata => avs_pcp_writedata,
       coe_ExtSync_exsync => coe_ExtSync_exsync,
       coe_Gpio_gpin => coe_Gpio_gpin,
       coe_Gpio_gpout => coe_Gpio_gpout,
       coe_NodeId_nodeid => coe_NodeId_nodeid,
       coe_PlkLed_lederr => coe_PlkLed_lederr,
       coe_PlkLed_ledst => coe_PlkLed_ledst,
       csi_c0_clock => clk,
       inr_irqSync_irq => inr_irqSync_irq,
       ins_irqOut_irq => ins_irqOut_irq,
       rsi_r0_reset => rst
  );

RAM : pdi_dpr
  generic map (
       LOG2_NUM_WORDS => 21,
       NUM_WORDS => 2**21
  )
  port map(
       address_a => avm_hostBridge_address( 22 downto 2 ),
       address_b => avm_hostBridge_address( 22 downto 2 ),
       byteena_a => avm_hostBridge_byteenable,
       byteena_b => avm_hostBridge_byteenable,
       clock_a => clk,
       clock_b => clk,
       data_a => avm_hostBridge_writedata,
       data_b => avm_hostBridge_writedata,
       q_a => avm_hostBridge_readdata,
       wren_a => avm_hostBridge_write,
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

hostAck <= not(avs_host_waitrequest);

done <= hostDone and pcpDone;

pcpAck <= not(avs_pcp_waitrequest);

avm_hostBridge_sel <= avm_hostBridge_write or avm_hostBridge_read;

avm_hostBridge_waitrequest <= not(avm_hostBridge_ready);

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
       iReaddata => avs_host_readdata( 31 downto 0 ),
       iRst => rst,
       oAddress => avs_host_address( 16 downto 0 ),
       oByteenable => avs_host_byteenable( 3 downto 0 ),
       oDone => hostDone,
       oRead => avs_host_read,
       oWrite => avs_host_write,
       oWritedata => avs_host_writedata( 31 downto 0 )
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
       iReaddata => avs_pcp_readdata( 31 downto 0 ),
       iRst => rst,
       oAddress => avs_pcp_address( 10 downto 0 ),
       oByteenable => avs_pcp_byteenable( 3 downto 0 ),
       oDone => pcpDone,
       oRead => avs_pcp_read,
       oWrite => avs_pcp_write,
       oWritedata => avs_pcp_writedata( 31 downto 0 )
  );


---- Power , ground assignment ----

GND <= GND_CONSTANT;
VCC <= VCC_CONSTANT;

end Bhv;
