-------------------------------------------------------------------------------
-- Entity : openMAC Testbench
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
use ieee.numeric_std.all;

library work;
use global.all;

entity tbOpenMAC is 
end tbOpenMAC;

architecture bhv of tbOpenMAC is

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
component edgeDet
  port (
       clk : in std_logic;
       din : in std_logic;
       rst : in std_logic;
       any : out std_logic;
       falling : out std_logic;
       rising : out std_logic
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
component OpenMAC_DMAFifo
  generic(
       fifo_data_width_g : natural := 16;
       fifo_word_size_g : natural := 32;
       fifo_word_size_log2_g : natural := 5
  );
  port (
       aclr : in std_logic;
       rd_clk : in std_logic;
       rd_req : in std_logic;
       wr_clk : in std_logic;
       wr_data : in std_logic_vector(fifo_data_width_g-1 downto 0);
       wr_req : in std_logic;
       rd_data : out std_logic_vector(fifo_data_width_g-1 downto 0);
       rd_empty : out std_logic;
       rd_full : out std_logic;
       rd_usedw : out std_logic_vector(fifo_word_size_log2_g-1 downto 0);
       wr_empty : out std_logic;
       wr_full : out std_logic;
       wr_usedw : out std_logic_vector(fifo_word_size_log2_g-1 downto 0)
  );
end component;
component openmac_ethernet
  generic(
       dma_highadr_g : integer := 31;
       endian_g : string := "little";
       gNumSmi : integer := 2;
       gen2ndCmpTimer_g : boolean := false;
       genHub_g : boolean := false;
       genPhyActLed_g : boolean := false;
       genSmiIO : boolean := true;
       gen_dma_observer_g : boolean := true;
       iPktBufSizeLog2_g : integer := 10;
       iPktBufSize_g : integer := 1024;
       m_burstcount_const_g : boolean := true;
       m_burstcount_width_g : integer := 4;
       m_data_width_g : integer := 16;
       m_rx_burst_size_g : integer := 16;
       m_rx_fifo_size_g : integer := 16;
       m_tx_burst_size_g : integer := 16;
       m_tx_fifo_size_g : integer := 16;
       simulate : boolean := false;
       useIntPktBuf_g : boolean := false;
       useRmii_g : boolean := true;
       useRxIntPktBuf_g : boolean := false
  );
  port (
       clk : in std_logic;
       clkx2 : in std_logic;
       m_clk : in std_logic;
       m_readdata : in std_logic_vector(m_data_width_g-1 downto 0) := (others => '0');
       m_readdatavalid : in std_logic;
       m_waitrequest : in std_logic;
       phy0_rx_dat : in std_logic_vector(1 downto 0);
       phy0_rx_dv : in std_logic;
       phy0_rx_err : in std_logic;
       phy0_smi_dio_I : in std_logic;
       phy1_rx_dat : in std_logic_vector(1 downto 0);
       phy1_rx_dv : in std_logic;
       phy1_rx_err : in std_logic;
       phy1_smi_dio_I : in std_logic;
       phyMii0_rx_clk : in std_logic;
       phyMii0_rx_dat : in std_logic_vector(3 downto 0);
       phyMii0_rx_dv : in std_logic;
       phyMii0_rx_err : in std_logic;
       phyMii0_tx_clk : in std_logic;
       phyMii1_rx_clk : in std_logic;
       phyMii1_rx_dat : in std_logic_vector(3 downto 0);
       phyMii1_rx_dv : in std_logic;
       phyMii1_rx_err : in std_logic;
       phyMii1_tx_clk : in std_logic;
       phy_smi_dio_I : in std_logic;
       pkt_address : in std_logic_vector(iPktBufSizeLog2_g-3 downto 0) := (others => '0');
       pkt_byteenable : in std_logic_vector(3 downto 0);
       pkt_chipselect : in std_logic;
       pkt_clk : in std_logic;
       pkt_read : in std_logic;
       pkt_write : in std_logic;
       pkt_writedata : in std_logic_vector(31 downto 0);
       rst : in std_logic;
       s_address : in std_logic_vector(11 downto 0);
       s_byteenable : in std_logic_vector(1 downto 0);
       s_chipselect : in std_logic;
       s_read : in std_logic;
       s_write : in std_logic;
       s_writedata : in std_logic_vector(15 downto 0);
       t_address : in std_logic_vector(1 downto 0);
       t_byteenable : in std_logic_vector(3 downto 0);
       t_chipselect : in std_logic;
       t_read : in std_logic;
       t_write : in std_logic;
       t_writedata : in std_logic_vector(31 downto 0);
       act_led : out std_logic;
       m_address : out std_logic_vector(29 downto 0);
       m_burstcount : out std_logic_vector(m_burstcount_width_g-1 downto 0);
       m_burstcounter : out std_logic_vector(m_burstcount_width_g-1 downto 0);
       m_byteenable : out std_logic_vector(m_data_width_g/8-1 downto 0);
       m_read : out std_logic;
       m_write : out std_logic;
       m_writedata : out std_logic_vector(m_data_width_g-1 downto 0);
       mac_rx_irq : out std_logic;
       mac_tx_irq : out std_logic;
       phy0_rst_n : out std_logic;
       phy0_smi_clk : out std_logic;
       phy0_smi_dio_O : out std_logic;
       phy0_smi_dio_T : out std_logic;
       phy0_tx_dat : out std_logic_vector(1 downto 0);
       phy0_tx_en : out std_logic;
       phy1_rst_n : out std_logic;
       phy1_smi_clk : out std_logic;
       phy1_smi_dio_O : out std_logic;
       phy1_smi_dio_T : out std_logic;
       phy1_tx_dat : out std_logic_vector(1 downto 0);
       phy1_tx_en : out std_logic;
       phyMii0_tx_dat : out std_logic_vector(3 downto 0);
       phyMii0_tx_en : out std_logic;
       phyMii1_tx_dat : out std_logic_vector(3 downto 0);
       phyMii1_tx_en : out std_logic;
       phy_rst_n : out std_logic;
       phy_smi_clk : out std_logic;
       phy_smi_dio_O : out std_logic;
       phy_smi_dio_T : out std_logic;
       pkt_readdata : out std_logic_vector(31 downto 0);
       pkt_waitrequest : out std_logic;
       s_irq : out std_logic;
       s_readdata : out std_logic_vector(15 downto 0);
       s_waitrequest : out std_logic;
       t_irq : out std_logic;
       t_readdata : out std_logic_vector(31 downto 0);
       t_tog : out std_logic;
       t_waitrequest : out std_logic;
       phy0_smi_dio : inout std_logic := '1';
       phy1_smi_dio : inout std_logic := '1';
       phy_smi_dio : inout std_logic := '1'
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
constant cDatawidth : integer := 16;

constant cEnableLoop : boolean := true;
constant cRmiiDelayExp : integer := 20; -- 2**10

constant cGenManCol : boolean := false; --collision with manual tx frame
constant cGenAutoCol : boolean := false; --collision with auto tx frame

constant cDmaDataWidth : integer := 32;
constant cDmaBurstWidth : integer := 6;


----     Constants     -----
constant DANGLING_INPUT_CONSTANT : std_logic := 'Z';
constant GND_CONSTANT   : std_logic := '0';

---- Signal declarations used on the diagram ----

signal ack : std_logic;
signal busMasterDone : std_logic;
signal clk100 : std_logic;
signal clk50 : std_logic;
signal done : std_logic := '1';
signal enable : std_logic;
signal fifo_rd_empty : std_logic;
signal fifo_rd_req : std_logic;
signal fifo_wr_req : std_logic;
signal fifo_wr_req_falling : std_logic;
signal GND : std_logic;
signal macDone : std_logic;
signal m_read : std_logic;
signal m_readdatavalid : std_logic;
signal m_waitrequest : std_logic;
signal m_write : std_logic;
signal NET799 : std_logic;
signal NET808 : std_logic;
signal phy0_rx_dv : std_logic;
signal phy0_rx_err : std_logic;
signal phy0_tx_en : std_logic;
signal phy1_rx_dv : std_logic;
signal phy1_rx_err : std_logic;
signal phy1_tx_en : std_logic;
signal read : std_logic;
signal reset : std_logic;
signal rst : std_logic;
signal sel : std_logic;
signal s_chipselect : std_logic;
signal s_read : std_logic;
signal s_waitrequest : std_logic;
signal s_write : std_logic;
signal write : std_logic;
signal address : std_logic_vector (cAddrwidth-1 downto 0);
signal byteenable : std_logic_vector (cDatawidth/8-1 downto 0);
signal fifo_rd_data : std_logic_vector (1 downto 0);
signal fifo_rd_usedw : std_logic_vector (cRmiiDelayExp-1 downto 0);
signal fifo_wr_data : std_logic_vector (1 downto 0);
signal fifo_wr_done : std_logic_vector (31 downto 0);
signal m_burstcount : std_logic_vector (cDmaBurstWidth-1 downto 0);
signal m_burstcounter : std_logic_vector (cDmaBurstWidth-1 downto 0);
signal phy0_rx_dat : std_logic_vector (1 downto 0);
signal phy0_tx_dat : std_logic_vector (1 downto 0);
signal phy1_rx_dat : std_logic_vector (1 downto 0);
signal phy1_tx_dat : std_logic_vector (1 downto 0);
signal readdata : std_logic_vector (cDatawidth-1 downto 0);
signal s_address : std_logic_vector (11 downto 0);
signal s_byteenable : std_logic_vector (1 downto 0);
signal s_readdata : std_logic_vector (15 downto 0);
signal s_writedata : std_logic_vector (15 downto 0);
signal writedata : std_logic_vector (cDatawidth-1 downto 0);

---- Declaration for Dangling input ----
signal Dangling_Input_Signal : STD_LOGIC;

begin

---- User Signal Assignments ----
--generate done signal
done <= busMasterDone and macDone;
--mac
macDone <= '1';
-- write
s_chipselect <= sel;
s_read <= read;
s_write <= write;
s_address <= address(s_address'high+1 downto 1);
s_byteenable <= byteenable;
s_writedata <= writedata;
-- read
readdata <= s_readdata;
ack <= not s_waitrequest;

--master
m_waitrequest <= not(m_read or m_write);
m_readdatavalid <= '0' when unsigned(m_burstcounter) = 0 or m_write = '1' else not m_read;

--phy if
genNoRx : if not cEnableLoop generate
begin
	phy0_rx_dv <= '0';
	phy0_rx_err <= '0';
	phy0_rx_dat <= "00";
end generate;

phy1_rx_dv <=
	phy0_tx_en after 2500 ns when cGenManCol else
	phy0_tx_en after 2500 ns when cGenAutoCol and unsigned(fifo_wr_done) = 1 else
	'0';
phy1_rx_err <= '0';
phy1_rx_dat <= 
	phy0_tx_dat after 2500 ns when cGenManCol else
	phy0_tx_dat after 2500 ns when cGenAutoCol and unsigned(fifo_wr_done) = 1 else
	"00";
fifo_wr_req <= phy0_tx_en;

genLoop : if cEnableLoop generate
begin
	fifo_wr_data <= phy0_tx_dat;
	phy0_rx_dv <= fifo_rd_req;
	phy0_rx_dat <= fifo_rd_data(1 downto 0);
	phy0_rx_err <= '0';
end generate;

procFifoRdReq : process(clk50, reset)
begin
	if reset = '1' then
		fifo_rd_req <= '0';
		fifo_wr_done <= (others => '0');
	elsif rising_edge(clk50) then
		if fifo_rd_empty = '1' then
			fifo_rd_req <= '0';
		elsif fifo_wr_req_falling = '1' then
			fifo_rd_req <= '1' after 1 us;
			fifo_wr_done <= std_logic_vector(unsigned(fifo_wr_done) + 1);
--		elsif unsigned(fifo_rd_usedw) > 106 and unsigned(fifo_wr_done) = 0 then
--			fifo_rd_req <= '1'; -- after 1 us;
--			fifo_wr_done <= std_logic_vector(unsigned(fifo_wr_done) + 1);
		end if;
	end if;
end process;

----  Component instantiations  ----

DUT : openmac_ethernet
  generic map (
       dma_highadr_g => 29,
       endian_g => "little",
       gNumSmi => 2,
       gen2ndCmpTimer_g => false,
       genHub_g => true,
       genPhyActLed_g => false,
       genSmiIO => true,
       gen_dma_observer_g => true,
       iPktBufSizeLog2_g => 10,
       iPktBufSize_g => 1024,
       m_burstcount_const_g => true,
       m_burstcount_width_g => cDmaBurstWidth,
       m_data_width_g => cDmaDataWidth,
       m_rx_burst_size_g => 2**(cDmaBurstWidth-1),
       m_rx_fifo_size_g => 3*2**cDmaBurstWidth,
       m_tx_burst_size_g => 2**(cDmaBurstWidth-1),
       m_tx_fifo_size_g => 3*2**cDmaBurstWidth,
       simulate => false,
       useIntPktBuf_g => false,
       useRmii_g => true,
       useRxIntPktBuf_g => false
  )
  port map(
       phyMii0_rx_dat(0) => Dangling_Input_Signal,
       phyMii0_rx_dat(1) => Dangling_Input_Signal,
       phyMii0_rx_dat(2) => Dangling_Input_Signal,
       phyMii0_rx_dat(3) => Dangling_Input_Signal,
       phyMii1_rx_dat(0) => Dangling_Input_Signal,
       phyMii1_rx_dat(1) => Dangling_Input_Signal,
       phyMii1_rx_dat(2) => Dangling_Input_Signal,
       phyMii1_rx_dat(3) => Dangling_Input_Signal,
       pkt_byteenable(0) => Dangling_Input_Signal,
       pkt_byteenable(1) => Dangling_Input_Signal,
       pkt_byteenable(2) => Dangling_Input_Signal,
       pkt_byteenable(3) => Dangling_Input_Signal,
       pkt_writedata(0) => Dangling_Input_Signal,
       pkt_writedata(1) => Dangling_Input_Signal,
       pkt_writedata(2) => Dangling_Input_Signal,
       pkt_writedata(3) => Dangling_Input_Signal,
       pkt_writedata(4) => Dangling_Input_Signal,
       pkt_writedata(5) => Dangling_Input_Signal,
       pkt_writedata(6) => Dangling_Input_Signal,
       pkt_writedata(7) => Dangling_Input_Signal,
       pkt_writedata(8) => Dangling_Input_Signal,
       pkt_writedata(9) => Dangling_Input_Signal,
       pkt_writedata(10) => Dangling_Input_Signal,
       pkt_writedata(11) => Dangling_Input_Signal,
       pkt_writedata(12) => Dangling_Input_Signal,
       pkt_writedata(13) => Dangling_Input_Signal,
       pkt_writedata(14) => Dangling_Input_Signal,
       pkt_writedata(15) => Dangling_Input_Signal,
       pkt_writedata(16) => Dangling_Input_Signal,
       pkt_writedata(17) => Dangling_Input_Signal,
       pkt_writedata(18) => Dangling_Input_Signal,
       pkt_writedata(19) => Dangling_Input_Signal,
       pkt_writedata(20) => Dangling_Input_Signal,
       pkt_writedata(21) => Dangling_Input_Signal,
       pkt_writedata(22) => Dangling_Input_Signal,
       pkt_writedata(23) => Dangling_Input_Signal,
       pkt_writedata(24) => Dangling_Input_Signal,
       pkt_writedata(25) => Dangling_Input_Signal,
       pkt_writedata(26) => Dangling_Input_Signal,
       pkt_writedata(27) => Dangling_Input_Signal,
       pkt_writedata(28) => Dangling_Input_Signal,
       pkt_writedata(29) => Dangling_Input_Signal,
       pkt_writedata(30) => Dangling_Input_Signal,
       pkt_writedata(31) => Dangling_Input_Signal,
       t_address(0) => Dangling_Input_Signal,
       t_address(1) => Dangling_Input_Signal,
       t_byteenable(0) => Dangling_Input_Signal,
       t_byteenable(1) => Dangling_Input_Signal,
       t_byteenable(2) => Dangling_Input_Signal,
       t_byteenable(3) => Dangling_Input_Signal,
       t_writedata(0) => Dangling_Input_Signal,
       t_writedata(1) => Dangling_Input_Signal,
       t_writedata(2) => Dangling_Input_Signal,
       t_writedata(3) => Dangling_Input_Signal,
       t_writedata(4) => Dangling_Input_Signal,
       t_writedata(5) => Dangling_Input_Signal,
       t_writedata(6) => Dangling_Input_Signal,
       t_writedata(7) => Dangling_Input_Signal,
       t_writedata(8) => Dangling_Input_Signal,
       t_writedata(9) => Dangling_Input_Signal,
       t_writedata(10) => Dangling_Input_Signal,
       t_writedata(11) => Dangling_Input_Signal,
       t_writedata(12) => Dangling_Input_Signal,
       t_writedata(13) => Dangling_Input_Signal,
       t_writedata(14) => Dangling_Input_Signal,
       t_writedata(15) => Dangling_Input_Signal,
       t_writedata(16) => Dangling_Input_Signal,
       t_writedata(17) => Dangling_Input_Signal,
       t_writedata(18) => Dangling_Input_Signal,
       t_writedata(19) => Dangling_Input_Signal,
       t_writedata(20) => Dangling_Input_Signal,
       t_writedata(21) => Dangling_Input_Signal,
       t_writedata(22) => Dangling_Input_Signal,
       t_writedata(23) => Dangling_Input_Signal,
       t_writedata(24) => Dangling_Input_Signal,
       t_writedata(25) => Dangling_Input_Signal,
       t_writedata(26) => Dangling_Input_Signal,
       t_writedata(27) => Dangling_Input_Signal,
       t_writedata(28) => Dangling_Input_Signal,
       t_writedata(29) => Dangling_Input_Signal,
       t_writedata(30) => Dangling_Input_Signal,
       t_writedata(31) => Dangling_Input_Signal,
       clk => clk50,
       clkx2 => clk100,
       m_burstcount => m_burstcount( cDmaBurstWidth-1 downto 0 ),
       m_burstcounter => m_burstcounter( cDmaBurstWidth-1 downto 0 ),
       m_clk => clk100,
       m_read => m_read,
       m_readdatavalid => m_readdatavalid,
       m_waitrequest => m_waitrequest,
       m_write => m_write,
       phy0_rx_dat => phy0_rx_dat,
       phy0_rx_dv => phy0_rx_dv,
       phy0_rx_err => phy0_rx_err,
       phy0_smi_dio_I => Dangling_Input_Signal,
       phy0_tx_dat => phy0_tx_dat,
       phy0_tx_en => phy0_tx_en,
       phy1_rx_dat => phy1_rx_dat,
       phy1_rx_dv => phy1_rx_dv,
       phy1_rx_err => phy1_rx_err,
       phy1_smi_dio_I => Dangling_Input_Signal,
       phy1_tx_dat => phy1_tx_dat,
       phy1_tx_en => phy1_tx_en,
       phyMii0_rx_clk => Dangling_Input_Signal,
       phyMii0_rx_dv => Dangling_Input_Signal,
       phyMii0_rx_err => Dangling_Input_Signal,
       phyMii0_tx_clk => Dangling_Input_Signal,
       phyMii1_rx_clk => Dangling_Input_Signal,
       phyMii1_rx_dv => Dangling_Input_Signal,
       phyMii1_rx_err => Dangling_Input_Signal,
       phyMii1_tx_clk => Dangling_Input_Signal,
       phy_smi_dio_I => Dangling_Input_Signal,
       pkt_chipselect => Dangling_Input_Signal,
       pkt_clk => Dangling_Input_Signal,
       pkt_read => Dangling_Input_Signal,
       pkt_write => Dangling_Input_Signal,
       rst => reset,
       s_address => s_address,
       s_byteenable => s_byteenable,
       s_chipselect => s_chipselect,
       s_read => s_read,
       s_readdata => s_readdata,
       s_waitrequest => s_waitrequest,
       s_write => s_write,
       s_writedata => s_writedata,
       t_chipselect => Dangling_Input_Signal,
       t_read => Dangling_Input_Signal,
       t_write => Dangling_Input_Signal
  );

RMII_DELAY : OpenMAC_DMAFifo
  generic map (
       fifo_data_width_g => 2,
       fifo_word_size_g => 2**cRmiiDelayExp,
       fifo_word_size_log2_g => cRmiiDelayExp
  )
  port map(
       aclr => reset,
       rd_clk => clk50,
       rd_data => fifo_rd_data( 1 downto 0 ),
       rd_empty => fifo_rd_empty,
       rd_req => fifo_rd_req,
       rd_usedw => fifo_rd_usedw( cRmiiDelayExp-1 downto 0 ),
       wr_clk => clk50,
       wr_data => fifo_wr_data( 1 downto 0 ),
       wr_req => fifo_wr_req
  );

TRIG_READ : edgeDet
  port map(
       clk => clk50,
       din => fifo_wr_req,
       falling => fifo_wr_req_falling,
       rst => rst
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
       gEnableDelay => 100 ns
  )
  port map(
       iReset => reset,
       oEnable => enable
  );

U4 : busMaster
  generic map (
       gAddrWidth => cAddrwidth,
       gDataWidth => cDatawidth,
       gStimuliFile => "openMAC/tb/tbOpenMAC_stim.txt"
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
       ack => NET799,
       clk => clk50,
       enable => write,
       rst => reset
  );

U6 : req_ack
  generic map (
       ack_delay_g => 1,
       zero_delay_g => false
  )
  port map(
       ack => NET808,
       clk => clk50,
       enable => read,
       rst => reset
  );

ack <= NET808 or NET799;

U8 : clkgen
  generic map (
       gPeriod => 10 ns
  )
  port map(
       iDone => done,
       oClk => clk100
  );


---- Power , ground assignment ----

GND <= GND_CONSTANT;

---- Dangling input signal assignment ----

Dangling_Input_Signal <= DANGLING_INPUT_CONSTANT;

end bhv;
