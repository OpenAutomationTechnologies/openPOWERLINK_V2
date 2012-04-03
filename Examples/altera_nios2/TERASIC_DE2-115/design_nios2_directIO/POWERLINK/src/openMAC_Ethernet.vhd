-------------------------------------------------------------------------------
-- Entity : openMAC_Ethernet
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
--
-- This is the top level of openMAC. 
-- It instantiates openMAC, openHUB, openFILTER and other components for the
--  MAC-layer.
--
-------------------------------------------------------------------------------
--
-- 2011-07-26   V0.01    zelenkaj    First version
-- 2011-10-11   V0.02    zelenkaj    ack for pkt was clocked by clk50
-- 2011-10-13   V0.03    zelenkaj    changed names of instances
-- 2011-11-07   V0.04    zelenkaj    added big/little endian consideration
--                                   minor changes in SMI core generation
-- 2011-11-28   V0.05    zelenkaj    Added DMA observer
-- 2011-11-29   V0.06    zelenkaj    waitrequest for mac_reg is gen. once
--                                   tx_off / rx_off is derived in openMAC
-- 2011-11-30   V0.07    zelenkaj    Added generic for DMA observer
--                                   Fixed generic assignments for DMA master
-- 2011-12-02   V0.08    zelenkaj    Added Dma Req Overflow
-- 2011-12-05   V0.09    zelenkaj    Reduced Dma Req overflow vector
-- 2012-01-26   V0.10    zelenkaj    Revised SMI to use one SMI with two phys
--
-------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;

entity openmac_ethernet is
  generic(
       genSmiIO : boolean := true;
       gNumSmi : integer := 2;
       gen2ndCmpTimer_g : boolean := false;
       simulate : boolean := false;
       dma_highadr_g : integer := 31;
       m_data_width_g : integer := 16;
       m_burstcount_width_g : integer := 4;
       m_burstcount_const_g : boolean := true;
       m_tx_fifo_size_g : integer := 16;
       m_rx_fifo_size_g : integer := 16;
       m_tx_burst_size_g : integer := 16;
       m_rx_burst_size_g : integer := 16;
       endian_g : string := "little";
       genPhyActLed_g : boolean := false;
       gen_dma_observer_g : boolean := true;
       useIntPktBuf_g : boolean := false;
       useRxIntPktBuf_g : boolean := false;
       iPktBufSize_g : integer := 1024;
       iPktBufSizeLog2_g : integer := 10;
       genHub_g : boolean := false;
       useRmii_g : boolean := true
  );
  port(
       clk : in std_logic;
       clkx2 : in std_logic;
       m_clk : in std_logic;
       m_readdatavalid : in std_logic;
       m_waitrequest : in std_logic;
       phy0_rx_dv : in std_logic;
       phy0_rx_err : in std_logic;
       phy0_smi_dio_I : in std_logic;
       phy1_rx_dv : in std_logic;
       phy1_rx_err : in std_logic;
       phy1_smi_dio_I : in std_logic;
       phyMii0_rx_clk : in std_logic;
       phyMii0_rx_dv : in std_logic;
       phyMii0_rx_err : in std_logic;
       phyMii0_tx_clk : in std_logic;
       phyMii1_rx_clk : in std_logic;
       phyMii1_rx_dv : in std_logic;
       phyMii1_rx_err : in std_logic;
       phyMii1_tx_clk : in std_logic;
       phy_smi_dio_I : in std_logic;
       pkt_chipselect : in std_logic;
       pkt_clk : in std_logic;
       pkt_read : in std_logic;
       pkt_write : in std_logic;
       rst : in std_logic;
       s_chipselect : in std_logic;
       s_read : in std_logic;
       s_write : in std_logic;
       t_chipselect : in std_logic;
       t_read : in std_logic;
       t_write : in std_logic;
       m_readdata : in std_logic_vector(m_data_width_g-1 downto 0);
       phy0_rx_dat : in std_logic_vector(1 downto 0);
       phy1_rx_dat : in std_logic_vector(1 downto 0);
       phyMii0_rx_dat : in std_logic_vector(3 downto 0);
       phyMii1_rx_dat : in std_logic_vector(3 downto 0);
       pkt_address : in std_logic_vector(iPktBufSizeLog2_g-3 downto 0);
       pkt_byteenable : in std_logic_vector(3 downto 0);
       pkt_writedata : in std_logic_vector(31 downto 0);
       s_address : in std_logic_vector(11 downto 0);
       s_byteenable : in std_logic_vector(1 downto 0);
       s_writedata : in std_logic_vector(15 downto 0);
       t_address : in std_logic_vector(1 downto 0);
       t_byteenable : in std_logic_vector(3 downto 0);
       t_writedata : in std_logic_vector(31 downto 0);
       act_led : out std_logic;
       m_read : out std_logic;
       m_write : out std_logic;
       mac_rx_irq : out std_logic;
       mac_tx_irq : out std_logic;
       phy0_rst_n : out std_logic;
       phy0_smi_clk : out std_logic;
       phy0_smi_dio_O : out std_logic;
       phy0_smi_dio_T : out std_logic;
       phy0_tx_en : out std_logic;
       phy1_rst_n : out std_logic;
       phy1_smi_clk : out std_logic;
       phy1_smi_dio_O : out std_logic;
       phy1_smi_dio_T : out std_logic;
       phy1_tx_en : out std_logic;
       phyMii0_tx_en : out std_logic;
       phyMii1_tx_en : out std_logic;
       phy_rst_n : out std_logic;
       phy_smi_clk : out std_logic;
       phy_smi_dio_O : out std_logic;
       phy_smi_dio_T : out std_logic;
       pkt_waitrequest : out std_logic;
       s_irq : out std_logic;
       s_waitrequest : out std_logic;
       t_irq : out std_logic;
       t_tog : out std_logic;
       t_waitrequest : out std_logic;
       m_address : out std_logic_vector(29 downto 0);
       m_burstcount : out std_logic_vector(m_burstcount_width_g-1 downto 0);
       m_burstcounter : out std_logic_vector(m_burstcount_width_g-1 downto 0);
       m_byteenable : out std_logic_vector(m_data_width_g/8-1 downto 0);
       m_writedata : out std_logic_vector(m_data_width_g-1 downto 0);
       phy0_tx_dat : out std_logic_vector(1 downto 0);
       phy1_tx_dat : out std_logic_vector(1 downto 0);
       phyMii0_tx_dat : out std_logic_vector(3 downto 0);
       phyMii1_tx_dat : out std_logic_vector(3 downto 0);
       pkt_readdata : out std_logic_vector(31 downto 0);
       s_readdata : out std_logic_vector(15 downto 0);
       t_readdata : out std_logic_vector(31 downto 0);
       phy0_smi_dio : inout std_logic := '1';
       phy1_smi_dio : inout std_logic := '1';
       phy_smi_dio : inout std_logic := '1'
  );
end openmac_ethernet;

architecture rtl of openmac_ethernet is

---- Component declarations -----

component addr_decoder
  generic(
       addrWidth_g : integer := 32;
       baseaddr_g : integer := 4096;
       highaddr_g : integer := 8191
  );
  port (
       addr : in std_logic_vector(addrWidth_g-1 downto 0);
       selin : in std_logic;
       selout : out std_logic
  );
end component;
component openFILTER
  generic(
       bypassFilter : boolean := false
  );
  port (
       Clk : in std_logic;
       Rst : in std_logic;
       RxDatIn : in std_logic_vector(1 downto 0);
       RxDvIn : in std_logic;
       RxErr : in std_logic := '0';
       TxDatIn : in std_logic_vector(1 downto 0);
       TxEnIn : in std_logic;
       nCheckShortFrames : in std_logic := '0';
       RxDatOut : out std_logic_vector(1 downto 0);
       RxDvOut : out std_logic;
       TxDatOut : out std_logic_vector(1 downto 0);
       TxEnOut : out std_logic
  );
end component;
component OpenHUB
  generic(
       Ports : integer := 3
  );
  port (
       Clk : in std_logic;
       Rst : in std_logic;
       RxDat0 : in std_logic_vector(Ports downto 1);
       RxDat1 : in std_logic_vector(Ports downto 1);
       RxDv : in std_logic_vector(Ports downto 1);
       TransmitMask : in std_logic_vector(Ports downto 1) := (others => '1');
       internPort : in integer range 1 to ports := 1;
       ReceivePort : out integer range 0 to ports;
       TxDat0 : out std_logic_vector(Ports downto 1);
       TxDat1 : out std_logic_vector(Ports downto 1);
       TxEn : out std_logic_vector(Ports downto 1)
  );
end component;
component OpenMAC
  generic(
       HighAdr : integer := 16;
       Simulate : boolean := false;
       Timer : boolean := false;
       TxDel : boolean := false;
       TxSyncOn : boolean := false
  );
  port (
       Clk : in std_logic;
       Dma_Ack : in std_logic;
       Dma_Din : in std_logic_vector(15 downto 0);
       Hub_Rx : in std_logic_vector(1 downto 0) := "00";
       Rst : in std_logic;
       S_Adr : in std_logic_vector(10 downto 1);
       S_Din : in std_logic_vector(15 downto 0);
       S_nBe : in std_logic_vector(1 downto 0);
       Sel_Cont : in std_logic := '0';
       Sel_Ram : in std_logic := '0';
       rCrs_Dv : in std_logic;
       rRx_Dat : in std_logic_vector(1 downto 0);
       s_nWr : in std_logic := '0';
       Dma_Addr : out std_logic_vector(HighAdr downto 1);
       Dma_Dout : out std_logic_vector(15 downto 0);
       Dma_Rd_Done : out std_logic;
       Dma_Req : out std_logic;
       Dma_Req_Overflow : out std_logic;
       Dma_Rw : out std_logic;
       Dma_Wr_Done : out std_logic;
       Mac_Zeit : out std_logic_vector(31 downto 0);
       S_Dout : out std_logic_vector(15 downto 0);
       nRx_Int : out std_logic;
       nTx_BegInt : out std_logic;
       nTx_Int : out std_logic;
       rTx_Dat : out std_logic_vector(1 downto 0);
       rTx_En : out std_logic
  );
end component;
component openMAC_cmp
  generic(
       gen2ndCmpTimer_g : boolean := false;
       mac_time_width_g : integer := 32
  );
  port (
       addr : in std_logic_vector(1 downto 0);
       clk : in std_logic;
       din : in std_logic_vector(31 downto 0);
       mac_time : in std_logic_vector(mac_time_width_g-1 downto 0);
       rst : in std_logic;
       wr : in std_logic;
       dout : out std_logic_vector(31 downto 0);
       irq : out std_logic;
       toggle : out std_logic
  );
end component;
component openMAC_DMAmaster
  generic(
       dma_highadr_g : integer := 31;
       endian_g : string := "little";
       fifo_data_width_g : integer := 16;
       gen_dma_observer_g : boolean := true;
       gen_rx_fifo_g : boolean := true;
       gen_tx_fifo_g : boolean := true;
       m_burstcount_const_g : boolean := true;
       m_burstcount_width_g : integer := 4;
       m_rx_burst_size_g : integer := 16;
       m_tx_burst_size_g : integer := 16;
       rx_fifo_word_size_g : integer := 32;
       simulate : boolean := false;
       tx_fifo_word_size_g : integer := 32
  );
  port (
       dma_addr : in std_logic_vector(dma_highadr_g downto 1);
       dma_clk : in std_logic;
       dma_dout : in std_logic_vector(15 downto 0);
       dma_req_overflow : in std_logic;
       dma_req_rd : in std_logic;
       dma_req_wr : in std_logic;
       m_clk : in std_logic;
       m_readdata : in std_logic_vector(fifo_data_width_g-1 downto 0);
       m_readdatavalid : in std_logic;
       m_waitrequest : in std_logic;
       mac_rx_off : in std_logic;
       mac_tx_off : in std_logic;
       rst : in std_logic;
       dma_ack_rd : out std_logic;
       dma_ack_wr : out std_logic;
       dma_din : out std_logic_vector(15 downto 0);
       dma_rd_err : out std_logic;
       dma_wr_err : out std_logic;
       m_address : out std_logic_vector(dma_highadr_g downto 0);
       m_burstcount : out std_logic_vector(m_burstcount_width_g-1 downto 0);
       m_burstcounter : out std_logic_vector(m_burstcount_width_g-1 downto 0);
       m_byteenable : out std_logic_vector(fifo_data_width_g/8-1 downto 0);
       m_read : out std_logic;
       m_write : out std_logic;
       m_writedata : out std_logic_vector(fifo_data_width_g-1 downto 0)
  );
end component;
component OpenMAC_DPRpackets
  generic(
       memSizeLOG2_g : integer := 10;
       memSize_g : integer := 1024
  );
  port (
       address_a : in std_logic_vector(memSizeLOG2_g-2 downto 0);
       address_b : in std_logic_vector(memSizeLOG2_g-3 downto 0);
       byteena_a : in std_logic_vector(1 downto 0) := (others => '1');
       byteena_b : in std_logic_vector(3 downto 0) := (others => '1');
       clock_a : in std_logic := '1';
       clock_b : in std_logic;
       data_a : in std_logic_vector(15 downto 0);
       data_b : in std_logic_vector(31 downto 0);
       rden_a : in std_logic := '1';
       rden_b : in std_logic := '1';
       wren_a : in std_logic := '0';
       wren_b : in std_logic := '0';
       q_a : out std_logic_vector(15 downto 0);
       q_b : out std_logic_vector(31 downto 0)
  );
end component;
component OpenMAC_MII
  port (
       Addr : in std_logic_vector(2 downto 0);
       Clk : in std_logic;
       Data_In : in std_logic_vector(15 downto 0);
       Mii_Di : in std_logic;
       Rst : in std_logic;
       Sel : in std_logic;
       nBe : in std_logic_vector(1 downto 0);
       nWr : in std_logic;
       Data_Out : out std_logic_vector(15 downto 0);
       Mii_Clk : out std_logic;
       Mii_Do : out std_logic;
       Mii_Doe : out std_logic;
       nResetOut : out std_logic
  );
end component;
component OpenMAC_phyAct
  generic(
       iBlinkFreq_g : integer := 6
  );
  port (
       clk : in std_logic;
       rst : in std_logic;
       rx_dv : in std_logic;
       tx_en : in std_logic;
       act_led : out std_logic
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
component rmii2mii
  port (
       clk50 : in std_logic;
       mRxClk : in std_logic;
       mRxDat : in std_logic_vector(3 downto 0);
       mRxDv : in std_logic;
       mRxEr : in std_logic;
       mTxClk : in std_logic;
       rTxDat : in std_logic_vector(1 downto 0);
       rTxEn : in std_logic;
       rst : in std_logic;
       mTxDat : out std_logic_vector(3 downto 0);
       mTxEn : out std_logic;
       rRxDat : out std_logic_vector(1 downto 0);
       rRxDv : out std_logic;
       rRxEr : out std_logic
  );
end component;

---- Architecture declarations -----
--constants for packet dma master
constant gen_tx_fifo_c : boolean := not useIntPktBuf_g;
constant gen_rx_fifo_c : boolean := not(useIntPktBuf_g and useRxIntPktBuf_g);
constant fifo_data_width_c : integer := m_data_width_g;
constant rx_fifo_word_size_c : integer := m_rx_fifo_size_g; --set  value power of 2
constant tx_fifo_word_size_c : integer := m_tx_fifo_size_g; --set value power of 2


----     Constants     -----
constant VCC_CONSTANT   : std_logic := '1';

---- Signal declarations used on the diagram ----

signal cmp_rd : std_logic;
signal cmp_rd_ack : std_logic;
signal cmp_wr : std_logic;
signal cmp_wr_ack : std_logic;
signal dmaErr_sel : std_logic;
signal dma_ack : std_logic;
signal dma_ack_rd_mst : std_logic;
signal dma_ack_read : std_logic;
signal dma_ack_rw : std_logic;
signal dma_ack_write : std_logic;
signal dma_rd_err : std_logic;
signal dma_req : std_logic;
signal dma_req_overflow : std_logic;
signal dma_req_read : std_logic;
signal dma_req_write : std_logic;
signal dma_rw : std_logic;
signal dma_wr_err : std_logic;
signal flt0_rx_dv : std_logic;
signal flt0_tx_en : std_logic;
signal flt1_rx_dv : std_logic;
signal flt1_tx_en : std_logic;
signal hub_intern_port : integer;
signal hub_rx_port : integer;
signal irqTable_sel : std_logic;
signal mac_rx_dv : std_logic;
signal mac_rx_irq_s : std_logic;
signal mac_rx_irq_s_n : std_logic;
signal mac_rx_off : std_logic;
signal mac_selcont : std_logic;
signal mac_selfilter : std_logic;
signal mac_selram : std_logic;
signal mac_tx_en : std_logic;
signal mac_tx_irq_s : std_logic;
signal mac_tx_irq_s_n : std_logic;
signal mac_tx_off : std_logic;
signal mac_write : std_logic;
signal mac_write_n : std_logic;
signal phy0_rx_dv_s : std_logic;
signal phy0_rx_err_s : std_logic;
signal phy0_tx_en_s : std_logic;
signal phy1_rx_dv_s : std_logic;
signal phy1_rx_err_s : std_logic;
signal phy1_tx_en_s : std_logic;
signal pkt_read_ack : std_logic;
signal pkt_write_ack : std_logic;
signal read_a : std_logic;
signal read_b : std_logic;
signal smi_clk : std_logic;
signal smi_di_s : std_logic;
signal smi_doe_s : std_logic;
signal smi_doe_s_n : std_logic;
signal smi_do_s : std_logic;
signal smi_rst_n : std_logic;
signal smi_sel : std_logic;
signal smi_write : std_logic;
signal smi_write_n : std_logic;
signal s_rd : std_logic;
signal s_rd_ack : std_logic;
signal s_wr : std_logic;
signal s_wr_ack : std_logic;
signal toggle : std_logic;
signal VCC : std_logic;
signal write_a : std_logic;
signal write_b : std_logic;
signal dma_addr : std_logic_vector (dma_highadr_g downto 1);
signal dma_addr_s : std_logic_vector (iPktBufSizeLog2_g-1 downto 1);
signal dma_be : std_logic_vector (1 downto 0);
signal dma_din : std_logic_vector (15 downto 0);
signal dma_din_mst : std_logic_vector (15 downto 0);
signal dma_din_s : std_logic_vector (15 downto 0);
signal dma_dout : std_logic_vector (15 downto 0);
signal dma_dout_s : std_logic_vector (15 downto 0);
signal flt0_rx_dat : std_logic_vector (1 downto 0);
signal flt0_tx_dat : std_logic_vector (1 downto 0);
signal flt1_rx_dat : std_logic_vector (1 downto 0);
signal flt1_tx_dat : std_logic_vector (1 downto 0);
signal hub_rx : std_logic_vector (1 downto 0);
signal hub_rx_dat0 : std_logic_vector (3 downto 1);
signal hub_rx_dat1 : std_logic_vector (3 downto 1);
signal hub_rx_dv : std_logic_vector (3 downto 1);
signal hub_tx_dat0 : std_logic_vector (3 downto 1);
signal hub_tx_dat1 : std_logic_vector (3 downto 1);
signal hub_tx_en : std_logic_vector (3 downto 1);
signal hub_tx_msk : std_logic_vector (3 downto 1);
signal irqTable : std_logic_vector (15 downto 0);
signal mac_addr : std_logic_vector (10 downto 1);
signal mac_be : std_logic_vector (1 downto 0);
signal mac_be_n : std_logic_vector (1 downto 0);
signal mac_din : std_logic_vector (15 downto 0);
signal mac_dout : std_logic_vector (15 downto 0);
signal mac_rx_dat : std_logic_vector (1 downto 0);
signal mac_time : std_logic_vector (31 downto 0);
signal mac_tx_dat : std_logic_vector (1 downto 0);
signal phy0_rx_dat_s : std_logic_vector (1 downto 0);
signal phy0_tx_dat_s : std_logic_vector (1 downto 0);
signal phy1_rx_dat_s : std_logic_vector (1 downto 0);
signal phy1_tx_dat_s : std_logic_vector (1 downto 0);
signal smi_addr : std_logic_vector (2 downto 0);
signal smi_be : std_logic_vector (1 downto 0);
signal smi_be_n : std_logic_vector (1 downto 0);
signal smi_din : std_logic_vector (15 downto 0);
signal smi_dout : std_logic_vector (15 downto 0);
signal s_address_s : std_logic_vector (s_address'length downto 0);

begin

---- User Signal Assignments ----
--assign address bus and be to openMAC
mac_addr <= 
	s_address(9 downto 1) & s_address(0) when mac_selfilter = '1' and endian_g = "little" else
	s_address(9 downto 1) & not s_address(0) when endian_g = "little" else
	s_address(9 downto 1) & s_address(0); -- when endian_g = "big" else

mac_be <= 
	s_byteenable(0) & s_byteenable(1) when endian_g = "little" else
	s_byteenable;
--convert word into byte addresses
s_address_s <= s_address & '0';
smi_addr <= s_address(2 downto 0);
smi_be <= s_byteenable;
--assign output data to readdata
s_readdata <=
	mac_dout(15 downto 8) & mac_dout(7 downto 0) when (mac_selram = '1' or mac_selcont = '1') and s_byteenable = "11" and endian_g = "little" else
	mac_dout(7 downto 0) & mac_dout(15 downto 8) when (mac_selram = '1' or mac_selcont = '1') and endian_g = "little" else
	mac_dout when (mac_selram = '1' or mac_selcont = '1') and endian_g = "big" else
	smi_dout when smi_sel = '1' else
	irqTable when irqTable_sel = '1' else
	(8 => dma_rd_err, 0 => dma_wr_err, others => '0') when dmaErr_sel = '1' else
	(others => '0');
--assign writedata to input data
mac_din <=
	s_writedata(15 downto 8) & s_writedata(7 downto 0) when s_byteenable = "11" and endian_g = "little" else
	s_writedata(7 downto 0) & s_writedata(15 downto 8) when endian_g = "little" else
	s_writedata; -- when endian_g = "big" else

smi_din <= s_writedata;

----  Component instantiations  ----

THE_MAC_TIME_CMP : openMAC_cmp
  generic map (
       gen2ndCmpTimer_g => gen2ndCmpTimer_g,
       mac_time_width_g => 32
  )
  port map(
       addr => t_address,
       clk => clk,
       din => t_writedata,
       dout => t_readdata,
       irq => t_irq,
       mac_time => mac_time( 31 downto 0 ),
       rst => rst,
       toggle => toggle,
       wr => cmp_wr
  );

THE_OPENMAC : OpenMAC
  generic map (
       HighAdr => dma_highadr_g,
       Simulate => simulate,
       Timer => true,
       TxDel => true,
       TxSyncOn => true
  )
  port map(
       Clk => clk,
       Dma_Ack => dma_ack,
       Dma_Addr => dma_addr( dma_highadr_g downto 1 ),
       Dma_Din => dma_din,
       Dma_Dout => dma_dout,
       Dma_Rd_Done => mac_tx_off,
       Dma_Req => dma_req,
       Dma_Req_Overflow => dma_req_overflow,
       Dma_Rw => dma_rw,
       Dma_Wr_Done => mac_rx_off,
       Hub_Rx => hub_rx,
       Mac_Zeit => mac_time,
       Rst => rst,
       S_Adr => mac_addr,
       S_Din => mac_din,
       S_Dout => mac_dout,
       S_nBe => mac_be_n,
       Sel_Cont => mac_selcont,
       Sel_Ram => mac_selram,
       nRx_Int => mac_rx_irq_s_n,
       nTx_Int => mac_tx_irq_s_n,
       rCrs_Dv => mac_rx_dv,
       rRx_Dat => mac_rx_dat,
       rTx_Dat => mac_tx_dat,
       rTx_En => mac_tx_en,
       s_nWr => mac_write_n
  );

THE_PHY_MGMT : OpenMAC_MII
  port map(
       Addr => smi_addr,
       Clk => clk,
       Data_In => smi_din,
       Data_Out => smi_dout,
       Mii_Clk => smi_clk,
       Mii_Di => smi_di_s,
       Mii_Do => smi_do_s,
       Mii_Doe => smi_doe_s_n,
       Rst => rst,
       Sel => smi_sel,
       nBe => smi_be_n,
       nResetOut => smi_rst_n,
       nWr => smi_write_n
  );

mac_rx_irq_s <= not(mac_rx_irq_s_n);

s_irq <= mac_tx_irq_s or mac_rx_irq_s;

mac_write_n <= not(mac_write);

mac_be_n(1) <= not(mac_be(1));
mac_be_n(0) <= not(mac_be(0));

smi_doe_s <= not(smi_doe_s_n);

smi_write_n <= not(smi_write);

smi_be_n(1) <= not(smi_be(1));
smi_be_n(0) <= not(smi_be(0));

s_wr <= s_write and s_chipselect;

irqTable(0) <= mac_tx_irq_s;

irqTable(1) <= mac_rx_irq_s;

mac_write <= s_write;

smi_write <= s_write;

cmp_wr <= t_write and t_chipselect;

dma_req_write <= not(dma_rw) and dma_req;

dma_ack <= dma_ack_write or dma_ack_read;

s_rd <= s_read and s_chipselect;

dma_req_read <= dma_rw and dma_req;

t_waitrequest <= not(cmp_wr_ack or cmp_rd_ack);

cmp_rd <= t_read and t_chipselect;

s_waitrequest <= not(s_rd_ack or s_wr_ack);

mac_tx_irq_s <= not(mac_tx_irq_s_n);

addrdec0 : addr_decoder
  generic map (
       addrWidth_g => s_address'length+1,
       baseaddr_g => 16#0000#,
       highaddr_g => 16#03FF#
  )
  port map(
       addr => s_address_s( s_address'length downto 0 ),
       selin => s_chipselect,
       selout => mac_selcont
  );

addrdec1 : addr_decoder
  generic map (
       addrWidth_g => s_address'length+1,
       baseaddr_g => 16#0800#,
       highaddr_g => 16#0FFF#
  )
  port map(
       addr => s_address_s( s_address'length downto 0 ),
       selin => s_chipselect,
       selout => mac_selram
  );

addrdec2 : addr_decoder
  generic map (
       addrWidth_g => s_address'length+1,
       baseaddr_g => 16#0800#,
       highaddr_g => 16#0BFF#
  )
  port map(
       addr => s_address_s( s_address'length downto 0 ),
       selin => s_chipselect,
       selout => mac_selfilter
  );

addrdec3 : addr_decoder
  generic map (
       addrWidth_g => s_address'length+1,
       baseaddr_g => 16#1000#,
       highaddr_g => 16#100F#
  )
  port map(
       addr => s_address_s( s_address'length downto 0 ),
       selin => s_chipselect,
       selout => smi_sel
  );

addrdec4 : addr_decoder
  generic map (
       addrWidth_g => s_address'length+1,
       baseaddr_g => 16#1010#,
       highaddr_g => 16#101F#
  )
  port map(
       addr => s_address_s( s_address'length downto 0 ),
       selin => s_chipselect,
       selout => irqTable_sel
  );

addrdec5 : addr_decoder
  generic map (
       addrWidth_g => s_address'length+1,
       baseaddr_g => 16#1020#,
       highaddr_g => 16#102F#
  )
  port map(
       addr => s_address_s( s_address'length downto 0 ),
       selin => s_chipselect,
       selout => dmaErr_sel
  );

regack0 : req_ack
  generic map (
       ack_delay_g => 1,
       zero_delay_g => true
  )
  port map(
       ack => s_wr_ack,
       clk => clk,
       enable => s_wr,
       rst => rst
  );

regack1 : req_ack
  generic map (
       ack_delay_g => 1,
       zero_delay_g => false
  )
  port map(
       ack => s_rd_ack,
       clk => clk,
       enable => s_rd,
       rst => rst
  );

regack2 : req_ack
  generic map (
       ack_delay_g => 1,
       zero_delay_g => false
  )
  port map(
       ack => cmp_rd_ack,
       clk => clk,
       enable => cmp_rd,
       rst => rst
  );

regack3 : req_ack
  generic map (
       ack_delay_g => 1,
       zero_delay_g => true
  )
  port map(
       ack => cmp_wr_ack,
       clk => clk,
       enable => cmp_wr,
       rst => rst
  );


---- Power , ground assignment ----

VCC <= VCC_CONSTANT;
dma_be(1) <= VCC;
dma_be(0) <= VCC;

---- Terminal assignment ----

    -- Output\buffer terminals
	mac_rx_irq <= mac_rx_irq_s;
	mac_tx_irq <= mac_tx_irq_s;
	t_tog <= toggle;


----  Generate statements  ----

genPhyActLed : if genPhyActLed_g generate
begin
  THE_PHY_ACT : OpenMAC_phyAct
    generic map (
         iBlinkFreq_g => 6
    )  
    port map(
         act_led => act_led,
         clk => clk,
         rst => rst,
         rx_dv => mac_rx_dv,
         tx_en => mac_tx_en
    );
end generate genPhyActLed;

genHub : if genHub_g generate
begin
  THE_OPENFILTER0 : openFILTER
    generic map (
         bypassFilter => not useRmii_g
    )  
    port map(
         Clk => clk,
         Rst => rst,
         RxDatIn => phy0_rx_dat_s,
         RxDatOut => flt0_rx_dat,
         RxDvIn => phy0_rx_dv_s,
         RxDvOut => flt0_rx_dv,
         RxErr => phy0_rx_err_s,
         TxDatIn => flt0_tx_dat,
         TxDatOut => phy0_tx_dat_s,
         TxEnIn => flt0_tx_en,
         TxEnOut => phy0_tx_en_s,
         nCheckShortFrames => VCC
    );
  
  THE_OPENFILTER1 : openFILTER
    generic map (
         bypassFilter => not useRmii_g
    )  
    port map(
         Clk => clk,
         Rst => rst,
         RxDatIn => phy1_rx_dat_s,
         RxDatOut => flt1_rx_dat,
         RxDvIn => phy1_rx_dv_s,
         RxDvOut => flt1_rx_dv,
         RxErr => phy1_rx_err_s,
         TxDatIn => flt1_tx_dat,
         TxDatOut => phy1_tx_dat_s,
         TxEnIn => flt1_tx_en,
         TxEnOut => phy1_tx_en_s,
         nCheckShortFrames => VCC
    );
  
  THE_OPENHUB : OpenHUB
    generic map (
         Ports => 3
    )  
    port map(
         Clk => clk,
         ReceivePort => hub_rx_port,
         Rst => rst,
         RxDat0 => hub_rx_dat0( 3 downto 1 ),
         RxDat1 => hub_rx_dat1( 3 downto 1 ),
         RxDv => hub_rx_dv( 3 downto 1 ),
         TransmitMask => hub_tx_msk( 3 downto 1 ),
         TxDat0 => hub_tx_dat0( 3 downto 1 ),
         TxDat1 => hub_tx_dat1( 3 downto 1 ),
         TxEn => hub_tx_en( 3 downto 1 ),
         internPort => hub_intern_port
    );

  --mac tx to hub rx
hub_rx_dv(1) <= mac_tx_en;
hub_rx_dat0(1) <= mac_tx_dat(0);
hub_rx_dat1(1) <= mac_tx_dat(1);

--hub tx to mac rx
mac_rx_dv <= hub_tx_en(1);
mac_rx_dat(0) <= hub_tx_dat0(1);
mac_rx_dat(1) <= hub_tx_dat1(1);
  --filter 0 to hub rx
hub_rx_dv(2) <= flt0_rx_dv;
hub_rx_dat0(2) <= flt0_rx_dat(0);
hub_rx_dat1(2) <= flt0_rx_dat(1);

--hub tx to filter 0
flt0_tx_en <= hub_tx_en(2);
flt0_tx_dat(0) <= hub_tx_dat0(2);
flt0_tx_dat(1) <= hub_tx_dat1(2);
  --filter 1 to hub rx
hub_rx_dv(3) <= flt1_rx_dv;
hub_rx_dat0(3) <= flt1_rx_dat(0);
hub_rx_dat1(3) <= flt1_rx_dat(1);

--hub tx to filter 1
flt1_tx_en <= hub_tx_en(3);
flt1_tx_dat(0) <= hub_tx_dat0(3);
flt1_tx_dat(1) <= hub_tx_dat1(3);
  --convert to std_logic_vector
hub_rx <= conv_std_logic_vector(hub_rx_port,hub_rx'length);

--set intern port
hub_intern_port <= 1;

--set tx mask
hub_tx_msk <= (others => '1');
end generate genHub;

genRmii2Mii0 : if not useRmii_g generate
begin
  THE_MII2RMII0 : rmii2mii
    port map(
         clk50 => clk,
         mRxClk => phyMii0_rx_clk,
         mRxDat => phyMii0_rx_dat,
         mRxDv => phyMii0_rx_dv,
         mRxEr => phyMii0_rx_err,
         mTxClk => phyMii0_tx_clk,
         mTxDat => phyMii0_tx_dat,
         mTxEn => phyMii0_tx_en,
         rRxDat => phy0_rx_dat_s,
         rRxDv => phy0_rx_dv_s,
         rRxEr => phy0_rx_err_s,
         rTxDat => phy0_tx_dat_s,
         rTxEn => phy0_tx_en_s,
         rst => rst
    );
end generate genRmii2Mii0;

genRmii2Mii1 : if not useRmii_g and genHub_g generate
begin
  THE_MII2RMII1 : rmii2mii
    port map(
         clk50 => clk,
         mRxClk => phyMii1_rx_clk,
         mRxDat => phyMii1_rx_dat,
         mRxDv => phyMii1_rx_dv,
         mRxEr => phyMii1_rx_err,
         mTxClk => phyMii1_tx_clk,
         mTxDat => phyMii1_tx_dat,
         mTxEn => phyMii1_tx_en,
         rRxDat => phy1_rx_dat_s,
         rRxDv => phy1_rx_dv_s,
         rRxEr => phy1_rx_err_s,
         rTxDat => phy1_tx_dat_s,
         rTxEn => phy1_tx_en_s,
         rst => rst
    );
end generate genRmii2Mii1;

genRmii100MegFFs : if useRmii_g generate
begin
  latchRxSignals :
  process (clk, rst)
  -- Section above this comment may be overwritten according to
  -- "Update sensitivity list automatically" option status
  begin
  if rst = '1' then
  	phy0_rx_dv_s <= '0';
  	phy0_rx_err_s <= '0';
  	phy0_rx_dat_s <= (others => '0');
  	phy1_rx_dv_s <= '0';
  	phy1_rx_err_s <= '0';
  	phy1_rx_dat_s <= (others => '0');
  elsif clk = '1' and clk'event then
  	phy0_rx_dv_s <= phy0_rx_dv;
  	phy0_rx_err_s <= phy0_rx_err;
  	phy0_rx_dat_s <= phy0_rx_dat;
  	phy1_rx_dv_s <= phy1_rx_dv;
  	phy1_rx_err_s <= phy1_rx_err;
  	phy1_rx_dat_s <= phy1_rx_dat;
  end if;
  end process;

  latchTxSignals :
  process (clkx2, rst)
  -- Section above this comment may be overwritten according to
  -- "Update sensitivity list automatically" option status
  begin
  if rst = '1' then
  	phy0_tx_en <= '0';
  	phy0_tx_dat <= (others => '0');
  	phy1_tx_en <= '0';
  	phy1_tx_dat <= (others => '0');
  elsif clkx2 = '0' and clkx2'event then
  	phy0_tx_en <= phy0_tx_en_s;
  	phy0_tx_dat <= phy0_tx_dat_s;
  	phy1_tx_en <= phy1_tx_en_s;
  	phy1_tx_dat <= phy1_tx_dat_s;
  end if;
  end process;
end generate genRmii100MegFFs;

genOneFilter : if genHub_g = false generate
begin
  THE_OPENFILTER : openFILTER
    generic map (
         bypassFilter => not useRmii_g
    )  
    port map(
         Clk => clk,
         Rst => rst,
         RxDatIn => phy0_rx_dat_s,
         RxDatOut => mac_rx_dat,
         RxDvIn => phy0_rx_dv_s,
         RxDvOut => mac_rx_dv,
         RxErr => phy0_rx_err_s,
         TxDatIn => mac_tx_dat,
         TxDatOut => phy0_tx_dat_s,
         TxEnIn => mac_tx_en,
         TxEnOut => phy0_tx_en_s,
         nCheckShortFrames => VCC
    );
end generate genOneFilter;

genPktBuf : if useIntPktBuf_g = TRUE generate
begin
  g5 : if useRxIntPktBuf_g = TRUE generate
  begin
    dma_ack_write <= dma_ack_rw;
  end generate g5;

  THE_MAC_PKT_BUF : OpenMAC_DPRpackets
    generic map (
         memSizeLOG2_g => iPktBufSizeLog2_g,
         memSize_g => iPktBufSize_g
    )  
    port map(
         address_a => dma_addr_s( iPktBufSizeLog2_g-1 downto 1 ),
         address_b => pkt_address( iPktBufSizeLog2_g-3 downto 0 ),
         byteena_a => dma_be,
         byteena_b => pkt_byteenable,
         clock_a => clk,
         clock_b => pkt_clk,
         data_a => dma_dout_s,
         data_b => pkt_writedata,
         q_a => dma_din_s,
         q_b => pkt_readdata,
         rden_a => read_a,
         rden_b => read_b,
         wren_a => write_a,
         wren_b => write_b
    );
  
  read_b <= pkt_read and pkt_chipselect;
  
  write_b <= pkt_write and pkt_chipselect;
  
  read_a <= dma_req_read;
  
  dma_ack_read <= dma_ack_rw;
  
  pkt_waitrequest <= not(pkt_write_ack or pkt_read_ack);
  
  regack4 : req_ack
    generic map (
         ack_delay_g => 1,
         zero_delay_g => true
    )  
    port map(
         ack => pkt_write_ack,
         clk => pkt_clk,
         enable => write_b,
         rst => rst
    );
  
  regack5 : req_ack
    generic map (
         ack_delay_g => 2,
         zero_delay_g => false
    )  
    port map(
         ack => pkt_read_ack,
         clk => pkt_clk,
         enable => read_b,
         rst => rst
    );

  --endian conversion
dma_dout_s <= 
	dma_dout(7 downto 0) & dma_dout(15 downto 8) when endian_g = "little" else
	dma_dout;
dma_din <= 
	dma_din_s(7 downto 0) & dma_din_s(15 downto 8) when endian_g = "little" else
	dma_din_s;
dma_addr_s(iPktBufSizeLog2_g-1 downto 1) <=
	dma_addr(iPktBufSizeLog2_g-1 downto 2) & dma_addr(1) when endian_g = "little" else
	dma_addr(iPktBufSizeLog2_g-1 downto 2) & not dma_addr(1);
  --write DPR from port A only if RX data is written to DPR
write_a <= dma_req_write when useRxIntPktBuf_g = TRUE else '0';

  genAck :
  process (clk, rst, dma_ack_rw)
  -- Section above this comment may be overwritten according to
  -- "Update sensitivity list automatically" option status
  -- declarations
  begin
  	if rst = '1' then
  		dma_ack_rw <= '0';
  	elsif clk = '1' and clk'event then
  		if dma_req = '1' and dma_ack_rw = '0' then
  			dma_ack_rw <= '1';
  		else
  			dma_ack_rw <= '0';
  		end if;
  	end if;
  end process;
end generate genPktBuf;

genDmaMaster : if not useIntPktBuf_g or (useIntPktBuf_g and not useRxIntPktBuf_g) generate
begin
  genReadDmaMaster : if not useIntPktBuf_g generate
  begin
    dma_ack_read <= dma_ack_rd_mst;
    
        U69_array: for U69_array_index in 0 to (dma_din'length - 1) generate
    	U69_array :
    		dma_din(U69_array_index+dma_din'Low) <= dma_din_mst(U69_array_index+dma_din_mst'Low);
    end generate;
  end generate genReadDmaMaster;

  THE_MAC_DMA_MASTER : openMAC_DMAmaster
    generic map (
         dma_highadr_g => dma_highadr_g,
         endian_g => endian_g,
         fifo_data_width_g => fifo_data_width_c,
         gen_dma_observer_g => gen_dma_observer_g,
         gen_rx_fifo_g => gen_rx_fifo_c,
         gen_tx_fifo_g => gen_tx_fifo_c,
         m_burstcount_const_g => m_burstcount_const_g,
         m_burstcount_width_g => m_burstcount'length,
         m_rx_burst_size_g => m_rx_burst_size_g,
         m_tx_burst_size_g => m_tx_burst_size_g,
         rx_fifo_word_size_g => rx_fifo_word_size_c,
         simulate => simulate,
         tx_fifo_word_size_g => tx_fifo_word_size_c
    )  
    port map(
         dma_ack_rd => dma_ack_rd_mst,
         dma_ack_wr => dma_ack_write,
         dma_addr => dma_addr( dma_highadr_g downto 1 ),
         dma_clk => clk,
         dma_din => dma_din_mst,
         dma_dout => dma_dout,
         dma_rd_err => dma_rd_err,
         dma_req_overflow => dma_req_overflow,
         dma_req_rd => dma_req_read,
         dma_req_wr => dma_req_write,
         dma_wr_err => dma_wr_err,
         m_address => m_address( 29 downto 0 ),
         m_burstcount => m_burstcount( m_burstcount_width_g-1 downto 0 ),
         m_burstcounter => m_burstcounter( m_burstcount_width_g-1 downto 0 ),
         m_byteenable => m_byteenable( m_data_width_g/8-1 downto 0 ),
         m_clk => m_clk,
         m_read => m_read,
         m_readdata => m_readdata( m_data_width_g-1 downto 0 ),
         m_readdatavalid => m_readdatavalid,
         m_waitrequest => m_waitrequest,
         m_write => m_write,
         m_writedata => m_writedata( m_data_width_g-1 downto 0 ),
         mac_rx_off => mac_rx_off,
         mac_tx_off => mac_tx_off,
         rst => rst
    );
end generate genDmaMaster;

genOneSmi : if gNumSmi = 1 or not genHub_g generate
begin
  genOneTriStateBuf : if genSmiIO generate
  begin
    smi_di_s <= phy_smi_dio;
    
    phy_smi_dio <= smi_do_s when smi_doe_s='1' else 'Z';
  end generate genOneTriStateBuf;

  dontGenOneTriStateBuf : if not genSmiIO generate
  begin
    smi_di_s <= phy_smi_dio_I;
    
    phy_smi_dio_O <= smi_do_s;
    
    phy_smi_dio_T <= smi_doe_s_n;
  end generate dontGenOneTriStateBuf;

  phy_rst_n <= smi_rst_n;
  
  phy_smi_clk <= smi_clk;
end generate genOneSmi;

genTwoSmi : if gNumSmi = 2 and genHub_g generate
begin
  genTwoTriStateBuf : if genSmiIO generate
  begin
    phy0_smi_dio <= smi_do_s when smi_doe_s='1' else 'Z';
    
    phy1_smi_dio <= smi_do_s when smi_doe_s='1' else 'Z';
    
    smi_di_s <= phy0_smi_dio and phy1_smi_dio;
  end generate genTwoTriStateBuf;

  dontGenTwoTriStateBuf : if not genSmiIO generate
  begin
    phy1_smi_dio_T <= smi_doe_s_n;
    
    smi_di_s <= phy0_smi_dio_I and phy1_smi_dio_I;
    
    phy0_smi_dio_T <= smi_doe_s_n;
    
    phy1_smi_dio_O <= smi_do_s;
    
    phy0_smi_dio_O <= smi_do_s;
  end generate dontGenTwoTriStateBuf;

  phy0_smi_clk <= smi_clk;
  
  phy0_rst_n <= smi_rst_n;
  
  phy1_smi_clk <= smi_clk;
  
  phy1_rst_n <= smi_rst_n;
end generate genTwoSmi;

end rtl;
