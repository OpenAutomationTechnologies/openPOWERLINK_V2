-------------------------------------------------------------------------------
--! @file axi_powerlink.vhd
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
--
-- This is the toplevel file for using the POWERLINK IP-Core
-- with Xilinx AXI.
--
-------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;
use ieee.math_real.log2;
use ieee.math_real.ceil;
use work.global.all;

library proc_common_v3_00_a;
use proc_common_v3_00_a.proc_common_pkg.all;
use proc_common_v3_00_a.ipif_pkg.all;

library axi_lite_ipif_v1_01_a;
use axi_lite_ipif_v1_01_a.axi_lite_ipif;

library axi_master_burst_v1_00_a;
use axi_master_burst_v1_00_a.axi_master_burst;

-- standard libraries declarations
library UNISIM;
use UNISIM.vcomponents.all;
-- pragma synthesis_off
library IEEE;
use IEEE.vital_timing.all;
-- pragma synthesis_on

-- other libraries declarations
library AXI_LITE_IPIF_V1_01_A;
library AXI_MASTER_BURST_V1_00_A;

entity axi_powerlink is
  generic(
       C_FAMILY : string := "spartan6";
       -- general
       C_GEN_PDI : boolean := false;
       C_GEN_PAR_IF : boolean := false;
       C_GEN_SPI_IF : boolean := false;
       C_GEN_AXI_BUS_IF : boolean := false;
       C_GEN_SIMPLE_IO : boolean := false;
       -- openMAC
       C_MAC_PKT_SIZE : integer := 1024;
       C_MAC_PKT_SIZE_LOG2 : integer := 10;
       C_MAC_RX_BUFFERS : integer := 16;
       C_USE_RMII : boolean := false;
       C_TX_INT_PKT : boolean := false;
       C_RX_INT_PKT : boolean := false;
       C_USE_2ND_PHY : boolean := true;
       C_NUM_SMI : integer range 1 to 2 := 2;
       C_MAC_GEN_SECOND_TIMER : boolean := false;
       --pdi
       C_PDI_REV : integer := 0;
       C_PCP_SYS_ID : integer := 0;
       C_PDI_GEN_ASYNC_BUF_0 : boolean := true;
       C_PDI_ASYNC_BUF_0 : integer := 50;
       C_PDI_GEN_ASYNC_BUF_1 : boolean := true;
       C_PDI_ASYNC_BUF_1 : integer := 50;
       C_PDI_GEN_LED : boolean := false;
       C_PDI_GEN_TIME_SYNC : boolean := true;
       C_PDI_GEN_EVENT : boolean := true;
       --global pdi and mac
       C_NUM_RPDO : integer := 3;
       C_RPDO_0_BUF_SIZE : integer := 100;
       C_RPDO_1_BUF_SIZE : integer := 100;
       C_RPDO_2_BUF_SIZE : integer := 100;
       C_NUM_TPDO : integer := 1;
       C_TPDO_BUF_SIZE : integer := 100;
       -- pap
       C_PAP_DATA_WIDTH : integer := 16;
       --C_PAP_BIG_END : boolean := false;
       C_PAP_LOW_ACT : boolean := false;
       -- spi
       C_SPI_CPOL : boolean := false;
       C_SPI_CPHA : boolean := false;
       --C_SPI_BIG_END : boolean := false;
       -- simpleIO
       C_PIO_VAL_LENGTH : integer := 50;
       -- debug
       C_OBSERVER_ENABLE : boolean := false;
       -- clock stabiliser
       C_INSTANCE_ODDR2 : boolean := false;
       -- sync IRQ pulse width
       C_USE_PULSE_2nd_CMP_TIMER : boolean := true;
       C_PULSE_WIDTH_2nd_CMP_TIMER : integer := 9;
       -- PDI AP AXI Slave
       C_S_AXI_PDI_AP_BASEADDR : std_logic_vector := X"00000000";
       C_S_AXI_PDI_AP_HIGHADDR : std_logic_vector := X"000FFFFF";
       C_S_AXI_PDI_AP_DATA_WIDTH : integer := 32;
       C_S_AXI_PDI_AP_ADDR_WIDTH : integer := 32;
       C_S_AXI_PDI_AP_USE_WSTRB : integer := 1;
       C_S_AXI_PDI_AP_DPHASE_TIMEOUT : integer := 8;
       -- PDI AP AXI Slave
       C_S_AXI_SMP_PCP_BASEADDR : std_logic_vector := X"00000000";
       C_S_AXI_SMP_PCP_HIGHADDR : std_logic_vector := X"000FFFFF";
       C_S_AXI_SMP_PCP_DATA_WIDTH : integer := 32;
       C_S_AXI_SMP_PCP_ADDR_WIDTH : integer := 32;
       C_S_AXI_SMP_PCP_USE_WSTRB : integer := 1;
       C_S_AXI_SMP_PCP_DPHASE_TIMEOUT : integer := 8;
       -- PDI PCP AXI Slave
       C_S_AXI_PDI_PCP_BASEADDR : std_logic_vector := X"00000000";
       C_S_AXI_PDI_PCP_HIGHADDR : std_logic_vector := X"000FFFFF";
       C_S_AXI_PDI_PCP_DATA_WIDTH : integer := 32;
       C_S_AXI_PDI_PCP_ADDR_WIDTH : integer := 32;
       C_S_AXI_PDI_PCP_USE_WSTRB : integer := 1;
       C_S_AXI_PDI_PCP_DPHASE_TIMEOUT : integer := 8;
       -- openMAC DMA AXI Master
       C_M_AXI_MAC_DMA_ADDR_WIDTH : INTEGER := 32;
       C_M_AXI_MAC_DMA_DATA_WIDTH : INTEGER := 32;
       C_M_AXI_MAC_DMA_NATIVE_DWIDTH : INTEGER := 32;
       C_M_AXI_MAC_DMA_LENGTH_WIDTH : INTEGER := 12;
       C_M_AXI_MAC_DMA_MAX_BURST_LEN : INTEGER := 16;
       C_MAC_DMA_BURST_SIZE_RX : INTEGER := 8; --in bytes
       C_MAC_DMA_BURST_SIZE_TX : INTEGER := 8; --in bytes
       C_MAC_DMA_FIFO_SIZE_RX : INTEGER := 32; --in bytes
       C_MAC_DMA_FIFO_SIZE_TX : INTEGER := 32; --in bytes
       -- openMAC PKT AXI Slave
       C_S_AXI_MAC_PKT_BASEADDR : std_logic_vector := X"00000000";
       C_S_AXI_MAC_PKT_HIGHADDR : std_logic_vector := X"000FFFFF";
       C_S_AXI_MAC_PKT_DATA_WIDTH : integer := 32;
       C_S_AXI_MAC_PKT_ADDR_WIDTH : integer := 32;
       C_S_AXI_MAC_PKT_USE_WSTRB : integer := 1;
       C_S_AXI_MAC_PKT_DPHASE_TIMEOUT : integer := 8;
       -- openMAC REG AXI Slave
       --- MAC_REG
       C_S_AXI_MAC_REG_RNG0_BASEADDR : std_logic_vector := X"00000000";
       C_S_AXI_MAC_REG_RNG0_HIGHADDR : std_logic_vector := X"0000FFFF";
       --- MAC_CMP
       C_S_AXI_MAC_REG_RNG1_BASEADDR : std_logic_vector := X"00000000";
       C_S_AXI_MAC_REG_RNG1_HIGHADDR : std_logic_vector := X"0000FFFF";
       C_S_AXI_MAC_REG_DATA_WIDTH : integer := 32;
       C_S_AXI_MAC_REG_ADDR_WIDTH : integer := 32;
       C_S_AXI_MAC_REG_USE_WSTRB : integer := 1;
       C_S_AXI_MAC_REG_DPHASE_TIMEOUT : integer := 8;
       C_S_AXI_MAC_REG_ACLK_FREQ_HZ : integer := 20 --clock frequency in Hz
  );
  port(
       M_AXI_MAC_DMA_aclk : in std_logic;
       M_AXI_MAC_DMA_aresetn : in std_logic;
       M_AXI_MAC_DMA_arready : in std_logic;
       M_AXI_MAC_DMA_awready : in std_logic;
       M_AXI_MAC_DMA_bvalid : in std_logic;
       M_AXI_MAC_DMA_rlast : in std_logic;
       M_AXI_MAC_DMA_rvalid : in std_logic;
       M_AXI_MAC_DMA_wready : in std_logic;
       S_AXI_MAC_PKT_ACLK : in std_logic;
       S_AXI_MAC_PKT_ARESETN : in std_logic;
       S_AXI_MAC_PKT_ARVALID : in std_logic;
       S_AXI_MAC_PKT_AWVALID : in std_logic;
       S_AXI_MAC_PKT_BREADY : in std_logic;
       S_AXI_MAC_PKT_RREADY : in std_logic;
       S_AXI_MAC_PKT_WVALID : in std_logic;
       S_AXI_MAC_REG_ACLK : in std_logic;
       S_AXI_MAC_REG_ARESETN : in std_logic;
       S_AXI_MAC_REG_ARVALID : in std_logic;
       S_AXI_MAC_REG_AWVALID : in std_logic;
       S_AXI_MAC_REG_BREADY : in std_logic;
       S_AXI_MAC_REG_RREADY : in std_logic;
       S_AXI_MAC_REG_WVALID : in std_logic;
       S_AXI_PDI_AP_ACLK : in std_logic;
       S_AXI_PDI_AP_ARESETN : in std_logic;
       S_AXI_PDI_AP_ARVALID : in std_logic;
       S_AXI_PDI_AP_AWVALID : in std_logic;
       S_AXI_PDI_AP_BREADY : in std_logic;
       S_AXI_PDI_AP_RREADY : in std_logic;
       S_AXI_PDI_AP_WVALID : in std_logic;
       S_AXI_PDI_PCP_ACLK : in std_logic;
       S_AXI_PDI_PCP_ARESETN : in std_logic;
       S_AXI_PDI_PCP_ARVALID : in std_logic;
       S_AXI_PDI_PCP_AWVALID : in std_logic;
       S_AXI_PDI_PCP_BREADY : in std_logic;
       S_AXI_PDI_PCP_RREADY : in std_logic;
       S_AXI_PDI_PCP_WVALID : in std_logic;
       S_AXI_SMP_PCP_ACLK : in std_logic;
       S_AXI_SMP_PCP_ARESETN : in std_logic;
       S_AXI_SMP_PCP_ARVALID : in std_logic;
       S_AXI_SMP_PCP_AWVALID : in std_logic;
       S_AXI_SMP_PCP_BREADY : in std_logic;
       S_AXI_SMP_PCP_RREADY : in std_logic;
       S_AXI_SMP_PCP_WVALID : in std_logic;
       clk100 : in std_logic;
       clk50 : in std_logic;
       pap_cs : in std_logic;
       pap_cs_n : in std_logic;
       pap_rd : in std_logic;
       pap_rd_n : in std_logic;
       pap_wr : in std_logic;
       pap_wr_n : in std_logic;
       phy0_RxDv : in std_logic;
       phy0_RxErr : in std_logic;
       phy0_SMIDat_I : in std_logic;
       phy0_link : in std_logic;
       phy1_RxDv : in std_logic;
       phy1_RxErr : in std_logic;
       phy1_SMIDat_I : in std_logic;
       phy1_link : in std_logic;
       phyMii0_RxClk : in std_logic;
       phyMii0_RxDv : in std_logic;
       phyMii0_RxEr : in std_logic;
       phyMii0_TxClk : in std_logic;
       phyMii1_RxClk : in std_logic;
       phyMii1_RxDv : in std_logic;
       phyMii1_RxEr : in std_logic;
       phyMii1_TxClk : in std_logic;
       phy_SMIDat_I : in std_logic;
       spi_clk : in std_logic;
       spi_mosi : in std_logic;
       spi_sel_n : in std_logic;
       M_AXI_MAC_DMA_bresp : in std_logic_vector(1 downto 0);
       M_AXI_MAC_DMA_rdata : in std_logic_vector(C_M_AXI_MAC_DMA_DATA_WIDTH-1 downto 0);
       M_AXI_MAC_DMA_rresp : in std_logic_vector(1 downto 0);
       S_AXI_MAC_PKT_ARADDR : in std_logic_vector(C_S_AXI_MAC_PKT_ADDR_WIDTH-1 downto 0);
       S_AXI_MAC_PKT_AWADDR : in std_logic_vector(C_S_AXI_MAC_PKT_ADDR_WIDTH-1 downto 0);
       S_AXI_MAC_PKT_WDATA : in std_logic_vector(C_S_AXI_MAC_PKT_DATA_WIDTH-1 downto 0);
       S_AXI_MAC_PKT_WSTRB : in std_logic_vector((C_S_AXI_MAC_PKT_DATA_WIDTH/8)-1 downto 0);
       S_AXI_MAC_REG_ARADDR : in std_logic_vector(C_S_AXI_MAC_REG_ADDR_WIDTH-1 downto 0);
       S_AXI_MAC_REG_AWADDR : in std_logic_vector(C_S_AXI_MAC_REG_ADDR_WIDTH-1 downto 0);
       S_AXI_MAC_REG_WDATA : in std_logic_vector(C_S_AXI_MAC_REG_DATA_WIDTH-1 downto 0);
       S_AXI_MAC_REG_WSTRB : in std_logic_vector((C_S_AXI_MAC_REG_DATA_WIDTH/8)-1 downto 0);
       S_AXI_PDI_AP_ARADDR : in std_logic_vector(C_S_AXI_PDI_AP_ADDR_WIDTH-1 downto 0);
       S_AXI_PDI_AP_AWADDR : in std_logic_vector(C_S_AXI_PDI_AP_ADDR_WIDTH-1 downto 0);
       S_AXI_PDI_AP_WDATA : in std_logic_vector(C_S_AXI_PDI_AP_DATA_WIDTH-1 downto 0);
       S_AXI_PDI_AP_WSTRB : in std_logic_vector((C_S_AXI_PDI_AP_DATA_WIDTH/8)-1 downto 0);
       S_AXI_PDI_PCP_ARADDR : in std_logic_vector(C_S_AXI_PDI_PCP_ADDR_WIDTH-1 downto 0);
       S_AXI_PDI_PCP_AWADDR : in std_logic_vector(C_S_AXI_PDI_PCP_ADDR_WIDTH-1 downto 0);
       S_AXI_PDI_PCP_WDATA : in std_logic_vector(C_S_AXI_PDI_PCP_DATA_WIDTH-1 downto 0);
       S_AXI_PDI_PCP_WSTRB : in std_logic_vector((C_S_AXI_PDI_PCP_DATA_WIDTH/8)-1 downto 0);
       S_AXI_SMP_PCP_ARADDR : in std_logic_vector(C_S_AXI_SMP_PCP_ADDR_WIDTH-1 downto 0);
       S_AXI_SMP_PCP_AWADDR : in std_logic_vector(C_S_AXI_SMP_PCP_ADDR_WIDTH-1 downto 0);
       S_AXI_SMP_PCP_WDATA : in std_logic_vector(C_S_AXI_SMP_PCP_DATA_WIDTH-1 downto 0);
       S_AXI_SMP_PCP_WSTRB : in std_logic_vector((C_S_AXI_SMP_PCP_DATA_WIDTH/8)-1 downto 0);
       pap_addr : in std_logic_vector(15 downto 0);
       pap_be : in std_logic_vector(C_PAP_DATA_WIDTH/8-1 downto 0);
       pap_be_n : in std_logic_vector(C_PAP_DATA_WIDTH/8-1 downto 0);
       pap_data_I : in std_logic_vector(C_PAP_DATA_WIDTH-1 downto 0);
       pap_gpio_I : in std_logic_vector(1 downto 0);
       phy0_RxDat : in std_logic_vector(1 downto 0);
       phy1_RxDat : in std_logic_vector(1 downto 0);
       phyMii0_RxDat : in std_logic_vector(3 downto 0);
       phyMii1_RxDat : in std_logic_vector(3 downto 0);
       pio_pconfig : in std_logic_vector(3 downto 0);
       pio_portInLatch : in std_logic_vector(3 downto 0);
       pio_portio_I : in std_logic_vector(31 downto 0);
       M_AXI_MAC_DMA_arvalid : out std_logic;
       M_AXI_MAC_DMA_awvalid : out std_logic;
       M_AXI_MAC_DMA_bready : out std_logic;
       M_AXI_MAC_DMA_md_error : out std_logic;
       M_AXI_MAC_DMA_rready : out std_logic;
       M_AXI_MAC_DMA_wlast : out std_logic;
       M_AXI_MAC_DMA_wvalid : out std_logic;
       S_AXI_MAC_PKT_ARREADY : out std_logic;
       S_AXI_MAC_PKT_AWREADY : out std_logic;
       S_AXI_MAC_PKT_BVALID : out std_logic;
       S_AXI_MAC_PKT_RVALID : out std_logic;
       S_AXI_MAC_PKT_WREADY : out std_logic;
       S_AXI_MAC_REG_ARREADY : out std_logic;
       S_AXI_MAC_REG_AWREADY : out std_logic;
       S_AXI_MAC_REG_BVALID : out std_logic;
       S_AXI_MAC_REG_RVALID : out std_logic;
       S_AXI_MAC_REG_WREADY : out std_logic;
       S_AXI_PDI_AP_ARREADY : out std_logic;
       S_AXI_PDI_AP_AWREADY : out std_logic;
       S_AXI_PDI_AP_BVALID : out std_logic;
       S_AXI_PDI_AP_RVALID : out std_logic;
       S_AXI_PDI_AP_WREADY : out std_logic;
       S_AXI_PDI_PCP_ARREADY : out std_logic;
       S_AXI_PDI_PCP_AWREADY : out std_logic;
       S_AXI_PDI_PCP_BVALID : out std_logic;
       S_AXI_PDI_PCP_RVALID : out std_logic;
       S_AXI_PDI_PCP_WREADY : out std_logic;
       S_AXI_SMP_PCP_ARREADY : out std_logic;
       S_AXI_SMP_PCP_AWREADY : out std_logic;
       S_AXI_SMP_PCP_BVALID : out std_logic;
       S_AXI_SMP_PCP_RVALID : out std_logic;
       S_AXI_SMP_PCP_WREADY : out std_logic;
       ap_asyncIrq : out std_logic;
       ap_asyncIrq_n : out std_logic;
       ap_syncIrq : out std_logic;
       ap_syncIrq_n : out std_logic;
       led_error : out std_logic;
       led_status : out std_logic;
       mac_irq : out std_logic;
       pap_ack : out std_logic;
       pap_ack_n : out std_logic;
       pap_data_T : out std_logic;
       phy0_Rst_n : out std_logic;
       phy0_SMIClk : out std_logic;
       phy0_SMIDat_O : out std_logic;
       phy0_SMIDat_T : out std_logic;
       phy0_TxEn : out std_logic;
       phy0_clk : out std_logic;
       phy1_Rst_n : out std_logic;
       phy1_SMIClk : out std_logic;
       phy1_SMIDat_O : out std_logic;
       phy1_SMIDat_T : out std_logic;
       phy1_TxEn : out std_logic;
       phy1_clk : out std_logic;
       phyMii0_TxEn : out std_logic;
       phyMii0_TxEr : out std_logic;
       phyMii1_TxEn : out std_logic;
       phyMii1_TxEr : out std_logic;
       phy_Rst_n : out std_logic;
       phy_SMIClk : out std_logic;
       phy_SMIDat_O : out std_logic;
       phy_SMIDat_T : out std_logic;
       pio_operational : out std_logic;
       spi_miso : out std_logic;
       tcp_irq : out std_logic;
       M_AXI_MAC_DMA_araddr : out std_logic_vector(C_M_AXI_MAC_DMA_ADDR_WIDTH-1 downto 0);
       M_AXI_MAC_DMA_arburst : out std_logic_vector(1 downto 0);
       M_AXI_MAC_DMA_arcache : out std_logic_vector(3 downto 0);
       M_AXI_MAC_DMA_arlen : out std_logic_vector(7 downto 0);
       M_AXI_MAC_DMA_arprot : out std_logic_vector(2 downto 0);
       M_AXI_MAC_DMA_arsize : out std_logic_vector(2 downto 0);
       M_AXI_MAC_DMA_awaddr : out std_logic_vector(C_M_AXI_MAC_DMA_ADDR_WIDTH-1 downto 0);
       M_AXI_MAC_DMA_awburst : out std_logic_vector(1 downto 0);
       M_AXI_MAC_DMA_awcache : out std_logic_vector(3 downto 0);
       M_AXI_MAC_DMA_awlen : out std_logic_vector(7 downto 0);
       M_AXI_MAC_DMA_awprot : out std_logic_vector(2 downto 0);
       M_AXI_MAC_DMA_awsize : out std_logic_vector(2 downto 0);
       M_AXI_MAC_DMA_wdata : out std_logic_vector(C_M_AXI_MAC_DMA_DATA_WIDTH-1 downto 0);
       M_AXI_MAC_DMA_wstrb : out std_logic_vector((C_M_AXI_MAC_DMA_DATA_WIDTH/8)-1 downto 0);
       S_AXI_MAC_PKT_BRESP : out std_logic_vector(1 downto 0);
       S_AXI_MAC_PKT_RDATA : out std_logic_vector(C_S_AXI_MAC_PKT_DATA_WIDTH-1 downto 0);
       S_AXI_MAC_PKT_RRESP : out std_logic_vector(1 downto 0);
       S_AXI_MAC_REG_BRESP : out std_logic_vector(1 downto 0);
       S_AXI_MAC_REG_RDATA : out std_logic_vector(C_S_AXI_MAC_REG_DATA_WIDTH-1 downto 0);
       S_AXI_MAC_REG_RRESP : out std_logic_vector(1 downto 0);
       S_AXI_PDI_AP_BRESP : out std_logic_vector(1 downto 0);
       S_AXI_PDI_AP_RDATA : out std_logic_vector(C_S_AXI_PDI_AP_DATA_WIDTH-1 downto 0);
       S_AXI_PDI_AP_RRESP : out std_logic_vector(1 downto 0);
       S_AXI_PDI_PCP_BRESP : out std_logic_vector(1 downto 0);
       S_AXI_PDI_PCP_RDATA : out std_logic_vector(C_S_AXI_PDI_PCP_DATA_WIDTH-1 downto 0);
       S_AXI_PDI_PCP_RRESP : out std_logic_vector(1 downto 0);
       S_AXI_SMP_PCP_BRESP : out std_logic_vector(1 downto 0);
       S_AXI_SMP_PCP_RDATA : out std_logic_vector(C_S_AXI_SMP_PCP_DATA_WIDTH-1 downto 0);
       S_AXI_SMP_PCP_RRESP : out std_logic_vector(1 downto 0);
       led_gpo : out std_logic_vector(7 downto 0);
       led_opt : out std_logic_vector(1 downto 0);
       led_phyAct : out std_logic_vector(1 downto 0);
       led_phyLink : out std_logic_vector(1 downto 0);
       pap_data_O : out std_logic_vector(C_PAP_DATA_WIDTH-1 downto 0);
       pap_gpio_O : out std_logic_vector(1 downto 0);
       pap_gpio_T : out std_logic_vector(1 downto 0);
       phy0_TxDat : out std_logic_vector(1 downto 0);
       phy1_TxDat : out std_logic_vector(1 downto 0);
       phyMii0_TxDat : out std_logic_vector(3 downto 0);
       phyMii1_TxDat : out std_logic_vector(3 downto 0);
       pio_portOutValid : out std_logic_vector(3 downto 0);
       pio_portio_O : out std_logic_vector(31 downto 0);
       pio_portio_T : out std_logic_vector(31 downto 0);
       test_port : out std_logic_vector(255 downto 0) := (others => '0')
  );
-- Entity declarations --
-- Click here to add additional declarations --
attribute SIGIS : string;


-- Entity attributes --
attribute SIGIS of M_AXI_MAC_DMA_aclk : signal is "Clk";

attribute SIGIS of M_AXI_MAC_DMA_aresetn : signal is "Rst";

attribute SIGIS of S_AXI_MAC_PKT_ACLK : signal is "Clk";

attribute SIGIS of S_AXI_MAC_PKT_ARESETN : signal is "Rst";

attribute SIGIS of S_AXI_MAC_REG_ACLK : signal is "Clk";

attribute SIGIS of S_AXI_MAC_REG_ARESETN : signal is "Rst";

attribute SIGIS of S_AXI_PDI_AP_ACLK : signal is "Clk";

attribute SIGIS of S_AXI_PDI_AP_ARESETN : signal is "Rst";

attribute SIGIS of S_AXI_PDI_PCP_ACLK : signal is "Clk";

attribute SIGIS of S_AXI_PDI_PCP_ARESETN : signal is "Rst";

attribute SIGIS of S_AXI_SMP_PCP_ACLK : signal is "Clk";

attribute SIGIS of S_AXI_SMP_PCP_ARESETN : signal is "Rst";

attribute SIGIS of clk100 : signal is "Clk";

attribute SIGIS of phy0_clk : signal is "Clk";

attribute SIGIS of phy1_clk : signal is "Clk";

end axi_powerlink;

architecture struct of axi_powerlink is

---- Architecture declarations -----
function get_max( a, b : integer)  return integer is
begin
    if a < b then
        return b;
    else
        return a;
    end if;
end get_max;


---- Component declarations -----

component clkXing
  generic(
       gCsNum : natural := 2;
       gDataWidth : natural := 32
  );
  port (
       iArst : in std_logic;
       iFastClk : in std_logic;
       iFastCs : in std_logic_vector(gCsNum-1 downto 0);
       iFastRNW : in std_logic;
       iSlowClk : in std_logic;
       iSlowRdAck : in std_logic;
       iSlowReaddata : in std_logic_vector(gDataWidth-1 downto 0);
       iSlowWrAck : in std_logic;
       oFastRdAck : out std_logic;
       oFastReaddata : out std_logic_vector(gDataWidth-1 downto 0);
       oFastWrAck : out std_logic;
       oSlowCs : out std_logic_vector(gCsNum-1 downto 0);
       oSlowRNW : out std_logic
  );
end component;
component ipif_master_handler
  generic(
       C_MAC_DMA_IPIF_AWIDTH : integer := 32;
       C_MAC_DMA_IPIF_NATIVE_DWIDTH : integer := 32;
       dma_highadr_g : integer := 31;
       gen_rx_fifo_g : boolean := true;
       gen_tx_fifo_g : boolean := true;
       m_burstcount_width_g : integer := 4
  );
  port (
       Bus2MAC_DMA_MstRd_d : in std_logic_vector(C_MAC_DMA_IPIF_NATIVE_DWIDTH-1 downto 0);
       Bus2MAC_DMA_MstRd_eof_n : in std_logic := '1';
       Bus2MAC_DMA_MstRd_rem : in std_logic_vector(C_MAC_DMA_IPIF_NATIVE_DWIDTH/8-1 downto 0);
       Bus2MAC_DMA_MstRd_sof_n : in std_logic := '1';
       Bus2MAC_DMA_MstRd_src_dsc_n : in std_logic := '1';
       Bus2MAC_DMA_MstRd_src_rdy_n : in std_logic := '1';
       Bus2MAC_DMA_MstWr_dst_dsc_n : in std_logic := '1';
       Bus2MAC_DMA_MstWr_dst_rdy_n : in std_logic := '1';
       Bus2MAC_DMA_Mst_CmdAck : in std_logic := '0';
       Bus2MAC_DMA_Mst_Cmd_Timeout : in std_logic := '0';
       Bus2MAC_DMA_Mst_Cmplt : in std_logic := '0';
       Bus2MAC_DMA_Mst_Error : in std_logic := '0';
       Bus2MAC_DMA_Mst_Rearbitrate : in std_logic := '0';
       MAC_DMA_CLK : in std_logic;
       MAC_DMA_Rst : in std_logic;
       m_address : in std_logic_vector(dma_highadr_g downto 0);
       m_burstcount : in std_logic_vector(m_burstcount_width_g-1 downto 0);
       m_burstcounter : in std_logic_vector(m_burstcount_width_g-1 downto 0);
       m_byteenable : in std_logic_vector(3 downto 0);
       m_read : in std_logic := '0';
       m_write : in std_logic := '0';
       m_writedata : in std_logic_vector(31 downto 0);
       MAC_DMA2Bus_MstRd_Req : out std_logic := '0';
       MAC_DMA2Bus_MstRd_dst_dsc_n : out std_logic := '1';
       MAC_DMA2Bus_MstRd_dst_rdy_n : out std_logic := '1';
       MAC_DMA2Bus_MstWr_Req : out std_logic := '0';
       MAC_DMA2Bus_MstWr_d : out std_logic_vector(C_MAC_DMA_IPIF_NATIVE_DWIDTH-1 downto 0);
       MAC_DMA2Bus_MstWr_eof_n : out std_logic := '1';
       MAC_DMA2Bus_MstWr_rem : out std_logic_vector(C_MAC_DMA_IPIF_NATIVE_DWIDTH/8-1 downto 0);
       MAC_DMA2Bus_MstWr_sof_n : out std_logic := '1';
       MAC_DMA2Bus_MstWr_src_dsc_n : out std_logic := '1';
       MAC_DMA2Bus_MstWr_src_rdy_n : out std_logic := '1';
       MAC_DMA2Bus_Mst_Addr : out std_logic_vector(C_MAC_DMA_IPIF_AWIDTH-1 downto 0);
       MAC_DMA2Bus_Mst_BE : out std_logic_vector(C_MAC_DMA_IPIF_NATIVE_DWIDTH/8-1 downto 0);
       MAC_DMA2Bus_Mst_Length : out std_logic_vector(11 downto 0);
       MAC_DMA2Bus_Mst_Lock : out std_logic := '0';
       MAC_DMA2Bus_Mst_Reset : out std_logic := '0';
       MAC_DMA2Bus_Mst_Type : out std_logic := '0';
       m_clk : out std_logic;
       m_readdata : out std_logic_vector(31 downto 0);
       m_readdatavalid : out std_logic := '0';
       m_waitrequest : out std_logic := '1'
  );
end component;
component openMAC_16to32conv
  generic(
       bus_address_width : integer := 10;
       gEndian : string := "little"
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
component powerlink
  generic(
       Simulate : integer := 0;
       endian_g : string := "little";
       gNumSmi : integer range 1 to 2 := 2;
       genABuf1_g : integer := 1;
       genABuf2_g : integer := 1;
       genEvent_g : integer := 0;
       genInternalAp_g : integer := 1;
       genIoBuf_g : integer := 1;
       genLedGadget_g : integer := 0;
       genOnePdiClkDomain_g : integer := 0;
       genPdi_g : integer := 1;
       genSimpleIO_g : integer := 0;
       genSmiIO : integer := 1;
       genSpiAp_g : integer := 0;
       genTimeSync_g : integer := 0;
       gen_dma_observer_g : integer := 1;
       iAsyBuf1Size_g : integer := 100;
       iAsyBuf2Size_g : integer := 100;
       iBufSizeLOG2_g : integer := 10;
       iBufSize_g : integer := 1024;
       iPdiRev_g : integer := 21930;
       iRpdo0BufSize_g : integer := 100;
       iRpdo1BufSize_g : integer := 100;
       iRpdo2BufSize_g : integer := 100;
       iRpdos_g : integer := 3;
       iTpdoBufSize_g : integer := 100;
       iTpdos_g : integer := 1;
       m_burstcount_const_g : integer := 1;
       m_burstcount_width_g : integer := 4;
       m_data_width_g : integer := 16;
       m_rx_burst_size_g : integer := 16;
       m_rx_fifo_size_g : integer := 16;
       m_tx_burst_size_g : integer := 16;
       m_tx_fifo_size_g : integer := 16;
       papBigEnd_g : integer := 0;
       papDataWidth_g : integer := 8;
       papLowAct_g : integer := 0;
       pcpSysId : integer := 1;
       pioValLen_g : integer := 50;
       spiBigEnd_g : integer := 0;
       spiCPHA_g : integer := 0;
       spiCPOL_g : integer := 0;
       use2ndCmpTimer_g : integer := 1;
       usePulse2ndCmpTimer_g : integer := 1;
       pulseWidth2ndCmpTimer_g : integer := 9;
       use2ndPhy_g : integer := 1;
       useIntPacketBuf_g : integer := 1;
       useRmii_g : integer := 1;
       useRxIntPacketBuf_g : integer := 1
  );
  port (
       ap_address : in std_logic_vector(12 downto 0);
       ap_byteenable : in std_logic_vector(3 downto 0);
       ap_chipselect : in std_logic;
       ap_read : in std_logic;
       ap_write : in std_logic;
       ap_writedata : in std_logic_vector(31 downto 0);
       clk50 : in std_logic;
       clkAp : in std_logic;
       clkEth : in std_logic;
       clkPcp : in std_logic;
       m_clk : in std_logic;
       m_readdata : in std_logic_vector(m_data_width_g-1 downto 0) := (others => '0');
       m_readdatavalid : in std_logic := '0';
       m_waitrequest : in std_logic;
       mac_address : in std_logic_vector(11 downto 0);
       mac_byteenable : in std_logic_vector(1 downto 0);
       mac_chipselect : in std_logic;
       mac_read : in std_logic;
       mac_write : in std_logic;
       mac_writedata : in std_logic_vector(15 downto 0);
       mbf_address : in std_logic_vector(ibufsizelog2_g-3 downto 0);
       mbf_byteenable : in std_logic_vector(3 downto 0);
       mbf_chipselect : in std_logic;
       mbf_read : in std_logic;
       mbf_write : in std_logic;
       mbf_writedata : in std_logic_vector(31 downto 0);
       pap_addr : in std_logic_vector(15 downto 0);
       pap_be : in std_logic_vector(papDataWidth_g/8-1 downto 0);
       pap_be_n : in std_logic_vector(papDataWidth_g/8-1 downto 0);
       pap_cs : in std_logic;
       pap_cs_n : in std_logic;
       pap_data_I : in std_logic_vector(papDataWidth_g-1 downto 0) := (others => '0');
       pap_gpio_I : in std_logic_vector(1 downto 0) := (others => '0');
       pap_rd : in std_logic;
       pap_rd_n : in std_logic;
       pap_wr : in std_logic;
       pap_wr_n : in std_logic;
       pcp_address : in std_logic_vector(12 downto 0);
       pcp_byteenable : in std_logic_vector(3 downto 0);
       pcp_chipselect : in std_logic;
       pcp_read : in std_logic;
       pcp_write : in std_logic;
       pcp_writedata : in std_logic_vector(31 downto 0);
       phy0_RxDat : in std_logic_vector(1 downto 0);
       phy0_RxDv : in std_logic;
       phy0_RxErr : in std_logic;
       phy0_SMIDat_I : in std_logic := '1';
       phy0_link : in std_logic := '0';
       phy1_RxDat : in std_logic_vector(1 downto 0) := (others => '0');
       phy1_RxDv : in std_logic;
       phy1_RxErr : in std_logic;
       phy1_SMIDat_I : in std_logic := '1';
       phy1_link : in std_logic := '0';
       phyMii0_RxClk : in std_logic;
       phyMii0_RxDat : in std_logic_vector(3 downto 0) := (others => '0');
       phyMii0_RxDv : in std_logic;
       phyMii0_RxEr : in std_logic;
       phyMii0_TxClk : in std_logic;
       phyMii1_RxClk : in std_logic;
       phyMii1_RxDat : in std_logic_vector(3 downto 0) := (others => '0');
       phyMii1_RxDv : in std_logic;
       phyMii1_RxEr : in std_logic;
       phyMii1_TxClk : in std_logic;
       phy_SMIDat_I : in std_logic := '1';
       pio_pconfig : in std_logic_vector(3 downto 0);
       pio_portInLatch : in std_logic_vector(3 downto 0);
       pio_portio_I : in std_logic_vector(31 downto 0) := (others => '0');
       pkt_clk : in std_logic;
       rst : in std_logic;
       rstAp : in std_logic;
       rstPcp : in std_logic;
       smp_address : in std_logic;
       smp_byteenable : in std_logic_vector(3 downto 0);
       smp_read : in std_logic;
       smp_write : in std_logic;
       smp_writedata : in std_logic_vector(31 downto 0);
       spi_clk : in std_logic;
       spi_mosi : in std_logic;
       spi_sel_n : in std_logic;
       tcp_address : in std_logic_vector(1 downto 0);
       tcp_byteenable : in std_logic_vector(3 downto 0);
       tcp_chipselect : in std_logic;
       tcp_read : in std_logic;
       tcp_write : in std_logic;
       tcp_writedata : in std_logic_vector(31 downto 0);
       ap_asyncIrq : out std_logic := '0';
       ap_asyncIrq_n : out std_logic := '1';
       ap_irq : out std_logic := '0';
       ap_irq_n : out std_logic := '1';
       ap_readdata : out std_logic_vector(31 downto 0) := (others => '0');
       ap_syncIrq : out std_logic := '0';
       ap_syncIrq_n : out std_logic := '1';
       ap_waitrequest : out std_logic;
       led_error : out std_logic := '0';
       led_gpo : out std_logic_vector(7 downto 0) := (others => '0');
       led_opt : out std_logic_vector(1 downto 0) := (others => '0');
       led_phyAct : out std_logic_vector(1 downto 0) := (others => '0');
       led_phyLink : out std_logic_vector(1 downto 0) := (others => '0');
       led_status : out std_logic := '0';
       m_address : out std_logic_vector(31 downto 0) := (others => '0');
       m_burstcount : out std_logic_vector(m_burstcount_width_g-1 downto 0);
       m_burstcounter : out std_logic_vector(m_burstcount_width_g-1 downto 0);
       m_byteenable : out std_logic_vector(m_data_width_g/8-1 downto 0) := (others => '0');
       m_read : out std_logic := '0';
       m_write : out std_logic := '0';
       m_writedata : out std_logic_vector(m_data_width_g-1 downto 0) := (others => '0');
       mac_irq : out std_logic := '0';
       mac_readdata : out std_logic_vector(15 downto 0) := (others => '0');
       mac_waitrequest : out std_logic;
       mbf_readdata : out std_logic_vector(31 downto 0) := (others => '0');
       mbf_waitrequest : out std_logic;
       pap_ack : out std_logic := '0';
       pap_ack_n : out std_logic := '1';
       pap_data_O : out std_logic_vector(papDataWidth_g-1 downto 0);
       pap_data_T : out std_logic;
       pap_gpio_O : out std_logic_vector(1 downto 0);
       pap_gpio_T : out std_logic_vector(1 downto 0);
       pcp_readdata : out std_logic_vector(31 downto 0) := (others => '0');
       pcp_waitrequest : out std_logic;
       phy0_Rst_n : out std_logic := '1';
       phy0_SMIClk : out std_logic := '0';
       phy0_SMIDat_O : out std_logic;
       phy0_SMIDat_T : out std_logic;
       phy0_TxDat : out std_logic_vector(1 downto 0) := (others => '0');
       phy0_TxEn : out std_logic := '0';
       phy1_Rst_n : out std_logic := '1';
       phy1_SMIClk : out std_logic := '0';
       phy1_SMIDat_O : out std_logic;
       phy1_SMIDat_T : out std_logic;
       phy1_TxDat : out std_logic_vector(1 downto 0) := (others => '0');
       phy1_TxEn : out std_logic := '0';
       phyMii0_TxDat : out std_logic_vector(3 downto 0) := (others => '0');
       phyMii0_TxEn : out std_logic := '0';
       phyMii0_TxEr : out std_logic := '0';
       phyMii1_TxDat : out std_logic_vector(3 downto 0) := (others => '0');
       phyMii1_TxEn : out std_logic := '0';
       phyMii1_TxEr : out std_logic := '0';
       phy_Rst_n : out std_logic := '1';
       phy_SMIClk : out std_logic := '0';
       phy_SMIDat_O : out std_logic;
       phy_SMIDat_T : out std_logic;
       pio_operational : out std_logic := '0';
       pio_portOutValid : out std_logic_vector(3 downto 0) := (others => '0');
       pio_portio_O : out std_logic_vector(31 downto 0);
       pio_portio_T : out std_logic_vector(31 downto 0);
       smp_readdata : out std_logic_vector(31 downto 0) := (others => '0');
       smp_waitrequest : out std_logic;
       spi_miso : out std_logic := '0';
       tcp_irq : out std_logic := '0';
       tcp_readdata : out std_logic_vector(31 downto 0) := (others => '0');
       tcp_waitrequest : out std_logic;
       pap_data : inout std_logic_vector(papDataWidth_g-1 downto 0) := (others => '0');
       pap_gpio : inout std_logic_vector(1 downto 0) := (others => '0');
       phy0_SMIDat : inout std_logic := '1';
       phy1_SMIDat : inout std_logic := '1';
       phy_SMIDat : inout std_logic := '1';
       pio_portio : inout std_logic_vector(31 downto 0) := (others => '0')
  );
end component;
component axi_lite_ipif
  generic(
       C_ARD_ADDR_RANGE_ARRAY : slv64_array_type := (X"0000_0000_7000_0000",X"0000_0000_7000_00FF",X"0000_0000_7000_0100",X"0000_0000_7000_01FF");
       C_ARD_NUM_CE_ARRAY : integer_array_type := (4,12);
       C_DPHASE_TIMEOUT : integer range 0 to 512 := 8;
       C_FAMILY : string := "virtex6";
       C_S_AXI_ADDR_WIDTH : integer := 32;
       C_S_AXI_DATA_WIDTH : integer range 32 to 32 := 32;
       C_S_AXI_MIN_SIZE : std_logic_vector(31 downto 0) := X"000001FF";
       C_USE_WSTRB : integer := 0
  );
  port (
       IP2Bus_Data : in std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
       IP2Bus_Error : in std_logic;
       IP2Bus_RdAck : in std_logic;
       IP2Bus_WrAck : in std_logic;
       S_AXI_ACLK : in std_logic;
       S_AXI_ARADDR : in std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
       S_AXI_ARESETN : in std_logic;
       S_AXI_ARVALID : in std_logic;
       S_AXI_AWADDR : in std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
       S_AXI_AWVALID : in std_logic;
       S_AXI_BREADY : in std_logic;
       S_AXI_RREADY : in std_logic;
       S_AXI_WDATA : in std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
       S_AXI_WSTRB : in std_logic_vector((C_S_AXI_DATA_WIDTH/8)-1 downto 0);
       S_AXI_WVALID : in std_logic;
       Bus2IP_Addr : out std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
       Bus2IP_BE : out std_logic_vector((C_S_AXI_DATA_WIDTH/8)-1 downto 0);
       Bus2IP_CS : out std_logic_vector((C_ARD_ADDR_RANGE_ARRAY'LENGTH)/2-1 downto 0);
       Bus2IP_Clk : out std_logic;
       Bus2IP_Data : out std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
       Bus2IP_RNW : out std_logic;
       Bus2IP_RdCE : out std_logic_vector(calc_num_ce(C_ARD_NUM_CE_ARRAY)-1 downto 0);
       Bus2IP_Resetn : out std_logic;
       Bus2IP_WrCE : out std_logic_vector(calc_num_ce(C_ARD_NUM_CE_ARRAY)-1 downto 0);
       S_AXI_ARREADY : out std_logic;
       S_AXI_AWREADY : out std_logic;
       S_AXI_BRESP : out std_logic_vector(1 downto 0);
       S_AXI_BVALID : out std_logic;
       S_AXI_RDATA : out std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
       S_AXI_RRESP : out std_logic_vector(1 downto 0);
       S_AXI_RVALID : out std_logic;
       S_AXI_WREADY : out std_logic
  );
end component;
component axi_master_burst
  generic(
       C_ADDR_PIPE_DEPTH : integer range 1 to 14 := 1;
       C_FAMILY : string := "virtex6";
       C_LENGTH_WIDTH : integer range 12 to 20 := 12;
       C_MAX_BURST_LEN : integer range 16 to 256 := 16;
       C_M_AXI_ADDR_WIDTH : integer range 32 to 32 := 32;
       C_M_AXI_DATA_WIDTH : integer range 32 to 256 := 32;
       C_NATIVE_DATA_WIDTH : integer range 32 to 128 := 32
  );
  port (
       ip2bus_mst_addr : in std_logic_vector(C_M_AXI_ADDR_WIDTH-1 downto 0);
       ip2bus_mst_be : in std_logic_vector((C_NATIVE_DATA_WIDTH/8)-1 downto 0);
       ip2bus_mst_length : in std_logic_vector(C_LENGTH_WIDTH-1 downto 0);
       ip2bus_mst_lock : in std_logic;
       ip2bus_mst_reset : in std_logic;
       ip2bus_mst_type : in std_logic;
       ip2bus_mstrd_dst_dsc_n : in std_logic;
       ip2bus_mstrd_dst_rdy_n : in std_logic;
       ip2bus_mstrd_req : in std_logic;
       ip2bus_mstwr_d : in std_logic_vector(C_NATIVE_DATA_WIDTH-1 downto 0);
       ip2bus_mstwr_eof_n : in std_logic;
       ip2bus_mstwr_rem : in std_logic_vector((C_NATIVE_DATA_WIDTH/8)-1 downto 0);
       ip2bus_mstwr_req : in std_logic;
       ip2bus_mstwr_sof_n : in std_logic;
       ip2bus_mstwr_src_dsc_n : in std_logic;
       ip2bus_mstwr_src_rdy_n : in std_logic;
       m_axi_aclk : in std_logic;
       m_axi_aresetn : in std_logic;
       m_axi_arready : in std_logic;
       m_axi_awready : in std_logic;
       m_axi_bresp : in std_logic_vector(1 downto 0);
       m_axi_bvalid : in std_logic;
       m_axi_rdata : in std_logic_vector(C_M_AXI_DATA_WIDTH-1 downto 0);
       m_axi_rlast : in std_logic;
       m_axi_rresp : in std_logic_vector(1 downto 0);
       m_axi_rvalid : in std_logic;
       m_axi_wready : in std_logic;
       bus2ip_mst_cmd_timeout : out std_logic;
       bus2ip_mst_cmdack : out std_logic;
       bus2ip_mst_cmplt : out std_logic;
       bus2ip_mst_error : out std_logic;
       bus2ip_mst_rearbitrate : out std_logic;
       bus2ip_mstrd_d : out std_logic_vector(C_NATIVE_DATA_WIDTH-1 downto 0);
       bus2ip_mstrd_eof_n : out std_logic;
       bus2ip_mstrd_rem : out std_logic_vector((C_NATIVE_DATA_WIDTH/8)-1 downto 0);
       bus2ip_mstrd_sof_n : out std_logic;
       bus2ip_mstrd_src_dsc_n : out std_logic;
       bus2ip_mstrd_src_rdy_n : out std_logic;
       bus2ip_mstwr_dst_dsc_n : out std_logic;
       bus2ip_mstwr_dst_rdy_n : out std_logic;
       m_axi_araddr : out std_logic_vector(C_M_AXI_ADDR_WIDTH-1 downto 0);
       m_axi_arburst : out std_logic_vector(1 downto 0);
       m_axi_arcache : out std_logic_vector(3 downto 0);
       m_axi_arlen : out std_logic_vector(7 downto 0);
       m_axi_arprot : out std_logic_vector(2 downto 0);
       m_axi_arsize : out std_logic_vector(2 downto 0);
       m_axi_arvalid : out std_logic;
       m_axi_awaddr : out std_logic_vector(C_M_AXI_ADDR_WIDTH-1 downto 0);
       m_axi_awburst : out std_logic_vector(1 downto 0);
       m_axi_awcache : out std_logic_vector(3 downto 0);
       m_axi_awlen : out std_logic_vector(7 downto 0);
       m_axi_awprot : out std_logic_vector(2 downto 0);
       m_axi_awsize : out std_logic_vector(2 downto 0);
       m_axi_awvalid : out std_logic;
       m_axi_bready : out std_logic;
       m_axi_rready : out std_logic;
       m_axi_wdata : out std_logic_vector(C_M_AXI_DATA_WIDTH-1 downto 0);
       m_axi_wlast : out std_logic;
       m_axi_wstrb : out std_logic_vector((C_M_AXI_DATA_WIDTH/8)-1 downto 0);
       m_axi_wvalid : out std_logic;
       md_error : out std_logic
  );
end component;

---- Architecture declarations -----
constant C_ADDR_PAD_ZERO : std_logic_vector(31 downto 0) := (others => '0');
-- openMAC REG PLB Slave
constant C_MAC_REG_BASE : std_logic_vector(63 downto 0) := C_ADDR_PAD_ZERO & C_S_AXI_MAC_REG_RNG0_BASEADDR;
constant C_MAC_REG_HIGH : std_logic_vector(63 downto 0) := C_ADDR_PAD_ZERO & C_S_AXI_MAC_REG_RNG0_HIGHADDR;
constant C_MAC_REG_MINSIZE : std_logic_vector(31 downto 0) := conv_std_logic_vector(get_max(conv_integer(C_S_AXI_MAC_REG_RNG0_HIGHADDR), conv_integer(C_S_AXI_MAC_REG_RNG1_HIGHADDR)), 32);
-- openMAC CMP PLB Slave
constant C_MAC_CMP_BASE : std_logic_vector(63 downto 0) := C_ADDR_PAD_ZERO & C_S_AXI_MAC_REG_RNG1_BASEADDR;
constant C_MAC_CMP_HIGH : std_logic_vector(63 downto 0) := C_ADDR_PAD_ZERO & C_S_AXI_MAC_REG_RNG1_HIGHADDR;
-- openMAC PKT PLB Slave
constant C_MAC_PKT_BASE : std_logic_vector(63 downto 0) := C_ADDR_PAD_ZERO & C_S_AXI_MAC_PKT_BASEADDR;
constant C_MAC_PKT_HIGH : std_logic_vector(63 downto 0) := C_ADDR_PAD_ZERO & C_S_AXI_MAC_PKT_HIGHADDR;
constant C_MAC_PKT_MINSIZE : std_logic_vector(31 downto 0) := C_S_AXI_MAC_PKT_HIGHADDR;
-- SimpleIO Slave
constant C_SMP_PCP_BASE : std_logic_vector(63 downto 0) := C_ADDR_PAD_ZERO & C_S_AXI_SMP_PCP_BASEADDR;
constant C_SMP_PCP_HIGH : std_logic_vector(63 downto 0) := C_ADDR_PAD_ZERO & C_S_AXI_SMP_PCP_HIGHADDR;
constant C_SMP_PCP_MINSIZE : std_logic_vector(31 downto 0) := C_S_AXI_SMP_PCP_HIGHADDR;
-- PDI PCP Slave
constant C_PDI_PCP_BASE : std_logic_vector(63 downto 0) := C_ADDR_PAD_ZERO & C_S_AXI_PDI_PCP_BASEADDR;
constant C_PDI_PCP_HIGH : std_logic_vector(63 downto 0) := C_ADDR_PAD_ZERO & C_S_AXI_PDI_PCP_HIGHADDR;
constant C_PDI_PCP_MINSIZE : std_logic_vector(31 downto 0) := C_S_AXI_PDI_PCP_HIGHADDR;
-- AP PCP Slave
constant C_PDI_AP_BASE : std_logic_vector(63 downto 0) := C_ADDR_PAD_ZERO & C_S_AXI_PDI_AP_BASEADDR;
constant C_PDI_AP_HIGH : std_logic_vector(63 downto 0) := C_ADDR_PAD_ZERO & C_S_AXI_PDI_AP_HIGHADDR;
constant C_PDI_AP_MINSIZE : std_logic_vector(31 downto 0) := C_S_AXI_PDI_AP_HIGHADDR;
-- POWERLINK IP-core
constant C_MAC_PKT_EN : boolean := C_TX_INT_PKT or C_RX_INT_PKT;
constant C_MAC_PKT_RX_EN : boolean := C_RX_INT_PKT;
constant C_DMA_EN : boolean := not C_TX_INT_PKT or not C_RX_INT_PKT;
constant C_PKT_BUF_EN : boolean := C_MAC_PKT_EN;
constant C_M_BURSTCOUNT_WIDTH : integer := integer(ceil(log2(real(get_max(C_MAC_DMA_BURST_SIZE_RX,C_MAC_DMA_BURST_SIZE_TX)/4)))) + 1; --in dwords
constant C_M_FIFO_SIZE_RX : integer := C_MAC_DMA_FIFO_SIZE_RX/4; --in dwords
constant C_M_FIFO_SIZE_TX : integer := C_MAC_DMA_FIFO_SIZE_TX/4; --in dwords


----     Constants     -----
constant VCC_CONSTANT   : std_logic := '1';
constant GND_CONSTANT   : std_logic := '0';

---- Signal declarations used on the diagram ----

signal ap_chipselect : std_logic;
signal ap_read : std_logic;
signal ap_waitrequest : std_logic;
signal ap_write : std_logic;
signal bus2MAC_DMA_mstrd_eof_n : std_logic;
signal bus2MAC_DMA_mstrd_sof_n : std_logic;
signal bus2MAC_DMA_mstrd_src_dsc_n : std_logic;
signal bus2MAC_DMA_mstrd_src_rdy_n : std_logic;
signal bus2MAC_DMA_mstwr_dst_dsc_n : std_logic;
signal bus2MAC_DMA_mstwr_dst_rdy_n : std_logic;
signal bus2MAC_DMA_mst_cmdack : std_logic;
signal bus2MAC_DMA_mst_cmd_timeout : std_logic;
signal bus2MAC_DMA_mst_cmplt : std_logic;
signal bus2MAC_DMA_mst_error : std_logic;
signal bus2MAC_DMA_mst_rearbitrate : std_logic;
signal Bus2MAC_PKT_Clk : std_logic;
signal Bus2MAC_PKT_Reset : std_logic := '0';
signal Bus2MAC_PKT_Resetn : std_logic;
signal Bus2MAC_PKT_RNW : std_logic;
signal Bus2MAC_REG_Clk : std_logic;
signal Bus2MAC_REG_Reset : std_logic := '0';
signal Bus2MAC_REG_Resetn : std_logic;
signal Bus2MAC_REG_RNW : std_logic;
signal Bus2MAC_REG_RNW_fast : std_logic;
signal Bus2MAC_REG_RNW_n : std_logic;
signal Bus2PDI_AP_Clk : std_logic;
signal Bus2PDI_AP_Reset : std_logic := '0';
signal Bus2PDI_AP_Resetn : std_logic;
signal Bus2PDI_AP_RNW : std_logic;
signal Bus2PDI_PCP_Clk : std_logic;
signal Bus2PDI_PCP_Reset : std_logic := '0';
signal Bus2PDI_PCP_Resetn : std_logic;
signal Bus2PDI_PCP_RNW : std_logic;
signal Bus2SMP_PCP_Clk : std_logic;
signal Bus2SMP_PCP_Reset : std_logic := '0';
signal Bus2SMP_PCP_Resetn : std_logic;
signal Bus2SMP_PCP_RNW : std_logic;
signal clkAp : std_logic;
signal clkPcp : std_logic;
signal GND : std_logic;
signal IP2Bus_Error_s : std_logic;
signal IP2Bus_RdAck_fast : std_logic;
signal IP2Bus_RdAck_s : std_logic;
signal IP2Bus_WrAck_fast : std_logic;
signal IP2Bus_WrAck_s : std_logic;
signal mac_chipselect : std_logic;
signal MAC_CMP2Bus_Error : std_logic;
signal MAC_CMP2Bus_RdAck : std_logic;
signal MAC_CMP2Bus_WrAck : std_logic;
signal MAC_DMA2bus_mstrd_dst_dsc_n : std_logic;
signal MAC_DMA2bus_mstrd_dst_rdy_n : std_logic;
signal MAC_DMA2bus_mstrd_req : std_logic;
signal MAC_DMA2bus_mstwr_eof_n : std_logic;
signal MAC_DMA2bus_mstwr_req : std_logic;
signal MAC_DMA2bus_mstwr_sof_n : std_logic;
signal MAC_DMA2bus_mstwr_src_dsc_n : std_logic;
signal MAC_DMA2bus_mstwr_src_rdy_n : std_logic;
signal MAC_DMA2bus_mst_lock : std_logic;
signal MAC_DMA2bus_mst_reset : std_logic;
signal MAC_DMA2bus_mst_type : std_logic;
signal MAC_DMA_areset : std_logic;
signal mac_irq_s : std_logic;
signal MAC_PKT2Bus_Error : std_logic;
signal MAC_PKT2Bus_RdAck : std_logic;
signal MAC_PKT2Bus_WrAck : std_logic;
signal mac_read : std_logic;
signal MAC_REG2Bus_Error : std_logic;
signal MAC_REG2Bus_RdAck : std_logic;
signal MAC_REG2Bus_WrAck : std_logic;
signal mac_waitrequest : std_logic;
signal mac_write : std_logic;
signal mbf_chipselect : std_logic;
signal mbf_read : std_logic;
signal mbf_waitrequest : std_logic;
signal mbf_write : std_logic;
signal m_clk : std_logic;
signal m_read : std_logic;
signal m_readdatavalid : std_logic;
signal m_waitrequest : std_logic;
signal m_write : std_logic;
signal NET38418 : std_ulogic;
signal NET38470 : std_ulogic;
signal pcp_chipselect : std_logic;
signal pcp_read : std_logic;
signal pcp_waitrequest : std_logic;
signal pcp_write : std_logic;
signal PDI_AP2Bus_Error : std_logic;
signal PDI_AP2Bus_RdAck : std_logic;
signal PDI_AP2Bus_WrAck : std_logic;
signal PDI_PCP2Bus_Error : std_logic;
signal PDI_PCP2Bus_RdAck : std_logic;
signal PDI_PCP2Bus_WrAck : std_logic;
signal pkt_clk : std_logic;
signal rst : std_logic := '0';
signal rstAp : std_logic := '0';
signal rstPcp : std_logic := '0';
signal smp_address : std_logic;
signal smp_chipselect : std_logic;
signal SMP_PCP2Bus_Error : std_logic;
signal SMP_PCP2Bus_RdAck : std_logic;
signal SMP_PCP2Bus_WrAck : std_logic;
signal smp_read : std_logic;
signal smp_waitrequest : std_logic;
signal smp_write : std_logic;
signal tcp_chipselect : std_logic;
signal tcp_irq_s : std_logic;
signal tcp_read : std_logic;
signal tcp_waitrequest : std_logic;
signal tcp_write : std_logic;
signal VCC : std_logic;
signal ap_address : std_logic_vector (12 downto 0);
signal ap_byteenable : std_logic_vector (3 downto 0);
signal ap_readdata : std_logic_vector (31 downto 0);
signal ap_writedata : std_logic_vector (31 downto 0);
signal bus2MAC_DMA_mstrd_d : std_logic_vector (C_M_AXI_MAC_DMA_NATIVE_DWIDTH-1 downto 0);
signal bus2MAC_DMA_mstrd_rem : std_logic_vector ((C_M_AXI_MAC_DMA_NATIVE_DWIDTH/8)-1 downto 0);
signal Bus2MAC_PKT_Addr : std_logic_vector (C_S_AXI_MAC_PKT_ADDR_WIDTH-1 downto 0);
signal Bus2MAC_PKT_BE : std_logic_vector ((C_S_AXI_MAC_PKT_DATA_WIDTH/8)-1 downto 0);
signal Bus2MAC_PKT_CS : std_logic_vector (0 downto 0);
signal Bus2MAC_PKT_Data : std_logic_vector (C_S_AXI_MAC_PKT_DATA_WIDTH-1 downto 0);
signal Bus2MAC_REG_Addr : std_logic_vector (C_S_AXI_MAC_REG_ADDR_WIDTH-1 downto 0);
signal Bus2MAC_REG_BE : std_logic_vector ((C_S_AXI_MAC_REG_DATA_WIDTH/8)-1 downto 0);
signal Bus2MAC_REG_CS : std_logic_vector (1 downto 0);
signal Bus2MAC_REG_CS_fast : std_logic_vector (1 downto 0);
signal Bus2MAC_REG_Data : std_logic_vector (C_S_AXI_MAC_REG_DATA_WIDTH-1 downto 0);
signal Bus2PDI_AP_Addr : std_logic_vector (C_S_AXI_PDI_AP_ADDR_WIDTH-1 downto 0);
signal Bus2PDI_AP_BE : std_logic_vector ((C_S_AXI_PDI_AP_DATA_WIDTH/8)-1 downto 0);
signal Bus2PDI_AP_CS : std_logic_vector (0 downto 0);
signal Bus2PDI_AP_Data : std_logic_vector (C_S_AXI_PDI_AP_DATA_WIDTH-1 downto 0);
signal Bus2PDI_PCP_Addr : std_logic_vector (C_S_AXI_PDI_PCP_ADDR_WIDTH-1 downto 0);
signal Bus2PDI_PCP_BE : std_logic_vector ((C_S_AXI_PDI_PCP_DATA_WIDTH/8)-1 downto 0);
signal Bus2PDI_PCP_CS : std_logic_vector (0 downto 0);
signal Bus2PDI_PCP_Data : std_logic_vector (C_S_AXI_PDI_PCP_DATA_WIDTH-1 downto 0);
signal Bus2SMP_PCP_Addr : std_logic_vector (C_S_AXI_SMP_PCP_ADDR_WIDTH-1 downto 0);
signal Bus2SMP_PCP_BE : std_logic_vector ((C_S_AXI_SMP_PCP_DATA_WIDTH/8)-1 downto 0);
signal Bus2SMP_PCP_CS : std_logic_vector (0 downto 0);
signal Bus2SMP_PCP_Data : std_logic_vector (C_S_AXI_SMP_PCP_DATA_WIDTH-1 downto 0);
signal IP2Bus_Data_fast : std_logic_vector (C_S_AXI_MAC_REG_DATA_WIDTH-1 downto 0);
signal IP2Bus_Data_s : std_logic_vector (C_S_AXI_MAC_REG_DATA_WIDTH-1 downto 0);
signal mac_address : std_logic_vector (11 downto 0);
signal mac_address_full : std_logic_vector (C_S_AXI_MAC_REG_ADDR_WIDTH-1 downto 0);
signal mac_byteenable : std_logic_vector (1 downto 0);
signal MAC_CMP2Bus_Data : std_logic_vector (C_S_AXI_MAC_REG_DATA_WIDTH-1 downto 0);
signal MAC_DMA2Bus_MstWr_d : std_logic_vector (C_M_AXI_MAC_DMA_NATIVE_DWIDTH-1 downto 0);
signal MAC_DMA2bus_mstwr_rem : std_logic_vector ((C_M_AXI_MAC_DMA_NATIVE_DWIDTH/8)-1 downto 0);
signal MAC_DMA2bus_mst_addr : std_logic_vector (C_M_AXI_MAC_DMA_ADDR_WIDTH-1 downto 0);
signal MAC_DMA2bus_mst_be : std_logic_vector ((C_M_AXI_MAC_DMA_NATIVE_DWIDTH/8)-1 downto 0);
signal MAC_DMA2bus_mst_length : std_logic_vector (C_M_AXI_MAC_DMA_LENGTH_WIDTH-1 downto 0);
signal MAC_PKT2Bus_Data : std_logic_vector (C_S_AXI_MAC_PKT_DATA_WIDTH-1 downto 0);
signal mac_readdata : std_logic_vector (15 downto 0);
signal MAC_REG2Bus_Data : std_logic_vector (C_S_AXI_MAC_REG_DATA_WIDTH-1 downto 0);
signal mac_writedata : std_logic_vector (15 downto 0);
signal mbf_address : std_logic_vector (C_MAC_PKT_SIZE_LOG2-3 downto 0);
signal mbf_byteenable : std_logic_vector (3 downto 0);
signal mbf_readdata : std_logic_vector (31 downto 0);
signal mbf_writedata : std_logic_vector (31 downto 0);
signal m_address : std_logic_vector (31 downto 0);
signal m_burstcount : std_logic_vector (C_M_BURSTCOUNT_WIDTH-1 downto 0);
signal m_burstcounter : std_logic_vector (C_M_BURSTCOUNT_WIDTH-1 downto 0);
signal m_byteenable : std_logic_vector (3 downto 0);
signal m_readdata : std_logic_vector (31 downto 0);
signal m_writedata : std_logic_vector (31 downto 0);
signal pcp_address : std_logic_vector (12 downto 0);
signal pcp_byteenable : std_logic_vector (3 downto 0);
signal pcp_readdata : std_logic_vector (31 downto 0);
signal pcp_writedata : std_logic_vector (31 downto 0);
signal PDI_AP2Bus_Data : std_logic_vector (C_S_AXI_PDI_AP_DATA_WIDTH-1 downto 0);
signal PDI_PCP2Bus_Data : std_logic_vector (C_S_AXI_PDI_PCP_DATA_WIDTH-1 downto 0);
signal smp_byteenable : std_logic_vector (3 downto 0);
signal SMP_PCP2Bus_Data : std_logic_vector (C_S_AXI_SMP_PCP_DATA_WIDTH-1 downto 0);
signal smp_readdata : std_logic_vector (31 downto 0);
signal smp_writedata : std_logic_vector (31 downto 0);
signal tcp_address : std_logic_vector (1 downto 0);
signal tcp_byteenable : std_logic_vector (3 downto 0);
signal tcp_readdata : std_logic_vector (31 downto 0);
signal tcp_writedata : std_logic_vector (31 downto 0);

begin

---- User Signal Assignments ----
-- connect mac reg with mac cmp or reg output signals
with Bus2MAC_REG_CS select
    IP2Bus_Data_s <=    MAC_CMP2Bus_Data    when "01",
                        MAC_REG2Bus_Data    when others; --"10" and others are decoded to MAC_REG

IP2Bus_WrAck_s <= MAC_REG2Bus_WrAck or MAC_CMP2Bus_WrAck;
IP2Bus_RdAck_s <= MAC_REG2Bus_RdAck or MAC_CMP2Bus_RdAck;
IP2Bus_Error_s <= MAC_REG2Bus_Error or MAC_CMP2Bus_Error;
mac_address <= mac_address_full(mac_address'range);
--mac_cmp assignments
---cmp_clk <= Bus2MAC_CMP_Clk;
tcp_writedata <= Bus2MAC_REG_Data;
tcp_read <= Bus2MAC_REG_RNW;
tcp_write <= not Bus2MAC_REG_RNW;
tcp_chipselect <= Bus2MAC_REG_CS(0);
tcp_byteenable <= Bus2MAC_REG_BE;
tcp_address <= Bus2MAC_REG_Addr(3 downto 2);

MAC_CMP2Bus_Data <= tcp_readdata;
MAC_CMP2Bus_RdAck <= tcp_chipselect and tcp_read and not tcp_waitrequest;
MAC_CMP2Bus_WrAck <= tcp_chipselect and tcp_write and not tcp_waitrequest;
MAC_CMP2Bus_Error <= '0';
--mac_pkt assignments
pkt_clk <= Bus2MAC_PKT_Clk;
Bus2MAC_PKT_Reset <= not Bus2MAC_PKT_Resetn;
mbf_writedata <= Bus2MAC_PKT_Data;
--    Bus2MAC_PKT_Data(7 downto 0) & Bus2MAC_PKT_Data(15 downto 8) &
--    Bus2MAC_PKT_Data(23 downto 16) & Bus2MAC_PKT_Data(31 downto 24);
mbf_read <= Bus2MAC_PKT_RNW;
mbf_write <= not Bus2MAC_PKT_RNW;
mbf_chipselect <= Bus2MAC_PKT_CS(0);
mbf_byteenable <= Bus2MAC_PKT_BE;
mbf_address <= Bus2MAC_PKT_Addr(C_MAC_PKT_SIZE_LOG2-1 downto 2);

MAC_PKT2Bus_Data <= mbf_readdata;
MAC_PKT2Bus_RdAck <= mbf_chipselect and mbf_read and not mbf_waitrequest;
MAC_PKT2Bus_WrAck <= mbf_chipselect and mbf_write and not mbf_waitrequest;
MAC_PKT2Bus_Error <= '0';
--test_port

--test_port(181 downto 179) <= mac_chipselect & mac_write & mac_read;
--test_port(178) <= mac_waitrequest;
--test_port(177 downto 176) <= mac_byteenable;
--
--test_port(171 downto 160) <= mac_address;
--test_port(159 downto 144) <= mac_writedata;
--test_port(143 downto 128) <= mac_readdata;
--
--test_port(104 downto 102) <= Bus2MAC_REG_CS & Bus2MAC_REG_RNW;
--test_port(101 downto 100) <= IP2Bus_WrAck_s & IP2Bus_RdAck_s;
--test_port(99 downto 96) <= Bus2MAC_REG_BE;
--
--test_port(95 downto 64) <= Bus2MAC_REG_Addr;
--test_port(63 downto 32) <= Bus2MAC_REG_Data;
--test_port(31 downto 0) <= IP2Bus_Data_s;

test_port(255 downto 251) <= m_read & m_write & m_waitrequest & m_readdatavalid & MAC_DMA2Bus_Mst_Type;

test_port(244 downto 240) <= MAC_DMA2Bus_MstWr_Req & MAC_DMA2Bus_MstWr_sof_n & MAC_DMA2Bus_MstWr_eof_n & MAC_DMA2Bus_MstWr_src_rdy_n & Bus2MAC_DMA_MstWr_dst_rdy_n;
test_port(234 downto 230) <= MAC_DMA2Bus_MstRd_Req & Bus2MAC_DMA_MstRd_sof_n & Bus2MAC_DMA_MstRd_eof_n & Bus2MAC_DMA_MstRd_src_rdy_n & MAC_DMA2Bus_MstRd_dst_rdy_n;

test_port(142 downto 140) <= Bus2MAC_DMA_Mst_Cmplt & Bus2MAC_DMA_Mst_Error & Bus2MAC_DMA_Mst_Cmd_Timeout;

test_port(MAC_DMA2Bus_Mst_Length'length+120-1 downto 120) <= MAC_DMA2Bus_Mst_Length;

test_port(m_burstcount'length+110-1 downto 110) <= m_burstcount;
test_port(m_burstcounter'length+96-1 downto 96) <= m_burstcounter;
test_port(95 downto 64) <= m_address;
test_port(63 downto 32) <= m_writedata;
test_port(31 downto 0) <= m_readdata;

----  Component instantiations  ----

MAC_REG_16to32 : openMAC_16to32conv
  generic map (
       bus_address_width => C_S_AXI_MAC_REG_ADDR_WIDTH,
       gEndian => "little"
  )
  port map(
       bus_ack_rd => MAC_REG2Bus_RdAck,
       bus_ack_wr => MAC_REG2Bus_WrAck,
       bus_address => Bus2MAC_REG_Addr( C_S_AXI_MAC_REG_ADDR_WIDTH-1 downto 0 ),
       bus_byteenable => Bus2MAC_REG_BE( (C_S_AXI_MAC_REG_DATA_WIDTH/8)-1 downto 0 ),
       bus_read => Bus2MAC_REG_RNW,
       bus_readdata => MAC_REG2Bus_Data( C_S_AXI_MAC_REG_DATA_WIDTH-1 downto 0 ),
       bus_select => Bus2MAC_REG_CS(1),
       bus_write => Bus2MAC_REG_RNW_n,
       bus_writedata => Bus2MAC_REG_Data( C_S_AXI_MAC_REG_DATA_WIDTH-1 downto 0 ),
       clk => clk50,
       rst => Bus2MAC_REG_Reset,
       s_address => mac_address_full( C_S_AXI_MAC_REG_ADDR_WIDTH-1 downto 0 ),
       s_byteenable => mac_byteenable,
       s_chipselect => mac_chipselect,
       s_read => mac_read,
       s_readdata => mac_readdata,
       s_waitrequest => mac_waitrequest,
       s_write => mac_write,
       s_writedata => mac_writedata
  );

MAC_REG_AXI_SINGLE_SLAVE : axi_lite_ipif
  generic map (
       C_ARD_ADDR_RANGE_ARRAY => (C_MAC_REG_BASE,C_MAC_REG_HIGH,C_MAC_CMP_BASE,C_MAC_CMP_HIGH),
       C_ARD_NUM_CE_ARRAY => (1,1),
       C_DPHASE_TIMEOUT => C_S_AXI_MAC_REG_DPHASE_TIMEOUT,
       C_FAMILY => C_FAMILY,
       C_S_AXI_ADDR_WIDTH => C_S_AXI_MAC_REG_ADDR_WIDTH,
       C_S_AXI_DATA_WIDTH => C_S_AXI_MAC_REG_DATA_WIDTH,
       C_S_AXI_MIN_SIZE => C_MAC_REG_MINSIZE,
       C_USE_WSTRB => C_S_AXI_MAC_REG_USE_WSTRB
  )
  port map(
       Bus2IP_Addr => Bus2MAC_REG_Addr( C_S_AXI_MAC_REG_ADDR_WIDTH-1 downto 0 ),
       Bus2IP_BE => Bus2MAC_REG_BE( (C_S_AXI_MAC_REG_DATA_WIDTH/8)-1 downto 0 ),
       Bus2IP_CS => Bus2MAC_REG_CS_fast( 1 downto 0 ),
       Bus2IP_Clk => Bus2MAC_REG_Clk,
       Bus2IP_Data => Bus2MAC_REG_Data( C_S_AXI_MAC_REG_DATA_WIDTH-1 downto 0 ),
       Bus2IP_RNW => Bus2MAC_REG_RNW_fast,
       Bus2IP_RdCE => open,
       Bus2IP_Resetn => Bus2MAC_REG_Resetn,
       Bus2IP_WrCE => open,
       IP2Bus_Data => IP2Bus_Data_fast( C_S_AXI_MAC_REG_DATA_WIDTH-1 downto 0 ),
       IP2Bus_Error => IP2Bus_Error_s,
       IP2Bus_RdAck => IP2Bus_RdAck_fast,
       IP2Bus_WrAck => IP2Bus_WrAck_fast,
       S_AXI_ACLK => S_AXI_MAC_REG_ACLK,
       S_AXI_ARADDR => S_AXI_MAC_REG_ARADDR( C_S_AXI_MAC_REG_ADDR_WIDTH-1 downto 0 ),
       S_AXI_ARESETN => S_AXI_MAC_REG_ARESETN,
       S_AXI_ARREADY => S_AXI_MAC_REG_ARREADY,
       S_AXI_ARVALID => S_AXI_MAC_REG_ARVALID,
       S_AXI_AWADDR => S_AXI_MAC_REG_AWADDR( C_S_AXI_MAC_REG_ADDR_WIDTH-1 downto 0 ),
       S_AXI_AWREADY => S_AXI_MAC_REG_AWREADY,
       S_AXI_AWVALID => S_AXI_MAC_REG_AWVALID,
       S_AXI_BREADY => S_AXI_MAC_REG_BREADY,
       S_AXI_BRESP => S_AXI_MAC_REG_BRESP,
       S_AXI_BVALID => S_AXI_MAC_REG_BVALID,
       S_AXI_RDATA => S_AXI_MAC_REG_RDATA( C_S_AXI_MAC_REG_DATA_WIDTH-1 downto 0 ),
       S_AXI_RREADY => S_AXI_MAC_REG_RREADY,
       S_AXI_RRESP => S_AXI_MAC_REG_RRESP,
       S_AXI_RVALID => S_AXI_MAC_REG_RVALID,
       S_AXI_WDATA => S_AXI_MAC_REG_WDATA( C_S_AXI_MAC_REG_DATA_WIDTH-1 downto 0 ),
       S_AXI_WREADY => S_AXI_MAC_REG_WREADY,
       S_AXI_WSTRB => S_AXI_MAC_REG_WSTRB( (C_S_AXI_MAC_REG_DATA_WIDTH/8)-1 downto 0 ),
       S_AXI_WVALID => S_AXI_MAC_REG_WVALID
  );

THE_POWERLINK_IP_CORE : powerlink
  generic map (
       Simulate => booleanToInteger(false),
       endian_g => "little",
       gNumSmi => C_NUM_SMI,
       genABuf1_g => booleanToInteger(C_PDI_GEN_ASYNC_BUF_0),
       genABuf2_g => booleanToInteger(C_PDI_GEN_ASYNC_BUF_1),
       genEvent_g => booleanToInteger(C_PDI_GEN_EVENT),
       genInternalAp_g => booleanToInteger(C_GEN_AXI_BUS_IF),
       genIoBuf_g => booleanToInteger(false),
       genLedGadget_g => booleanToInteger(C_PDI_GEN_LED),
       genOnePdiClkDomain_g => booleanToInteger(false),
       genPdi_g => booleanToInteger(C_GEN_PDI),
       genSimpleIO_g => booleanToInteger(C_GEN_SIMPLE_IO),
       genSmiIO => booleanToInteger(false),
       genSpiAp_g => booleanToInteger(C_GEN_SPI_IF),
       genTimeSync_g => booleanToInteger(C_PDI_GEN_TIME_SYNC),
       gen_dma_observer_g => booleanToInteger(C_OBSERVER_ENABLE),
       iAsyBuf1Size_g => C_PDI_ASYNC_BUF_0,
       iAsyBuf2Size_g => C_PDI_ASYNC_BUF_1,
       iBufSizeLOG2_g => C_MAC_PKT_SIZE_LOG2,
       iBufSize_g => C_MAC_PKT_SIZE,
       iPdiRev_g => C_PDI_REV,
       iRpdo0BufSize_g => C_RPDO_0_BUF_SIZE,
       iRpdo1BufSize_g => C_RPDO_1_BUF_SIZE,
       iRpdo2BufSize_g => C_RPDO_2_BUF_SIZE,
       iRpdos_g => C_NUM_RPDO,
       iTpdoBufSize_g => C_TPDO_BUF_SIZE,
       iTpdos_g => C_NUM_TPDO,
       m_burstcount_const_g => booleanToInteger(true),
       m_burstcount_width_g => C_M_BURSTCOUNT_WIDTH,
       m_data_width_g => 32,
       m_rx_burst_size_g => C_MAC_DMA_BURST_SIZE_RX/4,
       m_rx_fifo_size_g => C_M_FIFO_SIZE_RX,
       m_tx_burst_size_g => C_MAC_DMA_BURST_SIZE_TX/4,
       m_tx_fifo_size_g => C_M_FIFO_SIZE_TX,
       papBigEnd_g => booleanToInteger(false),
       papDataWidth_g => C_PAP_DATA_WIDTH,
       papLowAct_g => booleanToInteger(C_PAP_LOW_ACT),
       pcpSysId => C_PCP_SYS_ID,
       pioValLen_g => C_PIO_VAL_LENGTH,
       pulseWidth2ndCmpTimer_g => C_PULSE_WIDTH_2nd_CMP_TIMER,
       spiBigEnd_g => booleanToInteger(false),
       spiCPHA_g => booleanToInteger(C_SPI_CPHA),
       spiCPOL_g => booleanToInteger(C_SPI_CPOL),
       use2ndCmpTimer_g => booleanToInteger(C_MAC_GEN_SECOND_TIMER),
       use2ndPhy_g => booleanToInteger(C_USE_2ND_PHY),
       useIntPacketBuf_g => booleanToInteger(C_MAC_PKT_EN),
       usePulse2ndCmpTimer_g => booleanToInteger(C_USE_PULSE_2nd_CMP_TIMER),
       useRmii_g => booleanToInteger(C_USE_RMII),
       useRxIntPacketBuf_g => booleanToInteger(C_MAC_PKT_RX_EN)
  )
  port map(
       ap_address => ap_address,
       ap_asyncIrq => ap_asyncIrq,
       ap_asyncIrq_n => ap_asyncIrq_n,
       ap_byteenable => ap_byteenable,
       ap_chipselect => ap_chipselect,
       ap_irq => open,
       ap_irq_n => open,
       ap_read => ap_read,
       ap_readdata => ap_readdata,
       ap_syncIrq => ap_syncIrq,
       ap_syncIrq_n => ap_syncIrq_n,
       ap_waitrequest => ap_waitrequest,
       ap_write => ap_write,
       ap_writedata => ap_writedata,
       clk50 => clk50,
       clkAp => clkAp,
       clkEth => clk100,
       clkPcp => clkPcp,
       led_error => led_error,
       led_gpo => led_gpo,
       led_opt => led_opt,
       led_phyAct => led_phyAct,
       led_phyLink => led_phyLink,
       led_status => led_status,
       m_address => m_address,
       m_burstcount => m_burstcount( C_M_BURSTCOUNT_WIDTH-1 downto 0 ),
       m_burstcounter => m_burstcounter( C_M_BURSTCOUNT_WIDTH-1 downto 0 ),
       m_byteenable => m_byteenable( 3 downto 0 ),
       m_clk => m_clk,
       m_read => m_read,
       m_readdata => m_readdata( 31 downto 0 ),
       m_readdatavalid => m_readdatavalid,
       m_waitrequest => m_waitrequest,
       m_write => m_write,
       m_writedata => m_writedata( 31 downto 0 ),
       mac_address => mac_address,
       mac_byteenable => mac_byteenable,
       mac_chipselect => mac_chipselect,
       mac_irq => mac_irq_s,
       mac_read => mac_read,
       mac_readdata => mac_readdata,
       mac_waitrequest => mac_waitrequest,
       mac_write => mac_write,
       mac_writedata => mac_writedata,
       mbf_address => mbf_address( C_MAC_PKT_SIZE_LOG2-3 downto 0 ),
       mbf_byteenable => mbf_byteenable,
       mbf_chipselect => mbf_chipselect,
       mbf_read => mbf_read,
       mbf_readdata => mbf_readdata,
       mbf_waitrequest => mbf_waitrequest,
       mbf_write => mbf_write,
       mbf_writedata => mbf_writedata,
       pap_ack => pap_ack,
       pap_ack_n => pap_ack_n,
       pap_addr => pap_addr,
       pap_be => pap_be( C_PAP_DATA_WIDTH/8-1 downto 0 ),
       pap_be_n => pap_be_n( C_PAP_DATA_WIDTH/8-1 downto 0 ),
       pap_cs => pap_cs,
       pap_cs_n => pap_cs_n,
       pap_data => open,
       pap_data_I => pap_data_I( C_PAP_DATA_WIDTH-1 downto 0 ),
       pap_data_O => pap_data_O( C_PAP_DATA_WIDTH-1 downto 0 ),
       pap_data_T => pap_data_T,
       pap_gpio => open,
       pap_gpio_I => pap_gpio_I,
       pap_gpio_O => pap_gpio_O,
       pap_gpio_T => pap_gpio_T,
       pap_rd => pap_rd,
       pap_rd_n => pap_rd_n,
       pap_wr => pap_wr,
       pap_wr_n => pap_wr_n,
       pcp_address => pcp_address,
       pcp_byteenable => pcp_byteenable,
       pcp_chipselect => pcp_chipselect,
       pcp_read => pcp_read,
       pcp_readdata => pcp_readdata,
       pcp_waitrequest => pcp_waitrequest,
       pcp_write => pcp_write,
       pcp_writedata => pcp_writedata,
       phy0_Rst_n => phy0_Rst_n,
       phy0_RxDat => phy0_RxDat,
       phy0_RxDv => phy0_RxDv,
       phy0_RxErr => phy0_RxErr,
       phy0_SMIClk => phy0_SMIClk,
       phy0_SMIDat => open,
       phy0_SMIDat_I => phy0_SMIDat_I,
       phy0_SMIDat_O => phy0_SMIDat_O,
       phy0_SMIDat_T => phy0_SMIDat_T,
       phy0_TxDat => phy0_TxDat,
       phy0_TxEn => phy0_TxEn,
       phy0_link => phy0_link,
       phy1_Rst_n => phy1_Rst_n,
       phy1_RxDat => phy1_RxDat,
       phy1_RxDv => phy1_RxDv,
       phy1_RxErr => phy1_RxErr,
       phy1_SMIClk => phy1_SMIClk,
       phy1_SMIDat => open,
       phy1_SMIDat_I => phy1_SMIDat_I,
       phy1_SMIDat_O => phy1_SMIDat_O,
       phy1_SMIDat_T => phy1_SMIDat_T,
       phy1_TxDat => phy1_TxDat,
       phy1_TxEn => phy1_TxEn,
       phy1_link => phy1_link,
       phyMii0_RxClk => phyMii0_RxClk,
       phyMii0_RxDat => phyMii0_RxDat,
       phyMii0_RxDv => phyMii0_RxDv,
       phyMii0_RxEr => phyMii0_RxEr,
       phyMii0_TxClk => phyMii0_TxClk,
       phyMii0_TxDat => phyMii0_TxDat,
       phyMii0_TxEn => phyMii0_TxEn,
       phyMii0_TxEr => phyMii0_TxEr,
       phyMii1_RxClk => phyMii1_RxClk,
       phyMii1_RxDat => phyMii1_RxDat,
       phyMii1_RxDv => phyMii1_RxDv,
       phyMii1_RxEr => phyMii1_RxEr,
       phyMii1_TxClk => phyMii1_TxClk,
       phyMii1_TxDat => phyMii1_TxDat,
       phyMii1_TxEn => phyMii1_TxEn,
       phyMii1_TxEr => phyMii1_TxEr,
       phy_Rst_n => phy_Rst_n,
       phy_SMIClk => phy_SMIClk,
       phy_SMIDat => open,
       phy_SMIDat_I => phy_SMIDat_I,
       phy_SMIDat_O => phy_SMIDat_O,
       phy_SMIDat_T => phy_SMIDat_T,
       pio_operational => pio_operational,
       pio_pconfig => pio_pconfig,
       pio_portInLatch => pio_portInLatch,
       pio_portOutValid => pio_portOutValid,
       pio_portio => open,
       pio_portio_I => pio_portio_I,
       pio_portio_O => pio_portio_O,
       pio_portio_T => pio_portio_T,
       pkt_clk => pkt_clk,
       rst => rst,
       rstAp => rstAp,
       rstPcp => rstPcp,
       smp_address => smp_address,
       smp_byteenable => smp_byteenable,
       smp_read => smp_read,
       smp_readdata => smp_readdata,
       smp_waitrequest => smp_waitrequest,
       smp_write => smp_write,
       smp_writedata => smp_writedata,
       spi_clk => spi_clk,
       spi_miso => spi_miso,
       spi_mosi => spi_mosi,
       spi_sel_n => spi_sel_n,
       tcp_address => tcp_address,
       tcp_byteenable => tcp_byteenable,
       tcp_chipselect => tcp_chipselect,
       tcp_irq => tcp_irq_s,
       tcp_read => tcp_read,
       tcp_readdata => tcp_readdata,
       tcp_waitrequest => tcp_waitrequest,
       tcp_write => tcp_write,
       tcp_writedata => tcp_writedata
  );

MAC_DMA_areset <= not(M_AXI_MAC_DMA_aresetn);

Bus2MAC_REG_RNW_n <= not(Bus2MAC_REG_RNW);

Bus2MAC_REG_Reset <= not(Bus2MAC_REG_Resetn);

rstPcp <= Bus2SMP_PCP_Reset or Bus2PDI_PCP_Reset or Bus2MAC_PKT_Reset;

rstAp <= Bus2PDI_AP_Reset;

rst <= Bus2MAC_REG_Reset;

macRegClkXing : clkXing
  generic map (
       gCsNum => 2,
       gDataWidth => C_S_AXI_MAC_REG_DATA_WIDTH
  )
  port map(
       iArst => Bus2MAC_REG_Reset,
       iFastClk => Bus2MAC_REG_Clk,
       iFastCs => Bus2MAC_REG_CS_fast( 1 downto 0 ),
       iFastRNW => Bus2MAC_REG_RNW_fast,
       iSlowClk => clk50,
       iSlowRdAck => IP2Bus_RdAck_s,
       iSlowReaddata => IP2Bus_Data_s( C_S_AXI_MAC_REG_DATA_WIDTH-1 downto 0 ),
       iSlowWrAck => IP2Bus_WrAck_s,
       oFastRdAck => IP2Bus_RdAck_fast,
       oFastReaddata => IP2Bus_Data_fast( C_S_AXI_MAC_REG_DATA_WIDTH-1 downto 0 ),
       oFastWrAck => IP2Bus_WrAck_fast,
       oSlowCs => Bus2MAC_REG_CS( 1 downto 0 ),
       oSlowRNW => Bus2MAC_REG_RNW
  );


---- Power , ground assignment ----

GND <= GND_CONSTANT;
VCC <= VCC_CONSTANT;
MAC_REG2Bus_Error <= GND;

---- Terminal assignment ----

    -- Output\buffer terminals
    mac_irq <= mac_irq_s;
    tcp_irq <= tcp_irq_s;


----  Generate statements  ----

genMacDmaPlbBurst : if C_DMA_EN = TRUE generate
begin
  MAC_DMA_AXI_BURST_MASTER : axi_master_burst
    generic map (
         C_ADDR_PIPE_DEPTH => 1,
         C_FAMILY => C_FAMILY,
         C_LENGTH_WIDTH => C_M_AXI_MAC_DMA_LENGTH_WIDTH,
         C_MAX_BURST_LEN => C_M_AXI_MAC_DMA_MAX_BURST_LEN,
         C_M_AXI_ADDR_WIDTH => C_M_AXI_MAC_DMA_ADDR_WIDTH,
         C_M_AXI_DATA_WIDTH => C_M_AXI_MAC_DMA_DATA_WIDTH,
         C_NATIVE_DATA_WIDTH => C_M_AXI_MAC_DMA_NATIVE_DWIDTH
    )
    port map(
         bus2ip_mst_cmd_timeout => bus2MAC_DMA_mst_cmd_timeout,
         bus2ip_mst_cmdack => bus2MAC_DMA_mst_cmdack,
         bus2ip_mst_cmplt => bus2MAC_DMA_mst_cmplt,
         bus2ip_mst_error => bus2MAC_DMA_mst_error,
         bus2ip_mst_rearbitrate => bus2MAC_DMA_mst_rearbitrate,
         bus2ip_mstrd_d => bus2MAC_DMA_mstrd_d( C_M_AXI_MAC_DMA_NATIVE_DWIDTH-1 downto 0 ),
         bus2ip_mstrd_eof_n => bus2MAC_DMA_mstrd_eof_n,
         bus2ip_mstrd_rem => bus2MAC_DMA_mstrd_rem( (C_M_AXI_MAC_DMA_NATIVE_DWIDTH/8)-1 downto 0 ),
         bus2ip_mstrd_sof_n => bus2MAC_DMA_mstrd_sof_n,
         bus2ip_mstrd_src_dsc_n => bus2MAC_DMA_mstrd_src_dsc_n,
         bus2ip_mstrd_src_rdy_n => bus2MAC_DMA_mstrd_src_rdy_n,
         bus2ip_mstwr_dst_dsc_n => bus2MAC_DMA_mstwr_dst_dsc_n,
         bus2ip_mstwr_dst_rdy_n => bus2MAC_DMA_mstwr_dst_rdy_n,
         ip2bus_mst_addr => MAC_DMA2bus_mst_addr( C_M_AXI_MAC_DMA_ADDR_WIDTH-1 downto 0 ),
         ip2bus_mst_be => MAC_DMA2bus_mst_be( (C_M_AXI_MAC_DMA_NATIVE_DWIDTH/8)-1 downto 0 ),
         ip2bus_mst_length => MAC_DMA2bus_mst_length( C_M_AXI_MAC_DMA_LENGTH_WIDTH-1 downto 0 ),
         ip2bus_mst_lock => MAC_DMA2bus_mst_lock,
         ip2bus_mst_reset => MAC_DMA2bus_mst_reset,
         ip2bus_mst_type => MAC_DMA2bus_mst_type,
         ip2bus_mstrd_dst_dsc_n => MAC_DMA2bus_mstrd_dst_dsc_n,
         ip2bus_mstrd_dst_rdy_n => MAC_DMA2bus_mstrd_dst_rdy_n,
         ip2bus_mstrd_req => MAC_DMA2bus_mstrd_req,
         ip2bus_mstwr_d => MAC_DMA2bus_mstwr_d( C_M_AXI_MAC_DMA_NATIVE_DWIDTH-1 downto 0 ),
         ip2bus_mstwr_eof_n => MAC_DMA2bus_mstwr_eof_n,
         ip2bus_mstwr_rem => MAC_DMA2bus_mstwr_rem( (C_M_AXI_MAC_DMA_NATIVE_DWIDTH/8)-1 downto 0 ),
         ip2bus_mstwr_req => MAC_DMA2bus_mstwr_req,
         ip2bus_mstwr_sof_n => MAC_DMA2bus_mstwr_sof_n,
         ip2bus_mstwr_src_dsc_n => MAC_DMA2bus_mstwr_src_dsc_n,
         ip2bus_mstwr_src_rdy_n => MAC_DMA2bus_mstwr_src_rdy_n,
         m_axi_aclk => M_AXI_MAC_DMA_aclk,
         m_axi_araddr => M_AXI_MAC_DMA_araddr( C_M_AXI_MAC_DMA_ADDR_WIDTH-1 downto 0 ),
         m_axi_arburst => M_AXI_MAC_DMA_arburst,
         m_axi_arcache => M_AXI_MAC_DMA_arcache,
         m_axi_aresetn => M_AXI_MAC_DMA_aresetn,
         m_axi_arlen => M_AXI_MAC_DMA_arlen,
         m_axi_arprot => M_AXI_MAC_DMA_arprot,
         m_axi_arready => M_AXI_MAC_DMA_arready,
         m_axi_arsize => M_AXI_MAC_DMA_arsize,
         m_axi_arvalid => M_AXI_MAC_DMA_arvalid,
         m_axi_awaddr => M_AXI_MAC_DMA_awaddr( C_M_AXI_MAC_DMA_ADDR_WIDTH-1 downto 0 ),
         m_axi_awburst => M_AXI_MAC_DMA_awburst,
         m_axi_awcache => M_AXI_MAC_DMA_awcache,
         m_axi_awlen => M_AXI_MAC_DMA_awlen,
         m_axi_awprot => M_AXI_MAC_DMA_awprot,
         m_axi_awready => M_AXI_MAC_DMA_awready,
         m_axi_awsize => M_AXI_MAC_DMA_awsize,
         m_axi_awvalid => M_AXI_MAC_DMA_awvalid,
         m_axi_bready => M_AXI_MAC_DMA_bready,
         m_axi_bresp => M_AXI_MAC_DMA_bresp,
         m_axi_bvalid => M_AXI_MAC_DMA_bvalid,
         m_axi_rdata => M_AXI_MAC_DMA_rdata( C_M_AXI_MAC_DMA_DATA_WIDTH-1 downto 0 ),
         m_axi_rlast => M_AXI_MAC_DMA_rlast,
         m_axi_rready => M_AXI_MAC_DMA_rready,
         m_axi_rresp => M_AXI_MAC_DMA_rresp,
         m_axi_rvalid => M_AXI_MAC_DMA_rvalid,
         m_axi_wdata => M_AXI_MAC_DMA_wdata( C_M_AXI_MAC_DMA_DATA_WIDTH-1 downto 0 ),
         m_axi_wlast => M_AXI_MAC_DMA_wlast,
         m_axi_wready => M_AXI_MAC_DMA_wready,
         m_axi_wstrb => M_AXI_MAC_DMA_wstrb( (C_M_AXI_MAC_DMA_DATA_WIDTH/8)-1 downto 0 ),
         m_axi_wvalid => M_AXI_MAC_DMA_wvalid,
         md_error => M_AXI_MAC_DMA_md_error
    );
end generate genMacDmaPlbBurst;

genThePlbMaster : if C_DMA_EN = TRUE generate
begin
  THE_IPIF_MASTER_HANDLER : ipif_master_handler
    generic map (
         C_MAC_DMA_IPIF_AWIDTH => C_M_AXI_MAC_DMA_ADDR_WIDTH,
         C_MAC_DMA_IPIF_NATIVE_DWIDTH => C_M_AXI_MAC_DMA_NATIVE_DWIDTH,
         dma_highadr_g => m_address'high,
         gen_rx_fifo_g => not C_RX_INT_PKT,
         gen_tx_fifo_g => not C_TX_INT_PKT,
         m_burstcount_width_g => C_M_BURSTCOUNT_WIDTH
    )
    port map(
         Bus2MAC_DMA_MstRd_d => bus2MAC_DMA_mstrd_d( C_M_AXI_MAC_DMA_NATIVE_DWIDTH-1 downto 0 ),
         Bus2MAC_DMA_MstRd_eof_n => bus2MAC_DMA_mstrd_eof_n,
         Bus2MAC_DMA_MstRd_rem => bus2MAC_DMA_mstrd_rem( (C_M_AXI_MAC_DMA_NATIVE_DWIDTH/8)-1 downto 0 ),
         Bus2MAC_DMA_MstRd_sof_n => bus2MAC_DMA_mstrd_sof_n,
         Bus2MAC_DMA_MstRd_src_dsc_n => bus2MAC_DMA_mstrd_src_dsc_n,
         Bus2MAC_DMA_MstRd_src_rdy_n => bus2MAC_DMA_mstrd_src_rdy_n,
         Bus2MAC_DMA_MstWr_dst_dsc_n => bus2MAC_DMA_mstwr_dst_dsc_n,
         Bus2MAC_DMA_MstWr_dst_rdy_n => bus2MAC_DMA_mstwr_dst_rdy_n,
         Bus2MAC_DMA_Mst_CmdAck => bus2MAC_DMA_mst_cmdack,
         Bus2MAC_DMA_Mst_Cmd_Timeout => bus2MAC_DMA_mst_cmd_timeout,
         Bus2MAC_DMA_Mst_Cmplt => bus2MAC_DMA_mst_cmplt,
         Bus2MAC_DMA_Mst_Error => bus2MAC_DMA_mst_error,
         Bus2MAC_DMA_Mst_Rearbitrate => bus2MAC_DMA_mst_rearbitrate,
         MAC_DMA2Bus_MstRd_Req => MAC_DMA2bus_mstrd_req,
         MAC_DMA2Bus_MstRd_dst_dsc_n => MAC_DMA2bus_mstrd_dst_dsc_n,
         MAC_DMA2Bus_MstRd_dst_rdy_n => MAC_DMA2bus_mstrd_dst_rdy_n,
         MAC_DMA2Bus_MstWr_Req => MAC_DMA2bus_mstwr_req,
         MAC_DMA2Bus_MstWr_d => MAC_DMA2Bus_MstWr_d( C_M_AXI_MAC_DMA_NATIVE_DWIDTH-1 downto 0 ),
         MAC_DMA2Bus_MstWr_eof_n => MAC_DMA2bus_mstwr_eof_n,
         MAC_DMA2Bus_MstWr_rem => MAC_DMA2bus_mstwr_rem( (C_M_AXI_MAC_DMA_NATIVE_DWIDTH/8)-1 downto 0 ),
         MAC_DMA2Bus_MstWr_sof_n => MAC_DMA2bus_mstwr_sof_n,
         MAC_DMA2Bus_MstWr_src_dsc_n => MAC_DMA2bus_mstwr_src_dsc_n,
         MAC_DMA2Bus_MstWr_src_rdy_n => MAC_DMA2bus_mstwr_src_rdy_n,
         MAC_DMA2Bus_Mst_Addr => MAC_DMA2bus_mst_addr( C_M_AXI_MAC_DMA_ADDR_WIDTH-1 downto 0 ),
         MAC_DMA2Bus_Mst_BE => MAC_DMA2bus_mst_be( (C_M_AXI_MAC_DMA_NATIVE_DWIDTH/8)-1 downto 0 ),
         MAC_DMA2Bus_Mst_Length => MAC_DMA2bus_mst_length( C_M_AXI_MAC_DMA_LENGTH_WIDTH-1 downto 0 ),
         MAC_DMA2Bus_Mst_Lock => MAC_DMA2bus_mst_lock,
         MAC_DMA2Bus_Mst_Reset => MAC_DMA2bus_mst_reset,
         MAC_DMA2Bus_Mst_Type => MAC_DMA2bus_mst_type,
         MAC_DMA_CLK => M_AXI_MAC_DMA_aclk,
         MAC_DMA_Rst => MAC_DMA_areset,
         m_address => m_address( 31 downto 0 ),
         m_burstcount => m_burstcount( C_M_BURSTCOUNT_WIDTH-1 downto 0 ),
         m_burstcounter => m_burstcounter( C_M_BURSTCOUNT_WIDTH-1 downto 0 ),
         m_byteenable => m_byteenable,
         m_clk => m_clk,
         m_read => m_read,
         m_readdata => m_readdata,
         m_readdatavalid => m_readdatavalid,
         m_waitrequest => m_waitrequest,
         m_write => m_write,
         m_writedata => m_writedata
    );
end generate genThePlbMaster;

genMacPktPLbSingleSlave : if C_PKT_BUF_EN generate
begin
  MAC_PKT_AXI_SINGLE_SLAVE : axi_lite_ipif
    generic map (
         C_ARD_ADDR_RANGE_ARRAY => (C_MAC_PKT_BASE,C_MAC_PKT_HIGH),
         C_ARD_NUM_CE_ARRAY => (0=>1),
         C_DPHASE_TIMEOUT => C_S_AXI_MAC_PKT_DPHASE_TIMEOUT,
         C_FAMILY => C_FAMILY,
         C_S_AXI_ADDR_WIDTH => C_S_AXI_MAC_PKT_ADDR_WIDTH,
         C_S_AXI_DATA_WIDTH => C_S_AXI_MAC_PKT_DATA_WIDTH,
         C_S_AXI_MIN_SIZE => C_MAC_PKT_MINSIZE,
         C_USE_WSTRB => C_S_AXI_MAC_PKT_USE_WSTRB
    )
    port map(
         Bus2IP_Addr => Bus2MAC_PKT_Addr( C_S_AXI_MAC_PKT_ADDR_WIDTH-1 downto 0 ),
         Bus2IP_BE => Bus2MAC_PKT_BE( (C_S_AXI_MAC_PKT_DATA_WIDTH/8)-1 downto 0 ),
         Bus2IP_CS => Bus2MAC_PKT_CS( 0 downto 0 ),
         Bus2IP_Clk => Bus2MAC_PKT_Clk,
         Bus2IP_Data => Bus2MAC_PKT_Data( C_S_AXI_MAC_PKT_DATA_WIDTH-1 downto 0 ),
         Bus2IP_RNW => Bus2MAC_PKT_RNW,
         Bus2IP_RdCE => open,
         Bus2IP_Resetn => Bus2MAC_PKT_Resetn,
         Bus2IP_WrCE => open,
         IP2Bus_Data => MAC_PKT2Bus_Data( C_S_AXI_MAC_PKT_DATA_WIDTH-1 downto 0 ),
         IP2Bus_Error => MAC_PKT2Bus_Error,
         IP2Bus_RdAck => MAC_PKT2Bus_RdAck,
         IP2Bus_WrAck => MAC_PKT2Bus_WrAck,
         S_AXI_ACLK => S_AXI_MAC_PKT_ACLK,
         S_AXI_ARADDR => S_AXI_MAC_PKT_ARADDR( C_S_AXI_MAC_PKT_ADDR_WIDTH-1 downto 0 ),
         S_AXI_ARESETN => S_AXI_MAC_PKT_ARESETN,
         S_AXI_ARREADY => S_AXI_MAC_PKT_ARREADY,
         S_AXI_ARVALID => S_AXI_MAC_PKT_ARVALID,
         S_AXI_AWADDR => S_AXI_MAC_PKT_AWADDR( C_S_AXI_MAC_PKT_ADDR_WIDTH-1 downto 0 ),
         S_AXI_AWREADY => S_AXI_MAC_PKT_AWREADY,
         S_AXI_AWVALID => S_AXI_MAC_PKT_AWVALID,
         S_AXI_BREADY => S_AXI_MAC_PKT_BREADY,
         S_AXI_BRESP => S_AXI_MAC_PKT_BRESP,
         S_AXI_BVALID => S_AXI_MAC_PKT_BVALID,
         S_AXI_RDATA => S_AXI_MAC_PKT_RDATA( C_S_AXI_MAC_PKT_DATA_WIDTH-1 downto 0 ),
         S_AXI_RREADY => S_AXI_MAC_PKT_RREADY,
         S_AXI_RRESP => S_AXI_MAC_PKT_RRESP,
         S_AXI_RVALID => S_AXI_MAC_PKT_RVALID,
         S_AXI_WDATA => S_AXI_MAC_PKT_WDATA( C_S_AXI_MAC_PKT_DATA_WIDTH-1 downto 0 ),
         S_AXI_WREADY => S_AXI_MAC_PKT_WREADY,
         S_AXI_WSTRB => S_AXI_MAC_PKT_WSTRB( (C_S_AXI_MAC_PKT_DATA_WIDTH/8)-1 downto 0 ),
         S_AXI_WVALID => S_AXI_MAC_PKT_WVALID
    );
end generate genMacPktPLbSingleSlave;

genPdiPcp : if (C_GEN_PDI) generate
begin
  PDI_PCP_AXI_SINGLE_SLAVE : axi_lite_ipif
    generic map (
         C_ARD_ADDR_RANGE_ARRAY => (C_PDI_PCP_BASE,C_PDI_PCP_HIGH),
         C_ARD_NUM_CE_ARRAY => (0=>1),
         C_DPHASE_TIMEOUT => C_S_AXI_PDI_PCP_DPHASE_TIMEOUT,
         C_FAMILY => C_FAMILY,
         C_S_AXI_ADDR_WIDTH => C_S_AXI_PDI_PCP_ADDR_WIDTH,
         C_S_AXI_DATA_WIDTH => C_S_AXI_PDI_PCP_DATA_WIDTH,
         C_S_AXI_MIN_SIZE => C_PDI_PCP_MINSIZE,
         C_USE_WSTRB => C_S_AXI_PDI_PCP_USE_WSTRB
    )
    port map(
         Bus2IP_Addr => Bus2PDI_PCP_Addr( C_S_AXI_PDI_PCP_ADDR_WIDTH-1 downto 0 ),
         Bus2IP_BE => Bus2PDI_PCP_BE( (C_S_AXI_PDI_PCP_DATA_WIDTH/8)-1 downto 0 ),
         Bus2IP_CS => Bus2PDI_PCP_CS( 0 downto 0 ),
         Bus2IP_Clk => Bus2PDI_PCP_Clk,
         Bus2IP_Data => Bus2PDI_PCP_Data( C_S_AXI_PDI_PCP_DATA_WIDTH-1 downto 0 ),
         Bus2IP_RNW => Bus2PDI_PCP_RNW,
         Bus2IP_RdCE => open,
         Bus2IP_Resetn => Bus2PDI_PCP_Resetn,
         Bus2IP_WrCE => open,
         IP2Bus_Data => PDI_PCP2Bus_Data( C_S_AXI_PDI_PCP_DATA_WIDTH-1 downto 0 ),
         IP2Bus_Error => PDI_PCP2Bus_Error,
         IP2Bus_RdAck => PDI_PCP2Bus_RdAck,
         IP2Bus_WrAck => PDI_PCP2Bus_WrAck,
         S_AXI_ACLK => S_AXI_PDI_PCP_ACLK,
         S_AXI_ARADDR => S_AXI_PDI_PCP_ARADDR( C_S_AXI_PDI_PCP_ADDR_WIDTH-1 downto 0 ),
         S_AXI_ARESETN => S_AXI_PDI_PCP_ARESETN,
         S_AXI_ARREADY => S_AXI_PDI_PCP_ARREADY,
         S_AXI_ARVALID => S_AXI_PDI_PCP_ARVALID,
         S_AXI_AWADDR => S_AXI_PDI_PCP_AWADDR( C_S_AXI_PDI_PCP_ADDR_WIDTH-1 downto 0 ),
         S_AXI_AWREADY => S_AXI_PDI_PCP_AWREADY,
         S_AXI_AWVALID => S_AXI_PDI_PCP_AWVALID,
         S_AXI_BREADY => S_AXI_PDI_PCP_BREADY,
         S_AXI_BRESP => S_AXI_PDI_PCP_BRESP,
         S_AXI_BVALID => S_AXI_PDI_PCP_BVALID,
         S_AXI_RDATA => S_AXI_PDI_PCP_RDATA( C_S_AXI_PDI_PCP_DATA_WIDTH-1 downto 0 ),
         S_AXI_RREADY => S_AXI_PDI_PCP_RREADY,
         S_AXI_RRESP => S_AXI_PDI_PCP_RRESP,
         S_AXI_RVALID => S_AXI_PDI_PCP_RVALID,
         S_AXI_WDATA => S_AXI_PDI_PCP_WDATA( C_S_AXI_PDI_PCP_DATA_WIDTH-1 downto 0 ),
         S_AXI_WREADY => S_AXI_PDI_PCP_WREADY,
         S_AXI_WSTRB => S_AXI_PDI_PCP_WSTRB( (C_S_AXI_PDI_PCP_DATA_WIDTH/8)-1 downto 0 ),
         S_AXI_WVALID => S_AXI_PDI_PCP_WVALID
    );
end generate genPdiPcp;

genPcpPdiLink : if C_GEN_PDI generate
begin
  --pdi_pcp assignments
clkPcp <= Bus2PDI_PCP_Clk;
Bus2PDI_PCP_Reset <= not Bus2PDI_PCP_Resetn;
pcp_writedata <= Bus2PDI_PCP_Data;
--    Bus2MAC_PKT_Data(7 downto 0) & Bus2MAC_PKT_Data(15 downto 8) &
--    Bus2MAC_PKT_Data(23 downto 16) & Bus2MAC_PKT_Data(31 downto 24);
pcp_read <= Bus2PDI_PCP_RNW;
pcp_write <= not Bus2PDI_PCP_RNW;
pcp_chipselect <= Bus2PDI_PCP_CS(0);
pcp_byteenable <= Bus2PDI_PCP_BE;
pcp_address <= Bus2PDI_PCP_Addr(14 downto 2);

PDI_PCP2Bus_Data <= pcp_readdata;
PDI_PCP2Bus_RdAck <= pcp_chipselect and pcp_read and not pcp_waitrequest;
PDI_PCP2Bus_WrAck <= pcp_chipselect and pcp_write and not pcp_waitrequest;
PDI_PCP2Bus_Error <= '0';
end generate genPcpPdiLink;

genPdiAp : if (C_GEN_AXI_BUS_IF) generate
begin
  PDI_AP_AXI_SINGLE_SLAVE : axi_lite_ipif
    generic map (
         C_ARD_ADDR_RANGE_ARRAY => (C_PDI_AP_BASE,C_PDI_AP_HIGH),
         C_ARD_NUM_CE_ARRAY => (0=>1),
         C_DPHASE_TIMEOUT => C_S_AXI_PDI_AP_DPHASE_TIMEOUT,
         C_FAMILY => C_FAMILY,
         C_S_AXI_ADDR_WIDTH => C_S_AXI_PDI_AP_ADDR_WIDTH,
         C_S_AXI_DATA_WIDTH => C_S_AXI_PDI_AP_DATA_WIDTH,
         C_S_AXI_MIN_SIZE => C_PDI_AP_MINSIZE,
         C_USE_WSTRB => C_S_AXI_PDI_AP_USE_WSTRB
    )
    port map(
         Bus2IP_Addr => Bus2PDI_AP_Addr( C_S_AXI_PDI_AP_ADDR_WIDTH-1 downto 0 ),
         Bus2IP_BE => Bus2PDI_AP_BE( (C_S_AXI_PDI_AP_DATA_WIDTH/8)-1 downto 0 ),
         Bus2IP_CS => Bus2PDI_AP_CS( 0 downto 0 ),
         Bus2IP_Clk => Bus2PDI_AP_Clk,
         Bus2IP_Data => Bus2PDI_AP_Data( C_S_AXI_PDI_AP_DATA_WIDTH-1 downto 0 ),
         Bus2IP_RNW => Bus2PDI_AP_RNW,
         Bus2IP_RdCE => open,
         Bus2IP_Resetn => Bus2PDI_AP_Resetn,
         Bus2IP_WrCE => open,
         IP2Bus_Data => PDI_AP2Bus_Data( C_S_AXI_PDI_AP_DATA_WIDTH-1 downto 0 ),
         IP2Bus_Error => PDI_AP2Bus_Error,
         IP2Bus_RdAck => PDI_AP2Bus_RdAck,
         IP2Bus_WrAck => PDI_AP2Bus_WrAck,
         S_AXI_ACLK => S_AXI_PDI_AP_ACLK,
         S_AXI_ARADDR => S_AXI_PDI_AP_ARADDR( C_S_AXI_PDI_AP_ADDR_WIDTH-1 downto 0 ),
         S_AXI_ARESETN => S_AXI_PDI_AP_ARESETN,
         S_AXI_ARREADY => S_AXI_PDI_AP_ARREADY,
         S_AXI_ARVALID => S_AXI_PDI_AP_ARVALID,
         S_AXI_AWADDR => S_AXI_PDI_AP_AWADDR( C_S_AXI_PDI_AP_ADDR_WIDTH-1 downto 0 ),
         S_AXI_AWREADY => S_AXI_PDI_AP_AWREADY,
         S_AXI_AWVALID => S_AXI_PDI_AP_AWVALID,
         S_AXI_BREADY => S_AXI_PDI_AP_BREADY,
         S_AXI_BRESP => S_AXI_PDI_AP_BRESP,
         S_AXI_BVALID => S_AXI_PDI_AP_BVALID,
         S_AXI_RDATA => S_AXI_PDI_AP_RDATA( C_S_AXI_PDI_AP_DATA_WIDTH-1 downto 0 ),
         S_AXI_RREADY => S_AXI_PDI_AP_RREADY,
         S_AXI_RRESP => S_AXI_PDI_AP_RRESP,
         S_AXI_RVALID => S_AXI_PDI_AP_RVALID,
         S_AXI_WDATA => S_AXI_PDI_AP_WDATA( C_S_AXI_PDI_AP_DATA_WIDTH-1 downto 0 ),
         S_AXI_WREADY => S_AXI_PDI_AP_WREADY,
         S_AXI_WSTRB => S_AXI_PDI_AP_WSTRB( (C_S_AXI_PDI_AP_DATA_WIDTH/8)-1 downto 0 ),
         S_AXI_WVALID => S_AXI_PDI_AP_WVALID
    );
end generate genPdiAp;

genApPdiLink : if C_GEN_PDI generate
begin
  --ap_pcp assignments
clkAp <= Bus2PDI_AP_Clk;
Bus2PDI_AP_Reset <= not Bus2PDI_AP_Resetn;
ap_writedata <= Bus2PDI_AP_Data;
--    Bus2MAC_PKT_Data(7 downto 0) & Bus2MAC_PKT_Data(15 downto 8) &
--    Bus2MAC_PKT_Data(23 downto 16) & Bus2MAC_PKT_Data(31 downto 24);
ap_read <= Bus2PDI_AP_RNW;
ap_write <= not Bus2PDI_AP_RNW;
ap_chipselect <= Bus2PDI_AP_CS(0);
ap_byteenable <= Bus2PDI_AP_BE;
ap_address <= Bus2PDI_AP_Addr(14 downto 2);

PDI_AP2Bus_Data <= ap_readdata;
PDI_AP2Bus_RdAck <= ap_chipselect and ap_read and not ap_waitrequest;
PDI_AP2Bus_WrAck <= ap_chipselect and ap_write and not ap_waitrequest;
PDI_AP2Bus_Error <= '0';
end generate genApPdiLink;

genSmpIo : if (C_GEN_SIMPLE_IO) generate
begin
  SMP_IO_AXI_SINGLE_SLAVE : axi_lite_ipif
    generic map (
         C_ARD_ADDR_RANGE_ARRAY => (C_SMP_PCP_BASE,C_SMP_PCP_HIGH),
         C_ARD_NUM_CE_ARRAY => (0=>1),
         C_DPHASE_TIMEOUT => C_S_AXI_SMP_PCP_DPHASE_TIMEOUT,
         C_FAMILY => C_FAMILY,
         C_S_AXI_ADDR_WIDTH => C_S_AXI_SMP_PCP_ADDR_WIDTH,
         C_S_AXI_DATA_WIDTH => C_S_AXI_SMP_PCP_DATA_WIDTH,
         C_S_AXI_MIN_SIZE => C_SMP_PCP_MINSIZE,
         C_USE_WSTRB => C_S_AXI_SMP_PCP_USE_WSTRB
    )
    port map(
         Bus2IP_Addr => Bus2SMP_PCP_Addr( C_S_AXI_SMP_PCP_ADDR_WIDTH-1 downto 0 ),
         Bus2IP_BE => Bus2SMP_PCP_BE( (C_S_AXI_SMP_PCP_DATA_WIDTH/8)-1 downto 0 ),
         Bus2IP_CS => Bus2SMP_PCP_CS( 0 downto 0 ),
         Bus2IP_Clk => Bus2SMP_PCP_Clk,
         Bus2IP_Data => Bus2SMP_PCP_Data( C_S_AXI_SMP_PCP_DATA_WIDTH-1 downto 0 ),
         Bus2IP_RNW => Bus2SMP_PCP_RNW,
         Bus2IP_RdCE => open,
         Bus2IP_Resetn => Bus2SMP_PCP_Resetn,
         Bus2IP_WrCE => open,
         IP2Bus_Data => SMP_PCP2Bus_Data( C_S_AXI_SMP_PCP_DATA_WIDTH-1 downto 0 ),
         IP2Bus_Error => SMP_PCP2Bus_Error,
         IP2Bus_RdAck => SMP_PCP2Bus_RdAck,
         IP2Bus_WrAck => SMP_PCP2Bus_WrAck,
         S_AXI_ACLK => S_AXI_SMP_PCP_ACLK,
         S_AXI_ARADDR => S_AXI_SMP_PCP_ARADDR( C_S_AXI_SMP_PCP_ADDR_WIDTH-1 downto 0 ),
         S_AXI_ARESETN => S_AXI_SMP_PCP_ARESETN,
         S_AXI_ARREADY => S_AXI_SMP_PCP_ARREADY,
         S_AXI_ARVALID => S_AXI_SMP_PCP_ARVALID,
         S_AXI_AWADDR => S_AXI_SMP_PCP_AWADDR( C_S_AXI_SMP_PCP_ADDR_WIDTH-1 downto 0 ),
         S_AXI_AWREADY => S_AXI_SMP_PCP_AWREADY,
         S_AXI_AWVALID => S_AXI_SMP_PCP_AWVALID,
         S_AXI_BREADY => S_AXI_SMP_PCP_BREADY,
         S_AXI_BRESP => S_AXI_SMP_PCP_BRESP,
         S_AXI_BVALID => S_AXI_SMP_PCP_BVALID,
         S_AXI_RDATA => S_AXI_SMP_PCP_RDATA( C_S_AXI_SMP_PCP_DATA_WIDTH-1 downto 0 ),
         S_AXI_RREADY => S_AXI_SMP_PCP_RREADY,
         S_AXI_RRESP => S_AXI_SMP_PCP_RRESP,
         S_AXI_RVALID => S_AXI_SMP_PCP_RVALID,
         S_AXI_WDATA => S_AXI_SMP_PCP_WDATA( C_S_AXI_SMP_PCP_DATA_WIDTH-1 downto 0 ),
         S_AXI_WREADY => S_AXI_SMP_PCP_WREADY,
         S_AXI_WSTRB => S_AXI_SMP_PCP_WSTRB( (C_S_AXI_SMP_PCP_DATA_WIDTH/8)-1 downto 0 ),
         S_AXI_WVALID => S_AXI_SMP_PCP_WVALID
    );
end generate genSmpIo;

genSimpleIoSignals : if C_GEN_SIMPLE_IO generate
begin
  --SMP_PCP assignments
clkPcp <= Bus2SMP_PCP_Clk;
Bus2SMP_PCP_Reset <= not Bus2SMP_PCP_Resetn;
smp_writedata <= Bus2SMP_PCP_Data;
smp_read <= Bus2SMP_PCP_RNW and Bus2SMP_PCP_CS(0);
smp_write <= not Bus2SMP_PCP_RNW and Bus2SMP_PCP_CS(0);
smp_chipselect <= Bus2SMP_PCP_CS(0);
smp_byteenable <= Bus2SMP_PCP_BE;
smp_address <= Bus2SMP_PCP_Addr(2);

SMP_PCP2Bus_Data <= smp_readdata;
SMP_PCP2Bus_RdAck <= smp_chipselect and smp_read and not smp_waitrequest;
SMP_PCP2Bus_WrAck <= smp_chipselect and smp_write and not smp_waitrequest;
SMP_PCP2Bus_Error <= '0';
end generate genSimpleIoSignals;

oddr2_0 : if not C_INSTANCE_ODDR2 generate
begin
  phy0_clk <= clk50;

  phy1_clk <= clk50;
end generate oddr2_0;

oddr2_1 : if C_INSTANCE_ODDR2 generate
begin
  U10 : ODDR2
    port map(
         C0 => clk50,
         C1 => NET38418,
         CE => VCC,
         D0 => VCC,
         D1 => GND,
         Q => phy0_clk,
         R => GND,
         S => GND
    );

  U11 : ODDR2
    port map(
         C0 => clk50,
         C1 => NET38470,
         CE => VCC,
         D0 => VCC,
         D1 => GND,
         Q => phy1_clk,
         R => GND,
         S => GND
    );

  NET38470 <= not(clk50);

  NET38418 <= not(clk50);
end generate oddr2_1;

end struct;
