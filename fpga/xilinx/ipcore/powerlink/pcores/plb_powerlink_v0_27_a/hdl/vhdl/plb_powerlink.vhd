-------------------------------------------------------------------------------
-- Entity : plb_powerlink
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
-- This is the toplevel file for using the POWERLINK IP-Core
-- with Xilinx PLB V4.6.
--
-------------------------------------------------------------------------------
--
-- 2011-09-13   V0.01   zelenkaj    First version
-- 2011-11-24   V0.02   mairt       added slave interface for pdi pcp and pdi ap
-- 2011-11-26   V0.03   mairt       added slave interface for simpleIO
-- 2011-12-02   V0.04   zelenkaj    Exchanged IOs with _I, _O and _T
-- 2011-12-06   V0.05   zelenkaj    Changed instance names
-- 2011-12-07   V0.06   zelenkaj    Fixed address assignments for PDI PCP/AP
-- 2011-12-16   V0.07   mairt       added TX/RX burst size feature
-- 2012-01-19   V0.08   zelenkaj    Added bus to core clock ration feature
-- 2012-01-26   V0.09   zelenkaj    Added number of SMI generic feature
-- 2012-01-16   V0.10   zelenkaj    Replace plb_* with ipif_master_handler
-- 2012-01-27   V0.20   zelenkaj    Incremented PdiRev
-- 2012-02-01   V0.21   zelenkaj    Added attributes and RMII clk out
-- 2012-03-23   V0.22   zelenkaj    fixed to/downto issue
-- 2012-08-03   V0.23   zelenkaj    add pcp sys id
--
-------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;

use ieee.std_logic_unsigned.all;
use ieee.math_real.log2;
use ieee.math_real.ceil;

library proc_common_v3_00_a;
use proc_common_v3_00_a.proc_common_pkg.all;
use proc_common_v3_00_a.ipif_pkg.all;

library plbv46_slave_single_v1_01_a;
use plbv46_slave_single_v1_01_a.plbv46_slave_single;

-- standard libraries declarations
library UNISIM;
use UNISIM.vcomponents.all;
-- pragma synthesis_off
library IEEE;
use IEEE.vital_timing.all;
-- pragma synthesis_on

-- other libraries declarations
library PLBV46_MASTER_BURST_V1_01_A;
library PLBV46_SLAVE_SINGLE_V1_01_A;

entity plb_powerlink is
  generic(
       -- general
       C_GEN_PDI : boolean := false;
       C_GEN_PAR_IF : boolean := false;
       C_GEN_SPI_IF : boolean := false;
       C_GEN_PLB_BUS_IF : boolean := false;
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
       --pdi
       C_PDI_REV : integer := 0;
       C_PCP_SYS_ID : integer := 0;
       C_PDI_GEN_ASYNC_BUF_0 : boolean := true;
       C_PDI_ASYNC_BUF_0 : integer := 50;
       C_PDI_GEN_ASYNC_BUF_1 : boolean := true;
       C_PDI_ASYNC_BUF_1 : integer := 50;
       C_PDI_GEN_LED : boolean := false;
       C_PDI_GEN_TIME_SYNC : boolean := true;
       C_PDI_GEN_SECOND_TIMER : boolean := false;
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
       -- PDI AP PLB Slave
       C_PDI_AP_BASEADDR : std_logic_vector := X"00000000";
       C_PDI_AP_HIGHADDR : std_logic_vector := X"000FFFFF";
       C_PDI_AP_NUM_MASTERS : INTEGER := 1;
       C_PDI_AP_PLB_AWIDTH : INTEGER := 32;
       C_PDI_AP_PLB_DWIDTH : INTEGER := 32;
       C_PDI_AP_PLB_MID_WIDTH : INTEGER := 1;
       C_PDI_AP_PLB_P2P : INTEGER := 0;
       C_PDI_AP_PLB_NUM_MASTERS : INTEGER := 1;
       C_PDI_AP_PLB_NATIVE_DWIDTH : INTEGER := 32;
       C_PDI_AP_PLB_SUPPORT_BURSTS : INTEGER := 0;
       -- PDI AP PLB Slave
       C_SMP_PCP_BASEADDR : std_logic_vector := X"00000000";
       C_SMP_PCP_HIGHADDR : std_logic_vector := X"000FFFFF";
       C_SMP_PCP_NUM_MASTERS : INTEGER := 1;
       C_SMP_PCP_PLB_AWIDTH : INTEGER := 32;
       C_SMP_PCP_PLB_DWIDTH : INTEGER := 32;
       C_SMP_PCP_PLB_MID_WIDTH : INTEGER := 1;
       C_SMP_PCP_PLB_P2P : INTEGER := 0;
       C_SMP_PCP_PLB_NUM_MASTERS : INTEGER := 1;
       C_SMP_PCP_PLB_NATIVE_DWIDTH : INTEGER := 32;
       C_SMP_PCP_PLB_SUPPORT_BURSTS : INTEGER := 0;
       -- PDI PCP PLB Slave
       C_PDI_PCP_BASEADDR : std_logic_vector := X"00000000";
       C_PDI_PCP_HIGHADDR : std_logic_vector := X"000FFFFF";
       C_PDI_PCP_NUM_MASTERS : INTEGER := 1;
       C_PDI_PCP_PLB_AWIDTH : INTEGER := 32;
       C_PDI_PCP_PLB_DWIDTH : INTEGER := 32;
       C_PDI_PCP_PLB_MID_WIDTH : INTEGER := 1;
       C_PDI_PCP_PLB_P2P : INTEGER := 0;
       C_PDI_PCP_PLB_NUM_MASTERS : INTEGER := 1;
       C_PDI_PCP_PLB_NATIVE_DWIDTH : INTEGER := 32;
       C_PDI_PCP_PLB_SUPPORT_BURSTS : INTEGER := 0;
       -- openMAC CMP PLB Slave
       C_MAC_PKT_BASEADDR : std_logic_vector := X"00000000";
       C_MAC_PKT_HIGHADDR : std_logic_vector := X"000FFFFF";
       C_MAC_PKT_NUM_MASTERS : INTEGER := 1;
       C_MAC_PKT_PLB_AWIDTH : INTEGER := 32;
       C_MAC_PKT_PLB_DWIDTH : INTEGER := 32;
       C_MAC_PKT_PLB_MID_WIDTH : INTEGER := 1;
       C_MAC_PKT_PLB_P2P : INTEGER := 0;
       C_MAC_PKT_PLB_NUM_MASTERS : INTEGER := 1;
       C_MAC_PKT_PLB_NATIVE_DWIDTH : INTEGER := 32;
       C_MAC_PKT_PLB_SUPPORT_BURSTS : INTEGER := 0;
       -- openMAC DMA PLB Master
       C_MAC_DMA_PLB_AWIDTH : INTEGER := 32;
       C_MAC_DMA_PLB_DWIDTH : INTEGER := 32;
       C_MAC_DMA_PLB_NATIVE_DWIDTH : INTEGER := 32;
       C_MAC_DMA_BURST_SIZE_RX : INTEGER := 8; --in bytes
       C_MAC_DMA_BURST_SIZE_TX : INTEGER := 8; --in bytes
       C_MAC_DMA_FIFO_SIZE_RX : INTEGER := 32; --in bytes
       C_MAC_DMA_FIFO_SIZE_TX : INTEGER := 32; --in bytes
       -- openMAC REG PLB Slave
       C_MAC_REG_BASEADDR : std_logic_vector := X"00000000";
       C_MAC_REG_HIGHADDR : std_logic_vector := X"0000FFFF";
       C_MAC_CMP_BASEADDR : std_logic_vector := X"00000000";
       C_MAC_CMP_HIGHADDR : std_logic_vector := X"0000FFFF";
       C_MAC_REG_BUS2CORE_CLK_RATIO : integer := 2;
       C_MAC_REG_NUM_MASTERS : INTEGER := 1;
       C_MAC_REG_PLB_AWIDTH : INTEGER := 32;
       C_MAC_REG_PLB_DWIDTH : INTEGER := 32;
       C_MAC_REG_PLB_MID_WIDTH : INTEGER := 1;
       C_MAC_REG_PLB_P2P : INTEGER := 0;
       C_MAC_REG_PLB_NUM_MASTERS : INTEGER := 1;
       C_MAC_REG_PLB_NATIVE_DWIDTH : INTEGER := 32;
       C_MAC_REG_PLB_SUPPORT_BURSTS : INTEGER := 0
  );
  port(
       MAC_DMA_Clk : in std_logic;
       MAC_DMA_MAddrAck : in std_logic;
       MAC_DMA_MBusy : in std_logic;
       MAC_DMA_MIRQ : in std_logic;
       MAC_DMA_MRdBTerm : in std_logic;
       MAC_DMA_MRdDAck : in std_logic;
       MAC_DMA_MRdErr : in std_logic;
       MAC_DMA_MRearbitrate : in std_logic;
       MAC_DMA_MTimeout : in std_logic;
       MAC_DMA_MWrBTerm : in std_logic;
       MAC_DMA_MWrDAck : in std_logic;
       MAC_DMA_MWrErr : in std_logic;
       MAC_DMA_Rst : in std_logic;
       MAC_PKT_Clk : in std_logic;
       MAC_PKT_PAValid : in std_logic;
       MAC_PKT_RNW : in std_logic;
       MAC_PKT_Rst : in std_logic;
       MAC_PKT_SAValid : in std_logic;
       MAC_PKT_abort : in std_logic;
       MAC_PKT_busLock : in std_logic;
       MAC_PKT_lockErr : in std_logic;
       MAC_PKT_rdBurst : in std_logic;
       MAC_PKT_rdPendReq : in std_logic;
       MAC_PKT_rdPrim : in std_logic;
       MAC_PKT_wrBurst : in std_logic;
       MAC_PKT_wrPendReq : in std_logic;
       MAC_PKT_wrPrim : in std_logic;
       MAC_REG_Clk : in std_logic;
       MAC_REG_PAValid : in std_logic;
       MAC_REG_RNW : in std_logic;
       MAC_REG_Rst : in std_logic;
       MAC_REG_SAValid : in std_logic;
       MAC_REG_abort : in std_logic;
       MAC_REG_busLock : in std_logic;
       MAC_REG_lockErr : in std_logic;
       MAC_REG_rdBurst : in std_logic;
       MAC_REG_rdPendReq : in std_logic;
       MAC_REG_rdPrim : in std_logic;
       MAC_REG_wrBurst : in std_logic;
       MAC_REG_wrPendReq : in std_logic;
       MAC_REG_wrPrim : in std_logic;
       PDI_AP_Clk : in std_logic;
       PDI_AP_PAValid : in std_logic;
       PDI_AP_RNW : in std_logic;
       PDI_AP_Rst : in std_logic;
       PDI_AP_SAValid : in std_logic;
       PDI_AP_abort : in std_logic;
       PDI_AP_busLock : in std_logic;
       PDI_AP_lockErr : in std_logic;
       PDI_AP_rdBurst : in std_logic;
       PDI_AP_rdPendReq : in std_logic;
       PDI_AP_rdPrim : in std_logic;
       PDI_AP_wrBurst : in std_logic;
       PDI_AP_wrPendReq : in std_logic;
       PDI_AP_wrPrim : in std_logic;
       PDI_PCP_Clk : in std_logic;
       PDI_PCP_PAValid : in std_logic;
       PDI_PCP_RNW : in std_logic;
       PDI_PCP_Rst : in std_logic;
       PDI_PCP_SAValid : in std_logic;
       PDI_PCP_abort : in std_logic;
       PDI_PCP_busLock : in std_logic;
       PDI_PCP_lockErr : in std_logic;
       PDI_PCP_rdBurst : in std_logic;
       PDI_PCP_rdPendReq : in std_logic;
       PDI_PCP_rdPrim : in std_logic;
       PDI_PCP_wrBurst : in std_logic;
       PDI_PCP_wrPendReq : in std_logic;
       PDI_PCP_wrPrim : in std_logic;
       SMP_PCP_Clk : in std_logic;
       SMP_PCP_PAValid : in std_logic;
       SMP_PCP_RNW : in std_logic;
       SMP_PCP_Rst : in std_logic;
       SMP_PCP_SAValid : in std_logic;
       SMP_PCP_abort : in std_logic;
       SMP_PCP_busLock : in std_logic;
       SMP_PCP_lockErr : in std_logic;
       SMP_PCP_rdBurst : in std_logic;
       SMP_PCP_rdPendReq : in std_logic;
       SMP_PCP_rdPrim : in std_logic;
       SMP_PCP_wrBurst : in std_logic;
       SMP_PCP_wrPendReq : in std_logic;
       SMP_PCP_wrPrim : in std_logic;
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
       MAC_DMA_MRdDBus : in std_logic_vector(0 to C_MAC_DMA_PLB_DWIDTH-1);
       MAC_DMA_MRdWdAddr : in std_logic_vector(0 to 3);
       MAC_DMA_MSSize : in std_logic_vector(0 to 1);
       MAC_PKT_ABus : in std_logic_vector(0 to 31);
       MAC_PKT_BE : in std_logic_vector(0 to (C_MAC_PKT_PLB_DWIDTH/8)-1);
       MAC_PKT_MSize : in std_logic_vector(0 to 1);
       MAC_PKT_TAttribute : in std_logic_vector(0 to 15);
       MAC_PKT_UABus : in std_logic_vector(0 to 31);
       MAC_PKT_masterID : in std_logic_vector(0 to C_MAC_PKT_PLB_MID_WIDTH-1);
       MAC_PKT_rdPendPri : in std_logic_vector(0 to 1);
       MAC_PKT_reqPri : in std_logic_vector(0 to 1);
       MAC_PKT_size : in std_logic_vector(0 to 3);
       MAC_PKT_type : in std_logic_vector(0 to 2);
       MAC_PKT_wrDBus : in std_logic_vector(0 to C_MAC_PKT_PLB_DWIDTH-1);
       MAC_PKT_wrPendPri : in std_logic_vector(0 to 1);
       MAC_REG_ABus : in std_logic_vector(0 to 31);
       MAC_REG_BE : in std_logic_vector(0 to (C_MAC_REG_PLB_DWIDTH / 8) - 1);
       MAC_REG_MSize : in std_logic_vector(0 to 1);
       MAC_REG_TAttribute : in std_logic_vector(0 to 15);
       MAC_REG_UABus : in std_logic_vector(0 to 31);
       MAC_REG_masterID : in std_logic_vector(0 to C_MAC_REG_PLB_MID_WIDTH - 1);
       MAC_REG_rdPendPri : in std_logic_vector(0 to 1);
       MAC_REG_reqPri : in std_logic_vector(0 to 1);
       MAC_REG_size : in std_logic_vector(0 to 3);
       MAC_REG_type : in std_logic_vector(0 to 2);
       MAC_REG_wrDBus : in std_logic_vector(0 to C_MAC_REG_PLB_DWIDTH - 1);
       MAC_REG_wrPendPri : in std_logic_vector(0 to 1);
       PDI_AP_ABus : in std_logic_vector(0 to 31);
       PDI_AP_BE : in std_logic_vector(0 to (C_PDI_AP_PLB_DWIDTH/8)-1);
       PDI_AP_MSize : in std_logic_vector(0 to 1);
       PDI_AP_TAttribute : in std_logic_vector(0 to 15);
       PDI_AP_UABus : in std_logic_vector(0 to 31);
       PDI_AP_masterID : in std_logic_vector(0 to C_PDI_AP_PLB_MID_WIDTH-1);
       PDI_AP_rdPendPri : in std_logic_vector(0 to 1);
       PDI_AP_reqPri : in std_logic_vector(0 to 1);
       PDI_AP_size : in std_logic_vector(0 to 3);
       PDI_AP_type : in std_logic_vector(0 to 2);
       PDI_AP_wrDBus : in std_logic_vector(0 to C_PDI_AP_PLB_DWIDTH-1);
       PDI_AP_wrPendPri : in std_logic_vector(0 to 1);
       PDI_PCP_ABus : in std_logic_vector(0 to 31);
       PDI_PCP_BE : in std_logic_vector(0 to (C_PDI_PCP_PLB_DWIDTH/8)-1);
       PDI_PCP_MSize : in std_logic_vector(0 to 1);
       PDI_PCP_TAttribute : in std_logic_vector(0 to 15);
       PDI_PCP_UABus : in std_logic_vector(0 to 31);
       PDI_PCP_masterID : in std_logic_vector(0 to C_PDI_PCP_PLB_MID_WIDTH-1);
       PDI_PCP_rdPendPri : in std_logic_vector(0 to 1);
       PDI_PCP_reqPri : in std_logic_vector(0 to 1);
       PDI_PCP_size : in std_logic_vector(0 to 3);
       PDI_PCP_type : in std_logic_vector(0 to 2);
       PDI_PCP_wrDBus : in std_logic_vector(0 to C_PDI_PCP_PLB_DWIDTH-1);
       PDI_PCP_wrPendPri : in std_logic_vector(0 to 1);
       SMP_PCP_ABus : in std_logic_vector(0 to 31);
       SMP_PCP_BE : in std_logic_vector(0 to (C_SMP_PCP_PLB_DWIDTH/8)-1);
       SMP_PCP_MSize : in std_logic_vector(0 to 1);
       SMP_PCP_TAttribute : in std_logic_vector(0 to 15);
       SMP_PCP_UABus : in std_logic_vector(0 to 31);
       SMP_PCP_masterID : in std_logic_vector(0 to C_SMP_PCP_PLB_MID_WIDTH-1);
       SMP_PCP_rdPendPri : in std_logic_vector(0 to 1);
       SMP_PCP_reqPri : in std_logic_vector(0 to 1);
       SMP_PCP_size : in std_logic_vector(0 to 3);
       SMP_PCP_type : in std_logic_vector(0 to 2);
       SMP_PCP_wrDBus : in std_logic_vector(0 to C_SMP_PCP_PLB_DWIDTH-1);
       SMP_PCP_wrPendPri : in std_logic_vector(0 to 1);
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
       MAC_DMA_RNW : out std_logic;
       MAC_DMA_abort : out std_logic;
       MAC_DMA_busLock : out std_logic;
       MAC_DMA_error : out std_logic;
       MAC_DMA_lockErr : out std_logic;
       MAC_DMA_rdBurst : out std_logic;
       MAC_DMA_request : out std_logic;
       MAC_DMA_wrBurst : out std_logic;
       MAC_PKT_addrAck : out std_logic;
       MAC_PKT_rdBTerm : out std_logic;
       MAC_PKT_rdComp : out std_logic;
       MAC_PKT_rdDAck : out std_logic;
       MAC_PKT_rearbitrate : out std_logic;
       MAC_PKT_wait : out std_logic;
       MAC_PKT_wrBTerm : out std_logic;
       MAC_PKT_wrComp : out std_logic;
       MAC_PKT_wrDAck : out std_logic;
       MAC_REG_addrAck : out std_logic;
       MAC_REG_rdBTerm : out std_logic;
       MAC_REG_rdComp : out std_logic;
       MAC_REG_rdDAck : out std_logic;
       MAC_REG_rearbitrate : out std_logic;
       MAC_REG_wait : out std_logic;
       MAC_REG_wrBTerm : out std_logic;
       MAC_REG_wrComp : out std_logic;
       MAC_REG_wrDAck : out std_logic;
       PDI_AP_addrAck : out std_logic;
       PDI_AP_rdBTerm : out std_logic;
       PDI_AP_rdComp : out std_logic;
       PDI_AP_rdDAck : out std_logic;
       PDI_AP_rearbitrate : out std_logic;
       PDI_AP_wait : out std_logic;
       PDI_AP_wrBTerm : out std_logic;
       PDI_AP_wrComp : out std_logic;
       PDI_AP_wrDAck : out std_logic;
       PDI_PCP_addrAck : out std_logic;
       PDI_PCP_rdBTerm : out std_logic;
       PDI_PCP_rdComp : out std_logic;
       PDI_PCP_rdDAck : out std_logic;
       PDI_PCP_rearbitrate : out std_logic;
       PDI_PCP_wait : out std_logic;
       PDI_PCP_wrBTerm : out std_logic;
       PDI_PCP_wrComp : out std_logic;
       PDI_PCP_wrDAck : out std_logic;
       SMP_PCP_addrAck : out std_logic;
       SMP_PCP_rdBTerm : out std_logic;
       SMP_PCP_rdComp : out std_logic;
       SMP_PCP_rdDAck : out std_logic;
       SMP_PCP_rearbitrate : out std_logic;
       SMP_PCP_wait : out std_logic;
       SMP_PCP_wrBTerm : out std_logic;
       SMP_PCP_wrComp : out std_logic;
       SMP_PCP_wrDAck : out std_logic;
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
       MAC_DMA_ABus : out std_logic_vector(0 to 31);
       MAC_DMA_BE : out std_logic_vector(0 to (C_MAC_DMA_PLB_DWIDTH/8)-1);
       MAC_DMA_MSize : out std_logic_vector(0 to 1);
       MAC_DMA_TAttribute : out std_logic_vector(0 to 15);
       MAC_DMA_UABus : out std_logic_vector(0 to 31);
       MAC_DMA_priority : out std_logic_vector(0 to 1);
       MAC_DMA_size : out std_logic_vector(0 to 3);
       MAC_DMA_type : out std_logic_vector(0 to 2);
       MAC_DMA_wrDBus : out std_logic_vector(0 to C_MAC_DMA_PLB_DWIDTH-1);
       MAC_PKT_MBusy : out std_logic_vector(0 to C_MAC_PKT_NUM_MASTERS-1);
       MAC_PKT_MIRQ : out std_logic_vector(0 to C_MAC_PKT_NUM_MASTERS-1);
       MAC_PKT_MRdErr : out std_logic_vector(0 to C_MAC_PKT_NUM_MASTERS-1);
       MAC_PKT_MWrErr : out std_logic_vector(0 to C_MAC_PKT_NUM_MASTERS-1);
       MAC_PKT_SSize : out std_logic_vector(0 to 1);
       MAC_PKT_rdDBus : out std_logic_vector(0 to C_MAC_PKT_PLB_DWIDTH-1);
       MAC_PKT_rdWdAddr : out std_logic_vector(0 to 3);
       MAC_REG_MBusy : out std_logic_vector(0 to C_MAC_REG_NUM_MASTERS-1);
       MAC_REG_MIRQ : out std_logic_vector(0 to C_MAC_REG_NUM_MASTERS-1);
       MAC_REG_MRdErr : out std_logic_vector(0 to C_MAC_REG_NUM_MASTERS-1);
       MAC_REG_MWrErr : out std_logic_vector(0 to C_MAC_REG_NUM_MASTERS-1);
       MAC_REG_SSize : out std_logic_vector(0 to 1);
       MAC_REG_rdDBus : out std_logic_vector(0 to C_MAC_REG_PLB_DWIDTH-1);
       MAC_REG_rdWdAddr : out std_logic_vector(0 to 3);
       PDI_AP_MBusy : out std_logic_vector(0 to C_PDI_AP_PLB_NUM_MASTERS-1);
       PDI_AP_MIRQ : out std_logic_vector(0 to C_PDI_AP_PLB_NUM_MASTERS-1);
       PDI_AP_MRdErr : out std_logic_vector(0 to C_PDI_AP_PLB_NUM_MASTERS-1);
       PDI_AP_MWrErr : out std_logic_vector(0 to C_PDI_AP_PLB_NUM_MASTERS-1);
       PDI_AP_SSize : out std_logic_vector(0 to 1);
       PDI_AP_rdDBus : out std_logic_vector(0 to C_PDI_AP_PLB_DWIDTH-1);
       PDI_AP_rdWdAddr : out std_logic_vector(0 to 3);
       PDI_PCP_MBusy : out std_logic_vector(0 to C_PDI_PCP_NUM_MASTERS-1);
       PDI_PCP_MIRQ : out std_logic_vector(0 to C_PDI_PCP_NUM_MASTERS-1);
       PDI_PCP_MRdErr : out std_logic_vector(0 to C_PDI_PCP_NUM_MASTERS-1);
       PDI_PCP_MWrErr : out std_logic_vector(0 to C_PDI_PCP_NUM_MASTERS-1);
       PDI_PCP_SSize : out std_logic_vector(0 to 1);
       PDI_PCP_rdDBus : out std_logic_vector(0 to C_PDI_PCP_PLB_DWIDTH-1);
       PDI_PCP_rdWdAddr : out std_logic_vector(0 to 3);
       SMP_PCP_MBusy : out std_logic_vector(0 to C_SMP_PCP_PLB_NUM_MASTERS-1);
       SMP_PCP_MIRQ : out std_logic_vector(0 to C_SMP_PCP_PLB_NUM_MASTERS-1);
       SMP_PCP_MRdErr : out std_logic_vector(0 to C_SMP_PCP_PLB_NUM_MASTERS-1);
       SMP_PCP_MWrErr : out std_logic_vector(0 to C_SMP_PCP_PLB_NUM_MASTERS-1);
       SMP_PCP_SSize : out std_logic_vector(0 to 1);
       SMP_PCP_rdDBus : out std_logic_vector(0 to C_SMP_PCP_PLB_DWIDTH-1);
       SMP_PCP_rdWdAddr : out std_logic_vector(0 to 3);
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
attribute SIGIS of MAC_DMA_Clk : signal is "Clk";

attribute SIGIS of MAC_DMA_Rst : signal is "Rst";

attribute SIGIS of MAC_PKT_Clk : signal is "Clk";

attribute SIGIS of MAC_PKT_Rst : signal is "Rst";

attribute SIGIS of MAC_REG_Clk : signal is "Clk";

attribute SIGIS of MAC_REG_Rst : signal is "Rst";

attribute SIGIS of PDI_AP_Clk : signal is "Clk";

attribute SIGIS of PDI_AP_Rst : signal is "Rst";

attribute SIGIS of PDI_PCP_Clk : signal is "Clk";

attribute SIGIS of PDI_PCP_Rst : signal is "Rst";

attribute SIGIS of SMP_PCP_Clk : signal is "Clk";

attribute SIGIS of SMP_PCP_Rst : signal is "Rst";

attribute SIGIS of clk100 : signal is "Clk";

attribute SIGIS of clk50 : signal is "Clk";

attribute SIGIS of phy0_clk : signal is "Clk";

attribute SIGIS of phy1_clk : signal is "Clk";

end plb_powerlink;

architecture struct of plb_powerlink is

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
       Simulate : boolean := false;
       endian_g : string := "little";
       gNumSmi : integer range 1 to 2 := 2;
       genABuf1_g : boolean := true;
       genABuf2_g : boolean := true;
       genEvent_g : boolean := false;
       genInternalAp_g : boolean := true;
       genIoBuf_g : boolean := true;
       genLedGadget_g : boolean := false;
       genOnePdiClkDomain_g : boolean := false;
       genPdi_g : boolean := true;
       genSimpleIO_g : boolean := false;
       genSmiIO : boolean := true;
       genSpiAp_g : boolean := false;
       genTimeSync_g : boolean := false;
       gen_dma_observer_g : boolean := true;
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
       m_burstcount_const_g : boolean := true;
       m_burstcount_width_g : integer := 4;
       m_data_width_g : integer := 16;
       m_rx_burst_size_g : integer := 16;
       m_rx_fifo_size_g : integer := 16;
       m_tx_burst_size_g : integer := 16;
       m_tx_fifo_size_g : integer := 16;
       papBigEnd_g : boolean := false;
       papDataWidth_g : integer := 8;
       papLowAct_g : boolean := false;
       pcpSysId : integer := 1;
       pioValLen_g : integer := 50;
       spiBigEnd_g : boolean := false;
       spiCPHA_g : boolean := false;
       spiCPOL_g : boolean := false;
       use2ndCmpTimer_g : boolean := true;
       use2ndPhy_g : boolean := true;
       useIntPacketBuf_g : boolean := true;
       useRmii_g : boolean := true;
       useRxIntPacketBuf_g : boolean := true
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
component plbv46_master_burst
  generic(
       C_FAMILY : string := "virtex5";
       C_INHIBIT_CC_BLE_INCLUSION : integer range 0 to 1 := 0;
       C_MPLB_AWIDTH : integer range 32 to 36 := 32;
       C_MPLB_DWIDTH : integer range 32 to 128 := 32;
       C_MPLB_NATIVE_DWIDTH : integer range 32 to 128 := 32;
       C_MPLB_SMALLEST_SLAVE : integer range 32 to 128 := 32
  );
  port (
       IP2Bus_MstRd_Req : in std_logic;
       IP2Bus_MstRd_dst_dsc_n : in std_logic;
       IP2Bus_MstRd_dst_rdy_n : in std_logic;
       IP2Bus_MstWr_Req : in std_logic;
       IP2Bus_MstWr_d : in std_logic_vector(0 to C_MPLB_NATIVE_DWIDTH-1);
       IP2Bus_MstWr_eof_n : in std_logic;
       IP2Bus_MstWr_rem : in std_logic_vector(0 to (C_MPLB_NATIVE_DWIDTH/8)-1);
       IP2Bus_MstWr_sof_n : in std_logic;
       IP2Bus_MstWr_src_dsc_n : in std_logic;
       IP2Bus_MstWr_src_rdy_n : in std_logic;
       IP2Bus_Mst_Addr : in std_logic_vector(0 to C_MPLB_AWIDTH-1);
       IP2Bus_Mst_BE : in std_logic_vector(0 to (C_MPLB_NATIVE_DWIDTH/8)-1);
       IP2Bus_Mst_Length : in std_logic_vector(0 to 11);
       IP2Bus_Mst_Lock : in std_logic;
       IP2Bus_Mst_Reset : in std_logic;
       IP2Bus_Mst_Type : in std_logic;
       MPLB_Clk : in std_logic;
       MPLB_Rst : in std_logic;
       PLB_MAddrAck : in std_logic;
       PLB_MBusy : in std_logic;
       PLB_MIRQ : in std_logic;
       PLB_MRdBTerm : in std_logic;
       PLB_MRdDAck : in std_logic;
       PLB_MRdDBus : in std_logic_vector(0 to C_MPLB_DWIDTH-1);
       PLB_MRdErr : in std_logic;
       PLB_MRdWdAddr : in std_logic_vector(0 to 3);
       PLB_MRearbitrate : in std_logic;
       PLB_MSSize : in std_logic_vector(0 to 1);
       PLB_MTimeout : in std_logic;
       PLB_MWrBTerm : in std_logic;
       PLB_MWrDAck : in std_logic;
       PLB_MWrErr : in std_logic;
       Bus2IP_MstRd_d : out std_logic_vector(0 to C_MPLB_NATIVE_DWIDTH-1);
       Bus2IP_MstRd_eof_n : out std_logic;
       Bus2IP_MstRd_rem : out std_logic_vector(0 to (C_MPLB_NATIVE_DWIDTH/8)-1);
       Bus2IP_MstRd_sof_n : out std_logic;
       Bus2IP_MstRd_src_dsc_n : out std_logic;
       Bus2IP_MstRd_src_rdy_n : out std_logic;
       Bus2IP_MstWr_dst_dsc_n : out std_logic;
       Bus2IP_MstWr_dst_rdy_n : out std_logic;
       Bus2IP_Mst_CmdAck : out std_logic;
       Bus2IP_Mst_Cmd_Timeout : out std_logic;
       Bus2IP_Mst_Cmplt : out std_logic;
       Bus2IP_Mst_Error : out std_logic;
       Bus2IP_Mst_Rearbitrate : out std_logic;
       MD_Error : out std_logic;
       M_ABus : out std_logic_vector(0 to 31);
       M_BE : out std_logic_vector(0 to (C_MPLB_DWIDTH/8)-1);
       M_MSize : out std_logic_vector(0 to 1);
       M_RNW : out std_logic;
       M_TAttribute : out std_logic_vector(0 to 15);
       M_UABus : out std_logic_vector(0 to 31);
       M_abort : out std_logic;
       M_busLock : out std_logic;
       M_lockErr : out std_logic;
       M_priority : out std_logic_vector(0 to 1);
       M_rdBurst : out std_logic;
       M_request : out std_logic;
       M_size : out std_logic_vector(0 to 3);
       M_type : out std_logic_vector(0 to 2);
       M_wrBurst : out std_logic;
       M_wrDBus : out std_logic_vector(0 to C_MPLB_DWIDTH-1)
  );
end component;
component plbv46_slave_single
  generic(
       C_ARD_ADDR_RANGE_ARRAY : slv64_array_type := (X"0000_0000_7000_0000",X"0000_0000_7000_00FF",X"0000_0000_7000_0100",X"0000_0000_7000_01FF");
       C_ARD_NUM_CE_ARRAY : integer_array_type := (1,8);
       C_BUS2CORE_CLK_RATIO : integer range 1 to 2 := 1;
       C_FAMILY : string := "virtex4";
       C_INCLUDE_DPHASE_TIMER : integer range 0 to 1 := 1;
       C_SIPIF_DWIDTH : integer range 32 to 32 := 32;
       C_SPLB_AWIDTH : integer range 32 to 32 := 32;
       C_SPLB_DWIDTH : integer range 32 to 128 := 32;
       C_SPLB_MID_WIDTH : integer range 1 to 4 := 2;
       C_SPLB_NUM_MASTERS : integer range 1 to 16 := 8;
       C_SPLB_P2P : integer range 0 to 1 := 0
  );
  port (
       IP2Bus_Data : in std_logic_vector(0 to C_SIPIF_DWIDTH-1);
       IP2Bus_Error : in std_logic;
       IP2Bus_RdAck : in std_logic;
       IP2Bus_WrAck : in std_logic;
       PLB_ABus : in std_logic_vector(0 to 31);
       PLB_BE : in std_logic_vector(0 to (C_SPLB_DWIDTH/8)-1);
       PLB_MSize : in std_logic_vector(0 to 1);
       PLB_PAValid : in std_logic;
       PLB_RNW : in std_logic;
       PLB_SAValid : in std_logic;
       PLB_TAttribute : in std_logic_vector(0 to 15);
       PLB_UABus : in std_logic_vector(0 to 31);
       PLB_abort : in std_logic;
       PLB_busLock : in std_logic;
       PLB_lockErr : in std_logic;
       PLB_masterID : in std_logic_vector(0 to C_SPLB_MID_WIDTH-1);
       PLB_rdBurst : in std_logic;
       PLB_rdPendPri : in std_logic_vector(0 to 1);
       PLB_rdPendReq : in std_logic;
       PLB_rdPrim : in std_logic;
       PLB_reqPri : in std_logic_vector(0 to 1);
       PLB_size : in std_logic_vector(0 to 3);
       PLB_type : in std_logic_vector(0 to 2);
       PLB_wrBurst : in std_logic;
       PLB_wrDBus : in std_logic_vector(0 to C_SPLB_DWIDTH-1);
       PLB_wrPendPri : in std_logic_vector(0 to 1);
       PLB_wrPendReq : in std_logic;
       PLB_wrPrim : in std_logic;
       SPLB_Clk : in std_logic;
       SPLB_Rst : in std_logic;
       Bus2IP_Addr : out std_logic_vector(0 to C_SPLB_AWIDTH-1);
       Bus2IP_BE : out std_logic_vector(0 to (C_SIPIF_DWIDTH/8)-1);
       Bus2IP_CS : out std_logic_vector(0 to ((C_ARD_ADDR_RANGE_ARRAY'LENGTH)/2)-1);
       Bus2IP_Clk : out std_logic;
       Bus2IP_Data : out std_logic_vector(0 to C_SIPIF_DWIDTH-1);
       Bus2IP_RNW : out std_logic;
       Bus2IP_RdCE : out std_logic_vector(0 to calc_num_ce(C_ARD_NUM_CE_ARRAY)-1);
       Bus2IP_Reset : out std_logic;
       Bus2IP_WrCE : out std_logic_vector(0 to calc_num_ce(C_ARD_NUM_CE_ARRAY)-1);
       Sl_MBusy : out std_logic_vector(0 to C_SPLB_NUM_MASTERS-1);
       Sl_MIRQ : out std_logic_vector(0 to C_SPLB_NUM_MASTERS-1);
       Sl_MRdErr : out std_logic_vector(0 to C_SPLB_NUM_MASTERS-1);
       Sl_MWrErr : out std_logic_vector(0 to C_SPLB_NUM_MASTERS-1);
       Sl_SSize : out std_logic_vector(0 to 1);
       Sl_addrAck : out std_logic;
       Sl_rdBTerm : out std_logic;
       Sl_rdComp : out std_logic;
       Sl_rdDAck : out std_logic;
       Sl_rdDBus : out std_logic_vector(0 to C_SPLB_DWIDTH-1);
       Sl_rdWdAddr : out std_logic_vector(0 to 3);
       Sl_rearbitrate : out std_logic;
       Sl_wait : out std_logic;
       Sl_wrBTerm : out std_logic;
       Sl_wrComp : out std_logic;
       Sl_wrDAck : out std_logic
  );
end component;

---- Architecture declarations -----
constant C_FAMILY : string := "spartan6";
constant C_ADDR_PAD_ZERO : std_logic_vector(31 downto 0) := (others => '0');
-- openMAC REG PLB Slave
constant C_MAC_REG_BASE : std_logic_vector(63 downto 0) := C_ADDR_PAD_ZERO & C_MAC_REG_BASEADDR;
constant C_MAC_REG_HIGH : std_logic_vector(63 downto 0) := C_ADDR_PAD_ZERO & C_MAC_REG_HIGHADDR;
-- openMAC CMP PLB Slave
constant C_MAC_CMP_BASE : std_logic_vector(63 downto 0) := C_ADDR_PAD_ZERO & C_MAC_CMP_BASEADDR;
constant C_MAC_CMP_HIGH : std_logic_vector(63 downto 0) := C_ADDR_PAD_ZERO & C_MAC_CMP_HIGHADDR;
-- openMAC PKT PLB Slave
constant C_MAC_PKT_BASE : std_logic_vector(63 downto 0) := C_ADDR_PAD_ZERO & C_MAC_PKT_BASEADDR;
constant C_MAC_PKT_HIGH : std_logic_vector(63 downto 0) := C_ADDR_PAD_ZERO & C_MAC_PKT_HIGHADDR;
-- SimpleIO Slave
constant C_SMP_PCP_BASE : std_logic_vector(63 downto 0) := C_ADDR_PAD_ZERO & C_SMP_PCP_BASEADDR;
constant C_SMP_PCP_HIGH : std_logic_vector(63 downto 0) := C_ADDR_PAD_ZERO & C_SMP_PCP_HIGHADDR;
-- PDI PCP Slave
constant C_PDI_PCP_BASE : std_logic_vector(63 downto 0) := C_ADDR_PAD_ZERO & C_PDI_PCP_BASEADDR;
constant C_PDI_PCP_HIGH : std_logic_vector(63 downto 0) := C_ADDR_PAD_ZERO & C_PDI_PCP_HIGHADDR;
-- AP PCP Slave
constant C_PDI_AP_BASE : std_logic_vector(63 downto 0) := C_ADDR_PAD_ZERO & C_PDI_AP_BASEADDR;
constant C_PDI_AP_HIGH : std_logic_vector(63 downto 0) := C_ADDR_PAD_ZERO & C_PDI_AP_HIGHADDR;
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
signal Bus2MAC_CMP_Reset : std_logic;
signal Bus2MAC_DMA_MstRd_eof_n : std_logic;
signal Bus2MAC_DMA_MstRd_sof_n : std_logic;
signal Bus2MAC_DMA_MstRd_src_dsc_n : std_logic;
signal Bus2MAC_DMA_MstRd_src_rdy_n : std_logic;
signal Bus2MAC_DMA_MstWr_dst_dsc_n : std_logic;
signal Bus2MAC_DMA_MstWr_dst_rdy_n : std_logic;
signal Bus2MAC_DMA_Mst_CmdAck : std_logic;
signal Bus2MAC_DMA_Mst_Cmd_Timeout : std_logic;
signal Bus2MAC_DMA_Mst_Cmplt : std_logic;
signal Bus2MAC_DMA_Mst_Error : std_logic;
signal Bus2MAC_DMA_Mst_Rearbitrate : std_logic;
signal Bus2MAC_PKT_Clk : std_logic;
signal Bus2MAC_PKT_Reset : std_logic;
signal Bus2MAC_PKT_RNW : std_logic;
signal Bus2MAC_REG_Clk : std_logic;
signal Bus2MAC_REG_Reset : std_logic;
signal Bus2MAC_REG_RNW : std_logic;
signal Bus2MAC_REG_RNW_n : std_logic;
signal Bus2PDI_AP_Clk : std_logic;
signal Bus2PDI_AP_Reset : std_logic;
signal Bus2PDI_AP_RNW : std_logic;
signal Bus2PDI_PCP_Clk : std_logic;
signal Bus2PDI_PCP_Reset : std_logic;
signal Bus2PDI_PCP_RNW : std_logic;
signal Bus2SMP_PCP_Clk : std_logic;
signal Bus2SMP_PCP_Reset : std_logic;
signal Bus2SMP_PCP_RNW : std_logic;
signal clkAp : std_logic;
signal clkPcp : std_logic;
signal GND : std_logic;
signal IP2Bus_Error_s : std_logic;
signal IP2Bus_RdAck_s : std_logic;
signal IP2Bus_WrAck_s : std_logic;
signal mac_chipselect : std_logic;
signal MAC_CMP2Bus_Error : std_logic;
signal MAC_CMP2Bus_RdAck : std_logic;
signal MAC_CMP2Bus_WrAck : std_logic;
signal MAC_DMA2Bus_MstRd_dst_dsc_n : std_logic;
signal MAC_DMA2Bus_MstRd_dst_rdy_n : std_logic;
signal MAC_DMA2Bus_MstRd_Req : std_logic;
signal MAC_DMA2Bus_MstWr_eof_n : std_logic;
signal MAC_DMA2Bus_MstWr_Req : std_logic;
signal MAC_DMA2Bus_MstWr_sof_n : std_logic;
signal MAC_DMA2Bus_MstWr_src_dsc_n : std_logic;
signal MAC_DMA2Bus_MstWr_src_rdy_n : std_logic;
signal MAC_DMA2Bus_Mst_Lock : std_logic;
signal MAC_DMA2Bus_Mst_Reset : std_logic;
signal MAC_DMA2Bus_Mst_Type : std_logic;
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
signal NET118078 : std_ulogic;
signal NET118214 : std_ulogic;
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
signal rst : std_logic;
signal rstAp : std_logic;
signal rstPcp : std_logic;
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
signal Bus2MAC_DMA_MstRd_d : std_logic_vector (C_MAC_DMA_PLB_NATIVE_DWIDTH-1 downto 0);
signal Bus2MAC_DMA_MstRd_d_s : std_logic_vector (C_MAC_DMA_PLB_NATIVE_DWIDTH-1 downto 0);
signal Bus2MAC_DMA_MstRd_rem : std_logic_vector (0 to (C_MAC_DMA_PLB_NATIVE_DWIDTH/8)-1);
signal Bus2MAC_PKT_Addr : std_logic_vector (C_MAC_PKT_PLB_AWIDTH-1 downto 0);
signal Bus2MAC_PKT_BE : std_logic_vector ((C_MAC_PKT_PLB_DWIDTH/8)-1 downto 0);
signal Bus2MAC_PKT_CS : std_logic_vector (0 downto 0);
signal Bus2MAC_PKT_Data : std_logic_vector (C_MAC_PKT_PLB_DWIDTH-1 downto 0);
signal Bus2MAC_REG_Addr : std_logic_vector (C_MAC_REG_PLB_AWIDTH-1 downto 0);
signal Bus2MAC_REG_BE : std_logic_vector ((C_MAC_REG_PLB_DWIDTH/8)-1 downto 0);
signal Bus2MAC_REG_BE_s : std_logic_vector ((C_MAC_REG_PLB_DWIDTH/8)-1 downto 0);
signal Bus2MAC_REG_CS : std_logic_vector (1 downto 0);
signal Bus2MAC_REG_Data : std_logic_vector (C_MAC_REG_PLB_DWIDTH-1 downto 0);
signal Bus2MAC_REG_Data_s : std_logic_vector (C_MAC_REG_PLB_DWIDTH-1 downto 0);
signal Bus2PDI_AP_Addr : std_logic_vector (C_PDI_AP_PLB_AWIDTH-1 downto 0);
signal Bus2PDI_AP_BE : std_logic_vector ((C_PDI_AP_PLB_DWIDTH/8)-1 downto 0);
signal Bus2PDI_AP_CS : std_logic_vector (0 downto 0);
signal Bus2PDI_AP_Data : std_logic_vector (C_PDI_AP_PLB_DWIDTH-1 downto 0);
signal Bus2PDI_PCP_Addr : std_logic_vector (C_PDI_PCP_PLB_AWIDTH-1 downto 0);
signal Bus2PDI_PCP_BE : std_logic_vector ((C_PDI_PCP_PLB_DWIDTH/8)-1 downto 0);
signal Bus2PDI_PCP_CS : std_logic_vector (0 downto 0);
signal Bus2PDI_PCP_Data : std_logic_vector (C_PDI_PCP_PLB_DWIDTH-1 downto 0);
signal Bus2SMP_PCP_Addr : std_logic_vector (C_SMP_PCP_PLB_AWIDTH-1 downto 0);
signal Bus2SMP_PCP_BE : std_logic_vector ((C_SMP_PCP_PLB_DWIDTH/8)-1 downto 0);
signal Bus2SMP_PCP_CS : std_logic_vector (0 downto 0);
signal Bus2SMP_PCP_Data : std_logic_vector (C_SMP_PCP_PLB_DWIDTH-1 downto 0);
signal IP2Bus_Data_s : std_logic_vector (C_MAC_REG_PLB_DWIDTH-1 downto 0);
signal mac_address : std_logic_vector (C_MAC_REG_PLB_AWIDTH-1 downto 0);
signal mac_byteenable : std_logic_vector (1 downto 0);
signal MAC_CMP2Bus_Data : std_logic_vector (C_MAC_REG_PLB_DWIDTH-1 downto 0);
signal MAC_DMA2Bus_MstWr_d : std_logic_vector (C_MAC_DMA_PLB_NATIVE_DWIDTH-1 downto 0);
signal MAC_DMA2Bus_MstWr_d_s : std_logic_vector (C_MAC_DMA_PLB_NATIVE_DWIDTH-1 downto 0);
signal MAC_DMA2Bus_MstWr_rem : std_logic_vector (0 to (C_MAC_DMA_PLB_NATIVE_DWIDTH/8)-1);
signal MAC_DMA2Bus_Mst_Addr : std_logic_vector (0 to C_MAC_DMA_PLB_AWIDTH-1);
signal MAC_DMA2Bus_Mst_BE : std_logic_vector (0 to (C_MAC_DMA_PLB_NATIVE_DWIDTH/8)-1);
signal MAC_DMA2Bus_Mst_Length : std_logic_vector (0 to 11);
signal MAC_PKT2Bus_Data : std_logic_vector (C_MAC_PKT_PLB_DWIDTH-1 downto 0);
signal mac_readdata : std_logic_vector (15 downto 0);
signal MAC_REG2Bus_Data : std_logic_vector (C_MAC_REG_PLB_DWIDTH-1 downto 0);
signal MAC_REG2Bus_Data_s : std_logic_vector (C_MAC_REG_PLB_DWIDTH-1 downto 0);
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
signal PDI_AP2Bus_Data : std_logic_vector (C_PDI_AP_PLB_DWIDTH-1 downto 0);
signal PDI_PCP2Bus_Data : std_logic_vector (C_PDI_PCP_PLB_DWIDTH-1 downto 0);
signal smp_byteenable : std_logic_vector (3 downto 0);
signal SMP_PCP2Bus_Data : std_logic_vector (C_SMP_PCP_PLB_DWIDTH-1 downto 0);
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
	IP2Bus_Data_s(C_MAC_REG_PLB_DWIDTH-1 downto 0) <= MAC_REG2Bus_Data(C_MAC_REG_PLB_DWIDTH-1 downto 0) when "10",
		MAC_CMP2Bus_Data(C_MAC_REG_PLB_DWIDTH-1 downto 0) 												when "01",
		(others => '0') 																				when others;
		
with Bus2MAC_REG_CS select 
	IP2Bus_WrAck_s <= MAC_REG2Bus_WrAck 				when "10",
						MAC_CMP2Bus_WrAck 					when "01",
						'0'										when others;	

with Bus2MAC_REG_CS select 
	IP2Bus_RdAck_s <= MAC_REG2Bus_RdAck 				when "10",
						MAC_CMP2Bus_RdAck 					when "01",
						'0'										when others;	

with Bus2MAC_REG_CS select 
	IP2Bus_Error_s <= MAC_REG2Bus_Error 				when "10",
						MAC_CMP2Bus_Error 					when "01",
						'0'										when others;
Bus2MAC_REG_BE_s <=
    Bus2MAC_REG_BE(0) & Bus2MAC_REG_BE(1) & 
    Bus2MAC_REG_BE(2) & Bus2MAC_REG_BE(3);

Bus2MAC_REG_Data_s <=
    Bus2MAC_REG_Data(7 downto 0) & Bus2MAC_REG_Data(15 downto 8) & 
    Bus2MAC_REG_Data(23 downto 16) & Bus2MAC_REG_Data(31 downto 24);

MAC_REG2Bus_Data <=
    MAC_REG2Bus_Data_s(7 downto 0) & MAC_REG2Bus_Data_s(15 downto 8) & 
    MAC_REG2Bus_Data_s(23 downto 16) & MAC_REG2Bus_Data_s(31 downto 24);
--test_port

test_port(181 downto 179) <= mac_chipselect & mac_write & mac_read;
test_port(178) <= mac_waitrequest;
test_port(177 downto 176) <= mac_byteenable;

test_port(171 downto 160) <= mac_address(11 downto 0);
test_port(159 downto 144) <= mac_writedata;
test_port(143 downto 128) <= mac_readdata;

test_port(104 downto 102) <= Bus2MAC_REG_CS & Bus2MAC_REG_RNW;
test_port(101 downto 100) <= IP2Bus_WrAck_s & IP2Bus_RdAck_s;
test_port(99 downto 96) <= Bus2MAC_REG_BE;

test_port(95 downto 64) <= Bus2MAC_REG_Addr;
test_port(63 downto 32) <= Bus2MAC_REG_Data;
test_port(31 downto 0) <= IP2Bus_Data_s;

--test_port(255 downto 251) <= m_read & m_write & m_waitrequest & m_readdatavalid & MAC_DMA2Bus_Mst_Type;

--test_port(244 downto 240) <= MAC_DMA2Bus_MstWr_Req & MAC_DMA2Bus_MstWr_sof_n & MAC_DMA2Bus_MstWr_eof_n & MAC_DMA2Bus_MstWr_src_rdy_n & Bus2MAC_DMA_MstWr_dst_rdy_n;
--test_port(234 downto 230) <= MAC_DMA2Bus_MstRd_Req & Bus2MAC_DMA_MstRd_sof_n & Bus2MAC_DMA_MstRd_eof_n & Bus2MAC_DMA_MstRd_src_rdy_n & MAC_DMA2Bus_MstRd_dst_rdy_n;

--test_port(142 downto 140) <= Bus2MAC_DMA_Mst_Cmplt & Bus2MAC_DMA_Mst_Error & Bus2MAC_DMA_Mst_Cmd_Timeout;

--test_port(MAC_DMA2Bus_Mst_Length'length+120-1 downto 120) <= MAC_DMA2Bus_Mst_Length;

--test_port(m_burstcount'length+110-1 downto 110) <= m_burstcount;
--test_port(m_burstcounter'length+96-1 downto 96) <= m_burstcounter;
--test_port(95 downto 64) <= m_address;
--test_port(63 downto 32) <= m_writedata;
--test_port(31 downto 0) <= m_readdata;
--mac_cmp assignments
---cmp_clk <= Bus2MAC_CMP_Clk;
tcp_writedata <=
    Bus2MAC_REG_Data(7 downto 0) & Bus2MAC_REG_Data(15 downto 8) & 
    Bus2MAC_REG_Data(23 downto 16) & Bus2MAC_REG_Data(31 downto 24);
tcp_read <= Bus2MAC_REG_RNW;
tcp_write <= not Bus2MAC_REG_RNW;
tcp_chipselect <= Bus2MAC_REG_CS(0);
tcp_byteenable <=
    Bus2MAC_REG_BE(0) & Bus2MAC_REG_BE(1) & 
    Bus2MAC_REG_BE(2) & Bus2MAC_REG_BE(3);
tcp_address <= Bus2MAC_REG_Addr(3 downto 2);

MAC_CMP2Bus_Data <= 
    tcp_readdata(7 downto 0) & tcp_readdata(15 downto 8) & 
    tcp_readdata(23 downto 16) & tcp_readdata(31 downto 24);
MAC_CMP2Bus_RdAck <= tcp_chipselect and tcp_read and not tcp_waitrequest;
MAC_CMP2Bus_WrAck <= tcp_chipselect and tcp_write and not tcp_waitrequest;
MAC_CMP2Bus_Error <= '0';
--mac_pkt assignments
pkt_clk <= Bus2MAC_PKT_Clk;
mbf_writedata <=
    Bus2MAC_PKT_Data(7 downto 0) & Bus2MAC_PKT_Data(15 downto 8) &
    Bus2MAC_PKT_Data(23 downto 16) & Bus2MAC_PKT_Data(31 downto 24);
mbf_read <= Bus2MAC_PKT_RNW;
mbf_write <= not Bus2MAC_PKT_RNW;
mbf_chipselect <= Bus2MAC_PKT_CS(0);
mbf_byteenable <=
    Bus2MAC_PKT_BE(0) & Bus2MAC_PKT_BE(1) &
    Bus2MAC_PKT_BE(2) & Bus2MAC_PKT_BE(3);
mbf_address <= Bus2MAC_PKT_Addr(C_MAC_PKT_SIZE_LOG2-1 downto 2);

MAC_PKT2Bus_Data <=
    mbf_readdata(7 downto 0) & mbf_readdata(15 downto 8) &
    mbf_readdata(23 downto 16) & mbf_readdata(31 downto 24);
MAC_PKT2Bus_RdAck <= mbf_chipselect and mbf_read and not mbf_waitrequest;
MAC_PKT2Bus_WrAck <= mbf_chipselect and mbf_write and not mbf_waitrequest;
MAC_PKT2Bus_Error <= '0';

----  Component instantiations  ----

MAC_REG_16to32 : openMAC_16to32conv
  generic map (
       bus_address_width => C_MAC_REG_PLB_AWIDTH,
       gEndian => "big"
  )
  port map(
       bus_ack_rd => MAC_REG2Bus_RdAck,
       bus_ack_wr => MAC_REG2Bus_WrAck,
       bus_address => Bus2MAC_REG_Addr( C_MAC_REG_PLB_AWIDTH-1 downto 0 ),
       bus_byteenable => Bus2MAC_REG_BE_s( (C_MAC_REG_PLB_DWIDTH/8)-1 downto 0 ),
       bus_read => Bus2MAC_REG_RNW,
       bus_readdata => MAC_REG2Bus_Data_s( C_MAC_REG_PLB_DWIDTH-1 downto 0 ),
       bus_select => Bus2MAC_REG_CS(1),
       bus_write => Bus2MAC_REG_RNW_n,
       bus_writedata => Bus2MAC_REG_Data_s( C_MAC_REG_PLB_DWIDTH-1 downto 0 ),
       clk => clk50,
       rst => rst,
       s_address => mac_address( C_MAC_REG_PLB_AWIDTH-1 downto 0 ),
       s_byteenable => mac_byteenable,
       s_chipselect => mac_chipselect,
       s_read => mac_read,
       s_readdata => mac_readdata,
       s_waitrequest => mac_waitrequest,
       s_write => mac_write,
       s_writedata => mac_writedata
  );

MAC_REG_PLB_SINGLE_SLAVE : plbv46_slave_single
  generic map (
       C_ARD_ADDR_RANGE_ARRAY => (C_MAC_REG_BASE,C_MAC_REG_HIGH,C_MAC_CMP_BASE,C_MAC_CMP_HIGH),
       C_ARD_NUM_CE_ARRAY => (1, 1),
       C_BUS2CORE_CLK_RATIO => C_MAC_REG_BUS2CORE_CLK_RATIO,
       C_FAMILY => C_FAMILY,
       C_INCLUDE_DPHASE_TIMER => 0,
       C_SIPIF_DWIDTH => C_MAC_REG_PLB_DWIDTH,
       C_SPLB_AWIDTH => C_MAC_REG_PLB_AWIDTH,
       C_SPLB_DWIDTH => C_MAC_REG_PLB_DWIDTH,
       C_SPLB_MID_WIDTH => C_MAC_REG_PLB_MID_WIDTH,
       C_SPLB_NUM_MASTERS => C_MAC_REG_PLB_NUM_MASTERS,
       C_SPLB_P2P => C_MAC_REG_PLB_P2P
  )
  port map(
       Bus2IP_Addr => Bus2MAC_REG_Addr( C_MAC_REG_PLB_AWIDTH-1 downto 0 ),
       Bus2IP_BE => Bus2MAC_REG_BE( (C_MAC_REG_PLB_DWIDTH/8)-1 downto 0 ),
       Bus2IP_CS => Bus2MAC_REG_CS( 1 downto 0 ),
       Bus2IP_Clk => Bus2MAC_REG_Clk,
       Bus2IP_Data => Bus2MAC_REG_Data( C_MAC_REG_PLB_DWIDTH-1 downto 0 ),
       Bus2IP_RNW => Bus2MAC_REG_RNW,
       Bus2IP_Reset => Bus2MAC_REG_Reset,
       IP2Bus_Data => IP2Bus_Data_s( C_MAC_REG_PLB_DWIDTH-1 downto 0 ),
       IP2Bus_Error => IP2Bus_Error_s,
       IP2Bus_RdAck => IP2Bus_RdAck_s,
       IP2Bus_WrAck => IP2Bus_WrAck_s,
       PLB_ABus => MAC_REG_ABus,
       PLB_BE => MAC_REG_BE( 0 to (C_MAC_REG_PLB_DWIDTH / 8) - 1 ),
       PLB_MSize => MAC_REG_MSize,
       PLB_PAValid => MAC_REG_PAValid,
       PLB_RNW => MAC_REG_RNW,
       PLB_SAValid => MAC_REG_SAValid,
       PLB_TAttribute => MAC_REG_TAttribute,
       PLB_UABus => MAC_REG_UABus,
       PLB_abort => MAC_REG_abort,
       PLB_busLock => MAC_REG_busLock,
       PLB_lockErr => MAC_REG_lockErr,
       PLB_masterID => MAC_REG_masterID( 0 to C_MAC_REG_PLB_MID_WIDTH - 1 ),
       PLB_rdBurst => MAC_REG_rdBurst,
       PLB_rdPendPri => MAC_REG_rdPendPri,
       PLB_rdPendReq => MAC_REG_rdPendReq,
       PLB_rdPrim => MAC_REG_rdPrim,
       PLB_reqPri => MAC_REG_reqPri,
       PLB_size => MAC_REG_size,
       PLB_type => MAC_REG_type,
       PLB_wrBurst => MAC_REG_wrBurst,
       PLB_wrDBus => MAC_REG_wrDBus( 0 to C_MAC_REG_PLB_DWIDTH - 1 ),
       PLB_wrPendPri => MAC_REG_wrPendPri,
       PLB_wrPendReq => MAC_REG_wrPendReq,
       PLB_wrPrim => MAC_REG_wrPrim,
       SPLB_Clk => MAC_REG_Clk,
       SPLB_Rst => MAC_REG_Rst,
       Sl_MBusy => MAC_REG_MBusy( 0 to C_MAC_REG_NUM_MASTERS-1 ),
       Sl_MIRQ => MAC_REG_MIRQ( 0 to C_MAC_REG_NUM_MASTERS-1 ),
       Sl_MRdErr => MAC_REG_MRdErr( 0 to C_MAC_REG_NUM_MASTERS-1 ),
       Sl_MWrErr => MAC_REG_MWrErr( 0 to C_MAC_REG_NUM_MASTERS-1 ),
       Sl_SSize => MAC_REG_SSize,
       Sl_addrAck => MAC_REG_addrAck,
       Sl_rdBTerm => MAC_REG_rdBTerm,
       Sl_rdComp => MAC_REG_rdComp,
       Sl_rdDAck => MAC_REG_rdDAck,
       Sl_rdDBus => MAC_REG_rdDBus( 0 to C_MAC_REG_PLB_DWIDTH-1 ),
       Sl_rdWdAddr => MAC_REG_rdWdAddr,
       Sl_rearbitrate => MAC_REG_rearbitrate,
       Sl_wait => MAC_REG_wait,
       Sl_wrBTerm => MAC_REG_wrBTerm,
       Sl_wrComp => MAC_REG_wrComp,
       Sl_wrDAck => MAC_REG_wrDAck
  );

THE_POWERLINK_IP_CORE : powerlink
  generic map (
       Simulate => false,
       endian_g => "big",
       gNumSmi => C_NUM_SMI,
       genABuf1_g => C_PDI_GEN_ASYNC_BUF_0,
       genABuf2_g => C_PDI_GEN_ASYNC_BUF_1,
       genEvent_g => C_PDI_GEN_EVENT,
       genInternalAp_g => C_GEN_PLB_BUS_IF,
       genIoBuf_g => false,
       genLedGadget_g => C_PDI_GEN_LED,
       genOnePdiClkDomain_g => false,
       genPdi_g => C_GEN_PDI,
       genSimpleIO_g => C_GEN_SIMPLE_IO,
       genSmiIO => false,
       genSpiAp_g => C_GEN_SPI_IF,
       genTimeSync_g => C_PDI_GEN_TIME_SYNC,
       gen_dma_observer_g => C_OBSERVER_ENABLE,
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
       m_burstcount_const_g => true,
       m_burstcount_width_g => C_M_BURSTCOUNT_WIDTH,
       m_data_width_g => 32,
       m_rx_burst_size_g => C_MAC_DMA_BURST_SIZE_RX/4,
       m_rx_fifo_size_g => C_M_FIFO_SIZE_RX,
       m_tx_burst_size_g => C_MAC_DMA_BURST_SIZE_TX/4,
       m_tx_fifo_size_g => C_M_FIFO_SIZE_TX,
       papBigEnd_g => false,
       papDataWidth_g => C_PAP_DATA_WIDTH,
       papLowAct_g => C_PAP_LOW_ACT,
       pcpSysId => C_PCP_SYS_ID,
       pioValLen_g => C_PIO_VAL_LENGTH,
       spiBigEnd_g => false,
       spiCPHA_g => C_SPI_CPHA,
       spiCPOL_g => C_SPI_CPOL,
       use2ndCmpTimer_g => C_PDI_GEN_SECOND_TIMER,
       use2ndPhy_g => C_USE_2ND_PHY,
       useIntPacketBuf_g => C_MAC_PKT_EN,
       useRmii_g => C_USE_RMII,
       useRxIntPacketBuf_g => C_MAC_PKT_RX_EN
  )
  port map(
       mac_address(0) => mac_address(0),
       mac_address(1) => mac_address(1),
       mac_address(2) => mac_address(2),
       mac_address(3) => mac_address(3),
       mac_address(4) => mac_address(4),
       mac_address(5) => mac_address(5),
       mac_address(6) => mac_address(6),
       mac_address(7) => mac_address(7),
       mac_address(8) => mac_address(8),
       mac_address(9) => mac_address(9),
       mac_address(10) => mac_address(10),
       mac_address(11) => mac_address(11),
       ap_address => ap_address,
       ap_asyncIrq => ap_asyncIrq,
       ap_asyncIrq_n => ap_asyncIrq_n,
       ap_byteenable => ap_byteenable,
       ap_chipselect => ap_chipselect,
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
       pap_data_I => pap_data_I( C_PAP_DATA_WIDTH-1 downto 0 ),
       pap_data_O => pap_data_O( C_PAP_DATA_WIDTH-1 downto 0 ),
       pap_data_T => pap_data_T,
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
       phy_SMIDat_I => phy_SMIDat_I,
       phy_SMIDat_O => phy_SMIDat_O,
       phy_SMIDat_T => phy_SMIDat_T,
       pio_operational => pio_operational,
       pio_pconfig => pio_pconfig,
       pio_portInLatch => pio_portInLatch,
       pio_portOutValid => pio_portOutValid,
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

rst <= Bus2MAC_REG_Reset or Bus2MAC_CMP_Reset or MAC_DMA_RST or Bus2MAC_PKT_Reset;

Bus2MAC_REG_RNW_n <= not(Bus2MAC_REG_RNW);


---- Power , ground assignment ----

VCC <= VCC_CONSTANT;
GND <= GND_CONSTANT;
MAC_REG2Bus_Error <= GND;

---- Terminal assignment ----

    -- Output\buffer terminals
	mac_irq <= mac_irq_s;
	tcp_irq <= tcp_irq_s;


----  Generate statements  ----

genMacDmaPlbBurst : if C_DMA_EN = TRUE generate
begin
  MAC_DMA_PLB_BURST_MASTER : plbv46_master_burst
    generic map (
         C_FAMILY => C_FAMILY,
         C_INHIBIT_CC_BLE_INCLUSION => 1,
         C_MPLB_AWIDTH => C_MAC_DMA_PLB_AWIDTH,
         C_MPLB_DWIDTH => C_MAC_DMA_PLB_DWIDTH,
         C_MPLB_NATIVE_DWIDTH => C_MAC_DMA_PLB_NATIVE_DWIDTH,
         C_MPLB_SMALLEST_SLAVE => 32
    )  
    port map(
         Bus2IP_MstRd_d => Bus2MAC_DMA_MstRd_d( C_MAC_DMA_PLB_NATIVE_DWIDTH-1 downto 0 ),
         Bus2IP_MstRd_eof_n => Bus2MAC_DMA_MstRd_eof_n,
         Bus2IP_MstRd_rem => Bus2MAC_DMA_MstRd_rem( 0 to (C_MAC_DMA_PLB_NATIVE_DWIDTH/8)-1 ),
         Bus2IP_MstRd_sof_n => Bus2MAC_DMA_MstRd_sof_n,
         Bus2IP_MstRd_src_dsc_n => Bus2MAC_DMA_MstRd_src_dsc_n,
         Bus2IP_MstRd_src_rdy_n => Bus2MAC_DMA_MstRd_src_rdy_n,
         Bus2IP_MstWr_dst_dsc_n => Bus2MAC_DMA_MstWr_dst_dsc_n,
         Bus2IP_MstWr_dst_rdy_n => Bus2MAC_DMA_MstWr_dst_rdy_n,
         Bus2IP_Mst_CmdAck => Bus2MAC_DMA_Mst_CmdAck,
         Bus2IP_Mst_Cmd_Timeout => Bus2MAC_DMA_Mst_Cmd_Timeout,
         Bus2IP_Mst_Cmplt => Bus2MAC_DMA_Mst_Cmplt,
         Bus2IP_Mst_Error => Bus2MAC_DMA_Mst_Error,
         Bus2IP_Mst_Rearbitrate => Bus2MAC_DMA_Mst_Rearbitrate,
         IP2Bus_MstRd_Req => MAC_DMA2Bus_MstRd_Req,
         IP2Bus_MstRd_dst_dsc_n => MAC_DMA2Bus_MstRd_dst_dsc_n,
         IP2Bus_MstRd_dst_rdy_n => MAC_DMA2Bus_MstRd_dst_rdy_n,
         IP2Bus_MstWr_Req => MAC_DMA2Bus_MstWr_Req,
         IP2Bus_MstWr_d => MAC_DMA2Bus_MstWr_d( C_MAC_DMA_PLB_NATIVE_DWIDTH-1 downto 0 ),
         IP2Bus_MstWr_eof_n => MAC_DMA2Bus_MstWr_eof_n,
         IP2Bus_MstWr_rem => MAC_DMA2Bus_MstWr_rem( 0 to (C_MAC_DMA_PLB_NATIVE_DWIDTH/8)-1 ),
         IP2Bus_MstWr_sof_n => MAC_DMA2Bus_MstWr_sof_n,
         IP2Bus_MstWr_src_dsc_n => MAC_DMA2Bus_MstWr_src_dsc_n,
         IP2Bus_MstWr_src_rdy_n => MAC_DMA2Bus_MstWr_src_rdy_n,
         IP2Bus_Mst_Addr => MAC_DMA2Bus_Mst_Addr( 0 to C_MAC_DMA_PLB_AWIDTH-1 ),
         IP2Bus_Mst_BE => MAC_DMA2Bus_Mst_BE( 0 to (C_MAC_DMA_PLB_NATIVE_DWIDTH/8)-1 ),
         IP2Bus_Mst_Length => MAC_DMA2Bus_Mst_Length,
         IP2Bus_Mst_Lock => MAC_DMA2Bus_Mst_Lock,
         IP2Bus_Mst_Reset => MAC_DMA2Bus_Mst_Reset,
         IP2Bus_Mst_Type => MAC_DMA2Bus_Mst_Type,
         MD_Error => MAC_DMA_error,
         MPLB_Clk => MAC_DMA_Clk,
         MPLB_Rst => MAC_DMA_Rst,
         M_ABus => MAC_DMA_ABus,
         M_BE => MAC_DMA_BE( 0 to (C_MAC_DMA_PLB_DWIDTH/8)-1 ),
         M_MSize => MAC_DMA_MSize,
         M_RNW => MAC_DMA_RNW,
         M_TAttribute => MAC_DMA_TAttribute,
         M_UABus => MAC_DMA_UABus,
         M_abort => MAC_DMA_abort,
         M_busLock => MAC_DMA_busLock,
         M_lockErr => MAC_DMA_lockErr,
         M_priority => MAC_DMA_priority,
         M_rdBurst => MAC_DMA_rdBurst,
         M_request => MAC_DMA_request,
         M_size => MAC_DMA_size,
         M_type => MAC_DMA_type,
         M_wrBurst => MAC_DMA_wrBurst,
         M_wrDBus => MAC_DMA_wrDBus( 0 to C_MAC_DMA_PLB_DWIDTH-1 ),
         PLB_MAddrAck => MAC_DMA_MAddrAck,
         PLB_MBusy => MAC_DMA_MBusy,
         PLB_MIRQ => MAC_DMA_MIRQ,
         PLB_MRdBTerm => MAC_DMA_MRdBTerm,
         PLB_MRdDAck => MAC_DMA_MRdDAck,
         PLB_MRdDBus => MAC_DMA_MRdDBus( 0 to C_MAC_DMA_PLB_DWIDTH-1 ),
         PLB_MRdErr => MAC_DMA_MRdErr,
         PLB_MRdWdAddr => MAC_DMA_MRdWdAddr,
         PLB_MRearbitrate => MAC_DMA_MRearbitrate,
         PLB_MSSize => MAC_DMA_MSSize,
         PLB_MTimeout => MAC_DMA_MTimeout,
         PLB_MWrBTerm => MAC_DMA_MWrBTerm,
         PLB_MWrDAck => MAC_DMA_MWrDAck,
         PLB_MWrErr => MAC_DMA_MWrErr
    );
end generate genMacDmaPlbBurst;

oddr2_0 : if not C_INSTANCE_ODDR2 generate
begin
  phy0_clk <= clk50;
  
  phy1_clk <= clk50;
end generate oddr2_0;

oddr2_1 : if C_INSTANCE_ODDR2 generate
begin
  U4 : ODDR2
    port map(
         C0 => clk50,
         C1 => NET118078,
         CE => VCC,
         D0 => VCC,
         D1 => GND,
         Q => phy0_clk,
         R => GND,
         S => GND
    );
  
  NET118078 <= not(clk50);
  
  U6 : ODDR2
    port map(
         C0 => clk50,
         C1 => NET118214,
         CE => VCC,
         D0 => VCC,
         D1 => GND,
         Q => phy1_clk,
         R => GND,
         S => GND
    );
  
  NET118214 <= not(clk50);
end generate oddr2_1;

genThePlbMaster : if C_DMA_EN = TRUE generate
begin
  THE_IPIF_MASTER_HANDLER : ipif_master_handler
    generic map (
         dma_highadr_g => m_address'high,
         gen_rx_fifo_g => not C_RX_INT_PKT,
         gen_tx_fifo_g => not C_TX_INT_PKT,
         m_burstcount_width_g => C_M_BURSTCOUNT_WIDTH
    )  
    port map(
         Bus2MAC_DMA_MstRd_d => Bus2MAC_DMA_MstRd_d_s( C_MAC_DMA_PLB_NATIVE_DWIDTH-1 downto 0 ),
         Bus2MAC_DMA_MstRd_eof_n => Bus2MAC_DMA_MstRd_eof_n,
         Bus2MAC_DMA_MstRd_rem => Bus2MAC_DMA_MstRd_rem( 0 to (C_MAC_DMA_PLB_NATIVE_DWIDTH/8)-1 ),
         Bus2MAC_DMA_MstRd_sof_n => Bus2MAC_DMA_MstRd_sof_n,
         Bus2MAC_DMA_MstRd_src_dsc_n => Bus2MAC_DMA_MstRd_src_dsc_n,
         Bus2MAC_DMA_MstRd_src_rdy_n => Bus2MAC_DMA_MstRd_src_rdy_n,
         Bus2MAC_DMA_MstWr_dst_dsc_n => Bus2MAC_DMA_MstWr_dst_dsc_n,
         Bus2MAC_DMA_MstWr_dst_rdy_n => Bus2MAC_DMA_MstWr_dst_rdy_n,
         Bus2MAC_DMA_Mst_CmdAck => Bus2MAC_DMA_Mst_CmdAck,
         Bus2MAC_DMA_Mst_Cmd_Timeout => Bus2MAC_DMA_Mst_Cmd_Timeout,
         Bus2MAC_DMA_Mst_Cmplt => Bus2MAC_DMA_Mst_Cmplt,
         Bus2MAC_DMA_Mst_Error => Bus2MAC_DMA_Mst_Error,
         Bus2MAC_DMA_Mst_Rearbitrate => Bus2MAC_DMA_Mst_Rearbitrate,
         MAC_DMA2Bus_MstRd_Req => MAC_DMA2Bus_MstRd_Req,
         MAC_DMA2Bus_MstRd_dst_dsc_n => MAC_DMA2Bus_MstRd_dst_dsc_n,
         MAC_DMA2Bus_MstRd_dst_rdy_n => MAC_DMA2Bus_MstRd_dst_rdy_n,
         MAC_DMA2Bus_MstWr_Req => MAC_DMA2Bus_MstWr_Req,
         MAC_DMA2Bus_MstWr_d => MAC_DMA2Bus_MstWr_d_s( C_MAC_DMA_PLB_NATIVE_DWIDTH-1 downto 0 ),
         MAC_DMA2Bus_MstWr_eof_n => MAC_DMA2Bus_MstWr_eof_n,
         MAC_DMA2Bus_MstWr_rem => MAC_DMA2Bus_MstWr_rem( 0 to (C_MAC_DMA_PLB_NATIVE_DWIDTH/8)-1 ),
         MAC_DMA2Bus_MstWr_sof_n => MAC_DMA2Bus_MstWr_sof_n,
         MAC_DMA2Bus_MstWr_src_dsc_n => MAC_DMA2Bus_MstWr_src_dsc_n,
         MAC_DMA2Bus_MstWr_src_rdy_n => MAC_DMA2Bus_MstWr_src_rdy_n,
         MAC_DMA2Bus_Mst_Addr => MAC_DMA2Bus_Mst_Addr( 0 to C_MAC_DMA_PLB_AWIDTH-1 ),
         MAC_DMA2Bus_Mst_BE => MAC_DMA2Bus_Mst_BE( 0 to (C_MAC_DMA_PLB_NATIVE_DWIDTH/8)-1 ),
         MAC_DMA2Bus_Mst_Length => MAC_DMA2Bus_Mst_Length,
         MAC_DMA2Bus_Mst_Lock => MAC_DMA2Bus_Mst_Lock,
         MAC_DMA2Bus_Mst_Reset => MAC_DMA2Bus_Mst_Reset,
         MAC_DMA2Bus_Mst_Type => MAC_DMA2Bus_Mst_Type,
         MAC_DMA_CLK => MAC_DMA_CLK,
         MAC_DMA_Rst => MAC_DMA_Rst,
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

  Bus2MAC_DMA_MstRd_d_s <=    Bus2MAC_DMA_MstRd_d(7 downto 0) & Bus2MAC_DMA_MstRd_d(15 downto 8) &
                            Bus2MAC_DMA_MstRd_d(23 downto 16) & Bus2MAC_DMA_MstRd_d(31 downto 24);

MAC_DMA2Bus_MstWr_d <=      MAC_DMA2Bus_MstWr_d_s(7 downto 0) & MAC_DMA2Bus_MstWr_d_s(15 downto 8) &
                            MAC_DMA2Bus_MstWr_d_s(23 downto 16) & MAC_DMA2Bus_MstWr_d_s(31 downto 24);
end generate genThePlbMaster;

genMacPktPLbSingleSlave : if C_PKT_BUF_EN generate
begin
  MAC_PKT_PLB_SINGLE_SLAVE : plbv46_slave_single
    generic map (
         C_ARD_ADDR_RANGE_ARRAY => (C_MAC_PKT_BASE,C_MAC_PKT_HIGH),
         C_ARD_NUM_CE_ARRAY => (0 => 1),
         C_BUS2CORE_CLK_RATIO => 1,
         C_FAMILY => C_FAMILY,
         C_INCLUDE_DPHASE_TIMER => 0,
         C_SIPIF_DWIDTH => C_MAC_PKT_PLB_DWIDTH,
         C_SPLB_AWIDTH => C_MAC_PKT_PLB_AWIDTH,
         C_SPLB_DWIDTH => C_MAC_PKT_PLB_DWIDTH,
         C_SPLB_MID_WIDTH => C_MAC_PKT_PLB_MID_WIDTH,
         C_SPLB_NUM_MASTERS => C_MAC_PKT_PLB_NUM_MASTERS,
         C_SPLB_P2P => C_MAC_PKT_PLB_P2P
    )  
    port map(
         Bus2IP_Addr => Bus2MAC_PKT_Addr( C_MAC_PKT_PLB_AWIDTH-1 downto 0 ),
         Bus2IP_BE => Bus2MAC_PKT_BE( (C_MAC_PKT_PLB_DWIDTH/8)-1 downto 0 ),
         Bus2IP_CS => Bus2MAC_PKT_CS( 0 downto 0 ),
         Bus2IP_Clk => Bus2MAC_PKT_Clk,
         Bus2IP_Data => Bus2MAC_PKT_Data( C_MAC_PKT_PLB_DWIDTH-1 downto 0 ),
         Bus2IP_RNW => Bus2MAC_PKT_RNW,
         Bus2IP_Reset => Bus2MAC_PKT_Reset,
         IP2Bus_Data => MAC_PKT2Bus_Data( C_MAC_PKT_PLB_DWIDTH-1 downto 0 ),
         IP2Bus_Error => MAC_PKT2Bus_Error,
         IP2Bus_RdAck => MAC_PKT2Bus_RdAck,
         IP2Bus_WrAck => MAC_PKT2Bus_WrAck,
         PLB_ABus => MAC_PKT_ABus,
         PLB_BE => MAC_PKT_BE( 0 to (C_MAC_PKT_PLB_DWIDTH/8)-1 ),
         PLB_MSize => MAC_PKT_MSize,
         PLB_PAValid => MAC_PKT_PAValid,
         PLB_RNW => MAC_PKT_RNW,
         PLB_SAValid => MAC_PKT_SAValid,
         PLB_TAttribute => MAC_PKT_TAttribute,
         PLB_UABus => MAC_PKT_UABus,
         PLB_abort => MAC_PKT_abort,
         PLB_busLock => MAC_PKT_busLock,
         PLB_lockErr => MAC_PKT_lockErr,
         PLB_masterID => MAC_PKT_masterID( 0 to C_MAC_PKT_PLB_MID_WIDTH-1 ),
         PLB_rdBurst => MAC_PKT_rdBurst,
         PLB_rdPendPri => MAC_PKT_rdPendPri,
         PLB_rdPendReq => MAC_PKT_rdPendReq,
         PLB_rdPrim => MAC_PKT_rdPrim,
         PLB_reqPri => MAC_PKT_reqPri,
         PLB_size => MAC_PKT_size,
         PLB_type => MAC_PKT_type,
         PLB_wrBurst => MAC_PKT_wrBurst,
         PLB_wrDBus => MAC_PKT_wrDBus( 0 to C_MAC_PKT_PLB_DWIDTH-1 ),
         PLB_wrPendPri => MAC_PKT_wrPendPri,
         PLB_wrPendReq => MAC_PKT_wrPendReq,
         PLB_wrPrim => MAC_PKT_wrPrim,
         SPLB_Clk => MAC_PKT_Clk,
         SPLB_Rst => MAC_PKT_Rst,
         Sl_MBusy => MAC_PKT_MBusy( 0 to C_MAC_PKT_NUM_MASTERS-1 ),
         Sl_MIRQ => MAC_PKT_MIRQ( 0 to C_MAC_PKT_NUM_MASTERS-1 ),
         Sl_MRdErr => MAC_PKT_MRdErr( 0 to C_MAC_PKT_NUM_MASTERS-1 ),
         Sl_MWrErr => MAC_PKT_MWrErr( 0 to C_MAC_PKT_NUM_MASTERS-1 ),
         Sl_SSize => MAC_PKT_SSize,
         Sl_addrAck => MAC_PKT_addrAck,
         Sl_rdBTerm => MAC_PKT_rdBTerm,
         Sl_rdComp => MAC_PKT_rdComp,
         Sl_rdDAck => MAC_PKT_rdDAck,
         Sl_rdDBus => MAC_PKT_rdDBus( 0 to C_MAC_PKT_PLB_DWIDTH-1 ),
         Sl_rdWdAddr => MAC_PKT_rdWdAddr,
         Sl_rearbitrate => MAC_PKT_rearbitrate,
         Sl_wait => MAC_PKT_wait,
         Sl_wrBTerm => MAC_PKT_wrBTerm,
         Sl_wrComp => MAC_PKT_wrComp,
         Sl_wrDAck => MAC_PKT_wrDAck
    );
end generate genMacPktPLbSingleSlave;

genPdiPcp : if (C_GEN_PDI) generate
begin
  PDI_PCP_PLB_SINGLE_SLAVE : plbv46_slave_single
    generic map (
         C_ARD_ADDR_RANGE_ARRAY => (C_PDI_PCP_BASE,C_PDI_PCP_HIGH),
         C_ARD_NUM_CE_ARRAY => (0 => 1),
         C_BUS2CORE_CLK_RATIO => 1,
         C_FAMILY => C_FAMILY,
         C_INCLUDE_DPHASE_TIMER => 0,
         C_SIPIF_DWIDTH => C_PDI_PCP_PLB_DWIDTH,
         C_SPLB_AWIDTH => C_PDI_PCP_PLB_AWIDTH,
         C_SPLB_DWIDTH => C_PDI_PCP_PLB_DWIDTH,
         C_SPLB_MID_WIDTH => C_PDI_PCP_PLB_MID_WIDTH,
         C_SPLB_NUM_MASTERS => C_PDI_PCP_PLB_NUM_MASTERS,
         C_SPLB_P2P => C_PDI_PCP_PLB_P2P
    )  
    port map(
         Bus2IP_Addr => Bus2PDI_PCP_Addr( C_PDI_PCP_PLB_AWIDTH-1 downto 0 ),
         Bus2IP_BE => Bus2PDI_PCP_BE( (C_PDI_PCP_PLB_DWIDTH/8)-1 downto 0 ),
         Bus2IP_CS => Bus2PDI_PCP_CS( 0 downto 0 ),
         Bus2IP_Clk => Bus2PDI_PCP_Clk,
         Bus2IP_Data => Bus2PDI_PCP_Data( C_PDI_PCP_PLB_DWIDTH-1 downto 0 ),
         Bus2IP_RNW => Bus2PDI_PCP_RNW,
         Bus2IP_Reset => Bus2PDI_PCP_Reset,
         IP2Bus_Data => PDI_PCP2Bus_Data( C_PDI_PCP_PLB_DWIDTH-1 downto 0 ),
         IP2Bus_Error => PDI_PCP2Bus_Error,
         IP2Bus_RdAck => PDI_PCP2Bus_RdAck,
         IP2Bus_WrAck => PDI_PCP2Bus_WrAck,
         PLB_ABus => PDI_PCP_ABus,
         PLB_BE => PDI_PCP_BE( 0 to (C_PDI_PCP_PLB_DWIDTH/8)-1 ),
         PLB_MSize => PDI_PCP_MSize,
         PLB_PAValid => PDI_PCP_PAValid,
         PLB_RNW => PDI_PCP_RNW,
         PLB_SAValid => PDI_PCP_SAValid,
         PLB_TAttribute => PDI_PCP_TAttribute,
         PLB_UABus => PDI_PCP_UABus,
         PLB_abort => PDI_PCP_abort,
         PLB_busLock => PDI_PCP_busLock,
         PLB_lockErr => PDI_PCP_lockErr,
         PLB_masterID => PDI_PCP_masterID( 0 to C_PDI_PCP_PLB_MID_WIDTH-1 ),
         PLB_rdBurst => PDI_PCP_rdBurst,
         PLB_rdPendPri => PDI_PCP_rdPendPri,
         PLB_rdPendReq => PDI_PCP_rdPendReq,
         PLB_rdPrim => PDI_PCP_rdPrim,
         PLB_reqPri => PDI_PCP_reqPri,
         PLB_size => PDI_PCP_size,
         PLB_type => PDI_PCP_type,
         PLB_wrBurst => PDI_PCP_wrBurst,
         PLB_wrDBus => PDI_PCP_wrDBus( 0 to C_PDI_PCP_PLB_DWIDTH-1 ),
         PLB_wrPendPri => PDI_PCP_wrPendPri,
         PLB_wrPendReq => PDI_PCP_wrPendReq,
         PLB_wrPrim => PDI_PCP_wrPrim,
         SPLB_Clk => PDI_PCP_Clk,
         SPLB_Rst => PDI_PCP_Rst,
         Sl_MBusy => PDI_PCP_MBusy( 0 to C_PDI_PCP_NUM_MASTERS-1 ),
         Sl_MIRQ => PDI_PCP_MIRQ( 0 to C_PDI_PCP_NUM_MASTERS-1 ),
         Sl_MRdErr => PDI_PCP_MRdErr( 0 to C_PDI_PCP_NUM_MASTERS-1 ),
         Sl_MWrErr => PDI_PCP_MWrErr( 0 to C_PDI_PCP_NUM_MASTERS-1 ),
         Sl_SSize => PDI_PCP_SSize,
         Sl_addrAck => PDI_PCP_addrAck,
         Sl_rdBTerm => PDI_PCP_rdBTerm,
         Sl_rdComp => PDI_PCP_rdComp,
         Sl_rdDAck => PDI_PCP_rdDAck,
         Sl_rdDBus => PDI_PCP_rdDBus( 0 to C_PDI_PCP_PLB_DWIDTH-1 ),
         Sl_rdWdAddr => PDI_PCP_rdWdAddr,
         Sl_rearbitrate => PDI_PCP_rearbitrate,
         Sl_wait => PDI_PCP_wait,
         Sl_wrBTerm => PDI_PCP_wrBTerm,
         Sl_wrComp => PDI_PCP_wrComp,
         Sl_wrDAck => PDI_PCP_wrDAck
    );
end generate genPdiPcp;

genPcpPdiLink : if C_GEN_PDI generate
begin
  --pdi_pcp assignments
clkPcp <= Bus2PDI_PCP_Clk;
rstPcp <= Bus2PDI_PCP_Reset;
--pcp_writedata <= Bus2PDI_PCP_Data;
pcp_writedata <=  Bus2PDI_PCP_Data(7 downto 0) & Bus2PDI_PCP_Data(15 downto 8) & Bus2PDI_PCP_Data(23 downto 16) & Bus2PDI_PCP_Data(31 downto 24);
--pcp_writedata <=  Bus2PDI_PCP_Data(15 downto 0) & Bus2PDI_PCP_Data(31 downto 16) when Bus2PDI_PCP_BE = "1100" or Bus2PDI_PCP_BE = "0011" else
--            Bus2PDI_PCP_Data(15 downto 8) & Bus2PDI_PCP_Data(7 downto 0) & Bus2PDI_PCP_Data(31 downto 24) & Bus2PDI_PCP_Data(23 downto 16) when Bus2PDI_PCP_BE = "1000" or Bus2PDI_PCP_BE = "0100" or Bus2PDI_PCP_BE = "0010" or Bus2PDI_PCP_BE = "0001" else
--            Bus2PDI_PCP_Data;

pcp_read <= Bus2PDI_PCP_RNW;
pcp_write <= not Bus2PDI_PCP_RNW;
pcp_chipselect <= Bus2PDI_PCP_CS(0);

--pcp_byteenable <= Bus2PDI_PCP_BE;
pcp_byteenable <= Bus2PDI_PCP_BE(0) & Bus2PDI_PCP_BE(1) & Bus2PDI_PCP_BE(2) & Bus2PDI_PCP_BE(3);

pcp_address <= Bus2PDI_PCP_Addr(14 downto 2);

--PDI_PCP2Bus_Data <= pcp_readdata;
PDI_PCP2Bus_Data <=  pcp_readdata(7 downto 0) & pcp_readdata(15 downto 8) & pcp_readdata(23 downto 16) & pcp_readdata(31 downto 24);

PDI_PCP2Bus_RdAck <= pcp_chipselect and pcp_read and not pcp_waitrequest;
PDI_PCP2Bus_WrAck <= pcp_chipselect and pcp_write and not pcp_waitrequest;
PDI_PCP2Bus_Error <= '0';
end generate genPcpPdiLink;

genPdiAp : if (C_GEN_PLB_BUS_IF) generate
begin
  PDI_AP_PLB_SINGLE_SLAVE : plbv46_slave_single
    generic map (
         C_ARD_ADDR_RANGE_ARRAY => (C_PDI_AP_BASE,C_PDI_AP_HIGH),
         C_ARD_NUM_CE_ARRAY => (0 => 1),
         C_BUS2CORE_CLK_RATIO => 1,
         C_FAMILY => C_FAMILY,
         C_INCLUDE_DPHASE_TIMER => 0,
         C_SIPIF_DWIDTH => C_PDI_AP_PLB_DWIDTH,
         C_SPLB_AWIDTH => C_PDI_AP_PLB_AWIDTH,
         C_SPLB_DWIDTH => C_PDI_AP_PLB_DWIDTH,
         C_SPLB_MID_WIDTH => C_PDI_AP_PLB_MID_WIDTH,
         C_SPLB_NUM_MASTERS => C_PDI_AP_PLB_NUM_MASTERS,
         C_SPLB_P2P => C_PDI_AP_PLB_P2P
    )  
    port map(
         Bus2IP_Addr => Bus2PDI_AP_Addr( C_PDI_AP_PLB_AWIDTH-1 downto 0 ),
         Bus2IP_BE => Bus2PDI_AP_BE( (C_PDI_AP_PLB_DWIDTH/8)-1 downto 0 ),
         Bus2IP_CS => Bus2PDI_AP_CS( 0 downto 0 ),
         Bus2IP_Clk => Bus2PDI_AP_Clk,
         Bus2IP_Data => Bus2PDI_AP_Data( C_PDI_AP_PLB_DWIDTH-1 downto 0 ),
         Bus2IP_RNW => Bus2PDI_AP_RNW,
         Bus2IP_Reset => Bus2PDI_AP_Reset,
         IP2Bus_Data => PDI_AP2Bus_Data( C_PDI_AP_PLB_DWIDTH-1 downto 0 ),
         IP2Bus_Error => PDI_AP2Bus_Error,
         IP2Bus_RdAck => PDI_AP2Bus_RdAck,
         IP2Bus_WrAck => PDI_AP2Bus_WrAck,
         PLB_ABus => PDI_AP_ABus,
         PLB_BE => PDI_AP_BE( 0 to (C_PDI_AP_PLB_DWIDTH/8)-1 ),
         PLB_MSize => PDI_AP_MSize,
         PLB_PAValid => PDI_AP_PAValid,
         PLB_RNW => PDI_AP_RNW,
         PLB_SAValid => PDI_AP_SAValid,
         PLB_TAttribute => PDI_AP_TAttribute,
         PLB_UABus => PDI_AP_UABus,
         PLB_abort => PDI_AP_abort,
         PLB_busLock => PDI_AP_busLock,
         PLB_lockErr => PDI_AP_lockErr,
         PLB_masterID => PDI_AP_masterID( 0 to C_PDI_AP_PLB_MID_WIDTH-1 ),
         PLB_rdBurst => PDI_AP_rdBurst,
         PLB_rdPendPri => PDI_AP_rdPendPri,
         PLB_rdPendReq => PDI_AP_rdPendReq,
         PLB_rdPrim => PDI_AP_rdPrim,
         PLB_reqPri => PDI_AP_reqPri,
         PLB_size => PDI_AP_size,
         PLB_type => PDI_AP_type,
         PLB_wrBurst => PDI_AP_wrBurst,
         PLB_wrDBus => PDI_AP_wrDBus( 0 to C_PDI_AP_PLB_DWIDTH-1 ),
         PLB_wrPendPri => PDI_AP_wrPendPri,
         PLB_wrPendReq => PDI_AP_wrPendReq,
         PLB_wrPrim => PDI_AP_wrPrim,
         SPLB_Clk => PDI_AP_Clk,
         SPLB_Rst => PDI_AP_Rst,
         Sl_MBusy => PDI_AP_MBusy( 0 to C_PDI_AP_PLB_NUM_MASTERS-1 ),
         Sl_MIRQ => PDI_AP_MIRQ( 0 to C_PDI_AP_PLB_NUM_MASTERS-1 ),
         Sl_MRdErr => PDI_AP_MRdErr( 0 to C_PDI_AP_PLB_NUM_MASTERS-1 ),
         Sl_MWrErr => PDI_AP_MWrErr( 0 to C_PDI_AP_PLB_NUM_MASTERS-1 ),
         Sl_SSize => PDI_AP_SSize,
         Sl_addrAck => PDI_AP_addrAck,
         Sl_rdBTerm => PDI_AP_rdBTerm,
         Sl_rdComp => PDI_AP_rdComp,
         Sl_rdDAck => PDI_AP_rdDAck,
         Sl_rdDBus => PDI_AP_rdDBus( 0 to C_PDI_AP_PLB_DWIDTH-1 ),
         Sl_rdWdAddr => PDI_AP_rdWdAddr,
         Sl_rearbitrate => PDI_AP_rearbitrate,
         Sl_wait => PDI_AP_wait,
         Sl_wrBTerm => PDI_AP_wrBTerm,
         Sl_wrComp => PDI_AP_wrComp,
         Sl_wrDAck => PDI_AP_wrDAck
    );
end generate genPdiAp;

genApPdiLink : if C_GEN_PDI generate
begin
  --ap_pcp assignments
clkAp <= Bus2PDI_AP_Clk;
rstAp <= Bus2PDI_AP_Reset;

--ap_writedata <=  Bus2PDI_AP_Data;
ap_writedata <=  Bus2PDI_AP_Data(7 downto 0) & Bus2PDI_AP_Data(15 downto 8) & Bus2PDI_AP_Data(23 downto 16) & Bus2PDI_AP_Data(31 downto 24);

ap_read <= Bus2PDI_AP_RNW;
ap_write <= not Bus2PDI_AP_RNW;
ap_chipselect <= Bus2PDI_AP_CS(0);
--ap_byteenable <= Bus2PDI_AP_BE;
ap_byteenable <= Bus2PDI_AP_BE(0) & Bus2PDI_AP_BE(1) & Bus2PDI_AP_BE(2) & Bus2PDI_AP_BE(3);

ap_address <= Bus2PDI_AP_Addr(14 downto 2);

--PDI_AP2Bus_Data <=  ap_readdata;
PDI_AP2Bus_Data <=  ap_readdata(7 downto 0) & ap_readdata(15 downto 8) & ap_readdata(23 downto 16) & ap_readdata(31 downto 24);

PDI_AP2Bus_RdAck <= ap_chipselect and ap_read and not ap_waitrequest;
PDI_AP2Bus_WrAck <= ap_chipselect and ap_write and not ap_waitrequest;
PDI_AP2Bus_Error <= '0';
end generate genApPdiLink;

genSimpleIoSignals : if C_GEN_SIMPLE_IO generate
begin
  --SMP_PCP assignments
clkPcp <= Bus2SMP_PCP_Clk;
rstPcp <= Bus2SMP_PCP_Reset;
--smp_writedata <= Bus2SMP_PCP_Data;
smp_writedata <=  Bus2SMP_PCP_Data(7 downto 0) & Bus2SMP_PCP_Data(15 downto 8) & Bus2SMP_PCP_Data(23 downto 16) & Bus2SMP_PCP_Data(31 downto 24);

smp_read <= Bus2SMP_PCP_RNW and Bus2SMP_PCP_CS(0);
smp_write <= not Bus2SMP_PCP_RNW and Bus2SMP_PCP_CS(0);
smp_chipselect <= Bus2SMP_PCP_CS(0);
--smp_byteenable <= Bus2SMP_PCP_BE;
smp_byteenable <= Bus2SMP_PCP_BE(0) & Bus2SMP_PCP_BE(1) & Bus2SMP_PCP_BE(2) & Bus2SMP_PCP_BE(3);
smp_address <= Bus2SMP_PCP_Addr(2);

--SMP_PCP2Bus_Data <= smp_readdata;
SMP_PCP2Bus_Data <=  smp_readdata(7 downto 0) & smp_readdata(15 downto 8) & smp_readdata(23 downto 16) & smp_readdata(31 downto 24);

SMP_PCP2Bus_RdAck <= smp_chipselect and smp_read and not smp_waitrequest;
SMP_PCP2Bus_WrAck <= smp_chipselect and smp_write and not smp_waitrequest;
SMP_PCP2Bus_Error <= '0';
end generate genSimpleIoSignals;

genSmpIo : if (C_GEN_SIMPLE_IO) generate
begin
  SMP_IO_PLB_SINGLE_SLAVE : plbv46_slave_single
    generic map (
         C_ARD_ADDR_RANGE_ARRAY => (C_SMP_PCP_BASE,C_SMP_PCP_HIGH),
         C_ARD_NUM_CE_ARRAY => (0 => 1),
         C_BUS2CORE_CLK_RATIO => 1,
         C_FAMILY => C_FAMILY,
         C_INCLUDE_DPHASE_TIMER => 0,
         C_SIPIF_DWIDTH => C_SMP_PCP_PLB_DWIDTH,
         C_SPLB_AWIDTH => C_SMP_PCP_PLB_AWIDTH,
         C_SPLB_DWIDTH => C_SMP_PCP_PLB_DWIDTH,
         C_SPLB_MID_WIDTH => C_SMP_PCP_PLB_MID_WIDTH,
         C_SPLB_NUM_MASTERS => C_SMP_PCP_PLB_NUM_MASTERS,
         C_SPLB_P2P => C_SMP_PCP_PLB_P2P
    )  
    port map(
         Bus2IP_Addr => Bus2SMP_PCP_Addr( C_SMP_PCP_PLB_AWIDTH-1 downto 0 ),
         Bus2IP_BE => Bus2SMP_PCP_BE( (C_SMP_PCP_PLB_DWIDTH/8)-1 downto 0 ),
         Bus2IP_CS => Bus2SMP_PCP_CS( 0 downto 0 ),
         Bus2IP_Clk => Bus2SMP_PCP_Clk,
         Bus2IP_Data => Bus2SMP_PCP_Data( C_SMP_PCP_PLB_DWIDTH-1 downto 0 ),
         Bus2IP_RNW => Bus2SMP_PCP_RNW,
         Bus2IP_Reset => Bus2SMP_PCP_Reset,
         IP2Bus_Data => SMP_PCP2Bus_Data( C_SMP_PCP_PLB_DWIDTH-1 downto 0 ),
         IP2Bus_Error => SMP_PCP2Bus_Error,
         IP2Bus_RdAck => SMP_PCP2Bus_RdAck,
         IP2Bus_WrAck => SMP_PCP2Bus_WrAck,
         PLB_ABus => SMP_PCP_ABus,
         PLB_BE => SMP_PCP_BE( 0 to (C_SMP_PCP_PLB_DWIDTH/8)-1 ),
         PLB_MSize => SMP_PCP_MSize,
         PLB_PAValid => SMP_PCP_PAValid,
         PLB_RNW => SMP_PCP_RNW,
         PLB_SAValid => SMP_PCP_SAValid,
         PLB_TAttribute => SMP_PCP_TAttribute,
         PLB_UABus => SMP_PCP_UABus,
         PLB_abort => SMP_PCP_abort,
         PLB_busLock => SMP_PCP_busLock,
         PLB_lockErr => SMP_PCP_lockErr,
         PLB_masterID => SMP_PCP_masterID( 0 to C_SMP_PCP_PLB_MID_WIDTH-1 ),
         PLB_rdBurst => SMP_PCP_rdBurst,
         PLB_rdPendPri => SMP_PCP_rdPendPri,
         PLB_rdPendReq => SMP_PCP_rdPendReq,
         PLB_rdPrim => SMP_PCP_rdPrim,
         PLB_reqPri => SMP_PCP_reqPri,
         PLB_size => SMP_PCP_size,
         PLB_type => SMP_PCP_type,
         PLB_wrBurst => SMP_PCP_wrBurst,
         PLB_wrDBus => SMP_PCP_wrDBus( 0 to C_SMP_PCP_PLB_DWIDTH-1 ),
         PLB_wrPendPri => SMP_PCP_wrPendPri,
         PLB_wrPendReq => SMP_PCP_wrPendReq,
         PLB_wrPrim => SMP_PCP_wrPrim,
         SPLB_Clk => SMP_PCP_Clk,
         SPLB_Rst => SMP_PCP_Rst,
         Sl_MBusy => SMP_PCP_MBusy( 0 to C_SMP_PCP_PLB_NUM_MASTERS-1 ),
         Sl_MIRQ => SMP_PCP_MIRQ( 0 to C_SMP_PCP_PLB_NUM_MASTERS-1 ),
         Sl_MRdErr => SMP_PCP_MRdErr( 0 to C_SMP_PCP_PLB_NUM_MASTERS-1 ),
         Sl_MWrErr => SMP_PCP_MWrErr( 0 to C_SMP_PCP_PLB_NUM_MASTERS-1 ),
         Sl_SSize => SMP_PCP_SSize,
         Sl_addrAck => SMP_PCP_addrAck,
         Sl_rdBTerm => SMP_PCP_rdBTerm,
         Sl_rdComp => SMP_PCP_rdComp,
         Sl_rdDAck => SMP_PCP_rdDAck,
         Sl_rdDBus => SMP_PCP_rdDBus( 0 to C_SMP_PCP_PLB_DWIDTH-1 ),
         Sl_rdWdAddr => SMP_PCP_rdWdAddr,
         Sl_rearbitrate => SMP_PCP_rearbitrate,
         Sl_wait => SMP_PCP_wait,
         Sl_wrBTerm => SMP_PCP_wrBTerm,
         Sl_wrComp => SMP_PCP_wrComp,
         Sl_wrDAck => SMP_PCP_wrDAck
    );
end generate genSmpIo;

end struct;
