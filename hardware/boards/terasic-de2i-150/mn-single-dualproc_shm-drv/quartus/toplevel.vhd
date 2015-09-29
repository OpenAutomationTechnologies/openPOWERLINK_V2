-------------------------------------------------------------------------------
--! @file toplevel.vhd
--
--! @brief Toplevel of DE2i-150 development board PCIe-MN design part
--
--! @details This is the toplevel of the Nios MN FPGA Pcp design for the
--! Terasic DE2i-150 development Board.
--
-------------------------------------------------------------------------------
--
--    Copyright (c) 2015, Kalycito Infotech Private Limited.

--    Redistribution and use in source and binary forms, with or without
--    modification, are permitted provided that the following conditions
--    are met:
--
--    1. Redistributions of source code must retain the above copyright
--       notice, this list of conditions and the following disclaimer.
--
--    2. Redistributions in binary form must reproduce the above copyright
--       notice, this list of conditions and the following disclaimer in the
--       documentation and-or other materials provided with the distribution.
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
--! Use IEEE Library
library ieee;
--! Use Standard logic 1164 package
use ieee.std_logic_1164.all;

--! Use libcommon library
library libcommon;
--! Use global package
use libcommon.global.all;

entity toplevel is
    port (
        --------- CLOCK ---------
        --! External 50Mhz Clock input
        CLOCK_50           : in    std_logic;
        --------- ENET ---------
        --! Ethernet GTX Clock
        ENET_GTX_CLK        : out   std_logic;
        --! Ethernet active low INIT
        ENET_INT_N          : in    std_logic;
        --! Ethernet link100 input
        ENET_LINK100        : in    std_logic;
        --! Ethernet MDC
        ENET_MDC            : out   std_logic_vector(0 downto 0);
        --! Ethernet MDIO
        ENET_MDIO           : inout std_logic_vector(0 downto 0);
        --! Ethernet Active low Reset
        ENET_RST_N          : out   std_logic_vector(0 downto 0);
        --! Ethernet Rx Clock
        ENET_RX_CLK         : in    std_logic_vector(0 downto 0);
        --! Ethernet RX collision
        ENET_RX_COL         : in    std_logic;
        --! Ethernet CRS
        ENET_RX_CRS         : in    std_logic;
        --! Ethernet Rx Data
        ENET_RX_DATA        : in    std_logic_vector (3 downto 0);
        --! Ethernet Rx Data Valid
        ENET_RX_DV          : in    std_logic_vector(0 downto 0);
        --! Ethernet Rx Error
        ENET_RX_ER          : in    std_logic_vector(0 downto 0);
        --! Ethernet Tx Clock
        ENET_TX_CLK         : in    std_logic_vector(0 downto 0);
        --! Ethernet Tx Data
        ENET_TX_DATA        : out   std_logic_vector(3 downto 0);
        --! Ethernet Tx Enable
        ENET_TX_EN          : out   std_logic_vector(0 downto 0);
        --! Ethernet Tx Error
        ENET_TX_ER          : out   std_logic;
        --------- FAN ---------
        --! FAN control pins
        FAN_CTRL            : inout   std_logic;
        --------- FLASH MEMORY ---------
        --! Flash active low Chip Enable
        FL_CE_N             : out   std_logic_vector(0 downto 0);
        --! Flash active low Output Enable
        FL_OE_N             : out   std_logic_vector(0 downto 0);
        --! Flash Ready/Busy
        FL_RY               : in    std_logic;
        --! Flash Active low Write Enable
        FL_WE_N             : out   std_logic_vector(0 downto 0);
        -- Flash Active low Write
        FL_WP_N             : out   std_logic;
        --! Flash Active low Reset
        FL_RESET_N          : out   std_logic;
        --------- Flash SSRAM Shared PINS ---------
        --! Flash-SSRAM Data
        FS_DQ               : inout std_logic_vector (31 downto 0);
        --! Flash-SSRAN address
        FS_ADDR             : out   std_logic_vector (21 downto 0);
        --------- GPIO ---------
        GPIO                : inout std_logic_vector (35 downto 0);
        --------- KEY ---------
        --! Key input
        KEY  :   in     std_logic_vector (3 downto 0);
        --------- LEDG ---------
        --! LED Green
        LEDG     :   out     std_logic_vector (8 downto 0);
        --------- LEDR ---------
        --! LED Red
        LEDR : out   std_logic_vector (17 downto 0);
        --------- PCIE ---------
        --! PCIE Active low PReset
        PCIE_PERST_N     :   in  std_logic;
        --! PCIE Reference Clock
        PCIE_REFCLK_P    :   in  std_logic;
        --! PCIE RX
        PCIE_RX_P        :   in  std_logic_vector (0 downto 0);
        --! PCIE TX
        PCIE_TX_P        :   out  std_logic_vector (0 downto 0);
        --! PCIE Wakeup
        PCIE_WAKE_N      :   out std_logic;
        --------- SSRAM ---------
        --! SSRAM active low ADSC
        SSRAM_ADSC_N     : out   std_logic_vector(0 downto 0);
        --! SSRAM active low ADSP
        SSRAM_ADSP_N     : out   std_logic;
        --! SSRAM active low ADV
        SSRAM_ADV_N      : out   std_logic;
        --! SSRAM BE
        SSRAM_BE         : out   std_logic_vector (3 downto 0);
        --! SSRAM Clock
        SSRAM_CLK        : out   std_logic;
        --! SSRAM GW
        SSRAM_GW_N       : out   std_logic;
        --! SSRAM OE
        SSRAM_OE_N       : out   std_logic_vector(0 downto 0);
        --! SSRAM WE
        SSRAM_WE_N       : out   std_logic_vector(0 downto 0);
        --! SSRAM-0 Chip Enable
        SSRAM0_CE_N      : out   std_logic_vector(0 downto 0)
      );
end toplevel;

architecture rtl of toplevel is

-------------------------------------------------------
--  Signal declarations
---------------------------------------------------------

signal hb_50                : std_logic;
signal reset_n              : std_logic;
signal ledRG                : std_logic_vector (31 downto 0);
signal ledRG_b              : std_logic_vector (31 downto 0);

signal clk50                : std_logic;
signal clk100               : std_logic;
signal clk125               : std_logic;
signal pllLocked            : std_logic;

signal flash_reset_n        : std_logic_vector(0 downto 0);
signal nios_flash_reset_n   : std_logic;

    --! PLL component
    component pll
        port (
            inclk0  : in std_logic;
            c0      : out std_logic;
            c1      : out std_logic;
            c2      : out std_logic;
            locked  : out std_logic
        );
    end component pll;

    --! mnSingleDualProcDrv Component
    component mnSingleDualProcDrv is
        port (
            clk_clk                                    : in    std_logic                     := 'X';             -- clk
            clk_100_clk                                : in    std_logic                     := 'X';             -- clk
            clk_125_clk                                : in    std_logic                     := 'X';           -- clk
            reset_reset_n                              : in    std_logic                     := 'X';             -- reset_n
            led_external_connection_export             : out   std_logic_vector(20 downto 0);                    -- export
            button_external_connection_export          : in    std_logic_vector(3 downto 0)  := (others => 'X'); -- export
            pcie_ip_reconfig_togxb_data                : in    std_logic_vector(3 downto 0)  := (others => 'X'); -- data
            pcie_ip_refclk_export                      : in    std_logic                     := 'X';             -- export
            pcie_ip_test_in_test_in                    : in    std_logic_vector(39 downto 0) := (others => 'X'); -- test_in
            pcie_ip_pcie_rstn_export                   : in    std_logic                     := 'X';             -- export
            pcie_ip_clocks_sim_clk250_export           : out   std_logic;                                        -- clk250_export
            pcie_ip_clocks_sim_clk500_export           : out   std_logic;                                        -- clk500_export
            pcie_ip_clocks_sim_clk125_export           : out   std_logic;                                        -- clk125_export
            pcie_ip_reconfig_busy_busy_altgxb_reconfig : in    std_logic                     := 'X';             -- busy_altgxb_reconfig
            pcie_ip_pipe_ext_pipe_mode                 : in    std_logic                     := 'X';             -- pipe_mode
            pcie_ip_pipe_ext_phystatus_ext             : in    std_logic                     := 'X';             -- phystatus_ext
            pcie_ip_pipe_ext_rate_ext                  : out   std_logic;                                        -- rate_ext
            pcie_ip_pipe_ext_powerdown_ext             : out   std_logic_vector(1 downto 0);                     -- powerdown_ext
            pcie_ip_pipe_ext_txdetectrx_ext            : out   std_logic;                                        -- txdetectrx_ext
            pcie_ip_pipe_ext_rxelecidle0_ext           : in    std_logic                     := 'X';             -- rxelecidle0_ext
            pcie_ip_pipe_ext_rxdata0_ext               : in    std_logic_vector(7 downto 0)  := (others => 'X'); -- rxdata0_ext
            pcie_ip_pipe_ext_rxstatus0_ext             : in    std_logic_vector(2 downto 0)  := (others => 'X'); -- rxstatus0_ext
            pcie_ip_pipe_ext_rxvalid0_ext              : in    std_logic                     := 'X';             -- rxvalid0_ext
            pcie_ip_pipe_ext_rxdatak0_ext              : in    std_logic                     := 'X';             -- rxdatak0_ext
            pcie_ip_pipe_ext_txdata0_ext               : out   std_logic_vector(7 downto 0);                     -- txdata0_ext
            pcie_ip_pipe_ext_txdatak0_ext              : out   std_logic;                                        -- txdatak0_ext
            pcie_ip_pipe_ext_rxpolarity0_ext           : out   std_logic;                                        -- rxpolarity0_ext
            pcie_ip_pipe_ext_txcompl0_ext              : out   std_logic;                                        -- txcompl0_ext
            pcie_ip_pipe_ext_txelecidle0_ext           : out   std_logic;                                        -- txelecidle0_ext
            pcie_ip_rx_in_rx_datain_0                  : in    std_logic                     := 'X';             -- rx_datain_0
            pcie_ip_tx_out_tx_dataout_0                : out   std_logic;                                        -- tx_dataout_0
            pcie_ip_reconfig_fromgxb_0_data            : out   std_logic_vector(4 downto 0);                     -- data
            pcie_ip_powerdown_pll_powerdown            : in    std_logic                     := 'X';             -- pll_powerdown
            pcie_ip_powerdown_gxb_powerdown            : in    std_logic                     := 'X';              -- gxb_powerdown
            pcp_0_cpu_resetrequest_resetrequest        : in    std_logic                     := 'X';             -- resetrequest
            pcp_0_cpu_resetrequest_resettaken          : out   std_logic;                                        -- resettaken
            tri_state_0_tcm_data_out                   : inout std_logic_vector(31 downto 0) := (others => 'X'); -- DATA
            tri_state_0_tcm_byteenable_n_out           : out   std_logic_vector(3 downto 0);                     -- SSRAM_BE_N
            tri_state_0_tcm_address_out                : out   std_logic_vector(21 downto 0);                    -- ADDR
            tri_state_0_tcm_begintransfer_n_out        : out   std_logic_vector(0 downto 0);                     -- SSRAM_BT_N
            tri_state_0_tcm_write_n_out                : out   std_logic_vector(0 downto 0);                     -- SSRAM_WR_N
            tri_state_0_tcm_chipselect_n_out           : out   std_logic_vector(0 downto 0);                     -- SSRAM_0_CS_N
            tri_state_0_tcm_outputenable_n_out         : out   std_logic_vector(0 downto 0);                     -- SSRAM_OE_N
            openmac_mii_txEnable                       : out   std_logic_vector(0 downto 0);                     -- txEnable
            openmac_mii_txData                         : out   std_logic_vector(3 downto 0);                     -- txData
            openmac_mii_txClk                          : in    std_logic_vector(0 downto 0)  := (others => 'X'); -- txClk
            openmac_mii_rxError                        : in    std_logic_vector(0 downto 0)  := (others => 'X'); -- rxError
            openmac_mii_rxDataValid                    : in    std_logic_vector(0 downto 0)  := (others => 'X'); -- rxDataValid
            openmac_mii_rxData                         : in    std_logic_vector(3 downto 0)  := (others => 'X'); -- rxData
            openmac_mii_rxClk                          : in    std_logic_vector(0 downto 0)  := (others => 'X'); -- rxClk
            openmac_smi_nPhyRst                        : out   std_logic_vector(0 downto 0);                     -- nPhyRst
            openmac_smi_clk                            : out   std_logic_vector(0 downto 0);                     -- clk
            openmac_smi_dio                            : inout std_logic_vector(0 downto 0)  := (others => 'X'); -- dio
            benchmark_pio_export                       : out   std_logic_vector(7 downto 0)
            );
    end component mnSingleDualProcDrv;

begin

    -- PIN Declarations
    FL_RESET_N             <= flash_reset_n(0) and nios_flash_reset_n;
    SSRAM_CLK              <= clk100;
    PCIE_WAKE_N            <= cnInactivated;
    FL_WP_N                <= cnInactivated;
    SSRAM_GW_N             <= cnInactivated;
    SSRAM_ADSP_N           <= cnInactivated;
    SSRAM_ADV_N            <= cnInactivated;
    LEDR(17)               <= hb_50;

    LEDR(7 downto 0)       <= ledRG_b(7 downto 0);
    LEDR(16 downto 8)      <= ledRG (12 downto 4);
    LEDG(7 downto 4)       <= ledRG (3 downto 0);
    LEDG(3 downto 0)       <= b"1111";
    ENET_TX_ER             <= cInactivated;
    ENET_GTX_CLK           <= cInactivated;
    reset_n                <= pllLocked;

    --! Qsys module Instantiation
    inst : component  mnSingleDualProcDrv
         port map (
            clk_clk                                    => clk50,
            clk_100_clk                                => clk100,
            clk_125_clk                                => clk125,
            reset_reset_n                              => reset_n,
            pcie_ip_refclk_export                      => PCIE_REFCLK_P,
            pcie_ip_pcie_rstn_export                   => PCIE_PERST_N,
            pcie_ip_rx_in_rx_datain_0                  => PCIE_RX_P(0),
            pcie_ip_tx_out_tx_dataout_0                => PCIE_TX_P(0),
            pcie_ip_reconfig_fromgxb_0_data            => open,
            pcie_ip_clocks_sim_clk250_export           => open,
            pcie_ip_clocks_sim_clk500_export           => open,
            pcie_ip_clocks_sim_clk125_export           => open,
            pcie_ip_reconfig_busy_busy_altgxb_reconfig => open,
            pcie_ip_pipe_ext_pipe_mode                 => cInactivated,
            pcie_ip_pipe_ext_phystatus_ext             => cInactivated,
            pcie_ip_pipe_ext_rate_ext                  => open,
            pcie_ip_pipe_ext_powerdown_ext             => open,
            pcie_ip_pipe_ext_txdetectrx_ext            => open,
            pcie_ip_pipe_ext_rxelecidle0_ext           => cInactivated,
            pcie_ip_pipe_ext_rxdata0_ext               => (others => cInactivated),
            pcie_ip_pipe_ext_rxstatus0_ext             => (others => cInactivated),
            pcie_ip_pipe_ext_rxvalid0_ext              => cInactivated,
            pcie_ip_pipe_ext_rxdatak0_ext              => cInactivated,
            pcie_ip_pipe_ext_txdata0_ext               => open,
            pcie_ip_pipe_ext_txdatak0_ext              => open,
            pcie_ip_pipe_ext_rxpolarity0_ext           => open,
            pcie_ip_pipe_ext_txcompl0_ext              => open,
            pcie_ip_pipe_ext_txelecidle0_ext           => open,
            pcie_ip_reconfig_togxb_data                => open,
            pcp_0_cpu_resetrequest_resetrequest        => '0',
            pcp_0_cpu_resetrequest_resettaken          => open,
            pcie_ip_powerdown_pll_powerdown            => cInactivated,
            pcie_ip_powerdown_gxb_powerdown            => cInactivated,
            led_external_connection_export             => ledRG(20 downto 0),
            button_external_connection_export          => KEY,
            tri_state_0_tcm_data_out                   => FS_DQ,
            tri_state_0_tcm_byteenable_n_out           => SSRAM_BE,
            tri_state_0_tcm_address_out                => FS_ADDR,
            tri_state_0_tcm_begintransfer_n_out        => SSRAM_ADSC_N,
            tri_state_0_tcm_write_n_out                => SSRAM_WE_N,
            tri_state_0_tcm_chipselect_n_out           => SSRAM0_CE_N,
            tri_state_0_tcm_outputenable_n_out         => SSRAM_OE_N,
            openmac_mii_txEnable                       => ENET_TX_EN,
            openmac_mii_txData                         => ENET_TX_DATA,
            openmac_mii_txClk                          => ENET_TX_CLK,
            openmac_mii_rxError                        => ENET_RX_ER,
            openmac_mii_rxDataValid                    => ENET_RX_DV,
            openmac_mii_rxData                         => ENET_RX_DATA,
            openmac_mii_rxClk                          => ENET_RX_CLK,
            openmac_smi_nPhyRst                        => ENET_RST_N,
            openmac_smi_clk                            => ENET_MDC,
            openmac_smi_dio                            => ENET_MDIO,
            benchmark_pio_export                       => ledRG_b( 7 downto 0)
            );

   --! PLL Instantiation
    PLL1: component pll
        port map (
           inclk0   => CLOCK_50,
           c0       => clk50,
           c1       => clk100,
           c2       => clk125,
           locked   => pllLocked
        );

end rtl;
