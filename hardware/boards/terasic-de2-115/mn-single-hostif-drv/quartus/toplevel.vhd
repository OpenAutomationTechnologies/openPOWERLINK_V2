-------------------------------------------------------------------------------
--! @file toplevel.vhd
--
--! @brief Toplevel of Nios MN design Pcp part
--
--! @details This is the toplevel of the Nios MN FPGA Pcp design for the
--! INK DE2-115 Evaluation Board.
--
-------------------------------------------------------------------------------
--
--    (c) B&R, 2013
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

library ieee;
use ieee.std_logic_1164.all;

library libcommon;
use libcommon.global.all;

entity toplevel is
    port (
        -- 50 MHZ CLK IN
        EXT_CLK         : in    std_logic;
        -- PHY Interfaces
        PHY_GXCLK       : out   std_logic_vector(1 downto 0);
        PHY_RXCLK       : in    std_logic_vector(1 downto 0);
        PHY_RXER        : in    std_logic_vector(1 downto 0);
        PHY_RXDV        : in    std_logic_vector(1 downto 0);
        PHY_RXD         : in    std_logic_vector(7 downto 0);
        PHY_TXCLK       : in    std_logic_vector(1 downto 0);
        PHY_TXER        : out   std_logic_vector(1 downto 0);
        PHY_TXEN        : out   std_logic_vector(1 downto 0);
        PHY_TXD         : out   std_logic_vector(7 downto 0);
        PHY_MDIO        : inout std_logic_vector(1 downto 0);
        PHY_MDC         : out   std_logic_vector(1 downto 0);
        PHY_RESET_n     : out   std_logic_vector(1 downto 0);
        -- EPCS
        EPCS_DCLK       : out   std_logic;
        EPCS_SCE        : out   std_logic;
        EPCS_SDO        : out   std_logic;
        EPCS_DATA0      : in    std_logic;
        -- 2 MB SRAM
        SRAM_CE_n       : out   std_logic;
        SRAM_OE_n       : out   std_logic;
        SRAM_WE_n       : out   std_logic;
        SRAM_ADDR       : out   std_logic_vector(20 downto 1);
        SRAM_BE_n       : out   std_logic_vector(1 downto 0);
        SRAM_DQ         : inout std_logic_vector(15 downto 0);
        -- HOST Interface
        HOSTIF_AD       : inout std_logic_vector(16 downto 0);
        HOSTIF_BE       : in    std_logic_vector(1 downto 0);
        HOSTIF_CS_n     : in    std_logic;
        HOSTIF_WR_n     : in    std_logic;
        HOSTIF_RD_n     : in    std_logic;
        HOSTIF_ALE_n    : in    std_logic;
        HOSTIF_ACK_n    : out   std_logic;
        HOSTIF_IRQ_n    : out   std_logic;
        -- LED green
        LEDG            : out   std_logic_vector(1 downto 0)
    );
end toplevel;

architecture rtl of toplevel is

    component mnSingleHostifDrv is
        port (
            clk25_clk                                   : in    std_logic;
            clk50_clk                                   : in    std_logic                     := 'X';
            clk100_clk                                  : in    std_logic;
            reset_reset_n                               : in    std_logic                     := 'X';

            tri_state_0_tcm_address_out                 : out   std_logic_vector(20 downto 0);
            tri_state_0_tcm_byteenable_n_out            : out   std_logic_vector(1 downto 0);
            tri_state_0_tcm_read_n_out                  : out   std_logic;
            tri_state_0_tcm_write_n_out                 : out   std_logic;
            tri_state_0_tcm_data_out                    : inout std_logic_vector(15 downto 0) := (others => 'X');
            tri_state_0_tcm_chipselect_n_out            : out   std_logic;
            pcp_0_benchmark_pio_export                  : out   std_logic_vector(7 downto 0);
            -- OPENMAC
            openmac_0_mii_txEnable                      : out   std_logic_vector(1 downto 0);
            openmac_0_mii_txData                        : out   std_logic_vector(7 downto 0);
            openmac_0_mii_txClk                         : in    std_logic_vector(1 downto 0)  := (others => 'X');
            openmac_0_mii_rxError                       : in    std_logic_vector(1 downto 0)  := (others => 'X');
            openmac_0_mii_rxDataValid                   : in    std_logic_vector(1 downto 0)  := (others => 'X');
            openmac_0_mii_rxData                        : in    std_logic_vector(7 downto 0)  := (others => 'X');
            openmac_0_mii_rxClk                         : in    std_logic_vector(1 downto 0)  := (others => 'X');
            openmac_0_smi_nPhyRst                       : out   std_logic_vector(1 downto 0);
            openmac_0_smi_clk                           : out   std_logic_vector(1 downto 0);
            openmac_0_smi_dio                           : inout std_logic_vector(1 downto 0)  := (others => 'X');
            epcs_flash_dclk                             : out   std_logic;
            epcs_flash_sce                              : out   std_logic;
            epcs_flash_sdo                              : out   std_logic;
            epcs_flash_data0                            : in    std_logic                     := 'X';
            hostinterface_0_irqout_irq                  : out   std_logic;
            prl0_iPrlSlv_cs                             : in    std_logic                     := 'X';
            prl0_iPrlSlv_rd                             : in    std_logic                     := 'X';
            prl0_iPrlSlv_wr                             : in    std_logic                     := 'X';
            prl0_iPrlSlv_ale                            : in    std_logic                     := 'X';
            prl0_oPrlSlv_ack                            : out   std_logic;
            prl0_iPrlSlv_be                             : in    std_logic_vector(1 downto 0)  := (others => 'X');
            prl0_oPrlSlv_ad_o                           : out   std_logic_vector(16 downto 0);
            prl0_iPrlSlv_ad_i                           : in    std_logic_vector(16 downto 0) := (others => 'X');
            prl0_oPrlSlv_ad_oen                         : out   std_logic;
            -- CPU RESET REQUEST
            pcp_0_cpu_resetrequest_resetrequest         : in    std_logic                     := 'X';
            pcp_0_cpu_resetrequest_resettaken           : out   std_logic;
            -- LED green
            powerlink_led_export                        : out   std_logic_vector(1 downto 0)
        );
    end component mnSingleHostifDrv;

    -- PLL component
    component pll
        port (
            inclk0  : in std_logic;
            c0      : out std_logic;
            c1      : out std_logic;
            c2      : out std_logic;
            c3      : out std_logic;
            locked  : out std_logic
        );
    end component;

    signal clk25        : std_logic;
    signal clk50        : std_logic;
    signal clk100       : std_logic;
    signal clk100_p     : std_logic;
    signal pllLocked    : std_logic;
    signal sramAddr     : std_logic_vector(SRAM_ADDR'high downto 0);

    signal parHost_chipselect           : std_logic;
    signal parHost_read                 : std_logic;
    signal parHost_write                : std_logic;
    signal parHost_addressLatchEnable   : std_logic;
    signal parHost_acknowledge          : std_logic;
    signal parHost_ad_o                 : std_logic_vector(HOSTIF_AD'range);
    signal parHost_ad_i                 : std_logic_vector(HOSTIF_AD'range);
    signal parHost_ad_oen               : std_logic;

    signal host_irq                     : std_logic;
begin

    SRAM_ADDR <= sramAddr(SRAM_ADDR'range);

    PHY_GXCLK   <= (others => '0');
    PHY_TXER    <= (others => '0');

    HOSTIF_ACK_n                <= not parHost_acknowledge;
    HOSTIF_IRQ_n                <= not host_irq;
    parHost_chipselect          <= not HOSTIF_CS_n;
    parHost_write               <= not HOSTIF_WR_n;
    parHost_read                <= not HOSTIF_RD_n;
    parHost_addressLatchEnable  <= not HOSTIF_ALE_n;

    -- TRISTATE Buffer for AD bus
    HOSTIF_AD <= parHost_ad_o when parHost_ad_oen = '1' else (others => 'Z');
    parHost_ad_i <= HOSTIF_AD;

    inst : component mnSingleHostifDrv
        port map (
            clk25_clk                                   => clk25,
            clk50_clk                                   => clk50,
            clk100_clk                                  => clk100,
            reset_reset_n                               => pllLocked,

            pcp_0_cpu_resetrequest_resetrequest         => '0',
            pcp_0_cpu_resetrequest_resettaken           => open,

            openmac_0_mii_txEnable                      => PHY_TXEN,
            openmac_0_mii_txData                        => PHY_TXD,
            openmac_0_mii_txClk                         => PHY_TXCLK,
            openmac_0_mii_rxError                       => PHY_RXER,
            openmac_0_mii_rxDataValid                   => PHY_RXDV,
            openmac_0_mii_rxData                        => PHY_RXD,
            openmac_0_mii_rxClk                         => PHY_RXCLK,
            openmac_0_smi_nPhyRst                       => PHY_RESET_n,
            openmac_0_smi_clk                           => PHY_MDC,
            openmac_0_smi_dio                           => PHY_MDIO,

            tri_state_0_tcm_address_out                 => sramAddr,
            tri_state_0_tcm_read_n_out                  => SRAM_OE_n,
            tri_state_0_tcm_byteenable_n_out            => SRAM_BE_n,
            tri_state_0_tcm_write_n_out                 => SRAM_WE_n,
            tri_state_0_tcm_data_out                    => SRAM_DQ,
            tri_state_0_tcm_chipselect_n_out            => SRAM_CE_n,

            pcp_0_benchmark_pio_export                  => open,

            epcs_flash_dclk                             => EPCS_DCLK,
            epcs_flash_sce                              => EPCS_SCE,
            epcs_flash_sdo                              => EPCS_SDO,
            epcs_flash_data0                            => EPCS_DATA0,

            hostinterface_0_irqout_irq                  => host_irq,

            prl0_iPrlSlv_cs                             => parHost_chipselect,
            prl0_iPrlSlv_rd                             => parHost_read,
            prl0_iPrlSlv_wr                             => parHost_write,
            prl0_iPrlSlv_ale                            => parHost_addressLatchEnable,
            prl0_oPrlSlv_ack                            => parHost_acknowledge,
            prl0_iPrlSlv_be                             => HOSTIF_BE,
            prl0_oPrlSlv_ad_o                           => parHost_ad_o,
            prl0_iPrlSlv_ad_i                           => parHost_ad_i,
            prl0_oPrlSlv_ad_oen                         => parHost_ad_oen,

            powerlink_led_export                        => LEDG
        );

    -- Pll Instance
    pllInst : pll
        port map (
            inclk0  => EXT_CLK,
            c0      => clk50,
            c1      => clk100,
            c2      => clk25,
            c3      => open,
            locked  => pllLocked
        );

end rtl;
