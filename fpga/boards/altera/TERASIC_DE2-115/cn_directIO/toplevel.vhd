-------------------------------------------------------------------------------
--! @file toplevel.vhd
--
--! @brief Toplevel of Nios CN FPGA directIO part
--
--! @details This is the toplevel of the Nios CN FPGA directIO design for the
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

library work;

entity toplevel is
    port (
        -- 50 MHZ CLK IN:
        EXT_CLK             :  in  std_logic;
        -- PHY0 Interface:
        PHY0_GXCLK          :  out  std_logic;
        PHY0_RXCLK          :  in  std_logic;
        PHY0_RXER           :  in  std_logic;
        PHY0_RXDV           :  in  std_logic;
        PHY0_RXD            :  in  std_logic_vector(3 downto 0);
        PHY0_TXCLK          :  in  std_logic;
        PHY0_TXER           :  out  std_logic;
        PHY0_TXEN           :  out  std_logic;
        PHY0_TXD            :  out  std_logic_vector(3 downto 0);
        PHY0_LINK           :  in  std_logic;
        PHY0_MDIO           :  inout  std_logic;
        PHY0_MDC            :  out  std_logic;
        PHY0_RESET_n        :  out  std_logic;
        -- PHY1 Interface:
        PHY1_GXCLK          :  out  std_logic;
        PHY1_RXCLK          :  in  std_logic;
        PHY1_RXER           :  in  std_logic;
        PHY1_RXDV           :  in  std_logic;
        PHY1_RXD            :  in  std_logic_vector(3 downto 0);
        PHY1_TXCLK          :  in  std_logic;
        PHY1_TXER           :  out  std_logic;
        PHY1_TXEN           :  out  std_logic;
        PHY1_TXD            :  out  std_logic_vector(3 downto 0);
        PHY1_LINK           :  in  std_logic;
        PHY1_MDIO           :  inout  std_logic;
        PHY1_MDC            :  out  std_logic;
        PHY1_RESET_n        :  out  std_logic;
        -- EPCS:
        EPCS_DCLK           :  out  std_logic;
        EPCS_SCE            :  out  std_logic;
        EPCS_SDO            :  out  std_logic;
        EPCS_DATA0          :  in   std_logic;
        -- 2 MB SRAM:
        SRAM_CE_n           :  out  std_logic;
        SRAM_OE_n           :  out  std_logic;
        SRAM_WE_n           :  out  std_logic;
        SRAM_ADDR           :  out  std_logic_vector(20 downto 1);
        SRAM_BE_n           :  out  std_logic_vector(1 downto 0);
        SRAM_DQ             :  inout  std_logic_vector(15 downto 0);
        -- NODE_SWITCH:
        NODE_SWITCH         : in std_logic_vector(7 downto 0);
        -- SWITCH:
        SW                  : in std_logic_vector(7 downto 0);
        -- LED: high active
        LEDG                : out std_logic_vector(7 downto 0);
        LEDR                : out std_logic_vector(15 downto 0);
        -- HEX0 - HEX1: low active
        HEX0                : out std_logic_vector(6 downto 0);     -- low active
        HEX1                : out std_logic_vector(6 downto 0);     -- low active
        -- BENCHMARK_OUT:
        BENCHMARK_OUT       : out std_logic_vector(7 downto 0);
        -- LCD:
        -- LCD_BLON            : out std_logic;         -- is not connected on the Altera DE2-115 board!
        LCD_ON              : out std_logic;
        LCD_DATA            : inout std_logic_vector(7 downto 0);
        LCD_EN              : out   std_logic;
        LCD_RS              : out   std_logic;
        LCD_RW              : out   std_logic);
end toplevel;

architecture rtl of toplevel is

    component cn_directIO is
        port (
            iclk_50mhz_clk                                               : in    std_logic                     := 'X';             -- clk
            ireset_50mhz_reset_n                                         : in    std_logic                     := 'X';             -- reset_n
            iclk_100mhz_clk                                              : in    std_logic                     := 'X';             -- clk
            ireset_100mhz_reset_n                                        : in    std_logic                     := 'X';             -- reset_n
            epcs_flash_controller_0_external_dclk                        : out   std_logic;                                        -- dclk
            epcs_flash_controller_0_external_sce                         : out   std_logic;                                        -- sce
            epcs_flash_controller_0_external_sdo                         : out   std_logic;                                        -- sdo
            epcs_flash_controller_0_external_data0                       : in    std_logic                     := 'X';             -- data0
            -- PHY0:
            powerlink_0_phym0_SMIClk                                     : out   std_logic;                                        -- SMIClk
            powerlink_0_phym0_SMIDat                                     : inout std_logic                     := 'X';             -- SMIDat
            powerlink_0_phym0_Rst_n                                      : out   std_logic;                                        -- Rst_n
            powerlink_0_mii0_phyMii0_TxClk                               : in    std_logic                     := 'X';             -- phyMii0_TxClk
            powerlink_0_mii0_phyMii0_TxEn                                : out   std_logic;                                        -- phyMii0_TxEn
            powerlink_0_mii0_phyMii0_TxEr                                : out   std_logic;                                        -- phyMii0_TxEr
            powerlink_0_mii0_phyMii0_TxDat                               : out   std_logic_vector(3 downto 0);                     -- phyMii0_TxDat
            powerlink_0_mii0_phyMii0_RxClk                               : in    std_logic                     := 'X';             -- phyMii0_RxClk
            powerlink_0_mii0_phyMii0_RxDv                                : in    std_logic                     := 'X';             -- phyMii0_RxDv
            powerlink_0_mii0_phyMii0_RxEr                                : in    std_logic                     := 'X';             -- phyMii0_RxEr
            powerlink_0_mii0_phyMii0_RxDat                               : in    std_logic_vector(3 downto 0)  := (others => 'X'); -- phyMii0_RxDat
            -- PHY1:
            powerlink_0_phym1_SMIClk                                     : out   std_logic;                                        -- SMIClk
            powerlink_0_phym1_SMIDat                                     : inout std_logic                     := 'X';             -- SMIDat
            powerlink_0_phym1_Rst_n                                      : out   std_logic;                                        -- Rst_n
            powerlink_0_mii0_phyMii1_RxEr                                : in    std_logic                     := 'X';             -- phyMii1_RxEr
            powerlink_0_mii1_TxClk                                       : in    std_logic                     := 'X';             -- TxClk
            powerlink_0_mii1_TxEn                                        : out   std_logic;                                        -- TxEn
            powerlink_0_mii1_TxEr                                        : out   std_logic;                                        -- TxEr
            powerlink_0_mii1_TxDat                                       : out   std_logic_vector(3 downto 0);                     -- TxDat
            powerlink_0_mii1_RxClk                                       : in    std_logic                     := 'X';             -- RxClk
            powerlink_0_mii1_RxDv                                        : in    std_logic                     := 'X';             -- RxDv
            powerlink_0_mii1_RxDat                                       : in    std_logic_vector(3 downto 0)  := (others => 'X'); -- RxDat
            -- SMP -DirectIO
            powerlink_0_smp_pio_pconfig                                  : in    std_logic_vector(3 downto 0)  := (others => 'X'); -- pconfig
            powerlink_0_smp_pio_portInLatch                              : in    std_logic_vector(3 downto 0)  := (others => 'X'); -- portInLatch
            powerlink_0_smp_pio_portOutValid                             : out   std_logic_vector(3 downto 0);                     -- portOutValid
            powerlink_0_smp_pio_portio                                   : inout std_logic_vector(31 downto 0) := (others => 'X'); -- portio
            powerlink_0_smp_pio_operational                              : out   std_logic;                                        -- operational
            -- LCD
            lcd_external_data                                            : inout std_logic_vector(7 downto 0)  := (others => 'X'); -- data
            lcd_external_E                                               : out   std_logic;                                        -- E
            lcd_external_RS                                              : out   std_logic;                                        -- RS
            lcd_external_RW                                              : out   std_logic;                                        -- RW
            -- SRAM:
            tristate_bridge_0_bridge_0_out_tcm_outputenable_n_out        : out   std_logic_vector(0 downto 0);                     -- sram_0_tcm_outputenable_n_out
            tristate_bridge_0_bridge_0_out_tcm_write_n_out               : out   std_logic_vector(0 downto 0);                     -- sram_0_tcm_write_n_out
            tristate_bridge_0_bridge_0_out_tcm_data_out                  : inout std_logic_vector(15 downto 0) := (others => 'X'); -- sram_0_tcm_data_out
            tristate_bridge_0_bridge_0_out_tcm_byteenable_n_out          : out   std_logic_vector(1 downto 0);                     -- sram_0_tcm_byteenable_n_out
            tristate_bridge_0_bridge_0_out_tcm_chipselect_n_out          : out   std_logic_vector(0 downto 0);                     -- sram_0_tcm_chipselect_n_out
            tristate_bridge_0_bridge_0_out_tcm_address_out               : out   std_logic_vector(20 downto 0);                    -- sram_0_tcm_address_out
            -- BENCHMARK
            benchmark_pio_external_connection_export                     : out   std_logic_vector(7 downto 0);                     -- export
            -- NODE_SWITCH:
            node_switch_pio_external_connection_export                   : in    std_logic_vector(7 downto 0)  := (others => 'X'); -- export
            -- STATUS_LED:
            status_led_pio_external_connection_export                    : out   std_logic_vector(1 downto 0)                      -- export
        );
    end component cn_directIO;

    component altpll_inst is
        port
        (
            areset      : in STD_LOGIC  := '0';
            inclk0      : in STD_LOGIC  := '0';
            c0          : out STD_LOGIC ;
            c1          : out STD_LOGIC ;
            locked      : out STD_LOGIC
        );
    end component altpll_inst;
    component sevsegdec is
        port
        (
            d           : in STD_LOGIC_VECTOR (3 DOWNTO 0);
            seg_n       : out STD_LOGIC_VECTOR (6 DOWNTO 0);
            seg         : out STD_LOGIC_VECTOR (6 DOWNTO 0)
        );
    end component sevsegdec;

    -- CONSTANT DECLARTIONS:
    constant cPort_IO_configuration : std_logic_vector := "0001";
    constant cLeft_Index_HEX1       : natural := 31;
    constant cRight_Index_HEX1      : natural := cLeft_Index_HEX1-3;
    constant cLeft_Index_HEX0       : natural := 27;
    constant cRight_Index_HEX0      : natural := cLeft_Index_HEX0-3;

    -- SIGNAL DECLARATIONS:
    signal clk50Mhz         : std_logic;
    signal clk100Mhz        : std_logic;
    signal pllLocked        : std_logic;
    signal reset            : std_logic;
    signal sram_oe_n_vect, sram_ce_n_vect, sram_we_n_vect : std_logic_vector(0 downto 0);
    signal sram_addr_vect   : std_logic_vector(SRAM_ADDR'left downto 0);
    signal smp_pconfig      : std_logic_vector(3 downto 0);     -- '0'= output and '1'= input for the corrosponding smp_portio byte;
    signal smp_portInLatch  : std_logic_vector(3 downto 0);
    signal smp_portOutValid : std_logic_vector(3 downto 0);
    signal smp_portio       : std_logic_vector(31 downto 0) := (others => '0');
    signal smp_operational  : std_logic;
    signal status_leds      : std_logic_vector(1 downto 0);
    signal seg_HEX0, seg_HEX1 : std_logic_vector(6 downto 0);
begin
    -- INFORMATION:
        -- smp_portio(7 downto 0)   -> input  = SW(7-0)
        -- smp_portio(15 downto 8)  -> output = LEDR(7-0)
        -- smp_portio(23 downto 16) -> output = LEDR(15-8)
        -- smp_portio(31 downto 24) -> output = HEX1 and HEX0

    -- SIGNAL ASSIGNMENT:
    LCD_ON              <= '1';     -- enables lcd
    PHY0_GXCLK          <= '0';
    PHY1_GXCLK          <= '0';
    -- SRAM assignments:
    SRAM_OE_n           <= sram_oe_n_vect(0);
    SRAM_WE_n           <= sram_we_n_vect(0);
    SRAM_CE_n           <= sram_ce_n_vect(0);
    SRAM_ADDR           <= sram_addr_vect(SRAM_ADDR'left downto 1);
    -- HEX Segments:
    HEX0                <= seg_HEX0 when ( smp_pconfig(3) = '0' ) else (others => '1');
    HEX1                <= seg_HEX1 when ( smp_pconfig(3) = '0' ) else (others => '1');
    -- SMP_PORT assignments:
    smp_pconfig         <= cPort_IO_configuration;
    smp_portInLatch     <= smp_pconfig;
    smp_portio(7 downto 0)  <= SW(7 downto 0);
    -- LEDG assignments:
    LEDG(1 downto 0)    <= status_leds;
    LEDG(2)             <= smp_operational;
    LEDG(7 downto 7-3)  <= smp_pconfig;
    -- LEDR assignments:
    LEDR(7 downto 0)    <= smp_portio (15 downto 8);
    LEDR(15 downto 8)   <= smp_portio (23 downto 16);

    -- COMPONENT INSTANTIATION:
    HEX0_Decoder: sevsegdec
        port map(
            d       => smp_portio(cLeft_Index_HEX0 downto cRight_Index_HEX0),
            seg_n   => seg_HEX0,
            seg     => open
        );

    HEX1_Decoder: sevsegdec
        port map(
            d       => smp_portio(cLeft_Index_HEX1 downto cRight_Index_HEX1),
            seg_n   => seg_HEX1,
            seg     => open
         );

    pll: altpll_inst
        port map (
            areset  =>  reset,
            inclk0  =>  EXT_CLK,
            c0      =>  clk50Mhz,
            c1      =>  clk100Mhz,
            locked  =>  pllLocked
        );

    cndirectIO : component cn_directIO
        port map (
            iclk_50mhz_clk                                               => clk50Mhz,
            ireset_50mhz_reset_n                                         => pllLocked,
            iclk_100mhz_clk                                              => clk100Mhz,
            ireset_100mhz_reset_n                                        => pllLocked,
            epcs_flash_controller_0_external_dclk                        => EPCS_DCLK,
            epcs_flash_controller_0_external_sce                         => EPCS_SCE,
            epcs_flash_controller_0_external_sdo                         => EPCS_SDO,
            epcs_flash_controller_0_external_data0                       => EPCS_DATA0,
            powerlink_0_phym0_SMIClk                                     => PHY0_MDC,
            powerlink_0_phym0_SMIDat                                     => PHY0_MDIO,
            powerlink_0_phym0_Rst_n                                      => PHY0_RESET_n,
            powerlink_0_mii0_phyMii0_TxClk                               => PHY0_TXCLK,
            powerlink_0_mii0_phyMii0_TxEn                                => PHY0_TXEN,
            powerlink_0_mii0_phyMii0_TxEr                                => PHY0_TXER,
            powerlink_0_mii0_phyMii0_TxDat                               => PHY0_TXD,
            powerlink_0_mii0_phyMii0_RxClk                               => PHY0_RXCLK,
            powerlink_0_mii0_phyMii0_RxDv                                => PHY0_RXDV,
            powerlink_0_mii0_phyMii0_RxEr                                => PHY0_RXER,
            powerlink_0_mii0_phyMii0_RxDat                               => PHY0_RXD,
            powerlink_0_mii0_phyMii1_RxEr                                => PHY0_RXER,
            powerlink_0_phym1_SMIClk                                     => PHY1_MDC,
            powerlink_0_phym1_SMIDat                                     => PHY1_MDIO,
            powerlink_0_phym1_Rst_n                                      => PHY1_RESET_n,
            powerlink_0_mii1_TxClk                                       => PHY1_TXCLK,
            powerlink_0_mii1_TxEn                                        => PHY1_TXEN,
            powerlink_0_mii1_TxEr                                        => PHY1_TXER,
            powerlink_0_mii1_TxDat                                       => PHY1_TXD,
            powerlink_0_mii1_RxClk                                       => PHY1_RXCLK,
            powerlink_0_mii1_RxDv                                        => PHY1_RXDV,
            powerlink_0_mii1_RxDat                                       => PHY1_RXD,
            powerlink_0_smp_pio_pconfig                                  => smp_pconfig,
            powerlink_0_smp_pio_portInLatch                              => smp_portInLatch,
            powerlink_0_smp_pio_portOutValid                             => smp_portOutValid,
            powerlink_0_smp_pio_portio                                   => smp_portio,
            powerlink_0_smp_pio_operational                              => smp_operational,
            lcd_external_data                                            => LCD_DATA,
            lcd_external_E                                               => LCD_EN,
            lcd_external_RS                                              => LCD_RS,
            lcd_external_RW                                              => LCD_RW,
            tristate_bridge_0_bridge_0_out_tcm_outputenable_n_out        => sram_oe_n_vect,
            tristate_bridge_0_bridge_0_out_tcm_write_n_out               => sram_we_n_vect,
            tristate_bridge_0_bridge_0_out_tcm_data_out                  => SRAM_DQ,
            tristate_bridge_0_bridge_0_out_tcm_byteenable_n_out          => SRAM_BE_n,
            tristate_bridge_0_bridge_0_out_tcm_chipselect_n_out          => sram_ce_n_vect,
            tristate_bridge_0_bridge_0_out_tcm_address_out               => sram_addr_vect,
            benchmark_pio_external_connection_export                     => BENCHMARK_OUT,
            node_switch_pio_external_connection_export                   => NODE_SWITCH,
            status_led_pio_external_connection_export                    => status_leds
        );

end rtl;
