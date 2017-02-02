-------------------------------------------------------------------------------
--! @file toplevel.vhd
--
--! @brief Toplevel of Nios CN hostif gpio design
--
--! @details This is the toplevel of the Nios CN hostif gpio design for the
--! INK DE2-115 Evaluation Board.
--
-------------------------------------------------------------------------------
--
--    (c) B&R, 2017
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
        EXT_CLK             : in    std_logic;
        -- EPCS
        EPCS_DCLK           : out   std_logic;
        EPCS_SCE            : out   std_logic;
        EPCS_SDO            : out   std_logic;
        EPCS_DATA0          : in    std_logic;
        -- FLASH 8Mx8
        CFI_FLASH_ADDR      : out std_logic_vector(22 downto 0);
        CFI_FLASH_DATA      : inout std_logic_vector(7 downto 0);
        CFI_FLASH_WE_n      : out std_logic;
        CFI_FLASH_CE_n      : out std_logic;
        CFI_FLASH_OE_n      : out std_logic;
        CFI_FLASH_RESET_n   : out std_logic;
        CFI_FLASH_WP_n      : out std_logic;
        CFI_FLASH_RY        : in std_logic;
        -- 64 MBx2 SDRAM
        SDRAM_CLK           : out   std_logic;
        SDRAM_CAS_n         : out   std_logic;
        SDRAM_CKE           : out   std_logic;
        SDRAM_CS_n          : out   std_logic;
        SDRAM_RAS_n         : out   std_logic;
        SDRAM_WE_n          : out   std_logic;
        SDRAM_ADDR          : out   std_logic_vector(12 downto 0);
        SDRAM_BA            : out   std_logic_vector(1 downto 0);
        SDRAM_DQM           : out   std_logic_vector(3 downto 0);
        SDRAM_DQ            : inout std_logic_vector(31 downto 0);
        -- NODE_SWITCH
        NODE_SWITCH         : in    std_logic_vector(7 downto 0);
        -- KEY
        KEY_n               : in    std_logic_vector(3 downto 0);
        -- LED
        LEDG                : out   std_logic_vector(7 downto 0);
        LEDR                : out   std_logic_vector(15 downto 0);
        -- HEX LED
        HEX0                : out   std_logic_vector(6 downto 0);
        HEX1                : out   std_logic_vector(6 downto 0);
        HEX2                : out   std_logic_vector(6 downto 0);
        HEX3                : out   std_logic_vector(6 downto 0);
        HEX4                : out   std_logic_vector(6 downto 0);
        HEX5                : out   std_logic_vector(6 downto 0);
        HEX6                : out   std_logic_vector(6 downto 0);
        HEX7                : out   std_logic_vector(6 downto 0);
        -- LCD
        LCD_ON              : out   std_logic;
        LCD_BLON            : out   std_logic;
        LCD_DQ              : inout std_logic_vector(7 downto 0);
        LCD_E               : out   std_logic;
        LCD_RS              : out   std_logic;
        LCD_RW              : out   std_logic;
        -- HOST Interface
        HOSTIF_AD           : inout std_logic_vector(16 downto 0);
        HOSTIF_BE           : out   std_logic_vector(1 downto 0);
        HOSTIF_CS_n         : out   std_logic;
        HOSTIF_WR_n         : out   std_logic;
        HOSTIF_ALE_n        : out   std_logic;
        HOSTIF_RD_n         : out   std_logic;
        HOSTIF_ACK_n        : in    std_logic;
        HOSTIF_IRQ_n        : in    std_logic
    );
end toplevel;

architecture rtl of toplevel is

    component cnSingleHostifGpio is
        port (
            clk50_clk                                   : in    std_logic                     := 'X';             -- clk
            clk100_clk                                  : in    std_logic                     := 'X';             -- clk
            clk25_clk                                   : in    std_logic                     := 'X';             -- clk
            reset_reset_n                               : in    std_logic                     := 'X';             -- reset_n
            host_0_benchmark_pio_export                 : out   std_logic_vector(7 downto 0);                     -- export
            epcs_flash_dclk                             : out   std_logic;                                        -- dclk
            epcs_flash_sce                              : out   std_logic;                                        -- sce
            epcs_flash_sdo                              : out   std_logic;                                        -- sdo
            epcs_flash_data0                            : in    std_logic                     := 'X';             -- data0
            lcd_RS                                      : out   std_logic;                                        -- RS
            lcd_RW                                      : out   std_logic;                                        -- RW
            lcd_data                                    : inout std_logic_vector(7 downto 0)  := (others => 'X'); -- data
            lcd_E                                       : out   std_logic;                                        -- E
            sdram_0_addr                                : out   std_logic_vector(12 downto 0);                    -- addr
            sdram_0_ba                                  : out   std_logic_vector(1 downto 0);                     -- ba
            sdram_0_cas_n                               : out   std_logic;                                        -- cas_n
            sdram_0_cke                                 : out   std_logic;                                        -- cke
            sdram_0_cs_n                                : out   std_logic;                                        -- cs_n
            sdram_0_dq                                  : inout std_logic_vector(31 downto 0) := (others => 'X'); -- dq
            sdram_0_dqm                                 : out   std_logic_vector(3 downto 0);                     -- dqm
            sdram_0_ras_n                               : out   std_logic;                                        -- ras_n
            sdram_0_we_n                                : out   std_logic;                                        -- we_n
            node_switch_pio_export                      : in    std_logic_vector(7 downto 0)  := (others => 'X'); -- export
            sync_irq_irq                                : in    std_logic;
            prl0_oPrlMst_cs                             : out   std_logic;                                        -- oPrlMst_cs
            prl0_iPrlMst_ad_i                           : in    std_logic_vector(16 downto 0) := (others => 'X'); -- iPrlMst_ad_i
            prl0_oPrlMst_ad_o                           : out   std_logic_vector(16 downto 0);                    -- oPrlMst_ad_o
            prl0_oPrlMst_ad_oen                         : out   std_logic;                                        -- oPrlMst_ad_oen
            prl0_oPrlMst_be                             : out   std_logic_vector(1 downto 0);                     -- oPrlMst_be
            prl0_oPrlMst_ale                            : out   std_logic;                                        -- oPrlMst_ale
            prl0_oPrlMst_wr                             : out   std_logic;                                        -- oPrlMst_wr
            prl0_oPrlMst_rd                             : out   std_logic;                                        -- oPrlMst_rd
            prl0_iPrlMst_ack                            : in    std_logic                     := 'X';             -- iPrlMst_ack
            app_pio_in_port                             : in    std_logic_vector(31 downto 0) := (others => 'X'); -- in_port
            app_pio_out_port                            : out   std_logic_vector(31 downto 0);                    -- out_port
            tristate_cfi_flash_0_tcm_address_out        : out   std_logic_vector(22 downto 0);                    -- tcm_address_out
            tristate_cfi_flash_0_tcm_read_n_out         : out   std_logic;                                        -- tcm_read_n_out
            tristate_cfi_flash_0_tcm_write_n_out        : out   std_logic;                                        -- tcm_write_n_out
            tristate_cfi_flash_0_tcm_data_out           : inout std_logic_vector(7 downto 0)  := (others => 'X'); -- tcm_data_out
            tristate_cfi_flash_0_tcm_chipselect_n_out   : out  std_logic                                          -- tcm_chipselect_n_out
        );
    end component cnSingleHostifGpio;

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

    signal hostifCs     : std_logic;
    signal hostifWr     : std_logic;
    signal hostifRd     : std_logic;
    signal hostifAle    : std_logic;
    signal hostifAck    : std_logic;
    signal hostifAd_i   : std_logic_vector(HOSTIF_AD'range);
    signal hostifAd_o   : std_logic_vector(HOSTIF_AD'range);
    signal hostifAd_oen : std_logic;

    signal hostifIrq    : std_logic;

    type tSevenSegArray is array (natural range <>) of std_logic_vector(6 downto 0);
    constant cNumberOfHex   : natural := 8;
    signal hex              : std_logic_vector(cNumberOfHex*4-1 downto 0);
    signal sevenSegArray    : tSevenSegArray(cNumberOfHex-1 downto 0);

    signal app_input        : std_logic_vector(31 downto 0);
    signal app_output       : std_logic_vector(31 downto 0);
begin
    LCD_ON      <= '1';
    LCD_BLON    <= '1';

    SDRAM_CLK   <= clk100_p;

    HOSTIF_CS_n     <= not hostifCs;
    HOSTIF_WR_n     <= not hostifWr;
    HOSTIF_RD_n     <= not hostifRd;
    HOSTIF_ALE_n    <= not hostifAle;
    hostifAck       <= not HOSTIF_ACK_n;

    -- TRISTATE Buffer for AD bus
    HOSTIF_AD <= hostifAd_o when hostifAd_oen = '1' else (others => 'Z');
    hostifAd_i <= HOSTIF_AD;

    hostifIrq       <= not HOSTIF_IRQ_n;

    CFI_FLASH_RESET_n   <= cnInactivated;
    CFI_FLASH_WP_n      <= cnInactivated;
    ---------------------------------------------------------------------------
    -- Green LED assignments
    LEDG        <= (others => '0'); -- Reserved
    ---------------------------------------------------------------------------

    ---------------------------------------------------------------------------
    -- Red LED assignments
    LEDR        <= (others => '0'); -- Reserved
    ---------------------------------------------------------------------------

    ---------------------------------------------------------------------------
    -- Application Input and Output assignments

    -- Input: Map KEY nibble to Application Input
    app_input   <= x"0000000" & not KEY_n;

    -- Output: Map Application Output to HEX LEDs
    hex         <= app_output;
    ---------------------------------------------------------------------------

    inst : component cnSingleHostifGpio
        port map (
            clk25_clk                                   => clk25,
            clk50_clk                                   => clk50,
            clk100_clk                                  => clk100,
            reset_reset_n                               => pllLocked,

            node_switch_pio_export                      => NODE_SWITCH,

            host_0_benchmark_pio_export                 => open,

            epcs_flash_dclk                             => EPCS_DCLK,
            epcs_flash_sce                              => EPCS_SCE,
            epcs_flash_sdo                              => EPCS_SDO,
            epcs_flash_data0                            => EPCS_DATA0,

            sdram_0_addr                                => SDRAM_ADDR,
            sdram_0_ba                                  => SDRAM_BA,
            sdram_0_cas_n                               => SDRAM_CAS_n,
            sdram_0_cke                                 => SDRAM_CKE,
            sdram_0_cs_n                                => SDRAM_CS_n,
            sdram_0_dq                                  => SDRAM_DQ,
            sdram_0_dqm                                 => SDRAM_DQM,
            sdram_0_ras_n                               => SDRAM_RAS_n,
            sdram_0_we_n                                => SDRAM_WE_n,

            lcd_data                                    => LCD_DQ,
            lcd_E                                       => LCD_E,
            lcd_RS                                      => LCD_RS,
            lcd_RW                                      => LCD_RW,

            prl0_oPrlMst_cs                             => hostifCs,
            prl0_iPrlMst_ad_i                           => hostifAd_i,
            prl0_oPrlMst_ad_o                           => hostifAd_o,
            prl0_oPrlMst_ad_oen                         => hostifAd_oen,
            prl0_oPrlMst_be                             => HOSTIF_BE,
            prl0_oPrlMst_ale                            => hostifAle,
            prl0_oPrlMst_wr                             => hostifWr,
            prl0_oPrlMst_rd                             => hostifRd,
            prl0_iPrlMst_ack                            => hostifAck,

            sync_irq_irq                                => hostifIrq,

            app_pio_in_port                             => app_input,
            app_pio_out_port                            => app_output,

            tristate_cfi_flash_0_tcm_address_out        => CFI_FLASH_ADDR,
            tristate_cfi_flash_0_tcm_read_n_out         => CFI_FLASH_OE_n,
            tristate_cfi_flash_0_tcm_write_n_out        => CFI_FLASH_WE_n,
            tristate_cfi_flash_0_tcm_data_out           => CFI_FLASH_DATA,
            tristate_cfi_flash_0_tcm_chipselect_n_out   => CFI_FLASH_CE_n
        );

    -- Pll Instance
    pllInst : pll
        port map (
            inclk0  => EXT_CLK,
            c0      => clk50,
            c1      => clk100,
            c2      => clk25,
            c3      => clk100_p,
            locked  => pllLocked
        );

    -- bcd to 7 segment
    genBcdTo7Seg : for i in cNumberOfHex-1 downto 0 generate
        signal tmpHex : std_logic_vector(3 downto 0);
        signal tmpSev : std_logic_vector(6 downto 0);
    begin
        tmpHex <= hex((i+1)*4-1 downto i*4);
        sevenSegArray(i) <= tmpSev;

        bcdTo7Seg0 : entity libcommon.bcd2led
            port map (
                iBcdVal => tmpHex,
                oLed    => open,
                onLed   => tmpSev
            );
    end generate genBcdTo7Seg;

    -- assign outports to array
    HEX0 <= sevenSegArray(0);
    HEX1 <= sevenSegArray(1);
    HEX2 <= sevenSegArray(2);
    HEX3 <= sevenSegArray(3);
    HEX4 <= sevenSegArray(4);
    HEX5 <= sevenSegArray(5);
    HEX6 <= sevenSegArray(6);
    HEX7 <= sevenSegArray(7);
end rtl;
