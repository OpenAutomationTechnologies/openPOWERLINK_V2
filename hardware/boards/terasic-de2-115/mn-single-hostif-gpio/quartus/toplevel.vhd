-------------------------------------------------------------------------------
--! @file toplevel.vhd
--
--! @brief Toplevel of Nios MN design Host part
--
--! @details This is the toplevel of the Nios MN FPGA Host design for the
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

entity toplevel is
    port (
        -- 50 MHZ CLK IN
        EXT_CLK         : in  std_logic;
        -- EPCS
        EPCS_DCLK       : out  std_logic;
        EPCS_SCE        : out  std_logic;
        EPCS_SDO        : out  std_logic;
        EPCS_DATA0      : in  std_logic;
        -- 64 MBx2 SDRAM
        SDRAM_CLK       : out  std_logic;
        SDRAM_CAS_n     : out  std_logic;
        SDRAM_CKE       : out  std_logic;
        SDRAM_CS_n      : out  std_logic;
        SDRAM_RAS_n     : out  std_logic;
        SDRAM_WE_n      : out  std_logic;
        SDRAM_ADDR      : out  std_logic_vector(12 downto 0);
        SDRAM_BA        : out  std_logic_vector(1 downto 0);
        SDRAM_DQM       : out  std_logic_vector(3 downto 0);
        SDRAM_DQ        : inout  std_logic_vector(31 downto 0);
        -- LED green
        LEDG            : out  std_logic_vector(1 downto 0);
        -- LCD
        LCD_ON          : out std_logic;
        LCD_BLON        : out std_logic;
        LCD_DQ          : inout std_logic_vector(7 downto 0);
        LCD_E           : out std_logic;
        LCD_RS          : out std_logic;
        LCD_RW          : out std_logic;
        -- HOST Interface
        HOSTIF_AD       : inout std_logic_vector(15 downto 0);
        HOSTIF_BE       : out std_logic_vector(1 downto 0);
        HOSTIF_CS_n     : out std_logic;
        HOSTIF_WR_n     : out std_logic;
        HOSTIF_ALE_n    : out std_logic;
        HOSTIF_RD_n     : out std_logic;
        HOSTIF_ACK_n    : in std_logic;
        HOSTIF_IRQ_n    : in std_logic
    );
end toplevel;

architecture rtl of toplevel is

    component mnSingleHostifGpio is
        port (
            clk25_clk                           : in    std_logic;
            clk50_clk                           : in    std_logic                     := 'X';
            clk100_clk                          : in    std_logic;
            reset_reset_n                       : in    std_logic                     := 'X';

            host_0_benchmark_pio_export         : out   std_logic_vector(7 downto 0);
            status_led_pio_export               : out   std_logic_vector(1 downto 0);
            epcs_flash_dclk                     : out   std_logic;
            epcs_flash_sce                      : out   std_logic;
            epcs_flash_sdo                      : out   std_logic;
            epcs_flash_data0                    : in    std_logic                     := 'X';
            host_0_sdram_0_addr                 : out   std_logic_vector(12 downto 0);
            host_0_sdram_0_ba                   : out   std_logic_vector(1 downto 0);
            host_0_sdram_0_cas_n                : out   std_logic;
            host_0_sdram_0_cke                  : out   std_logic;
            host_0_sdram_0_cs_n                 : out   std_logic;
            host_0_sdram_0_dq                   : inout std_logic_vector(31 downto 0) := (others => 'X');
            host_0_sdram_0_dqm                  : out   std_logic_vector(3 downto 0);
            host_0_sdram_0_ras_n                : out   std_logic;
            host_0_sdram_0_we_n                 : out   std_logic;
            multiplexedadbus_0_cs               : out   std_logic;
            multiplexedadbus_0_ad               : inout std_logic_vector(15 downto 0) := (others => 'X');
            multiplexedadbus_0_be               : out   std_logic_vector(1 downto 0);
            multiplexedadbus_0_ale              : out   std_logic;
            multiplexedadbus_0_wr               : out   std_logic;
            multiplexedadbus_0_rd               : out   std_logic;
            multiplexedadbus_0_ack              : in    std_logic                     := 'X';
            host_0_irq_irq                      : in    std_logic                     := 'X';
            lcd_data                            : inout std_logic_vector(7 downto 0)  := (others => 'X');
            lcd_E                               : out   std_logic;
            lcd_RS                              : out   std_logic;
            lcd_RW                              : out   std_logic
        );
    end component mnSingleHostifGpio;

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
    signal hostifIrq    : std_logic;

begin

    LCD_ON      <= '1';
    LCD_BLON    <= '1';

    SDRAM_CLK <= clk100_p;

    HOSTIF_CS_n     <= not hostifCs;
    HOSTIF_WR_n     <= not hostifWr;
    HOSTIF_RD_n     <= not hostifRd;
    HOSTIF_ALE_n    <= not hostifAle;
    hostifAck       <= not HOSTIF_ACK_n;
    hostifIrq       <= not HOSTIF_IRQ_n;

    inst : component mnSingleHostifGpio
        port map (
            clk25_clk                       => clk25,
            clk50_clk                       => clk50,
            clk100_clk                      => clk100,
            reset_reset_n                   => pllLocked,

            status_led_pio_export           => LEDG,

            host_0_benchmark_pio_export     => open,

            epcs_flash_dclk                 => EPCS_DCLK,
            epcs_flash_sce                  => EPCS_SCE,
            epcs_flash_sdo                  => EPCS_SDO,
            epcs_flash_data0                => EPCS_DATA0,

            host_0_sdram_0_addr             => SDRAM_ADDR,
            host_0_sdram_0_ba               => SDRAM_BA,
            host_0_sdram_0_cas_n            => SDRAM_CAS_n,
            host_0_sdram_0_cke              => SDRAM_CKE,
            host_0_sdram_0_cs_n             => SDRAM_CS_n,
            host_0_sdram_0_dq               => SDRAM_DQ,
            host_0_sdram_0_dqm              => SDRAM_DQM,
            host_0_sdram_0_ras_n            => SDRAM_RAS_n,
            host_0_sdram_0_we_n             => SDRAM_WE_n,

            multiplexedadbus_0_cs           => hostifCs,
            multiplexedadbus_0_ad           => HOSTIF_AD,
            multiplexedadbus_0_be           => HOSTIF_BE,
            multiplexedadbus_0_ale          => hostifAle,
            multiplexedadbus_0_wr           => hostifWr,
            multiplexedadbus_0_rd           => hostifRd,
            multiplexedadbus_0_ack          => hostifAck,
            host_0_irq_irq                  => hostifIrq,

            lcd_data                        => LCD_DQ,
            lcd_E                           => LCD_E,
            lcd_RS                          => LCD_RS,
            lcd_RW                          => LCD_RW
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

end rtl;
