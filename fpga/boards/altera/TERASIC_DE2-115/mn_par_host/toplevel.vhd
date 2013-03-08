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

library work;

entity toplevel is
    port (
        -- 50 MHZ CLK IN
        EXT_CLK :  in  std_logic;
        -- EPCS
        EPCS_DCLK :  out  std_logic;
        EPCS_SCE :  out  std_logic;
        EPCS_SDO :  out  std_logic;
        EPCS_DATA0 :  in  std_logic;
        -- 64 MBx2 SDRAM
        SDRAM_CLK :  out  std_logic;
        SDRAM_CAS_n :  out  std_logic;
        SDRAM_CKE :  out  std_logic;
        SDRAM_CS_n :  out  std_logic;
        SDRAM_RAS_n :  out  std_logic;
        SDRAM_WE_n :  out  std_logic;
        SDRAM_ADDR :  out  std_logic_vector(12 downto 0);
        SDRAM_BA :  out  std_logic_vector(1 downto 0);
        SDRAM_DQM :  out  std_logic_vector(3 downto 0);
        SDRAM_DQ :  inout  std_logic_vector(31 downto 0);
        -- KEY
        KEY :  in  std_logic_vector(3 downto 0);
        -- SW
        SW :  in  std_logic_vector(7 downto 0);
        -- LED green
        LEDG :  out  std_logic_vector(1 downto 0);
        -- LED red
        LEDR :  out  std_logic_vector(3 downto 0);
        -- LCD
        LCD_EN :  out  std_logic;
        LCD_RS :  out  std_logic;
        LCD_ON :  out  std_logic;
        LCD_RW :  out  std_logic;
        LCD_DATA :  inout  std_logic_vector(7 downto 0);
        -- HOST Interface
        HOSTIF_AD : inout std_logic_vector(15 downto 0);
        HOSTIF_BE : out std_logic_vector(1 downto 0);
        HOSTIF_CS_n : out std_logic;
        HOSTIF_WR_n : out std_logic;
        HOSTIF_ALE_n : out std_logic;
        HOSTIF_RD_n : out std_logic;
        HOSTIF_ACK_n : in std_logic;
        HOSTIF_IRQ_n : in std_logic
    );
end toplevel;

architecture rtl of toplevel is

    component top_mn is
        port (
            clk_clk                                               : in    std_logic                     := 'X';             -- clk
            reset_reset_n                                         : in    std_logic                     := 'X';             -- reset_n
            clk100sdram_clk                                       : out   std_logic;                                        -- clk
            altpll_0_areset_conduit_export                        : in    std_logic                     := 'X';             -- export
            altpll_0_locked_conduit_export                        : out   std_logic;                                        -- export
            altpll_0_phasedone_conduit_export                     : out   std_logic;                                        -- export
            host_0_benchmark_pio_external_connection_export       : out   std_logic_vector(7 downto 0);                     -- export
            host_0_sw_pio_external_connection_export              : in    std_logic_vector(7 downto 0)  := (others => 'X'); -- export
            host_0_key_pio_external_connection_export             : in    std_logic_vector(3 downto 0)  := (others => 'X'); -- export
            host_0_led_pio_external_connection_export             : out   std_logic_vector(3 downto 0);                     -- export
            host_0_led_status_error_pio_external_connection_export: out   std_logic_vector(1 downto 0);                     -- export
            host_0_lcd_0_external_data                            : inout std_logic_vector(7 downto 0)  := (others => 'X'); -- data
            host_0_lcd_0_external_E                               : out   std_logic;                                        -- E
            host_0_lcd_0_external_RS                              : out   std_logic;                                        -- RS
            host_0_lcd_0_external_RW                              : out   std_logic;                                        -- RW
            host_0_epcs_flash_controller_0_external_dclk          : out   std_logic;                                        -- dclk
            host_0_epcs_flash_controller_0_external_sce           : out   std_logic;                                        -- sce
            host_0_epcs_flash_controller_0_external_sdo           : out   std_logic;                                        -- sdo
            host_0_epcs_flash_controller_0_external_data0         : in    std_logic                     := 'X';             -- data0
            host_0_sdram_0_wire_addr                              : out   std_logic_vector(12 downto 0);                    -- addr
            host_0_sdram_0_wire_ba                                : out   std_logic_vector(1 downto 0);                     -- ba
            host_0_sdram_0_wire_cas_n                             : out   std_logic;                                        -- cas_n
            host_0_sdram_0_wire_cke                               : out   std_logic;                                        -- cke
            host_0_sdram_0_wire_cs_n                              : out   std_logic;                                        -- cs_n
            host_0_sdram_0_wire_dq                                : inout std_logic_vector(31 downto 0) := (others => 'X'); -- dq
            host_0_sdram_0_wire_dqm                               : out   std_logic_vector(3 downto 0);                     -- dqm
            host_0_sdram_0_wire_ras_n                             : out   std_logic;                                        -- ras_n
            host_0_sdram_0_wire_we_n                              : out   std_logic;                                        -- we_n
            host_0_irq_bridge_0_receiver_irq_irq                  : in    std_logic                     := 'X';             -- irq
            adbus_0_cs                                            : out   std_logic;                                        -- cs
            adbus_0_ad                                            : inout std_logic_vector(15 downto 0) := (others => 'X'); -- ad
            adbus_0_be                                            : out   std_logic_vector(1 downto 0);                     -- be
            adbus_0_ale                                           : out   std_logic;                                        -- ale
            adbus_0_wr                                            : out   std_logic;                                        -- wr
            adbus_0_rd                                            : out   std_logic;                                        -- rd
            adbus_0_ack                                           : in    std_logic                     := 'X'              -- ack
        );
    end component top_mn;

    signal pllLocked : std_logic;

    signal hostifCs : std_logic;
    signal hostifWr : std_logic;
    signal hostifRd : std_logic;
    signal hostifAle : std_logic;
    signal hostifAck : std_logic;

begin

    LCD_ON <= '1';

    inst : component top_mn
        port map (
            clk_clk                                                 => EXT_CLK,
            altpll_0_areset_conduit_export                          => '0',
            altpll_0_locked_conduit_export                          => pllLocked,
            altpll_0_phasedone_conduit_export                       => open,
            reset_reset_n                                           => pllLocked,

            host_0_led_status_error_pio_external_connection_export  => LEDG,

            host_0_benchmark_pio_external_connection_export         => open,

            host_0_key_pio_external_connection_export               => KEY,

            host_0_sw_pio_external_connection_export                => SW,

            host_0_led_pio_external_connection_export               => LEDR,

            host_0_lcd_0_external_data                              => LCD_DATA,
            host_0_lcd_0_external_E                                 => LCD_EN,
            host_0_lcd_0_external_RS                                => LCD_RS,
            host_0_lcd_0_external_RW                                => LCD_RW,

            host_0_epcs_flash_controller_0_external_dclk            => EPCS_DCLK,
            host_0_epcs_flash_controller_0_external_sce             => EPCS_SCE,
            host_0_epcs_flash_controller_0_external_sdo             => EPCS_SDO,
            host_0_epcs_flash_controller_0_external_data0           => EPCS_DATA0,

            clk100sdram_clk                                         => SDRAM_CLK,
            host_0_sdram_0_wire_addr                                => SDRAM_ADDR,
            host_0_sdram_0_wire_ba                                  => SDRAM_BA,
            host_0_sdram_0_wire_cas_n                               => SDRAM_CAS_n,
            host_0_sdram_0_wire_cke                                 => SDRAM_CKE,
            host_0_sdram_0_wire_cs_n                                => SDRAM_CS_n,
            host_0_sdram_0_wire_dq                                  => SDRAM_DQ,
            host_0_sdram_0_wire_dqm                                 => SDRAM_DQM,
            host_0_sdram_0_wire_ras_n                               => SDRAM_RAS_n,
            host_0_sdram_0_wire_we_n                                => SDRAM_WE_n,

            host_0_irq_bridge_0_receiver_irq_irq                    => not HOSTIF_IRQ_n,

            adbus_0_cs                                              => hostifCs,
            adbus_0_ad                                              => HOSTIF_AD,
            adbus_0_be                                              => HOSTIF_BE,
            adbus_0_ale                                             => hostifAle,
            adbus_0_wr                                              => hostifWr,
            adbus_0_rd                                              => hostifRd,
            adbus_0_ack                                             => hostifAck
        );

        HOSTIF_CS_n <= not hostifCs;
        HOSTIF_WR_n <= not hostifWr;
        HOSTIF_RD_n <= not hostifRd;
        HOSTIF_ALE_n <= not hostifAle;
        hostifAck <= not HOSTIF_ACK_n;

end rtl;
