# ------------------------------------------------------------------------------
# SDC for ALTERA C5 SOC BOARD
# ------------------------------------------------------------------------------

# ------------------------------------------------------------------------------
# Clock definitions
# -> Define clocks in design (depends on PLL settings!)
#    (under "Compilation Report" - "TimeQuest Timing Analyzer" - "Clocks")
# -> Derive PLL clocks
set ext_clk     fpga_clk_50
set clk50       pllInst|pll_inst|altera_pll_i|general[0].gpll~PLL_OUTPUT_COUNTER|divclk
set clk100      pllInst|pll_inst|altera_pll_i|general[1].gpll~PLL_OUTPUT_COUNTER|divclk
set afi_wr_clk  soc_inst|ddr3_emif_0|pll0|pll_afi_clk
set wr_clk      soc_inst|ddr3_emif_0|pll0|pll_write_clk
set ddr_clks    soc_inst|ddr3_emif_0|*

derive_pll_clocks -create_base_clocks
derive_clock_uncertainty

# ------------------------------------------------------------------------------
# JTAG definitions
# -> For enhancing USB Blaster II to be reliable, 25MHz
create_clock -name {altera_reserved_tck} -period 40 {altera_reserved_tck}
set_input_delay -clock altera_reserved_tck -clock_fall 3 [get_ports altera_reserved_tdi]
set_input_delay -clock altera_reserved_tck -clock_fall 3 [get_ports altera_reserved_tms]
set_output_delay -clock altera_reserved_tck 3 [get_ports altera_reserved_tdo]

# ------------------------------------------------------------------------------
# FPGA DDR
# -> Create clock group for FPGA ipcores and FPGA DDR memory controller
set_clock_groups -exclusive -group ${ext_clk} ${clk50} ${clk100} \
                            -group ${ddr_clks}

# ------------------------------------------------------------------------------
# FPGA IO ports
# -> Cut path
set_false_path -from [get_ports {fpga_button_pio[0]}] -to *
set_false_path -from [get_ports {fpga_button_pio[1]}] -to *
set_false_path -from [get_ports {fpga_dipsw_pio[0]}] -to *
set_false_path -from [get_ports {fpga_dipsw_pio[1]}] -to *
set_false_path -from [get_ports {fpga_dipsw_pio[2]}] -to *
set_false_path -from [get_ports {fpga_dipsw_pio[3]}] -to *
set_false_path -from * -to [get_ports {fpga_led_pio[0]}]
set_false_path -from * -to [get_ports {fpga_led_pio[1]}]
set_false_path -from * -to [get_ports {fpga_led_pio[2]}]
set_false_path -from * -to [get_ports {fpga_led_pio[3]}]

# ------------------------------------------------------------------------------
# HPS peripherals
# -> Cut path (ignore due to hard ipcores)
set_false_path -from * -to [get_ports {hps_emac1_TX_CLK}]
set_false_path -from * -to [get_ports {hps_emac1_TXD0}]
set_false_path -from * -to [get_ports {hps_emac1_TXD1}]
set_false_path -from * -to [get_ports {hps_emac1_TXD2}]
set_false_path -from * -to [get_ports {hps_emac1_TXD3}]
set_false_path -from * -to [get_ports {hps_emac1_MDC}]
set_false_path -from * -to [get_ports {hps_emac1_TX_CTL}]
set_false_path -from * -to [get_ports {hps_qspi_SS0}]
set_false_path -from * -to [get_ports {hps_qspi_CLK}]
set_false_path -from * -to [get_ports {hps_sdio_CLK}]
set_false_path -from * -to [get_ports {hps_usb1_STP}]
set_false_path -from * -to [get_ports {hps_spim0_CLK}]
set_false_path -from * -to [get_ports {hps_spim0_MOSI}]
set_false_path -from * -to [get_ports {hps_spim0_SS0}]
set_false_path -from * -to [get_ports {hps_uart0_TX}]
set_false_path -from * -to [get_ports {hps_can0_TX}]
set_false_path -from * -to [get_ports {hps_trace_CLK}]
set_false_path -from * -to [get_ports {hps_trace_D0}]
set_false_path -from * -to [get_ports {hps_trace_D1}]
set_false_path -from * -to [get_ports {hps_trace_D2}]
set_false_path -from * -to [get_ports {hps_trace_D3}]
set_false_path -from * -to [get_ports {hps_trace_D4}]
set_false_path -from * -to [get_ports {hps_trace_D5}]
set_false_path -from * -to [get_ports {hps_trace_D6}]
set_false_path -from * -to [get_ports {hps_trace_D7}]
set_false_path -from * -to [get_ports {hps_emac1_MDIO}]
set_false_path -from * -to [get_ports {hps_qspi_IO0}]
set_false_path -from * -to [get_ports {hps_qspi_IO1}]
set_false_path -from * -to [get_ports {hps_qspi_IO2}]
set_false_path -from * -to [get_ports {hps_qspi_IO3}]
set_false_path -from * -to [get_ports {hps_sdio_CMD}]
set_false_path -from * -to [get_ports {hps_sdio_D0}]
set_false_path -from * -to [get_ports {hps_sdio_D1}]
set_false_path -from * -to [get_ports {hps_sdio_D2}]
set_false_path -from * -to [get_ports {hps_sdio_D3}]
set_false_path -from * -to [get_ports {hps_usb1_D0}]
set_false_path -from * -to [get_ports {hps_usb1_D1}]
set_false_path -from * -to [get_ports {hps_usb1_D2}]
set_false_path -from * -to [get_ports {hps_usb1_D3}]
set_false_path -from * -to [get_ports {hps_usb1_D4}]
set_false_path -from * -to [get_ports {hps_usb1_D5}]
set_false_path -from * -to [get_ports {hps_usb1_D6}]
set_false_path -from * -to [get_ports {hps_usb1_D7}]
set_false_path -from * -to [get_ports {hps_i2c0_SDA}]
set_false_path -from * -to [get_ports {hps_i2c0_SCL}]
set_false_path -from * -to [get_ports {hps_gpio_GPIO09}]
set_false_path -from * -to [get_ports {hps_gpio_GPIO35}]
set_false_path -from * -to [get_ports {hps_gpio_GPIO41}]
set_false_path -from * -to [get_ports {hps_gpio_GPIO42}]
set_false_path -from * -to [get_ports {hps_gpio_GPIO43}]
set_false_path -from * -to [get_ports {hps_gpio_GPIO44}]
set_false_path -from [get_ports {hps_emac1_MDIO}] -to *
set_false_path -from [get_ports {hps_qspi_IO0}] -to *
set_false_path -from [get_ports {hps_qspi_IO1}] -to *
set_false_path -from [get_ports {hps_qspi_IO2}] -to *
set_false_path -from [get_ports {hps_qspi_IO3}] -to *
set_false_path -from [get_ports {hps_sdio_CMD}] -to *
set_false_path -from [get_ports {hps_sdio_D0}] -to *
set_false_path -from [get_ports {hps_sdio_D1}] -to *
set_false_path -from [get_ports {hps_sdio_D2}] -to *
set_false_path -from [get_ports {hps_sdio_D3}] -to *
set_false_path -from [get_ports {hps_usb1_D0}] -to *
set_false_path -from [get_ports {hps_usb1_D1}] -to *
set_false_path -from [get_ports {hps_usb1_D2}] -to *
set_false_path -from [get_ports {hps_usb1_D3}] -to *
set_false_path -from [get_ports {hps_usb1_D4}] -to *
set_false_path -from [get_ports {hps_usb1_D5}] -to *
set_false_path -from [get_ports {hps_usb1_D6}] -to *
set_false_path -from [get_ports {hps_usb1_D7}] -to *
set_false_path -from [get_ports {hps_i2c0_SDA}] -to *
set_false_path -from [get_ports {hps_i2c0_SCL}] -to *
set_false_path -from [get_ports {hps_gpio_GPIO09}] -to *
set_false_path -from [get_ports {hps_gpio_GPIO35}] -to *
set_false_path -from [get_ports {hps_gpio_GPIO41}] -to *
set_false_path -from [get_ports {hps_gpio_GPIO42}] -to *
set_false_path -from [get_ports {hps_gpio_GPIO43}] -to *
set_false_path -from [get_ports {hps_gpio_GPIO44}] -to *
set_false_path -from [get_ports {hps_emac1_RX_CTL}] -to *
set_false_path -from [get_ports {hps_emac1_RX_CLK}] -to *
set_false_path -from [get_ports {hps_emac1_RXD0}] -to *
set_false_path -from [get_ports {hps_emac1_RXD1}] -to *
set_false_path -from [get_ports {hps_emac1_RXD2}] -to *
set_false_path -from [get_ports {hps_emac1_RXD3}] -to *
set_false_path -from [get_ports {hps_usb1_CLK}] -to *
set_false_path -from [get_ports {hps_usb1_DIR}] -to *
set_false_path -from [get_ports {hps_usb1_NXT}] -to *
set_false_path -from [get_ports {hps_spim0_MISO}] -to *
set_false_path -from [get_ports {hps_uart0_RX}] -to *
set_false_path -from [get_ports {hps_can0_RX}] -to *

# ------------------------------------------------------------------------------
# HPS clocks
# -> Create clocks (avoid unconstrained paths)
create_clock -period "1 MHz" [get_ports hps_i2c0_SCL]
create_clock -period "48 MHz" [get_ports hps_usb1_CLK]
