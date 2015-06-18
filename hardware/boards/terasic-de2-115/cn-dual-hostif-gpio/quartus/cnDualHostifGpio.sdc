# ------------------------------------------------------------------------------
# SDC for TERASIC DE2-115 CN dual GPIO
# ------------------------------------------------------------------------------

source ../../common/timing/sram.sdc
source ../../common/timing/sdram.sdc
source ../../common/timing/jtag.sdc

# ------------------------------------------------------------------------------
# Clock definitions
# -> Define clocks in design (depends on PLL settings!)
#    (under "Compilation Report" - "TimeQuest Timing Analyzer" - "Clocks")
# -> Derive PLL clocks

set ext_clk     EXT_CLK
set clk50       pllInst|altpll_component|auto_generated|pll1|clk[0]
set clk100      pllInst|altpll_component|auto_generated|pll1|clk[1]
set clk25       pllInst|altpll_component|auto_generated|pll1|clk[2]
set clk100_p    pllInst|altpll_component|auto_generated|pll1|clk[3]

derive_pll_clocks -create_base_clocks
derive_clock_uncertainty

# ------------------------------------------------------------------------------
# SRAM definitions

timing_sram $clk100

# ------------------------------------------------------------------------------
# SDRAM definitions

timing_sdram $clk100_p $clk100

# ------------------------------------------------------------------------------
# JTAG definitions

timing_jtag

# ------------------------------------------------------------------------------
# EPCS
# -> Cut path
set_false_path -from [get_registers *]      -to [get_ports EPCS_DCLK]
set_false_path -from [get_registers *]      -to [get_ports EPCS_SCE]
set_false_path -from [get_registers *]      -to [get_ports EPCS_SDO]
set_false_path -from [get_ports EPCS_DATA0] -to [get_registers *]

# ------------------------------------------------------------------------------
# Other IOs
set_false_path -from [get_registers *]      -to [get_ports LEDG[*]]
set_false_path -from [get_registers *]      -to [get_ports LCD_*]
set_false_path -from [get_registers *]      -to [get_ports LCD_DQ[*]]
set_false_path -from [get_registers *]      -to [get_ports BENCHMARK[*]]
set_false_path -from [get_registers *]      -to [get_ports BENCHMARK_AP[*]]
set_false_path -from [get_ports LCD_DQ[*]]  -to [get_registers *]
set_false_path -from [get_ports KEY_n[*]]   -to [get_registers *]
