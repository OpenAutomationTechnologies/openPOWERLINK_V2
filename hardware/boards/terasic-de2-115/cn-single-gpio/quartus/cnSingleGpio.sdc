# ------------------------------------------------------------------------------
# SDC for TERASIC DE2-115 CN single GPIO
# ------------------------------------------------------------------------------

source ../../common/timing/sram.sdc
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

derive_pll_clocks -create_base_clocks
derive_clock_uncertainty

# ------------------------------------------------------------------------------
# SRAM definitions

timing_sram $clk100

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
# Node switch
# -> Cut path
set_false_path -from [get_ports NODE_SWITCH[*]] -to [get_registers *]

# ------------------------------------------------------------------------------
# Other IOs
# -> Cut path
set_false_path -from [get_registers *]      -to [get_ports LEDG[*]]
set_false_path -from [get_registers *]      -to [get_ports LEDR[*]]
set_false_path -from [get_registers *]      -to [get_ports HEX?[*]]
set_false_path -from [get_registers *]      -to [get_ports LCD_*]
set_false_path -from [get_registers *]      -to [get_ports LCD_DQ[*]]
set_false_path -from [get_registers *]      -to [get_ports BENCHMARK[*]]
set_false_path -from [get_ports KEY[*]]     -to [get_registers *]
set_false_path -from [get_ports LCD_DQ[*]]  -to [get_registers *]
