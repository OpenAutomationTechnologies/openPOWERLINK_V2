# ------------------------------------------------------------------------------
# SDC for TERASIC DE2I-150 development board
# ------------------------------------------------------------------------------

source ../../common/timing/ssram.sdc
source ../../common/timing/jtag.sdc

# ------------------------------------------------------------------------------
# Clock definitions
# -> Define clocks in design (depends on PLL settings!)
#    (under "Compilation Report" - "TimeQuest Timing Analyzer" - "Clocks")
# -> Derive PLL clocks

set ext_clk     EXT_CLK
set clk50       pllInst|altpll_component|auto_generated|pll1|clk[0]
set clk100      pllInst|altpll_component|auto_generated|pll1|clk[1]
set clk125      pllInst|altpll_component|auto_generated|pll1|clk[2]
set clk25       pllInst|altpll_component|auto_generated|pll1|clk[3]
set clkpciecore inst|pcie_subsytem|pcie_ip|pcie_internal_hip|cyclone_iii.cycloneiv_hssi_pcie_hip|coreclkout

derive_pll_clocks -create_base_clocks
derive_clock_uncertainty

# -> Ignore phy ref clock for now
set_false_path -from * -to [get_ports ENET_GTX_CLK]

# ------------------------------------------------------------------------------
# SSRAM definitions

timing_ssram $clk100

# ------------------------------------------------------------------------------
# JTAG definitions

timing_jtag

# ------------------------------------------------------------------------------
# EPCS
# -> Cut path
set_false_path -from [get_registers *]      -to [get_ports oFlash_Clk]
set_false_path -from [get_registers *]      -to [get_ports oFlash_nCS]
set_false_path -from [get_registers *]      -to [get_ports oFlash_DI]
set_false_path -from [get_ports iFlash_DO]  -to [get_registers *]

# ------------------------------------------------------------------------------
# Remote update core
# -> Cut from/to remote update core clock
set_clock_groups -asynchronous -group [get_clocks $clk25]

# ------------------------------------------------------------------------------
# other I/Os
# -> Cut path
set_false_path -from [get_registers *]      -to [get_ports LEDG[*]]
set_false_path -from [get_registers *]      -to [get_ports BENCHMARK_PCP[*]]
set_false_path -from [get_registers *]      -to [get_ports BENCHMARK_PCIE[*]]

#--------------------------------------------------------------------------------
# PCIE
# -> Cut path
set_false_path -from [get_ports PCIE_PERST_N]  -to [get_registers *]
