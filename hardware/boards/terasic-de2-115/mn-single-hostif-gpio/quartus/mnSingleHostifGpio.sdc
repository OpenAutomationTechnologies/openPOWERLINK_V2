# ------------------------------------------------------------------------------
# SDC for TERASIC DE2-115 MN dual GPIO
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
set_false_path -from [get_registers *] -to [get_ports LEDG[*]]
set_false_path -from [get_registers *] -to [get_ports LCD_*]
set_false_path -from [get_registers *] -to [get_ports LCD_DQ[*]]
set_false_path -from [get_ports LCD_DQ[*]] -to [get_registers *]

# ------------------------------------------------------------------------------
# Host interface definitions
# -> Adbus ipcore driven with 100 MHz
create_generated_clock -source $clk100 -name CLKHOSTIF_virt

set hostif_tper     10.0
set hostif_tsd      2.5
set hostif_thd      2.5
set hostif_tco      7.0

set hostif_in_max   [expr $hostif_tper - $hostif_tsd]
set hostif_in_min   $hostif_thd
set hostif_out_max  [expr $hostif_tper - $hostif_tco]
set hostif_out_max  0

## TCO
set_output_delay -clock CLKHOSTIF_virt -max $hostif_out_max [get_ports HOSTIF_AD[*]]
set_output_delay -clock CLKHOSTIF_virt -min $hostif_out_max [get_ports HOSTIF_AD[*]]
## TCO
set_output_delay -clock CLKHOSTIF_virt -max $hostif_out_max [get_ports HOSTIF_BE[*]]
set_output_delay -clock CLKHOSTIF_virt -min $hostif_out_max [get_ports HOSTIF_BE[*]]
## TCO
set_output_delay -clock CLKHOSTIF_virt -max $hostif_out_max [get_ports HOSTIF_CS_n]
set_output_delay -clock CLKHOSTIF_virt -min $hostif_out_max [get_ports HOSTIF_CS_n]
## TCO
set_output_delay -clock CLKHOSTIF_virt -max $hostif_out_max [get_ports HOSTIF_WR_n]
set_output_delay -clock CLKHOSTIF_virt -min $hostif_out_max [get_ports HOSTIF_WR_n]
## TCO
set_output_delay -clock CLKHOSTIF_virt -max $hostif_out_max [get_ports HOSTIF_ALE_n]
set_output_delay -clock CLKHOSTIF_virt -min $hostif_out_max [get_ports HOSTIF_ALE_n]
## TCO
set_output_delay -clock CLKHOSTIF_virt -max $hostif_out_max [get_ports HOSTIF_RD_n]
set_output_delay -clock CLKHOSTIF_virt -min $hostif_out_max [get_ports HOSTIF_RD_n]
## TSU / TH
set_input_delay -clock CLKHOSTIF_virt -max $hostif_in_max [get_ports HOSTIF_AD[*]]
set_input_delay -clock CLKHOSTIF_virt -min $hostif_in_min [get_ports HOSTIF_AD[*]]
## TSU / TH
set_input_delay -clock CLKHOSTIF_virt -max $hostif_in_max [get_ports HOSTIF_ACK_n]
set_input_delay -clock CLKHOSTIF_virt -min $hostif_in_min [get_ports HOSTIF_ACK_n]
## cut
set_false_path -from [get_ports HOSTIF_IRQ_n] -to [get_registers *]
