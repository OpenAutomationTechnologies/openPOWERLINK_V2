# SDC file for POWERLINK Master reference design with
# - Nios II (PCP) with 100 MHz

# ----------------------------------------------------------------------------------
# clock definitions
## define the clocks in your design (depends on your PLL settings!)
##  (under "Compilation Report" - "TimeQuest Timing Analyzer" - "Clocks")
set ext_clk         EXT_CLK

set clk50           pllInst|altpll_component|auto_generated|pll1|clk[0]
set clk100          pllInst|altpll_component|auto_generated|pll1|clk[1]
# ----------------------------------------------------------------------------------

# ----------------------------------------------------------------------------------
# note: changes below this line have to be done carefully

# ----------------------------------------------------------------------------------
# constrain JTAG
create_clock -period 10MHz {altera_reserved_tck}
set_clock_groups -asynchronous -group {altera_reserved_tck}
set_input_delay -clock {altera_reserved_tck} 20 [get_ports altera_reserved_tdi]
set_input_delay -clock {altera_reserved_tck} 20 [get_ports altera_reserved_tms]
set_output_delay -clock {altera_reserved_tck} 20 [get_ports altera_reserved_tdo]
# ----------------------------------------------------------------------------------

# ----------------------------------------------------------------------------------
# derive pll clocks (generated + input)
derive_pll_clocks -create_base_clocks
derive_clock_uncertainty
# ----------------------------------------------------------------------------------

# ----------------------------------------------------------------------------------
# create virtual clocks

## used by HOST INTERFACE
create_generated_clock -source $clk100 -name CLKHOSTIF_virt

# cut reset input
#set_false_path -from [get_ports RESET_n] -to [get_registers *]
# ----------------------------------------------------------------------------------

# ----------------------------------------------------------------------------------
# Host Interface
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
# ----------------------------------------------------------------------------------

# ----------------------------------------------------------------------------------
# Set clock groups (cut paths)
set_clock_groups -asynchronous \
                    -group $clk50 \
                    -group $clk100 \
                    -group $ext_clk
# ----------------------------------------------------------------------------------

# ----------------------------------------------------------------------------------
# IOs
## cut paths
###EPCS
set_false_path -from [get_registers *] -to [get_ports EPCS_DCLK]
set_false_path -from [get_registers *] -to [get_ports EPCS_SCE]
set_false_path -from [get_registers *] -to [get_ports EPCS_SDO]
set_false_path -from [get_ports EPCS_DATA0] -to [get_registers *]
###IOs
set_false_path -from [get_registers *] -to [get_ports LEDG[*]]
set_false_path -from [get_registers *] -to [get_ports LCD_*]
set_false_path -from [get_registers *] -to [get_ports LCD_DQ[*]]
set_false_path -from [get_ports LCD_DQ[*]] -to [get_registers *]
#### example for output: set_false_path -from [get_registers *] -to [get_ports LED[*]]
#### example for input:  set_false_path -from [get_ports BUTTON[*]] -to [get_registers *]
#############################################################
# add here your slow IOs...
#############################################################
# ----------------------------------------------------------------------------------