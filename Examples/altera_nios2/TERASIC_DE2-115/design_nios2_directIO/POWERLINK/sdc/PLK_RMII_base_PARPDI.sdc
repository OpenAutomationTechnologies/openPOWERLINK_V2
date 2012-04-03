# SDC file for POWERLINK Slave reference design with
# - RMII phys (DP83640)
# - SRAM (10 ns - IS61WV51216BLL)
# - Nios II (PCP) with 90 MHz
# - PDI for parallel async interface (8/16bit)

# ----------------------------------------------------------------------------------
# clock definitions
## define the clocks in your design (depends on your PLL settings!)
##  (under "Compilation Report" - "TimeQuest Timing Analyzer" - "Clocks")
set ext_clk		EXT_CLK
set clk50 		inst|the_altpll_0|sd1|pll7|clk[0]
set clk100		inst|the_altpll_0|sd1|pll7|clk[2]
set clkPcp		inst|the_altpll_0|sd1|pll7|clk[1]
set clkAp		inst|the_altpll_0|sd1|pll7|clk[3]
set clk25		inst|the_altpll_0|sd1|pll7|clk[4]

## define which clock drives SRAM controller
set clkSRAM		$clkPcp
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
## used by RMII
create_generated_clock -source $clk50 -name CLK50_virt
## used by SRAM
create_generated_clock -source $clkSRAM -name CLKSRAM_virt

# define clock groups
## clock group A includes openMAC + RMII phys
set clkGroupA	[format "%s %s %s" $clk50 $clk100 CLK50_virt]
## clock group B includes Nios II + SRAM
set clkGroupB	[format "%s %s" $clkPcp CLKSRAM_virt]

set_clock_groups -asynchronous 	-group $clkGroupA \
											-group $clkGroupB \
											-group $clkAp \
											-group $clk25 \
											-group $ext_clk

# cut reset input
set_false_path -from [get_ports RESET_n] -to [get_registers *]
# ----------------------------------------------------------------------------------

# ----------------------------------------------------------------------------------
# sram (IS61WV51216BLL-10TLI)
## SRAM is driven by 90 MHz fsm.
## Note: The SOPC inserts 2 write and 2 read cycles, thus, the SRAM "sees" 45 MHz!
set sram_clk		45.0
set sram_tper		[expr 1000.0 / $sram_clk]
## delay Address Access Time (tAA) = 10.0 ns
set sram_ddel		10.0
## pcb delay
set sram_tpcb		0.1
## fpga settings...
set sram_tco		5.5
set sram_tsu		[expr $sram_tper - $sram_ddel - $sram_tco - 2*$sram_tpcb]
set sram_th			0.0
set sram_tcom		0.0

set sram_in_max	[expr $sram_tper - $sram_tsu]
set sram_in_min	$sram_th
set sram_out_max	[expr $sram_tper - $sram_tco]
set sram_out_min	$sram_tcom

## TSU / TH
set_input_delay -clock CLKSRAM_virt -max $sram_in_max [get_ports SRAM_DQ[*]]
set_input_delay -clock CLKSRAM_virt -min $sram_in_min [get_ports SRAM_DQ[*]]
## TCO
set_output_delay -clock CLKSRAM_virt -max $sram_out_max [get_ports SRAM_DQ[*]]
set_output_delay -clock CLKSRAM_virt -min $sram_out_min [get_ports SRAM_DQ[*]]
## TCO
set_output_delay -clock CLKSRAM_virt -max $sram_out_max [get_ports SRAM_ADDR[*]]
set_output_delay -clock CLKSRAM_virt -min $sram_out_min [get_ports SRAM_ADDR[*]]
## TCO
set_output_delay -clock CLKSRAM_virt -max $sram_out_max [get_ports SRAM_BE_n[*]]
set_output_delay -clock CLKSRAM_virt -min $sram_out_min [get_ports SRAM_BE_n[*]]
## TCO
set_output_delay -clock CLKSRAM_virt -max $sram_out_max [get_ports SRAM_OE_n]
set_output_delay -clock CLKSRAM_virt -min $sram_out_min [get_ports SRAM_OE_n]
## TCO
set_output_delay -clock CLKSRAM_virt -max $sram_out_max [get_ports SRAM_WE_n]
set_output_delay -clock CLKSRAM_virt -min $sram_out_min [get_ports SRAM_WE_n]
## TCO
set_output_delay -clock CLKSRAM_virt -max $sram_out_max [get_ports SRAM_CE_n]
set_output_delay -clock CLKSRAM_virt -min $sram_out_min [get_ports SRAM_CE_n]

## relax timing...
## Note: Nios II is running with 90 MHz, but Tri-State-bridge reads with 45 MHz.
### from FPGA to SRAM
set_multicycle_path -from [get_clocks $clkSRAM] -to [get_clocks CLKSRAM_virt] -setup -start 2
set_multicycle_path -from [get_clocks $clkSRAM] -to [get_clocks CLKSRAM_virt] -hold -start 1
### from SRAM to FPGA
set_multicycle_path -from [get_clocks CLKSRAM_virt] -to [get_clocks $clkSRAM] -setup -end 2
set_multicycle_path -from [get_clocks CLKSRAM_virt] -to [get_clocks $clkSRAM] -hold -end 1
# ----------------------------------------------------------------------------------

# ----------------------------------------------------------------------------------
# RMII
## phy = NATIONAL DP83640
set phy_tsu			4.0
set phy_th			2.0
set phy_tco			14.0
set phy_tcomin		2.0
## pcb delay
set phy_tpcb		0.1

set phy_in_max		[expr $phy_tco + $phy_tpcb]
set phy_in_min		[expr $phy_tcomin - $phy_tpcb]
set phy_out_max	[expr $phy_tsu + $phy_tpcb]
set phy_out_min	[expr $phy_th - $phy_tpcb]

## input
set_input_delay -clock CLK50_virt -max $phy_in_max [get_ports {PHY?_RXDV PHY?_RXER PHY?_RXD[*]}]
set_input_delay -clock CLK50_virt -min $phy_in_min [get_ports {PHY?_RXDV PHY?_RXER PHY?_RXD[*]}]
## output
set_output_delay -clock CLK50_virt -max $phy_out_max [get_ports {PHY?_TXEN PHY?_TXD[*]}]
set_output_delay -clock CLK50_virt -min $phy_out_min [get_ports {PHY?_TXEN PHY?_TXD[*]}]
## cut path
set_false_path -from [get_registers *] -to [get_ports PHY?_RESET_n]
set_false_path -from [get_registers *] -to [get_ports PHY?_MDC]
set_false_path -from [get_registers *] -to [get_ports PHY?_MDIO]
set_false_path -from [get_ports PHY?_MDIO] -to [get_registers *]
set_false_path -from [get_ports PHY?_LINK] -to [get_registers *]
## multicycle
## Note: TX signals are latched at falling edge of 100 MHz signal
### from FPGA to PHY
set_multicycle_path -from [get_clocks $clk100] -to [get_ports {PHY?_TXEN PHY?_TXD[*]}] -setup -start 2
set_multicycle_path -from [get_clocks $clk100] -to [get_ports {PHY?_TXEN PHY?_TXD[*]}] -hold -start 1
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
#### example for output: set_false_path -from [get_registers *] -to [get_ports LED[*]]
#### example for input:  set_false_path -from [get_ports BUTTON[*]] -to [get_registers *]
#############################################################
# add here your slow IOs...
#############################################################
# ----------------------------------------------------------------------------------

# ----------------------------------------------------------------------------------
# PDI async parallel interface (8/16bit)
## PDI interface timing constraints
### write pulse width
set pdi_tpwr		20.0
### hold address valid (time to next rising edge of write signal)
set pdi_tha			40.0
### data setup/hold time
set pdi_tsd			5.0
set pdi_thd			2.0

## create parallel port write strobe as clock
### define period to "worst case" (=one writes subsequently patterns to FPGA)
set pdi_tperwr		[expr $pdi_tpwr + $pdi_tha]

#############################################################
# uncomment following lines if PDI_WR is used

### write signal is high active (falling edge)
# create clock
create_clock -name WRITE_EDGE -period 60.0 [get_ports {PDI_WR}] -waveform {0.0 20.0}
# create virtual clock
create_clock -name WRITE_EDGE_virt -period 60.0 -waveform {0.0 20.0}
# set clock group
set_clock_groups -asynchronous -group {WRITE_EDGE WRITE_EDGE_virt}
# and setup/hold requirement
set_input_delay -clock WRITE_EDGE_virt -max [expr $pdi_tperwr - $pdi_tsd] [get_ports {PDI_DATA[*]}] -clock_fall
set_input_delay -clock WRITE_EDGE_virt -min $pdi_thd [get_ports {PDI_DATA[*]}] -clock_fall

# uncomment following lines if PDI_WR_n is used

#### write signal is low active (rising edge)
## create clock
#create_clock -name WRITE_EDGE_NEG -period 60.0 [get_ports {PDI_WR_n}] -waveform {0.0 40.0}
## create virtual clock
#create_clock -name WRITE_EDGE_NEG_virt -period 60.0 -waveform {0.0 40.0}
## set clock group
#set_clock_groups -asynchronous -group {WRITE_EDGE WRITE_EDGE_NEG_virt}
## and setup/hold requirement
#set_input_delay -clock WRITE_EDGE_NEG_virt -max [expr $pdi_tperwr - $pdi_tsd] [get_ports {PDI_DATA[*]}]
#set_input_delay -clock WRITE_EDGE_NEG_virt -min $pdi_thd [get_ports {PDI_DATA[*]}]
#############################################################

### input delay for others
#### set input delay
set pdi_max_in		15.0
set pdi_min_in		0.0
####
set_input_delay -clock CLK50_virt -max $pdi_max_in [get_ports {PDI_ADDR[*]}]
set_input_delay -clock CLK50_virt -min $pdi_min_in [get_ports {PDI_ADDR[*]}]
####
set_input_delay -clock CLK50_virt -max $pdi_max_in [get_ports {PDI_BE*[*]}]
set_input_delay -clock CLK50_virt -min $pdi_min_in [get_ports {PDI_BE*[*]}]
####
set_input_delay -clock CLK50_virt -max $pdi_max_in [get_ports {PDI_CS*}]
set_input_delay -clock CLK50_virt -min $pdi_min_in [get_ports {PDI_CS*}]
####
set_input_delay -clock CLK50_virt -max $pdi_max_in [get_ports {PDI_RD*}]
set_input_delay -clock CLK50_virt -min $pdi_min_in [get_ports {PDI_RD*}]
####
set_input_delay -clock CLK50_virt -max $pdi_max_in [get_ports {PDI_WR*}]
set_input_delay -clock CLK50_virt -min $pdi_min_in [get_ports {PDI_WR*}]

## clock-2-output requirements
### set output delay
set pdi_max_out 	5.0
set pdi_min_out 	0.0

set_output_delay -clock CLK50_virt -max $pdi_max_out [get_ports {PDI_DATA[*]}]
set_output_delay -clock CLK50_virt -min $pdi_min_out [get_ports {PDI_DATA[*]}]

## cut paths (as long as we don't use them...]
set_false_path -from [get_registers *] -to [get_ports {PDI_GPIO[*]]}]
# ----------------------------------------------------------------------------------