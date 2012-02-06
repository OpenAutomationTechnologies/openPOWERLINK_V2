# SDC file for POWERLINK Slave reference design with
# - MII phys (88E1111)
# - SRAM (10 ns - IS61WV102416BLL)
# - Nios II (PCP) with 100 MHz

# ----------------------------------------------------------------------------------
# clock definitions
## define the clocks in your design (depends on your PLL settings!)
##  (under "Compilation Report" - "TimeQuest Timing Analyzer" - "Clocks")
set ext_clk			EXT_CLK
set clk50 			inst|the_altpll_0|sd1|pll7|clk[0]
set clkPcp			inst|the_altpll_0|sd1|pll7|clk[1]
set clk25			inst|the_altpll_0|sd1|pll7|clk[2]
#set clkAp			inst|the_altpll_0|sd1|pll7|clk[3]
#set clkAp_SDRAM	inst|the_altpll_0|sd1|pll7|clk[4]

set p0TxClk		PHY0_TXCLK
set p0RxClk		PHY0_RXCLK
set p1TxClk		PHY1_TXCLK
set p1RxClk		PHY1_RXCLK

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
## used by SRAM
create_generated_clock -source $clkSRAM -name CLKSRAM_virt

# cut reset input
#set_false_path -from [get_ports RESET_n] -to [get_registers *]
# ----------------------------------------------------------------------------------

# ----------------------------------------------------------------------------------
# sram (IS61WV102416BLL-10TLI)
## SRAM is driven by 100 MHz fsm.
## Note: The SOPC inserts 2 write and 2 read cycles, thus, the SRAM "sees" 50 MHz!
set sram_clk		50.0
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
# MII
# phy = MARVELL 88E1111
set phy_tper		40.0
set phy_tout2clk	10.0
set phy_tclk2out	10.0
set phy_tsu			10.0
set phy_th			0.0
# pcb delay
set phy_tpcb		0.1

set phy_in_max		[expr $phy_tper - ($phy_tout2clk - $phy_tpcb)]
set phy_in_min		[expr $phy_tclk2out - $phy_tpcb]
set phy_out_max	[expr $phy_tsu + $phy_tpcb]
set phy_out_min	[expr $phy_tclk2out - $phy_tpcb]

##PHY0
## real clock
create_clock -period 25MHz -name phy0_rxclk [get_ports $p0RxClk]
create_clock -period 25MHz -name phy0_txclk [get_ports $p0TxClk]
## virtual clock
create_clock -period 25MHz -name phy0_vrxclk
create_clock -period 25MHz -name phy0_vtxclk
## input
set_input_delay -clock phy0_vrxclk -max $phy_in_max [get_ports {PHY0_RXDV PHY0_RXD[*]}]
set_input_delay -clock phy0_vrxclk -min $phy_in_min [get_ports {PHY0_RXDV PHY0_RXD[*]}]
## output
set_output_delay -clock phy0_vtxclk -max $phy_out_max [get_ports {PHY0_TXEN PHY0_TXD[*]}]
set_output_delay -clock phy0_vtxclk -min $phy_out_min [get_ports {PHY0_TXEN PHY0_TXD[*]}]
## cut path
set_false_path -from [get_registers *] -to [get_ports PHY0_GXCLK]
set_false_path -from [get_registers *] -to [get_ports PHY0_RESET_n]
set_false_path -from [get_registers *] -to [get_ports PHY0_MDC]
set_false_path -from [get_registers *] -to [get_ports PHY0_MDIO]
set_false_path -from [get_ports PHY0_MDIO] -to [get_registers *]
set_false_path -from [get_ports PHY0_LINK] -to [get_registers *]
set_false_path -from [get_ports PHY0_RXER] -to [get_registers *]

##PHY1
## real clock
create_clock -period 25MHz -name phy1_rxclk [get_ports $p1RxClk]
create_clock -period 25MHz -name phy1_txclk [get_ports $p1TxClk]
## virtual clock
create_clock -period 25MHz -name phy1_vrxclk
create_clock -period 25MHz -name phy1_vtxclk
## input
set_input_delay -clock phy1_vrxclk -max $phy_in_max [get_ports {PHY1_RXDV PHY1_RXD[*]}]
set_input_delay -clock phy1_vrxclk -min $phy_in_min [get_ports {PHY1_RXDV PHY1_RXD[*]}]
## output
set_output_delay -clock phy1_vtxclk -max $phy_out_max [get_ports {PHY1_TXEN PHY1_TXD[*]}]
set_output_delay -clock phy1_vtxclk -min $phy_out_min [get_ports {PHY1_TXEN PHY1_TXD[*]}]
## cut path
set_false_path -from [get_registers *] -to [get_ports PHY1_GXCLK]
set_false_path -from [get_registers *] -to [get_ports PHY1_RESET_n]
set_false_path -from [get_registers *] -to [get_ports PHY1_MDC]
set_false_path -from [get_registers *] -to [get_ports PHY1_MDIO]
set_false_path -from [get_ports PHY1_MDIO] -to [get_registers *]
set_false_path -from [get_ports PHY1_LINK] -to [get_registers *]
set_false_path -from [get_ports PHY1_RXER] -to [get_registers *]

# ----------------------------------------------------------------------------------

# ----------------------------------------------------------------------------------
# Set clock groups (cut paths)
set_clock_groups -asynchronous 	-group $clk50 \
											-group [format "%s %s" $clkPcp CLKSRAM_virt] \
											-group $clk25 \
											-group [format "%s %s" phy0_rxclk phy0_vrxclk] \
											-group [format "%s %s" phy0_txclk phy0_vtxclk] \
											-group [format "%s %s" phy1_rxclk phy1_vrxclk] \
											-group [format "%s %s" phy1_txclk phy1_vtxclk] \
											-group $ext_clk \
											
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