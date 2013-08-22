# SDC file for POWERLINK Slave reference design with
# - RMII phys (DP83640)
# - SRAM (10 ns - IS61WV51216BLL)
# - Nios II (PCP) with 100 MHz

# ----------------------------------------------------------------------------------
# clock definitions
## define the clocks in your design (depends on your PLL settings!)
##  (under "Compilation Report" - "TimeQuest Timing Analyzer" - "Clocks")
set ext_clk				ext_clk
set clk50 				inst|altpll_component|auto_generated|pll1|clk[0]
set clkPcp				inst|altpll_component|auto_generated|pll1|clk[1]
#set clk25				inst|the_altpll_0|sd1|pll7|clk[2]

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
set clkGroup	[format "%s %s %s %s" $clk50 $clkPcp CLKSRAM_virt CLK50_virt]


set_clock_groups -asynchronous 	-group $clkGroup \
				-group $ext_clk

# cut reset input
#set_false_path -from [get_ports rstn] -to [get_registers *]
# ----------------------------------------------------------------------------------

# ----------------------------------------------------------------------------------
# sram (IS61WV51216BLL-10TLI)
# SRAM is driven by 90 MHz fsm.
# Note: The SOPC inserts 2 write and 2 read cycles, thus, the SRAM "sees" 45 MHz!
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
set_input_delay -clock CLKSRAM_virt -max $sram_in_max [get_ports vData_inout[*]]
set_input_delay -clock CLKSRAM_virt -min $sram_in_min [get_ports vData_inout[*]]
## TCO
set_output_delay -clock CLKSRAM_virt -max $sram_out_max [get_ports vData_inout[*]]
set_output_delay -clock CLKSRAM_virt -min $sram_out_min [get_ports vData_inout[*]]
## TCO
set_output_delay -clock CLKSRAM_virt -max $sram_out_max [get_ports vAddr_out[*]]
set_output_delay -clock CLKSRAM_virt -min $sram_out_min [get_ports vAddr_out[*]]
## TCO
set_output_delay -clock CLKSRAM_virt -max $sram_out_max [get_ports vnBWE_out[*]]
set_output_delay -clock CLKSRAM_virt -min $sram_out_min [get_ports vnBWE_out[*]]
## TCO
set_output_delay -clock CLKSRAM_virt -max $sram_out_max [get_ports fnOE_RAM_out]
set_output_delay -clock CLKSRAM_virt -min $sram_out_min [get_ports fnOE_RAM_out]
## TCO
set_output_delay -clock CLKSRAM_virt -max $sram_out_max [get_ports fnWE_RAM_out]
set_output_delay -clock CLKSRAM_virt -min $sram_out_min [get_ports fnWE_RAM_out]
## TCO
set_output_delay -clock CLKSRAM_virt -max $sram_out_max [get_ports fnCS_RAM_out]
set_output_delay -clock CLKSRAM_virt -min $sram_out_min [get_ports fnCS_RAM_out]

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
set_input_delay -clock CLK50_virt -max $phy_in_max [get_ports {fETH?_ECRSDV_in vETH?_RXD_in[*]}]
set_input_delay -clock CLK50_virt -min $phy_in_min [get_ports {fETH?_ECRSDV_in vETH?_RXD_in[*]}]
## output
set_output_delay -clock CLK50_virt -max $phy_out_max [get_ports {fETH?_TXEN_out vETH?_TXD_out[*]}]
set_output_delay -clock CLK50_virt -min $phy_out_min [get_ports {fETH?_TXEN_out vETH?_TXD_out[*]}]
## cut path
set_false_path -from [get_registers *] -to [get_ports fnETH_RST_out]
set_false_path -from [get_registers *] -to [get_ports fETH_MDC_out]
set_false_path -from [get_registers *] -to [get_ports fETH_MDIO_inout]
#set_false_path -from [get_ports phy?_link] -to [get_registers *]
## multicycle
## Note: TX signals are latched at falling edge of 100 MHz signal
### from FPGA to PHY
set_multicycle_path -from [get_clocks $clkPcp] -to [get_ports {fETH?_TXEN_out vETH?_TXD_out[*]}] -setup -start 2
set_multicycle_path -from [get_clocks $clkPcp] -to [get_ports {fETH?_TXEN_out vETH?_TXD_out[*]}] -hold -start 1
# ----------------------------------------------------------------------------------

# ----------------------------------------------------------------------------------
# IOs
## cut paths
###EPCS
set_false_path -from [get_registers *] -to [get_ports fSPI_CLK_Flash_out]
set_false_path -from [get_registers *] -to [get_ports fnSPI_CS_Flash_out]
set_false_path -from [get_registers *] -to [get_ports fSPI_MTSR_Flash_out]
set_false_path -from [get_ports fSPI_MRST_Flash_in] -to [get_registers *]

###SPI
set_false_path -from [get_ports fHex_MRST_in] -to [get_registers *]
set_false_path -from [get_registers *] -to [get_ports fHex_PL_out]
set_false_path -from [get_registers *] -to [get_ports fHex_CLK_out]

###Debug
set_false_path -from [get_registers *] -to [get_ports vDBG_out[*]]

###IOs
#### example for output: set_false_path -from [get_registers *] -to [get_ports LED[*]]
#### example for input:  set_false_path -from [get_ports BUTTON[*]] -to [get_registers *]
#############################################################
# add here your slow IOs...
#############################################################
# ----------------------------------------------------------------------------------
