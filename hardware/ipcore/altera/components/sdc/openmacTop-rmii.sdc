###############################################################################
# OPENMAC Timing Constraints for
# RMII Phys
###############################################################################

###############################################################################
# LOCAL FUNCTIONS

# Get full path to clocks and registers.
proc getRegisterFullPath { regString dir } {
    set clk     [get_fanins $regString -clock]
    if { $dir == "INPUT" } {
        set data    [get_fanins $regString -synch]
    } else {
        set data    [get_fanouts $regString]
    }
    set regs    [get_registers $regString]

    foreach_in_collection fanin $clk {
        set clkName [get_node_info $fanin -name]
        lappend lstClk $clkName
    }

    foreach_in_collection reg $regs {
        set regName [get_node_info $reg -name]
        lappend lstReg $regName
    }

    foreach_in_collection fanin $data {
        set dataName [get_node_info $fanin -name]
        lappend lstData $dataName
    }

    set ret [list $lstClk $lstReg $lstData]

    return $ret
}

# Create for every found register:
# * Input or Output delay with corresponding Virtual Clock
# lst_reg : search strings for registers
# dir : direction "INPUT" or "OUTPUT"
proc createRegisterTiming { lst_reg hubPorts dir virtClk tmax tmin} {
    foreach reg $lst_reg {
        set regFull     [getRegisterFullPath $reg $dir]
        set lstReg      [lindex $regFull 1]
        set numReg      [llength $lstReg]
        set lstPort     [lindex $regFull 2]
        set numPort     [llength $lstPort]

        for {set i 0} {$i < $hubPorts} {incr i} {
            set numOffset [expr $i * $numReg / $hubPorts]

            for {set j 0} {$j < [expr $numReg / $hubPorts]} {incr j} {
                set tmpPort [lindex $lstPort [expr $numOffset + $j]]

                # Assign delay
                if { ${dir} == "INPUT" } {
                    set_input_delay -clock $virtClk -max $tmax [get_ports $tmpPort]
                    set_input_delay -clock $virtClk -min $tmin [get_ports $tmpPort]
                } else {
                    set_output_delay -clock $virtClk -max $tmax [get_ports $tmpPort]
                    set_output_delay -clock $virtClk -min $tmin [get_ports $tmpPort]
                }
            }
        }
    }
}

# Checks if the provided register(s) exist(s).
# Returns number of found registers.
proc checkRegExist { reg } {
    set cnt 0

    foreach_in_collection reg [get_registers ${reg} -nowarn] {
        incr cnt
    }

    return ${cnt}
}

###############################################################################
# PCB delay (FPGA <--> PHY(s)) [ns]
set tpcb            0.1

# RMII Timing [ns]
set rmii_tsu        4.0
set rmii_th         2.0
set rmii_tco        14.0
set rmii_tcomin     2.0

# I/O MIN/MAX DELAY [ns]
set rmii_in_max     [expr $rmii_tco    + $tpcb ]
set rmii_in_min     [expr $rmii_tcomin - $tpcb ]
set rmii_out_max    [expr $rmii_tsu    + $tpcb ]
set rmii_out_min    [expr $rmii_th     - $tpcb ]

###############################################################################
# RMII CLOCK RATE
set rmii_rx_clk_rate    50.0MHz
set rmii_tx_clk_rate    100.0MHz

set tmpVirtClk          virt_phy_clk

###############################################################################
# HIERARCHY
set instOpenMacTop  *alteraOpenmacTop
set instSmi         ${instOpenMacTop}*phyMgmt
set instDmaMaster   ${instOpenMacTop}*master_handler

###############################################################################
# REGISTERS

## Capture registers
set reg_smiIn           ${instSmi}*shift_reg[0]
set reg_rxPath          ${instOpenMacTop}*rmiiRxPath_reg*
set reg_rxError         ${instOpenMacTop}*rmiiRxPathError_reg*

# Create input register list
set lst_reg_in  [list ${reg_rxPath} ${reg_rxError}]

## Output registers
set reg_phyRst          ${instSmi}*nPhyReset
set reg_smiClk          ${instSmi}*smiClk*
set reg_smiOut          ${instSmi}*smiDataOut*
set reg_txPath          ${instOpenMacTop}*rmiiTxPath_reg*

# Create output register list
set lst_reg_out [list ${reg_txPath}]

# Get number of hub ports
set hubPorts [llength [lindex [getRegisterFullPath $reg_rxError "INPUT"] 1]]

###############################################################################
# SETUP / HOLD Timing for inputs

## RX Path

# First obtain clock by getting any rx register
set lstReg      [lindex [getRegisterFullPath $reg_rxPath "INPUT"] 1]
set firstReg    [lindex $lstReg 0]
set rxClk      rmii_rx_clk

create_clock -name $rxClk -period $rmii_rx_clk_rate $lstReg
create_generated_clock -source $firstReg -name $tmpVirtClk

createRegisterTiming $lst_reg_in $hubPorts "INPUT" $tmpVirtClk $rmii_in_max $rmii_in_min

###############################################################################
# CLOCK TO OUTPUT Timing for outputs

## TX Path

# First obtain clock and get any rx register
set lstReg      [lindex [getRegisterFullPath $reg_txPath "OUTPUT"] 1]
set firstReg    [lindex $lstReg 0]
set txClk      rmii_tx_clk

create_clock -name $txClk -period $rmii_tx_clk_rate $lstReg

# Relax timing, since tx_clk uses falling edge of 100 MHz.
set_multicycle_path -from $txClk -to $tmpVirtClk -setup -start 2
set_multicycle_path -from $txClk -to $tmpVirtClk -hold -start 1

createRegisterTiming $lst_reg_out $hubPorts "OUTPUT" $tmpVirtClk $rmii_out_max $rmii_out_min

###############################################################################
# TIMING IGNORE

## SMI data in
set_false_path -from [get_ports *] -to [get_registers ${reg_smiIn}]

## SMI data out
set_false_path -from [get_registers ${reg_smiOut}] -to [get_ports *]

## SMI clock out
set_false_path -from [get_registers ${reg_smiClk}] -to [get_ports *]

## Phy reset
set_false_path -from [get_registers  ${reg_phyRst}] -to [get_ports *]

## MAC DMA to MASTER HANDLER path
if {[checkRegExist ${instDmaMaster}*tx_rd_cnt[*]] != 0} {
    set_false_path -from * -to [get_registers ${instDmaMaster}*tx_rd_cnt[*]]
    set_false_path -from * -to [get_registers ${instDmaMaster}*tx_cnt[*]]
}

if {[checkRegExist ${instDmaMaster}*rx_cnt[*]] != 0} {
    set_false_path -from * -to [get_registers ${instDmaMaster}*rx_cnt[*]]
}

## SYNCHRONIZER
if {[checkRegExist ${instOpenMacTop}*synchronizer*metaReg[*]] != 0} {
    set_false_path -from [get_registers *] -to [get_registers ${instOpenMacTop}*synchronizer*metaReg[*]]
}
