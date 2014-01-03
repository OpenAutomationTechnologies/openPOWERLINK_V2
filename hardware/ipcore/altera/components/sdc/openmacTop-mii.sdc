###############################################################################
# OPENMAC Timing Constraints for
# MII Phys
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
# * Clocks + Virtual Clocks ==> Clock Group
# * Input or Output delay with corresponding Clock
# lst_reg_in : search strings for registers
# dir : direction "INPUT" or "OUTPUT"
proc createRegisterTiming { lst_reg_in hubPorts dir tclk tmax tmin} {
    # Only the first run may create clocks
    set clkCreateDone FALSE

    # Run through the list...
    foreach reg $lst_reg_in {
        # First get the clock
        set regFull [getRegisterFullPath $reg $dir]
        set lstClk  [lindex $regFull 0]
        set numClk  [llength $lstClk]
        set lstReg  [lindex $regFull 1]
        set numReg  [llength $lstReg]
        set lstPort [lindex $regFull 2]
        set numPort [llength $lstPort]

        # Generate clocks and virtual clocks
        if { ${clkCreateDone} == FALSE } {
            set clkCreateDone TRUE
            foreach tmpClk $lstClk {
                # Create real clock
                create_clock -period $tclk -name $tmpClk [get_ports $tmpClk]

                # Create virtual clock
                set tmpVirtClk virt_${tmpClk}
                create_clock -period $tclk -name $tmpVirtClk

                # Add both to a group
                set_clock_groups -exclusive -group [format "%s %s" $tmpClk $tmpVirtClk]
            }
        }

        for {set i 0} {$i < $hubPorts} {incr i} {
            set tmpClk [lindex $lstClk $i]
            set numOffset [expr $i * $numReg / $hubPorts]

            # Just build the virtual clock name again...
            set tmpVirtClk virt_${tmpClk}

            for {set j 0} {$j < [expr $numReg / $hubPorts]} {incr j} {
                set tmpReg [lindex $lstReg [expr $numOffset + $j]]
                set tmpPort [lindex $lstPort [expr $numOffset + $j]]

                # Assign delay
                if { ${dir} == "INPUT" } {
                    set_input_delay -clock $tmpVirtClk -max $tmax [get_ports $tmpPort]
                    set_input_delay -clock $tmpVirtClk -min $tmin [get_ports $tmpPort]
                } else {
                    set_output_delay -clock $tmpVirtClk -max $tmax [get_ports $tmpPort]
                    set_output_delay -clock $tmpVirtClk -min $tmin [get_ports $tmpPort]
                }
            }
        }
    }
}

###############################################################################
# PCB delay (FPGA <--> PHY(s)) [ns]
set tpcb            0.1

# MII Timing [ns]
set mii_tper        40.0
set mii_tout2clk    10.0
set mii_tclk2out    10.0
set mii_tsu         10.0
set mii_th          0.0

# I/O MIN/MAX DELAY [ns]
set mii_in_max      [expr $mii_tper - ( $mii_tout2clk + $tpcb ) ]
set mii_in_min      [expr               $mii_tclk2out - $tpcb   ]
set mii_out_max     [expr               $mii_tsu      + $tpcb   ]
set mii_out_min     [expr               $mii_tclk2out - $tpcb   ]

###############################################################################
# MII CLOCK RATE
set mii_clk_rate    25.0MHz

###############################################################################
# HIERARCHY
set instOpenMacTop  *alteraOpenmacTop
set instSmi         $instOpenMacTop*phyMgmt
set instConvMiiCore $instOpenMacTop*convRmiiToMii

###############################################################################
# REGISTERS

## Capture registers
set reg_smiIn           ${instSmi}*shift_reg[0]
set reg_rxData          ${instConvMiiCore}*rxData_reg*
set reg_rxDataValid     ${instConvMiiCore}*rxDataValid_reg*
set reg_rxError         ${instConvMiiCore}*rxError_reg*

# Create input register list
set lst_reg_in  [list ${reg_rxData} ${reg_rxDataValid} ${reg_rxError}]

## Output registers
set reg_phyRst          ${instSmi}*nPhyReset
set reg_smiClk          ${instSmi}*smiClk*
set reg_smiOut          ${instSmi}*smiDataOut*
set reg_txData          ${instConvMiiCore}*txData_reg*
set reg_txEnable        ${instConvMiiCore}*txEnable_reg*

# Create output register list
set lst_reg_out [list ${reg_txData} ${reg_txEnable}]

# Get number of hub ports
set hubPorts [llength [lindex [getRegisterFullPath $reg_rxError "INPUT"] 1]]

###############################################################################
# SETUP / HOLD Timing for inputs

## RX Path

createRegisterTiming $lst_reg_in $hubPorts "INPUT" $mii_clk_rate $mii_in_max $mii_in_min

###############################################################################
# CLOCK TO OUTPUT Timing for outputs

## TX Path

createRegisterTiming $lst_reg_out $hubPorts "OUTPUT" $mii_clk_rate $mii_out_max $mii_out_min

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
