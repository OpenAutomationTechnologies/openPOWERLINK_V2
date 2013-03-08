##############################################################################
# Host Interface Timing Constraints for
#  (De-)Multiplexed Asynchronous Parallel Bus
##############################################################################

##############################################################################
# I/O MIN/MAX DELAY
set hostif_in_max   7.0
set hostif_in_min   1.0
set hostif_out_max  7.0
set hostif_out_min  0.0

##############################################################################
# HIERARCHY
set instHostif  *alteraHostInterface
set instPar     $instHostif*parallelInterface
set instIrqGen  $instHostif*irqGen

##############################################################################
# SETUP / HOLD Timing for inputs

## CHIPSELECT
set_max_delay -from [get_ports *] -to [get_registers $instPar*syncChipselect|s0] $hostif_in_max
set_min_delay -from [get_ports *] -to [get_registers $instPar*syncChipselect|s0] $hostif_in_min

## WRITE
set_max_delay -from [get_ports *] -to [get_registers $instPar*syncWrite|s0] $hostif_in_max
set_min_delay -from [get_ports *] -to [get_registers $instPar*syncWrite|s0] $hostif_in_min

## READ
set_max_delay -from [get_ports *] -to [get_registers $instPar*syncRead|s0] $hostif_in_max
set_min_delay -from [get_ports *] -to [get_registers $instPar*syncRead|s0] $hostif_in_min

## ADDRESSLATCHENABLE
if {[get_registers $instPar*syncAle|s0] != ""} {
    set_max_delay -from [get_ports *] -to [get_registers $instPar*syncAle|s0] $hostif_in_max
    set_min_delay -from [get_ports *] -to [get_registers $instPar*syncAle|s0] $hostif_in_min
}

## BYTEENABLE
set_max_delay -from [get_ports *] -to [get_registers $instPar*byteenableRegister[*]] $hostif_in_max
set_min_delay -from [get_ports *] -to [get_registers $instPar*byteenableRegister[*]] $hostif_in_min

## ADDRESSDATABUS to ADDRESS REGISTER
set_max_delay -from [get_ports *] -to [get_registers $instPar*addressRegister[*]] $hostif_in_max
set_min_delay -from [get_ports *] -to [get_registers $instPar*addressRegister[*]] $hostif_in_min

## ADDRESSDATABUS to WRITE REGISTER
set_max_delay -from [get_ports *] -to [get_registers $instPar*writeDataRegister[*]] $hostif_in_max
set_min_delay -from [get_ports *] -to [get_registers $instPar*writeDataRegister[*]] $hostif_in_min

##############################################################################
# CLOCK TO OUTPUT Timing for outputs

## ACKNOWLEDGE
set_max_delay -from [get_registers $instPar*hostAck_reg] -to [get_ports *] $hostif_out_max
set_min_delay -from [get_registers $instPar*hostAck_reg] -to [get_ports *] $hostif_out_min

## ADDRESSDATABUS from READ REGISTER
set_max_delay -from [get_registers $instPar*readDataRegister[*]] -to [get_ports *] $hostif_out_max
set_min_delay -from [get_registers $instPar*readDataRegister[*]] -to [get_ports *] $hostif_out_min

## ADDRESSDATABUS OUTPUT ENABLE
set_max_delay -from [get_registers $instPar*hostDataEnable_reg] -to [get_ports *] $hostif_out_max
set_min_delay -from [get_registers $instPar*hostDataEnable_reg] -to [get_ports *] $hostif_out_min

##############################################################################
# TIMING IGNORE

## IRQ
set_false_path -from [get_registers $instIrqGen*irq_reg] -to [get_ports *]
