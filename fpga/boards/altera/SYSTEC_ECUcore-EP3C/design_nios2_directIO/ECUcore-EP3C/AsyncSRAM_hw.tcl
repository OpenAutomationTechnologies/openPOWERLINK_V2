# +-----------------------------------
# | module AsyncSRAM
# |
set_module_property DESCRIPTION "32bit SRAM Interface"
set_module_property NAME AsyncSRAM
set_module_property VERSION 2.0
set_module_property INTERNAL false
set_module_property GROUP "Memories and Memory Controllers/SRAM"
set_module_property AUTHOR "SYS TEC electronic GmbH"
set_module_property DISPLAY_NAME IS61WV51216
set_module_property TOP_LEVEL_HDL_FILE ""
set_module_property INSTANTIATE_IN_SYSTEM_MODULE false
set_module_property EDITABLE true
# |
# +-----------------------------------

# +-----------------------------------
# | files
# |
# |
# +-----------------------------------

# +-----------------------------------
# | parameters
# |
# |
# +-----------------------------------

# +-----------------------------------
# | connection point avalon_tristate_slave
# |
add_interface avalon_tristate_slave avalon_tristate end
set_interface_property avalon_tristate_slave activeCSThroughReadLatency false
set_interface_property avalon_tristate_slave bridgesToMaster ""
set_interface_property avalon_tristate_slave holdTime 3
set_interface_property avalon_tristate_slave isMemoryDevice true
set_interface_property avalon_tristate_slave isNonVolatileStorage false
set_interface_property avalon_tristate_slave maximumPendingReadTransactions 0
set_interface_property avalon_tristate_slave printableDevice false
set_interface_property avalon_tristate_slave readLatency 0
set_interface_property avalon_tristate_slave readWaitTime 19
set_interface_property avalon_tristate_slave setupTime 0
set_interface_property avalon_tristate_slave timingUnits Nanoseconds
set_interface_property avalon_tristate_slave writeWaitTime 10

set_interface_property avalon_tristate_slave ENABLED true

add_interface_port avalon_tristate_slave addr address Input 19
add_interface_port avalon_tristate_slave data data Bidir 32
add_interface_port avalon_tristate_slave ncs chipselect_n Input 1
add_interface_port avalon_tristate_slave wrn write_n Input 1
add_interface_port avalon_tristate_slave rdn read_n Input 1
add_interface_port avalon_tristate_slave ben byteenable_n Input 4
# |
# +-----------------------------------
