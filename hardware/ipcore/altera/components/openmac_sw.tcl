#
# openMAC _SW.TCL
#

# Create driver for host interface
create_driver openmac

# Define hw name for the driver
set_sw_property hw_class_name openmac

# Define driver version
set_sw_property version 1.0.2

# Define minimum version of hw compatible
set_sw_property min_compatible_hw_version 1.0.2

# Don't initialize driver in alt_sys_init()
set_sw_property auto_initialize false

# Where to copy driver into BSP
set_sw_property bsp_subdirectory drivers

# This driver supports enhanced Interrupt Api only
set_sw_property supported_interrupt_apis enhanced_interrupt_api

#
# Header/Source Files
#

# create BSP subdir for hostinterface driver
set_sw_property bsp_subdirectory openmac

# source files
#none

# header files
#none

# include header file
add_sw_property include_directory include

# support HAL BSP only
add_sw_property supported_bsp_type HAL

#
# Callback
#
set_sw_property callback_source_file "tcl/openmac.tcl"
set_sw_property generation_callback generationCallback
