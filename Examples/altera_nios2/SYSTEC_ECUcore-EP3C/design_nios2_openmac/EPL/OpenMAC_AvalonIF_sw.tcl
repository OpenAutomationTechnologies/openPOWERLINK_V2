# Create a new driver
create_driver OpenMAC_driver

set_sw_property hw_class_name OpenMAC

set_sw_property version 9.1

set_sw_property min_compatible_hw_version 1.0

#set_sw_property isr_preemption_supported true
#et_sw_property supported_interrupt_apis "legacy_interrupt_api enhanced_interrupt_api"
set_sw_property supported_interrupt_apis "legacy_interrupt_api"


add_sw_property supported_bsp_type HAL

# End of file
