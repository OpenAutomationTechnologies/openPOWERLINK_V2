# ------------------------------------------------------------------------------
# SDC for TERASIC DE2I-150 JTAG
# ------------------------------------------------------------------------------

proc timing_jtag {  } {
    set_clock_groups -asynchronous -group {altera_reserved_tck}
    set_input_delay -clock {altera_reserved_tck} 20 [get_ports altera_reserved_tdi]
    set_input_delay -clock {altera_reserved_tck} 20 [get_ports altera_reserved_tms]
    set_output_delay -clock {altera_reserved_tck} 20 [get_ports altera_reserved_tdo]

    return 0
}
