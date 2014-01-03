#!/usr/bin/tclsh
# This file provides general functions to be used for Sopc/Qsys components.

namespace eval ::qsysUtil:: {
    # Define package version
    set version 0.0.1

    # Export procedures
    namespace export addHdlParam
    namespace export addGuiParam
    namespace export addSysParam
}

# This procedure adds an HDL parameter.
proc ::qsysUtil::addHdlParam { name type default visible } {
    add_parameter           ${name} ${type}         ${default}
    set_parameter_property  ${name} DERIVED         TRUE
    set_parameter_property  ${name} HDL_PARAMETER   TRUE
    set_parameter_property  ${name} VISIBLE         ${visible}
}

# This procedure adds a GUI parameter.
proc ::qsysUtil::addGuiParam { name type default display units range } {
    add_parameter           ${name} ${type}         ${default}
    set_parameter_property  ${name} DISPLAY_NAME    ${display}
    set_parameter_property  ${name} DISPLAY_UNITS   ${units}
    set_parameter_property  ${name} ALLOWED_RANGES  ${range}
}

# This procedure adds a SYSTEM INFO parameter
proc ::qsysUtil::addSysParam { name type default sysInfo visible } {
    add_parameter           ${name} ${type}         ${default}
    set_parameter_property  ${name} SYSTEM_INFO     ${sysInfo}
    set_parameter_property  ${name} VISIBLE         ${visible}
}

package provide qsysUtil $qsysUtil::version
