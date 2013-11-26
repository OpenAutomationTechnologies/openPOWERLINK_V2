#!/usr/bin/tclsh
# This file provides general functions for ipcore configuration in tcl.

namespace eval ::ipcoreUtil {
    # Define package version
    set version 0.0.1

    # Export "constants"
    variable cFalse 0
    variable cTrue  1

    # Export procedures
    namespace export logDualis
}

# This procedure calculates the log dualis of the input param.
# Note that param should be an integer
proc ::ipcoreUtil::logDualis { param } {
    # initialize climping values
    set accu    1
    set result  0

    # ceil the input parameter
    set val [expr int(ceil(${param})) ]

    while {${accu} < ${val}} {
        set accu    [expr ${accu} * 2 ]
        set result  [expr ${result} + 1 ]
    }

    return ${result}
}

package provide ipcoreUtil $ipcoreUtil::version
