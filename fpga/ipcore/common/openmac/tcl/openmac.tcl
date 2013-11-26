#!/usr/bin/tclsh
# This file provides configuration function for openMAC ipcore.

# Use package ipcoreUtil for general functions...
package require ipcoreUtil 0.0.1

namespace eval ::openmac:: {
    # Define package version
    set version 0.0.1

    # Export "constants"
    variable cPktBufLocal   1
    variable cPktBufExtern  2
    variable cPhyPortRmii   1
    variable cPhyPortMii    2

    # Export procedures
    namespace export getPktBufSize
    namespace export getBurstCountWidth
    namespace export getFifoLength
    namespace export genHeaderFile
}

# Returns the local packet buffer size. If no local buffers are used, returns 0.
proc ::openmac::getPktBufSize { txBufLoc txBufSize rxBufLoc rxBufSize } {
    # Total local packet buffer size
    set pktBufSize 0

    if { ${txBufLoc} == $::openmac::cPktBufLocal } {
        set pktBufSize [expr ${pktBufSize} + ${txBufSize} ]
    }

    if { ${rxBufLoc} == $::openmac::cPktBufLocal } {
        set pktBufSize [expr ${pktBufSize} + ${rxBufSize} ]
    }

    return ${pktBufSize}
}

# Returns the burst count width. If no external buffers are used, returns 1.
proc ::openmac::getBurstCountWidth { txBufLoc txBurstSize rxBufLoc rxBurstSize } {
    # First set burst size of locals to zero.
    if { ${txBufLoc} != $::openmac::cPktBufExtern } {
        set txBurstSize     0
    }

    if { ${rxBufLoc} != $::openmac::cPktBufExtern } {
        set rxBurstSize     0
    }

    # Find burst count width, get maximum and log2 + 1
    if { ${txBurstSize} > ${rxBurstSize} } {
        set maxBurstSize    ${txBurstSize}
    } else {
        set maxBurstSize    ${rxBurstSize}
    }

    set burstCountWidth [expr [ipcoreUtil::logDualis ${maxBurstSize}] + 1]

    return ${burstCountWidth}
}

# Returns the fifo length depending on the burst length.
proc ::openmac::getFifoLength { burstLength } {
    # Minimum fifo length:
    set fifoLength  16

    # Size of max transfers stored
    set maxTransfer [expr ${burstLength} * 2 ]

    if { ${maxTransfer} > ${fifoLength} } {
        set fifoLength  ${maxTransfer}
    }

    return ${fifoLength}
}

# Generates the openmac header file.
proc ::openmac::genHeaderFile { filePath fileName lst_name lst_val } {
    set fp      [open "${filePath}/${fileName}.h" "w"]
    set prefix  "OPENMAC"

    # Start with header
    puts $fp "//--------------------------------------------------------------"
    puts $fp "// Note that this header file is generated!"
    puts $fp "// Don't try to change settings here!"
    puts $fp "//--------------------------------------------------------------"
    puts $fp "#ifndef __INC_${fileName}_H__"
    puts $fp "#define __INC_${fileName}_H__"
    puts $fp ""

    # write "constants"
    puts $fp "//--------------------------------------------------------------"
    puts $fp "// Constants"
    puts $fp "#define ${prefix}_PKTBUF_LOCAL ${::openmac::cPktBufLocal}"
    puts $fp "#define ${prefix}_PKTBUF_EXTERN ${::openmac::cPktBufExtern}"
    puts $fp "#define ${prefix}_PHYPORT_RMII ${::openmac::cPhyPortRmii}"
    puts $fp "#define ${prefix}_PHYPORT_MII ${::openmac::cPhyPortMii}"
    puts $fp "//--------------------------------------------------------------"
    puts $fp ""

    # generate cmacros
    puts $fp "//--------------------------------------------------------------"
    puts $fp "// Configuration"
    foreach name $lst_name val $lst_val {
        puts $fp "#define ${prefix}_${name} ${val}"
    }
    puts $fp "//--------------------------------------------------------------"
    puts $fp ""

    # add offsets to RAM, MII, IRQ and DOB
    puts $fp "//--------------------------------------------------------------"
    puts $fp "// Offsets within MAC REG"
    puts $fp "#define ${prefix}_RAM_BASE    (${prefix}_REG_BASE + 0x0800)"
    puts $fp "#define ${prefix}_PHY_BASE    (${prefix}_REG_BASE + 0x1000)"
    puts $fp "#define ${prefix}_IRQ_BASE    (${prefix}_REG_BASE + 0x1010)"
    puts $fp "#define ${prefix}_DOB_BASE    (${prefix}_REG_BASE + 0x1020)"
    puts $fp "//--------------------------------------------------------------"
    puts $fp ""

    # Stop with the header end stuff
    puts $fp "#endif /*__INC_${fileName}_H__*/"

    close $fp
}

package provide openmac $openmac::version
