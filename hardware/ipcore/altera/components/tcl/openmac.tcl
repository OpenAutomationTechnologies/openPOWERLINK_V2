#!/usr/bin/tclsh
# This file provides the generation callback invoked by nios2-bsp generator.

# Generation callback for nios2-bsp
proc generationCallback { instName tgtDir bspDir } {
    # Get path of this file
    set thisFileLoc [pwd]
    # File name of generate header file
    set fileName    "openmac_cfg"
    # Path of generate header file
    set filePath    $tgtDir
    # List of cmacros set in sopcinfo
    set lst_name    [list PHYCNT DMAOBSERV PKTLOCTX PKTLOCRX PKTBUFSIZE TIMERPULSE TIMERPULSECONTROL TIMERPULSEREGWIDTH]

    # Get writeFile_* functions by relative path
    source "${thisFileLoc}/../../../common/util/tcl/writeFile.tcl"

    # Insert local packages.
    source "${thisFileLoc}/../../../common/util/tcl/ipcoreUtil.tcl"
    source "${thisFileLoc}/../../../common/openmac/tcl/openmac.tcl"

    # Use package openmac...
    package require openmac 0.0.1

    puts ""
    puts "***********************************************************"
    puts ""
    puts " Create ${fileName}.h with settings from ${instName} ..."
    puts ""

    # Get values of cmacros set in sopcinfo
    set lst_val [getCmacroValues $lst_name]

    # Get base/span of openmac macReg and macTimer
    set macRegBase      [format 0x%X [get_base_addr ${instName}_macReg]]
    set macRegSpan      [get_addr_span ${instName}_macReg]
    set macTimerBase    [format 0x%X [get_base_addr ${instName}_macTimer]]
    set macTimerSpan    [get_addr_span ${instName}_macTimer]

    # Handle no packet buffer used
    set txLoc [get_module_assignment embeddedsw.CMacro.PKTLOCTX]
    set rxLoc [get_module_assignment embeddedsw.CMacro.PKTLOCRX]
    if { $txLoc == ${::openmac::cPktBufLocal} || $rxLoc == ${::openmac::cPktBufLocal} } {
        set pktBufBase  [format 0x%X [get_base_addr ${instName}_pktBuf]]
        set pktBufSpan  [get_addr_span ${instName}_pktBuf]
    } else {
        set pktBufBase -1
        set pktBufSpan 0
    }

    set lst_name    [concat REG_BASE REG_SPAN TIMER_BASE TIMER_SPAN PKT_BASE PKT_SPAN $lst_name]
    set lst_val     [concat $macRegBase $macRegSpan $macTimerBase $macTimerSpan $pktBufBase $pktBufSpan $lst_val]

    # Now generate the header file
    ::openmac::genHeaderFile $filePath $fileName $lst_name $lst_val

    puts "***********************************************************"
    puts ""

}

# This procedure gets the values of a list of cmacros
proc getCmacroValues { listCmacros } {
    set tmp ""
    foreach cmacro $listCmacros {
        set tmp [concat $tmp [get_module_assignment embeddedsw.CMacro.$cmacro]]
    }

    return $tmp
}
