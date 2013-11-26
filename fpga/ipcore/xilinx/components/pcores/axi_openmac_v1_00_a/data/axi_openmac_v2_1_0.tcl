###############################################################################
##
##    (c) B&R, 2013
##
##    Redistribution and use in source and binary forms, with or without
##    modification, are permitted provided that the following conditions
##    are met:
##
##    1. Redistributions of source code must retain the above copyright
##       notice, this list of conditions and the following disclaimer.
##
##    2. Redistributions in binary form must reproduce the above copyright
##       notice, this list of conditions and the following disclaimer in the
##       documentation and/or other materials provided with the distribution.
##
##    3. Neither the name of B&R nor the names of its
##       contributors may be used to endorse or promote products derived
##       from this software without prior written permission. For written
##       permission, please contact office@br-automation.com
##
##    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
##    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
##    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
##    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
##    COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
##    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
##    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
##    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
##    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
##    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
##    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
##    POSSIBILITY OF SUCH DAMAGE.
##
###############################################################################

###############################################################################
# PACKAGES
###############################################################################

#uses "xillib.tcl"

# Get path of this file
variable thisPath [file dirname [file normalize [info script]]]

# Insert local packages.
source "${thisPath}/../../../../../common/util/tcl/ipcoreUtil.tcl"
source "${thisPath}/../../../../../common/openmac/tcl/openmac.tcl"

# Use package ipcoreUtil for general functions...
package require ipcoreUtil 0.0.1

# Use package openmac...
package require openmac 0.0.1

# Assign constants from packages
proc cFalse         {} { return $::ipcoreUtil::cFalse     }
proc cTrue          {} { return $::ipcoreUtil::cTrue      }
proc cPktBufLocal   {} { return $::openmac::cPktBufLocal  }
proc cPktBufExtern  {} { return $::openmac::cPktBufExtern }
proc cPhyPortRmii   {} { return $::openmac::cPhyPortRmii  }
proc cPhyPortMii    {} { return $::openmac::cPhyPortMii   }

###################################################
## Internal helper functions
###################################################

# Returns the hw parameter value.
proc getParentHwParam { parent_handle param_name } {
    return [xget_hw_parameter_value [xget_hw_parent_handle $parent_handle] $param_name]
}

# Checks if the specified interface handle is connected.
proc checkConnected { busif_handle } {
    if { [xget_hw_value $busif_handle] == "" } {
        return [cFalse]
    } else {
        return [cTrue]
    }
}

###################################################
## Driver generate statement
###################################################
proc generate {drv_handle} {
    puts "###################################"
    puts "OPENMAC IP-Core found!"

    # Get parameter values to be written to header file
    set periph [xget_periphs $drv_handle]

    # Base/Span
    set regBase     [xget_param_value $periph C_S_AXI_MAC_REG_RNG0_BASEADDR]
    set regHigh     [xget_param_value $periph C_S_AXI_MAC_REG_RNG0_HIGHADDR]
    set regSpan     [expr $regHigh - $regBase + 1]
    set timerBase   [xget_param_value $periph C_S_AXI_MAC_REG_RNG1_BASEADDR]
    set timerHigh   [xget_param_value $periph C_S_AXI_MAC_REG_RNG1_HIGHADDR]
    set timerSpan   [expr $timerHigh - $timerBase + 1]
    set pktBase     [xget_param_value $periph C_S_AXI_MAC_PKT_BASEADDR]
    set pktHigh     [xget_param_value $periph C_S_AXI_MAC_PKT_HIGHADDR]
    set pktSpan     [expr $pktHigh - $pktBase + 1]

    # HDL parameters
    set phyCount    [xget_param_value $periph gPhyPortCount]
    set txBufLoc    [xget_param_value $periph gPacketBufferLocTx]
    set rxBufLoc    [xget_param_value $periph gPacketBufferLocRx]
    set tmrCount    [xget_param_value $periph gTimerCount]
    set tmrPulsEn   [xget_param_value $periph gTimerEnablePulseWidth]
    set tmrPulseWdt [xget_param_value $periph gTimerPulseRegWidth]
    set dmaObserv   [xget_param_value $periph gEnableDmaObserver]
    set pktBufSize  [expr int(pow(2, [xget_param_value $periph "gPacketBufferLog2Size"]))]

    # Construct CMACROs
    set lst_name    [list REG_BASE REG_SPAN TIMER_BASE TIMER_SPAN PKT_BASE PKT_SPAN PHYCNT DMAOBSERV PKTLOCTX PKTLOCRX PKTBUFSIZE TIMERCNT TIMERPULSE TIMERPULSEREGWIDTH]
    set lst_val     [list ${regBase} ${regSpan} ${timerBase} ${timerSpan} ${pktBase} ${pktSpan} ${phyCount} ${dmaObserv} ${txBufLoc} ${rxBufLoc} ${pktBufSize} ${tmrCount} ${tmrPulsEn} ${tmrPulseWdt}]

    # Generate header file
    set filePath "../../include"
    set fileName "openmac_cfg"
    ::openmac::genHeaderFile $filePath $fileName $lst_name $lst_val

    puts "###################################"
}

###################################################
## System level drc procedure
###################################################
proc syslevel_drc_proc { ipinst_handle } {
    return 0;
}

###################################################
## IP level drc procedure
###################################################
proc iplevel_drc_proc { ipinst_handle } {
    # Configuration
    set txBufLoc [xget_hw_parameter_value $ipinst_handle "gui_txBufLoc"]
    set rxBufLoc [xget_hw_parameter_value $ipinst_handle "gui_rxBufLoc"]
    set phyType  [xget_hw_parameter_value $ipinst_handle "gui_phyType"]

    # Interface handler
    set busif_reg [xget_hw_busif_handle $ipinst_handle "S_AXI_MAC_REG"]
    set busif_pkt [xget_hw_busif_handle $ipinst_handle "S_AXI_MAC_PKT"]
    set busif_dma [xget_hw_busif_handle $ipinst_handle "M_AXI_MAC_DMA"]
    set irqif_tmr [xget_hw_port_handle $ipinst_handle "TIMER_IRQ"]
    set irqif_mac [xget_hw_port_handle $ipinst_handle "MAC_IRQ"]
    set clkif_50  [xget_hw_port_handle $ipinst_handle "iClk50"]
    set clkif_100 [xget_hw_port_handle $ipinst_handle "iClk100"]

    # System parameters
    set clkfq_50  [xget_hw_parameter_value $ipinst_handle "C_iClk50_FREQ_HZ"]
    set clkfq_100 [xget_hw_parameter_value $ipinst_handle "C_iClk100_FREQ_HZ"]

    # Check connection of MAC REG
    if { [checkConnected $busif_reg] == [cFalse]} {
        error "The interface \"S_AXI_MAC_REG\" is not connected!" "" "mdt_error"
        return 1
    }

    # Check DMA interface depending on location
    if { $txBufLoc == [cPktBufExtern] || $rxBufLoc == [cPktBufExtern] } {
        if { [checkConnected $busif_dma] == [cFalse] } {
            error "The interface \"M_AXI_MAC_DMA\" is not connected!" "" "mdt_error"
            return 1
        }
    }

    # Check packet buffer interface depending on location
    if { $txBufLoc == [cPktBufLocal] || $rxBufLoc == [cPktBufLocal] } {
        # Check connection of MAC DMA
        if { [checkConnected $busif_pkt] == [cFalse] } {
            error "The interface \"S_AXI_MAC_PKT\" is not connected!" "" "mdt_error"
            return 1
        }
    }

    # Check timer interrupt is connected
    if { [checkConnected $irqif_tmr] == [cFalse] } {
        error "The interrupt \"TIMER_IRQ\" is not connected!" "" "mdt_error"
        return 1
    }

    # Check mac interrupt is connected
    if { [checkConnected $irqif_mac] == [cFalse] } {
        error "The interrupt \"MAC_IRQ\" is not connected!" "" "mdt_error"
        return 1
    }

    # Check if iClk50 is connected if RMII is used
    if { [checkConnected $clkif_50] == [cFalse] } {
        error "The clock \"iClk50\" is not connected! Please connect it to 50 MHz clock source!" "" "mdt_error"
        return 1
    }

    # Check if iClk50 is connected to 50 MHz
    if { $clkfq_50 != 50000000 } {
        error "The clock \"iClk50\" is not connected to 50 MHz clock source! ($clkfq_50)" "" "mdt_error"
        return 1
    }

    # Check RMII connections
    if { $phyType == [cPhyPortRmii] } {
        # Check if iClk100 is connected
        if { [checkConnected $clkif_100] == [cFalse] } {
            error "The clock \"iClk100\" is not connected! Please connect it to 100 MHz clock source!" "" "mdt_error"
            return 1
        }

        # Check if iClk100 is connected to 100 MHz
        if { $clkfq_100 != 100000000 } {
            error "The clock \"iClk100\" is not connected to 100 MHz clock source! ($clkfq_100)" "" "mdt_error"
            return 1
        }
    }

    return 0;
}

###################################################
## Parameter IP level update procedures
###################################################

# This function returns the gui parameter directly.
proc get_gPhyPortCount { param_handle } {
    return [getParentHwParam $param_handle "gui_phyCount"]
}

# This function returns the gui parameter directly.
proc get_gPhyPortType { param_handle } {
    return [getParentHwParam $param_handle "gui_phyType"]
}

# This function returns the number of phys if extra SMI is enabled, otherwise 1.
proc get_gSmiPortCount { param_handle } {
    set phyCount [getParentHwParam $param_handle "gui_phyCount"]
    set extraSmi [getParentHwParam $param_handle "gui_extraSmi"]

    if { $extraSmi != [cFalse] } {
        return $phyCount
    } else {
        return 1
    }
}

# This function always returns "little".
proc get_gEndianness { param_handle } {
    return "little"
}

# This function returns the gui parameter directly.
proc get_gEnableActivity { param_handle } {
    return [getParentHwParam $param_handle "gui_actEn"]
}

# This function returns CTRUE if any packet location is external.
proc get_gEnableDmaObserver { param_handle } {
    set txBufLoc [getParentHwParam $param_handle "gui_txBufLoc"]
    set rxBufLoc [getParentHwParam $param_handle "gui_rxBufLoc"]

    if { $txBufLoc == [cPktBufExtern] || $rxBufLoc == [cPktBufExtern] } {
        return [cTrue]
    } else {
        return [cFalse]
    }
}

# This function returns 32.
proc get_gDmaAddrWidth { param_handle } {
    return 32
}

# This function returns 32.
proc get_gDmaDataWidth { param_handle } {
    return 32
}

# The function obtains the maximum of read and write burst size and returns
# log2 of the size.
proc get_gDmaBurstCountWidth { param_handle } {
    set txBufLoc    [getParentHwParam $param_handle "gui_txBufLoc"]
    set txBurstSize [getParentHwParam $param_handle "gui_txBurstSize"]
    set rxBufLoc    [getParentHwParam $param_handle "gui_rxBufLoc"]
    set rxBurstSize [getParentHwParam $param_handle "gui_rxBurstSize"]

    return [::openmac::getBurstCountWidth $txBufLoc $txBurstSize $rxBufLoc $rxBurstSize]
}

# This function returns the gui parameter directly.
proc get_gDmaWriteBurstLength { param_handle } {
    return [getParentHwParam $param_handle "gui_rxBurstSize"]
}

# This function returns the gui parameter directly.
proc get_gDmaReadBurstLength { param_handle } {
    return [getParentHwParam $param_handle "gui_txBurstSize"]
}

# The function obtains the configured burst length and returns the necessary
# fifo length.
proc get_gDmaWriteFifoLength { param_handle } {
    set burstLength [getParentHwParam $param_handle "gui_rxBurstSize"]

    return [::openmac::getFifoLength $burstLength]
}

# The function obtains the configured burst length and returns the necessary
# fifo length.
proc get_gDmaReadFifoLength { param_handle } {
    set burstLength [getParentHwParam $param_handle "gui_txBurstSize"]

    return [::openmac::getFifoLength $burstLength]
}

# This function returns the gui parameter directly.
proc get_gPacketBufferLocTx { param_handle } {
    return [getParentHwParam $param_handle "gui_txBufLoc"]
}

# This function returns the gui parameter directly.
proc get_gPacketBufferLocRx { param_handle } {
    return [getParentHwParam $param_handle "gui_rxBufLoc"]
}

# Returns the local packet buffer size. If no local buffers are used, returns 0.
proc getPktBufSize { param_handle } {
    set txBufLoc            [getParentHwParam $param_handle "gui_txBufLoc"]
    set rxBufLoc            [getParentHwParam $param_handle "gui_rxBufLoc"]
    set txBufSize           [expr 1024 * [getParentHwParam $param_handle "gui_txBufSize"]]
    set rxBufSize           [expr 1024 * [getParentHwParam $param_handle "gui_rxBufSize"]]

    return [::openmac::getPktBufSize $txBufLoc $txBufSize $rxBufLoc $rxBufSize]
}

# This function returns the gui parameter directly.
proc get_gPacketBufferLog2Size { param_handle } {
    # The log2 value determines pktbuf address width, this value doesn't matter
    # (it avoids warnings) since packet buffer below 1k are unrealistic!
    set minLog2         4

    # Obtain packet buffer size and get log2 of it...
    set pktBufSize      [getPktBufSize $param_handle]
    set pktBufSizeLog2  [::ipcoreUtil::logDualis ${pktBufSize}]

    if { ${pktBufSizeLog2} < ${minLog2} } {
        set pktBufSizeLog2  ${minLog2}
    }

    return ${pktBufSizeLog2}
}

# This function returns the gui parameter directly.
proc get_gTimerCount { param_handle } {
    return [getParentHwParam $param_handle "gui_tmrCount"]
}

# This function returns the gui parameter directly.
proc get_gTimerEnablePulseWidth { param_handle } {
    return [getParentHwParam $param_handle "gui_tmrPulseEn"]
}

# This function returns the gui parameter directly.
proc get_gTimerPulseRegWidth { param_handle } {
    return [getParentHwParam $param_handle "gui_tmrPulseWdt"]
}

# This function returns the DMA maximum burst length
proc get_C_M_AXI_MAC_DMA_MAX_BURST_LEN { param_handle } {
    set maxBurstLen_low     16
    set maxBurstLen_high    256
    set txBufLoc            [getParentHwParam $param_handle "gui_txBufLoc"]
    set rxBufLoc            [getParentHwParam $param_handle "gui_rxBufLoc"]
    set txBurstSize         [getParentHwParam $param_handle "gui_txBurstSize"]
    set rxBurstSize         [getParentHwParam $param_handle "gui_rxBurstSize"]

    if { $txBufLoc == [cPktBufExtern] && $rxBufLoc == [cPktBufExtern] } {
        # Set maximum burst size
        if { $rxBurstSize > $txBurstSize } {
            set maxBurstLen $rxBurstSize
        } else {
            set maxBurstLen $txBurstSize
        }
    } elseif { $txBufLoc == [cPktBufExtern] } {
        # Only set txBurstSize
        set maxBurstLen $txBurstSize
    } elseif { $rxBufLoc == [cPktBufExtern] } {
        # Only set rxBurstSize
        set maxBurstLen $rxBurstSize
    } else {
        # Set minimum value (it will be optimized away anyway!)
        set maxBurstLen 0
    }

    # Clip to 16 ... 256
    if { $maxBurstLen < $maxBurstLen_low } {
        set maxBurstLen $maxBurstLen_low
    } elseif { $maxBurstLen > $maxBurstLen_high } {
        set maxBurstLen $maxBurstLen_high
    }

    return $maxBurstLen
}

# This function returns the minimum size value for MAC REG.
proc get_C_S_AXI_MAC_REG_MIN_SIZE { param_handle } {
    set macRegHigh      [getParentHwParam $param_handle "C_S_AXI_MAC_REG_RNG0_HIGHADDR"]
    set macTimerHigh    [getParentHwParam $param_handle "C_S_AXI_MAC_REG_RNG1_HIGHADDR"]

    if { $macRegHigh > $macTimerHigh } {
        return $macRegHigh
    } else {
        return $macTimerHigh
    }
}

# This function returns the minimum size value for MAC PKT.
proc get_C_S_AXI_MAC_PKT_MIN_SIZE { param_handle } {
    return [getParentHwParam $param_handle "C_S_AXI_MAC_PKT_HIGHADDR"]
}
