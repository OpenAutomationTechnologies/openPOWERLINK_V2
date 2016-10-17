###############################################################################
##
##    (c) B&R, 2016
##    (c) Weidmueller Interface GmbH & Co. KG, 2016
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
##    3. Neither the name of the copyright holders nor the names of its
##       contributors may be used to endorse or promote products derived
##       from this software without specific prior written permission.
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

# Returns the next power of 2 value
# Use built-in log functions to avoid hang with invalid input values.
# Use wide instead of int avoiding integer overflow error.
proc getNextPowerOfTwo { value } {
    return [expr wide(pow(2, round(log($value)/log(2.0))))]
}

###################################################
## Parameter IP level update procedures
###################################################

# This function returns the number of phys if extra SMI is enabled, otherwise 1.
proc get_gSmiPortCount {phyCount extraSmi} {
    if { $extraSmi != [cFalse] } {
        return $phyCount
    } else {
        return 1
    }
}

# This function returns CTRUE if any packet location is external.
proc get_gEnableDmaObserver {txBufLoc rxBufLoc} {
    if { $txBufLoc == [cPktBufExtern] || $rxBufLoc == [cPktBufExtern] } {
        return [cTrue]
    } else {
        return [cFalse]
    }
}

# This function returns the gui parameter directly.
proc get_PacketBufferLog2Size {txBufLoc txBufSize rxBufLoc rxBufSize} {
    # The log2 value determines pktbuf address width, this value doesn't matter
    # (it avoids warnings) since packet buffer below 1k are unrealistic!
    set minLog2         4

    # Obtain packet buffer size and get log2 of it...
    set pktBufSize  [::openmac::getPktBufSize $txBufLoc [expr 1024 * $txBufSize] $rxBufLoc [expr 1024 * $rxBufSize]]
    set pktBufSizeLog2  [::ipcoreUtil::logDualis ${pktBufSize}]

    if { ${pktBufSizeLog2} < ${minLog2} } {
        set pktBufSizeLog2  ${minLog2}
    }

    return ${pktBufSizeLog2}
}

# This function returns the DMA maximum burst length
proc get_C_M_AXI_MAC_DMA_MAX_BURST_LEN {txBufLoc txBurstSize rxBufLoc rxBurstSize} {
    set maxBurstLen_low     16
    set maxBurstLen_high    256

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
proc get_C_S_AXI_MAC_REG_MIN_SIZE {macRegBase macTimerBase macRegHigh macTimerHigh} {

    # Get lowest base
    if { $macRegBase < $macTimerBase } {
        set base $macRegBase
    } else {
        set base $macTimerBase
    }

    # Get highest high
    if { $macRegHigh > $macTimerHigh } {
        set high $macRegHigh
    } else {
        set high $macTimerHigh
    }

    if { $base > $high } {
        return 4096
    }

    set span [expr $high - $base]

    return [expr [getNextPowerOfTwo $span] - 1]
}

# This function returns the minimum size value for MAC PKT.
proc get_C_S_AXI_MAC_PKT_MIN_SIZE {base high} {

    if { $base > $high } {
        return 4096
    }

    set span [expr $high - $base]

    return [expr [getNextPowerOfTwo $span] - 1]
}
