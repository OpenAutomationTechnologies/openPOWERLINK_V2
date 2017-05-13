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

# Checks if the specified interface handle is connected.
proc checkConnected { pin_handle } {
    if { [get_property IS_CONNECTED $pin_handle] != 1 } {
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
    set periph [::hsm::utils::get_common_driver_ips $drv_handle]

    # Base/Span
    set regBase     [::hsm::utils::get_param_value $periph C_S_AXI_MAC_REG_RNG0_BASEADDR]
    set regHigh     [::hsm::utils::get_param_value $periph C_S_AXI_MAC_REG_RNG0_HIGHADDR]
    set regSpan     [::hsm::utils::format_address_string [expr $regHigh - $regBase + 1]]
    set timerBase   [::hsm::utils::get_param_value $periph C_S_AXI_MAC_REG_RNG1_BASEADDR]
    set timerHigh   [::hsm::utils::get_param_value $periph C_S_AXI_MAC_REG_RNG1_HIGHADDR]
    set timerSpan   [::hsm::utils::format_address_string [expr $timerHigh - $timerBase + 1]]
    set pktBase     [::hsm::utils::get_param_value $periph C_S_AXI_MAC_PKT_BASEADDR]
    set pktHigh     [::hsm::utils::get_param_value $periph C_S_AXI_MAC_PKT_HIGHADDR]
    set pktSpan     [::hsm::utils::format_address_string [expr $pktHigh - $pktBase + 1]]

    # HDL parameters
    set phyCount    [::hsm::utils::get_param_value $periph gPhyPortCount]
    set txBufLoc    [::hsm::utils::get_param_value $periph gPacketBufferLocTx]
    set rxBufLoc    [::hsm::utils::get_param_value $periph gPacketBufferLocRx]
    set tmrPulse    [::hsm::utils::get_param_value $periph gui_tmrPulse]
    set tmrPulsEn   [::hsm::utils::get_param_value $periph gTimerEnablePulseWidth]
    set tmrPulseWdt [::hsm::utils::get_param_value $periph gTimerPulseRegWidth]
    set dmaObserv   [::hsm::utils::get_param_value $periph gEnableDmaObserver]
    set pktBufSize  [expr int(pow(2, [::hsm::utils::get_param_value $periph gPacketBufferLog2Size]))]

    # Construct CMACROs
    set lst_name    [list REG_BASE REG_SPAN TIMER_BASE TIMER_SPAN PKT_BASE PKT_SPAN PHYCNT DMAOBSERV PKTLOCTX PKTLOCRX PKTBUFSIZE TIMERPULSE TIMERPULSECONTROL TIMERPULSEREGWIDTH]
    set lst_val     [list ${regBase} ${regSpan} ${timerBase} ${timerSpan} ${pktBase} ${pktSpan} ${phyCount} ${dmaObserv} ${txBufLoc} ${rxBufLoc} ${pktBufSize} ${tmrPulse} ${tmrPulsEn} ${tmrPulseWdt}]

    # Generate header file
    set filePath "../../include"
    set fileName "openmac_cfg"
    ::openmac::genHeaderFile $filePath $fileName $lst_name $lst_val

    puts "###################################"
}

###################################################
## DRC procedure
###################################################
proc drc_proc { drv_handle } {

    set periph [::hsm::utils::get_common_driver_ips $drv_handle]

    # Configuration
    set txBufLoc [::hsm::utils::get_param_value $periph gui_txBufLoc]
    set rxBufLoc [::hsm::utils::get_param_value $periph gui_rxBufLoc]
    set phyType  [::hsm::utils::get_param_value $periph gui_phyType]

    # Interface handler
    set busif_reg [get_intf_pins S_AXI_MAC_REG -of_objects $periph]
    set busif_pkt [get_intf_pins S_AXI_MAC_PKT -of_objects $periph]
    set busif_dma [get_intf_pins M_AXI_MAC_DMA -of_objects $periph]
    set irqif_tmr [get_pins TIMER_IRQ -of_objects $periph]
    set irqif_mac [get_pins MAC_IRQ -of_objects $periph]
    set clkif_50  [get_pins iClk50 -of_objects $periph]
    set clkif_100 [get_pins iClk100 -of_objects $periph]

    # Check connection of MAC REG
    if { [checkConnected $busif_reg] == [cFalse]} {
        error "The interface \"S_AXI_MAC_REG\" is not connected!"
    }

    # Check DMA interface depending on location
    if { $txBufLoc == [cPktBufExtern] || $rxBufLoc == [cPktBufExtern] } {
        if { [checkConnected $busif_dma] == [cFalse] } {
            error "The interface \"M_AXI_MAC_DMA\" is not connected!"
        }
    }

    # Check packet buffer interface depending on location
    if { $txBufLoc == [cPktBufLocal] || $rxBufLoc == [cPktBufLocal] } {
        # Check connection of MAC DMA
        if { [checkConnected $busif_pkt] == [cFalse] } {
            error "The interface \"S_AXI_MAC_PKT\" is not connected!"
        }
    }

    # Check timer interrupt is connected
    if { [checkConnected $irqif_tmr] == [cFalse] } {
        error "The interrupt \"TIMER_IRQ\" is not connected!"
    }

    # Check mac interrupt is connected
    if { [checkConnected $irqif_mac] == [cFalse] } {
        error "The interrupt \"MAC_IRQ\" is not connected!"
    }

    # Check if iClk50 is connected if RMII is used
    if { [checkConnected $clkif_50] == [cFalse] } {
        error "The clock \"iClk50\" is not connected! Please connect it to 50 MHz clock source!"
    }

    # Check if iClk50 is connected to 50 MHz
    # set clkfq_50  [::hsm::utils::get_clk_pin_freq $periph iClk50]
    set clkfq_50 [get_property CLK_FREQ $clkif_50]
    if { $clkfq_50 != 50000000 } {
        error "The clock \"iClk50\" is not connected to 50 MHz clock source! ($clkfq_50)"
    }

    # Check RMII connections
    if { $phyType == [cPhyPortRmii] } {
        # Check if iClk100 is connected
        if { [checkConnected $clkif_100] == [cFalse] } {
            error "The clock \"iClk100\" is not connected! Please connect it to 100 MHz clock source!"
        }

        # Check if iClk100 is connected to 100 MHz
        # set clkfq_100 [::hsm::utils::get_clk_pin_freq $periph iClk100]
        set clkfq_100 [get_property CLK_FREQ $clkif_100]
        if { $clkfq_100 != 100000000 } {
            error "The clock \"iClk100\" is not connected to 100 MHz clock source! ($clkfq_100)"
        }
    }
}
