#------------------------------------------------------------------------------------------------------------------------
#-- POWERLINK SOPC COMPONENT
#--
#-- 	  Copyright (C) 2010 B&R
#--
#--    Redistribution and use in source and binary forms, with or without
#--    modification, are permitted provided that the following conditions
#--    are met:
#--
#--    1. Redistributions of source code must retain the above copyright
#--       notice, this list of conditions and the following disclaimer.
#--
#--    2. Redistributions in binary form must reproduce the above copyright
#--       notice, this list of conditions and the following disclaimer in the
#--       documentation and/or other materials provided with the distribution.
#--
#--    3. Neither the name of B&R nor the names of its
#--       contributors may be used to endorse or promote products derived
#--       from this software without prior written permission. For written
#--       permission, please contact office@br-automation.com
#--
#--    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#--    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#--    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#--    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#--    COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#--    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#--    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#--    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#--    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#--    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#--    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#--    POSSIBILITY OF SUCH DAMAGE.
#--
#------------------------------------------------------------------------------------------------------------------------
#-- Version History
#------------------------------------------------------------------------------------------------------------------------
#-- 2010-08-24	V0.01	zelenkaj	first generation
#-- 2010-09-13	V0.02	zelenkaj	added selection Rmii / Mii
#-- 2010-10-04  V0.03	zelenkaj	bugfix: Rmii / Mii selection was faulty
#-- 2010-10-11  V0.04	zelenkaj	changed pdi dpr size calculation
#-- 2010-10-18	V0.05	zelenkaj	added selection Big/Little Endian (pdi_par)
#--									use bidirectional data bus (pdi_par)
#-- 2010-11-15	V0.06	zelenkaj	bugfix: rpdo header was calculated twice
#-- 2010-11-22	V0.07	zelenkaj	Added 2 GPIO signals to parallel interface
#--									Added Operational Flag to simple I/O interface
#--									Omitted T/RPDO descriptor sections in DPR
#--									Added ability to verify connected clock rates (to clkEth and clk50meg)
#--									Added generic to set duration of valid assertion (portio)
#-- 2010-11-29	V0.08	zelenkaj	Changed several Endianness sel. to one for AP
#--									Allocation of ping-pong tx buffers (necessary by openPOWERLINK stack)
#-- 2010-11-30	V0.09	zelenkaj	Added other picture as Block Diagram (3 design approaches)
#-- 2010-12-06	V0.10	zelenkaj	Changed Cmacros
#--									Added openMAC only parameter
#--									Added SPI IRQ active high/low choice
#-- 2010-12-07	V0.11	zelenkaj	Bugfix: AP IRQ was generated incorrect for SPI
#--									Code clean up
#-- 2011-01-25	V0.12	zelenkaj	Added generic internal/external packet storage
#-- 2011-02-24	V0.13	zelenkaj	Bugfix: openMAC only with RMII generates division by zero
#--									minor changes (naming conventions Mii->SMI)
#-- 2011-03-14	V0.14	zelenkaj	Added generic for packet storage (RX int/ext)
#-- 2011-03-21	V0.15	zelenkaj	bugfix: packet buffer padding wasn't considered
#-- 2011-03-28	V0.20	zelenkaj	Added: LED
#--									Added: Events
#--									Added/Changed: Asynchronous buffer 2x Ping-Pong
#-- 2011-04-04	V0.21	zelenkaj	minor: led_status is the official name
#--									minor: parallel interface uses ack instead of ready
#-- 2011-04-26	V0.22	zelenkaj	prepared for pdi clock domain configuration, but not allowed to change by SOPC
#-- 2011-04-28	V0.23	zelenkaj	second cmp timer of openMAC is optinal by generic
#--									added link to IP-core documentation
#--									added description to parameters (shown in SOPC GUI)
#-- 2011-05-06	V0.24	zelenkaj	some naming convention changes
#--									bug fix: use the RX_ER signal, it has important meaning!
#-- 2011-05-09  V0.30	zelenkaj	Hardware Acceleration (HW ACC) added.
#-- 2011-06-06	V0.31	zelenkaj	PDI status/control register enhanced by 8 bytes
#-- 2011-06-20	V0.32	zelenkaj	RPDO size is set once for all
#--									big/little endian option forwarded to system.h only, not to vhdl!
#-- 2011-07-23	V0.33	zelenkaj	added RXERR for RMII
#-- 2011-07-25	V0.34	zelenkaj	LED gadget and asynchronous buffer optional, reset of pdi revision
#-- 2011-08-08	V0.35	zelenkaj	LED gadget enhancement -> added 8 general purpose outputs
#-- 2011-08-02	V1.00	zelenkaj	exchanged Avalon interface with entity openMAC_Ethernet
#-- 2011-09-05	V1.01	zelenkaj	PDI SPI async Irq low/high active was not terminated
#-- 2011-09-06	V1.02	zelenkaj	async-buffer limitation is deactivated
#-- 2011-09-14	V1.03	zelenkaj	extract of components into own files
#-- 2011-10-10	V1.04	zelenkaj	async-buffer limitation fixed again..
#-- 2011-10-13	V1.05	zelenkaj	file names changed..
#-- 2011-10-14	V1.06	zelenkaj	rmii2mii fifos are deleted (dma fifo is abused for..)
#-- 2011-11-07	V1.07	zelenkaj	added generic for dma master qualifiers
#-- 2011-11-17	V1.08	zelenkaj	pdi dpr vhd-file renamed
#-- 2011-11-21	V1.09	zelenkaj	added time synchronization feature
#-- 2011-11-28	V1.10	zelenkaj	added waitrequest signals to pdi pcp/ap
#-- 2011-11-29	V1.11	zelenkaj	event feature is optional
#-- 2011-11-30	V1.12	zelenkaj	Added generic for DMA observer
#-- 2011-12-12	V1.13	zelenkaj	Changed packet location enumerator
#-- 2011-12-14	V1.14	zelenkaj	Changed documentation path/filename
#-- 2011-12-15	V1.15	zelenkaj	Changed openMAC only RX buffer configuration
#-- 2012-01-04	V1.16	zelenkaj	Added feature to create mif files for openMAC DPR and PDI DPR
#-- 2012-01-09  V1.17   zelenkaj    Added ap_syncIrq for external AP
#-- 2012-01-11	V1.18	zelenkaj	Async Irq is omitted if event hw support is disabled
#-- 2012-01-12	V1.19	zelenkaj	Added macro to system.h in case of low-jitter SYNC
#-- 2012-01-25	V1.20	zelenkaj	Added special initialization to pdi_dpr.mif
#-- 2012-01-26	V1.21	zelenkaj    Added generic for SMI generation and one SMI ports
#--                                 Renamed label for SYNC IRQ feature
#--                                 Added "expert mode" for the advanced users
#--                                 Omit hwacc options, since we are fast enough!
#--                                 Minor delete of system.h parameter
#-- 2012-01-27  V1.30   zelenkaj    Incremented PdiRev
#-- 2012-01-31  V1.31   zelenkaj    moved hw event support into "exper mode"
#--                                 fixed expert mode for dma observer
#------------------------------------------------------------------------------------------------------------------------

package require -exact sopc 10.1

set_module_property DESCRIPTION "POWERLINK IP-core"
set_module_property NAME powerlink
set_module_property VERSION 2.0
set_module_property INTERNAL false
set_module_property GROUP POWERLINK
set_module_property AUTHOR "Michael Hogger and Joerg Zelenka"
set_module_property DISPLAY_NAME "POWERLINK"
set_module_property TOP_LEVEL_HDL_FILE powerlink.vhd
set_module_property TOP_LEVEL_HDL_MODULE powerlink
set_module_property INSTANTIATE_IN_SYSTEM_MODULE true
set_module_property EDITABLE FALSE
set_module_property ANALYZE_HDL TRUE
set_module_property ICON_PATH img/br.png
add_documentation_link "POWERLINK IP-Core Documentation" "doc/POWERLINK-IP-Core_Altera.pdf"

#files
add_file src/powerlink.vhd {SYNTHESIS SIMULATION}
add_file src/pdi.vhd {SYNTHESIS SIMULATION}
add_file src/pdi_par.vhd {SYNTHESIS SIMULATION}
add_file src/pdi_dpr_Altera.vhd {SYNTHESIS SIMULATION}
add_file src/pdi_tripleVBufLogic.vhd {SYNTHESIS SIMULATION}
add_file src/pdi_apIrqGen.vhd {SYNTHESIS SIMULATION}
add_file src/pdi_controlStatusReg.vhd {SYNTHESIS SIMULATION}
add_file src/pdi_event.vhd {SYNTHESIS SIMULATION}
add_file src/pdi_led.vhd {SYNTHESIS SIMULATION}
add_file src/pdi_simpleReg.vhd {SYNTHESIS SIMULATION}
add_file src/OpenFILTER.vhd {SYNTHESIS SIMULATION}
add_file src/OpenHUB.vhd {SYNTHESIS SIMULATION}
add_file src/OpenMAC.vhd {SYNTHESIS SIMULATION}
add_file src/openMAC_Ethernet.vhd {SYNTHESIS SIMULATION}
add_file src/openMAC_cmp.vhd {SYNTHESIS SIMULATION}
add_file src/openMAC_phyAct.vhd {SYNTHESIS SIMULATION}
add_file src/OpenMAC_DPR_Altera.vhd {SYNTHESIS SIMULATION}
add_file src/OpenMAC_DMAFifo_Altera.vhd {SYNTHESIS SIMULATION}
add_file src/OpenMAC_DMAmaster.vhd {SYNTHESIS SIMULATION}
add_file src/OpenMAC_DMAmaster/dma_handler.vhd {SYNTHESIS SIMULATION}
add_file src/OpenMAC_DMAmaster/master_handler.vhd {SYNTHESIS SIMULATION}
add_file src/OpenMAC_PHYMI.vhd {SYNTHESIS SIMULATION}
add_file src/OpenMAC_rmii2mii.vhd {SYNTHESIS SIMULATION}
add_file src/portio.vhd {SYNTHESIS SIMULATION}
add_file src/portio_cnt.vhd {SYNTHESIS SIMULATION}
add_file src/spi.vhd {SYNTHESIS SIMULATION}
add_file src/spi_sreg.vhd {SYNTHESIS SIMULATION}
add_file src/pdi_spi.vhd {SYNTHESIS SIMULATION}
add_file src/lib/addr_decoder.vhd {SYNTHESIS SIMULATION}
add_file src/lib/edgedet.vhd {SYNTHESIS SIMULATION}
add_file src/lib/req_ack.vhd {SYNTHESIS SIMULATION}
add_file src/lib/sync.vhd {SYNTHESIS SIMULATION}
add_file src/lib/slow2fastSync.vhd {SYNTHESIS SIMULATION}
add_file src/lib/memMap.vhd {SYNTHESIS SIMULATION}

#callbacks
set_module_property VALIDATION_CALLBACK my_validation_callback
set_module_property ELABORATION_CALLBACK my_elaboration_callback

#FPGA REVISION
add_parameter iPdiRev_g INTEGER 0x0000
set_parameter_property iPdiRev_g HDL_PARAMETER true
set_parameter_property iPdiRev_g VISIBLE false
set_parameter_property iPdiRev_g DERIVED TRUE

#parameters
add_parameter expertMode BOOLEAN false
set_parameter_property expertMode DISPLAY_NAME "Enable Expert Mode"
set_parameter_property expertMode DESCRIPTION "The \"Expert Mode\" activates settings of the IP-Core for advanced users."

add_parameter clkRateEth INTEGER 0
set_parameter_property clkRateEth SYSTEM_INFO {CLOCK_RATE clkEth}
set_parameter_property clkRateEth VISIBLE false

add_parameter clkRate50 INTEGER 0
set_parameter_property clkRate50 SYSTEM_INFO {CLOCK_RATE clk50meg}
set_parameter_property clkRate50 VISIBLE false

add_parameter clkRatePcp INTEGER 0
set_parameter_property clkRatePcp SYSTEM_INFO {CLOCK_RATE pcp_clk}
set_parameter_property clkRatePcp VISIBLE false

add_parameter configPowerlink STRING "CN with Processor Interface"
set_parameter_property configPowerlink DISPLAY_NAME "POWERLINK Slave Design Configuration"
set_parameter_property configPowerlink ALLOWED_RANGES {"Direct I/O CN" "CN with Processor Interface" "openMAC only"}
set_parameter_property configPowerlink DISPLAY_HINT radio
set_parameter_property configPowerlink DESCRIPTION "The \"POWERLINK Slave Design Configuration\" allows selecting one of the three design approaches. \"Direct I/O CN\" generates a POWERLINK Slave with a 32 bit I/O port. \"CN with Processor Interface\" enables the possibility of adding an application processor (AP) to the design. The third choice \"openMAC only\" disables the generation of the Process Data Interface (PDI). All three selections generate the MAC-layer (openMAC)."

add_parameter configApInterface STRING "Avalon"
set_parameter_property configApInterface VISIBLE true
set_parameter_property configApInterface DISPLAY_NAME "Interface to AP"
set_parameter_property configApInterface ALLOWED_RANGES {"Avalon" "Parallel" "SPI"}
set_parameter_property configApInterface DISPLAY_HINT radio
set_parameter_property configApInterface DESCRIPTION "The \"Interface to AP\" selection allows to chose between three possibilities for connection the Application Processor (AP). \"Avalon\" generates an Avalon Memory Mapped Slave for the AP to connect to the PDI (this approach is intended for a Nios II AP). The \"Parallel\" choice generates an asynchronous parallel address-data-interface, which can be used to connect an FPGA-external Application Processor. The third selection \"SPI\" introduces a serial connection to an FPGA-external device, which must be an SPI master."

add_parameter configApParallelInterface STRING "8bit"
set_parameter_property configApParallelInterface VISIBLE false
set_parameter_property configApParallelInterface DISPLAY_NAME "Size of Parallel Interface to AP"
set_parameter_property configApParallelInterface ALLOWED_RANGES {"8bit" "16bit"}
set_parameter_property configApParallelInterface DESCRIPTION "The \"Size of Parallel Interface to AP\" selects the parallel interface data width. It is possible to set 8 or 16 bit, depending on your requirements."

add_parameter configApParSigs STRING "High Active"
set_parameter_property configApParSigs VISIBLE false
set_parameter_property configApParSigs DISPLAY_NAME "Active State of Control Signal (Cs, Wr, Rd and Be)"
set_parameter_property configApParSigs ALLOWED_RANGES {"High Active" "Low Active"}
set_parameter_property configApParSigs DESCRIPTION "Set the active states of the parallel interface control signals. Optionally you can add inverters in the top-level design which instantiates the SOPC block."

add_parameter configApParOutSigs STRING "High Active"
set_parameter_property configApParOutSigs VISIBLE false
set_parameter_property configApParOutSigs DISPLAY_NAME "Active State of Output Signals (Irq and Ack)"
set_parameter_property configApParOutSigs ALLOWED_RANGES {"High Active" "Low Active"}
set_parameter_property configApParOutSigs DESCRIPTION "Set the active states of the parallel interface output signals. Optionally you can add inverters in the top-level design which instantiates the SOPC block."

add_parameter configApEndian STRING "Little"
set_parameter_property configApEndian VISIBLE false
set_parameter_property configApEndian DISPLAY_NAME "Endianness of AP"
set_parameter_property configApEndian ALLOWED_RANGES {"Little" "Big"}

add_parameter configApSpi_CPOL STRING "0"
set_parameter_property configApSpi_CPOL VISIBLE false
set_parameter_property configApSpi_CPOL DISPLAY_NAME "SPI CPOL"
set_parameter_property configApSpi_CPOL ALLOWED_RANGES {"0" "1"}

add_parameter configApSpi_CPHA STRING "0"
set_parameter_property configApSpi_CPHA VISIBLE false
set_parameter_property configApSpi_CPHA DISPLAY_NAME "SPI CPHA"
set_parameter_property configApSpi_CPHA ALLOWED_RANGES {"0" "1"}

add_parameter configApSpi_IRQ STRING "High Active"
set_parameter_property configApSpi_IRQ VISIBLE false
set_parameter_property configApSpi_IRQ DISPLAY_NAME "Active State of Output Signals (Irq)"
set_parameter_property configApSpi_IRQ ALLOWED_RANGES {"High Active" "Low Active"}
set_parameter_property configApSpi_IRQ DESCRIPTION "Set the active states of SPI link's interrupt signal (Irq). Optionally you can add an inverter in the top-level design which instantiates the SOPC block."

add_parameter rpdoNum INTEGER 3
set_parameter_property rpdoNum ALLOWED_RANGES {1 2 3}
set_parameter_property rpdoNum DISPLAY_NAME "Number of RPDO Buffers"
set_parameter_property rpdoNum DESCRIPTION "This parameter sets the maximum RPDO channels of your POWERLINK Slave."

add_parameter rpdo0size INTEGER 1
set_parameter_property rpdo0size ALLOWED_RANGES 1:1490
set_parameter_property rpdo0size UNITS bytes
set_parameter_property rpdo0size DISPLAY_NAME "RPDO Buffer Size"
set_parameter_property rpdo0size DESCRIPTION "The RPDO Buffer Size is the data size limit of each individual RPDO channel."

add_parameter tpdoNum INTEGER 1
set_parameter_property tpdoNum ALLOWED_RANGES 1
set_parameter_property tpdoNum DISPLAY_NAME "Number of TPDO Buffers"
set_parameter_property tpdoNum DESCRIPTION "This parameter sets the maximum TPDO channels of your POWERLINK Slave. Note: The design is limited to one TPDO."

add_parameter tpdo0size INTEGER 1
set_parameter_property tpdo0size ALLOWED_RANGES 1:1490
set_parameter_property tpdo0size UNITS bytes
set_parameter_property tpdo0size DISPLAY_NAME "TPDO Buffer Size"
set_parameter_property tpdo0size DESCRIPTION "The TPDO Buffer Size is the data size limit of each individual TPDO channel."

add_parameter genLedGadget BOOLEAN true
set_parameter_property genLedGadget DISPLAY_NAME "Enable LED outputs"
set_parameter_property genLedGadget DESCRIPTION "The POWERLINK Slave provides an optional LED output port."

add_parameter genEvent BOOLEAN true
set_parameter_property genEvent DISPLAY_NAME "Enable Event Hardware Support"
set_parameter_property genEvent DESCRIPTION "The POWERLINK Slave provides hardware resources for immediate event handling."

add_parameter asyncBuf1Size INTEGER 1514
set_parameter_property asyncBuf1Size ALLOWED_RANGES 20:2044
set_parameter_property asyncBuf1Size UNITS bytes
set_parameter_property asyncBuf1Size DISPLAY_NAME "Asynchronous Buffer Nr. 1 Size"
set_parameter_property asyncBuf1Size DESCRIPTION "The Asynchronous Buffers are used for communication and asynchronous data transfer between PCP and AP. (Asynchronous Buffer Nr. 1 is mandatory)"

add_parameter asyncBuf2Size INTEGER 1514
set_parameter_property asyncBuf1Size ALLOWED_RANGES 0:2044
set_parameter_property asyncBuf2Size UNITS bytes
set_parameter_property asyncBuf2Size DISPLAY_NAME "Asynchronous Buffer Nr. 2 Size"
set_parameter_property asyncBuf2Size DESCRIPTION "The Asynchronous Buffers are used for communication and asynchronous data transfer between PCP and AP."

add_parameter phyIF STRING "RMII"
set_parameter_property phyIF VISIBLE true
set_parameter_property phyIF DISPLAY_NAME "Ethernet Phy Interface"
set_parameter_property phyIF ALLOWED_RANGES {"RMII" "MII"}
set_parameter_property phyIF DISPLAY_HINT radio
set_parameter_property phyIF DESCRIPTION "The \"Ethernet Phy Interface\" depends on the used Ethernet Phy ICs on your PCB. Note: Prefer RMII (Reduced Media Independent Interface, slave mode) since the resource utilization within the FPGA is a minimum. MII (Media Independent Interface) introduces extra Logic Elements (LE)."

add_parameter packetLoc STRING "TX and RX into DPRAM"
set_parameter_property packetLoc VISIBLE true
set_parameter_property packetLoc DISPLAY_NAME "Packet Buffer Location"
set_parameter_property packetLoc ALLOWED_RANGES {"TX and RX into DPRAM" "TX into DPRAM and RX over Avalon Master" "TX and RX over Avalon Master"}
set_parameter_property packetLoc DISPLAY_HINT radio
set_parameter_property packetLoc DESCRIPTION "The \"Packet Buffer Location\" is the most important setting for your POWERLINK Slave. \"TX and RX into DPRAM\" instantiates a dual ported RAM into the FPGA with the appropriate size of your settings. Prefer this choice if you have enough memory blocks (M9K) available. The setting \"TX into DPRAM and RX over Avalon Master\" places TX buffers of openMAC in the dual ported RAM, however, the RX buffers are stored in the heap of your Nios II system. Take this solution if you have a limited amount of memory blocks (M9K) and your heap is located in high-latency memory. The third selection \"TX and RX over Avalon Master\" locates TX and RX packets in your heap. Use this choice only if your heap is located in low-latency memory (SRAM 10 ns) and you are very restricted by memory block (M9K) availability."

add_parameter validSet INTEGER "1"
set_parameter_property validSet VISIBLE false
set_parameter_property validSet ALLOWED_RANGES 1:128
set_parameter_property validSet DISPLAY_NAME "Valid signal set Clock Cycles"
set_parameter_property validSet DESCRIPTION "The \"Direct I/O\" Slave provides a pulse if the output data is valid. The pulse length can be set to a multiple of the system clock."

add_parameter validAssertDuration STRING "1000"
set_parameter_property validAssertDuration VISIBLE false
set_parameter_property validAssertDuration DISPLAY_NAME "Implemented Valid signal set duration (Clock Cycles x Clock Period)"
set_parameter_property validAssertDuration DISPLAY_UNITS "ns"
set_parameter_property validAssertDuration ENABLED false
set_parameter_property validAssertDuration DERIVED TRUE

add_parameter macTxBuf INTEGER 1514
set_parameter_property macTxBuf UNITS bytes
set_parameter_property macTxBuf DISPLAY_NAME "openMAC TX Buffer Size"
set_parameter_property macTxBuf DESCRIPTION "If \"openMAC only\" is selected, the MAC buffer size has to be set manually."

add_parameter macRxBuf INTEGER 16
set_parameter_property macRxBuf ALLOWED_RANGES 1:16
set_parameter_property macRxBuf DISPLAY_NAME "openMAC Number RX Buffers (MTU = 1500 byte)"
set_parameter_property macRxBuf DESCRIPTION "If \"openMAC only\" is selected, the number of MAC buffers has to be set manually (MTU = 1500 byte)."

add_parameter hwSupportSyncIrq BOOLEAN FALSE
set_parameter_property hwSupportSyncIrq VISIBLE true
set_parameter_property hwSupportSyncIrq DISPLAY_NAME "Use low-jitter SYNC IRQ with SoC timestamps for AP synchronization"
set_parameter_property hwSupportSyncIrq DESCRIPTION "The Application Processor (AP) is synchronized to the POWERLINK cycles. In order to reduce FPGA-resource consumption you can disable the low-jitter SYNC interrupt if your application does not require low-jitter synchronization."

add_parameter mac2phys BOOLEAN TRUE
set_parameter_property mac2phys VISIBLE true
set_parameter_property mac2phys DISPLAY_NAME "Enable second Ethernet Phy Interface"
set_parameter_property mac2phys DESCRIPTION "The POWERLINK Slave allows a second Ethernet interface for flexible network topologies by using the FPGA-internal openHUB IP-core. Enable the option if you want to connect a second phy to the FPGA."

add_parameter macGen2ndSmi BOOLEAN TRUE
set_parameter_property macGen2ndSmi VISIBLE true
set_parameter_property macGen2ndSmi DISPLAY_NAME "Enable second Phy Serial Management Interface (SMI)"
set_parameter_property macGen2ndSmi DESCRIPTION "The POWERLINK Slave allows a second Serial Management Interface, which is used for the Ethernet Phy configuration. If the two available Phys share one SMI, disable this setting!"

add_parameter macTxBurstSize INTEGER 4
set_parameter_property macTxBurstSize ALLOWED_RANGES {1 4 8 16}
set_parameter_property macTxBurstSize DISPLAY_NAME "Number of Words per DMA Read Transfer (TX direction)"
set_parameter_property macTxBurstSize DESCRIPTION "Sets the number of words (2 bytes) for each openMAC DMA read transfer (TX direction). A value of 1 refers to single beat transfers and disables burst transfers."

add_parameter macRxBurstSize INTEGER 4
set_parameter_property macRxBurstSize ALLOWED_RANGES {1 4 8 16}
set_parameter_property macRxBurstSize DISPLAY_NAME "Number of Words per DMA Write Transfer (RX direction)"
set_parameter_property macRxBurstSize DESCRIPTION "Sets the number of words (2 bytes) for each openMAC DMA write transfer (RX direction). A value of 1 refers to single beat transfers and disables burst transfers."

add_parameter enDmaObserver BOOLEAN false
set_parameter_property enDmaObserver DISPLAY_NAME "Enable packet DMA transfer monitor circuit"
set_parameter_property enDmaObserver DESCRIPTION "The DMA monitor verifies the error-free Ethernet packet data transfer to/from the memory."

#parameters for PDI HDL
add_parameter genOnePdiClkDomain_g BOOLEAN false
set_parameter_property genOnePdiClkDomain_g HDL_PARAMETER true
set_parameter_property genOnePdiClkDomain_g VISIBLE false
set_parameter_property genOnePdiClkDomain_g DERIVED TRUE

add_parameter genPdi_g BOOLEAN true
set_parameter_property genPdi_g HDL_PARAMETER true
set_parameter_property genPdi_g VISIBLE false
set_parameter_property genPdi_g DERIVED TRUE

add_parameter genInternalAp_g BOOLEAN true
set_parameter_property genInternalAp_g HDL_PARAMETER true
set_parameter_property genInternalAp_g VISIBLE false
set_parameter_property genInternalAp_g DERIVED TRUE

add_parameter genSimpleIO_g BOOLEAN false
set_parameter_property genSimpleIO_g HDL_PARAMETER true
set_parameter_property genSimpleIO_g VISIBLE false
set_parameter_property genSimpleIO_g DERIVED TRUE

add_parameter genSpiAp_g BOOLEAN false
set_parameter_property genSpiAp_g HDL_PARAMETER true
set_parameter_property genSpiAp_g VISIBLE false
set_parameter_property genSpiAp_g DERIVED TRUE

add_parameter genABuf1_g BOOLEAN true
set_parameter_property genABuf1_g HDL_PARAMETER true
set_parameter_property genABuf1_g VISIBLE false
set_parameter_property genABuf1_g DERIVED TRUE

add_parameter genABuf2_g BOOLEAN true
set_parameter_property genABuf2_g HDL_PARAMETER true
set_parameter_property genABuf2_g VISIBLE false
set_parameter_property genABuf2_g DERIVED TRUE

add_parameter genLedGadget_g BOOLEAN true
set_parameter_property genLedGadget_g HDL_PARAMETER true
set_parameter_property genLedGadget_g VISIBLE false
set_parameter_property genLedGadget_g DERIVED TRUE

add_parameter genEvent_g BOOLEAN true
set_parameter_property genEvent_g HDL_PARAMETER true
set_parameter_property genEvent_g VISIBLE false
set_parameter_property genEvent_g DERIVED TRUE

add_parameter genTimeSync_g BOOLEAN true
set_parameter_property genTimeSync_g HDL_PARAMETER true
set_parameter_property genTimeSync_g VISIBLE false
set_parameter_property genTimeSync_g DERIVED TRUE

add_parameter iRpdos_g INTEGER 1
set_parameter_property iRpdos_g HDL_PARAMETER true
set_parameter_property iRpdos_g ALLOWED_RANGES {0 1 2 3}
set_parameter_property iRpdos_g VISIBLE false
set_parameter_property iRpdos_g DERIVED TRUE

add_parameter iTpdos_g INTEGER 1
set_parameter_property iTpdos_g HDL_PARAMETER true
set_parameter_property iTpdos_g ALLOWED_RANGES {0 1}
set_parameter_property iTpdos_g VISIBLE false
set_parameter_property iTpdos_g DERIVED TRUE

add_parameter iTpdoBufSize_g INTEGER 1
set_parameter_property iTpdoBufSize_g HDL_PARAMETER true
set_parameter_property iTpdoBufSize_g VISIBLE false
set_parameter_property iTpdoBufSize_g DERIVED TRUE

add_parameter iRpdo0BufSize_g INTEGER 1
set_parameter_property iRpdo0BufSize_g HDL_PARAMETER true
set_parameter_property iRpdo0BufSize_g VISIBLE false
set_parameter_property iRpdo0BufSize_g DERIVED TRUE

add_parameter iRpdo1BufSize_g INTEGER 1
set_parameter_property iRpdo1BufSize_g HDL_PARAMETER true
set_parameter_property iRpdo1BufSize_g VISIBLE false
set_parameter_property iRpdo1BufSize_g DERIVED TRUE

add_parameter iRpdo2BufSize_g INTEGER 1
set_parameter_property iRpdo2BufSize_g HDL_PARAMETER true
set_parameter_property iRpdo2BufSize_g VISIBLE false
set_parameter_property iRpdo2BufSize_g DERIVED TRUE

add_parameter iAsyBuf1Size_g INTEGER 1514
set_parameter_property iAsyBuf1Size_g HDL_PARAMETER true
set_parameter_property iAsyBuf1Size_g VISIBLE false
set_parameter_property iAsyBuf1Size_g DERIVED TRUE

add_parameter iAsyBuf2Size_g INTEGER 1514
set_parameter_property iAsyBuf2Size_g HDL_PARAMETER true
set_parameter_property iAsyBuf2Size_g VISIBLE false
set_parameter_property iAsyBuf2Size_g DERIVED TRUE

#parameters for OPENMAC HDL
add_parameter Simulate BOOLEAN false
set_parameter_property Simulate HDL_PARAMETER true
set_parameter_property Simulate VISIBLE false
set_parameter_property Simulate DERIVED TRUE

add_parameter iBufSize_g INTEGER 1024
set_parameter_property iBufSize_g HDL_PARAMETER true
set_parameter_property iBufSize_g VISIBLE false
set_parameter_property iBufSize_g DERIVED TRUE

add_parameter iBufSizeLOG2_g INTEGER 10
set_parameter_property iBufSizeLOG2_g HDL_PARAMETER true
set_parameter_property iBufSizeLOG2_g VISIBLE false
set_parameter_property iBufSizeLOG2_g DERIVED TRUE

add_parameter useRmii_g BOOLEAN true
set_parameter_property useRmii_g HDL_PARAMETER true
set_parameter_property useRmii_g VISIBLE false
set_parameter_property useRmii_g DERIVED true

add_parameter useIntPacketBuf_g BOOLEAN true
set_parameter_property useIntPacketBuf_g HDL_PARAMETER true
set_parameter_property useIntPacketBuf_g VISIBLE false
set_parameter_property useIntPacketBuf_g DERIVED true

add_parameter useRxIntPacketBuf_g BOOLEAN true
set_parameter_property useRxIntPacketBuf_g HDL_PARAMETER true
set_parameter_property useRxIntPacketBuf_g VISIBLE false
set_parameter_property useRxIntPacketBuf_g DERIVED true

add_parameter use2ndCmpTimer_g BOOLEAN true
set_parameter_property use2ndCmpTimer_g HDL_PARAMETER true
set_parameter_property use2ndCmpTimer_g VISIBLE false
set_parameter_property use2ndCmpTimer_g DERIVED true

add_parameter use2ndPhy_g BOOLEAN true
set_parameter_property use2ndPhy_g HDL_PARAMETER true
set_parameter_property use2ndPhy_g VISIBLE false
set_parameter_property use2ndPhy_g DERIVED true

add_parameter gNumSmi INTEGER 1
set_parameter_property gNumSmi ALLOWED_RANGES {1 2}
set_parameter_property gNumSmi HDL_PARAMETER true
set_parameter_property gNumSmi VISIBLE false
set_parameter_property gNumSmi DERIVED true

add_parameter gen_dma_observer_g BOOLEAN false
set_parameter_property gen_dma_observer_g HDL_PARAMETER true
set_parameter_property gen_dma_observer_g VISIBLE false
set_parameter_property gen_dma_observer_g DERIVED true

#the following generic is fixed to 16 bit
add_parameter m_data_width_g INTEGER 16
set_parameter_property m_data_width_g HDL_PARAMETER true
set_parameter_property m_data_width_g VISIBLE false
set_parameter_property m_data_width_g DERIVED true

add_parameter m_burstcount_width_g INTEGER 1
set_parameter_property m_burstcount_width_g HDL_PARAMETER true
set_parameter_property m_burstcount_width_g VISIBLE false
set_parameter_property m_burstcount_width_g DERIVED true

add_parameter m_burstcount_const_g BOOLEAN TRUE
set_parameter_property m_burstcount_const_g HDL_PARAMETER true
set_parameter_property m_burstcount_const_g VISIBLE false
set_parameter_property m_burstcount_const_g DERIVED true

add_parameter m_tx_burst_size_g INTEGER 1
set_parameter_property m_tx_burst_size_g HDL_PARAMETER true
set_parameter_property m_tx_burst_size_g VISIBLE false
set_parameter_property m_tx_burst_size_g DERIVED true

add_parameter m_rx_burst_size_g INTEGER 1
set_parameter_property m_rx_burst_size_g HDL_PARAMETER true
set_parameter_property m_rx_burst_size_g VISIBLE false
set_parameter_property m_rx_burst_size_g DERIVED true

add_parameter m_tx_fifo_size_g INTEGER 16
set_parameter_property m_tx_fifo_size_g HDL_PARAMETER true
set_parameter_property m_tx_fifo_size_g VISIBLE false
set_parameter_property m_tx_fifo_size_g DERIVED true

add_parameter m_rx_fifo_size_g INTEGER 16
set_parameter_property m_rx_fifo_size_g HDL_PARAMETER true
set_parameter_property m_rx_fifo_size_g VISIBLE false
set_parameter_property m_rx_fifo_size_g DERIVED true

#parameters for parallel interface
add_parameter papDataWidth_g INTEGER 16
set_parameter_property papDataWidth_g HDL_PARAMETER true
set_parameter_property papDataWidth_g VISIBLE false
set_parameter_property papDataWidth_g DERIVED TRUE

add_parameter papLowAct_g BOOLEAN false
set_parameter_property papLowAct_g HDL_PARAMETER true
set_parameter_property papLowAct_g VISIBLE false
set_parameter_property papLowAct_g DERIVED TRUE

add_parameter papBigEnd_g BOOLEAN false
set_parameter_property papBigEnd_g HDL_PARAMETER true
set_parameter_property papBigEnd_g VISIBLE false
set_parameter_property papBigEnd_g DERIVED TRUE

#parameters for SPI
add_parameter spiCPOL_g BOOLEAN false
set_parameter_property spiCPOL_g HDL_PARAMETER true
set_parameter_property spiCPOL_g VISIBLE false
set_parameter_property spiCPOL_g DERIVED TRUE

add_parameter spiCPHA_g BOOLEAN false
set_parameter_property spiCPHA_g HDL_PARAMETER true
set_parameter_property spiCPHA_g VISIBLE false
set_parameter_property spiCPHA_g DERIVED TRUE

add_parameter spiBigEnd_g BOOLEAN false
set_parameter_property spiBigEnd_g HDL_PARAMETER true
set_parameter_property spiBigEnd_g VISIBLE false
set_parameter_property spiBigEnd_g DERIVED TRUE

#parameters for portio
add_parameter pioValLen_g INTEGER 50
set_parameter_property pioValLen_g HDL_PARAMETER true
set_parameter_property pioValLen_g VISIBLE false
set_parameter_property pioValLen_g DERIVED TRUE

proc my_validation_callback {} {
#do some preparation stuff
	set configPowerlink 			[get_parameter_value configPowerlink]
	set configApInterface 			[get_parameter_value configApInterface]
	set configApParallelInterface 	[get_parameter_value configApParallelInterface]
	set spiCpol						[get_parameter_value configApSpi_CPOL]
	set spiCpha						[get_parameter_value configApSpi_CPHA]
	set rpdos						[get_parameter_value rpdoNum]
	set tpdos						[get_parameter_value tpdoNum]
	set rpdo0size					[get_parameter_value rpdo0size]
	set rpdo1size					[get_parameter_value rpdo0size]
	#set rpdo1size					[get_parameter_value rpdo1size]
	set rpdo2size					[get_parameter_value rpdo0size]
	#set rpdo2size					[get_parameter_value rpdo2size]
	set tpdo0size					[get_parameter_value tpdo0size]
	set asyncBuf1Size				[get_parameter_value asyncBuf1Size]
	set asyncBuf2Size				[get_parameter_value asyncBuf2Size]
	set ledGadgetEn					[get_parameter_value genLedGadget]
	
	set macTxBuf					[get_parameter_value macTxBuf]
	set macRxBuf					[get_parameter_value macRxBuf]
	set useLowJitterSync			[get_parameter_value hwSupportSyncIrq]
	
	set mii							[get_parameter_value phyIF]
	set ploc						[get_parameter_value packetLoc]
	set enDmaObserver 				[get_parameter_value enDmaObserver]
    set genEvent                    [get_parameter_value genEvent]
    
    set expert                      [get_parameter_value expertMode]
	
	if {$mii == "RMII"} {
		set_parameter_value useRmii_g true
	} else {
		set_parameter_value useRmii_g false
		send_message info "Consider to use RMII to reduce resource usage!"
	}
	
	if {$ploc == "TX and RX into DPRAM"} {
		set_parameter_value useIntPacketBuf_g true
		set_parameter_value useRxIntPacketBuf_g true
	} elseif {$ploc == "TX into DPRAM and RX over Avalon Master" } {
		set_parameter_value useIntPacketBuf_g true
		set_parameter_value useRxIntPacketBuf_g false
		send_message info "Connect the Avalon Master 'MAC_DMA' to the memory where Heap is located!"
	} elseif {$ploc == "TX and RX over Avalon Master"} {
		set_parameter_value useIntPacketBuf_g false
		set_parameter_value useRxIntPacketBuf_g false
		send_message info "Connect the Avalon Master 'MAC_DMA' to low latency memory! Heap must be located in the same memory!"
	} else {
		send_message error "error 0x01"
	}
	
	if {$expert} {
        set macTxBurstSize [get_parameter_value macTxBurstSize]
    	set macRxBurstSize [get_parameter_value macRxBurstSize]
    } else {
        #no expert mode set them to one per default
        set macTxBurstSize 1
        set macRxBurstSize 1
    }
	
	#burst size setting allowed?!
	if {$ploc == "TX and RX into DPRAM"} {
		#no
		set_parameter_property macTxBurstSize VISIBLE false
		set_parameter_property macRxBurstSize VISIBLE false
		
		#no bursts...
		set macTxBurstSize 0
		set macRxBurstSize 0
	} else {
		#yes!
		
		if {$ploc == "TX into DPRAM and RX over Avalon Master"} {
			set_parameter_property macTxBurstSize VISIBLE false
            if {$expert} {
			    set_parameter_property macRxBurstSize VISIBLE true
			} else {
                set_parameter_property macRxBurstSize VISIBLE false
            }
            
			#no tx bursts...
			set macTxBurstSize 0
		} elseif {$ploc == "TX and RX over Avalon Master"} {
            if {$expert} {
    			set_parameter_property macTxBurstSize VISIBLE true
    			set_parameter_property macRxBurstSize VISIBLE true
            } else {
                set_parameter_property macTxBurstSize VISIBLE false
    			set_parameter_property macRxBurstSize VISIBLE false
            }
		} else {
			#oje :(
			send_message error "error 0x01a"
		}
		
		#find the max burst size to be handled
		if {$macTxBurstSize > $macRxBurstSize} {
			set maxMacBurstSize $macTxBurstSize
		} else {
			set maxMacBurstSize $macRxBurstSize
		}
		
		set m_burstcntwidth [expr int(ceil(log($maxMacBurstSize) / log(2.))) + 1]
		
		set_parameter_value m_burstcount_width_g $m_burstcntwidth
		#also define the fifo size
		set txFifoSize 16
		set rxFifoSize 16
		
		if {[expr $macTxBurstSize * 2] > $txFifoSize} {
			set txFifoSize [expr $macTxBurstSize * 2]
		}
		
		if {[expr $macRxBurstSize * 2] > $rxFifoSize} {
			set rxFifoSize [expr $macRxBurstSize * 2]
		}
		
		set_parameter_value m_tx_fifo_size_g $txFifoSize
		set_parameter_value m_tx_burst_size_g $macTxBurstSize
		
		set_parameter_value m_rx_fifo_size_g $rxFifoSize
		set_parameter_value m_rx_burst_size_g $macRxBurstSize
		
		if {$macTxBurstSize > 1 || $macRxBurstSize > 1} {
			send_message info "The Avalon Master 'MAC_DMA' performs 16bit burst transfers (TX=$macTxBurstSize RX=$macRxBurstSize)."
		} else {
			#no burst transfers
		}
	}
	
	set memRpdo 0
	set memTpdo 0
	
	#add to RPDOs and Async buffers the header (since it isn't done by vhdl anymore)
	set rpdo0size 					[expr $rpdo0size + 16]
	set rpdo1size 					[expr $rpdo1size + 16]
	set rpdo2size 					[expr $rpdo2size + 16]
	set tpdo0size 					[expr $tpdo0size + 0]
	
	#async buffers set to zero are omitted
	
	#hold on! asyncBuf1Size may not be lower 20 byte!!!
	if {$asyncBuf1Size < 20} {
		send_message error "Set Asynchronous Buffer Nr. 1 Size to at least 20 byte!"
	}
	
	#set boolean generic
	if {$asyncBuf1Size == 0} {
		set_parameter_value genABuf1_g false
	} else {
		set asyncBuf1Size			[expr $asyncBuf1Size + 4]
		set_parameter_value genABuf1_g true
	}
	
	if {$asyncBuf2Size == 0} {
		set_parameter_value genABuf2_g false
	} else {
		set asyncBuf2Size			[expr $asyncBuf2Size + 4]
		set_parameter_value genABuf2_g true
	}
	
	set genPdi false
	set genAvalonAp false
	set genSimpleIO false
	set genSpiAp false
	
	#some constants from openMAC
	set macPktLength	4
	# tx buffer header (header + packet length)
	set macTxHd			[expr  0 + $macPktLength]
	# rx buffer header (header + packet length)
	set macRxHd 		[expr 26 + $macPktLength]
	# max rx buffers
	set macRxBuffers 	16
	# max tx buffers
	set macTxBuffers	16
	# mtu by ieee
	set mtu 			1500
	# eth header
	set ethHd			14
	# crc size by ieee
	set crc				4
	# min data size of a packet
	set minDatSize		46
	# min packet size (ethheader + mindata + crc + tx buffer header)
	set minPktBufSize	[expr $ethHd + $minDatSize + $crc + $macTxHd]
	# max packet size (ethheader + mtu + crc + tx buffer header)
	set maxPktBufSize	[expr $ethHd + $mtu + $crc + $macTxHd]

#so, now verify which configuration should be set
	#default assignments...
	set_parameter_property configApInterface VISIBLE false
	set_parameter_property configApParallelInterface VISIBLE false
	set_parameter_property configApParSigs VISIBLE false
	set_parameter_property configApParOutSigs VISIBLE false
	set_parameter_property configApEndian VISIBLE false
	set_parameter_property configApSpi_CPOL VISIBLE false
	set_parameter_property configApSpi_CPHA VISIBLE false
	set_parameter_property configApSpi_IRQ VISIBLE false
	set_parameter_property genLedGadget VISIBLE false
	set_parameter_property genEvent VISIBLE false
	set_parameter_property asyncBuf1Size VISIBLE false
	set_parameter_property asyncBuf2Size VISIBLE false
	set_parameter_property rpdo0size VISIBLE false
	set_parameter_property tpdo0size VISIBLE false
	set_parameter_property validAssertDuration VISIBLE false
	set_parameter_property validSet VISIBLE false
	set_parameter_property macTxBuf VISIBLE false
	set_parameter_property macRxBuf VISIBLE false
	set_parameter_property hwSupportSyncIrq VISIBLE false
	set_parameter_property enDmaObserver VISIBLE false
	
	set_parameter_property mac2phys VISIBLE true
    set_parameter_property macGen2ndSmi VISIBLE false
	
	set_parameter_property rpdoNum VISIBLE true
	set_parameter_property tpdoNum VISIBLE true
	
	if {$configPowerlink == "openMAC only"} {
		#no PDI, only openMAC
		if {$ploc == "TX and RX into DPRAM"} {
			set_parameter_property macTxBuf VISIBLE true
			set_parameter_property macRxBuf VISIBLE true
		} elseif {$ploc == "TX into DPRAM and RX over Avalon Master"} {
			set_parameter_property macTxBuf VISIBLE true
		} elseif {$ploc == "TX and RX over Avalon Master"} {
			#nothing to set
		} else {
			send_message error "error 0x02"
		}
		
		set_parameter_property rpdoNum VISIBLE false
		set_parameter_property tpdoNum VISIBLE false
	} elseif {$configPowerlink == "Direct I/O CN"} {
		#CN is only a Direct I/O CN, so there are only 4bytes I/Os
		if {$rpdos == 1} {
			set rpdo0size [expr 4 + 16]
			set rpdo1size 0
			set rpdo2size 0
			set macRxBuffers 4
		} elseif {$rpdos == 2} {
			set rpdo0size [expr 4 + 16]
			set rpdo1size [expr 4 + 16]
			set rpdo2size 0
			set macRxBuffers 5
		} elseif {$rpdos == 3} {
			set rpdo0size [expr 4 + 16]
			set rpdo1size [expr 4 + 16]
			set rpdo2size [expr 4 + 16]
			set macRxBuffers 6
		}
		#and fix tpdo size
		set tpdo0size 4
		
		set genSimpleIO true
		
		set_parameter_property validAssertDuration VISIBLE true
		set_parameter_property validSet VISIBLE true
		
	} elseif {$configPowerlink == "CN with Processor Interface"} {
		#CN is connected to AP processor, so enable everything for this
		set_parameter_property configApInterface VISIBLE true
		set_parameter_property asyncBuf1Size VISIBLE true
		set_parameter_property asyncBuf2Size VISIBLE true
		set_parameter_property genLedGadget VISIBLE true
        if {$expert} {
            #in case of expert mode event hw support can be set
		    set_parameter_property genEvent VISIBLE true
            set_parameter_value genEvent_g $genEvent
            if {$genEvent} {
            } else {
                send_message warning "Event Hardware Support is mandatory for CN API library!"
            }
        } else {
            #no expert mode => TRUE!
            set_parameter_property genEvent VISIBLE false
            set_parameter_value genEvent_g true
        }
		#AP can be big or little endian - allow choice
		set_parameter_property configApEndian VISIBLE true
		set_parameter_property hwSupportSyncIrq VISIBLE true
		
		#set the led gadget enable generic
		set_parameter_value genLedGadget_g $ledGadgetEn
		
		set genPdi true
		
		#set rpdo size to zero if not used
		if {$rpdos == 1} {
			set_parameter_property rpdo0size VISIBLE true
			set rpdo1size 0
			set rpdo2size 0
			set macRxBuffers 4
			set memRpdo [expr ($rpdo0size)*3]
		} elseif {$rpdos == 2} {
			set_parameter_property rpdo0size VISIBLE true
			set rpdo2size 0
			set macRxBuffers 5
			set memRpdo [expr ($rpdo0size + $rpdo1size)*3]
		} elseif {$rpdos == 3} {
			set_parameter_property rpdo0size VISIBLE true
			set macRxBuffers 6
			set memRpdo [expr ($rpdo0size + $rpdo1size + $rpdo2size )*3]
		}
		set_parameter_property tpdo0size VISIBLE true
		set memTpdo [expr ($tpdo0size)*3]
		
		if {$configApInterface == "Avalon"} {
			#avalon is used for the ap!
			set genAvalonAp true
			
		} elseif {$configApInterface == "Parallel"} {
			#the parallel interface is used
			
			set_parameter_property configApParallelInterface VISIBLE true
			set_parameter_property configApParSigs VISIBLE true
			set_parameter_property configApParOutSigs VISIBLE true
			
		} elseif {$configApInterface == "SPI"} {
			#let's use spi
			set_parameter_property configApSpi_CPOL VISIBLE true
			set_parameter_property configApSpi_CPHA VISIBLE true
			set_parameter_property configApSpi_IRQ VISIBLE true
			
			set genSpiAp true
			
		}
	}
	
	#calc tx packet size
	set IdRes 	[expr 176 				+ $crc + $macTxHd]
	set StRes 	[expr 72 				+ $crc + $macTxHd]
	set NmtReq 	[expr $ethHd + $mtu		+ $crc + $macTxHd]
	set nonEpl	[expr $ethHd + $mtu		+ $crc + $macTxHd]
	set PRes	[expr 24 + $tpdo0size	+ $crc + $macTxHd]
	#sync response for poll-resp-ch (44 bytes + padding = 60bytes)
	set SyncRes [expr 60				+ $crc + $macTxHd]
	
	if {$PRes < $minPktBufSize} {
		#PRes buffer is smaller 64 bytes => padding!
		set PRes $minPktBufSize
	}
	
	#the following error is catched by the allowed range of pdo size
	if {$PRes > $maxPktBufSize} {
		send_message error "TPDO Size is too large. Allowed Range 1...1490 bytes!"
	}
	
	#align all tx buffers
	set IdRes 	[expr ($IdRes + 3) & ~3]
	set StRes 	[expr ($StRes + 3) & ~3]
	set NmtReq 	[expr ($NmtReq + 3) & ~3]
	set nonEpl 	[expr ($nonEpl + 3) & ~3]
	set PRes 	[expr ($PRes + 3) & ~3]
	set SyncRes [expr ($SyncRes + 3) & ~3]
	
	#calculate tx buffer size out of tpdos and other packets
	set txBufSize [expr $IdRes + $StRes + $NmtReq + $nonEpl + $PRes + $SyncRes]
	set macTxBuffers 6
	
	#openPOWERLINK allocates TX buffers twice (ping-pong)
	set txBufSize [expr $txBufSize * 2]
	set macTxBuffers [expr $macTxBuffers * 2]
	
	#calculate rx buffer size out of packets per cycle
	set rxBufSize [expr $ethHd + $mtu + $crc + $macRxHd]
	set rxBufSize [expr ($rxBufSize + 3) & ~3]
	set rxBufSize [expr $macRxBuffers * $rxBufSize]
	
	if {$configPowerlink == "openMAC only"} {
		#overwrite done calulations
		# for tx
		set txBufSize $macTxBuf
		# for rx using MTU and number of buffers...
		set rxBufSize [expr $macRxBuf * ($ethHd + $mtu + $crc + $macRxHd)]
		
		# forward number of RX buffers
		set macRxBuffers $macRxBuf
		
		#set unsupported to zeros or allowed range
		set macTxBuffers 0
		set rpdos 0
		set tpdos 0
		set rpdo0size 0
		set rpdo1size 0
		set rpdo2size 0
		set tpdo0size 0
		set asyncBuf1Size 0
		set asyncBuf2Size 0
	}
	
	if {$ploc == "TX and RX into DPRAM"} {
		set macBufSize [expr $txBufSize + $rxBufSize]
		set log2MacBufSize [expr int(ceil(log($macBufSize) / log(2.)))]
		set enDmaObserver false
	} elseif {$ploc == "TX into DPRAM and RX over Avalon Master" } {
		set macBufSize $txBufSize
        if {$expert} {
            #expert may enable it manually
		    set_parameter_property enDmaObserver VISIBLE true
        } else {
            #no expert is not visible but set to true
            set enDmaObserver true
        }
		#no rx buffers are stored in dpram => set to zero
		set rxBufSize 0
		set log2MacBufSize [expr int(ceil(log($macBufSize) / log(2.)))]
		set macRxBuffers 16
	} elseif {$ploc == "TX and RX over Avalon Master"} {
        if {$expert} {
            #expert may enable it manually
		    set_parameter_property enDmaObserver VISIBLE true
        } else {
            #no expert is not visible but set to true
            set enDmaObserver true
        }
		#any value to avoid errors in SOPC
		set macBufSize 0
		#no rx and tx buffers are stored in dpram => set to zero
		set rxBufSize 0
		set txBufSize 0
		set log2MacBufSize 3
	} else {
		set macBufSize 0
		set rxBufSize 0
		set txBufSize 0
		set log2MacBufSize 3
	}
	
	#set pdi generics
	set_parameter_value iRpdos_g			$rpdos
	set_parameter_value iTpdos_g			$tpdos
	set_parameter_value iTpdoBufSize_g		$tpdo0size
	set_parameter_value iRpdo0BufSize_g		$rpdo0size
	set_parameter_value iRpdo1BufSize_g		$rpdo1size
	set_parameter_value iRpdo2BufSize_g		$rpdo2size
	set_parameter_value iAsyBuf1Size_g		$asyncBuf1Size
	set_parameter_value iAsyBuf2Size_g		$asyncBuf2Size
	
#now, let's set generics to HDL
	set_parameter_value genPdi_g			$genPdi
	set_parameter_value genInternalAp_g		$genAvalonAp
	set_parameter_value genSimpleIO_g		$genSimpleIO
	set_parameter_value genSpiAp_g			$genSpiAp
	
	set_parameter_value Simulate			false
	set_parameter_value iBufSize_g			$macBufSize
	set_parameter_value iBufSizeLOG2_g		$log2MacBufSize
	
	set_parameter_value gen_dma_observer_g	$enDmaObserver
	
	if {[get_parameter_value configApParallelInterface] == "8bit"} {
		set_parameter_value papDataWidth_g	8
	} else {
		set_parameter_value papDataWidth_g	16
	}
	if {[get_parameter_value configApEndian] == "Little"} {
		set_parameter_value papBigEnd_g	false
		set_parameter_value spiBigEnd_g	false
	} else {
#		big/little endian conversion is considered by software, THX Michael!
#		set_parameter_value papBigEnd_g	true
#		set_parameter_value spiBigEnd_g	true
		set_parameter_value papBigEnd_g	false
		set_parameter_value spiBigEnd_g	false
	}
	if {[get_parameter_value configApParSigs] == "Low Active"} {
		set_parameter_value papLowAct_g	true
	} else {
		set_parameter_value papLowAct_g	false
	}
	if {$spiCpol == "1"} {
		set_parameter_value spiCPOL_g true
	} else {
		set_parameter_value spiCPOL_g false
	}
	if {$spiCpha == "1"} {
		set_parameter_value spiCPHA_g true
	} else {
		set_parameter_value spiCPHA_g false
	}
	
	#generate 2 phy port
	set_parameter_value use2ndPhy_g false
	set_module_assignment embeddedsw.CMacro.PHYCNT 1
    #default
    set_parameter_value gNumSmi 1
	if {[get_parameter_value mac2phys]} {
		set_parameter_value use2ndPhy_g true
		set_module_assignment embeddedsw.CMacro.PHYCNT 2
        #two phys are used
        set_parameter_property macGen2ndSmi VISIBLE true
        if {[get_parameter_value macGen2ndSmi]} {
            #generate 2nd SMI
            set_parameter_value gNumSmi 2
        } else {
            #generate one SMI
            set_parameter_value gNumSmi 1
        }
	} else {
        #one phy is used
        set_parameter_property macGen2ndSmi VISIBLE false
    }
	
	#generate 2nd timer cmp if pdi and if set in sopc
	# otherwise not (e.g. openMAC only, DirectIO or no selected)
	set_parameter_value use2ndCmpTimer_g FALSE
    set_parameter_value genTimeSync_g FALSE
	if {$configPowerlink == "CN with Processor Interface"} {
		if {$useLowJitterSync} {
			set_parameter_value use2ndCmpTimer_g true
            set_parameter_value genTimeSync_g true
		} else {
		}
	} else {
	}
	
	#forward parameters to system.h
	
	# workaround: strings are erroneous => no blanks, etc.
	if {$ledGadgetEn} {
		set_module_assignment embeddedsw.CMacro.LEDGADGET			TRUE
	} else {
		set_module_assignment embeddedsw.CMacro.LEDGADGET			FALSE
	}
	
	if {$enDmaObserver} {
		set_module_assignment embeddedsw.CMacro.DMAOBSERV			TRUE
	} else {
		set_module_assignment embeddedsw.CMacro.DMAOBSERV			FALSE
	}
	
	if {[get_parameter_value hwSupportSyncIrq]} {
		set_module_assignment embeddedsw.CMacro.TIMESYNCHW		    TRUE
	} else {
		set_module_assignment embeddedsw.CMacro.TIMESYNCHW			FALSE
	}
	
	if {[get_parameter_value genEvent_g]} {
		set_module_assignment embeddedsw.CMacro.EVENT			TRUE
	} else {
		set_module_assignment embeddedsw.CMacro.EVENT			FALSE
	}
	
	if {$configPowerlink == "Direct I/O CN"} {
																	#direct I/O
		set_module_assignment embeddedsw.CMacro.CONFIG				0
	} elseif {$configPowerlink == "CN with Processor Interface"} {
		if {$configApInterface == "Avalon"} {
																	#Avalon
			set_module_assignment embeddedsw.CMacro.CONFIG			4
		} elseif {$configApInterface == "Parallel"} {
			if {[get_parameter_value configApParallelInterface] == "8bit"} {
																	#parallel 8bit
				set_module_assignment embeddedsw.CMacro.CONFIG		1
			} else {
																	#parallel 16bit
				set_module_assignment embeddedsw.CMacro.CONFIG		2
			}
		} else {
																	#SPI
			set_module_assignment embeddedsw.CMacro.CONFIG			3
		}
	} elseif {$configPowerlink == "openMAC only"} {
																	#openMAC only
		set_module_assignment embeddedsw.CMacro.CONFIG				5
	}
	
	if {[get_parameter_value configApEndian] == "Little"} {
																	#little endian
		set_module_assignment embeddedsw.CMacro.CONFIGAPENDIAN		0
	} else {
																	#big endian
		set_module_assignment embeddedsw.CMacro.CONFIGAPENDIAN		1
	}
	
	if {$ploc == "TX and RX into DPRAM"} {							#all packets stored in openMAC DPRAM
		set_module_assignment embeddedsw.CMacro.PKTLOC				0
	} elseif {$ploc == "TX into DPRAM and RX over Avalon Master"} {	#Rx packets stored in heap
		set_module_assignment embeddedsw.CMacro.PKTLOC				1
	} elseif {$ploc == "TX and RX over Avalon Master"} {			#all packets stored in heap
		set_module_assignment embeddedsw.CMacro.PKTLOC				2
	} else {
		send_message error "error 0x03"
	}
	
	#####################################
	# here set the PDI revision number  #
	set_parameter_value iPdiRev_g 2
	#####################################
	
	# here you can change manually to use only one PDI Clk domain
	set_parameter_value genOnePdiClkDomain_g false
	
	set_module_assignment embeddedsw.CMacro.MACBUFSIZE				$macBufSize
	set_module_assignment embeddedsw.CMacro.MACRXBUFSIZE			$rxBufSize
	set_module_assignment embeddedsw.CMacro.MACRXBUFFERS			$macRxBuffers
	set_module_assignment embeddedsw.CMacro.MACTXBUFSIZE			$txBufSize
	set_module_assignment embeddedsw.CMacro.MACTXBUFFERS			$macTxBuffers
	set_module_assignment embeddedsw.CMacro.PDIRPDOS				$rpdos
	set_module_assignment embeddedsw.CMacro.PDITPDOS				$tpdos
	set_module_assignment embeddedsw.CMacro.FPGAREV					[get_parameter_value iPdiRev_g]
	
	#####################################################################
	# create mif files for dpr (openMAC and PDI)
	
	set pdiDprMifName 		"../mif/pdi_dpr.mif"
	set macDpr1616MifName 	"../mif/dpr_16_16.mif"
	set macDpr1632MifName 	"../mif/dpr_16_32.mif"
	
	if {$configPowerlink == "openMAC only" || $configPowerlink == "Direct I/O CN"} {
		# no pdi present, therefore delete pdi dpr mif
		if {[file isdirectory [file dirname $pdiDprMifName]]} {
			file delete $pdiDprMifName
		}
	} else {
		# for the pdi dpr the size has to be calculated
		# taken from pdi.vhd constant intCntStReg_c
		set pdiDprCntStReg [expr 4 * 22]
		
		# align buffers before calculate dpr size
		set tpdo0size [expr ($tpdo0size + 3) & ~3]
		set rpdo0size [expr ($rpdo0size + 3) & ~3]
		set rpdo1size [expr ($rpdo1size + 3) & ~3]
		set rpdo2size [expr ($rpdo2size + 3) & ~3]
		set asyncBuf1Size [expr ($asyncBuf1Size + 3) & ~3]
		set asyncBuf2Size [expr ($asyncBuf2Size + 3) & ~3]
		
		set pdiDprSize [expr $pdiDprCntStReg + 3* $tpdo0size + 3* $rpdo0size + 3* $rpdo1size + 3* $rpdo2size + 2* $asyncBuf1Size + 2* $asyncBuf2Size]
		set pdiDprSize [expr $pdiDprSize / 4]
		set pdiDprHigh [expr $pdiDprSize - 1]
		
		# create pdi dpr mif
		set pdiDprMifData "WIDTH = 32;\n\DEPTH = $pdiDprSize;\nADDRESS_RADIX = UNS;\nDATA_RADIX = HEX;\n\nCONTENT BEGIN\n\t\[ 0 .. 3 \] : 0;\n\t4 : 00EEFFFF;\n\t\[ 5 .. $pdiDprHigh \] : 0;\nEND;\n"
		
		if {![file isdirectory [file dirname $pdiDprMifName]]} {
			file mkdir $pdiDprMifName
		}
		
		set pdiDprMifId [open $pdiDprMifName "w"]
		puts -nonewline $pdiDprMifId $pdiDprMifData
		close $pdiDprMifId
	}
	
	# create openMAC 16/16 dpr mif
	set macDprSize 256
	set macDprHigh [expr $macDprSize - 1]
	set macDprMifData "WIDTH = 16;\nDEPTH = $macDprSize;\nADDRESS_RADIX = UNS;\nDATA_RADIX = HEX;\n\nCONTENT BEGIN\n\t\[ 0 .. $macDprHigh \] : 0;\nEND;\n"
	set macDprMifName $macDpr1616MifName
	
	if {![file isdirectory [file dirname $macDprMifName]]} {
		file mkdir $macDprMifName
	}
	
	set macDprMifId [open $macDprMifName "w"]
	puts -nonewline $macDprMifId $macDprMifData
	close $macDprMifId
	
	# create openMAC 16/32 dpr mif
	set macDprSize 256
	set macDprHigh [expr $macDprSize - 1]
	set macDprMifData "WIDTH = 16;\nDEPTH = $macDprSize;\nADDRESS_RADIX = UNS;\nDATA_RADIX = HEX;\n\nCONTENT BEGIN\n\t\[ 0 .. $macDprHigh \] : 0;\nEND;\n"
	set macDprMifName $macDpr1632MifName
	
	if {![file isdirectory [file dirname $macDprMifName]]} {
		file mkdir $macDprMifName
	}
	
	set macDprMifId [open $macDprMifName "w"]
	puts -nonewline $macDprMifId $macDprMifData
	close $macDprMifId
	#####################################################################
	
}

#display
add_display_item "Block Diagram" id0 icon img/block_diagram.png
add_display_item "General Settings" expertMode PARAMETER
add_display_item "General Settings" configPowerlink PARAMETER
add_display_item "Process Data Interface Settings" configApInterface PARAMETER
add_display_item "Process Data Interface Settings" configApParallelInterface PARAMETER
add_display_item "Process Data Interface Settings" configApParOutSigs PARAMETER
add_display_item "Process Data Interface Settings" configApParSigs PARAMETER
add_display_item "Process Data Interface Settings" configApSpi_IRQ PARAMETER
add_display_item "Process Data Interface Settings" configApEndian PARAMETER
add_display_item "Process Data Interface Settings" configApSpi_CPOL PARAMETER
add_display_item "Process Data Interface Settings" configApSpi_CPHA PARAMETER
add_display_item "Process Data Interface Settings" validSet PARAMETER
add_display_item "Process Data Interface Settings" validAssertDuration PARAMETER
add_display_item "Process Data Interface Settings" hwSupportSyncIrq PARAMETER
add_display_item "Process Data Interface Settings" genLedGadget PARAMETER
add_display_item "Process Data Interface Settings" genEvent PARAMETER
add_display_item "Receive Process Data" rpdoNum PARAMETER
add_display_item "Transmit Process Data" tpdoNum PARAMETER
add_display_item "Transmit Process Data" tpdo0size PARAMETER
add_display_item "Receive Process Data" rpdo0size PARAMETER
add_display_item "Asynchronous Buffer" asyncBuf1Size  PARAMETER
add_display_item "Asynchronous Buffer" asyncBuf2Size  PARAMETER
add_display_item "openMAC" phyIF  PARAMETER
add_display_item "openMAC" mac2phys PARAMETER
add_display_item "openMAC" macGen2ndSmi PARAMETER
add_display_item "openMAC" packetLoc  PARAMETER
add_display_item "openMAC" enDmaObserver PARAMETER
add_display_item "openMAC" macTxBurstSize  PARAMETER
add_display_item "openMAC" macRxBurstSize  PARAMETER
add_display_item "openMAC" macTxBuf  PARAMETER
add_display_item "openMAC" macRxBuf  PARAMETER

#INTERFACES

#Clock Sinks
##pcp clk
add_interface pcp_clk clock end
set_interface_property pcp_clk ENABLED true
add_interface_port pcp_clk clkPcp clk Input 1
add_interface_port pcp_clk rstPcp reset Input 1

##ap clk
add_interface ap_clk clock end
set_interface_property ap_clk ENABLED true
add_interface_port ap_clk clkAp clk Input 1
add_interface_port ap_clk rstAp reset Input 1

##clk Ethernet
add_interface clkEth clock end
set_interface_property clkEth ENABLED true
add_interface_port clkEth clkEth clk Input 1

##clk 50MHz
add_interface clk50meg clock end
set_interface_property clk50meg ENABLED true
add_interface_port clk50meg clk50 clk Input 1
add_interface_port clk50meg rst reset Input 1

##master clock
add_interface clkMaster clock end
set_interface_property clkMaster ENABLED true
add_interface_port clkMaster m_clk clk Input 1

##pkt buffer clock
add_interface clkPkt clock end
set_interface_property clkPkt ENABLED true
add_interface_port clkPkt pkt_clk clk Input 1

#openMAC
##Avalon Memory Mapped Slave: Compare Unit 
add_interface MAC_CMP avalon end
set_interface_property MAC_CMP addressAlignment DYNAMIC
set_interface_property MAC_CMP associatedClock clk50meg
set_interface_property MAC_CMP burstOnBurstBoundariesOnly false
set_interface_property MAC_CMP explicitAddressSpan 0
set_interface_property MAC_CMP holdTime 0
set_interface_property MAC_CMP isMemoryDevice false
set_interface_property MAC_CMP isNonVolatileStorage false
set_interface_property MAC_CMP linewrapBursts false
set_interface_property MAC_CMP maximumPendingReadTransactions 0
set_interface_property MAC_CMP printableDevice false
set_interface_property MAC_CMP readLatency 0
set_interface_property MAC_CMP readWaitTime 1
set_interface_property MAC_CMP setupTime 0
set_interface_property MAC_CMP timingUnits Cycles
set_interface_property MAC_CMP writeWaitTime 0
set_interface_property MAC_CMP ENABLED true
add_interface_port MAC_CMP tcp_read read Input 1
add_interface_port MAC_CMP tcp_write write Input 1
add_interface_port MAC_CMP tcp_byteenable byteenable Input 4
add_interface_port MAC_CMP tcp_address address Input 2
add_interface_port MAC_CMP tcp_writedata writedata Input 32
add_interface_port MAC_CMP tcp_readdata readdata Output 32
add_interface_port MAC_CMP tcp_chipselect chipselect Input 1
add_interface_port MAC_CMP tcp_waitrequest waitrequest Output 1

##MAC COMPARE IRQ source
add_interface MACCMP_IRQ interrupt end
set_interface_property MACCMP_IRQ associatedAddressablePoint MAC_CMP
set_interface_property MACCMP_IRQ ASSOCIATED_CLOCK clk50meg
set_interface_property MACCMP_IRQ ENABLED true
add_interface_port MACCMP_IRQ tcp_irq irq Output 1

##Avalon Memory Mapped Slave: MAC_REG Register
add_interface MAC_REG avalon end
set_interface_property MAC_REG addressAlignment DYNAMIC
set_interface_property MAC_REG associatedClock clk50meg
set_interface_property MAC_REG burstOnBurstBoundariesOnly false
set_interface_property MAC_REG explicitAddressSpan 0
set_interface_property MAC_REG holdTime 0
set_interface_property MAC_REG isMemoryDevice false
set_interface_property MAC_REG isNonVolatileStorage false
set_interface_property MAC_REG linewrapBursts false
set_interface_property MAC_REG maximumPendingReadTransactions 0
set_interface_property MAC_REG printableDevice false
set_interface_property MAC_REG readLatency 0
set_interface_property MAC_REG readWaitTime 1
set_interface_property MAC_REG setupTime 0
set_interface_property MAC_REG timingUnits Cycles
set_interface_property MAC_REG writeWaitTime 0
set_interface_property MAC_REG ENABLED true
add_interface_port MAC_REG mac_chipselect chipselect Input 1
add_interface_port MAC_REG mac_read read Input 1
add_interface_port MAC_REG mac_write write Input 1
add_interface_port MAC_REG mac_byteenable byteenable Input 2
add_interface_port MAC_REG mac_address address Input 12
add_interface_port MAC_REG mac_writedata writedata Input 16
add_interface_port MAC_REG mac_readdata readdata Output 16
add_interface_port MAC_REG mac_waitrequest waitrequest Output 1

##MAC IRQ source
add_interface MAC_IRQ interrupt end
set_interface_property MAC_IRQ associatedAddressablePoint MAC_REG
set_interface_property MAC_IRQ ASSOCIATED_CLOCK clk50meg
set_interface_property MAC_IRQ ENABLED true
add_interface_port MAC_IRQ mac_irq irq Output 1

##Export Phy Management shared
add_interface PHYM conduit end
set_interface_property PHYM ENABLED true
add_interface_port PHYM phy_SMIClk export Output 1
add_interface_port PHYM phy_SMIDat export Bidir 1
add_interface_port PHYM phy_Rst_n export Output 1

##Export Phy Management 0
add_interface PHYM0 conduit end
set_interface_property PHYM0 ENABLED true
add_interface_port PHYM0 phy0_SMIClk export Output 1
add_interface_port PHYM0 phy0_SMIDat export Bidir 1
add_interface_port PHYM0 phy0_Rst_n export Output 1

##Export Phy Management 1
add_interface PHYM1 conduit end
set_interface_property PHYM1 ENABLED true
add_interface_port PHYM1 phy1_SMIClk export Output 1
add_interface_port PHYM1 phy1_SMIDat export Bidir 1
add_interface_port PHYM1 phy1_Rst_n export Output 1

##Export Phy Links
add_interface PHYL conduit end
set_interface_property PHYL ENABLED true
add_interface_port PHYL phy0_link export Input 1
add_interface_port PHYL phy1_link export Input 1

##Export Rmii Phy 0
add_interface RMII0 conduit end
set_interface_property RMII0 ENABLED true
add_interface_port RMII0 phy0_RxDat export Input 2
add_interface_port RMII0 phy0_RxDv export Input 1
add_interface_port RMII0 phy0_TxDat export Output 2
add_interface_port RMII0 phy0_TxEn export Output 1
add_interface_port RMII0 phy0_RxErr export Input 1

##Export Rmii Phy 1
add_interface RMII1 conduit end
set_interface_property RMII1 ENABLED true
add_interface_port RMII1 phy1_RxDat export Input 2
add_interface_port RMII1 phy1_RxDv export Input 1
add_interface_port RMII1 phy1_TxDat export Output 2
add_interface_port RMII1 phy1_TxEn export Output 1
add_interface_port RMII1 phy1_RxErr export Input 1

##Export Mii Phy 0
add_interface MII0 conduit end
set_interface_property MII0 ENABLED false
add_interface_port MII0 phyMii0_TxClk export Input 1
add_interface_port MII0 phyMii0_TxEn export Output 1
add_interface_port MII0 phyMii0_TxEr export Output 1
add_interface_port MII0 phyMii0_TxDat export Output 4
add_interface_port MII0 phyMii0_RxClk export Input 1
add_interface_port MII0 phyMii0_RxDv export Input 1
add_interface_port MII0 phyMii0_RxEr export Input 1
add_interface_port MII0 phyMii0_RxDat export Input 4

##Export Mii Phy 1
add_interface MII1 conduit end
set_interface_property MII1 ENABLED false
add_interface_port MII1 phyMii1_TxClk export Input 1
add_interface_port MII1 phyMii1_TxEn export Output 1
add_interface_port MII1 phyMii1_TxEr export Output 1
add_interface_port MII1 phyMii1_TxDat export Output 4
add_interface_port MII1 phyMii1_RxClk export Input 1
add_interface_port MII1 phyMii1_RxDv export Input 1
add_interface_port MII0 phyMii1_RxEr export Input 1
add_interface_port MII1 phyMii1_RxDat export Input 4

##Avalon Memory Mapped Slave: MAC_BUF Buffer
add_interface MAC_BUF avalon end
set_interface_property MAC_BUF addressAlignment DYNAMIC
set_interface_property MAC_BUF associatedClock clkPkt
set_interface_property MAC_BUF burstOnBurstBoundariesOnly false
set_interface_property MAC_BUF explicitAddressSpan 0
set_interface_property MAC_BUF holdTime 0
set_interface_property MAC_BUF isMemoryDevice false
set_interface_property MAC_BUF isNonVolatileStorage false
set_interface_property MAC_BUF linewrapBursts false
set_interface_property MAC_BUF maximumPendingReadTransactions 0
set_interface_property MAC_BUF printableDevice false
set_interface_property MAC_BUF readLatency 0
set_interface_property MAC_BUF readWaitStates 2
set_interface_property MAC_BUF readWaitTime 2
set_interface_property MAC_BUF setupTime 0
set_interface_property MAC_BUF timingUnits Cycles
set_interface_property MAC_BUF writeWaitTime 0
set_interface_property MAC_BUF ENABLED true
add_interface_port MAC_BUF mbf_chipselect chipselect Input 1
add_interface_port MAC_BUF mbf_read read Input 1
add_interface_port MAC_BUF mbf_write write Input 1
add_interface_port MAC_BUF mbf_byteenable byteenable Input 4
add_interface_port MAC_BUF mbf_address address Input "(iBufSizeLOG2_g-2)"
add_interface_port MAC_BUF mbf_writedata writedata Input 32
add_interface_port MAC_BUF mbf_readdata readdata Output 32
add_interface_port MAC_BUF mbf_waitrequest waitrequest Output 1

##Avalon Memory Mapped Master: MAC_DMA
add_interface MAC_DMA avalon start
set_interface_property MAC_DMA burstOnBurstBoundariesOnly false
#not yet supported: set_interface_property MAC_DMA constantBurstBehavior false
set_interface_property MAC_DMA linewrapBursts false
set_interface_property MAC_DMA ASSOCIATED_CLOCK clkMaster
set_interface_property MAC_DMA ENABLED false
add_interface_port MAC_DMA m_read read Output 1
add_interface_port MAC_DMA m_write write Output 1
add_interface_port MAC_DMA m_byteenable byteenable Output 2
add_interface_port MAC_DMA m_address address Output 30
add_interface_port MAC_DMA m_writedata writedata Output 16
add_interface_port MAC_DMA m_readdata readdata Input 16
add_interface_port MAC_DMA m_waitrequest waitrequest Input 1
add_interface_port MAC_DMA m_readdatavalid readdatavalid Input 1
add_interface_port MAC_DMA m_burstcount burstcount Output "m_burstcount_width_g"

#PDI
##Avalon Memory Mapped Slave: PCP
add_interface PDI_PCP avalon end
set_interface_property PDI_PCP addressAlignment DYNAMIC
set_interface_property PDI_PCP associatedClock pcp_clk
set_interface_property PDI_PCP burstOnBurstBoundariesOnly false
set_interface_property PDI_PCP explicitAddressSpan 0
set_interface_property PDI_PCP holdTime 0
set_interface_property PDI_PCP isMemoryDevice false
set_interface_property PDI_PCP isNonVolatileStorage false
set_interface_property PDI_PCP linewrapBursts false
set_interface_property PDI_PCP maximumPendingReadTransactions 0
set_interface_property PDI_PCP printableDevice false
set_interface_property PDI_PCP readLatency 0
set_interface_property PDI_PCP readWaitStates 2
set_interface_property PDI_PCP readWaitTime 2
set_interface_property PDI_PCP setupTime 0
set_interface_property PDI_PCP timingUnits Cycles
set_interface_property PDI_PCP writeWaitTime 0
set_interface_property PDI_PCP ENABLED true
add_interface_port PDI_PCP pcp_chipselect chipselect Input 1
add_interface_port PDI_PCP pcp_read read Input 1
add_interface_port PDI_PCP pcp_write write Input 1
add_interface_port PDI_PCP pcp_byteenable byteenable Input 4
add_interface_port PDI_PCP pcp_address address Input 13
add_interface_port PDI_PCP pcp_writedata writedata Input 32
add_interface_port PDI_PCP pcp_readdata readdata Output 32
add_interface_port PDI_PCP pcp_waitrequest waitrequest Output 1

##Avalon Memory Mapped Slave: AP
add_interface PDI_AP avalon end
set_interface_property PDI_AP addressAlignment DYNAMIC
set_interface_property PDI_AP associatedClock ap_clk
set_interface_property PDI_AP burstOnBurstBoundariesOnly false
set_interface_property PDI_AP explicitAddressSpan 0
set_interface_property PDI_AP holdTime 0
set_interface_property PDI_AP isMemoryDevice false
set_interface_property PDI_AP isNonVolatileStorage false
set_interface_property PDI_AP linewrapBursts false
set_interface_property PDI_AP maximumPendingReadTransactions 0
set_interface_property PDI_AP printableDevice false
set_interface_property PDI_AP readLatency 0
set_interface_property PDI_AP readWaitStates 2
set_interface_property PDI_AP readWaitTime 2
set_interface_property PDI_AP setupTime 0
set_interface_property PDI_AP timingUnits Cycles
set_interface_property PDI_AP writeWaitTime 0
set_interface_property PDI_AP ENABLED true
add_interface_port PDI_AP ap_chipselect chipselect Input 1
add_interface_port PDI_AP ap_read read Input 1
add_interface_port PDI_AP ap_write write Input 1
add_interface_port PDI_AP ap_byteenable byteenable Input 4
add_interface_port PDI_AP ap_address address Input 13
add_interface_port PDI_AP ap_writedata writedata Input 32
add_interface_port PDI_AP ap_readdata readdata Output 32
add_interface_port PDI_AP ap_waitrequest waitrequest Output 1

###PDI AP IRQ source
add_interface PDI_AP_IRQ interrupt end
set_interface_property PDI_AP_IRQ associatedAddressablePoint PDI_AP
set_interface_property PDI_AP_IRQ ASSOCIATED_CLOCK clk50meg
set_interface_property PDI_AP_IRQ ENABLED true
add_interface_port PDI_AP_IRQ ap_irq irq Output 1

##AP external IRQ
add_interface AP_EX_IRQ conduit end
set_interface_property AP_EX_IRQ ENABLED false
add_interface_port AP_EX_IRQ ap_syncIrq export Output 1
add_interface_port AP_EX_IRQ ap_syncIrq_n export Output 1
add_interface_port AP_EX_IRQ ap_asyncIrq export Output 1
add_interface_port AP_EX_IRQ ap_asyncIrq_n export Output 1

##SPI AP export
add_interface SPI_AP conduit end
set_interface_property SPI_AP ENABLED false
add_interface_port SPI_AP spi_clk export Input 1
add_interface_port SPI_AP spi_sel_n export Input 1
add_interface_port SPI_AP spi_mosi export Input 1
add_interface_port SPI_AP spi_miso export Output 1

##Parallel AP Interface export
add_interface PAR_AP conduit end
set_interface_property PAR_AP ENABLED false
###control signals
add_interface_port PAR_AP pap_cs export Input 1
add_interface_port PAR_AP pap_rd export Input 1
add_interface_port PAR_AP pap_wr export Input 1
add_interface_port PAR_AP pap_be export Input "papDataWidth_g/8"
add_interface_port PAR_AP pap_cs_n export Input 1
add_interface_port PAR_AP pap_rd_n export Input 1
add_interface_port PAR_AP pap_wr_n export Input 1
add_interface_port PAR_AP pap_be_n export Input "papDataWidth_g/8"
###bus
add_interface_port PAR_AP pap_addr export Input 16
add_interface_port PAR_AP pap_data export Bidir "papDataWidth_g"
###ack
add_interface_port PAR_AP pap_ack export Output 1
add_interface_port PAR_AP pap_ack_n export Output 1
###GPIO
add_interface_port PAR_AP pap_gpio export Bidir 2

#Simple I/O
##Avalon Memory Mapped Slave: SMP
add_interface SMP avalon end
set_interface_property SMP addressAlignment DYNAMIC
set_interface_property SMP associatedClock pcp_clk
set_interface_property SMP burstOnBurstBoundariesOnly false
set_interface_property SMP explicitAddressSpan 0
set_interface_property SMP holdTime 0
set_interface_property SMP isMemoryDevice false
set_interface_property SMP isNonVolatileStorage false
set_interface_property SMP linewrapBursts false
set_interface_property SMP maximumPendingReadTransactions 0
set_interface_property SMP printableDevice false
set_interface_property SMP readLatency 0
set_interface_property SMP readWaitTime 1
set_interface_property SMP setupTime 0
set_interface_property SMP timingUnits Cycles
set_interface_property SMP writeWaitTime 0
set_interface_property SMP ENABLED false
add_interface_port SMP smp_address address Input 1
add_interface_port SMP smp_read read Input 1
add_interface_port SMP smp_readdata readdata Output 32
add_interface_port SMP smp_write write Input 1
add_interface_port SMP smp_writedata writedata Input 32
add_interface_port SMP smp_byteenable byteenable Input 4

##Portio export
add_interface SMP_PIO conduit end
set_interface_property SMP_PIO ENABLED false
add_interface_port SMP_PIO pio_pconfig export Input 4
add_interface_port SMP_PIO pio_portInLatch export Input 4
add_interface_port SMP_PIO pio_portOutValid export Output 4
add_interface_port SMP_PIO pio_portio export Bidir 32
add_interface_port SMP_PIO pio_operational export Output 1

#LED gadget
add_interface LED_GADGET conduit end
set_interface_property LED_GADGET ENABLED false
add_interface_port LED_GADGET led_error export Output 1
add_interface_port LED_GADGET led_status export Output 1
add_interface_port LED_GADGET led_phyLink export Output 2
add_interface_port LED_GADGET led_phyAct export Output 2
add_interface_port LED_GADGET led_opt export Output 2
add_interface_port LED_GADGET led_gpo export Output 8

proc my_elaboration_callback {} {
#get system info...
set EthernetClkRate [get_parameter_value clkRateEth]
set ClkRate50meg [get_parameter_value clkRate50]

#valid set
set ClkPcp [get_parameter_value clkRatePcp]
if {$ClkPcp == 0} {
	# avoid 1 / zero!
	set ClkPcp 1
}
set ClkPcpPeriod [expr 1. / $ClkPcp * 1000 * 1000 * 1000]
set validTicks [get_parameter_value validSet]
if {$validTicks <= 0} {
	set validTicks 1
}
set validLength [expr $validTicks * $ClkPcpPeriod]

set_parameter_value validAssertDuration $validLength
set_parameter_value pioValLen_g $validTicks

if {$ClkRate50meg == 50000000} {

} else {
	send_message error "MAC_CMP and MAC_REG must be connected to 50MHz Clock!"
}

#if {$ClkPcp == 50000000} {
#
#} else {
#	send_message error "PDI must be connected to 50MHz Clock!"
#}

#find out, which interfaces (avalon, exports, etc) are not necessary for the configurated device!
	#set defaults
	set_interface_property ap_clk ENABLED false
	set_interface_property PDI_PCP ENABLED false
	set_interface_property PDI_AP ENABLED false
	set_interface_property PDI_AP_IRQ ENABLED false
	set_interface_property SPI_AP ENABLED false
	set_interface_property PAR_AP ENABLED false
	set_interface_property SMP ENABLED false
	set_interface_property SMP_PIO ENABLED false
	set_interface_property AP_EX_IRQ ENABLED false
	set_interface_property LED_GADGET ENABLED false
	
	set_interface_property MAC_DMA ENABLED false
	set_interface_property clkMaster ENABLED false
	set_interface_property MAC_BUF ENABLED false
	set_interface_property clkPkt ENABLED false
	
	if {[get_parameter_value macTxBurstSize] > 1 || [get_parameter_value macRxBurstSize] > 1} {
		#we want to burst!
		set_port_property m_burstcount termination false
	} else {
		#don't want to burst!
		set_port_property m_burstcount termination true
	}
	
	#verify which packet location is set and disable/enable dma/dpr
	if {[get_parameter_value packetLoc] == "TX and RX into DPRAM"} {
		#use internal packet buffering
		set_interface_property MAC_BUF ENABLED true
		set_interface_property clkPkt ENABLED true
	} elseif {[get_parameter_value packetLoc] == "TX into DPRAM and RX over Avalon Master"} {
		#use internal packet buffering
		set_interface_property MAC_BUF ENABLED true
		set_interface_property clkPkt ENABLED true
		#use DMA for Rx packets
		set_interface_property MAC_DMA ENABLED true
		set_interface_property clkMaster ENABLED true
	} elseif {[get_parameter_value packetLoc] == "TX and RX over Avalon Master"} {
		#use external packet buffering
		set_interface_property MAC_DMA ENABLED true
		set_interface_property clkMaster ENABLED true
	} else {
		send_message error "error 0x04"
	}
	
	if {[get_parameter_value useRmii_g]} {
		set_interface_property RMII0 ENABLED true
		set_interface_property RMII1 ENABLED true
		set_interface_property MII0 ENABLED false
		set_interface_property MII1 ENABLED false
		set_interface_property clkEth ENABLED true
		if {$EthernetClkRate == 100000000} {
		
		} else {
			send_message error "Clock Source of 100MHz required!"
		}
	} else {
		set_interface_property RMII0 ENABLED false
		set_interface_property RMII1 ENABLED false
		set_interface_property MII0 ENABLED true
		set_interface_property MII1 ENABLED true
		set_interface_property clkEth ENABLED false
	}
	
	#okay, maybe only one phy port is set by the user!?
	if {[get_parameter_value mac2phys]} {
		#yes, two phys please!
		#do nothing here, it is already set correctly above ;)
		#only not terminate the phy link input
		set_port_property phy1_link termination false
        #do we need 2nd SMI!?
        if {[get_parameter_value macGen2ndSmi]} {
            #yes we do, enable PHYM0 and PHYM1
            # and disable PHYM
            set_interface_property PHYM0 ENABLED true
            set_interface_property PHYM1 ENABLED true
            set_interface_property PHYM ENABLED false
        } else {
            #no, terminate 2nd SMI
            set_interface_property PHYM0 ENABLED false
            set_interface_property PHYM1 ENABLED false
            set_interface_property PHYM ENABLED true
        }
	} else {
		#no, leave me one phy only!
        set_interface_property PHYM ENABLED true
		set_interface_property RMII1 ENABLED false
		set_interface_property MII1 ENABLED false
		#phy management (0 + 1) can be omitted too...
        set_interface_property PHYM0 ENABLED false
        set_interface_property PHYM1 ENABLED false
		set_port_property phy1_link termination true
	}
	
	#if the MAC DMA master only write data to memory (RX), then we can disable read and readdata
	if {[get_parameter_value packetLoc] == "TX into DPRAM and RX over Avalon Master"} {
		set_port_property m_read termination true
		set_port_property m_readdata termination true
		set_port_property m_readdatavalid termination true
	}
	
	if {[get_parameter_value configPowerlink] == "openMAC only"} {
		#don't need pcp_clk
		set_interface_property pcp_clk ENABLED false
	} elseif {[get_parameter_value configPowerlink] == "Direct I/O CN"} {
		#the Direct I/O CN requires:
		# MAC stuff
		# portio export
		# Avalon SMP
		set_interface_property SMP ENABLED true
		set_interface_property SMP_PIO ENABLED true
	} else {
		#CN with Processor Interface requires:
		# MAC stuff
		# PDI_PCP
		set_interface_property PDI_PCP ENABLED true
		
		if {[get_parameter_value configApInterface] == "Avalon"} {
		# AP as Avalon (PDI_AP)
			set_interface_property PDI_AP ENABLED true
			set_interface_property PDI_AP_IRQ ENABLED true
			set_interface_property ap_clk ENABLED true
			
			if {[get_parameter_value genLedGadget]} {
				set_interface_property LED_GADGET ENABLED true
			}
			
		} elseif {[get_parameter_value configApInterface] == "Parallel"} {
		# AP is external (PAR_AP)
			set_interface_property PAR_AP ENABLED true
			set_interface_property AP_EX_IRQ ENABLED true
			
			if {[get_parameter_value genLedGadget]} {
				set_interface_property LED_GADGET ENABLED true
			}
			
			if {[get_parameter_value papDataWidth_g] == 8} {
				#we don't need byteenable for 8bit data bus width!
				set_port_property pap_be termination true
			}
			if {[get_parameter_value configApParOutSigs] == "Low Active"} {
				#low active output signals (ap_irq_n and pap_ack_n) are used
				set_port_property pap_ack termination true
				set_port_property ap_syncIrq termination true
				set_port_property ap_asyncIrq termination true
			} else {
				#high active output signals (ap_irq and pap_ack) are used
				set_port_property pap_ack_n termination true
				set_port_property ap_syncIrq_n termination true
				set_port_property ap_asyncIrq_n termination true
			}
			
			#if event support disabled terminate async irq
			if {[get_parameter_value genEvent_g]} {
			
			} else {
				set_port_property ap_asyncIrq termination true
				set_port_property ap_asyncIrq_n termination true
			}
			
			if {[get_parameter_value configApParSigs] == "Low Active"} {
				#low active input signals (pap_cs_n, pap_rd_n, pap_wr_n and pap_be_n) are used
				set_port_property pap_cs termination true
				set_port_property pap_rd termination true
				set_port_property pap_wr termination true
				set_port_property pap_be termination true
			} else {
				#high active input signals (pap_cs, pap_rd, pap_wr and pap_be) are used
				set_port_property pap_cs_n termination true
				set_port_property pap_rd_n termination true
				set_port_property pap_wr_n termination true
				set_port_property pap_be_n termination true
			}
		} elseif {[get_parameter_value configApInterface] == "SPI"} {
		# AP is external via SPI (SPI_AP)
			set_interface_property SPI_AP ENABLED true
			set_interface_property AP_EX_IRQ ENABLED true
			
			if {[get_parameter_value genLedGadget]} {
				set_interface_property LED_GADGET ENABLED true
			}
			
			if {[get_parameter_value configApSpi_IRQ] == "Low Active"} {
				#low active output signal (irq_n) is used
				set_port_property ap_syncIrq termination true
				set_port_property ap_asyncIrq termination true
			} else {
				#high active output signal (irq) is used
				set_port_property ap_syncIrq_n termination true
				set_port_property ap_asyncIrq_n termination true
			}
			
			#if event support disabled terminate async irq
			if {[get_parameter_value genEvent_g]} {
			
			} else {
				set_port_property ap_asyncIrq termination true
				set_port_property ap_asyncIrq_n termination true
			}
		}
	}
}
