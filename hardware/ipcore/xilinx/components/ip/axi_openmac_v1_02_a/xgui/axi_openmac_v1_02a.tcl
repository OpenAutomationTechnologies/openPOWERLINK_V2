
# Loading additional proc with user specified bodies to compute parameter values.
source [file join [file dirname [file dirname [info script]]] gui/axi_openmac_v1_02a.gtcl]

# Definitional proc to organize widgets for parameters.
proc init_gui { IPINST } {
  set Component_Name  [  ipgui::add_param $IPINST -name "Component_Name" -display_name {Component Name}]
  set_property tooltip {Component Name} ${Component_Name}
  #Adding Page
  set User  [  ipgui::add_page $IPINST -name "User" -display_name {User}]
  set_property tooltip {User} ${User}
  #Adding Group
  set Phy_Interface  [  ipgui::add_group $IPINST -name "Phy Interface" -parent ${User} -display_name {Phy Interface}]
  set_property tooltip {Phy Interface} ${Phy_Interface}
  set gui_phyType  [  ipgui::add_param $IPINST -name "gui_phyType" -parent ${Phy_Interface} -display_name {Phy(s) interface type} -widget comboBox]
  set_property tooltip {Select the Phy(s) Media Independent Interface type.  Note that RMII is recommended since no extra resources are necessary!} ${gui_phyType}
  set gui_phyCount  [  ipgui::add_param $IPINST -name "gui_phyCount" -parent ${Phy_Interface} -display_name {Number of Phys}]
  set_property tooltip {Set the number of connected Phys.} ${gui_phyCount}
  set gui_extraSmi  [  ipgui::add_param $IPINST -name "gui_extraSmi" -parent ${Phy_Interface} -display_name {Extra SMI ports} -widget comboBox]
  set_property tooltip {Set this option to TRUE if every connected Phy is connected via an extra SMI connection. If all connected Phys share the same SMI, set the option to FALSE.} ${gui_extraSmi}

  #Adding Group
  set Packet_Buffer  [  ipgui::add_group $IPINST -name "Packet Buffer" -parent ${User} -display_name {Packet Buffer}]
  set_property tooltip {Packet Buffer} ${Packet_Buffer}
  set gui_txBufLoc  [  ipgui::add_param $IPINST -name "gui_txBufLoc" -parent ${Packet_Buffer} -display_name {Tx Buffer Location} -widget comboBox]
  set_property tooltip {Select the Tx buffer location to LOCAL or EXTERNAL. The LOCAL configuration allocates the Tx buffers in BRAM resources - this configuration is preferred if the external memory is having a dynamic access latency. The EXTERNAL configuration inserts an AXI master to fetch Tx frames from external memory - this configuration requires a fast memory connection.} ${gui_txBufLoc}
  set gui_txBufSize  [  ipgui::add_param $IPINST -name "gui_txBufSize" -parent ${Packet_Buffer} -display_name {Tx Buffer Size [KiB]}]
  set_property tooltip {Set the LOCAL Tx buffer size in KiB. Note that this setting affects the BRAM resource utilization!} ${gui_txBufSize}
  set gui_txBurstSize  [  ipgui::add_param $IPINST -name "gui_txBurstSize" -parent ${Packet_Buffer} -display_name {Tx Burst Size [WORD]} -widget comboBox]
  set_property tooltip {Set the number of words transferred in each burst if the Tx buffer location is EXTERNAL.} ${gui_txBurstSize}
  set gui_rxBufLoc  [  ipgui::add_param $IPINST -name "gui_rxBufLoc" -parent ${Packet_Buffer} -display_name {Rx Buffer Location} -widget comboBox]
  set_property tooltip {Select the Rx buffer location to LOCAL or EXTERNAL. The LOCAL configuration allocates the Rx buffers in BRAM resources - this configuration is preferred if enough BRAM resources are available. The EXTERNAL configuration inserts an AXI master to transfer Rx frames to external memory - this configuration is preferred to save BRAM resources.} ${gui_rxBufLoc}
  set gui_rxBufSize  [  ipgui::add_param $IPINST -name "gui_rxBufSize" -parent ${Packet_Buffer} -display_name {Rx Buffer Size [KiB]}]
  set_property tooltip {Set the LOCAL Rx buffer size in KiB. Note that this setting affects the BRAM resource utilization!} ${gui_rxBufSize}
  set gui_rxBurstSize  [  ipgui::add_param $IPINST -name "gui_rxBurstSize" -parent ${Packet_Buffer} -display_name {Rx Burst Size [WORD]} -widget comboBox]
  set_property tooltip {Set the number of words transferred in each burst if the Rx buffer location is EXTERNAL.} ${gui_rxBurstSize}

  #Adding Group
  set Timer  [  ipgui::add_group $IPINST -name "Timer" -parent ${User} -display_name {Timer}]
  set_property tooltip {Timer} ${Timer}
  set gui_tmrPulse  [  ipgui::add_param $IPINST -name "gui_tmrPulse" -parent ${Timer} -display_name {Enable Timer Pulse} -widget comboBox]
  set_property tooltip {Enable the optional timer for pulse generation. It can be used for external synchronization.} ${gui_tmrPulse}
  set gui_tmrPulseEn  [  ipgui::add_param $IPINST -name "gui_tmrPulseEn" -parent ${Timer} -display_name {Timer Pulse Width Control} -widget comboBox]
  set_property tooltip {Enable the timer pulse width control of timer 2. Otherwise the generated pulse is asserted only for one clock cycle.} ${gui_tmrPulseEn}
  set gui_tmrPulseWdt  [  ipgui::add_param $IPINST -name "gui_tmrPulseWdt" -parent ${Timer} -display_name {Timer Pulse Width register width}]
  set_property tooltip {This setting is only valid if "Timer Pulse Width Control" is enabled! Determine the timer 2 pulse width control register width. Example: Generate a pulse of 1 us (fclk=50 MHz) requires 1 us * 50 MHz = 50 ticks. To generate 50 ticks a width of log2(50) ~ 6 is needed.} ${gui_tmrPulseWdt}

  #Adding Group
  set Others  [  ipgui::add_group $IPINST -name "Others" -parent ${User} -display_name {Others}]
  set_property tooltip {Others} ${Others}
  set gui_actEn  [  ipgui::add_param $IPINST -name "gui_actEn" -parent ${Others} -display_name {Packet activity LED} -widget comboBox]
  set_property tooltip {Use the MAC RMII signals to generate an activity signal. It can be used to drive LEDs.} ${gui_actEn}


  #Adding Page
  set Page_0  [  ipgui::add_page $IPINST -name "Page 0" -display_name {System}]
  set_property tooltip {System} ${Page_0}
  #Adding Group
  set Addresses  [  ipgui::add_group $IPINST -name "Addresses" -parent ${Page_0} -display_name {Addresses}]
  set_property tooltip {Addresses} ${Addresses}
  set C_S_AXI_MAC_REG_RNG0_BASEADDR  [  ipgui::add_param $IPINST -name "C_S_AXI_MAC_REG_RNG0_BASEADDR" -parent ${Addresses} -display_name {C_S_AXI_MAC_REG_RNG0_BASEADDR}]
  set_property tooltip {C_S_AXI_MAC_REG_RNG0_BASEADDR} ${C_S_AXI_MAC_REG_RNG0_BASEADDR}
  set C_S_AXI_MAC_REG_RNG0_HIGHADDR  [  ipgui::add_param $IPINST -name "C_S_AXI_MAC_REG_RNG0_HIGHADDR" -parent ${Addresses} -display_name {C_S_AXI_MAC_REG_RNG0_HIGHADDR}]
  set_property tooltip {C_S_AXI_MAC_REG_RNG0_HIGHADDR} ${C_S_AXI_MAC_REG_RNG0_HIGHADDR}
  set C_S_AXI_MAC_REG_RNG1_BASEADDR  [  ipgui::add_param $IPINST -name "C_S_AXI_MAC_REG_RNG1_BASEADDR" -parent ${Addresses} -display_name {C_S_AXI_MAC_REG_RNG1_BASEADDR}]
  set_property tooltip {C_S_AXI_MAC_REG_RNG1_BASEADDR} ${C_S_AXI_MAC_REG_RNG1_BASEADDR}
  set C_S_AXI_MAC_REG_RNG1_HIGHADDR  [  ipgui::add_param $IPINST -name "C_S_AXI_MAC_REG_RNG1_HIGHADDR" -parent ${Addresses} -display_name {C_S_AXI_MAC_REG_RNG1_HIGHADDR}]
  set_property tooltip {C_S_AXI_MAC_REG_RNG1_HIGHADDR} ${C_S_AXI_MAC_REG_RNG1_HIGHADDR}
  set C_S_AXI_MAC_PKT_BASEADDR  [  ipgui::add_param $IPINST -name "C_S_AXI_MAC_PKT_BASEADDR" -parent ${Addresses} -display_name {C_S_AXI_MAC_PKT_BASEADDR}]
  set_property tooltip {C_S_AXI_MAC_PKT_BASEADDR} ${C_S_AXI_MAC_PKT_BASEADDR}
  set C_S_AXI_MAC_PKT_HIGHADDR  [  ipgui::add_param $IPINST -name "C_S_AXI_MAC_PKT_HIGHADDR" -parent ${Addresses} -display_name {C_S_AXI_MAC_PKT_HIGHADDR}]
  set_property tooltip {C_S_AXI_MAC_PKT_HIGHADDR} ${C_S_AXI_MAC_PKT_HIGHADDR}

  #Adding Group
  set AXI_MAC_DMA  [  ipgui::add_group $IPINST -name "AXI MAC DMA" -parent ${Page_0} -display_name {AXI MAC DMA}]
  set_property tooltip {AXI MAC DMA} ${AXI_MAC_DMA}
  set C_M_AXI_MAC_DMA_ADDR_WIDTH  [  ipgui::add_param $IPINST -name "C_M_AXI_MAC_DMA_ADDR_WIDTH" -parent ${AXI_MAC_DMA} -display_name {C_M_AXI_MAC_DMA_ADDR_WIDTH}]
  set_property tooltip {C_M_AXI_MAC_DMA_ADDR_WIDTH} ${C_M_AXI_MAC_DMA_ADDR_WIDTH}
  set C_M_AXI_MAC_DMA_DATA_WIDTH  [  ipgui::add_param $IPINST -name "C_M_AXI_MAC_DMA_DATA_WIDTH" -parent ${AXI_MAC_DMA} -display_name {C_M_AXI_MAC_DMA_DATA_WIDTH}]
  set_property tooltip {C_M_AXI_MAC_DMA_DATA_WIDTH} ${C_M_AXI_MAC_DMA_DATA_WIDTH}
  set C_M_AXI_MAC_DMA_NATIVE_DWIDTH  [  ipgui::add_param $IPINST -name "C_M_AXI_MAC_DMA_NATIVE_DWIDTH" -parent ${AXI_MAC_DMA} -display_name {C_M_AXI_MAC_DMA_NATIVE_DWIDTH} -widget comboBox]
  set_property tooltip {C_M_AXI_MAC_DMA_NATIVE_DWIDTH} ${C_M_AXI_MAC_DMA_NATIVE_DWIDTH}
  set C_M_AXI_MAC_DMA_LENGTH_WIDTH  [  ipgui::add_param $IPINST -name "C_M_AXI_MAC_DMA_LENGTH_WIDTH" -parent ${AXI_MAC_DMA} -display_name {C_M_AXI_MAC_DMA_LENGTH_WIDTH} -widget comboBox]
  set_property tooltip {C_M_AXI_MAC_DMA_LENGTH_WIDTH} ${C_M_AXI_MAC_DMA_LENGTH_WIDTH}
  set C_M_AXI_MAC_DMA_MAX_BURST_LEN  [  ipgui::add_param $IPINST -name "C_M_AXI_MAC_DMA_MAX_BURST_LEN" -parent ${AXI_MAC_DMA} -display_name {C_M_AXI_MAC_DMA_MAX_BURST_LEN}]
  set_property tooltip {C_M_AXI_MAC_DMA_MAX_BURST_LEN} ${C_M_AXI_MAC_DMA_MAX_BURST_LEN}
  set C_M_AXI_MAC_DMA_PROTOCOL  [  ipgui::add_param $IPINST -name "C_M_AXI_MAC_DMA_PROTOCOL" -parent ${AXI_MAC_DMA} -display_name {C_M_AXI_MAC_DMA_PROTOCOL}]
  set_property tooltip {C_M_AXI_MAC_DMA_PROTOCOL} ${C_M_AXI_MAC_DMA_PROTOCOL}

  #Adding Group
  set AXI_MAC_REG  [  ipgui::add_group $IPINST -name "AXI MAC REG" -parent ${Page_0} -display_name {AXI MAC REG}]
  set_property tooltip {AXI MAC REG} ${AXI_MAC_REG}
  set C_S_AXI_MAC_REG_NUM_ADDR_RANGES  [  ipgui::add_param $IPINST -name "C_S_AXI_MAC_REG_NUM_ADDR_RANGES" -parent ${AXI_MAC_REG} -display_name {C_S_AXI_MAC_REG_NUM_ADDR_RANGES}]
  set_property tooltip {C_S_AXI_MAC_REG_NUM_ADDR_RANGES} ${C_S_AXI_MAC_REG_NUM_ADDR_RANGES}
  set C_S_AXI_MAC_REG_MIN_SIZE  [  ipgui::add_param $IPINST -name "C_S_AXI_MAC_REG_MIN_SIZE" -parent ${AXI_MAC_REG} -display_name {C_S_AXI_MAC_REG_MIN_SIZE}]
  set_property tooltip {C_S_AXI_MAC_REG_MIN_SIZE} ${C_S_AXI_MAC_REG_MIN_SIZE}
  set C_S_AXI_MAC_REG_DATA_WIDTH  [  ipgui::add_param $IPINST -name "C_S_AXI_MAC_REG_DATA_WIDTH" -parent ${AXI_MAC_REG} -display_name {C_S_AXI_MAC_REG_DATA_WIDTH}]
  set_property tooltip {C_S_AXI_MAC_REG_DATA_WIDTH} ${C_S_AXI_MAC_REG_DATA_WIDTH}
  set C_S_AXI_MAC_REG_ADDR_WIDTH  [  ipgui::add_param $IPINST -name "C_S_AXI_MAC_REG_ADDR_WIDTH" -parent ${AXI_MAC_REG} -display_name {C_S_AXI_MAC_REG_ADDR_WIDTH}]
  set_property tooltip {C_S_AXI_MAC_REG_ADDR_WIDTH} ${C_S_AXI_MAC_REG_ADDR_WIDTH}
  set C_S_AXI_MAC_REG_ACLK_FREQ_HZ  [  ipgui::add_param $IPINST -name "C_S_AXI_MAC_REG_ACLK_FREQ_HZ" -parent ${AXI_MAC_REG} -display_name {C_S_AXI_MAC_REG_ACLK_FREQ_HZ}]
  set_property tooltip {C_S_AXI_MAC_REG_ACLK_FREQ_HZ} ${C_S_AXI_MAC_REG_ACLK_FREQ_HZ}
  set C_S_AXI_MAC_REG_USE_WSTRB  [  ipgui::add_param $IPINST -name "C_S_AXI_MAC_REG_USE_WSTRB" -parent ${AXI_MAC_REG} -display_name {C_S_AXI_MAC_REG_USE_WSTRB}]
  set_property tooltip {C_S_AXI_MAC_REG_USE_WSTRB} ${C_S_AXI_MAC_REG_USE_WSTRB}
  set C_S_AXI_MAC_REG_DPHASE_TIMEOUT  [  ipgui::add_param $IPINST -name "C_S_AXI_MAC_REG_DPHASE_TIMEOUT" -parent ${AXI_MAC_REG} -display_name {C_S_AXI_MAC_REG_DPHASE_TIMEOUT}]
  set_property tooltip {C_S_AXI_MAC_REG_DPHASE_TIMEOUT} ${C_S_AXI_MAC_REG_DPHASE_TIMEOUT}
  set C_S_AXI_MAC_REG_PROTOCOL  [  ipgui::add_param $IPINST -name "C_S_AXI_MAC_REG_PROTOCOL" -parent ${AXI_MAC_REG} -display_name {C_S_AXI_MAC_REG_PROTOCOL}]
  set_property tooltip {C_S_AXI_MAC_REG_PROTOCOL} ${C_S_AXI_MAC_REG_PROTOCOL}
  set C_S_AXI_MAC_REG_CLK_XING  [  ipgui::add_param $IPINST -name "C_S_AXI_MAC_REG_CLK_XING" -parent ${AXI_MAC_REG} -display_name {C_S_AXI_MAC_REG_CLK_XING} -widget comboBox]
  set_property tooltip {C_S_AXI_MAC_REG_CLK_XING} ${C_S_AXI_MAC_REG_CLK_XING}

  #Adding Group
  set AXI_MAC_PKT  [  ipgui::add_group $IPINST -name "AXI MAC PKT" -parent ${Page_0} -display_name {AXI MAC PKT}]
  set_property tooltip {AXI MAC PKT} ${AXI_MAC_PKT}
  set C_S_AXI_MAC_PKT_MIN_SIZE  [  ipgui::add_param $IPINST -name "C_S_AXI_MAC_PKT_MIN_SIZE" -parent ${AXI_MAC_PKT} -display_name {C_S_AXI_MAC_PKT_MIN_SIZE}]
  set_property tooltip {C_S_AXI_MAC_PKT_MIN_SIZE} ${C_S_AXI_MAC_PKT_MIN_SIZE}
  set C_S_AXI_MAC_PKT_DATA_WIDTH  [  ipgui::add_param $IPINST -name "C_S_AXI_MAC_PKT_DATA_WIDTH" -parent ${AXI_MAC_PKT} -display_name {C_S_AXI_MAC_PKT_DATA_WIDTH}]
  set_property tooltip {C_S_AXI_MAC_PKT_DATA_WIDTH} ${C_S_AXI_MAC_PKT_DATA_WIDTH}
  set C_S_AXI_MAC_PKT_ADDR_WIDTH  [  ipgui::add_param $IPINST -name "C_S_AXI_MAC_PKT_ADDR_WIDTH" -parent ${AXI_MAC_PKT} -display_name {C_S_AXI_MAC_PKT_ADDR_WIDTH}]
  set_property tooltip {C_S_AXI_MAC_PKT_ADDR_WIDTH} ${C_S_AXI_MAC_PKT_ADDR_WIDTH}
  set C_S_AXI_MAC_PKT_USE_WSTRB  [  ipgui::add_param $IPINST -name "C_S_AXI_MAC_PKT_USE_WSTRB" -parent ${AXI_MAC_PKT} -display_name {C_S_AXI_MAC_PKT_USE_WSTRB}]
  set_property tooltip {C_S_AXI_MAC_PKT_USE_WSTRB} ${C_S_AXI_MAC_PKT_USE_WSTRB}
  set C_S_AXI_MAC_PKT_DPHASE_TIMEOUT  [  ipgui::add_param $IPINST -name "C_S_AXI_MAC_PKT_DPHASE_TIMEOUT" -parent ${AXI_MAC_PKT} -display_name {C_S_AXI_MAC_PKT_DPHASE_TIMEOUT}]
  set_property tooltip {C_S_AXI_MAC_PKT_DPHASE_TIMEOUT} ${C_S_AXI_MAC_PKT_DPHASE_TIMEOUT}
  set C_S_AXI_MAC_PKT_PROTOCOL  [  ipgui::add_param $IPINST -name "C_S_AXI_MAC_PKT_PROTOCOL" -parent ${AXI_MAC_PKT} -display_name {C_S_AXI_MAC_PKT_PROTOCOL}]
  set_property tooltip {C_S_AXI_MAC_PKT_PROTOCOL} ${C_S_AXI_MAC_PKT_PROTOCOL}


  #Adding Page
  set VHDL_Paramete  [  ipgui::add_page $IPINST -name "VHDL Paramete" -display_name {VHDL Configuration}]
  set_property tooltip {VHDL Configuration} ${VHDL_Paramete}
  set gPhyPortCount  [  ipgui::add_param $IPINST -name "gPhyPortCount" -parent ${VHDL_Paramete} -display_name {gPhyPortCount}]
  set_property tooltip {gPhyPortCount} ${gPhyPortCount}
  set gPhyPortType  [  ipgui::add_param $IPINST -name "gPhyPortType" -parent ${VHDL_Paramete} -display_name {gPhyPortType}]
  set_property tooltip {gPhyPortType} ${gPhyPortType}
  set gSmiPortCount  [  ipgui::add_param $IPINST -name "gSmiPortCount" -parent ${VHDL_Paramete} -display_name {gSmiPortCount}]
  set_property tooltip {gSmiPortCount} ${gSmiPortCount}
  set gEndianness  [  ipgui::add_param $IPINST -name "gEndianness" -parent ${VHDL_Paramete} -display_name {gEndianness}]
  set_property tooltip {gEndianness} ${gEndianness}
  set gEnableActivity  [  ipgui::add_param $IPINST -name "gEnableActivity" -parent ${VHDL_Paramete} -display_name {gEnableActivity}]
  set_property tooltip {gEnableActivity} ${gEnableActivity}
  set gEnableDmaObserver  [  ipgui::add_param $IPINST -name "gEnableDmaObserver" -parent ${VHDL_Paramete} -display_name {gEnableDmaObserver}]
  set_property tooltip {gEnableDmaObserver} ${gEnableDmaObserver}
  set gDmaAddrWidth  [  ipgui::add_param $IPINST -name "gDmaAddrWidth" -parent ${VHDL_Paramete} -display_name {gDmaAddrWidth}]
  set_property tooltip {gDmaAddrWidth} ${gDmaAddrWidth}
  set gDmaDataWidth  [  ipgui::add_param $IPINST -name "gDmaDataWidth" -parent ${VHDL_Paramete} -display_name {gDmaDataWidth}]
  set_property tooltip {gDmaDataWidth} ${gDmaDataWidth}
  set gDmaBurstCountWidth  [  ipgui::add_param $IPINST -name "gDmaBurstCountWidth" -parent ${VHDL_Paramete} -display_name {gDmaBurstCountWidth}]
  set_property tooltip {gDmaBurstCountWidth} ${gDmaBurstCountWidth}
  set gDmaWriteBurstLength  [  ipgui::add_param $IPINST -name "gDmaWriteBurstLength" -parent ${VHDL_Paramete} -display_name {gDmaWriteBurstLength}]
  set_property tooltip {gDmaWriteBurstLength} ${gDmaWriteBurstLength}
  set gDmaReadBurstLength  [  ipgui::add_param $IPINST -name "gDmaReadBurstLength" -parent ${VHDL_Paramete} -display_name {gDmaReadBurstLength}]
  set_property tooltip {gDmaReadBurstLength} ${gDmaReadBurstLength}
  set gDmaWriteFifoLength  [  ipgui::add_param $IPINST -name "gDmaWriteFifoLength" -parent ${VHDL_Paramete} -display_name {gDmaWriteFifoLength}]
  set_property tooltip {gDmaWriteFifoLength} ${gDmaWriteFifoLength}
  set gDmaReadFifoLength  [  ipgui::add_param $IPINST -name "gDmaReadFifoLength" -parent ${VHDL_Paramete} -display_name {gDmaReadFifoLength}]
  set_property tooltip {gDmaReadFifoLength} ${gDmaReadFifoLength}
  set gPacketBufferLocTx  [  ipgui::add_param $IPINST -name "gPacketBufferLocTx" -parent ${VHDL_Paramete} -display_name {gPacketBufferLocTx}]
  set_property tooltip {gPacketBufferLocTx} ${gPacketBufferLocTx}
  set gPacketBufferLocRx  [  ipgui::add_param $IPINST -name "gPacketBufferLocRx" -parent ${VHDL_Paramete} -display_name {gPacketBufferLocRx}]
  set_property tooltip {gPacketBufferLocRx} ${gPacketBufferLocRx}
  set gPacketBufferLog2Size  [  ipgui::add_param $IPINST -name "gPacketBufferLog2Size" -parent ${VHDL_Paramete} -display_name {gPacketBufferLog2Size}]
  set_property tooltip {gPacketBufferLog2Size} ${gPacketBufferLog2Size}
  set gTimerEnablePulse  [  ipgui::add_param $IPINST -name "gTimerEnablePulse" -parent ${VHDL_Paramete} -display_name {gTimerEnablePulse}]
  set_property tooltip {gTimerEnablePulse} ${gTimerEnablePulse}
  set gTimerEnablePulseWidth  [  ipgui::add_param $IPINST -name "gTimerEnablePulseWidth" -parent ${VHDL_Paramete} -display_name {gTimerEnablePulseWidth}]
  set_property tooltip {gTimerEnablePulseWidth} ${gTimerEnablePulseWidth}
  set gTimerPulseRegWidth  [  ipgui::add_param $IPINST -name "gTimerPulseRegWidth" -parent ${VHDL_Paramete} -display_name {gTimerPulseRegWidth}]
  set_property tooltip {gTimerPulseRegWidth} ${gTimerPulseRegWidth}
  set Empty  [  ipgui::add_static_text $IPINST -name "Empty" -parent ${VHDL_Paramete} -text {}]
  set_property tooltip {Empty} ${Empty}
  set Info  [  ipgui::add_static_text $IPINST -name "Info" -parent ${VHDL_Paramete} -text {Info: All parameter on this page are calculated and read only. }]
  set_property tooltip {Info} ${Info}


}

proc update_PARAM_VALUE.C_M_AXI_MAC_DMA_MAX_BURST_LEN { PARAM_VALUE.C_M_AXI_MAC_DMA_MAX_BURST_LEN PARAM_VALUE.gui_txBufLoc PARAM_VALUE.gui_txBurstSize PARAM_VALUE.gui_rxBufLoc PARAM_VALUE.gui_rxBurstSize } {
    # Procedure called to update C_M_AXI_MAC_DMA_MAX_BURST_LEN when any of the dependent parameters in the arguments change

    set C_M_AXI_MAC_DMA_MAX_BURST_LEN ${PARAM_VALUE.C_M_AXI_MAC_DMA_MAX_BURST_LEN}
    set gui_txBufLoc ${PARAM_VALUE.gui_txBufLoc}
    set gui_txBurstSize ${PARAM_VALUE.gui_txBurstSize}
    set gui_rxBufLoc ${PARAM_VALUE.gui_rxBufLoc}
    set gui_rxBurstSize ${PARAM_VALUE.gui_rxBurstSize}
    set values(gui_txBufLoc) [get_property value $gui_txBufLoc]
    set values(gui_txBurstSize) [get_property value $gui_txBurstSize]
    set values(gui_rxBufLoc) [get_property value $gui_rxBufLoc]
    set values(gui_rxBurstSize) [get_property value $gui_rxBurstSize]
    set_property value [gen_USERPARAMETER_C_M_AXI_MAC_DMA_MAX_BURST_LEN_VALUE $values(gui_txBufLoc) $values(gui_txBurstSize) $values(gui_rxBufLoc) $values(gui_rxBurstSize)] $C_M_AXI_MAC_DMA_MAX_BURST_LEN
}

proc validate_PARAM_VALUE.C_M_AXI_MAC_DMA_MAX_BURST_LEN { PARAM_VALUE.C_M_AXI_MAC_DMA_MAX_BURST_LEN } {
    # Procedure called to validate C_M_AXI_MAC_DMA_MAX_BURST_LEN
    return true
}

proc update_PARAM_VALUE.C_S_AXI_MAC_PKT_BASEADDR { PARAM_VALUE.C_S_AXI_MAC_PKT_BASEADDR PARAM_VALUE.gui_txBufLoc PARAM_VALUE.gui_rxBufLoc } {
    # Procedure called to update C_S_AXI_MAC_PKT_BASEADDR when any of the dependent parameters in the arguments change

    set C_S_AXI_MAC_PKT_BASEADDR ${PARAM_VALUE.C_S_AXI_MAC_PKT_BASEADDR}
    set gui_txBufLoc ${PARAM_VALUE.gui_txBufLoc}
    set gui_rxBufLoc ${PARAM_VALUE.gui_rxBufLoc}
    set values(gui_txBufLoc) [get_property value $gui_txBufLoc]
    set values(gui_rxBufLoc) [get_property value $gui_rxBufLoc]
    if { [gen_USERPARAMETER_C_S_AXI_MAC_PKT_BASEADDR_ENABLEMENT $values(gui_txBufLoc) $values(gui_rxBufLoc)] } {
        set_property enabled true $C_S_AXI_MAC_PKT_BASEADDR
    } else {
        set_property enabled false $C_S_AXI_MAC_PKT_BASEADDR
    }
}

proc validate_PARAM_VALUE.C_S_AXI_MAC_PKT_BASEADDR { PARAM_VALUE.C_S_AXI_MAC_PKT_BASEADDR } {
    # Procedure called to validate C_S_AXI_MAC_PKT_BASEADDR
    return true
}

proc update_PARAM_VALUE.C_S_AXI_MAC_PKT_HIGHADDR { PARAM_VALUE.C_S_AXI_MAC_PKT_HIGHADDR PARAM_VALUE.gui_txBufLoc PARAM_VALUE.gui_rxBufLoc } {
    # Procedure called to update C_S_AXI_MAC_PKT_HIGHADDR when any of the dependent parameters in the arguments change

    set C_S_AXI_MAC_PKT_HIGHADDR ${PARAM_VALUE.C_S_AXI_MAC_PKT_HIGHADDR}
    set gui_txBufLoc ${PARAM_VALUE.gui_txBufLoc}
    set gui_rxBufLoc ${PARAM_VALUE.gui_rxBufLoc}
    set values(gui_txBufLoc) [get_property value $gui_txBufLoc]
    set values(gui_rxBufLoc) [get_property value $gui_rxBufLoc]
    if { [gen_USERPARAMETER_C_S_AXI_MAC_PKT_HIGHADDR_ENABLEMENT $values(gui_txBufLoc) $values(gui_rxBufLoc)] } {
        set_property enabled true $C_S_AXI_MAC_PKT_HIGHADDR
    } else {
        set_property enabled false $C_S_AXI_MAC_PKT_HIGHADDR
    }
}

proc validate_PARAM_VALUE.C_S_AXI_MAC_PKT_HIGHADDR { PARAM_VALUE.C_S_AXI_MAC_PKT_HIGHADDR } {
    # Procedure called to validate C_S_AXI_MAC_PKT_HIGHADDR
    return true
}

proc update_PARAM_VALUE.C_S_AXI_MAC_PKT_MIN_SIZE { PARAM_VALUE.C_S_AXI_MAC_PKT_MIN_SIZE PARAM_VALUE.C_S_AXI_MAC_PKT_BASEADDR PARAM_VALUE.C_S_AXI_MAC_PKT_HIGHADDR } {
    # Procedure called to update C_S_AXI_MAC_PKT_MIN_SIZE when any of the dependent parameters in the arguments change

    set C_S_AXI_MAC_PKT_MIN_SIZE ${PARAM_VALUE.C_S_AXI_MAC_PKT_MIN_SIZE}
    set C_S_AXI_MAC_PKT_BASEADDR ${PARAM_VALUE.C_S_AXI_MAC_PKT_BASEADDR}
    set C_S_AXI_MAC_PKT_HIGHADDR ${PARAM_VALUE.C_S_AXI_MAC_PKT_HIGHADDR}
    set values(C_S_AXI_MAC_PKT_BASEADDR) [get_property value $C_S_AXI_MAC_PKT_BASEADDR]
    set values(C_S_AXI_MAC_PKT_HIGHADDR) [get_property value $C_S_AXI_MAC_PKT_HIGHADDR]
    set_property value [gen_USERPARAMETER_C_S_AXI_MAC_PKT_MIN_SIZE_VALUE $values(C_S_AXI_MAC_PKT_BASEADDR) $values(C_S_AXI_MAC_PKT_HIGHADDR)] $C_S_AXI_MAC_PKT_MIN_SIZE
}

proc validate_PARAM_VALUE.C_S_AXI_MAC_PKT_MIN_SIZE { PARAM_VALUE.C_S_AXI_MAC_PKT_MIN_SIZE } {
    # Procedure called to validate C_S_AXI_MAC_PKT_MIN_SIZE
    return true
}

proc update_PARAM_VALUE.C_S_AXI_MAC_REG_MIN_SIZE { PARAM_VALUE.C_S_AXI_MAC_REG_MIN_SIZE PARAM_VALUE.C_S_AXI_MAC_REG_RNG0_BASEADDR PARAM_VALUE.C_S_AXI_MAC_REG_RNG1_BASEADDR PARAM_VALUE.C_S_AXI_MAC_REG_RNG0_HIGHADDR PARAM_VALUE.C_S_AXI_MAC_REG_RNG1_HIGHADDR } {
    # Procedure called to update C_S_AXI_MAC_REG_MIN_SIZE when any of the dependent parameters in the arguments change

    set C_S_AXI_MAC_REG_MIN_SIZE ${PARAM_VALUE.C_S_AXI_MAC_REG_MIN_SIZE}
    set C_S_AXI_MAC_REG_RNG0_BASEADDR ${PARAM_VALUE.C_S_AXI_MAC_REG_RNG0_BASEADDR}
    set C_S_AXI_MAC_REG_RNG1_BASEADDR ${PARAM_VALUE.C_S_AXI_MAC_REG_RNG1_BASEADDR}
    set C_S_AXI_MAC_REG_RNG0_HIGHADDR ${PARAM_VALUE.C_S_AXI_MAC_REG_RNG0_HIGHADDR}
    set C_S_AXI_MAC_REG_RNG1_HIGHADDR ${PARAM_VALUE.C_S_AXI_MAC_REG_RNG1_HIGHADDR}
    set values(C_S_AXI_MAC_REG_RNG0_BASEADDR) [get_property value $C_S_AXI_MAC_REG_RNG0_BASEADDR]
    set values(C_S_AXI_MAC_REG_RNG1_BASEADDR) [get_property value $C_S_AXI_MAC_REG_RNG1_BASEADDR]
    set values(C_S_AXI_MAC_REG_RNG0_HIGHADDR) [get_property value $C_S_AXI_MAC_REG_RNG0_HIGHADDR]
    set values(C_S_AXI_MAC_REG_RNG1_HIGHADDR) [get_property value $C_S_AXI_MAC_REG_RNG1_HIGHADDR]
    set_property value [gen_USERPARAMETER_C_S_AXI_MAC_REG_MIN_SIZE_VALUE $values(C_S_AXI_MAC_REG_RNG0_BASEADDR) $values(C_S_AXI_MAC_REG_RNG1_BASEADDR) $values(C_S_AXI_MAC_REG_RNG0_HIGHADDR) $values(C_S_AXI_MAC_REG_RNG1_HIGHADDR)] $C_S_AXI_MAC_REG_MIN_SIZE
}

proc validate_PARAM_VALUE.C_S_AXI_MAC_REG_MIN_SIZE { PARAM_VALUE.C_S_AXI_MAC_REG_MIN_SIZE } {
    # Procedure called to validate C_S_AXI_MAC_REG_MIN_SIZE
    return true
}

proc update_PARAM_VALUE.gDmaBurstCountWidth { PARAM_VALUE.gDmaBurstCountWidth PARAM_VALUE.gui_txBufLoc PARAM_VALUE.gui_txBurstSize PARAM_VALUE.gui_rxBufLoc PARAM_VALUE.gui_rxBurstSize } {
    # Procedure called to update gDmaBurstCountWidth when any of the dependent parameters in the arguments change

    set gDmaBurstCountWidth ${PARAM_VALUE.gDmaBurstCountWidth}
    set gui_txBufLoc ${PARAM_VALUE.gui_txBufLoc}
    set gui_txBurstSize ${PARAM_VALUE.gui_txBurstSize}
    set gui_rxBufLoc ${PARAM_VALUE.gui_rxBufLoc}
    set gui_rxBurstSize ${PARAM_VALUE.gui_rxBurstSize}
    set values(gui_txBufLoc) [get_property value $gui_txBufLoc]
    set values(gui_txBurstSize) [get_property value $gui_txBurstSize]
    set values(gui_rxBufLoc) [get_property value $gui_rxBufLoc]
    set values(gui_rxBurstSize) [get_property value $gui_rxBurstSize]
    set_property value [gen_USERPARAMETER_gDmaBurstCountWidth_VALUE $values(gui_txBufLoc) $values(gui_txBurstSize) $values(gui_rxBufLoc) $values(gui_rxBurstSize)] $gDmaBurstCountWidth
}

proc validate_PARAM_VALUE.gDmaBurstCountWidth { PARAM_VALUE.gDmaBurstCountWidth } {
    # Procedure called to validate gDmaBurstCountWidth
    return true
}

proc update_PARAM_VALUE.gDmaReadBurstLength { PARAM_VALUE.gDmaReadBurstLength PARAM_VALUE.gui_txBurstSize } {
    # Procedure called to update gDmaReadBurstLength when any of the dependent parameters in the arguments change

    set gDmaReadBurstLength ${PARAM_VALUE.gDmaReadBurstLength}
    set gui_txBurstSize ${PARAM_VALUE.gui_txBurstSize}
    set values(gui_txBurstSize) [get_property value $gui_txBurstSize]
    set_property value [gen_USERPARAMETER_gDmaReadBurstLength_VALUE $values(gui_txBurstSize)] $gDmaReadBurstLength
}

proc validate_PARAM_VALUE.gDmaReadBurstLength { PARAM_VALUE.gDmaReadBurstLength } {
    # Procedure called to validate gDmaReadBurstLength
    return true
}

proc update_PARAM_VALUE.gDmaReadFifoLength { PARAM_VALUE.gDmaReadFifoLength PARAM_VALUE.gui_txBurstSize } {
    # Procedure called to update gDmaReadFifoLength when any of the dependent parameters in the arguments change

    set gDmaReadFifoLength ${PARAM_VALUE.gDmaReadFifoLength}
    set gui_txBurstSize ${PARAM_VALUE.gui_txBurstSize}
    set values(gui_txBurstSize) [get_property value $gui_txBurstSize]
    set_property value [gen_USERPARAMETER_gDmaReadFifoLength_VALUE $values(gui_txBurstSize)] $gDmaReadFifoLength
}

proc validate_PARAM_VALUE.gDmaReadFifoLength { PARAM_VALUE.gDmaReadFifoLength } {
    # Procedure called to validate gDmaReadFifoLength
    return true
}

proc update_PARAM_VALUE.gDmaWriteBurstLength { PARAM_VALUE.gDmaWriteBurstLength PARAM_VALUE.gui_rxBurstSize } {
    # Procedure called to update gDmaWriteBurstLength when any of the dependent parameters in the arguments change

    set gDmaWriteBurstLength ${PARAM_VALUE.gDmaWriteBurstLength}
    set gui_rxBurstSize ${PARAM_VALUE.gui_rxBurstSize}
    set values(gui_rxBurstSize) [get_property value $gui_rxBurstSize]
    set_property value [gen_USERPARAMETER_gDmaWriteBurstLength_VALUE $values(gui_rxBurstSize)] $gDmaWriteBurstLength
}

proc validate_PARAM_VALUE.gDmaWriteBurstLength { PARAM_VALUE.gDmaWriteBurstLength } {
    # Procedure called to validate gDmaWriteBurstLength
    return true
}

proc update_PARAM_VALUE.gDmaWriteFifoLength { PARAM_VALUE.gDmaWriteFifoLength PARAM_VALUE.gui_rxBurstSize } {
    # Procedure called to update gDmaWriteFifoLength when any of the dependent parameters in the arguments change

    set gDmaWriteFifoLength ${PARAM_VALUE.gDmaWriteFifoLength}
    set gui_rxBurstSize ${PARAM_VALUE.gui_rxBurstSize}
    set values(gui_rxBurstSize) [get_property value $gui_rxBurstSize]
    set_property value [gen_USERPARAMETER_gDmaWriteFifoLength_VALUE $values(gui_rxBurstSize)] $gDmaWriteFifoLength
}

proc validate_PARAM_VALUE.gDmaWriteFifoLength { PARAM_VALUE.gDmaWriteFifoLength } {
    # Procedure called to validate gDmaWriteFifoLength
    return true
}

proc update_PARAM_VALUE.gEnableActivity { PARAM_VALUE.gEnableActivity PARAM_VALUE.gui_actEn } {
    # Procedure called to update gEnableActivity when any of the dependent parameters in the arguments change

    set gEnableActivity ${PARAM_VALUE.gEnableActivity}
    set gui_actEn ${PARAM_VALUE.gui_actEn}
    set values(gui_actEn) [get_property value $gui_actEn]
    set_property value [gen_USERPARAMETER_gEnableActivity_VALUE $values(gui_actEn)] $gEnableActivity
}

proc validate_PARAM_VALUE.gEnableActivity { PARAM_VALUE.gEnableActivity } {
    # Procedure called to validate gEnableActivity
    return true
}

proc update_PARAM_VALUE.gEnableDmaObserver { PARAM_VALUE.gEnableDmaObserver PARAM_VALUE.gui_txBufLoc PARAM_VALUE.gui_rxBufLoc } {
    # Procedure called to update gEnableDmaObserver when any of the dependent parameters in the arguments change

    set gEnableDmaObserver ${PARAM_VALUE.gEnableDmaObserver}
    set gui_txBufLoc ${PARAM_VALUE.gui_txBufLoc}
    set gui_rxBufLoc ${PARAM_VALUE.gui_rxBufLoc}
    set values(gui_txBufLoc) [get_property value $gui_txBufLoc]
    set values(gui_rxBufLoc) [get_property value $gui_rxBufLoc]
    set_property value [gen_USERPARAMETER_gEnableDmaObserver_VALUE $values(gui_txBufLoc) $values(gui_rxBufLoc)] $gEnableDmaObserver
}

proc validate_PARAM_VALUE.gEnableDmaObserver { PARAM_VALUE.gEnableDmaObserver } {
    # Procedure called to validate gEnableDmaObserver
    return true
}

proc update_PARAM_VALUE.gPacketBufferLocRx { PARAM_VALUE.gPacketBufferLocRx PARAM_VALUE.gui_rxBufLoc } {
    # Procedure called to update gPacketBufferLocRx when any of the dependent parameters in the arguments change

    set gPacketBufferLocRx ${PARAM_VALUE.gPacketBufferLocRx}
    set gui_rxBufLoc ${PARAM_VALUE.gui_rxBufLoc}
    set values(gui_rxBufLoc) [get_property value $gui_rxBufLoc]
    set_property value [gen_USERPARAMETER_gPacketBufferLocRx_VALUE $values(gui_rxBufLoc)] $gPacketBufferLocRx
}

proc validate_PARAM_VALUE.gPacketBufferLocRx { PARAM_VALUE.gPacketBufferLocRx } {
    # Procedure called to validate gPacketBufferLocRx
    return true
}

proc update_PARAM_VALUE.gPacketBufferLocTx { PARAM_VALUE.gPacketBufferLocTx PARAM_VALUE.gui_txBufLoc } {
    # Procedure called to update gPacketBufferLocTx when any of the dependent parameters in the arguments change

    set gPacketBufferLocTx ${PARAM_VALUE.gPacketBufferLocTx}
    set gui_txBufLoc ${PARAM_VALUE.gui_txBufLoc}
    set values(gui_txBufLoc) [get_property value $gui_txBufLoc]
    set_property value [gen_USERPARAMETER_gPacketBufferLocTx_VALUE $values(gui_txBufLoc)] $gPacketBufferLocTx
}

proc validate_PARAM_VALUE.gPacketBufferLocTx { PARAM_VALUE.gPacketBufferLocTx } {
    # Procedure called to validate gPacketBufferLocTx
    return true
}

proc update_PARAM_VALUE.gPacketBufferLog2Size { PARAM_VALUE.gPacketBufferLog2Size PARAM_VALUE.gui_txBufLoc PARAM_VALUE.gui_txBufSize PARAM_VALUE.gui_rxBufLoc PARAM_VALUE.gui_rxBufSize } {
    # Procedure called to update gPacketBufferLog2Size when any of the dependent parameters in the arguments change

    set gPacketBufferLog2Size ${PARAM_VALUE.gPacketBufferLog2Size}
    set gui_txBufLoc ${PARAM_VALUE.gui_txBufLoc}
    set gui_txBufSize ${PARAM_VALUE.gui_txBufSize}
    set gui_rxBufLoc ${PARAM_VALUE.gui_rxBufLoc}
    set gui_rxBufSize ${PARAM_VALUE.gui_rxBufSize}
    set values(gui_txBufLoc) [get_property value $gui_txBufLoc]
    set values(gui_txBufSize) [get_property value $gui_txBufSize]
    set values(gui_rxBufLoc) [get_property value $gui_rxBufLoc]
    set values(gui_rxBufSize) [get_property value $gui_rxBufSize]
    set_property value [gen_USERPARAMETER_gPacketBufferLog2Size_VALUE $values(gui_txBufLoc) $values(gui_txBufSize) $values(gui_rxBufLoc) $values(gui_rxBufSize)] $gPacketBufferLog2Size
}

proc validate_PARAM_VALUE.gPacketBufferLog2Size { PARAM_VALUE.gPacketBufferLog2Size } {
    # Procedure called to validate gPacketBufferLog2Size
    return true
}

proc update_PARAM_VALUE.gPhyPortCount { PARAM_VALUE.gPhyPortCount PARAM_VALUE.gui_phyCount } {
    # Procedure called to update gPhyPortCount when any of the dependent parameters in the arguments change

    set gPhyPortCount ${PARAM_VALUE.gPhyPortCount}
    set gui_phyCount ${PARAM_VALUE.gui_phyCount}
    set values(gui_phyCount) [get_property value $gui_phyCount]
    set_property value [gen_USERPARAMETER_gPhyPortCount_VALUE $values(gui_phyCount)] $gPhyPortCount
}

proc validate_PARAM_VALUE.gPhyPortCount { PARAM_VALUE.gPhyPortCount } {
    # Procedure called to validate gPhyPortCount
    return true
}

proc update_PARAM_VALUE.gPhyPortType { PARAM_VALUE.gPhyPortType PARAM_VALUE.gui_phyType } {
    # Procedure called to update gPhyPortType when any of the dependent parameters in the arguments change

    set gPhyPortType ${PARAM_VALUE.gPhyPortType}
    set gui_phyType ${PARAM_VALUE.gui_phyType}
    set values(gui_phyType) [get_property value $gui_phyType]
    set_property value [gen_USERPARAMETER_gPhyPortType_VALUE $values(gui_phyType)] $gPhyPortType
}

proc validate_PARAM_VALUE.gPhyPortType { PARAM_VALUE.gPhyPortType } {
    # Procedure called to validate gPhyPortType
    return true
}

proc update_PARAM_VALUE.gSmiPortCount { PARAM_VALUE.gSmiPortCount PARAM_VALUE.gui_phyCount PARAM_VALUE.gui_extraSmi } {
    # Procedure called to update gSmiPortCount when any of the dependent parameters in the arguments change

    set gSmiPortCount ${PARAM_VALUE.gSmiPortCount}
    set gui_phyCount ${PARAM_VALUE.gui_phyCount}
    set gui_extraSmi ${PARAM_VALUE.gui_extraSmi}
    set values(gui_phyCount) [get_property value $gui_phyCount]
    set values(gui_extraSmi) [get_property value $gui_extraSmi]
    set_property value [gen_USERPARAMETER_gSmiPortCount_VALUE $values(gui_phyCount) $values(gui_extraSmi)] $gSmiPortCount
}

proc validate_PARAM_VALUE.gSmiPortCount { PARAM_VALUE.gSmiPortCount } {
    # Procedure called to validate gSmiPortCount
    return true
}

proc update_PARAM_VALUE.gTimerEnablePulse { PARAM_VALUE.gTimerEnablePulse PARAM_VALUE.gui_tmrPulse } {
    # Procedure called to update gTimerEnablePulse when any of the dependent parameters in the arguments change

    set gTimerEnablePulse ${PARAM_VALUE.gTimerEnablePulse}
    set gui_tmrPulse ${PARAM_VALUE.gui_tmrPulse}
    set values(gui_tmrPulse) [get_property value $gui_tmrPulse]
    set_property value [gen_USERPARAMETER_gTimerEnablePulse_VALUE $values(gui_tmrPulse)] $gTimerEnablePulse
}

proc validate_PARAM_VALUE.gTimerEnablePulse { PARAM_VALUE.gTimerEnablePulse } {
    # Procedure called to validate gTimerEnablePulse
    return true
}

proc update_PARAM_VALUE.gTimerEnablePulseWidth { PARAM_VALUE.gTimerEnablePulseWidth PARAM_VALUE.gui_tmrPulseEn } {
    # Procedure called to update gTimerEnablePulseWidth when any of the dependent parameters in the arguments change

    set gTimerEnablePulseWidth ${PARAM_VALUE.gTimerEnablePulseWidth}
    set gui_tmrPulseEn ${PARAM_VALUE.gui_tmrPulseEn}
    set values(gui_tmrPulseEn) [get_property value $gui_tmrPulseEn]
    set_property value [gen_USERPARAMETER_gTimerEnablePulseWidth_VALUE $values(gui_tmrPulseEn)] $gTimerEnablePulseWidth
}

proc validate_PARAM_VALUE.gTimerEnablePulseWidth { PARAM_VALUE.gTimerEnablePulseWidth } {
    # Procedure called to validate gTimerEnablePulseWidth
    return true
}

proc update_PARAM_VALUE.gTimerPulseRegWidth { PARAM_VALUE.gTimerPulseRegWidth PARAM_VALUE.gui_tmrPulseWdt } {
    # Procedure called to update gTimerPulseRegWidth when any of the dependent parameters in the arguments change

    set gTimerPulseRegWidth ${PARAM_VALUE.gTimerPulseRegWidth}
    set gui_tmrPulseWdt ${PARAM_VALUE.gui_tmrPulseWdt}
    set values(gui_tmrPulseWdt) [get_property value $gui_tmrPulseWdt]
    set_property value [gen_USERPARAMETER_gTimerPulseRegWidth_VALUE $values(gui_tmrPulseWdt)] $gTimerPulseRegWidth
}

proc validate_PARAM_VALUE.gTimerPulseRegWidth { PARAM_VALUE.gTimerPulseRegWidth } {
    # Procedure called to validate gTimerPulseRegWidth
    return true
}

proc update_PARAM_VALUE.gui_rxBufSize { PARAM_VALUE.gui_rxBufSize PARAM_VALUE.gui_rxBufLoc } {
    # Procedure called to update gui_rxBufSize when any of the dependent parameters in the arguments change

    set gui_rxBufSize ${PARAM_VALUE.gui_rxBufSize}
    set gui_rxBufLoc ${PARAM_VALUE.gui_rxBufLoc}
    set values(gui_rxBufLoc) [get_property value $gui_rxBufLoc]
    if { [gen_USERPARAMETER_gui_rxBufSize_ENABLEMENT $values(gui_rxBufLoc)] } {
        set_property enabled true $gui_rxBufSize
    } else {
        set_property enabled false $gui_rxBufSize
    }
}

proc validate_PARAM_VALUE.gui_rxBufSize { PARAM_VALUE.gui_rxBufSize } {
    # Procedure called to validate gui_rxBufSize
    return true
}

proc update_PARAM_VALUE.gui_rxBurstSize { PARAM_VALUE.gui_rxBurstSize PARAM_VALUE.gui_rxBufLoc } {
    # Procedure called to update gui_rxBurstSize when any of the dependent parameters in the arguments change

    set gui_rxBurstSize ${PARAM_VALUE.gui_rxBurstSize}
    set gui_rxBufLoc ${PARAM_VALUE.gui_rxBufLoc}
    set values(gui_rxBufLoc) [get_property value $gui_rxBufLoc]
    if { [gen_USERPARAMETER_gui_rxBurstSize_ENABLEMENT $values(gui_rxBufLoc)] } {
        set_property enabled true $gui_rxBurstSize
    } else {
        set_property enabled false $gui_rxBurstSize
    }
}

proc validate_PARAM_VALUE.gui_rxBurstSize { PARAM_VALUE.gui_rxBurstSize } {
    # Procedure called to validate gui_rxBurstSize
    return true
}

proc update_PARAM_VALUE.gui_tmrPulseEn { PARAM_VALUE.gui_tmrPulseEn PARAM_VALUE.gui_tmrPulse } {
    # Procedure called to update gui_tmrPulseEn when any of the dependent parameters in the arguments change

    set gui_tmrPulseEn ${PARAM_VALUE.gui_tmrPulseEn}
    set gui_tmrPulse ${PARAM_VALUE.gui_tmrPulse}
    set values(gui_tmrPulse) [get_property value $gui_tmrPulse]
    if { [gen_USERPARAMETER_gui_tmrPulseEn_ENABLEMENT $values(gui_tmrPulse)] } {
        set_property enabled true $gui_tmrPulseEn
    } else {
        set_property enabled false $gui_tmrPulseEn
    }
}

proc validate_PARAM_VALUE.gui_tmrPulseEn { PARAM_VALUE.gui_tmrPulseEn } {
    # Procedure called to validate gui_tmrPulseEn
    return true
}

proc update_PARAM_VALUE.gui_tmrPulseWdt { PARAM_VALUE.gui_tmrPulseWdt PARAM_VALUE.gui_tmrPulse PARAM_VALUE.gui_tmrPulseEn } {
    # Procedure called to update gui_tmrPulseWdt when any of the dependent parameters in the arguments change

    set gui_tmrPulseWdt ${PARAM_VALUE.gui_tmrPulseWdt}
    set gui_tmrPulse ${PARAM_VALUE.gui_tmrPulse}
    set gui_tmrPulseEn ${PARAM_VALUE.gui_tmrPulseEn}
    set values(gui_tmrPulse) [get_property value $gui_tmrPulse]
    set values(gui_tmrPulseEn) [get_property value $gui_tmrPulseEn]
    if { [gen_USERPARAMETER_gui_tmrPulseWdt_ENABLEMENT $values(gui_tmrPulse) $values(gui_tmrPulseEn)] } {
        set_property enabled true $gui_tmrPulseWdt
    } else {
        set_property enabled false $gui_tmrPulseWdt
    }
}

proc validate_PARAM_VALUE.gui_tmrPulseWdt { PARAM_VALUE.gui_tmrPulseWdt } {
    # Procedure called to validate gui_tmrPulseWdt
    return true
}

proc update_PARAM_VALUE.gui_txBufSize { PARAM_VALUE.gui_txBufSize PARAM_VALUE.gui_txBufLoc } {
    # Procedure called to update gui_txBufSize when any of the dependent parameters in the arguments change

    set gui_txBufSize ${PARAM_VALUE.gui_txBufSize}
    set gui_txBufLoc ${PARAM_VALUE.gui_txBufLoc}
    set values(gui_txBufLoc) [get_property value $gui_txBufLoc]
    if { [gen_USERPARAMETER_gui_txBufSize_ENABLEMENT $values(gui_txBufLoc)] } {
        set_property enabled true $gui_txBufSize
    } else {
        set_property enabled false $gui_txBufSize
    }
}

proc validate_PARAM_VALUE.gui_txBufSize { PARAM_VALUE.gui_txBufSize } {
    # Procedure called to validate gui_txBufSize
    return true
}

proc update_PARAM_VALUE.gui_txBurstSize { PARAM_VALUE.gui_txBurstSize PARAM_VALUE.gui_txBufLoc } {
    # Procedure called to update gui_txBurstSize when any of the dependent parameters in the arguments change

    set gui_txBurstSize ${PARAM_VALUE.gui_txBurstSize}
    set gui_txBufLoc ${PARAM_VALUE.gui_txBufLoc}
    set values(gui_txBufLoc) [get_property value $gui_txBufLoc]
    if { [gen_USERPARAMETER_gui_txBurstSize_ENABLEMENT $values(gui_txBufLoc)] } {
        set_property enabled true $gui_txBurstSize
    } else {
        set_property enabled false $gui_txBurstSize
    }
}

proc validate_PARAM_VALUE.gui_txBurstSize { PARAM_VALUE.gui_txBurstSize } {
    # Procedure called to validate gui_txBurstSize
    return true
}

proc update_PARAM_VALUE.gui_extraSmi { PARAM_VALUE.gui_extraSmi } {
    # Procedure called to update gui_extraSmi when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.gui_extraSmi { PARAM_VALUE.gui_extraSmi } {
    # Procedure called to validate gui_extraSmi
    return true
}

proc update_PARAM_VALUE.gui_phyCount { PARAM_VALUE.gui_phyCount } {
    # Procedure called to update gui_phyCount when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.gui_phyCount { PARAM_VALUE.gui_phyCount } {
    # Procedure called to validate gui_phyCount
    return true
}

proc update_PARAM_VALUE.gui_phyType { PARAM_VALUE.gui_phyType } {
    # Procedure called to update gui_phyType when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.gui_phyType { PARAM_VALUE.gui_phyType } {
    # Procedure called to validate gui_phyType
    return true
}

proc update_PARAM_VALUE.gui_tmrPulse { PARAM_VALUE.gui_tmrPulse } {
    # Procedure called to update gui_tmrPulse when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.gui_tmrPulse { PARAM_VALUE.gui_tmrPulse } {
    # Procedure called to validate gui_tmrPulse
    return true
}

proc update_PARAM_VALUE.C_S_AXI_MAC_PKT_PROTOCOL { PARAM_VALUE.C_S_AXI_MAC_PKT_PROTOCOL } {
    # Procedure called to update C_S_AXI_MAC_PKT_PROTOCOL when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.C_S_AXI_MAC_PKT_PROTOCOL { PARAM_VALUE.C_S_AXI_MAC_PKT_PROTOCOL } {
    # Procedure called to validate C_S_AXI_MAC_PKT_PROTOCOL
    return true
}

proc update_PARAM_VALUE.C_S_AXI_MAC_REG_PROTOCOL { PARAM_VALUE.C_S_AXI_MAC_REG_PROTOCOL } {
    # Procedure called to update C_S_AXI_MAC_REG_PROTOCOL when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.C_S_AXI_MAC_REG_PROTOCOL { PARAM_VALUE.C_S_AXI_MAC_REG_PROTOCOL } {
    # Procedure called to validate C_S_AXI_MAC_REG_PROTOCOL
    return true
}

proc update_PARAM_VALUE.C_M_AXI_MAC_DMA_PROTOCOL { PARAM_VALUE.C_M_AXI_MAC_DMA_PROTOCOL } {
    # Procedure called to update C_M_AXI_MAC_DMA_PROTOCOL when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.C_M_AXI_MAC_DMA_PROTOCOL { PARAM_VALUE.C_M_AXI_MAC_DMA_PROTOCOL } {
    # Procedure called to validate C_M_AXI_MAC_DMA_PROTOCOL
    return true
}

proc update_PARAM_VALUE.gui_actEn { PARAM_VALUE.gui_actEn } {
    # Procedure called to update gui_actEn when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.gui_actEn { PARAM_VALUE.gui_actEn } {
    # Procedure called to validate gui_actEn
    return true
}

proc update_PARAM_VALUE.gui_rxBufLoc { PARAM_VALUE.gui_rxBufLoc } {
    # Procedure called to update gui_rxBufLoc when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.gui_rxBufLoc { PARAM_VALUE.gui_rxBufLoc } {
    # Procedure called to validate gui_rxBufLoc
    return true
}

proc update_PARAM_VALUE.gui_txBufLoc { PARAM_VALUE.gui_txBufLoc } {
    # Procedure called to update gui_txBufLoc when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.gui_txBufLoc { PARAM_VALUE.gui_txBufLoc } {
    # Procedure called to validate gui_txBufLoc
    return true
}

proc update_PARAM_VALUE.C_S_AXI_MAC_REG_CLK_XING { PARAM_VALUE.C_S_AXI_MAC_REG_CLK_XING } {
    # Procedure called to update C_S_AXI_MAC_REG_CLK_XING when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.C_S_AXI_MAC_REG_CLK_XING { PARAM_VALUE.C_S_AXI_MAC_REG_CLK_XING } {
    # Procedure called to validate C_S_AXI_MAC_REG_CLK_XING
    return true
}

proc update_PARAM_VALUE.gDmaDataWidth { PARAM_VALUE.gDmaDataWidth } {
    # Procedure called to update gDmaDataWidth when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.gDmaDataWidth { PARAM_VALUE.gDmaDataWidth } {
    # Procedure called to validate gDmaDataWidth
    return true
}

proc update_PARAM_VALUE.gDmaAddrWidth { PARAM_VALUE.gDmaAddrWidth } {
    # Procedure called to update gDmaAddrWidth when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.gDmaAddrWidth { PARAM_VALUE.gDmaAddrWidth } {
    # Procedure called to validate gDmaAddrWidth
    return true
}

proc update_PARAM_VALUE.gEndianness { PARAM_VALUE.gEndianness } {
    # Procedure called to update gEndianness when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.gEndianness { PARAM_VALUE.gEndianness } {
    # Procedure called to validate gEndianness
    return true
}

proc update_PARAM_VALUE.C_S_AXI_MAC_PKT_DPHASE_TIMEOUT { PARAM_VALUE.C_S_AXI_MAC_PKT_DPHASE_TIMEOUT } {
    # Procedure called to update C_S_AXI_MAC_PKT_DPHASE_TIMEOUT when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.C_S_AXI_MAC_PKT_DPHASE_TIMEOUT { PARAM_VALUE.C_S_AXI_MAC_PKT_DPHASE_TIMEOUT } {
    # Procedure called to validate C_S_AXI_MAC_PKT_DPHASE_TIMEOUT
    return true
}

proc update_PARAM_VALUE.C_S_AXI_MAC_PKT_USE_WSTRB { PARAM_VALUE.C_S_AXI_MAC_PKT_USE_WSTRB } {
    # Procedure called to update C_S_AXI_MAC_PKT_USE_WSTRB when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.C_S_AXI_MAC_PKT_USE_WSTRB { PARAM_VALUE.C_S_AXI_MAC_PKT_USE_WSTRB } {
    # Procedure called to validate C_S_AXI_MAC_PKT_USE_WSTRB
    return true
}

proc update_PARAM_VALUE.C_S_AXI_MAC_PKT_ADDR_WIDTH { PARAM_VALUE.C_S_AXI_MAC_PKT_ADDR_WIDTH } {
    # Procedure called to update C_S_AXI_MAC_PKT_ADDR_WIDTH when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.C_S_AXI_MAC_PKT_ADDR_WIDTH { PARAM_VALUE.C_S_AXI_MAC_PKT_ADDR_WIDTH } {
    # Procedure called to validate C_S_AXI_MAC_PKT_ADDR_WIDTH
    return true
}

proc update_PARAM_VALUE.C_S_AXI_MAC_PKT_DATA_WIDTH { PARAM_VALUE.C_S_AXI_MAC_PKT_DATA_WIDTH } {
    # Procedure called to update C_S_AXI_MAC_PKT_DATA_WIDTH when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.C_S_AXI_MAC_PKT_DATA_WIDTH { PARAM_VALUE.C_S_AXI_MAC_PKT_DATA_WIDTH } {
    # Procedure called to validate C_S_AXI_MAC_PKT_DATA_WIDTH
    return true
}

proc update_PARAM_VALUE.C_S_AXI_MAC_REG_DPHASE_TIMEOUT { PARAM_VALUE.C_S_AXI_MAC_REG_DPHASE_TIMEOUT } {
    # Procedure called to update C_S_AXI_MAC_REG_DPHASE_TIMEOUT when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.C_S_AXI_MAC_REG_DPHASE_TIMEOUT { PARAM_VALUE.C_S_AXI_MAC_REG_DPHASE_TIMEOUT } {
    # Procedure called to validate C_S_AXI_MAC_REG_DPHASE_TIMEOUT
    return true
}

proc update_PARAM_VALUE.C_S_AXI_MAC_REG_USE_WSTRB { PARAM_VALUE.C_S_AXI_MAC_REG_USE_WSTRB } {
    # Procedure called to update C_S_AXI_MAC_REG_USE_WSTRB when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.C_S_AXI_MAC_REG_USE_WSTRB { PARAM_VALUE.C_S_AXI_MAC_REG_USE_WSTRB } {
    # Procedure called to validate C_S_AXI_MAC_REG_USE_WSTRB
    return true
}

proc update_PARAM_VALUE.C_S_AXI_MAC_REG_ACLK_FREQ_HZ { PARAM_VALUE.C_S_AXI_MAC_REG_ACLK_FREQ_HZ } {
    # Procedure called to update C_S_AXI_MAC_REG_ACLK_FREQ_HZ when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.C_S_AXI_MAC_REG_ACLK_FREQ_HZ { PARAM_VALUE.C_S_AXI_MAC_REG_ACLK_FREQ_HZ } {
    # Procedure called to validate C_S_AXI_MAC_REG_ACLK_FREQ_HZ
    return true
}

proc update_PARAM_VALUE.C_S_AXI_MAC_REG_ADDR_WIDTH { PARAM_VALUE.C_S_AXI_MAC_REG_ADDR_WIDTH } {
    # Procedure called to update C_S_AXI_MAC_REG_ADDR_WIDTH when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.C_S_AXI_MAC_REG_ADDR_WIDTH { PARAM_VALUE.C_S_AXI_MAC_REG_ADDR_WIDTH } {
    # Procedure called to validate C_S_AXI_MAC_REG_ADDR_WIDTH
    return true
}

proc update_PARAM_VALUE.C_S_AXI_MAC_REG_DATA_WIDTH { PARAM_VALUE.C_S_AXI_MAC_REG_DATA_WIDTH } {
    # Procedure called to update C_S_AXI_MAC_REG_DATA_WIDTH when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.C_S_AXI_MAC_REG_DATA_WIDTH { PARAM_VALUE.C_S_AXI_MAC_REG_DATA_WIDTH } {
    # Procedure called to validate C_S_AXI_MAC_REG_DATA_WIDTH
    return true
}

proc update_PARAM_VALUE.C_S_AXI_MAC_REG_RNG1_HIGHADDR { PARAM_VALUE.C_S_AXI_MAC_REG_RNG1_HIGHADDR } {
    # Procedure called to update C_S_AXI_MAC_REG_RNG1_HIGHADDR when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.C_S_AXI_MAC_REG_RNG1_HIGHADDR { PARAM_VALUE.C_S_AXI_MAC_REG_RNG1_HIGHADDR } {
    # Procedure called to validate C_S_AXI_MAC_REG_RNG1_HIGHADDR
    return true
}

proc update_PARAM_VALUE.C_S_AXI_MAC_REG_RNG1_BASEADDR { PARAM_VALUE.C_S_AXI_MAC_REG_RNG1_BASEADDR } {
    # Procedure called to update C_S_AXI_MAC_REG_RNG1_BASEADDR when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.C_S_AXI_MAC_REG_RNG1_BASEADDR { PARAM_VALUE.C_S_AXI_MAC_REG_RNG1_BASEADDR } {
    # Procedure called to validate C_S_AXI_MAC_REG_RNG1_BASEADDR
    return true
}

proc update_PARAM_VALUE.C_S_AXI_MAC_REG_RNG0_HIGHADDR { PARAM_VALUE.C_S_AXI_MAC_REG_RNG0_HIGHADDR } {
    # Procedure called to update C_S_AXI_MAC_REG_RNG0_HIGHADDR when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.C_S_AXI_MAC_REG_RNG0_HIGHADDR { PARAM_VALUE.C_S_AXI_MAC_REG_RNG0_HIGHADDR } {
    # Procedure called to validate C_S_AXI_MAC_REG_RNG0_HIGHADDR
    return true
}

proc update_PARAM_VALUE.C_S_AXI_MAC_REG_RNG0_BASEADDR { PARAM_VALUE.C_S_AXI_MAC_REG_RNG0_BASEADDR } {
    # Procedure called to update C_S_AXI_MAC_REG_RNG0_BASEADDR when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.C_S_AXI_MAC_REG_RNG0_BASEADDR { PARAM_VALUE.C_S_AXI_MAC_REG_RNG0_BASEADDR } {
    # Procedure called to validate C_S_AXI_MAC_REG_RNG0_BASEADDR
    return true
}

proc update_PARAM_VALUE.C_S_AXI_MAC_REG_NUM_ADDR_RANGES { PARAM_VALUE.C_S_AXI_MAC_REG_NUM_ADDR_RANGES } {
    # Procedure called to update C_S_AXI_MAC_REG_NUM_ADDR_RANGES when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.C_S_AXI_MAC_REG_NUM_ADDR_RANGES { PARAM_VALUE.C_S_AXI_MAC_REG_NUM_ADDR_RANGES } {
    # Procedure called to validate C_S_AXI_MAC_REG_NUM_ADDR_RANGES
    return true
}

proc update_PARAM_VALUE.C_M_AXI_MAC_DMA_LENGTH_WIDTH { PARAM_VALUE.C_M_AXI_MAC_DMA_LENGTH_WIDTH } {
    # Procedure called to update C_M_AXI_MAC_DMA_LENGTH_WIDTH when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.C_M_AXI_MAC_DMA_LENGTH_WIDTH { PARAM_VALUE.C_M_AXI_MAC_DMA_LENGTH_WIDTH } {
    # Procedure called to validate C_M_AXI_MAC_DMA_LENGTH_WIDTH
    return true
}

proc update_PARAM_VALUE.C_M_AXI_MAC_DMA_NATIVE_DWIDTH { PARAM_VALUE.C_M_AXI_MAC_DMA_NATIVE_DWIDTH } {
    # Procedure called to update C_M_AXI_MAC_DMA_NATIVE_DWIDTH when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.C_M_AXI_MAC_DMA_NATIVE_DWIDTH { PARAM_VALUE.C_M_AXI_MAC_DMA_NATIVE_DWIDTH } {
    # Procedure called to validate C_M_AXI_MAC_DMA_NATIVE_DWIDTH
    return true
}

proc update_PARAM_VALUE.C_M_AXI_MAC_DMA_DATA_WIDTH { PARAM_VALUE.C_M_AXI_MAC_DMA_DATA_WIDTH } {
    # Procedure called to update C_M_AXI_MAC_DMA_DATA_WIDTH when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.C_M_AXI_MAC_DMA_DATA_WIDTH { PARAM_VALUE.C_M_AXI_MAC_DMA_DATA_WIDTH } {
    # Procedure called to validate C_M_AXI_MAC_DMA_DATA_WIDTH
    return true
}

proc update_PARAM_VALUE.C_M_AXI_MAC_DMA_ADDR_WIDTH { PARAM_VALUE.C_M_AXI_MAC_DMA_ADDR_WIDTH } {
    # Procedure called to update C_M_AXI_MAC_DMA_ADDR_WIDTH when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.C_M_AXI_MAC_DMA_ADDR_WIDTH { PARAM_VALUE.C_M_AXI_MAC_DMA_ADDR_WIDTH } {
    # Procedure called to validate C_M_AXI_MAC_DMA_ADDR_WIDTH
    return true
}


proc update_MODELPARAM_VALUE.C_M_AXI_MAC_DMA_ADDR_WIDTH { MODELPARAM_VALUE.C_M_AXI_MAC_DMA_ADDR_WIDTH PARAM_VALUE.C_M_AXI_MAC_DMA_ADDR_WIDTH } {
    # Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
    set_property value [get_property value ${PARAM_VALUE.C_M_AXI_MAC_DMA_ADDR_WIDTH}] ${MODELPARAM_VALUE.C_M_AXI_MAC_DMA_ADDR_WIDTH}
}

proc update_MODELPARAM_VALUE.C_M_AXI_MAC_DMA_DATA_WIDTH { MODELPARAM_VALUE.C_M_AXI_MAC_DMA_DATA_WIDTH PARAM_VALUE.C_M_AXI_MAC_DMA_DATA_WIDTH } {
    # Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
    set_property value [get_property value ${PARAM_VALUE.C_M_AXI_MAC_DMA_DATA_WIDTH}] ${MODELPARAM_VALUE.C_M_AXI_MAC_DMA_DATA_WIDTH}
}

proc update_MODELPARAM_VALUE.C_M_AXI_MAC_DMA_NATIVE_DWIDTH { MODELPARAM_VALUE.C_M_AXI_MAC_DMA_NATIVE_DWIDTH PARAM_VALUE.C_M_AXI_MAC_DMA_NATIVE_DWIDTH } {
    # Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
    set_property value [get_property value ${PARAM_VALUE.C_M_AXI_MAC_DMA_NATIVE_DWIDTH}] ${MODELPARAM_VALUE.C_M_AXI_MAC_DMA_NATIVE_DWIDTH}
}

proc update_MODELPARAM_VALUE.C_M_AXI_MAC_DMA_LENGTH_WIDTH { MODELPARAM_VALUE.C_M_AXI_MAC_DMA_LENGTH_WIDTH PARAM_VALUE.C_M_AXI_MAC_DMA_LENGTH_WIDTH } {
    # Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
    set_property value [get_property value ${PARAM_VALUE.C_M_AXI_MAC_DMA_LENGTH_WIDTH}] ${MODELPARAM_VALUE.C_M_AXI_MAC_DMA_LENGTH_WIDTH}
}

proc update_MODELPARAM_VALUE.C_M_AXI_MAC_DMA_MAX_BURST_LEN { MODELPARAM_VALUE.C_M_AXI_MAC_DMA_MAX_BURST_LEN PARAM_VALUE.C_M_AXI_MAC_DMA_MAX_BURST_LEN } {
    # Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
    set_property value [get_property value ${PARAM_VALUE.C_M_AXI_MAC_DMA_MAX_BURST_LEN}] ${MODELPARAM_VALUE.C_M_AXI_MAC_DMA_MAX_BURST_LEN}
}

proc update_MODELPARAM_VALUE.C_S_AXI_MAC_REG_NUM_ADDR_RANGES { MODELPARAM_VALUE.C_S_AXI_MAC_REG_NUM_ADDR_RANGES PARAM_VALUE.C_S_AXI_MAC_REG_NUM_ADDR_RANGES } {
    # Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
    set_property value [get_property value ${PARAM_VALUE.C_S_AXI_MAC_REG_NUM_ADDR_RANGES}] ${MODELPARAM_VALUE.C_S_AXI_MAC_REG_NUM_ADDR_RANGES}
}

proc update_MODELPARAM_VALUE.C_S_AXI_MAC_REG_RNG0_BASEADDR { MODELPARAM_VALUE.C_S_AXI_MAC_REG_RNG0_BASEADDR PARAM_VALUE.C_S_AXI_MAC_REG_RNG0_BASEADDR } {
    # Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
    set_property value [get_property value ${PARAM_VALUE.C_S_AXI_MAC_REG_RNG0_BASEADDR}] ${MODELPARAM_VALUE.C_S_AXI_MAC_REG_RNG0_BASEADDR}
}

proc update_MODELPARAM_VALUE.C_S_AXI_MAC_REG_RNG0_HIGHADDR { MODELPARAM_VALUE.C_S_AXI_MAC_REG_RNG0_HIGHADDR PARAM_VALUE.C_S_AXI_MAC_REG_RNG0_HIGHADDR } {
    # Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
    set_property value [get_property value ${PARAM_VALUE.C_S_AXI_MAC_REG_RNG0_HIGHADDR}] ${MODELPARAM_VALUE.C_S_AXI_MAC_REG_RNG0_HIGHADDR}
}

proc update_MODELPARAM_VALUE.C_S_AXI_MAC_REG_RNG1_BASEADDR { MODELPARAM_VALUE.C_S_AXI_MAC_REG_RNG1_BASEADDR PARAM_VALUE.C_S_AXI_MAC_REG_RNG1_BASEADDR } {
    # Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
    set_property value [get_property value ${PARAM_VALUE.C_S_AXI_MAC_REG_RNG1_BASEADDR}] ${MODELPARAM_VALUE.C_S_AXI_MAC_REG_RNG1_BASEADDR}
}

proc update_MODELPARAM_VALUE.C_S_AXI_MAC_REG_RNG1_HIGHADDR { MODELPARAM_VALUE.C_S_AXI_MAC_REG_RNG1_HIGHADDR PARAM_VALUE.C_S_AXI_MAC_REG_RNG1_HIGHADDR } {
    # Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
    set_property value [get_property value ${PARAM_VALUE.C_S_AXI_MAC_REG_RNG1_HIGHADDR}] ${MODELPARAM_VALUE.C_S_AXI_MAC_REG_RNG1_HIGHADDR}
}

proc update_MODELPARAM_VALUE.C_S_AXI_MAC_REG_MIN_SIZE { MODELPARAM_VALUE.C_S_AXI_MAC_REG_MIN_SIZE PARAM_VALUE.C_S_AXI_MAC_REG_MIN_SIZE } {
    # Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
    set_property value [get_property value ${PARAM_VALUE.C_S_AXI_MAC_REG_MIN_SIZE}] ${MODELPARAM_VALUE.C_S_AXI_MAC_REG_MIN_SIZE}
}

proc update_MODELPARAM_VALUE.C_S_AXI_MAC_REG_DATA_WIDTH { MODELPARAM_VALUE.C_S_AXI_MAC_REG_DATA_WIDTH PARAM_VALUE.C_S_AXI_MAC_REG_DATA_WIDTH } {
    # Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
    set_property value [get_property value ${PARAM_VALUE.C_S_AXI_MAC_REG_DATA_WIDTH}] ${MODELPARAM_VALUE.C_S_AXI_MAC_REG_DATA_WIDTH}
}

proc update_MODELPARAM_VALUE.C_S_AXI_MAC_REG_ADDR_WIDTH { MODELPARAM_VALUE.C_S_AXI_MAC_REG_ADDR_WIDTH PARAM_VALUE.C_S_AXI_MAC_REG_ADDR_WIDTH } {
    # Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
    set_property value [get_property value ${PARAM_VALUE.C_S_AXI_MAC_REG_ADDR_WIDTH}] ${MODELPARAM_VALUE.C_S_AXI_MAC_REG_ADDR_WIDTH}
}

proc update_MODELPARAM_VALUE.C_S_AXI_MAC_REG_ACLK_FREQ_HZ { MODELPARAM_VALUE.C_S_AXI_MAC_REG_ACLK_FREQ_HZ PARAM_VALUE.C_S_AXI_MAC_REG_ACLK_FREQ_HZ } {
    # Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
    set_property value [get_property value ${PARAM_VALUE.C_S_AXI_MAC_REG_ACLK_FREQ_HZ}] ${MODELPARAM_VALUE.C_S_AXI_MAC_REG_ACLK_FREQ_HZ}
}

proc update_MODELPARAM_VALUE.C_S_AXI_MAC_REG_USE_WSTRB { MODELPARAM_VALUE.C_S_AXI_MAC_REG_USE_WSTRB PARAM_VALUE.C_S_AXI_MAC_REG_USE_WSTRB } {
    # Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
    set_property value [get_property value ${PARAM_VALUE.C_S_AXI_MAC_REG_USE_WSTRB}] ${MODELPARAM_VALUE.C_S_AXI_MAC_REG_USE_WSTRB}
}

proc update_MODELPARAM_VALUE.C_S_AXI_MAC_REG_DPHASE_TIMEOUT { MODELPARAM_VALUE.C_S_AXI_MAC_REG_DPHASE_TIMEOUT PARAM_VALUE.C_S_AXI_MAC_REG_DPHASE_TIMEOUT } {
    # Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
    set_property value [get_property value ${PARAM_VALUE.C_S_AXI_MAC_REG_DPHASE_TIMEOUT}] ${MODELPARAM_VALUE.C_S_AXI_MAC_REG_DPHASE_TIMEOUT}
}

proc update_MODELPARAM_VALUE.C_S_AXI_MAC_REG_CLK_XING { MODELPARAM_VALUE.C_S_AXI_MAC_REG_CLK_XING PARAM_VALUE.C_S_AXI_MAC_REG_CLK_XING } {
    # Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
    set_property value [get_property value ${PARAM_VALUE.C_S_AXI_MAC_REG_CLK_XING}] ${MODELPARAM_VALUE.C_S_AXI_MAC_REG_CLK_XING}
}

proc update_MODELPARAM_VALUE.C_S_AXI_MAC_PKT_BASEADDR { MODELPARAM_VALUE.C_S_AXI_MAC_PKT_BASEADDR PARAM_VALUE.C_S_AXI_MAC_PKT_BASEADDR } {
    # Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
    set_property value [get_property value ${PARAM_VALUE.C_S_AXI_MAC_PKT_BASEADDR}] ${MODELPARAM_VALUE.C_S_AXI_MAC_PKT_BASEADDR}
}

proc update_MODELPARAM_VALUE.C_S_AXI_MAC_PKT_HIGHADDR { MODELPARAM_VALUE.C_S_AXI_MAC_PKT_HIGHADDR PARAM_VALUE.C_S_AXI_MAC_PKT_HIGHADDR } {
    # Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
    set_property value [get_property value ${PARAM_VALUE.C_S_AXI_MAC_PKT_HIGHADDR}] ${MODELPARAM_VALUE.C_S_AXI_MAC_PKT_HIGHADDR}
}

proc update_MODELPARAM_VALUE.C_S_AXI_MAC_PKT_MIN_SIZE { MODELPARAM_VALUE.C_S_AXI_MAC_PKT_MIN_SIZE PARAM_VALUE.C_S_AXI_MAC_PKT_MIN_SIZE } {
    # Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
    set_property value [get_property value ${PARAM_VALUE.C_S_AXI_MAC_PKT_MIN_SIZE}] ${MODELPARAM_VALUE.C_S_AXI_MAC_PKT_MIN_SIZE}
}

proc update_MODELPARAM_VALUE.C_S_AXI_MAC_PKT_DATA_WIDTH { MODELPARAM_VALUE.C_S_AXI_MAC_PKT_DATA_WIDTH PARAM_VALUE.C_S_AXI_MAC_PKT_DATA_WIDTH } {
    # Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
    set_property value [get_property value ${PARAM_VALUE.C_S_AXI_MAC_PKT_DATA_WIDTH}] ${MODELPARAM_VALUE.C_S_AXI_MAC_PKT_DATA_WIDTH}
}

proc update_MODELPARAM_VALUE.C_S_AXI_MAC_PKT_ADDR_WIDTH { MODELPARAM_VALUE.C_S_AXI_MAC_PKT_ADDR_WIDTH PARAM_VALUE.C_S_AXI_MAC_PKT_ADDR_WIDTH } {
    # Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
    set_property value [get_property value ${PARAM_VALUE.C_S_AXI_MAC_PKT_ADDR_WIDTH}] ${MODELPARAM_VALUE.C_S_AXI_MAC_PKT_ADDR_WIDTH}
}

proc update_MODELPARAM_VALUE.C_S_AXI_MAC_PKT_USE_WSTRB { MODELPARAM_VALUE.C_S_AXI_MAC_PKT_USE_WSTRB PARAM_VALUE.C_S_AXI_MAC_PKT_USE_WSTRB } {
    # Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
    set_property value [get_property value ${PARAM_VALUE.C_S_AXI_MAC_PKT_USE_WSTRB}] ${MODELPARAM_VALUE.C_S_AXI_MAC_PKT_USE_WSTRB}
}

proc update_MODELPARAM_VALUE.C_S_AXI_MAC_PKT_DPHASE_TIMEOUT { MODELPARAM_VALUE.C_S_AXI_MAC_PKT_DPHASE_TIMEOUT PARAM_VALUE.C_S_AXI_MAC_PKT_DPHASE_TIMEOUT } {
    # Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
    set_property value [get_property value ${PARAM_VALUE.C_S_AXI_MAC_PKT_DPHASE_TIMEOUT}] ${MODELPARAM_VALUE.C_S_AXI_MAC_PKT_DPHASE_TIMEOUT}
}

proc update_MODELPARAM_VALUE.gPhyPortCount { MODELPARAM_VALUE.gPhyPortCount PARAM_VALUE.gPhyPortCount } {
    # Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
    set_property value [get_property value ${PARAM_VALUE.gPhyPortCount}] ${MODELPARAM_VALUE.gPhyPortCount}
}

proc update_MODELPARAM_VALUE.gPhyPortType { MODELPARAM_VALUE.gPhyPortType PARAM_VALUE.gPhyPortType } {
    # Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
    set_property value [get_property value ${PARAM_VALUE.gPhyPortType}] ${MODELPARAM_VALUE.gPhyPortType}
}

proc update_MODELPARAM_VALUE.gSmiPortCount { MODELPARAM_VALUE.gSmiPortCount PARAM_VALUE.gSmiPortCount } {
    # Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
    set_property value [get_property value ${PARAM_VALUE.gSmiPortCount}] ${MODELPARAM_VALUE.gSmiPortCount}
}

proc update_MODELPARAM_VALUE.gEndianness { MODELPARAM_VALUE.gEndianness PARAM_VALUE.gEndianness } {
    # Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
    set_property value [get_property value ${PARAM_VALUE.gEndianness}] ${MODELPARAM_VALUE.gEndianness}
}

proc update_MODELPARAM_VALUE.gEnableActivity { MODELPARAM_VALUE.gEnableActivity PARAM_VALUE.gEnableActivity } {
    # Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
    set_property value [get_property value ${PARAM_VALUE.gEnableActivity}] ${MODELPARAM_VALUE.gEnableActivity}
}

proc update_MODELPARAM_VALUE.gEnableDmaObserver { MODELPARAM_VALUE.gEnableDmaObserver PARAM_VALUE.gEnableDmaObserver } {
    # Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
    set_property value [get_property value ${PARAM_VALUE.gEnableDmaObserver}] ${MODELPARAM_VALUE.gEnableDmaObserver}
}

proc update_MODELPARAM_VALUE.gDmaAddrWidth { MODELPARAM_VALUE.gDmaAddrWidth PARAM_VALUE.gDmaAddrWidth } {
    # Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
    set_property value [get_property value ${PARAM_VALUE.gDmaAddrWidth}] ${MODELPARAM_VALUE.gDmaAddrWidth}
}

proc update_MODELPARAM_VALUE.gDmaDataWidth { MODELPARAM_VALUE.gDmaDataWidth PARAM_VALUE.gDmaDataWidth } {
    # Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
    set_property value [get_property value ${PARAM_VALUE.gDmaDataWidth}] ${MODELPARAM_VALUE.gDmaDataWidth}
}

proc update_MODELPARAM_VALUE.gDmaBurstCountWidth { MODELPARAM_VALUE.gDmaBurstCountWidth PARAM_VALUE.gDmaBurstCountWidth } {
    # Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
    set_property value [get_property value ${PARAM_VALUE.gDmaBurstCountWidth}] ${MODELPARAM_VALUE.gDmaBurstCountWidth}
}

proc update_MODELPARAM_VALUE.gDmaWriteBurstLength { MODELPARAM_VALUE.gDmaWriteBurstLength PARAM_VALUE.gDmaWriteBurstLength } {
    # Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
    set_property value [get_property value ${PARAM_VALUE.gDmaWriteBurstLength}] ${MODELPARAM_VALUE.gDmaWriteBurstLength}
}

proc update_MODELPARAM_VALUE.gDmaReadBurstLength { MODELPARAM_VALUE.gDmaReadBurstLength PARAM_VALUE.gDmaReadBurstLength } {
    # Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
    set_property value [get_property value ${PARAM_VALUE.gDmaReadBurstLength}] ${MODELPARAM_VALUE.gDmaReadBurstLength}
}

proc update_MODELPARAM_VALUE.gDmaWriteFifoLength { MODELPARAM_VALUE.gDmaWriteFifoLength PARAM_VALUE.gDmaWriteFifoLength } {
    # Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
    set_property value [get_property value ${PARAM_VALUE.gDmaWriteFifoLength}] ${MODELPARAM_VALUE.gDmaWriteFifoLength}
}

proc update_MODELPARAM_VALUE.gDmaReadFifoLength { MODELPARAM_VALUE.gDmaReadFifoLength PARAM_VALUE.gDmaReadFifoLength } {
    # Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
    set_property value [get_property value ${PARAM_VALUE.gDmaReadFifoLength}] ${MODELPARAM_VALUE.gDmaReadFifoLength}
}

proc update_MODELPARAM_VALUE.gPacketBufferLocTx { MODELPARAM_VALUE.gPacketBufferLocTx PARAM_VALUE.gPacketBufferLocTx } {
    # Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
    set_property value [get_property value ${PARAM_VALUE.gPacketBufferLocTx}] ${MODELPARAM_VALUE.gPacketBufferLocTx}
}

proc update_MODELPARAM_VALUE.gPacketBufferLocRx { MODELPARAM_VALUE.gPacketBufferLocRx PARAM_VALUE.gPacketBufferLocRx } {
    # Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
    set_property value [get_property value ${PARAM_VALUE.gPacketBufferLocRx}] ${MODELPARAM_VALUE.gPacketBufferLocRx}
}

proc update_MODELPARAM_VALUE.gPacketBufferLog2Size { MODELPARAM_VALUE.gPacketBufferLog2Size PARAM_VALUE.gPacketBufferLog2Size } {
    # Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
    set_property value [get_property value ${PARAM_VALUE.gPacketBufferLog2Size}] ${MODELPARAM_VALUE.gPacketBufferLog2Size}
}

proc update_MODELPARAM_VALUE.gTimerEnablePulse { MODELPARAM_VALUE.gTimerEnablePulse PARAM_VALUE.gTimerEnablePulse } {
    # Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
    set_property value [get_property value ${PARAM_VALUE.gTimerEnablePulse}] ${MODELPARAM_VALUE.gTimerEnablePulse}
}

proc update_MODELPARAM_VALUE.gTimerEnablePulseWidth { MODELPARAM_VALUE.gTimerEnablePulseWidth PARAM_VALUE.gTimerEnablePulseWidth } {
    # Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
    set_property value [get_property value ${PARAM_VALUE.gTimerEnablePulseWidth}] ${MODELPARAM_VALUE.gTimerEnablePulseWidth}
}

proc update_MODELPARAM_VALUE.gTimerPulseRegWidth { MODELPARAM_VALUE.gTimerPulseRegWidth PARAM_VALUE.gTimerPulseRegWidth } {
    # Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
    set_property value [get_property value ${PARAM_VALUE.gTimerPulseRegWidth}] ${MODELPARAM_VALUE.gTimerPulseRegWidth}
}

