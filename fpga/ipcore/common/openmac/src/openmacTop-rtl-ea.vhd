-------------------------------------------------------------------------------
--! @file openmacTop-rtl-ea.vhd
--
--! @brief OpenMAC toplevel file including openMAC, openHUB and openFILTER
--
--! @details This is the openMAC toplevel file including the MAC layer IP-Cores.
--!          Additional components are provided for packet buffer storage.
-------------------------------------------------------------------------------
--
--    (c) B&R, 2013
--
--    Redistribution and use in source and binary forms, with or without
--    modification, are permitted provided that the following conditions
--    are met:
--
--    1. Redistributions of source code must retain the above copyright
--       notice, this list of conditions and the following disclaimer.
--
--    2. Redistributions in binary form must reproduce the above copyright
--       notice, this list of conditions and the following disclaimer in the
--       documentation and/or other materials provided with the distribution.
--
--    3. Neither the name of B&R nor the names of its
--       contributors may be used to endorse or promote products derived
--       from this software without prior written permission. For written
--       permission, please contact office@br-automation.com
--
--    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
--    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
--    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
--    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
--    COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
--    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
--    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
--    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
--    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
--    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
--    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
--    POSSIBILITY OF SUCH DAMAGE.
--
-------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
--! use global library
use work.global.all;
--! use openmac package
use work.openmacPkg.all;

entity openmacTop is
    generic (
        -----------------------------------------------------------------------
        -- Phy configuration
        -----------------------------------------------------------------------
        --! Number of Phy ports
        gPhyPortCount           : natural := 2;
        --! Phy port interface type (Rmii or Mii)
        gPhyPortType            : natural := cPhyPortRmii;
        --! Number of SMI phy ports
        gSmiPortCount           : natural := 1;
        -----------------------------------------------------------------------
        -- General configuration
        -----------------------------------------------------------------------
        --! Endianness ("little" or "big")
        gEndianness             : string := "little";
        --! Enable packet activity generator (e.g. connect to LED)
        gEnableActivity         : natural := cFalse;
        --! Enable DMA observer circuit
        gEnableDmaObserver      : natural := cFalse;
        -----------------------------------------------------------------------
        -- DMA configuration
        -----------------------------------------------------------------------
        --! DMA address width (byte-addressing)
        gDmaAddrWidth           : natural := 32;
        --! DMA data width
        gDmaDataWidth           : natural := 16;
        --! DMA burst count width
        gDmaBurstCountWidth     : natural := 4;
        --! DMA write burst length (Rx packets) [words]
        gDmaWriteBurstLength    : natural := 16;
        --! DMA read burst length (Tx packets) [words]
        gDmaReadBurstLength     : natural := 16;
        --! DMA write FIFO length (Rx packets) [words]
        gDmaWriteFifoLength     : natural := 16;
        --! DMA read FIFO length (Tx packets) [words]
        gDmaReadFifoLength      : natural := 16;
        -----------------------------------------------------------------------
        -- Packet buffer configuration
        -----------------------------------------------------------------------
        --! Packet buffer location for Tx packets
        gPacketBufferLocTx      : natural := cPktBufLocal;
        --! Packet buffer location for Rx packets
        gPacketBufferLocRx      : natural := cPktBufLocal;
        --! Packet buffer log2(size) [log2(bytes)]
        gPacketBufferLog2Size   : natural := 10;
        -----------------------------------------------------------------------
        -- MAC timer configuration
        -----------------------------------------------------------------------
        --! Number of timers
        gTimerCount             : natural := 2;
        --! Enable timer pulse width control
        gTimerEnablePulseWidth  : natural := cFalse;
        --! Timer pulse width register width
        gTimerPulseRegWidth     : natural := 10
    );
    port (
        -----------------------------------------------------------------------
        -- Clock and reset signal pairs
        -----------------------------------------------------------------------
        --! Main clock used for openMAC, openHUB and openFILTER (freq = 50 MHz)
        iClk                    : in    std_logic;
        --! Main reset used for openMAC, openHUB and openFILTER
        iRst                    : in    std_logic;
        --! DMA master clock
        iDmaClk                 : in    std_logic;
        --! DMA master reset
        iDmaRst                 : in    std_logic;
        --! Packet buffer clock
        iPktBufClk              : in    std_logic;
        --! Packet buffer reset
        iPktBufRst              : in    std_logic;
        --! Twice main clock used for Rmii Tx path
        iClk2x                  : in    std_logic;
        -----------------------------------------------------------------------
        -- MAC REG memory mapped slave
        -----------------------------------------------------------------------
        --! MM slave MAC REGISTER chipselect
        iMacReg_chipselect      : in    std_logic;
        --! MM slave MAC REGISTER write
        iMacReg_write           : in    std_logic;
        --! MM slave MAC REGISTER read
        iMacReg_read            : in    std_logic;
        --! MM slave MAC REGISTER waitrequest
        oMacReg_waitrequest     : out   std_logic;
        --! MM slave MAC REGISTER byteenable
        iMacReg_byteenable      : in    std_logic_vector(cMacRegDataWidth/cByteLength-1 downto 0);
        --! MM slave MAC REGISTER address
        iMacReg_address         : in    std_logic_vector(cMacRegAddrWidth-1 downto 0);
        --! MM slave MAC REGISTER writedata
        iMacReg_writedata       : in    std_logic_vector(cMacRegDataWidth-1 downto 0);
        --! MM slave MAC REGISTER readdata
        oMacReg_readdata        : out   std_logic_vector(cMacRegDataWidth-1 downto 0);
        -----------------------------------------------------------------------
        -- MAC TIMER memory mapped slave
        -----------------------------------------------------------------------
        --! MM slave MAC TIMER chipselect
        iMacTimer_chipselect    : in    std_logic;
        --! MM slave MAC TIMER write
        iMacTimer_write         : in    std_logic;
        --! MM slave MAC TIMER read
        iMacTimer_read          : in    std_logic;
        --! MM slave MAC TIMER waitrequest
        oMacTimer_waitrequest   : out   std_logic;
        --! MM slave MAC TIMER address
        iMacTimer_address       : in    std_logic_vector(cMacTimerAddrWidth-1 downto 0);
        --! MM slave MAC TIMER writedata
        iMacTimer_writedata     : in    std_logic_vector(cMacTimerDataWidth-1 downto 0);
        --! MM slave MAC TIMER readdata
        oMacTimer_readdata      : out   std_logic_vector(cMacTimerDataWidth-1 downto 0);
        -----------------------------------------------------------------------
        -- MAC PACKET BUFFER memory mapped slave
        -----------------------------------------------------------------------
        --! MM slave MAC PACKET BUFFER chipselect
        iPktBuf_chipselect      : in    std_logic;
        --! MM slave MAC PACKET BUFFER write
        iPktBuf_write           : in    std_logic;
        --! MM slave MAC PACKET BUFFER read
        iPktBuf_read            : in    std_logic;
        --! MM slave MAC PACKET BUFFER waitrequest
        oPktBuf_waitrequest     : out   std_logic;
        --! MM slave MAC PACKET BUFFER byteenable
        iPktBuf_byteenable      : in    std_logic_vector(cPktBufDataWidth/cByteLength-1 downto 0);
        --! MM slave MAC PACKET BUFFER address (width given by gPacketBufferLog2Size)
        iPktBuf_address         : in    std_logic_vector(gPacketBufferLog2Size-1 downto 0);
        --! MM slave MAC PACKET BUFFER writedata
        iPktBuf_writedata       : in    std_logic_vector(cPktBufDataWidth-1 downto 0);
        --! MM slave MAC PACKET BUFFER readdata
        oPktBuf_readdata        : out   std_logic_vector(cPktBufDataWidth-1 downto 0);
        -----------------------------------------------------------------------
        -- MAC DMA memory mapped master
        -----------------------------------------------------------------------
        --! MM master MAC DMA write
        oDma_write              : out   std_logic;
        --! MM master MAC DMA read
        oDma_read               : out   std_logic;
        --! MM master MAC DMA waitrequest
        iDma_waitrequest        : in    std_logic;
        --! MM master MAC DMA readdatavalid
        iDma_readdatavalid      : in    std_logic;
        --! MM master MAC DMA byteenable
        oDma_byteenable         : out   std_logic_vector(gDmaDataWidth/cByteLength-1 downto 0);
        --! MM master MAC DMA address
        oDma_address            : out   std_logic_vector(gDmaAddrWidth-1 downto 0);
        --! MM master MAC DMA burstcount
        oDma_burstcount         : out   std_logic_vector(gDmaBurstCountWidth-1 downto 0);
        --! MM master MAC DMA burstcounter (holds current burst count value)
        oDma_burstcounter       : out   std_logic_vector(gDmaBurstCountWidth-1 downto 0);
        --! MM master MAC DMA writedata
        oDma_writedata          : out   std_logic_vector(gDmaDataWidth-1 downto 0);
        --! MM master MAC DMA readdata
        iDma_readdata           : in    std_logic_vector(gDmaDataWidth-1 downto 0);
        -----------------------------------------------------------------------
        -- Interrupts
        -----------------------------------------------------------------------
        --! MAC TIMER interrupt
        oMacTimer_interrupt     : out   std_logic;
        --! MAC Tx interrupt
        oMacTx_interrupt        : out   std_logic;
        --! MAC Rx interrupt
        oMacRx_interrupt        : out   std_logic;
        -----------------------------------------------------------------------
        -- Rmii Phy ports
        -----------------------------------------------------------------------
        --! Rmii Rx ports
        iRmii_Rx                : in    tRmiiPathArray(gPhyPortCount-1 downto 0);
        --! Rmii Rx error ports
        iRmii_RxError           : in    std_logic_vector(gPhyPortCount-1 downto 0);
        --! Rmii Tx ports
        oRmii_Tx                : out   tRmiiPathArray(gPhyPortCount-1 downto 0);
        -----------------------------------------------------------------------
        -- Mii Phy ports
        -----------------------------------------------------------------------
        --! Mii Rx ports
        iMii_Rx                 : in    tMiiPathArray(gPhyPortCount-1 downto 0);
        --! Mii Rx error ports
        iMii_RxError            : in    std_logic_vector(gPhyPortCount-1 downto 0);
        --! Mii Rx Clocks
        iMii_RxClk              : in    std_logic_vector(gPhyPortCount-1 downto 0);
        --! Mii Tx ports
        oMii_Tx                 : out   tMiiPathArray(gPhyPortCount-1 downto 0);
        --! Mii Tx Clocks
        iMii_TxClk              : in    std_logic_vector(gPhyPortCount-1 downto 0);
        -----------------------------------------------------------------------
        -- Phy management interface
        -----------------------------------------------------------------------
        --! Phy reset (low-active)
        onPhy_reset             : out   std_logic_vector(gSmiPortCount-1 downto 0);
        --! SMI clock
        oSmi_clk                : out   std_logic_vector(gSmiPortCount-1 downto 0);
        --! SMI data output enable (tri-state buffer)
        oSmi_data_outEnable     : out   std_logic;
        --! SMI data output (tri-state buffer)
        oSmi_data_out           : out   std_logic_vector(gSmiPortCount-1 downto 0);
        --! SMI data input (tri-state buffer)
        iSmi_data_in            : in    std_logic_vector(gSmiPortCount-1 downto 0);
        -----------------------------------------------------------------------
        -- Other ports
        -----------------------------------------------------------------------
        --! Packet activity (enabled with gEnableActivity)
        oActivity               : out   std_logic;
        --! MAC TIMER outputs
        oMacTimer               : out   std_logic_vector(gTimerCount-1 downto 0)
    );
end openmacTop;

architecture rtl of openmacTop is
    ---------------------------------------------------------------------------
    -- Constants
    ---------------------------------------------------------------------------
    --! Lowest index of Phy port
    constant cPhyPortLow    : natural := cHubIntPort+1;
    --! Highest index of Phy port
    constant cPhyPortHigh   : natural := gPhyPortCount+1;
    ---------------------------------------------------------------------------
    -- Component types
    ---------------------------------------------------------------------------
    --! openMAC port type
    type tOpenMacPort is record
        rst                 : std_logic;
        clk                 : std_logic;
        nReg_write          : std_logic;
        reg_selectRam       : std_logic;
        reg_selectCont      : std_logic;
        nReg_byteenable     : std_logic_vector(1 downto 0);
        reg_address         : std_logic_vector(10 downto 1);
        reg_writedata       : std_logic_vector(15 downto 0);
        reg_readdata        : std_logic_vector(15 downto 0);
        nTxInterrupt        : std_logic;
        nRxInterrupt        : std_logic;
        dma_readDone        : std_logic;
        dma_writeDone       : std_logic;
        dma_request         : std_logic;
        nDma_write          : std_logic;
        dma_acknowledge     : std_logic;
        dma_requestOverflow : std_logic;
        dma_readLength      : std_logic_vector(11 downto 0);
        dma_address         : std_logic_vector(gDmaAddrWidth-1 downto 1);
        dma_writedata       : std_logic_vector(15 downto 0);
        dma_readdata        : std_logic_vector(15 downto 0);
        rmii                : tRmii;
        hubRx               : std_logic_vector(1 downto 0);
        macTime             : std_logic_vector(cMacTimeWidth-1 downto 0);
    end record;

    --! Phy management port type
    type tPhyMgmtPort is record
        rst                 : std_logic;
        clk                 : std_logic;
        address             : std_logic_vector(3 downto 1);
        chipselect          : std_logic;
        nByteenable         : std_logic_vector(1 downto 0);
        nWrite              : std_logic;
        writedata           : std_logic_vector(15 downto 0);
        readdata            : std_logic_vector(15 downto 0);
        smiClk              : std_logic;
        smiDataIn           : std_logic;
        smiDataOut          : std_logic;
        smiDataOutEnable    : std_logic;
        nPhyReset           : std_logic;
    end record;

    --! openMAC TIMER port type
    type tOpenMacTimerPort is record
        rst         : std_logic;
        clk         : std_logic;
        write       : std_logic;
        address     : std_logic_vector(3 downto 2);
        writedata   : std_logic_vector(31 downto 0);
        readdata    : std_logic_vector(31 downto 0);
        macTime     : std_logic_vector(cMacTimeWidth-1 downto 0);
        interrupt   : std_logic;
        toggle      : std_logic;
    end record;

    --! openHUB port type
    type tOpenHubPort is record
        rst             : std_logic;
        clk             : std_logic;
        rx              : tRmiiPathArray(cPhyPortHigh downto cHubIntPort);
        tx              : tRmiiPathArray(cPhyPortHigh downto cHubIntPort);
        internPort      : integer range cHubIntPort to cPhyPortHigh;
        transmitMask    : std_logic_vector(cPhyPortHigh downto cHubIntPort);
        receivePort     : integer range 0 to cPhyPortHigh;
    end record;

    --! openFILTER port type
    type tOpenFilterPort is record
        rst                 : std_logic;
        clk                 : std_logic;
        rxIn                : tRmiiPathArray(cPhyPortHigh downto cPhyPortLow);
        rxOut               : tRmiiPathArray(cPhyPortHigh downto cPhyPortLow);
        txIn                : tRmiiPathArray(cPhyPortHigh downto cPhyPortLow);
        txOut               : tRmiiPathArray(cPhyPortHigh downto cPhyPortLow);
        rxError             : std_logic_vector(cPhyPortHigh downto cPhyPortLow);
    end record;

    --! RMII-to-MII converter port type
    type tConvRmiiToMiiPort is record
        rst         : std_logic;
        clk         : std_logic;
        rmiiTx      : tRmiiPathArray(cPhyPortHigh downto cPhyPortLow);
        rmiiRx      : tRmiiPathArray(cPhyPortHigh downto cPhyPortLow);
        miiTx       : tMiiPathArray(cPhyPortHigh downto cPhyPortLow);
        miiTxClk    : std_logic_vector(cPhyPortHigh downto cPhyPortLow);
        miiRx       : tMiiPathArray(cPhyPortHigh downto cPhyPortLow);
        miiRxError  : std_logic_vector(cPhyPortHigh downto cPhyPortLow);
        miiRxClk    : std_logic_vector(cPhyPortHigh downto cPhyPortLow);
    end record;

    --! Packet Buffer single port type
    type tPacketBufferSinglePort is record
        clk         : std_logic;
        rst         : std_logic;
        enable      : std_logic;
        write       : std_logic;
        address     : std_logic_vector(gPacketBufferLog2Size-1 downto logDualis(cPktBufDataWidth/cByteLength));
        byteenable  : std_logic_vector(cPktBufDataWidth/cByteLength-1 downto 0);
        writedata   : std_logic_vector(cPktBufDataWidth-1 downto 0);
        readdata    : std_logic_vector(cPktBufDataWidth-1 downto 0);
    end record;

    --! Packet Buffer dual port type (port to DMA and host)
    type tPacketBufferPort is record
        dma             : tPacketBufferSinglePort;
        dma_ack         : std_logic;
        dma_highWordSel : std_logic;
        host            : tPacketBufferSinglePort;
    end record;

    --! DMA port type
    type tDmaMaster_dmaPort is record
        clk         : std_logic;
        rst         : std_logic; --FIXME: Rename in openMAC_DMAmaster
        reqWrite    : std_logic;
        reqRead     : std_logic;
        reqOverflow : std_logic;
        ackWrite    : std_logic;
        ackRead     : std_logic;
        readError   : std_logic;
        readLength  : std_logic_vector(11 downto 0);
        writeError  : std_logic;
        address     : std_logic_vector(gDmaAddrWidth-1 downto 1);
        writedata   : std_logic_vector(15 downto 0);
        readdata    : std_logic_vector(15 downto 0);
    end record;

    --! Master port type
    type tDmaMaster_masterPort is record
        clk             : std_logic;
        rst             : std_logic; --FIXME: Add to openMAC_DMAmaster
        write           : std_logic;
        read            : std_logic;
        readdatavalid   : std_logic;
        waitrequest     : std_logic;
        address         : std_logic_vector(gDmaAddrWidth-1 downto 0);
        byteenable      : std_logic_vector(gDmaDataWidth/cByteLength-1 downto 0);
        burstcount      : std_logic_vector(gDmaBurstCountWidth-1 downto 0);
        burstcounter    : std_logic_vector(gDmaBurstCountWidth-1 downto 0);
        writedata       : std_logic_vector(gDmaDataWidth-1 downto 0);
        readdata        : std_logic_vector(gDmaDataWidth-1 downto 0);
    end record;

    --! DMA Master port type
    type tDmaMasterPort is record
        dma         : tDmaMaster_dmaPort;
        master      : tDmaMaster_masterPort;
        mac_rxOff   : std_logic;
        mac_txOff   : std_logic;
    end record;

    --! Activity port type
    type tActivityPort is record
        clk         : std_logic;
        rst         : std_logic;
        txEnable    : std_logic;
        rxDataValid : std_logic;
        activity    : std_logic;
    end record;

    ---------------------------------------------------------------------------
    -- Configuration
    ---------------------------------------------------------------------------
    -- MAC TIMER
    --! Function to convert number of timers into generate boolean for
    --! second timer.
    function macTimer_gen2ndTimer (timerCnt : natural) return boolean is
        variable vRet_tmp : boolean;
    begin
        --default
        vRet_tmp := FALSE;

        case timerCnt is
            when 1 =>
                vRet_tmp := FALSE;
            when 2 =>
                vRet_tmp := TRUE;
            when others =>
                assert (FALSE)
                report "The MAC TIMER only supports 1 and 2 timers!"
                severity failure;
        end case;
        return vRet_tmp;
    end function macTimer_gen2ndTimer;

    --! MAC Timer generate second compare timer
    constant cMacTimer_2ndTimer : boolean := macTimer_gen2ndTimer(gTimerCount);

    ---------------------------------------------------------------------------
    -- Memory map
    ---------------------------------------------------------------------------
    --! Select vector for MacReg
    signal selVector_macReg : std_logic_vector(cMemMapCount-1 downto 0);
    --! Alias for select DMA Error
    alias sel_dmaError      : std_logic is selVector_macReg(cMemMapIndex_dmaError);
    --! Alias for select IRQ Table
    alias sel_irqTable      : std_logic is selVector_macReg(cMemMapIndex_irqTable);
    --! Alias for select SMI
    alias sel_smi           : std_logic is selVector_macReg(cMemMapIndex_smi);
    --! Alias for select MAC RAM
    alias sel_macRam        : std_logic is selVector_macReg(cMemMapIndex_macRam);
    --! Alias for select MAC Filter
    alias sel_macFilter     : std_logic is selVector_macReg(cMemMapIndex_macFilter);
    --! Alias for select MAC Content
    alias sel_macCont       : std_logic is selVector_macReg(cMemMapIndex_macCont);

    --! Acknowledge vector for MacReg
    signal ackVector    : tMemAccessAckArray(cMemAccessDelayCount-1 downto 0);
    --! Alias for acknowledge PKT BUF
    alias ack_pktBuf    : tMemAccessAck is ackVector(cMemAccessDelayIndex_pktBuf);
    --! Alias for acknowledge MAC TIMER
    alias ack_macTimer  : tMemAccessAck is ackVector(cMemAccessDelayIndex_macTimer);
    --! Alias for acknowledge MAC REG
    alias ack_macReg    : tMemAccessAck is ackVector(cMemAccessDelayIndex_macReg);

    ---------------------------------------------------------------------------
    -- Interrupt signals
    ---------------------------------------------------------------------------
    --! Interrupt table vector
    signal irqTable         : tMacRegIrqTable; --aliases are used to assign it
    --! Alias for MAC Tx Interrupt
    alias irqTable_macTx    : std_logic is irqTable(cMacRegIrqTable_macTx);
    --! Alias for MAC Rx Interrupt
    alias irqTable_macRx    : std_logic is irqTable(cMacRegIrqTable_macRx);

    ---------------------------------------------------------------------------
    -- DMA error signals
    ---------------------------------------------------------------------------
    --! DMA error table
    signal dmaErrorTable        : tMacDmaErrorTable; --aliases are used to assign it
    --! Alias for DMA read error
    alias dmaErrorTable_read    : std_logic is dmaErrorTable(cMacDmaErrorTable_read);
    --! Alias for DMA write error
    alias dmaErrorTable_write   : std_logic is dmaErrorTable(cMacDmaErrorTable_write);

    ---------------------------------------------------------------------------
    -- RMII registers for latching input and output path (improves timing)
    ---------------------------------------------------------------------------
    --! Rmii Rx paths
    signal rmiiRxPath_reg       : tRmiiPathArray(gPhyPortCount-1 downto 0);
    --! Rmii Rx error paths
    signal rmiiRxPathError_reg  : std_logic_vector(gPhyPortCount-1 downto 0);
    --! Rmii Tx paths
    signal rmiiTxPath_reg       : tRmiiPathArray(gPhyPortCount-1 downto 0);

    ---------------------------------------------------------------------------
    -- MII signals
    ---------------------------------------------------------------------------
    --! Mii Tx paths
    signal miiTxPath        : tMiiPathArray(gPhyPortCount-1 downto 0);

    ---------------------------------------------------------------------------
    -- Instances
    ---------------------------------------------------------------------------
    --! Instance openMAC port
    signal inst_openmac         : tOpenMacPort;
    --! Instance phy management port
    signal inst_phyMgmt         : tPhyMgmtPort;
    --! Instance openMAC TIMER port
    signal inst_openmacTimer    : tOpenMacTimerPort;
    --! Instance openHUB port
    signal inst_openhub         : tOpenHubPort;
    --! Instances openFILTER port
    signal inst_openfilter      : tOpenFilterPort;
    --! Instances RMII-to-MII converter port
    signal inst_convRmiiToMii   : tConvRmiiToMiiPort;
    --! Instance Packet buffer port
    signal inst_pktBuffer       : tPacketBufferPort;
    --! Instance DMA master port
    signal inst_dmaMaster       : tDmaMasterPort;
    --! Instance activity port
    signal inst_activity        : tActivityPort;
begin
    ---------------------------------------------------------------------------
    -- Assign toplevel
    ---------------------------------------------------------------------------
    --! In this block the entity's output ports are assigned to internals.
    TOPLEVELMAP : block
    begin
        --! This process assigns the oMacReg_readdata port to the selected
        --! memory.
        ASSIGN_MACREG_RDDATA : process (
            selVector_macReg,
            irqTable,
            dmaErrorTable,
            iMacReg_byteenable,
            inst_openmac.reg_readdata,
            inst_phyMgmt.readdata
        )
        begin
            --default assignment
            oMacReg_readdata <= (others => cInactivated);

            if sel_macRam = cActivated or sel_macCont = cActivated then
                oMacReg_readdata <= inst_openmac.reg_readdata;

                -- swap bytes if big endian and selected word
                if gEndianness = "big" and iMacReg_byteenable = "11" then
                    oMacReg_readdata <= byteSwap(inst_openmac.reg_readdata);
                end if;
            elsif sel_smi = cActivated then
                oMacReg_readdata <= inst_phyMgmt.readdata;

                -- swap bytes if big endian and selected word
                if gEndianness = "big" and iMacReg_byteenable = "11" then
                    oMacReg_readdata <= byteSwap(inst_phyMgmt.readdata);
                end if;
            elsif sel_irqTable = cActivated then
                oMacReg_readdata <= irqTable;

                -- swap bytes if big endian
                if gEndianness = "big" then
                    oMacReg_readdata <= byteSwap(irqTable);
                end if;
            elsif sel_dmaError = cActivated then
                oMacReg_readdata <= dmaErrorTable;

                -- swap byte if big endian
                if gEndianness = "big" then
                    oMacReg_readdata <= byteSwap(dmaErrorTable);
                end if;
            end if;
        end process ASSIGN_MACREG_RDDATA;

        oMacReg_waitrequest <= not(ack_macReg.write or ack_macReg.read);

        oMacTimer_readdata      <= inst_openmacTimer.readdata;
        oMacTimer_waitrequest   <= not(ack_macTimer.write or ack_macTimer.read);

        oPktBuf_readdata        <= inst_pktBuffer.host.readdata;
        oPktBuf_waitrequest     <= not(ack_pktBuf.write or ack_pktBuf.read);

        oDma_address        <= inst_dmaMaster.master.address;
        oDma_burstcount     <= inst_dmaMaster.master.burstcount;
        oDma_burstcounter   <= inst_dmaMaster.master.burstcounter;
        oDma_byteenable     <= inst_dmaMaster.master.byteenable;
        oDma_read           <= inst_dmaMaster.master.read;
        oDma_write          <= inst_dmaMaster.master.write;
        oDma_writedata      <= inst_dmaMaster.master.writedata;

        oMacTimer_interrupt <= inst_openmacTimer.interrupt;
        oMacTx_interrupt    <= not inst_openmac.nTxInterrupt;
        oMacRx_interrupt    <= not inst_openmac.nRxInterrupt;

        oRmii_Tx <= rmiiTxPath_reg;

        oMii_Tx <= miiTxPath;

        oSmi_clk            <= (others => inst_phyMgmt.smiClk);
        oSmi_data_out       <= (others => inst_phyMgmt.smiDataOut);
        oSmi_data_outEnable <= inst_phyMgmt.smiDataOutEnable;
        onPhy_reset         <= (others => inst_phyMgmt.nPhyReset);

        oActivity <= inst_activity.activity;

        ASSIGNMACTIMER : process (
            inst_openmacTimer
        )
        begin
            for i in oMacTimer'range loop
                if i = 0 then
                    oMacTimer(i) <= inst_openmacTimer.interrupt;
                elsif i = 1 then
                    oMacTimer(i) <= inst_openmacTimer.toggle;
                else
                    -- unsupported timer assigned to zero
                    oMacTimer(i) <= cInactivated;
                end if;
            end loop;
        end process ASSIGNMACTIMER;
    end block TOPLEVELMAP;

    ---------------------------------------------------------------------------
    -- Assign instances
    ---------------------------------------------------------------------------
    --! In this block the instances are assigned.
    INSTANCEMAP : block
    begin
        -----------------------------------------------------------------------
        -- The openMAC
        -----------------------------------------------------------------------
        inst_openmac.rst                <= iRst;
        inst_openmac.clk                <= iClk;
        inst_openmac.nReg_write         <= not iMacReg_write;
        inst_openmac.reg_selectRam      <= sel_macRam;
        inst_openmac.reg_selectCont     <= sel_macCont;
        inst_openmac.nReg_byteenable    <= not iMacReg_byteenable;
        inst_openmac.reg_address        <= iMacReg_address(inst_openmac.reg_address'range);

        --! This process assignes the register writedata. Additionally the DMA
        --! read path and the request acknowlegde signals are assigned from the
        --! configured sources (packet buffer vs. DMA).
        ASSIGN_MAC : process (
            iMacReg_writedata,
            iMacReg_byteenable,
            inst_pktBuffer,
            inst_dmaMaster
        )
            --! DMA ack variable for or'ing all those acks.
            variable vAck_tmp       : std_logic;
            --! Alias for packet buffer high word
            alias pktBufRead_high   : std_logic_vector(15 downto 0) is
                inst_pktBuffer.dma.readdata(cPktBufDataWidth-1 downto cPktBufDataWidth/2);
            --! Alias for packet buffer low word
            alias pktBufRead_low    : std_logic_vector(15 downto 0) is
                inst_pktBuffer.dma.readdata(cPktBufDataWidth/2-1 downto 0);
        begin
            -- no defaults, so take care!

            -------------------------------------------------------------------
            -- writedata is directly assigned, or byte-swapped
            inst_openmac.reg_writedata <= iMacReg_writedata;

            -- swap bytes if big endian and selected word
            if gEndianness = "big" and iMacReg_byteenable = "11" then
                inst_openmac.reg_writedata <= byteSwap(iMacReg_writedata);
            end if;
            -------------------------------------------------------------------

            -------------------------------------------------------------------
            -- Initialize the ack variable to zero, then or it with all sources.
            vAck_tmp := cInactivated;

            -- Assign read acknowledge.
            case gPacketBufferLocTx is
                when cPktBufExtern =>
                    -- Tx packets come from DMA master
                    vAck_tmp := vAck_tmp or inst_dmaMaster.dma.ackRead;
                when cPktBufLocal =>
                    -- Tx packets come from packet buffer
                    vAck_tmp := vAck_tmp or inst_pktBuffer.dma_ack;
                when others =>
                    assert (FALSE)
                    report "The Tx packet buffer location is unknown! Don't know how to ack!"
                    severity failure;
            end case;

            -- Assign write acknowledge.
            case gPacketBufferLocRx is
                when cPktBufExtern =>
                    -- Rx packets go to DMA master
                    vAck_tmp := vAck_tmp or inst_dmaMaster.dma.ackWrite;
                when cPktBufLocal =>
                    -- Rx packets go to packet buffer
                    vAck_tmp := vAck_tmp or inst_pktBuffer.dma_ack;
                when others =>
                    assert (FALSE)
                    report "The Rx packet buffer location is unknown! Don't know how to ack!"
                    severity failure;
            end case;

            -- Assign the variable state to the signal.
            inst_openmac.dma_acknowledge <= vAck_tmp;
            -------------------------------------------------------------------

            -------------------------------------------------------------------
            -- Decide on the readdata source for the DMA.
            case gPacketBufferLocTx is
                when cPktBufExtern =>
                    -- Tx packet come from DMA master
                    inst_openmac.dma_readdata <= inst_dmaMaster.dma.readdata;
                when cPktBufLocal =>
                    -- Tx packets come from packet buffer, but select the right word
                    if inst_pktBuffer.dma_highWordSel = cActivated then
                        inst_openmac.dma_readdata <= pktBufRead_high;
                    else
                        inst_openmac.dma_readdata <= pktBufRead_low;
                    end if;
                when others =>
                    assert (FALSE)
                    report "The Tx packet buffer location is unknown! Don't know the source!"
                    severity failure;
            end case;
        end process ASSIGN_MAC;

        -- Note that internal port is crossed!
        inst_openmac.rmii.rx <= inst_openhub.tx(cHubIntPort);

        -- Clip the hub receive port to two bits
        inst_openmac.hubRx <= std_logic_vector(
            resize(
                to_unsigned(inst_openhub.receivePort, logDualis(cPhyPortHigh)),
            inst_openmac.hubRx'length)
        );

        -- Assign interrupts to irq Table
        irqTable_macRx <= not inst_openmac.nRxInterrupt;
        irqTable_macTx <= not inst_openmac.nTxInterrupt;

        -----------------------------------------------------------------------
        -- The phy management
        -----------------------------------------------------------------------
        inst_phyMgmt.clk            <= iClk;
        inst_phyMgmt.rst            <= iRst;
        inst_phyMgmt.address        <= iMacReg_address(inst_phyMgmt.address'range);
        inst_phyMgmt.chipselect     <= sel_smi;
        inst_phyMgmt.nByteenable    <= not iMacReg_byteenable;
        inst_phyMgmt.nWrite         <= not iMacReg_write;

        inst_phyMgmt.writedata  <=  iMacReg_writedata when gEndianness = "little" else
                                    iMacReg_writedata when gEndianness = "big" and iMacReg_byteenable /= "11" else
                                    byteSwap(iMacReg_writedata);

        inst_phyMgmt.smiDataIn          <= reduceAnd(iSmi_data_in);

        -----------------------------------------------------------------------
        -- The openMAC timer
        -----------------------------------------------------------------------
        inst_openmacTimer.clk       <= iClk;
        inst_openmacTimer.rst       <= iRst;
        inst_openmacTimer.write     <= iMacTimer_write;
        inst_openmacTimer.address   <= iMacTimer_address(inst_openmacTimer.address'range);
        inst_openmacTimer.writedata <= iMacTimer_writedata;

        -- this is the mac time
        inst_openmacTimer.macTime   <= inst_openmac.macTime;

        -----------------------------------------------------------------------
        -- The openHUB
        -----------------------------------------------------------------------
        inst_openhub.rst            <= iRst;
        inst_openhub.clk            <= iClk;
        inst_openhub.internPort     <= cHubIntPort;
        -- Enable all ports
        inst_openhub.transmitMask   <= (others => cActivated);

        --! This process simply assigns the hub ports.
        ASSIGN_HUB : process (
            inst_openmac.rmii.tx,
            inst_openfilter
        )
        begin
            -- no defaults, so take care!

            -- Loop through phy ports and internal hub port (MAC).
            for i in cPhyPortHigh downto cHubIntPort loop
                -- Assign internal port to mac, others to filter.
                if i = cHubIntPort then
                    -- Note that internal port is crossed!
                    inst_openhub.rx(i) <= inst_openmac.rmii.tx;
                else
                    inst_openhub.rx(i) <= inst_openfilter.rxOut(i);
                end if;
            end loop;
        end process ASSIGN_HUB;

        -----------------------------------------------------------------------
        -- The openFILTER(s)
        -----------------------------------------------------------------------
        inst_openfilter.rst <= iRst;
        inst_openfilter.clk <= iClk;

        --! This process assigns the phy ports to the generated filters.
        ASSIGN_FILTERS : process (
            inst_convRmiiToMii,
            inst_openhub,
            rmiiRxPath_reg,
            rmiiRxPathError_reg
        )
        begin
            -- no defaults, so take care!

            -- Loop through phy ports only (internal hub port is skipped).
            for i in cPhyPortHigh downto cPhyPortLow loop
                -- assign from phy or RMII-to-MII converter
                case gPhyPortType is
                    when cPhyPortRmii =>
                        inst_openfilter.rxIn(i)     <= rmiiRxPath_reg(i-(cPhyPortLow));
                        inst_openfilter.rxError(i)  <= rmiiRxPathError_reg(i-(cPhyPortLow));
                    when cPhyPortMii =>
                        inst_openfilter.rxIn(i)     <= inst_convRmiiToMii.rmiiRx(i);
                        -- Filter Rx error is always zero, since converter already dumps packets!
                        inst_openfilter.rxError(i)  <= cInactivated;
                    when others =>
                        assert (FALSE)
                        report "Wrong phy port type in ASSIGN_FILTERS!"
                        severity failure;
                end case;

                -- assign from hub
                inst_openfilter.txIn(i)  <= inst_openhub.tx(i);
            end loop;
        end process ASSIGN_FILTERS;

        -----------------------------------------------------------------------
        -- The RMII-to-MII converter(s)
        -----------------------------------------------------------------------
        inst_convRmiiToMii.clk      <= iClk;
        inst_convRmiiToMii.rst      <= iRst;
        inst_convRmiiToMii.rmiiTx   <= inst_openfilter.txOut;

        inst_convRmiiToMii.miiRx        <= iMii_Rx;
        inst_convRmiiToMii.miiRxError   <= iMii_RxError;
        inst_convRmiiToMii.miiRxClk     <= iMii_RxClk;
        inst_convRmiiToMii.miiTxClk     <= iMii_TxClk;

        -----------------------------------------------------------------------
        -- The PACKET BUFFER
        -----------------------------------------------------------------------
        -- Assign DMA port side
        inst_pktBuffer.dma.clk      <= inst_openmac.clk;
        inst_pktBuffer.dma.rst      <= inst_openmac.rst;
        inst_pktBuffer.dma.enable   <= cActivated;

        -- Note: DMA has data width of 16 bit and Packet Buffer DPRAM hsa 32 bit.
        --       => Conversion necessary!
        assert ((cPktBufDataWidth = 32) and (inst_openmac.dma_writedata'length = 16))
        report "Revise DMA to packet store path. Implementation is fixed to 32/16!"
        severity failure;

        --! This process assigns the openMAC DMA ports to the packet buffer.
        ASSIGN_DMAPORT : process (
            inst_openmac
        )
            --! This variable is assigned to the DMA's word address
            variable vDmaAddrWord_tmp   : std_logic_vector(inst_openmac.dma_address'range);
            --! This variable is assigned to the DMA's dword address
            variable vDmaAddrDword_tmp  : std_logic_vector(vDmaAddrWord_tmp'left downto 2);
        begin
            -- no defaults, so take care!

            -- Assign DMA address to variables
            vDmaAddrWord_tmp    := inst_openmac.dma_address;
            vDmaAddrDword_tmp   := vDmaAddrWord_tmp(vDmaAddrDword_tmp'range); -- only assigned LEFT downto 2

            -- Packet buffer address is for 32 bit (dwords)
            inst_pktBuffer.dma.address <= vDmaAddrDword_tmp(inst_pktBuffer.dma.address'range);

            -- DMA writes words, so duplicate words to packet buffer
            inst_pktBuffer.dma.writedata <= inst_openmac.dma_writedata & inst_openmac.dma_writedata;

            -- Packet buffer write strobe is logically and'd of request and write
            -- Also the packet location is considered!
            if (inst_openmac.dma_request = cActivated and
                inst_openmac.nDma_write = cnActivated and
                gPacketBufferLocRx = cPktBufLocal) then
                inst_pktBuffer.dma.write <= cActivated;
            else
                inst_pktBuffer.dma.write <= cInactivated;
            end if;

            -- Duplicated words are present at writeport, select DMA's word addr bit
            if vDmaAddrWord_tmp(vDmaAddrWord_tmp'right) = cActivated then
                -- select high word
                inst_pktBuffer.dma.byteenable <= "1100";
            else
                -- select low word
                inst_pktBuffer.dma.byteenable <= "0011";
            end if;
        end process ASSIGN_DMAPORT;

        -- Assign Host port side
        inst_pktBuffer.host.clk         <= iPktBufClk;
        inst_pktBuffer.host.rst         <= iPktBufRst;
        inst_pktBuffer.host.enable      <= cActivated;
        inst_pktBuffer.host.write       <= iPktBuf_chipselect and iPktBuf_write;
        inst_pktBuffer.host.byteenable  <= iPktBuf_byteenable;
        inst_pktBuffer.host.address     <= iPktBuf_address(inst_pktBuffer.host.address'range);
        inst_pktBuffer.host.writedata   <= iPktBuf_writedata;

        -----------------------------------------------------------------------
        -- The DMA master
        -----------------------------------------------------------------------
        inst_dmaMaster.dma.clk              <= inst_openmac.clk;
        inst_dmaMaster.dma.rst              <= inst_openmac.rst;
        inst_dmaMaster.dma.reqWrite         <= inst_openmac.dma_request and not inst_openmac.nDma_write;
        inst_dmaMaster.dma.reqRead          <= inst_openmac.dma_request and inst_openmac.nDma_write;
        inst_dmaMaster.dma.reqOverflow      <= inst_openmac.dma_requestOverflow;
        inst_dmaMaster.dma.readLength       <= inst_openmac.dma_readLength;
        inst_dmaMaster.dma.address          <= inst_openmac.dma_address;
        inst_dmaMaster.dma.writedata        <= inst_openmac.dma_writedata;
        inst_dmaMaster.master.clk           <= iDmaClk;
        inst_dmaMaster.master.rst           <= iDmaRst;
        inst_dmaMaster.master.readdatavalid <= iDma_readdatavalid;
        inst_dmaMaster.master.waitrequest   <= iDma_waitrequest;
        inst_dmaMaster.master.readdata      <= iDma_readdata;
        inst_dmaMaster.mac_rxOff            <= inst_openmac.dma_writeDone;
        inst_dmaMaster.mac_txOff            <= inst_openmac.dma_readDone;

        -- Assign DMA error table
        dmaErrorTable_read  <= inst_dmaMaster.dma.readError;
        dmaErrorTable_write <= inst_dmaMaster.dma.writeError;

        -----------------------------------------------------------------------
        -- The activity generator
        -----------------------------------------------------------------------
        inst_activity.clk           <= inst_openmac.clk;
        inst_activity.rst           <= inst_openmac.rst;
        inst_activity.rxDataValid   <= inst_openmac.rmii.rx.enable;
        inst_activity.txEnable      <= inst_openmac.rmii.tx.enable;
    end block INSTANCEMAP;

    ---------------------------------------------------------------------------
    -- Component instatiations
    ---------------------------------------------------------------------------
    --! The openMAC core instance implements the MAC-layer.
    --! All features are enabled to provide time-triggered packet Tx and
    --! dynamic response delay.
    THEOPENMAC : entity work.openMAC
        generic map (
            HighAdr     => gDmaAddrWidth-1,
            Timer       => TRUE,
            TxSyncOn    => TRUE,
            TxDel       => TRUE,
            Simulate    => FALSE
        )
        port map (
            Rst                 => inst_openmac.rst,
            Clk                 => inst_openmac.clk,
            s_nWr               => inst_openmac.nReg_write,
            Sel_Ram             => inst_openmac.reg_selectRam,
            Sel_Cont            => inst_openmac.reg_selectCont,
            S_nBe               => inst_openmac.nReg_byteenable,
            S_Adr               => inst_openmac.reg_address,
            S_Din               => inst_openmac.reg_writedata,
            S_Dout              => inst_openmac.reg_readdata,
            nTx_Int             => inst_openmac.nTxInterrupt,
            nRx_Int             => inst_openmac.nRxInterrupt,
            nTx_BegInt          => open, --unused interrupt
            Dma_Rd_Done         => inst_openmac.dma_readDone,
            Dma_Wr_Done         => inst_openmac.dma_writeDone,
            Dma_Req             => inst_openmac.dma_request,
            Dma_Rw              => inst_openmac.nDma_write,
            Dma_Ack             => inst_openmac.dma_acknowledge,
            Dma_Req_Overflow    => inst_openmac.dma_requestOverflow,
            Dma_Rd_Len          => inst_openmac.dma_readLength,
            Dma_Addr            => inst_openmac.dma_address,
            Dma_Dout            => inst_openmac.dma_writedata,
            Dma_Din             => inst_openmac.dma_readdata,
            rRx_Dat             => inst_openmac.rmii.rx.data,
            rCrs_Dv             => inst_openmac.rmii.rx.enable,
            rTx_Dat             => inst_openmac.rmii.tx.data,
            rTx_En              => inst_openmac.rmii.tx.enable,
            Hub_Rx              => inst_openmac.hubRx,
            Mac_Zeit            => inst_openmac.macTime
        );

    --! The phy management core is an SMI master to communication with the
    --! phys.
    THEPHYMGMT : entity work.phyMgmt
        port map (
            iRst                => inst_phyMgmt.rst,
            iClk                => inst_phyMgmt.clk,
            iAddress            => inst_phyMgmt.address,
            iSelect             => inst_phyMgmt.chipselect,
            inByteenable        => inst_phyMgmt.nByteenable,
            inWrite             => inst_phyMgmt.nWrite,
            iWritedata          => inst_phyMgmt.writedata,
            oReaddata           => inst_phyMgmt.readdata,
            oSmiClk             => inst_phyMgmt.smiClk,
            iSmiDataIn          => inst_phyMgmt.smiDataIn,
            oSmiDataOut         => inst_phyMgmt.smiDataOut,
            oSmiDataOutEnable   => inst_phyMgmt.smiDataOutEnable,
            onPhyReset          => inst_phyMgmt.nPhyReset
        );

    --! The openMAC timer instance provides hardware timers referencing to the
    --! openMAC's MAC Time (Mac_Zeit).
    THEOPENMACTIMER : entity work.openmacTimer
        generic map (
            gMacTimeWidth           => cMacTimeWidth,
            gMacTimer_2ndTimer      => cMacTimer_2ndTimer,
            gTimerPulseRegWidth     => gTimerPulseRegWidth,
            gTimerEnablePulseWidth  => (gTimerEnablePulseWidth /= cFalse)
        )
        port map (
            iRst        => inst_openmacTimer.rst,
            iClk        => inst_openmacTimer.clk,
            iWrite      => inst_openmacTimer.write,
            iAddress    => inst_openmacTimer.address,
            iWritedata  => inst_openmacTimer.writedata,
            oReaddata   => inst_openmacTimer.readdata,
            iMacTime    => inst_openmacTimer.macTime,
            oIrq        => inst_openmacTimer.interrupt,
            oToggle     => inst_openmacTimer.toggle
        );

    ---------------------------------------------------------------------------
    -- Conditional component instatiations
    ---------------------------------------------------------------------------
    --! Generate address decoders for MAC REG memory map.
    GENMACREG_ADDRDEC : for i in cMemMapTable'range generate
        THEADDRDEC : entity work.addrDecode
            generic map (
                gAddrWidth  => iMacReg_address'length,
                gBaseAddr   => cMemMapTable(i).base,
                gHighAddr   => cMemMapTable(i).high
            )
            port map (
                iEnable     => iMacReg_chipselect,
                iAddress    => iMacReg_address,
                oSelect     => selVector_macReg(i)
            );
    end generate GENMACREG_ADDRDEC;

    --! Generate write/read acknowlegde for MAC REG, MAC TIMER and PKT BUFFER.
    GENMACREG_ACK : for i in cMemAccessDelayTable'range generate
        signal selAndWrite  : std_logic;
        signal selAndRead   : std_logic;
        signal ackRst       : std_logic;
        signal ackClk       : std_logic;
    begin
        --! Assign the corresponding clock and reset signals.
        ASSIGN_CLKRST : process (
            iClk,
            iPktBufClk,
            iRst,
            iPktBufRst
        )
        begin
            -- no defaults, so take care!

            case i is
                when cMemAccessDelayIndex_pktBuf =>
                    ackClk  <= iPktBufClk;
                    ackRst <= iPktBufRst;
                when cMemAccessDelayIndex_macTimer =>
                    ackClk <= iClk;
                    ackRst <= iRst;
                when cMemAccessDelayIndex_macReg =>
                    ackClk <= iClk;
                    ackRst <= iRst;
                when others =>
                    assert (FALSE)
                    report "Unknown access delay index. Don't know which ackClk/ackRst to use!"
                    severity failure;
            end case;
        end process ASSIGN_CLKRST;

        --! This process assigns the selAndWrite and selAndRead signals used
        --! to enable the acknowledge generators.
        ASSIGN_SELANDRW : process (
            iPktBuf_chipselect,
            iPktBuf_write,
            iPktBuf_read,
            iMacTimer_chipselect,
            iMacTimer_write,
            iMacTimer_read,
            iMacReg_chipselect,
            iMacReg_write,
            iMacReg_read
        )
        begin
            --default
            selAndWrite <= cInactivated;
            selAndRead  <= cInactivated;

            case i is
                when cMemAccessDelayIndex_pktBuf =>
                    selAndWrite <= iPktBuf_chipselect and iPktBuf_write;
                    selAndRead  <= iPktBuf_chipselect and iPktBuf_read;
                when cMemAccessDelayIndex_macTimer =>
                    selAndWrite <= iMacTimer_chipselect and iMacTimer_write;
                    selAndRead  <= iMacTimer_chipselect and iMacTimer_read;
                when cMemAccessDelayIndex_macReg =>
                    selAndWrite <= iMacReg_chipselect and iMacReg_write;
                    selAndRead  <= iMacReg_chipselect and iMacReg_read;
                when others =>
                    assert (FALSE)
                    report "For generate overrun! Check cMemAccessDelayTable!"
                    severity failure;
            end case;
        end process ASSIGN_SELANDRW;

        --! Generate ack delay for writes.
        GENWR_ACKDELAY : if cMemAccessDelayTable(i).write > 0 generate
            THE_CNT : entity work.cnt
                generic map (
                    gCntWidth   => logDualis(cMemAccessDelayTable(i).write + 1),
                    gTcntVal    => cMemAccessDelayTable(i).write
                )
                port map (
                    iArst   => ackRst,
                    iClk    => ackClk,
                    iEnable => selAndWrite,
                    iSrst   => cInactivated,
                    oCnt    => open,
                    oTcnt   => ackVector(i).write
                );
        end generate GENWR_ACKDELAY;
        --! Generate no ack delay for writes.
        GENWR_NOACKDELAY : if cMemAccessDelayTable(i).write = 0 generate
            ackVector(i).write <= selAndWrite;
        end generate GENWR_NOACKDELAY;

        --! Generate ack delay for reads.
        GENRD_ACKDELAY : if cMemAccessDelayTable(i).read > 0 generate
            THE_CNT : entity work.cnt
                generic map (
                    gCntWidth   => logDualis(cMemAccessDelayTable(i).read + 1),
                    gTcntVal    => cMemAccessDelayTable(i).read
                )
                port map (
                    iArst   => ackRst,
                    iClk    => ackClk,
                    iEnable => selAndRead,
                    iSrst   => cInactivated,
                    oCnt    => open,
                    oTcnt   => ackVector(i).read
                );
        end generate GENRD_ACKDELAY;
        --! generate no ack delay for reads.
        GENRD_NOACKDELAY : if cMemAccessDelayTable(i).read = 0 generate
            ackVector(i).read <= selAndRead;
        end generate GENRD_NOACKDELAY;
    end generate GENMACREG_ACK;

    --! The openHUB instance is only needed if more than one phy port is
    --! configured.
    GEN_HUB : if gPhyPortCount > 1 generate
        THEOPENHUB : entity work.openhub
            generic map (
                gPortCount => gPhyPortCount+1 -- plus MAC port
            )
            port map (
                iRst        => inst_openhub.rst,
                iClk        => inst_openhub.clk,
                iRx         => inst_openhub.rx,
                oTx         => inst_openhub.tx,
                iIntPort    => inst_openhub.internPort,
                iTxMask     => inst_openhub.transmitMask,
                oRxPort     => inst_openhub.receivePort
            );
    end generate GEN_HUB;

    --! In case of HUB bypass assign inst_openhub although!
    GEN_HUBBYPASS : if gPhyPortCount = 1 generate
        --! Port number for external phy port
        constant cExtPort   : natural := cPhyPortLow;
        --! Port number for internal MAC port
        constant cIntPort   : natural := cHubIntPort;
    begin
        inst_openhub.tx(cExtPort)   <= inst_openhub.rx(cIntPort);

        inst_openhub.tx(cIntPort)   <= inst_openhub.rx(cExtPort);

        inst_openhub.receivePort        <= cExtPort;
    end generate GEN_HUBBYPASS;

    --! Generate openFILTER instances for every phy port.
    GEN_FILTER : for i in cPhyPortHigh downto cPhyPortLow generate
        THEOPENFILTER : entity work.openfilter
            port map (
                iRst        => inst_openfilter.rst,
                iClk        => inst_openfilter.clk,
                iRx         => inst_openfilter.rxIn(i),
                oRx         => inst_openfilter.rxOut(i),
                iTx         => inst_openfilter.txIn(i),
                oTx         => inst_openfilter.txOut(i),
                iRxError    => inst_openfilter.rxError(i)
            );
    end generate GEN_FILTER;

    --! Generate MII signals for every phy port. Note that this includes
    --! the RMII-to-MII converter.
    GEN_MII : if gPhyPortType = cPhyPortMii generate
        GEN_CONVERTER : for i in cPhyPortHigh downto cPhyPortLow generate
            -- convert phy port to filter index range
            miiTxPath(i-cHubIntPort-1)  <= inst_convRmiiToMii.miiTx(i);
            THERMII2MIICONVERTER : entity work.convRmiiToMii
                port map (
                    iRst        => inst_convRmiiToMii.rst,
                    iClk        => inst_convRmiiToMii.clk,
                    iRmiiTx     => inst_convRmiiToMii.rmiiTx(i),
                    oRmiiRx     => inst_convRmiiToMii.rmiiRx(i),
                    iMiiRxClk   => inst_convRmiiToMii.miiRxClk(i),
                    iMiiRx      => inst_convRmiiToMii.miiRx(i),
                    iMiiRxError => inst_convRmiiToMii.miiRxError(i),
                    iMiiTxClk   => inst_convRmiiToMii.miiTxClk(i),
                    oMiiTx      => inst_convRmiiToMii.miiTx(i)
                );
        end generate GEN_CONVERTER;
    end generate GEN_MII;

    --! Generate RMII signals for every phy port. The Rx path is latched with
    --! iClk and the Tx path is lactehd with iClk2x.
    GEN_RMII : if gPhyPortType = cPhyPortRmii generate
        --! Latch Rx signals before they come to internals.
        LATCHRXPATH : process(iRst, iClk)
        begin
            if iRst = cActivated then
                rmiiRxPathError_reg <= (others => cInactivated);
                rmiiRxPath_reg      <= (others => cRmiiPathInit);
            elsif rising_edge(iClk) then
                rmiiRxPathError_reg <= iRmii_RxError;
                rmiiRxPath_reg      <= iRmii_Rx;
            end if;
        end process LATCHRXPATH;

        --! Latch Tx signals with falling edge before they leave.
        LATCHTXPATH : process(iRst, iClk2x)
        begin
            if iRst = cActivated then
                rmiiTxPath_reg <= (others => cRmiiPathInit);
            elsif falling_edge(iClk2x) then
                -- convert phy port to filter index range
                for i in gPhyPortCount-1 downto 0 loop
                    rmiiTxPath_reg(i) <= inst_openfilter.txOut(i+cPhyPortLow);
                end loop;
            end if;
        end process LATCHTXPATH;
    end generate GEN_RMII;

    --! Generate PACKET BUFFER DPRAM if Rx or Tx packets are stored localy.
    GEN_PKTBUFFER : if (gPacketBufferLocTx = cPktBufLocal or
                        gPacketBufferLocRx = cPktBufLocal) generate
        --! Packet buffer number of words (e.g. 1024 byte => 256 dword)
        constant cNumberOfWords : natural := 2**(gPacketBufferLog2Size - logDualis(cPktBufDataWidth/cByteLength));
        --! DMA path reset
        signal dmaRst   : std_logic;
        --! DMA path clock
        signal dmaClk   : std_logic;
    begin
        --! This is the packet buffer DPRAM storing Tx and/or Rx packets.
        --! Port A is connected to the openMAC DMA and port B is connected to
        --! the memory mapped slave port (host).
        THEPKTBUF : entity work.dpRam
            generic map (
                gWordWidth      => cPktBufDataWidth,
                gNumberOfWords  => cNumberOfWords,
                gInitFile       => "UNUSED"
            )
            port map (
                iClk_A          => inst_pktBuffer.dma.clk,
                iEnable_A       => inst_pktBuffer.dma.enable,
                iWriteEnable_A  => inst_pktBuffer.dma.write,
                iAddress_A      => inst_pktBuffer.dma.address,
                iByteenable_A   => inst_pktBuffer.dma.byteenable,
                iWritedata_A    => inst_pktBuffer.dma.writedata,
                oReaddata_A     => inst_pktBuffer.dma.readdata,
                iClk_B          => inst_pktBuffer.host.clk,
                iEnable_B       => inst_pktBuffer.host.enable,
                iWriteEnable_B  => inst_pktBuffer.host.write,
                iByteenable_B   => inst_pktBuffer.host.byteenable,
                iAddress_B      => inst_pktBuffer.host.address,
                iWritedata_B    => inst_pktBuffer.host.writedata,
                oReaddata_B     => inst_pktBuffer.host.readdata
            );

        dmaRst  <= inst_pktBuffer.dma.rst;
        dmaClk  <= inst_pktBuffer.dma.clk;

        --! This process generates the DMA acknowlegde pulse. It is generated
        --! for read and/or write requests with same delay.
        DMAACKPULSE : process(dmaRst, dmaClk)
        begin
            if dmaRst = cActivated then
                inst_pktBuffer.dma_ack <= cInactivated;
            elsif rising_edge(dmaClk) then
                -- generate pulse => default
                inst_pktBuffer.dma_ack <= cInactivated;
                if (inst_openmac.dma_request = cActivated and
                    inst_pktBuffer.dma_ack = cInactivated) then
                    -- Check if it is a write or read
                    if inst_openmac.nDma_write = cnActivated then
                        -- It is a write (Rx packet), check generic...
                        if gPacketBufferLocRx = cPktBufLocal then
                            inst_pktBuffer.dma_ack <= cActivated;
                        end if;
                    else
                        -- It is a read (Tx packet), check generic...
                        if gPacketBufferLocTx = cPktBufLocal then
                            inst_pktBuffer.dma_ack <= cActivated;
                        end if;
                    end if;
                end if;
            end if;
        end process DMAACKPULSE;

        --! This process stores the DMA high word select for DMA read path.
        --! Note: This workaround is needed since data is latched after dma ack.
        DMAHIGHWORDSEL : process(dmaRst, dmaClk)
        begin
            if dmaRst = cActivated then
                inst_pktBuffer.dma_highWordSel <= cInactivated;
            elsif rising_edge(dmaClk) then
                if inst_openmac.dma_request = cActivated and inst_openmac.nDma_write = cnInactivated then
                    inst_pktBuffer.dma_highWordSel <= inst_openmac.dma_address(1);
                end if;
            end if;
        end process DMAHIGHWORDSEL;
    end generate GEN_PKTBUFFER;

    --! Generate DMA Master instances if Rx or Tx packets are stored externally.
    GEN_DMAMASTER : if (gPacketBufferLocTx = cPktBufExtern or
                        gPacketBufferLocRx = cPktBufExtern) generate
        --! This is the DMA master handling the Tx and/or Rx packet transfers
        --! to/from the interconnect.
        THEDMAMASTER: entity work.openMAC_DMAmaster
            generic map (
                dma_highadr_g           => gDmaAddrWidth-1,
                fifo_data_width_g       => gDmaDataWidth,
                gen_dma_observer_g      => (gEnableDmaObserver /= cFalse),
                gen_rx_fifo_g           => (gPacketBufferLocRx = cPktBufExtern),
                gen_tx_fifo_g           => (gPacketBufferLocTx = cPktBufExtern),
                m_burstcount_const_g    => TRUE, --TODO: check if okay
                m_burstcount_width_g    => gDmaBurstCountWidth,
                m_rx_burst_size_g       => gDmaWriteBurstLength,
                rx_fifo_word_size_g     => gDmaWriteFifoLength,
                m_tx_burst_size_g       => gDmaReadBurstLength,
                tx_fifo_word_size_g     => gDmaReadFifoLength,
                simulate                => FALSE
            )
            port map (
                dma_clk             => inst_dmaMaster.dma.clk,
                rst                 => inst_dmaMaster.dma.rst,
                dma_req_wr          => inst_dmaMaster.dma.reqWrite,
                dma_req_rd          => inst_dmaMaster.dma.reqRead,
                dma_req_overflow    => inst_dmaMaster.dma.reqOverflow,
                dma_ack_wr          => inst_dmaMaster.dma.ackWrite,
                dma_ack_rd          => inst_dmaMaster.dma.ackRead,
                dma_rd_err          => inst_dmaMaster.dma.readError,
                dma_rd_len          => inst_dmaMaster.dma.readLength,
                dma_wr_err          => inst_dmaMaster.dma.writeError,
                dma_addr            => inst_dmaMaster.dma.address,
                dma_dout            => inst_dmaMaster.dma.writedata,
                dma_din             => inst_dmaMaster.dma.readdata,
                m_clk               => inst_dmaMaster.master.clk,
                --FIXME: m_rst               => inst_dmaMaster.master.rst,
                m_write             => inst_dmaMaster.master.write,
                m_read              => inst_dmaMaster.master.read,
                m_readdatavalid     => inst_dmaMaster.master.readdatavalid,
                m_waitrequest       => inst_dmaMaster.master.waitrequest,
                m_address           => inst_dmaMaster.master.address,
                m_byteenable        => inst_dmaMaster.master.byteenable,
                m_burstcount        => inst_dmaMaster.master.burstcount,
                m_burstcounter      => inst_dmaMaster.master.burstcounter,
                m_writedata         => inst_dmaMaster.master.writedata,
                m_readdata          => inst_dmaMaster.master.readdata,
                mac_rx_off          => inst_dmaMaster.mac_rxOff,
                mac_tx_off          => inst_dmaMaster.mac_txOff
            );
    end generate GEN_DMAMASTER;

    --! Generate MAC activity, which can be used for LED control.
    GEN_ACTIVITY : if gEnableActivity = cTrue generate
        --! The activity generate uses the Tx enable and Rx data valid signal
        --! to detect a packet run.
        THEACTIVITY : entity work.phyActGen
            generic map (
                gActivityFreq   => cActivityFreq,
                gClkFreq        => cClkFreq
            )
            port map (
                iRst        => inst_activity.rst,
                iClk        => inst_activity.clk,
                iTxEnable   => inst_activity.txEnable,
                iRxValid    => inst_activity.rxDataValid,
                oActivity   => inst_activity.activity
            );
    end generate GEN_ACTIVITY;
end rtl;
