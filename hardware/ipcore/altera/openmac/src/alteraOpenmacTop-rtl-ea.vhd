-------------------------------------------------------------------------------
--! @file alteraOpenmacTop-rtl-ea.vhd
--
--! @brief OpenMAC toplevel for Altera
--
--! @details This is the openMAC toplevel for Altera platform.
-------------------------------------------------------------------------------
--
--    (c) B&R, 2014
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

--! Common library
library libcommon;
--! Use common library global package
use libcommon.global.all;

--! Work library
library work;
--! use openmac package
use work.openmacPkg.all;

entity alteraOpenmacTop is
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
        --! Enable pulse timer
        gTimerEnablePulse       : natural := cFalse;
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
        csi_mainClk_clock           : in    std_logic;
        --! Main reset used for openMAC, openHUB and openFILTER
        rsi_mainRst_reset           : in    std_logic;
        --! DMA master clock
        csi_dmaClk_clock            : in    std_logic;
        --! DMA master reset
        rsi_dmaRst_reset            : in    std_logic;
        --! Packet buffer clock
        csi_pktClk_clock            : in    std_logic;
        --! Packet buffer reset
        rsi_pktRst_reset            : in    std_logic;
        --! Twice main clock used for Rmii Tx path
        csi_mainClkx2_clock         : in    std_logic;
        -----------------------------------------------------------------------
        -- MAC REG memory mapped slave
        -----------------------------------------------------------------------
        --! MM slave MAC REGISTER chipselect
        avs_macReg_chipselect       : in    std_logic;
        --! MM slave MAC REGISTER write
        avs_macReg_write            : in    std_logic;
        --! MM slave MAC REGISTER read
        avs_macReg_read             : in    std_logic;
        --! MM slave MAC REGISTER waitrequest
        avs_macReg_waitrequest      : out   std_logic;
        --! MM slave MAC REGISTER byteenable
        avs_macReg_byteenable       : in    std_logic_vector(cMacRegDataWidth/cByteLength-1 downto 0);
        --! MM slave MAC REGISTER address
        avs_macReg_address          : in    std_logic_vector(cMacRegAddrWidth-1 downto 1);
        --! MM slave MAC REGISTER writedata
        avs_macReg_writedata        : in    std_logic_vector(cMacRegDataWidth-1 downto 0);
        --! MM slave MAC REGISTER readdata
        avs_macReg_readdata         : out   std_logic_vector(cMacRegDataWidth-1 downto 0);
        -----------------------------------------------------------------------
        -- MAC TIMER memory mapped slave
        -----------------------------------------------------------------------
        --! MM slave MAC TIMER chipselect
        avs_macTimer_chipselect     : in    std_logic;
        --! MM slave MAC TIMER write
        avs_macTimer_write          : in    std_logic;
        --! MM slave MAC TIMER read
        avs_macTimer_read           : in    std_logic;
        --! MM slave MAC TIMER waitrequest
        avs_macTimer_waitrequest    : out   std_logic;
        --! MM slave MAC TIMER address
        avs_macTimer_address        : in    std_logic_vector(cMacTimerAddrWidth-1 downto 2);
        --! MM slave MAC TIMER byteenable
        avs_macTimer_byteenable     : in    std_logic_vector(cMacTimerDataWidth/cByteLength-1 downto 0);
        --! MM slave MAC TIMER writedata
        avs_macTimer_writedata      : in    std_logic_vector(cMacTimerDataWidth-1 downto 0);
        --! MM slave MAC TIMER readdata
        avs_macTimer_readdata       : out   std_logic_vector(cMacTimerDataWidth-1 downto 0);
        -----------------------------------------------------------------------
        -- MAC PACKET BUFFER memory mapped slave
        -----------------------------------------------------------------------
        --! MM slave MAC PACKET BUFFER chipselect
        avs_pktBuf_chipselect       : in    std_logic;
        --! MM slave MAC PACKET BUFFER write
        avs_pktBuf_write            : in    std_logic;
        --! MM slave MAC PACKET BUFFER read
        avs_pktBuf_read             : in    std_logic;
        --! MM slave MAC PACKET BUFFER waitrequest
        avs_pktBuf_waitrequest      : out   std_logic;
        --! MM slave MAC PACKET BUFFER byteenable
        avs_pktBuf_byteenable       : in    std_logic_vector(cPktBufDataWidth/8-1 downto 0);
        --! MM slave MAC PACKET BUFFER address (width given by gPacketBufferLog2Size)
        avs_pktBuf_address          : in    std_logic_vector(gPacketBufferLog2Size-1 downto 2);
        --! MM slave MAC PACKET BUFFER writedata
        avs_pktBuf_writedata        : in    std_logic_vector(cPktBufDataWidth-1 downto 0);
        --! MM slave MAC PACKET BUFFER readdata
        avs_pktBuf_readdata         : out   std_logic_vector(cPktBufDataWidth-1 downto 0);
        -----------------------------------------------------------------------
        -- MAC DMA memory mapped master
        -----------------------------------------------------------------------
        --! MM master MAC DMA write
        avm_dma_write               : out   std_logic;
        --! MM master MAC DMA read
        avm_dma_read                : out   std_logic;
        --! MM master MAC DMA waitrequest
        avm_dma_waitrequest         : in    std_logic;
        --! MM master MAC DMA readdatavalid
        avm_dma_readdatavalid       : in    std_logic;
        --! MM master MAC DMA byteenable
        avm_dma_byteenable          : out   std_logic_vector(gDmaDataWidth/8-1 downto 0);
        --! MM master MAC DMA address
        avm_dma_address             : out   std_logic_vector(gDmaAddrWidth-1 downto 0);
        --! MM master MAC DMA burstcount
        avm_dma_burstcount          : out   std_logic_vector(gDmaBurstCountWidth-1 downto 0);
        --! MM master MAC DMA writedata
        avm_dma_writedata           : out   std_logic_vector(gDmaDataWidth-1 downto 0);
        --! MM master MAC DMA readdata
        avm_dma_readdata            : in    std_logic_vector(gDmaDataWidth-1 downto 0);
        -----------------------------------------------------------------------
        -- Interrupts
        -----------------------------------------------------------------------
        --! MAC TIMER interrupt
        ins_timerIrq_irq            : out   std_logic;
        --! MAC interrupt
        ins_macIrq_irq              : out   std_logic;
        --! MAC Timer pulse interrupt
        ins_timerPulse_irq          : out   std_logic;
        -----------------------------------------------------------------------
        -- Rmii Phy ports
        -----------------------------------------------------------------------
        --! Rmii Rx Crs data valid ports
        coe_rmii_rxCrsDataValid    : in    std_logic_vector(gPhyPortCount-1 downto 0);
        --! Rmii Rx data ports
        coe_rmii_rxData         : in    std_logic_vector(gPhyPortCount*2-1 downto 0);
        --! Rmii Rx error ports
        coe_rmii_rxError        : in    std_logic_vector(gPhyPortCount-1 downto 0);
        --! Rmii Tx enable ports
        coe_rmii_txEnable       : out   std_logic_vector(gPhyPortCount-1 downto 0);
        --! Rmii Tx data ports
        coe_rmii_txData         : out   std_logic_vector(gPhyPortCount*2-1 downto 0);

        -----------------------------------------------------------------------
        -- Mii Phy ports
        -----------------------------------------------------------------------
        --! Mii Rx data valid ports
        coe_mii_rxDataValid     : in    std_logic_vector(gPhyPortCount-1 downto 0);
        --! Mii Rx data ports
        coe_mii_rxData          : in    std_logic_vector(gPhyPortCount*4-1 downto 0);
        --! Mii Rx error ports
        coe_mii_rxError         : in    std_logic_vector(gPhyPortCount-1 downto 0);
        --! Mii Rx Clocks
        coe_mii_rxClk           : in    std_logic_vector(gPhyPortCount-1 downto 0);
        --! Mii Tx enable ports
        coe_mii_txEnable        : out   std_logic_vector(gPhyPortCount-1 downto 0);
        --! Mii Tx data ports
        coe_mii_txData          : out   std_logic_vector(gPhyPortCount*4-1 downto 0);
        --! Mii Tx Clocks
        coe_mii_txClk           : in    std_logic_vector(gPhyPortCount-1 downto 0);
        -----------------------------------------------------------------------
        -- Phy management interface
        -----------------------------------------------------------------------
        --! Phy reset (low-active)
        coe_smi_nPhyRst         : out   std_logic_vector(gSmiPortCount-1 downto 0);
        --! SMI clock
        coe_smi_clk             : out   std_logic_vector(gSmiPortCount-1 downto 0);
        --! SMI data I/OI (tri-state buffer)
        coe_smi_dio             : inout std_logic_vector(gSmiPortCount-1 downto 0);
        -----------------------------------------------------------------------
        -- Other ports
        -----------------------------------------------------------------------
        --! Packet activity (enabled with gEnableActivity)
        coe_pktActivity         : out   std_logic
    );
end alteraOpenmacTop;

architecture rtl of alteraOpenmacTop is
    --! Byte address of macReg
    signal macReg_address       : std_logic_vector(avs_macReg_address'left downto 0);
    --! Byte address of macTimer
    signal macTimer_address     : std_logic_vector(avs_macTimer_address'left downto 0);
    --! Byte address of pktBuf
    signal pktBuf_address       : std_logic_vector(avs_pktBuf_address'left downto 0);

    --! Mac Tx interrupt
    signal macTx_interrupt      : std_logic;
    --! Mac Rx interrupt
    signal macRx_interrupt      : std_logic;

    --! Rmii Tx path
    signal rmiiTx               : tRmiiPathArray(gPhyPortCount-1 downto 0);
    --! Rmii Rx path
    signal rmiiRx               : tRmiiPathArray(gPhyPortCount-1 downto 0);
    --! Mii Tx path
    signal miiTx                : tMiiPathArray(gPhyPortCount-1 downto 0);
    --! Mii Rx path
    signal miiRx                : tMiiPathArray(gPhyPortCount-1 downto 0);

    --! Smi tri-state-buffer input
    signal smi_data_in          : std_logic_vector(gSmiPortCount-1 downto 0);
    --! Smi tri-state-buffer output
    signal smi_data_out         : std_logic_vector(gSmiPortCount-1 downto 0);
    --! Smi tri-state-buffer output enable
    signal smi_data_outEnable   : std_logic;
begin
    ---------------------------------------------------------------------------
    -- Map outputs
    ---------------------------------------------------------------------------
    -- Mac interrupts are or'd to single line.
    ins_macIrq_irq <= macTx_interrupt or macRx_interrupt;

    -- Phy Tx path
    rmiiPathArrayToStdLogicVector(
        iVector => rmiiTx,
        oEnable => coe_rmii_txEnable,
        oData   => coe_rmii_txData
    );

    miiPathArrayToStdLogicVector(
        iVector => miiTx,
        oEnable => coe_mii_txEnable,
        oData   => coe_mii_txData
    );

    ---------------------------------------------------------------------------
    -- Map inputs
    ---------------------------------------------------------------------------
    -- Assign byte addresses.
    macReg_address      <= avs_macReg_address & "0"; --word to byte
    macTimer_address    <= avs_macTimer_address & "00"; --dword to byte
    pktBuf_address      <= avs_pktBuf_address & "00"; --dword to byte

    -- Phy Rx path
    stdLogicVectorToRmiiPathArray(
        iEnable => coe_rmii_rxCrsDataValid,
        iData   => coe_rmii_rxData,
        oVector => rmiiRx
    );

    stdLogicVectorToMiiPathArray(
        iEnable => coe_mii_rxDataValid,
        iData   => coe_mii_rxData,
        oVector => miiRx
    );

    ---------------------------------------------------------------------------
    -- Map IOs
    ---------------------------------------------------------------------------
    -- Assign SMI IO buffers
    coe_smi_dio <=  smi_data_out    when smi_data_outEnable = cActivated else
                    (others => 'Z');
    -- Simply assign the input vector.
    smi_data_in <= coe_smi_dio;

    --! This is the openMAC toplevel instantiation.
    THEOPENMACTOP : entity work.openmacTop
    generic map (
        gPhyPortCount           => gPhyPortCount,
        gPhyPortType            => gPhyPortType,
        gSmiPortCount           => gSmiPortCount,
        gEndianness             => gEndianness,
        gEnableActivity         => gEnableActivity,
        gEnableDmaObserver      => gEnableDmaObserver,
        gDmaAddrWidth           => gDmaAddrWidth,
        gDmaDataWidth           => gDmaDataWidth,
        gDmaBurstCountWidth     => gDmaBurstCountWidth,
        gDmaWriteBurstLength    => gDmaWriteBurstLength,
        gDmaReadBurstLength     => gDmaReadBurstLength,
        gDmaWriteFifoLength     => gDmaWriteFifoLength,
        gDmaReadFifoLength      => gDmaReadFifoLength,
        gPacketBufferLocTx      => gPacketBufferLocTx,
        gPacketBufferLocRx      => gPacketBufferLocRx,
        gPacketBufferLog2Size   => gPacketBufferLog2Size,
        gTimerEnablePulse       => gTimerEnablePulse,
        gTimerEnablePulseWidth  => gTimerEnablePulseWidth,
        gTimerPulseRegWidth     => gTimerPulseRegWidth
    )
    port map (
        iClk                    => csi_mainClk_clock,
        iRst                    => rsi_mainRst_reset,
        iDmaClk                 => csi_dmaClk_clock,
        iDmaRst                 => rsi_dmaRst_reset,
        iPktBufClk              => csi_pktClk_clock,
        iPktBufRst              => rsi_pktRst_reset,
        iClk2x                  => csi_mainClkx2_clock,
        iMacReg_chipselect      => avs_macReg_chipselect,
        iMacReg_write           => avs_macReg_write,
        iMacReg_read            => avs_macReg_read,
        oMacReg_waitrequest     => avs_macReg_waitrequest,
        iMacReg_byteenable      => avs_macReg_byteenable,
        iMacReg_address         => macReg_address,
        iMacReg_writedata       => avs_macReg_writedata,
        oMacReg_readdata        => avs_macReg_readdata,
        iMacTimer_chipselect    => avs_macTimer_chipselect,
        iMacTimer_write         => avs_macTimer_write,
        iMacTimer_read          => avs_macTimer_read,
        oMacTimer_waitrequest   => avs_macTimer_waitrequest,
        iMacTimer_address       => macTimer_address,
        iMacTimer_byteenable    => avs_macTimer_byteenable,
        iMacTimer_writedata     => avs_macTimer_writedata,
        oMacTimer_readdata      => avs_macTimer_readdata,
        iPktBuf_chipselect      => avs_pktBuf_chipselect,
        iPktBuf_write           => avs_pktBuf_write,
        iPktBuf_read            => avs_pktBuf_read,
        oPktBuf_waitrequest     => avs_pktBuf_waitrequest,
        iPktBuf_byteenable      => avs_pktBuf_byteenable,
        iPktBuf_address         => pktBuf_address,
        iPktBuf_writedata       => avs_pktBuf_writedata,
        oPktBuf_readdata        => avs_pktBuf_readdata,
        oDma_write              => avm_dma_write,
        oDma_read               => avm_dma_read,
        iDma_waitrequest        => avm_dma_waitrequest,
        iDma_readdatavalid      => avm_dma_readdatavalid,
        oDma_byteenable         => avm_dma_byteenable,
        oDma_address            => avm_dma_address,
        oDma_burstcount         => avm_dma_burstcount,
        oDma_burstcounter       => open, --current burst counter state unused
        oDma_writedata          => avm_dma_writedata,
        iDma_readdata           => avm_dma_readdata,
        oMacTimer_interrupt     => ins_timerIrq_irq,
        oMacTimer_pulse         => ins_timerPulse_irq,
        oMacTx_interrupt        => macTx_interrupt,
        oMacRx_interrupt        => macRx_interrupt,
        iRmii_Rx                => rmiiRx,
        iRmii_RxError           => coe_rmii_rxError,
        oRmii_Tx                => rmiiTx,
        iMii_Rx                 => miiRx,
        iMii_RxError            => coe_mii_rxError,
        iMii_RxClk              => coe_mii_rxClk,
        oMii_Tx                 => miiTx,
        iMii_TxClk              => coe_mii_txClk,
        onPhy_reset             => coe_smi_nPhyRst,
        oSmi_clk                => coe_smi_clk,
        oSmi_data_outEnable     => smi_data_outEnable,
        oSmi_data_out           => smi_data_out,
        iSmi_data_in            => smi_data_in,
        oActivity               => coe_pktActivity
    );
end rtl;
