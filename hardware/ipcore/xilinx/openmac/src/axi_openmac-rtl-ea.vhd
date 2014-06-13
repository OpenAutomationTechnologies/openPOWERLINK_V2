-------------------------------------------------------------------------------
--! @file axi_openmac-rtl-ea.vhd
--
--! @brief OpenMAC toplevel for Xilinx
--
--! @details This is the openMAC toplevel for Xilinx platform with AXI.
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

--! Common Xilinx library
library proc_common_v3_00_a;
use proc_common_v3_00_a.proc_common_pkg.all;
use proc_common_v3_00_a.ipif_pkg.all;

--! AXI Lite IPIF library
library axi_lite_ipif_v1_01_a;
--! Use AXI lite ipif
use axi_lite_ipif_v1_01_a.axi_lite_ipif;

--! AXI Master Burst library
library axi_master_burst_v1_00_a;
--! Use AXI master burst
use axi_master_burst_v1_00_a.axi_master_burst;

--! Unisim library
library unisim;
--! Use ODDR2 instance primitive
use unisim.vcomponents.oddr2;

entity axi_openmac is
    generic (
        -----------------------------------------------------------------------
        -- General parameters
        -----------------------------------------------------------------------
        --! Xilinx FPGA familiy
        C_FAMILY                        : string := "spartan6";
        -----------------------------------------------------------------------
        -- AXI DMA
        -----------------------------------------------------------------------
        --! AXI master DMA address width
        C_M_AXI_MAC_DMA_ADDR_WIDTH      : integer := 32;
        --! AXI master DMA data width
        C_M_AXI_MAC_DMA_DATA_WIDTH      : integer := 32;
        --! AXI master DMA native data width
        C_M_AXI_MAC_DMA_NATIVE_DWIDTH   : integer := 32;
        --! AXI master DMA burst length width
        C_M_AXI_MAC_DMA_LENGTH_WIDTH    : integer := 12;
        --! AXI master DMA burst length
        C_M_AXI_MAC_DMA_MAX_BURST_LEN   : integer := 16;
        -----------------------------------------------------------------------
        -- AXI REG
        -----------------------------------------------------------------------
        --! AXI slave REG address ranges
        C_S_AXI_MAC_REG_NUM_ADDR_RANGES : integer := 2;
        --! AXI slave REG range 0 base
        C_S_AXI_MAC_REG_RNG0_BASEADDR   : std_logic_vector := x"ffffffff";
        --! AXI slave REG range 0 high
        C_S_AXI_MAC_REG_RNG0_HIGHADDR   : std_logic_vector := x"00000000";
        --! AXI slave REG range 1 base
        C_S_AXI_MAC_REG_RNG1_BASEADDR   : std_logic_vector := x"ffffffff";
        --! AXI slave REG range 1 high
        C_S_AXI_MAC_REG_RNG1_HIGHADDR   : std_logic_vector := x"00000000";
        --! AXI slave REG minimum size
        C_S_AXI_MAC_REG_MIN_SIZE        : std_logic_vector := x"00001fff";
        --! AXI slave REG data width
        C_S_AXI_MAC_REG_DATA_WIDTH      : integer := 32;
        --! AXI slave REG address width
        C_S_AXI_MAC_REG_ADDR_WIDTH      : integer := 32;
        --! AXI slave REG clock frequency
        C_S_AXI_MAC_REG_ACLK_FREQ_HZ    : integer := 50000000;
        --! AXI slave REG use write strobes
        C_S_AXI_MAC_REG_USE_WSTRB       : integer := 1;
        --! AXI slave REG enable data phase timeout timer
        C_S_AXI_MAC_REG_DPHASE_TIMEOUT  : integer := 0;
        -----------------------------------------------------------------------
        -- AXI REG
        -----------------------------------------------------------------------
        --! AXI slave PKT base
        C_S_AXI_MAC_PKT_BASEADDR        : std_logic_vector := x"ffffffff";
        --! AXI slave PKT high
        C_S_AXI_MAC_PKT_HIGHADDR        : std_logic_vector := x"00000000";
        --! AXI slave REG minimum size
        C_S_AXI_MAC_PKT_MIN_SIZE        : std_logic_vector := x"0000ffff";
        --! AXI slave PKT data width
        C_S_AXI_MAC_PKT_DATA_WIDTH      : integer := 32;
        --! AXI slave PKT address width
        C_S_AXI_MAC_PKT_ADDR_WIDTH      : integer := 32;
        --! AXI slave PKT use write strobes
        C_S_AXI_MAC_PKT_USE_WSTRB       : integer := 1;
        --! AXI slave PKT enable data phase timeout timer
        C_S_AXI_MAC_PKT_DPHASE_TIMEOUT  : integer := 0;
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
        iClk50                      : in    std_logic;
        --! Twice main clock used for Rmii Tx path
        iClk100                     : in    std_logic;
        -----------------------------------------------------------------------
        -- MAC REG memory mapped slave
        -----------------------------------------------------------------------
        --! AXI slave REG clock
        S_AXI_MAC_REG_ACLK          : in    std_logic;
        --! AXI slave REG reset (low-active)
        S_AXI_MAC_REG_ARESETN       : in    std_logic;
        --! AXI slave REG address read valid
        S_AXI_MAC_REG_ARVALID       : in    std_logic;
        --! AXI slave REG address write valid
        S_AXI_MAC_REG_AWVALID       : in    std_logic;
        --! AXI slave REG response ready
        S_AXI_MAC_REG_BREADY        : in    std_logic;
        --! AXI slave REG read ready
        S_AXI_MAC_REG_RREADY        : in    std_logic;
        --! AXI slave REG write valid
        S_AXI_MAC_REG_WVALID        : in    std_logic;
        --! AXI slave REG read address
        S_AXI_MAC_REG_ARADDR        : in    std_logic_vector(C_S_AXI_MAC_REG_ADDR_WIDTH-1 downto 0);
        --! AXI slave REG write address
        S_AXI_MAC_REG_AWADDR        : in    std_logic_vector(C_S_AXI_MAC_REG_ADDR_WIDTH-1 downto 0);
        --! AXI slave REG write data
        S_AXI_MAC_REG_WDATA         : in    std_logic_vector(C_S_AXI_MAC_REG_DATA_WIDTH-1 downto 0);
        --! AXI slave REG write strobe
        S_AXI_MAC_REG_WSTRB         : in    std_logic_vector(C_S_AXI_MAC_REG_DATA_WIDTH/8-1 downto 0);
        --! AXI slave REG read address ready
        S_AXI_MAC_REG_ARREADY       : out   std_logic;
        --! AXI slave REG write address ready
        S_AXI_MAC_REG_AWREADY       : out   std_logic;
        --! AXI slave REG write response valid
        S_AXI_MAC_REG_BVALID        : out   std_logic;
        --! AXI slave REG read valid
        S_AXI_MAC_REG_RVALID        : out   std_logic;
        --! AXI slave REG write ready
        S_AXI_MAC_REG_WREADY        : out   std_logic;
        --! AXI slave REG write response
        S_AXI_MAC_REG_BRESP         : out   std_logic_vector(1 downto 0);
        --! AXI slave REG read data
        S_AXI_MAC_REG_RDATA         : out   std_logic_vector(C_S_AXI_MAC_REG_DATA_WIDTH-1 downto 0);
        --! AXI slave REG read response
        S_AXI_MAC_REG_RRESP         : out   std_logic_vector(1 downto 0);
        -----------------------------------------------------------------------
        -- MAC PACKET BUFFER memory mapped slave
        -----------------------------------------------------------------------
        --! AXI slave PKT clock
        S_AXI_MAC_PKT_ACLK          : in    std_logic;
        --! AXI slave PKT reset (low-active)
        S_AXI_MAC_PKT_ARESETN       : in    std_logic;
        --! AXI slave PKT address read valid
        S_AXI_MAC_PKT_ARVALID       : in    std_logic;
        --! AXI slave PKT address write valid
        S_AXI_MAC_PKT_AWVALID       : in    std_logic;
        --! AXI slave PKT response ready
        S_AXI_MAC_PKT_BREADY        : in    std_logic;
        --! AXI slave PKT read ready
        S_AXI_MAC_PKT_RREADY        : in    std_logic;
        --! AXI slave PKT write valid
        S_AXI_MAC_PKT_WVALID        : in    std_logic;
        --! AXI slave PKT read address
        S_AXI_MAC_PKT_ARADDR        : in    std_logic_vector(C_S_AXI_MAC_PKT_ADDR_WIDTH-1 downto 0);
        --! AXI slave PKT write address
        S_AXI_MAC_PKT_AWADDR        : in    std_logic_vector(C_S_AXI_MAC_PKT_ADDR_WIDTH-1 downto 0);
        --! AXI slave PKT write data
        S_AXI_MAC_PKT_WDATA         : in    std_logic_vector(C_S_AXI_MAC_PKT_DATA_WIDTH-1 downto 0);
        --! AXI slave PKT write strobe
        S_AXI_MAC_PKT_WSTRB         : in    std_logic_vector(C_S_AXI_MAC_PKT_DATA_WIDTH/8-1 downto 0);
        --! AXI slave PKT read address ready
        S_AXI_MAC_PKT_ARREADY       : out   std_logic;
        --! AXI slave PKT write address ready
        S_AXI_MAC_PKT_AWREADY       : out   std_logic;
        --! AXI slave PKT write response valid
        S_AXI_MAC_PKT_BVALID        : out   std_logic;
        --! AXI slave PKT read valid
        S_AXI_MAC_PKT_RVALID        : out   std_logic;
        --! AXI slave PKT write ready
        S_AXI_MAC_PKT_WREADY        : out   std_logic;
        --! AXI slave PKT write response
        S_AXI_MAC_PKT_BRESP         : out   std_logic_vector(1 downto 0);
        --! AXI slave PKT read data
        S_AXI_MAC_PKT_RDATA         : out   std_logic_vector(C_S_AXI_MAC_PKT_DATA_WIDTH-1 downto 0);
        --! AXI slave PKT read response
        S_AXI_MAC_PKT_RRESP         : out   std_logic_vector(1 downto 0);
        -----------------------------------------------------------------------
        -- MAC DMA memory mapped master
        -----------------------------------------------------------------------
        --! DMA master clock
        M_AXI_MAC_DMA_ACLK          : in    std_logic;
        --! DMA master reset (low-active)
        M_AXI_MAC_DMA_ARESETN       : in    std_logic;
        --! AXI master DMA error
        M_AXI_MAC_DMA_MD_ERROR      : out   std_logic;
        --! AXI master DMA read address ready
        M_AXI_MAC_DMA_ARREADY       : in    std_logic;
        --! AXI master DMA write address ready
        M_AXI_MAC_DMA_AWREADY       : in    std_logic;
        --! AXI master DMA write response ready
        M_AXI_MAC_DMA_BVALID        : in    std_logic;
        --! AXI master DMA read last
        M_AXI_MAC_DMA_RLAST         : in    std_logic;
        --! AXI master DMA read valid
        M_AXI_MAC_DMA_RVALID        : in    std_logic;
        --! AXI master DMA write ready
        M_AXI_MAC_DMA_WREADY        : in    std_logic;
        --! AXI master DMA write response
        M_AXI_MAC_DMA_BRESP         : in    std_logic_vector(1 downto 0);
        --! AXI master DMA read data
        M_AXI_MAC_DMA_RDATA         : in    std_logic_vector(C_M_AXI_MAC_DMA_DATA_WIDTH-1 downto 0);
        --! AXI master DMA read response
        M_AXI_MAC_DMA_RRESP         : in    std_logic_vector(1 downto 0);
        --! AXI master DMA read address valid
        M_AXI_MAC_DMA_ARVALID       : out   std_logic;
        --! AXI master DMA write address valid
        M_AXI_MAC_DMA_AWVALID       : out   std_logic;
        --! AXI master DMA response ready
        M_AXI_MAC_DMA_BREADY        : out   std_logic;
        --! AXI master DMA read ready
        M_AXI_MAC_DMA_RREADY        : out   std_logic;
        --! AXI master DMA write last
        M_AXI_MAC_DMA_WLAST         : out   std_logic;
        --! AXI master DMA write valid
        M_AXI_MAC_DMA_WVALID        : out   std_logic;
        --! AXI master DMA read address
        M_AXI_MAC_DMA_ARADDR        : out   std_logic_vector(C_M_AXI_MAC_DMA_ADDR_WIDTH-1 downto 0);
        --! AXI master DMA burst type
        M_AXI_MAC_DMA_ARBURST       : out   std_logic_vector(1 downto 0);
        --! AXI master DMA memory type
        M_AXI_MAC_DMA_ARCACHE       : out   std_logic_vector(3 downto 0);
        --! AXI master DMA burst length
        M_AXI_MAC_DMA_ARLEN         : out   std_logic_vector(7 downto 0);
        --! AXI master DMA protection type
        M_AXI_MAC_DMA_ARPROT        : out   std_logic_vector(2 downto 0);
        --! AXI master DMA burst size
        M_AXI_MAC_DMA_ARSIZE        : out   std_logic_vector(2 downto 0);
        --! AXI master DMA write address
        M_AXI_MAC_DMA_AWADDR        : out   std_logic_vector(C_M_AXI_MAC_DMA_ADDR_WIDTH-1 downto 0);
        --! AXI master DMA burst type
        M_AXI_MAC_DMA_AWBURST       : out   std_logic_vector(1 downto 0);
        --! AXI master DMA memory type
        M_AXI_MAC_DMA_AWCACHE       : out   std_logic_vector(3 downto 0);
        --! AXI master DMA burst length
        M_AXI_MAC_DMA_AWLEN         : out   std_logic_vector(7 downto 0);
        --! AXI master DMA protection type
        M_AXI_MAC_DMA_AWPROT        : out   std_logic_vector(2 downto 0);
        --! AXI master DMA burst size
        M_AXI_MAC_DMA_AWSIZE        : out   std_logic_vector(2 downto 0);
        --! AXI master DMA write data
        M_AXI_MAC_DMA_WDATA         : out   std_logic_vector(C_M_AXI_MAC_DMA_DATA_WIDTH-1 downto 0);
        --! AXI master DMA write strobe
        M_AXI_MAC_DMA_WSTRB         : out   std_logic_vector(C_M_AXI_MAC_DMA_DATA_WIDTH/8-1 downto 0);
        -----------------------------------------------------------------------
        -- Interrupts
        -----------------------------------------------------------------------
        --! MAC TIMER interrupt
        TIMER_IRQ                   : out   std_logic;
        --! MAC interrupt
        MAC_IRQ                     : out   std_logic;
        -----------------------------------------------------------------------
        -- Rmii Phy ports
        -----------------------------------------------------------------------
        --! Rmii Clock ports (optional)
        oRmii_clk                   : out   std_logic_vector(gPhyPortCount-1 downto 0);
        --! Rmii Rx data valid ports
        iRmii_rxDataValid           : in    std_logic_vector(gPhyPortCount-1 downto 0);
        --! Rmii Rx data ports
        iRmii_rxData                : in    std_logic_vector(gPhyPortCount*2-1 downto 0);
        --! Rmii Rx error ports
        iRmii_rxError               : in    std_logic_vector(gPhyPortCount-1 downto 0);
        --! Rmii Tx enable ports
        oRmii_txEnable              : out   std_logic_vector(gPhyPortCount-1 downto 0);
        --! Rmii Tx data ports
        oRmii_txData                : out   std_logic_vector(gPhyPortCount*2-1 downto 0);

        -----------------------------------------------------------------------
        -- Mii Phy ports
        -----------------------------------------------------------------------
        --! Mii Rx data valid ports
        iMii_rxDataValid            : in    std_logic_vector(gPhyPortCount-1 downto 0);
        --! Mii Rx data ports
        iMii_rxData                 : in    std_logic_vector(gPhyPortCount*4-1 downto 0);
        --! Mii Rx error ports
        iMii_rxError                : in    std_logic_vector(gPhyPortCount-1 downto 0);
        --! Mii Rx Clocks
        iMii_rxClk                  : in    std_logic_vector(gPhyPortCount-1 downto 0);
        --! Mii Tx enable ports
        oMii_txEnable               : out   std_logic_vector(gPhyPortCount-1 downto 0);
        --! Mii Tx data ports
        oMii_txData                 : out   std_logic_vector(gPhyPortCount*4-1 downto 0);
        --! Mii Tx Clocks
        iMii_txClk                  : in    std_logic_vector(gPhyPortCount-1 downto 0);
        -----------------------------------------------------------------------
        -- Phy management interface
        -----------------------------------------------------------------------
        --! Phy reset (low-active)
        oSmi_nPhyRst                : out   std_logic_vector(gSmiPortCount-1 downto 0);
        --! SMI clock
        oSmi_clk                    : out   std_logic_vector(gSmiPortCount-1 downto 0);
        --! SMI data I/O input
        iSmi_dio                    : in    std_logic_vector(gSmiPortCount-1 downto 0);
        --! SMI data I/O output
        oSmi_dio                    : out   std_logic_vector(gSmiPortCount-1 downto 0);
        --! SMI data I/O tristate
        oSmi_dio_tri                : out   std_logic;
        -----------------------------------------------------------------------
        -- Other ports
        -----------------------------------------------------------------------
        --! Packet activity (enabled with gEnableActivity)
        oPktActivity                : out   std_logic;
        --! MAC TIMER outputs
        oMacTimerOut                : out   std_logic_vector(gTimerCount-1 downto 0)
    );
end axi_openmac;

architecture rtl of axi_openmac is
    --! Address zero padding vector
    constant cZeroPadAddress    : std_logic_vector(31 downto 0) := (others => cInactivated);

    --! Address array for MAC REG IPIF
    constant cMacReg_addressArray   : SLV64_ARRAY_TYPE := (
        (cZeroPadAddress & C_S_AXI_MAC_REG_RNG0_BASEADDR),
        (cZeroPadAddress & C_S_AXI_MAC_REG_RNG0_HIGHADDR),
        (cZeroPadAddress & C_S_AXI_MAC_REG_RNG1_BASEADDR),
        (cZeroPadAddress & C_S_AXI_MAC_REG_RNG1_HIGHADDR)
    );
    --! Address array for PKT BUF IPIF
    constant cPktBuf_addressArray   : SLV64_ARRAY_TYPE := (
        (cZeroPadAddress & C_S_AXI_MAC_PKT_BASEADDR),
        (cZeroPadAddress & C_S_AXI_MAC_PKT_HIGHADDR)
    );

    --! Chipselect for MAC REG --> MAC REG
    constant cMacReg_csMacReg   : natural := 1;
    --! Chipselect for MAC REG --> MAC TIMER
    constant cMacReg_csMacTimer : natural := 0;
    --! Chipselect for PKT BUF
    constant cPktBuf_cs         : natural := 0;

    --! Clock Reset type
    type tClkRst is record
        clk     : std_logic;
        rst     : std_logic;
        regClk  : std_logic;
        regRst  : std_logic;
        dmaClk  : std_logic;
        dmaRst  : std_logic;
        pktClk  : std_logic;
        pktRst  : std_logic;
        clk2x   : std_logic;
    end record;

    --! Mac Reg type
    type tMacReg is record
        chipselect  : std_logic;
        write       : std_logic;
        read        : std_logic;
        waitrequest : std_logic;
        byteenable  : std_logic_vector(cMacRegDataWidth/cByteLength-1 downto 0);
        address     : std_logic_vector(cMacRegAddrWidth-1 downto 0);
        writedata   : std_logic_vector(cMacRegDataWidth-1 downto 0);
        readdata    : std_logic_vector(cMacRegDataWidth-1 downto 0);
    end record;

    --! Mac Timer type
    type tMacTimer is record
        chipselect  : std_logic;
        write       : std_logic;
        read        : std_logic;
        waitrequest : std_logic;
        address     : std_logic_vector(cMacTimerAddrWidth-1 downto 0);
        writedata   : std_logic_vector(cMacTimerDataWidth-1 downto 0);
        readdata    : std_logic_vector(cMacTimerDataWidth-1 downto 0);
    end record;

    --! Pkt Buf type
    type tPktBuf is record
        chipselect  : std_logic;
        write       : std_logic;
        read        : std_logic;
        waitrequest : std_logic;
        byteenable  : std_logic_vector(cPktBufDataWidth/cByteLength-1 downto 0);
        address     : std_logic_vector(gPacketBufferLog2Size-1 downto 0);
        writedata   : std_logic_vector(cPktBufDataWidth-1 downto 0);
        readdata    : std_logic_vector(cPktBufDataWidth-1 downto 0);
    end record;

    --! Dma type
    type tDma is record
        write           : std_logic;
        read            : std_logic;
        waitrequest     : std_logic;
        readdatavalid   : std_logic;
        byteenable      : std_logic_vector(gDmaDataWidth/cByteLength-1 downto 0);
        address         : std_logic_vector(gDmaAddrWidth-1 downto 0);
        burstcount      : std_logic_vector(gDmaBurstCountWidth-1 downto 0);
        burstcounter    : std_logic_vector(gDmaBurstCountWidth-1 downto 0);
        writedata       : std_logic_vector(gDmaDataWidth-1 downto 0);
        readdata        : std_logic_vector(gDmaDataWidth-1 downto 0);
    end record;

    --! AXI lite slave for MAC REG
    type tAxiSlaveMacReg is record
        axi_aclk    : std_logic;
        axi_aresetn : std_logic;
        axi_awaddr  : std_logic_vector(C_S_AXI_MAC_REG_ADDR_WIDTH-1 downto 0);
        axi_awvalid : std_logic;
        axi_awready : std_logic;
        axi_wdata   : std_logic_vector(C_S_AXI_MAC_REG_DATA_WIDTH-1 downto 0);
        axi_wstrb   : std_logic_vector(C_S_AXI_MAC_REG_DATA_WIDTH/8-1 downto 0);
        axi_wvalid  : std_logic;
        axi_wready  : std_logic;
        axi_bresp   : std_logic_vector(1 downto 0);
        axi_bvalid  : std_logic;
        axi_bready  : std_logic;
        axi_araddr  : std_logic_vector(C_S_AXI_MAC_REG_ADDR_WIDTH-1 downto 0);
        axi_arvalid : std_logic;
        axi_arready : std_logic;
        axi_rdata   : std_logic_vector(C_S_AXI_MAC_REG_DATA_WIDTH-1 downto 0);
        axi_rresp   : std_logic_vector(1 downto 0);
        axi_rvalid  : std_logic;
        axi_rready  : std_logic;
        ipif_clk    : std_logic;
        ipif_resetn : std_logic;
        ipif_addr   : std_logic_vector(C_S_AXI_MAC_REG_ADDR_WIDTH-1 downto 0);
        ipif_rnw    : std_logic;
        ipif_be     : std_logic_vector(C_S_AXI_MAC_REG_DATA_WIDTH/8-1 downto 0);
        ipif_cs     : std_logic_vector(((cMacReg_addressArray'length)/2-1) downto 0);
        ipif_wrdata : std_logic_vector(C_S_AXI_MAC_REG_DATA_WIDTH-1 downto 0);
        ipif_rddata : std_logic_vector(C_S_AXI_MAC_REG_DATA_WIDTH-1 downto 0);
        ipif_wrack  : std_logic;
        ipif_rdack  : std_logic;
        ipif_error  : std_logic;
    end record;

    --! AXI lite slave for PKT BUF
    type tAxiSlavePktBuf is record
        axi_aclk    : std_logic;
        axi_aresetn : std_logic;
        axi_awaddr  : std_logic_vector(C_S_AXI_MAC_PKT_ADDR_WIDTH-1 downto 0);
        axi_awvalid : std_logic;
        axi_awready : std_logic;
        axi_wdata   : std_logic_vector(C_S_AXI_MAC_PKT_DATA_WIDTH-1 downto 0);
        axi_wstrb   : std_logic_vector(C_S_AXI_MAC_PKT_DATA_WIDTH/8-1 downto 0);
        axi_wvalid  : std_logic;
        axi_wready  : std_logic;
        axi_bresp   : std_logic_vector(1 downto 0);
        axi_bvalid  : std_logic;
        axi_bready  : std_logic;
        axi_araddr  : std_logic_vector(C_S_AXI_MAC_PKT_ADDR_WIDTH-1 downto 0);
        axi_arvalid : std_logic;
        axi_arready : std_logic;
        axi_rdata   : std_logic_vector(C_S_AXI_MAC_PKT_DATA_WIDTH-1 downto 0);
        axi_rresp   : std_logic_vector(1 downto 0);
        axi_rvalid  : std_logic;
        axi_rready  : std_logic;
        ipif_clk    : std_logic;
        ipif_resetn : std_logic;
        ipif_addr   : std_logic_vector(C_S_AXI_MAC_PKT_ADDR_WIDTH-1 downto 0);
        ipif_rnw    : std_logic;
        ipif_be     : std_logic_vector(C_S_AXI_MAC_PKT_DATA_WIDTH/8-1 downto 0);
        ipif_cs     : std_logic_vector(((cPktBuf_addressArray'length)/2-1) downto 0);
        ipif_wrdata : std_logic_vector(C_S_AXI_MAC_PKT_DATA_WIDTH-1 downto 0);
        ipif_rddata : std_logic_vector(C_S_AXI_MAC_PKT_DATA_WIDTH-1 downto 0);
        ipif_wrack  : std_logic;
        ipif_rdack  : std_logic;
        ipif_error  : std_logic;
    end record;

    --! AXI master for DMA
    type tAxiMasterDma is record
        axi_aclk                : std_logic;
        axi_aresetn             : std_logic;
        md_error                : std_logic;
        axi_arready             : std_logic;
        axi_arvalid             : std_logic;
        axi_araddr              : std_logic_vector(C_M_AXI_MAC_DMA_ADDR_WIDTH-1 downto 0);
        axi_arlen               : std_logic_vector(7 downto 0);
        axi_arsize              : std_logic_vector(2 downto 0);
        axi_arburst             : std_logic_vector(1 downto 0);
        axi_arprot              : std_logic_vector(2 downto 0);
        axi_arcache             : std_logic_vector(3 downto 0);
        axi_rready              : std_logic;
        axi_rvalid              : std_logic;
        axi_rdata               : std_logic_vector(C_M_AXI_MAC_DMA_DATA_WIDTH-1 downto 0);
        axi_rresp               : std_logic_vector(1 downto 0);
        axi_rlast               : std_logic;
        axi_awready             : std_logic;
        axi_awvalid             : std_logic;
        axi_awaddr              : std_logic_vector(C_M_AXI_MAC_DMA_ADDR_WIDTH-1 downto 0);
        axi_awlen               : std_logic_vector(7 downto 0);
        axi_awsize              : std_logic_vector(2 downto 0);
        axi_awburst             : std_logic_vector(1 downto 0);
        axi_awprot              : std_logic_vector(2 downto 0);
        axi_awcache             : std_logic_vector(3 downto 0);
        axi_wready              : std_logic;
        axi_wvalid              : std_logic;
        axi_wdata               : std_logic_vector(C_M_AXI_MAC_DMA_DATA_WIDTH-1 downto 0);
        axi_wstrb               : std_logic_vector(C_M_AXI_MAC_DMA_DATA_WIDTH/8-1 downto 0);
        axi_wlast               : std_logic;
        axi_bready              : std_logic;
        axi_bvalid              : std_logic;
        axi_bresp               : std_logic_vector(1 downto 0);
        ipif_mstrd_req          : std_logic;
        ipif_mstwr_req          : std_logic;
        ipif_mst_addr           : std_logic_vector(C_M_AXI_MAC_DMA_ADDR_WIDTH-1 downto 0);
        ipif_mst_length         : std_logic_vector(C_M_AXI_MAC_DMA_LENGTH_WIDTH-1 downto 0);
        ipif_mst_be             : std_logic_vector(C_M_AXI_MAC_DMA_NATIVE_DWIDTH/8-1 downto 0);
        ipif_mst_type           : std_logic;
        ipif_mst_lock           : std_logic;
        ipif_mst_reset          : std_logic;
        ipif_mst_cmdack         : std_logic;
        ipif_mst_cmplt          : std_logic;
        ipif_mst_error          : std_logic;
        ipif_mst_rearbitrate    : std_logic;
        ipif_mst_cmd_timeout    : std_logic;
        ipif_mstrd_d            : std_logic_vector(C_M_AXI_MAC_DMA_NATIVE_DWIDTH-1 downto 0 );
        ipif_mstrd_rem          : std_logic_vector(C_M_AXI_MAC_DMA_NATIVE_DWIDTH/8-1 downto 0);
        ipif_mstrd_sof_n        : std_logic;
        ipif_mstrd_eof_n        : std_logic;
        ipif_mstrd_src_rdy_n    : std_logic;
        ipif_mstrd_src_dsc_n    : std_logic;
        ipif_mstrd_dst_rdy_n    : std_logic;
        ipif_mstrd_dst_dsc_n    : std_logic;
        ipif_mstwr_d            : std_logic_vector(C_M_AXI_MAC_DMA_NATIVE_DWIDTH-1 downto 0);
        ipif_mstwr_rem          : std_logic_vector(C_M_AXI_MAC_DMA_NATIVE_DWIDTH/8-1 downto 0);
        ipif_mstwr_sof_n        : std_logic;
        ipif_mstwr_eof_n        : std_logic;
        ipif_mstwr_src_rdy_n    : std_logic;
        ipif_mstwr_src_dsc_n    : std_logic;
        ipif_mstwr_dst_rdy_n    : std_logic;
        ipif_mstwr_dst_dsc_n    : std_logic;
    end record;

    --! Clock xing for MAC REG port
    type tClkXingMacRegPort is record
        clk         : std_logic;
        cs          : std_logic_vector(((cMacReg_addressArray'length)/2-1) downto 0);
        rnw         : std_logic;
        readdata    : std_logic_vector(C_S_AXI_MAC_REG_DATA_WIDTH-1 downto 0);
        wrAck       : std_logic;
        rdAck       : std_logic;
    end record;

    --! Clock xing for MAC REG
    type tClkXingMacReg is record
        rst     : std_logic;
        fast    : tClkXingMacRegPort;
        slow    : tClkXingMacRegPort;
    end record;

    --! Data width converter for MAC REG
    type tConvMacReg is record
        rst                 : std_logic;
        clk                 : std_logic;
        master_select       : std_logic;
        master_write        : std_logic;
        master_read         : std_logic;
        master_byteenable   : std_logic_vector(3 downto 0);
        master_writedata    : std_logic_vector(31 downto 0);
        master_readdata     : std_logic_vector(31 downto 0);
        master_address      : std_logic_vector(C_S_AXI_MAC_REG_ADDR_WIDTH-1 downto 0);
        master_WriteAck     : std_logic;
        master_ReadAck      : std_logic;
        slave_select        : std_logic;
        slave_write         : std_logic;
        slave_read          : std_logic;
        slave_address       : std_logic_vector(C_S_AXI_MAC_REG_ADDR_WIDTH-1 downto 0);
        slave_byteenable    : std_logic_vector(1 downto 0);
        slave_readdata      : std_logic_vector(15 downto 0);
        slave_writedata     : std_logic_vector(15 downto 0);
        slave_ack           : std_logic;
    end record;

    --! IPIF handler for MAC DMA
    type tIpifMasterHandler is record
        rst                    : std_logic;
        clk                    : std_logic;
        ipif_cmdAck            : std_logic;
        ipif_cmplt             : std_logic;
        ipif_error             : std_logic;
        ipif_rearbitrate       : std_logic;
        ipif_cmdTimeout        : std_logic;
        ipif_type              : std_logic;
        ipif_addr              : std_logic_vector(C_M_AXI_MAC_DMA_ADDR_WIDTH-1 downto 0);
        ipif_length            : std_logic_vector(C_M_AXI_MAC_DMA_LENGTH_WIDTH-1 downto 0);
        ipif_be                : std_logic_vector(3 downto 0);
        ipif_lock              : std_logic;
        ipif_reset             : std_logic;
        ipif_rdData            : std_logic_vector(31 downto 0);
        ipif_rdRem             : std_logic_vector(3 downto 0);
        ipif_rdReq             : std_logic;
        nIpif_rdSof            : std_logic;
        nIpif_rdEof            : std_logic;
        nIpif_rdSrcRdy         : std_logic;
        nIpif_rdSrcDsc         : std_logic;
        nIpif_rdDstRdy         : std_logic;
        nIpif_rdDstDsc         : std_logic;
        ipif_wrData            : std_logic_vector(31 downto 0);
        ipif_wrRem             : std_logic_vector(3 downto 0);
        ipif_wrReq             : std_logic;
        nIpif_wrSof            : std_logic;
        nIpif_wrEof            : std_logic;
        nIpif_wrSrcRdy         : std_logic;
        nIpif_wrSrcDsc         : std_logic;
        nIpif_wrDstRdy         : std_logic;
        nIpif_wrDstDsc         : std_logic;
        masterRead             : std_logic;
        masterWrite            : std_logic;
        masterAddress          : std_logic_vector(gDmaAddrWidth-1 downto 0);
        masterWritedata        : std_logic_vector(31 downto 0);
        masterBurstcount       : std_logic_vector(gDmaBurstCountWidth-1 downto 0);
        masterBurstcounter     : std_logic_vector(gDmaBurstCountWidth-1 downto 0);
        masterReaddata         : std_logic_vector(31 downto 0);
        masterWaitrequest      : std_logic;
        masterReaddatavalid    : std_logic;
    end record;

    --! Clock and resets
    signal intf_clkRst          : tClkRst;
    --! Mac Reg
    signal intf_macReg          : tMacReg;
    --! Mac Timer
    signal intf_macTimer        : tMacTimer;
    --! Packet buffer
    signal intf_pktBuf          : tPktBuf;
    --! Dma
    signal intf_dma             : tDma;
    --! Mac Reg IPIF
    signal ipif_macReg          : tAxiSlaveMacReg;
    --! Packet buffer IPIF
    signal ipif_pktBuf          : tAxiSlavePktBuf;
    --! Dma IPIF
    signal ipif_dma             : tAxiMasterDma;
    --! Clock Xing for MAC REG IPIF
    signal xing_macReg          : tClkXingMacReg;
    --! Dara width converter for MAC REG IPIF
    signal conv_macReg          : tConvMacReg;
    --! Dma IPIF master handler
    signal ipif_dmaMasterHdler  : tIpifMasterHandler;

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
    MAC_IRQ <= macTx_interrupt or macRx_interrupt;

    -- Phy Tx path
    rmiiPathArrayToStdLogicVector(
        iVector => rmiiTx,
        oEnable => oRmii_txEnable,
        oData   => oRmii_txData
    );

    miiPathArrayToStdLogicVector(
        iVector => miiTx,
        oEnable => oMii_txEnable,
        oData   => oMii_txData
    );

    ---------------------------------------------------------------------------
    -- Map inputs
    ---------------------------------------------------------------------------
    -- Clock and resets
    intf_clkRst.clk     <= iClk50;
    intf_clkRst.clk2x   <= iClk100;
    intf_clkRst.regClk  <= S_AXI_MAC_REG_ACLK;
    intf_clkRst.pktClk  <= S_AXI_MAC_PKT_ACLK;
    intf_clkRst.dmaClk  <= M_AXI_MAC_DMA_ACLK;
    intf_clkRst.rst     <= not S_AXI_MAC_REG_ARESETN;
    intf_clkRst.regRst  <= not S_AXI_MAC_REG_ARESETN;
    intf_clkRst.pktRst  <= not S_AXI_MAC_PKT_ARESETN;
    intf_clkRst.dmaRst  <= not M_AXI_MAC_DMA_ARESETN;

    -- Phy Rx path
    stdLogicVectorToRmiiPathArray(
        iEnable => iRmii_rxDataValid,
        iData   => iRmii_rxData,
        oVector => rmiiRx
    );

    stdLogicVectorToMiiPathArray(
        iEnable => iMii_rxDataValid,
        iData   => iMii_rxData,
        oVector => miiRx
    );

    ---------------------------------------------------------------------------
    -- Map IOs
    ---------------------------------------------------------------------------
    -- Assign SMI IO (the tristate buffer shall be assigned by toplevel)
    oSmi_dio        <= smi_data_out;
    oSmi_dio_tri    <= not smi_data_outEnable;
    smi_data_in     <= iSmi_dio;

    ---------------------------------------------------------------------------
    -- Map Instances
    ---------------------------------------------------------------------------
    -- MAC REG --> ipif_macReg
    ipif_macReg.axi_aclk        <= intf_clkRst.regClk;
    ipif_macReg.axi_aresetn     <= not intf_clkRst.regRst;
    ipif_macReg.axi_awaddr      <= S_AXI_MAC_REG_AWADDR;
    ipif_macReg.axi_awvalid     <= S_AXI_MAC_REG_AWVALID;
    S_AXI_MAC_REG_AWREADY       <= ipif_macReg.axi_awready;
    ipif_macReg.axi_wdata       <= S_AXI_MAC_REG_WDATA;
    ipif_macReg.axi_wstrb       <= S_AXI_MAC_REG_WSTRB;
    ipif_macReg.axi_wvalid      <= S_AXI_MAC_REG_WVALID;
    S_AXI_MAC_REG_WREADY        <= ipif_macReg.axi_wready;
    S_AXI_MAC_REG_BRESP         <= ipif_macReg.axi_bresp;
    S_AXI_MAC_REG_BVALID        <= ipif_macReg.axi_bvalid;
    ipif_macReg.axi_bready      <= S_AXI_MAC_REG_BREADY;
    ipif_macReg.axi_araddr      <= S_AXI_MAC_REG_ARADDR;
    ipif_macReg.axi_arvalid     <= S_AXI_MAC_REG_ARVALID;
    S_AXI_MAC_REG_ARREADY       <= ipif_macReg.axi_arready;
    S_AXI_MAC_REG_RDATA         <= ipif_macReg.axi_rdata;
    S_AXI_MAC_REG_RRESP         <= ipif_macReg.axi_rresp;
    S_AXI_MAC_REG_RVALID        <= ipif_macReg.axi_rvalid;
    ipif_macReg.axi_rready      <= S_AXI_MAC_REG_RREADY;

    -- xing_macReg <-- conv_macReg or intf_macTimer
    --! This process assigns the read and ack path from macReg and macTimer
    --! to the clock crossing slow inputs, depending on the selected target.
    ASSIGN_XING_MACREG : process (conv_macReg, intf_macTimer)
    begin
        -- default is MAC REG source
        xing_macReg.slow.readdata   <= conv_macReg.master_readdata;
        xing_macReg.slow.wrAck      <= conv_macReg.master_WriteAck;
        xing_macReg.slow.rdAck      <= conv_macReg.master_ReadAck;

        if intf_macTimer.chipselect = cActivated then
            xing_macReg.slow.readdata   <= intf_macTimer.readdata;
            xing_macReg.slow.wrAck      <= intf_macTimer.write and not intf_macTimer.waitrequest;
            xing_macReg.slow.rdAck      <= intf_macTimer.read and not intf_macTimer.waitrequest;
        end if;
    end process ASSIGN_XING_MACREG;

    -- ipif_macReg --> xing_macReg
    --unused output: ipif_macReg.ipif_resetn;
    xing_macReg.rst             <= intf_clkRst.regRst;
    xing_macReg.fast.clk        <= ipif_macReg.ipif_clk;
    xing_macReg.slow.clk        <= intf_clkRst.clk;
    xing_macReg.fast.rnw        <= ipif_macReg.ipif_rnw;
    xing_macReg.fast.cs         <= ipif_macReg.ipif_cs;
    ipif_macReg.ipif_rddata     <= xing_macReg.fast.readdata;
    ipif_macReg.ipif_wrack      <= xing_macReg.fast.wrAck;
    ipif_macReg.ipif_rdack      <= xing_macReg.fast.rdAck;
    ipif_macReg.ipif_error      <= cInactivated; --unused

    -- ipif_macReg --> conv_macReg | xing_macReg --> conv_macReg
    conv_macReg.rst                 <= intf_clkRst.rst;
    conv_macReg.clk                 <= intf_clkRst.clk;
    conv_macReg.master_select       <= xing_macReg.slow.cs(cMacReg_csMacReg);
    conv_macReg.master_write        <= not xing_macReg.slow.rnw;
    conv_macReg.master_read         <= xing_macReg.slow.rnw;
    conv_macReg.master_byteenable   <= ipif_macReg.ipif_be;
    conv_macReg.master_writedata    <= ipif_macReg.ipif_wrdata;
    conv_macReg.master_address      <= ipif_macReg.ipif_addr(conv_macReg.master_address'range);

    -- conv_macReg --> intf_macReg
    intf_macReg.chipselect          <= conv_macReg.slave_select;
    intf_macReg.write               <= conv_macReg.slave_write;
    intf_macReg.read                <= conv_macReg.slave_read;
    intf_macReg.address             <= conv_macReg.slave_address(intf_macReg.address'range);
    intf_macReg.byteenable          <= conv_macReg.slave_byteenable;
    conv_macReg.slave_readdata      <= intf_macReg.readdata;
    intf_macReg.writedata           <= conv_macReg.slave_writedata;
    conv_macReg.slave_ack           <= not intf_macReg.waitrequest;

    -- ipif_macReg --> intf_macTimer | xing_macReg --> intf_macTimer
    intf_macTimer.chipselect    <= xing_macReg.slow.cs(cMacReg_csMacTimer);
    intf_macTimer.write         <= not xing_macReg.slow.rnw;
    intf_macTimer.read          <= xing_macReg.slow.rnw;
    intf_macTimer.address       <= ipif_macReg.ipif_addr(intf_macTimer.address'range);
    intf_macTimer.writedata     <= ipif_macReg.ipif_wrdata;

    -- MAC PKT --> ipif_pktBuf
    ipif_pktBuf.axi_aclk        <= intf_clkRst.pktClk;
    ipif_pktBuf.axi_aresetn     <= not intf_clkRst.pktRst;
    ipif_pktBuf.axi_awaddr      <= S_AXI_MAC_PKT_AWADDR;
    ipif_pktBuf.axi_awvalid     <= S_AXI_MAC_PKT_AWVALID;
    S_AXI_MAC_PKT_AWREADY       <= ipif_pktBuf.axi_awready;
    ipif_pktBuf.axi_wdata       <= S_AXI_MAC_PKT_WDATA;
    ipif_pktBuf.axi_wstrb       <= S_AXI_MAC_PKT_WSTRB;
    ipif_pktBuf.axi_wvalid      <= S_AXI_MAC_PKT_WVALID;
    S_AXI_MAC_PKT_WREADY        <= ipif_pktBuf.axi_wready;
    S_AXI_MAC_PKT_BRESP         <= ipif_pktBuf.axi_bresp;
    S_AXI_MAC_PKT_BVALID        <= ipif_pktBuf.axi_bvalid;
    ipif_pktBuf.axi_bready      <= S_AXI_MAC_PKT_BREADY;
    ipif_pktBuf.axi_araddr      <= S_AXI_MAC_PKT_ARADDR;
    ipif_pktBuf.axi_arvalid     <= S_AXI_MAC_PKT_ARVALID;
    S_AXI_MAC_PKT_ARREADY       <= ipif_pktBuf.axi_arready;
    S_AXI_MAC_PKT_RDATA         <= ipif_pktBuf.axi_rdata;
    S_AXI_MAC_PKT_RRESP         <= ipif_pktBuf.axi_rresp;
    S_AXI_MAC_PKT_RVALID        <= ipif_pktBuf.axi_rvalid;
    ipif_pktBuf.axi_rready      <= S_AXI_MAC_PKT_RREADY;

    -- ipif_pktBuf --> intf_pktBuf
    --unused output: ipif_pktBuf.ipif_clk
    --unused output: ipif_pktBuf.ipif_resetn
    intf_pktBuf.address     <= ipif_pktBuf.ipif_addr(intf_pktBuf.address'range);
    intf_pktBuf.write       <= not ipif_pktBuf.ipif_rnw;
    intf_pktBuf.read        <= ipif_pktBuf.ipif_rnw;
    intf_pktBuf.byteenable  <= ipif_pktBuf.ipif_be;
    intf_pktBuf.chipselect  <= ipif_pktBuf.ipif_cs(cPktBuf_cs);
    intf_pktBuf.writedata   <= ipif_pktBuf.ipif_wrdata;
    ipif_pktBuf.ipif_rddata <= intf_pktBuf.readdata;
    ipif_pktBuf.ipif_wrack  <= intf_pktBuf.chipselect and intf_pktBuf.write and not intf_pktBuf.waitrequest;
    ipif_pktBuf.ipif_rdack  <= intf_pktBuf.chipselect and intf_pktBuf.read and not intf_pktBuf.waitrequest;
    ipif_pktBuf.ipif_error  <= cInactivated; --unused

    -- MAC DMA --> ipif_dma
    ipif_dma.axi_aclk           <= intf_clkRst.dmaClk;
    ipif_dma.axi_aresetn        <= not intf_clkRst.dmaRst;
    M_AXI_MAC_DMA_MD_ERROR      <= ipif_dma.md_error;
    ipif_dma.axi_arready        <= M_AXI_MAC_DMA_ARREADY;
    M_AXI_MAC_DMA_ARVALID       <= ipif_dma.axi_arvalid;
    M_AXI_MAC_DMA_ARADDR        <= ipif_dma.axi_araddr;
    M_AXI_MAC_DMA_ARLEN         <= ipif_dma.axi_arlen;
    M_AXI_MAC_DMA_ARSIZE        <= ipif_dma.axi_arsize;
    M_AXI_MAC_DMA_ARBURST       <= ipif_dma.axi_arburst;
    M_AXI_MAC_DMA_ARPROT        <= ipif_dma.axi_arprot;
    M_AXI_MAC_DMA_ARCACHE       <= ipif_dma.axi_arcache;
    M_AXI_MAC_DMA_RREADY        <= ipif_dma.axi_rready;
    ipif_dma.axi_rvalid         <= M_AXI_MAC_DMA_RVALID;
    ipif_dma.axi_rdata          <= M_AXI_MAC_DMA_RDATA;
    ipif_dma.axi_rresp          <= M_AXI_MAC_DMA_RRESP;
    ipif_dma.axi_rlast          <= M_AXI_MAC_DMA_RLAST;
    ipif_dma.axi_awready        <= M_AXI_MAC_DMA_AWREADY;
    M_AXI_MAC_DMA_AWVALID       <= ipif_dma.axi_awvalid;
    M_AXI_MAC_DMA_AWADDR        <= ipif_dma.axi_awaddr;
    M_AXI_MAC_DMA_AWLEN         <= ipif_dma.axi_awlen;
    M_AXI_MAC_DMA_AWSIZE        <= ipif_dma.axi_awsize;
    M_AXI_MAC_DMA_AWBURST       <= ipif_dma.axi_awburst;
    M_AXI_MAC_DMA_AWPROT        <= ipif_dma.axi_awprot;
    M_AXI_MAC_DMA_AWCACHE       <= ipif_dma.axi_awcache;
    ipif_dma.axi_wready         <= M_AXI_MAC_DMA_WREADY;
    M_AXI_MAC_DMA_WVALID        <= ipif_dma.axi_wvalid;
    M_AXI_MAC_DMA_WDATA         <= ipif_dma.axi_wdata;
    M_AXI_MAC_DMA_WSTRB         <= ipif_dma.axi_wstrb;
    M_AXI_MAC_DMA_WLAST         <= ipif_dma.axi_wlast;
    M_AXI_MAC_DMA_BREADY        <= ipif_dma.axi_bready;
    ipif_dma.axi_bvalid         <= M_AXI_MAC_DMA_BVALID;
    ipif_dma.axi_bresp          <= M_AXI_MAC_DMA_BRESP;

    -- ipif_dma --> ipif_dmaMasterHdler
    ipif_dmaMasterHdler.rst                 <= intf_clkRst.dmaRst;
    ipif_dmaMasterHdler.clk                 <= intf_clkRst.dmaClk;
    ipif_dma.ipif_mstrd_req                 <= ipif_dmaMasterHdler.ipif_rdReq;
    ipif_dma.ipif_mstwr_req                 <= ipif_dmaMasterHdler.ipif_wrReq;
    ipif_dma.ipif_mst_addr                  <= ipif_dmaMasterHdler.ipif_addr(ipif_dma.ipif_mst_addr'range);
    ipif_dma.ipif_mst_length                <= ipif_dmaMasterHdler.ipif_length;
    ipif_dma.ipif_mst_be                    <= ipif_dmaMasterHdler.ipif_be;
    ipif_dma.ipif_mst_type                  <= ipif_dmaMasterHdler.ipif_type;
    ipif_dma.ipif_mst_lock                  <= ipif_dmaMasterHdler.ipif_lock;
    ipif_dma.ipif_mst_reset                 <= ipif_dmaMasterHdler.ipif_reset;
    ipif_dmaMasterHdler.ipif_cmdAck         <= ipif_dma.ipif_mst_cmdack;
    ipif_dmaMasterHdler.ipif_cmplt          <= ipif_dma.ipif_mst_cmplt;
    ipif_dmaMasterHdler.ipif_error          <= ipif_dma.ipif_mst_error;
    ipif_dmaMasterHdler.ipif_rearbitrate    <= ipif_dma.ipif_mst_rearbitrate;
    ipif_dmaMasterHdler.ipif_cmdTimeout     <= ipif_dma.ipif_mst_cmd_timeout;
    ipif_dmaMasterHdler.ipif_rdData         <= ipif_dma.ipif_mstrd_d;
    ipif_dmaMasterHdler.ipif_rdRem          <= ipif_dma.ipif_mstrd_rem;
    ipif_dmaMasterHdler.nIpif_rdSof         <= ipif_dma.ipif_mstrd_sof_n;
    ipif_dmaMasterHdler.nIpif_rdEof         <= ipif_dma.ipif_mstrd_eof_n;
    ipif_dmaMasterHdler.nIpif_rdSrcRdy      <= ipif_dma.ipif_mstrd_src_rdy_n;
    ipif_dmaMasterHdler.nIpif_rdSrcDsc      <= ipif_dma.ipif_mstrd_src_dsc_n;
    ipif_dma.ipif_mstrd_dst_rdy_n           <= ipif_dmaMasterHdler.nIpif_rdDstRdy;
    ipif_dma.ipif_mstrd_dst_dsc_n           <= ipif_dmaMasterHdler.nIpif_rdDstDsc;
    ipif_dma.ipif_mstwr_d                   <= ipif_dmaMasterHdler.ipif_wrData;
    ipif_dma.ipif_mstwr_rem                 <= ipif_dmaMasterHdler.ipif_wrRem;
    ipif_dma.ipif_mstwr_sof_n               <= ipif_dmaMasterHdler.nIpif_wrSof;
    ipif_dma.ipif_mstwr_eof_n               <= ipif_dmaMasterHdler.nIpif_wrEof;
    ipif_dma.ipif_mstwr_src_rdy_n           <= ipif_dmaMasterHdler.nIpif_wrSrcRdy;
    ipif_dma.ipif_mstwr_src_dsc_n           <= ipif_dmaMasterHdler.nIpif_wrSrcDsc;
    ipif_dmaMasterHdler.nIpif_wrDstRdy      <= ipif_dma.ipif_mstwr_dst_rdy_n;
    ipif_dmaMasterHdler.nIpif_wrDstDsc      <= ipif_dma.ipif_mstwr_dst_dsc_n;

    -- ipif_dmaMasterHdler --> intf_dma
    ipif_dmaMasterHdler.masterRead          <= intf_dma.read;
    ipif_dmaMasterHdler.masterWrite         <= intf_dma.write;
    ipif_dmaMasterHdler.masterAddress       <= intf_dma.address;
    ipif_dmaMasterHdler.masterWritedata     <= intf_dma.writedata;
    ipif_dmaMasterHdler.masterBurstcount    <= intf_dma.burstcount;
    ipif_dmaMasterHdler.masterBurstcounter  <= intf_dma.burstcounter;
    intf_dma.readdata                       <= ipif_dmaMasterHdler.masterReaddata;
    intf_dma.waitrequest                    <= ipif_dmaMasterHdler.masterWaitrequest;
    intf_dma.readdatavalid                  <= ipif_dmaMasterHdler.masterReaddatavalid;

    ---------------------------------------------------------------------------
    -- Instantiations
    ---------------------------------------------------------------------------
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
            gTimerCount             => gTimerCount,
            gTimerEnablePulseWidth  => gTimerEnablePulseWidth,
            gTimerPulseRegWidth     => gTimerPulseRegWidth
        )
        port map (
            iClk                    => intf_clkRst.clk,
            iRst                    => intf_clkRst.rst,
            iDmaClk                 => intf_clkRst.dmaClk,
            iDmaRst                 => intf_clkRst.dmaRst,
            iPktBufClk              => intf_clkRst.pktClk,
            iPktBufRst              => intf_clkRst.pktRst,
            iClk2x                  => intf_clkRst.clk2x,
            iMacReg_chipselect      => intf_macReg.chipselect,
            iMacReg_write           => intf_macReg.write,
            iMacReg_read            => intf_macReg.read,
            oMacReg_waitrequest     => intf_macReg.waitrequest,
            iMacReg_byteenable      => intf_macReg.byteenable,
            iMacReg_address         => intf_macReg.address,
            iMacReg_writedata       => intf_macReg.writedata,
            oMacReg_readdata        => intf_macReg.readdata,
            iMacTimer_chipselect    => intf_macTimer.chipselect,
            iMacTimer_write         => intf_macTimer.write,
            iMacTimer_read          => intf_macTimer.read,
            oMacTimer_waitrequest   => intf_macTimer.waitrequest,
            iMacTimer_address       => intf_macTimer.address,
            iMacTimer_writedata     => intf_macTimer.writedata,
            oMacTimer_readdata      => intf_macTimer.readdata,
            iPktBuf_chipselect      => intf_pktBuf.chipselect,
            iPktBuf_write           => intf_pktBuf.write,
            iPktBuf_read            => intf_pktBuf.read,
            oPktBuf_waitrequest     => intf_pktBuf.waitrequest,
            iPktBuf_byteenable      => intf_pktBuf.byteenable,
            iPktBuf_address         => intf_pktBuf.address,
            iPktBuf_writedata       => intf_pktBuf.writedata,
            oPktBuf_readdata        => intf_pktBuf.readdata,
            oDma_write              => intf_dma.write,
            oDma_read               => intf_dma.read,
            iDma_waitrequest        => intf_dma.waitrequest,
            iDma_readdatavalid      => intf_dma.readdatavalid,
            oDma_byteenable         => intf_dma.byteenable,
            oDma_address            => intf_dma.address,
            oDma_burstcount         => intf_dma.burstcount,
            oDma_burstcounter       => intf_dma.burstcounter,
            oDma_writedata          => intf_dma.writedata,
            iDma_readdata           => intf_dma.readdata,
            oMacTimer_interrupt     => TIMER_IRQ,
            oMacTx_interrupt        => macTx_interrupt,
            oMacRx_interrupt        => macRx_interrupt,
            iRmii_Rx                => rmiiRx,
            iRmii_RxError           => iRmii_rxError,
            oRmii_Tx                => rmiiTx,
            iMii_Rx                 => miiRx,
            iMii_RxError            => iMii_rxError,
            iMii_RxClk              => iMii_rxClk,
            oMii_Tx                 => miiTx,
            iMii_TxClk              => iMii_txClk,
            onPhy_reset             => oSmi_nPhyRst,
            oSmi_clk                => oSmi_clk,
            oSmi_data_outEnable     => smi_data_outEnable,
            oSmi_data_out           => smi_data_out,
            iSmi_data_in            => smi_data_in,
            oActivity               => oPktActivity,
            oMacTimer               => oMacTimerOut
        );

    --! The MAC REG AXI lite IPIF converts the AXI interface to IPIF.
    THEMACREG_AXILITE : entity axi_lite_ipif_v1_01_a.axi_lite_ipif
        generic map (
            C_S_AXI_DATA_WIDTH      => C_S_AXI_MAC_REG_DATA_WIDTH,
            C_S_AXI_ADDR_WIDTH      => C_S_AXI_MAC_REG_ADDR_WIDTH,
            C_S_AXI_MIN_SIZE        => C_S_AXI_MAC_REG_MIN_SIZE,
            C_USE_WSTRB             => C_S_AXI_MAC_REG_USE_WSTRB,
            C_DPHASE_TIMEOUT        => C_S_AXI_MAC_REG_DPHASE_TIMEOUT,
            C_ARD_ADDR_RANGE_ARRAY  => cMacReg_addressArray,
            C_ARD_NUM_CE_ARRAY      => (1, 1),
            C_FAMILY                => C_FAMILY
        )
        port map (
            S_AXI_ACLK      => ipif_macReg.axi_aclk,
            S_AXI_ARESETN   => ipif_macReg.axi_aresetn,
            S_AXI_AWADDR    => ipif_macReg.axi_awaddr,
            S_AXI_AWVALID   => ipif_macReg.axi_awvalid,
            S_AXI_AWREADY   => ipif_macReg.axi_awready,
            S_AXI_WDATA     => ipif_macReg.axi_wdata,
            S_AXI_WSTRB     => ipif_macReg.axi_wstrb,
            S_AXI_WVALID    => ipif_macReg.axi_wvalid,
            S_AXI_WREADY    => ipif_macReg.axi_wready,
            S_AXI_BRESP     => ipif_macReg.axi_bresp,
            S_AXI_BVALID    => ipif_macReg.axi_bvalid,
            S_AXI_BREADY    => ipif_macReg.axi_bready,
            S_AXI_ARADDR    => ipif_macReg.axi_araddr,
            S_AXI_ARVALID   => ipif_macReg.axi_arvalid,
            S_AXI_ARREADY   => ipif_macReg.axi_arready,
            S_AXI_RDATA     => ipif_macReg.axi_rdata,
            S_AXI_RRESP     => ipif_macReg.axi_rresp,
            S_AXI_RVALID    => ipif_macReg.axi_rvalid,
            S_AXI_RREADY    => ipif_macReg.axi_rready,
            Bus2IP_Clk      => ipif_macReg.ipif_clk,
            Bus2IP_Resetn   => ipif_macReg.ipif_resetn,
            Bus2IP_Addr     => ipif_macReg.ipif_addr,
            Bus2IP_RNW      => ipif_macReg.ipif_rnw,
            Bus2IP_BE       => ipif_macReg.ipif_be,
            Bus2IP_CS       => ipif_macReg.ipif_cs,
            Bus2IP_RdCE     => open, --don't need that feature
            Bus2IP_WrCE     => open, --don't need that feature
            Bus2IP_Data     => ipif_macReg.ipif_wrdata,
            IP2Bus_Data     => ipif_macReg.ipif_rddata,
            IP2Bus_WrAck    => ipif_macReg.ipif_wrack,
            IP2Bus_RdAck    => ipif_macReg.ipif_rdack,
            IP2Bus_Error    => ipif_macReg.ipif_error
        );

    --! The clock Xing ipcore transfers the signals in the AXI clock domain to
    --! the iClk50 domain.
    THEMACREG_CLKXING : entity libcommon.clkXing
        generic map (
            gCsNum      => xing_macReg.fast.cs'length,
            gDataWidth  => xing_macReg.fast.readdata'length
        )
        port map (
            iArst           => xing_macReg.rst,
            iFastClk        => xing_macReg.fast.clk,
            iFastCs         => xing_macReg.fast.cs,
            iFastRNW        => xing_macReg.fast.rnw,
            oFastReaddata   => xing_macReg.fast.readdata,
            oFastWrAck      => xing_macReg.fast.wrAck,
            oFastRdAck      => xing_macReg.fast.rdAck,
            iSlowClk        => xing_macReg.slow.clk,
            oSlowCs         => xing_macReg.slow.cs,
            oSlowRNW        => xing_macReg.slow.rnw,
            iSlowReaddata   => xing_macReg.slow.readdata,
            iSlowWrAck      => xing_macReg.slow.wrAck,
            iSlowRdAck      => xing_macReg.slow.rdAck
        );

    --! The memory mapped slave converter changes from AXI's data width to 16 bit.
    THEMACREG_MMCONV : entity work.mmSlaveConv
        generic map (
            gEndian             => gEndianness,
            gMasterAddrWidth    => conv_macReg.master_address'length
        )
        port map (
            iRst                => conv_macReg.rst,
            iClk                => conv_macReg.clk,
            iMaster_select      => conv_macReg.master_select,
            iMaster_write       => conv_macReg.master_write,
            iMaster_read        => conv_macReg.master_read,
            iMaster_byteenable  => conv_macReg.master_byteenable,
            iMaster_writedata   => conv_macReg.master_writedata,
            oMaster_readdata    => conv_macReg.master_readdata,
            iMaster_address     => conv_macReg.master_address,
            oMaster_WriteAck    => conv_macReg.master_WriteAck,
            oMaster_ReadAck     => conv_macReg.master_ReadAck,
            oSlave_select       => conv_macReg.slave_select,
            oSlave_write        => conv_macReg.slave_write,
            oSlave_read         => conv_macReg.slave_read,
            oSlave_address      => conv_macReg.slave_address,
            oSlave_byteenable   => conv_macReg.slave_byteenable,
            iSlave_readdata     => conv_macReg.slave_readdata,
            oSlave_writedata    => conv_macReg.slave_writedata,
            iSlave_ack          => conv_macReg.slave_ack
        );

    --! Generate the packet buffer IPIF if any location is set to local.
    GEN_THEMACPKT : if gPacketBufferLocRx = cPktBufLocal or gPacketBufferLocTx = cPktBufLocal generate
        --! The MAC PKT BUF AXI lite IPIF converts the AXI interface to IPIF.
        THEMACREG_AXILITE : entity axi_lite_ipif_v1_01_a.axi_lite_ipif
            generic map (
                C_S_AXI_DATA_WIDTH      => C_S_AXI_MAC_PKT_DATA_WIDTH,
                C_S_AXI_ADDR_WIDTH      => C_S_AXI_MAC_PKT_ADDR_WIDTH,
                C_S_AXI_MIN_SIZE        => C_S_AXI_MAC_PKT_MIN_SIZE,
                C_USE_WSTRB             => C_S_AXI_MAC_PKT_USE_WSTRB,
                C_DPHASE_TIMEOUT        => C_S_AXI_MAC_PKT_DPHASE_TIMEOUT,
                C_ARD_ADDR_RANGE_ARRAY  => cPktBuf_addressArray,
                C_ARD_NUM_CE_ARRAY      => (0 => 1),
                C_FAMILY                => C_FAMILY
            )
            port map (
                S_AXI_ACLK      => ipif_pktBuf.axi_aclk,
                S_AXI_ARESETN   => ipif_pktBuf.axi_aresetn,
                S_AXI_AWADDR    => ipif_pktBuf.axi_awaddr,
                S_AXI_AWVALID   => ipif_pktBuf.axi_awvalid,
                S_AXI_AWREADY   => ipif_pktBuf.axi_awready,
                S_AXI_WDATA     => ipif_pktBuf.axi_wdata,
                S_AXI_WSTRB     => ipif_pktBuf.axi_wstrb,
                S_AXI_WVALID    => ipif_pktBuf.axi_wvalid,
                S_AXI_WREADY    => ipif_pktBuf.axi_wready,
                S_AXI_BRESP     => ipif_pktBuf.axi_bresp,
                S_AXI_BVALID    => ipif_pktBuf.axi_bvalid,
                S_AXI_BREADY    => ipif_pktBuf.axi_bready,
                S_AXI_ARADDR    => ipif_pktBuf.axi_araddr,
                S_AXI_ARVALID   => ipif_pktBuf.axi_arvalid,
                S_AXI_ARREADY   => ipif_pktBuf.axi_arready,
                S_AXI_RDATA     => ipif_pktBuf.axi_rdata,
                S_AXI_RRESP     => ipif_pktBuf.axi_rresp,
                S_AXI_RVALID    => ipif_pktBuf.axi_rvalid,
                S_AXI_RREADY    => ipif_pktBuf.axi_rready,
                Bus2IP_Clk      => ipif_pktBuf.ipif_clk,
                Bus2IP_Resetn   => ipif_pktBuf.ipif_resetn,
                Bus2IP_Addr     => ipif_pktBuf.ipif_addr,
                Bus2IP_RNW      => ipif_pktBuf.ipif_rnw,
                Bus2IP_BE       => ipif_pktBuf.ipif_be,
                Bus2IP_CS       => ipif_pktBuf.ipif_cs,
                Bus2IP_RdCE     => open, --don't need that feature
                Bus2IP_WrCE     => open, --don't need that feature
                Bus2IP_Data     => ipif_pktBuf.ipif_wrdata,
                IP2Bus_Data     => ipif_pktBuf.ipif_rddata,
                IP2Bus_WrAck    => ipif_pktBuf.ipif_wrack,
                IP2Bus_RdAck    => ipif_pktBuf.ipif_rdack,
                IP2Bus_Error    => ipif_pktBuf.ipif_error
            );
    end generate GEN_THEMACPKT;

    GEN_THEMACDMA : if gPacketBufferLocRx = cPktBufExtern or gPacketBufferLocTx = cPktBufExtern generate
        --! The MAC DMA AXI master IPIF converts the AXI interface to IPIF.
        THEMACDMA_AXI : entity axi_master_burst_v1_00_a.axi_master_burst
            generic map (
                C_M_AXI_ADDR_WIDTH  => C_M_AXI_MAC_DMA_ADDR_WIDTH,
                C_M_AXI_DATA_WIDTH  => C_M_AXI_MAC_DMA_DATA_WIDTH,
                C_MAX_BURST_LEN     => C_M_AXI_MAC_DMA_MAX_BURST_LEN,
                C_ADDR_PIPE_DEPTH   => 1,
                C_NATIVE_DATA_WIDTH => C_M_AXI_MAC_DMA_NATIVE_DWIDTH,
                C_LENGTH_WIDTH      => C_M_AXI_MAC_DMA_LENGTH_WIDTH,
                C_FAMILY            => C_FAMILY
            )
            port map (
                m_axi_aclk              => ipif_dma.axi_aclk,
                m_axi_aresetn           => ipif_dma.axi_aresetn,
                md_error                => ipif_dma.md_error,
                m_axi_arready           => ipif_dma.axi_arready,
                m_axi_arvalid           => ipif_dma.axi_arvalid,
                m_axi_araddr            => ipif_dma.axi_araddr,
                m_axi_arlen             => ipif_dma.axi_arlen,
                m_axi_arsize            => ipif_dma.axi_arsize,
                m_axi_arburst           => ipif_dma.axi_arburst,
                m_axi_arprot            => ipif_dma.axi_arprot,
                m_axi_arcache           => ipif_dma.axi_arcache,
                m_axi_rready            => ipif_dma.axi_rready,
                m_axi_rvalid            => ipif_dma.axi_rvalid,
                m_axi_rdata             => ipif_dma.axi_rdata,
                m_axi_rresp             => ipif_dma.axi_rresp,
                m_axi_rlast             => ipif_dma.axi_rlast,
                m_axi_awready           => ipif_dma.axi_awready,
                m_axi_awvalid           => ipif_dma.axi_awvalid,
                m_axi_awaddr            => ipif_dma.axi_awaddr,
                m_axi_awlen             => ipif_dma.axi_awlen,
                m_axi_awsize            => ipif_dma.axi_awsize,
                m_axi_awburst           => ipif_dma.axi_awburst,
                m_axi_awprot            => ipif_dma.axi_awprot,
                m_axi_awcache           => ipif_dma.axi_awcache,
                m_axi_wready            => ipif_dma.axi_wready,
                m_axi_wvalid            => ipif_dma.axi_wvalid,
                m_axi_wdata             => ipif_dma.axi_wdata,
                m_axi_wstrb             => ipif_dma.axi_wstrb,
                m_axi_wlast             => ipif_dma.axi_wlast,
                m_axi_bready            => ipif_dma.axi_bready,
                m_axi_bvalid            => ipif_dma.axi_bvalid,
                m_axi_bresp             => ipif_dma.axi_bresp,
                ip2bus_mstrd_req        => ipif_dma.ipif_mstrd_req,
                ip2bus_mstwr_req        => ipif_dma.ipif_mstwr_req,
                ip2bus_mst_addr         => ipif_dma.ipif_mst_addr,
                ip2bus_mst_length       => ipif_dma.ipif_mst_length,
                ip2bus_mst_be           => ipif_dma.ipif_mst_be,
                ip2bus_mst_type         => ipif_dma.ipif_mst_type,
                ip2bus_mst_lock         => ipif_dma.ipif_mst_lock,
                ip2bus_mst_reset        => ipif_dma.ipif_mst_reset,
                bus2ip_mst_cmdack       => ipif_dma.ipif_mst_cmdack,
                bus2ip_mst_cmplt        => ipif_dma.ipif_mst_cmplt,
                bus2ip_mst_error        => ipif_dma.ipif_mst_error,
                bus2ip_mst_rearbitrate  => ipif_dma.ipif_mst_rearbitrate,
                bus2ip_mst_cmd_timeout  => ipif_dma.ipif_mst_cmd_timeout,
                bus2ip_mstrd_d          => ipif_dma.ipif_mstrd_d,
                bus2ip_mstrd_rem        => ipif_dma.ipif_mstrd_rem,
                bus2ip_mstrd_sof_n      => ipif_dma.ipif_mstrd_sof_n,
                bus2ip_mstrd_eof_n      => ipif_dma.ipif_mstrd_eof_n,
                bus2ip_mstrd_src_rdy_n  => ipif_dma.ipif_mstrd_src_rdy_n,
                bus2ip_mstrd_src_dsc_n  => ipif_dma.ipif_mstrd_src_dsc_n,
                ip2bus_mstrd_dst_rdy_n  => ipif_dma.ipif_mstrd_dst_rdy_n,
                ip2bus_mstrd_dst_dsc_n  => ipif_dma.ipif_mstrd_dst_dsc_n,
                ip2bus_mstwr_d          => ipif_dma.ipif_mstwr_d,
                ip2bus_mstwr_rem        => ipif_dma.ipif_mstwr_rem,
                ip2bus_mstwr_sof_n      => ipif_dma.ipif_mstwr_sof_n,
                ip2bus_mstwr_eof_n      => ipif_dma.ipif_mstwr_eof_n,
                ip2bus_mstwr_src_rdy_n  => ipif_dma.ipif_mstwr_src_rdy_n,
                ip2bus_mstwr_src_dsc_n  => ipif_dma.ipif_mstwr_src_dsc_n,
                bus2ip_mstwr_dst_rdy_n  => ipif_dma.ipif_mstwr_dst_rdy_n,
                bus2ip_mstwr_dst_dsc_n  => ipif_dma.ipif_mstwr_dst_dsc_n
            );

        --! The IPIF master handler converts the IPIF master signals to the
        --! openMAC's DMA interface.
        THEMACDMA_IPIF_HANDLER : entity work.ipifMasterHandler
            generic map (
                gMasterAddrWidth        => ipif_dmaMasterHdler.masterAddress'length,
                gMasterBurstCountWidth  => ipif_dmaMasterHdler.masterBurstcount'length,
                gIpifAddrWidth          => ipif_dmaMasterHdler.ipif_addr'length,
                gIpifLength             => ipif_dmaMasterHdler.ipif_length'length
            )
            port map (
                iRst                    => ipif_dmaMasterHdler.rst,
                iClk                    => ipif_dmaMasterHdler.clk,
                iIpif_cmdAck            => ipif_dmaMasterHdler.ipif_cmdAck,
                iIpif_cmplt             => ipif_dmaMasterHdler.ipif_cmplt,
                iIpif_error             => ipif_dmaMasterHdler.ipif_error,
                iIpif_rearbitrate       => ipif_dmaMasterHdler.ipif_rearbitrate,
                iIpif_cmdTimeout        => ipif_dmaMasterHdler.ipif_cmdTimeout,
                oIpif_type              => ipif_dmaMasterHdler.ipif_type,
                oIpif_addr              => ipif_dmaMasterHdler.ipif_addr,
                oIpif_length            => ipif_dmaMasterHdler.ipif_length,
                oIpif_be                => ipif_dmaMasterHdler.ipif_be,
                oIpif_lock              => ipif_dmaMasterHdler.ipif_lock,
                oIpif_reset             => ipif_dmaMasterHdler.ipif_reset,
                iIpif_rdData            => ipif_dmaMasterHdler.ipif_rdData,
                iIpif_rdRem             => ipif_dmaMasterHdler.ipif_rdRem,
                oIpif_rdReq             => ipif_dmaMasterHdler.ipif_rdReq,
                inIpif_rdSof            => ipif_dmaMasterHdler.nIpif_rdSof,
                inIpif_rdEof            => ipif_dmaMasterHdler.nIpif_rdEof,
                inIpif_rdSrcRdy         => ipif_dmaMasterHdler.nIpif_rdSrcRdy,
                inIpif_rdSrcDsc         => ipif_dmaMasterHdler.nIpif_rdSrcDsc,
                onIpif_rdDstRdy         => ipif_dmaMasterHdler.nIpif_rdDstRdy,
                onIpif_rdDstDsc         => ipif_dmaMasterHdler.nIpif_rdDstDsc,
                oIpif_wrData            => ipif_dmaMasterHdler.ipif_wrData,
                oIpif_wrRem             => ipif_dmaMasterHdler.ipif_wrRem,
                oIpif_wrReq             => ipif_dmaMasterHdler.ipif_wrReq,
                onIpif_wrSof            => ipif_dmaMasterHdler.nIpif_wrSof,
                onIpif_wrEof            => ipif_dmaMasterHdler.nIpif_wrEof,
                onIpif_wrSrcRdy         => ipif_dmaMasterHdler.nIpif_wrSrcRdy,
                onIpif_wrSrcDsc         => ipif_dmaMasterHdler.nIpif_wrSrcDsc,
                inIpif_wrDstRdy         => ipif_dmaMasterHdler.nIpif_wrDstRdy,
                inIpif_wrDstDsc         => ipif_dmaMasterHdler.nIpif_wrDstDsc,
                iMasterRead             => ipif_dmaMasterHdler.masterRead,
                iMasterWrite            => ipif_dmaMasterHdler.masterWrite,
                iMasterAddress          => ipif_dmaMasterHdler.masterAddress,
                iMasterWritedata        => ipif_dmaMasterHdler.masterWritedata,
                iMasterBurstcount       => ipif_dmaMasterHdler.masterBurstcount,
                iMasterBurstcounter     => ipif_dmaMasterHdler.masterBurstcounter,
                oMasterReaddata         => ipif_dmaMasterHdler.masterReaddata,
                oMasterWaitrequest      => ipif_dmaMasterHdler.masterWaitrequest,
                oMasterReaddatavalid    => ipif_dmaMasterHdler.masterReaddatavalid
            );
    end generate GEN_THEMACDMA;

    GEN_RMII_CLK : if gPhyPortType = cPhyPortRmii generate
        GEN_ODDR2 : for i in oRmii_clk'range generate
            signal rmiiClk  : std_logic;
            signal nRmiiClk : std_logic;
        begin
            -- Assign rmii clock (used by openMAC) and the inverted to ODDR2.
            rmiiClk     <= intf_clkRst.clk;
            nRmiiClk    <= not rmiiClk;

            --! This is a dual data rate output FF used to output the internal
            --! RMII clock.
            THEODDR2 : oddr2
                generic map (
                    DDR_ALIGNMENT   => "NONE",  -- align D0 with C0 and D1 with C1 edge
                    INIT            => '0',     -- initialize Q with '0'
                    SRTYPE          => "SYNC"   -- take default, since RS are unused
                )
                port map (
                    D0  => cActivated,
                    D1  => cInactivated,
                    C0  => rmiiClk,
                    C1  => nRmiiClk,
                    CE  => cActivated,
                    R   => cInactivated, --unused
                    S   => cInactivated, --unused
                    Q   => oRmii_clk(i)
                );
        end generate GEN_ODDR2;
    end generate GEN_RMII_CLK;
end rtl;
