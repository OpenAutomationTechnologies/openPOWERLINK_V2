-------------------------------------------------------------------------------
--! @file hostInterface.vhd
--
--! @brief toplevel of host interface
--
--! @details The toplevel instantiates the necessary components for the
--! host interface like the Dynamic Bridge and the Status-/Control Registers.
--
-------------------------------------------------------------------------------
--
--    (c) B&R, 2012
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
--! use global library
use work.global.all;
--! use host interface package for specific types
use work.hostInterfacePkg.all;

entity hostInterface is
    generic (
        --! Version major
        gVersionMajor       : natural := 16#FF#;
        --! Version minor
        gVersionMinor       : natural := 16#FF#;
        --! Version revision
        gVersionRevision    : natural := 16#FF#;
        --! Version count
        gVersionCount       : natural := 0;
        --! Use memory blocks or registers for translation address storage (registers = 0, memory blocks /= 0)
        gBridgeUseMemBlock  : natural := 0;
        -- Base address mapping
        --! Base address Dynamic Buffer 0
        gBaseDynBuf0        : natural := 16#00800#;
        --! Base address Dynamic Buffer 1
        gBaseDynBuf1        : natural := 16#01000#;
        --! Base address Error Counter
        gBaseErrCntr        : natural := 16#01800#;
        --! Base address TX NMT Queue
        gBaseTxNmtQ         : natural := 16#02800#;
        --! Base address TX Generic Queue
        gBaseTxGenQ         : natural := 16#03800#;
        --! Base address TX SyncRequest Queue
        gBaseTxSynQ         : natural := 16#04800#;
        --! Base address TX Virtual Ethernet Queue
        gBaseTxVetQ         : natural := 16#05800#;
        --! Base address RX Virtual Ethernet Queue
        gBaseRxVetQ         : natural := 16#06800#;
        --! Base address Kernel-to-User Queue
        gBaseK2UQ           : natural := 16#07000#;
        --! Base address User-to-Kernel Queue
        gBaseU2KQ           : natural := 16#09000#;
        --! Base address Tpdo
        gBaseTpdo           : natural := 16#0B000#;
        --! Base address Rpdo
        gBaseRpdo           : natural := 16#0E000#;
        --! Base address Reserved (-1 = high address of Rpdo)
        gBaseRes            : natural := 16#14000#
    );
    port (
        --! Clock Source input
        iClk                    : in std_logic;
        --! Reset Source input
        iRst                    : in std_logic;
        -- Memory Mapped Slave for Host
        --! MM slave host address
        iHostAddress            : in std_logic_vector(16 downto 2);
        --! MM slave host byteenable
        iHostByteenable         : in std_logic_vector(3 downto 0);
        --! MM slave host read
        iHostRead               : in std_logic;
        --! MM slave host readdata
        oHostReaddata           : out std_logic_vector(31 downto 0);
        --! MM slave host write
        iHostWrite              : in std_logic;
        --! MM slave host writedata
        iHostWritedata          : in std_logic_vector(31 downto 0);
        --! MM slave host waitrequest
        oHostWaitrequest        : out std_logic;
        -- Memory Mapped Slave for PCP
        --! MM slave pcp address
        iPcpAddress             : in std_logic_vector(10 downto 2);
        --! MM slave pcp byteenable
        iPcpByteenable          : in std_logic_vector(3 downto 0);
        --! MM slave pcp read
        iPcpRead                : in std_logic;
        --! MM slave pcp readdata
        oPcpReaddata            : out std_logic_vector(31 downto 0);
        --! MM slave pcp write
        iPcpWrite               : in std_logic;
        --! MM slave pcp writedata
        iPcpWritedata           : in std_logic_vector(31 downto 0);
        --! MM slave pcp waitrequest
        oPcpWaitrequest         : out std_logic;
        -- Memory Mapped Master for Host via Dynamic Bridge
        --! MM master hostBridge address
        oHostBridgeAddress      : out std_logic_vector(29 downto 0);
        --! MM master hostBridge byteenable
        oHostBridgeByteenable   : out std_logic_vector(3 downto 0);
        --! MM master hostBridge read
        oHostBridgeRead         : out std_logic;
        --! MM master hostBridge readdata
        iHostBridgeReaddata     : in std_logic_vector(31 downto 0);
        --! MM master hostBridge write
        oHostBridgeWrite        : out std_logic;
        --! MM master hostBridge writedata
        oHostBridgeWritedata    : out std_logic_vector(31 downto 0);
        --! MM master hostBridge waitrequest
        iHostBridgeWaitrequest  : in std_logic;
        --! Interrupt internal sync signal (from openMAC)
        iIrqIntSync             : in std_logic;
        --! External sync source
        iIrqExtSync             : in std_logic;
        --! Interrupt output signal
        oIrq                    : out std_logic;
        --! Node Id
        iNodeId                 : in std_logic_vector(7 downto 0);
        --! POWERLINK Error LED
        oPlkLedError            : out std_logic;
        --! POWERLINK Status LED
        oPlkLedStatus           : out std_logic
    );
end hostInterface;

architecture Rtl of hostInterface is
    --! Magic
    constant cMagic                 : natural := 16#504C4B00#;
    --! Base address array
    constant cBaseAddressArray      : tArrayStd32 := (
        std_logic_vector(to_unsigned(gBaseDynBuf0,  cArrayStd32ElementSize)),
        std_logic_vector(to_unsigned(gBaseDynBuf1,  cArrayStd32ElementSize)),
        std_logic_vector(to_unsigned(gBaseErrCntr,  cArrayStd32ElementSize)),
        std_logic_vector(to_unsigned(gBaseTxNmtQ,   cArrayStd32ElementSize)),
        std_logic_vector(to_unsigned(gBaseTxGenQ,   cArrayStd32ElementSize)),
        std_logic_vector(to_unsigned(gBaseTxSynQ,   cArrayStd32ElementSize)),
        std_logic_vector(to_unsigned(gBaseTxVetQ,   cArrayStd32ElementSize)),
        std_logic_vector(to_unsigned(gBaseRxVetQ,   cArrayStd32ElementSize)),
        std_logic_vector(to_unsigned(gBaseK2UQ,     cArrayStd32ElementSize)),
        std_logic_vector(to_unsigned(gBaseU2KQ,     cArrayStd32ElementSize)),
        std_logic_vector(to_unsigned(gBaseTpdo,     cArrayStd32ElementSize)),
        std_logic_vector(to_unsigned(gBaseRpdo,     cArrayStd32ElementSize)),
        std_logic_vector(to_unsigned(gBaseRes,      cArrayStd32ElementSize))
    );
    --! Base address array count
    constant cBaseAddressArrayCount : natural := cBaseAddressArray'length;
    --! Base address set by host
    constant cBaseAddressHostCount  : natural := 2;
    --! Base address set by pcp
    constant cBaseAddressPcpCount   : natural := cBaseAddressArrayCount-cBaseAddressHostCount;
    --! Number of interrupt sources (sync not included)
    constant cIrqSourceCount        : natural := 3;

    --! Bridge fsm type
    type tFsm is (
        sIdle,
        sReqAddr,
        sAccess,
        sDone
    );

    --! select the bridge logic
    signal bridgeSel                : std_logic;
    --! invalid address range selected
    signal invalidSel               : std_logic;
    --! select status control registers
    signal statCtrlSel              : std_logic;
    --! write status control register
    signal statCtrlWrite            : std_logic;
    --! read status control register
    signal statCtrlRead             : std_logic;
    --! waitrequest from status/control
    signal statCtrlWaitrequest      : std_logic;
    --! readdata from status/control
    signal statCtrlReaddata         : std_logic_vector(oHostReaddata'range);
    --! Bridge request signal
    signal bridgeRequest            : std_logic;
    --! Bridge enable control
    signal bridgeEnable             : std_logic;
    --! Bridge address is valid
    signal bridgeAddrValid          : std_logic;
    --! LED from status/control registers
    signal statCtrlLed              : std_logic_vector(1 downto 0);
    --! The magic bridge outputs the dword address
    signal hostBridgeAddress_dword  : std_logic_vector(oHostBridgeAddress'length-1 downto 2);
    --! Bridge transfer done strobe
    signal bridgeTfDone             : std_logic;
    --! Bridge read data
    signal bridgeReaddata           : std_logic_vector(iHostBridgeReaddata'range);

    --! Bridge state machine
    signal fsm      : tFsm;
    --! Bridge state machine, next state
    signal fsm_next : tFsm;

    -- base set signals
    --! BaseSet Write
    signal baseSetWrite         : std_logic;
    --! BaseSet Read
    signal baseSetRead          : std_logic;
    --! BaseSet byteenable
    signal baseSetByteenable    : std_logic_vector(3 downto 0);
    --! BaseSet Writedata
    signal baseSetWritedata     : std_logic_vector(hostBridgeAddress_dword'range);
    --! BaseSet Readdata
    signal baseSetReaddata      : std_logic_vector(hostBridgeAddress_dword'range);
    --! BaseSet Address
    signal baseSetAddress       : std_logic_vector(logDualis(cBaseAddressArrayCount)-1 downto 0);
    --! BaseSet acknowledge
    signal baseSetAck           : std_logic;

    -- interrupt signals
    --! Irq master enable
    signal irqMasterEnable  : std_logic;
    --! Irq source enable
    signal irqSourceEnable  : std_logic_vector(cIrqSourceCount downto 0);
    --! Irq acknowledge
    signal irqAcknowledge   : std_logic_vector(cIrqSourceCount downto 0);
    --! Irq source pending
    signal irqSourcePending : std_logic_vector(cIrqSourceCount downto 0);
    --! Irq source set (no sync!)
    signal irqSourceSet     : std_logic_vector(cIrqSourceCount downto 1);
    --! sync signal
    signal syncSig : std_logic;
    --! synchronized ext sync
    signal extSync_sync     : std_logic;
    --! external sync signal
    signal extSyncEnable    : std_logic;
    --! external sync config
    signal extSyncConfig    : std_logic_vector(cExtSyncEdgeConfigWidth-1 downto 0);
    --! external sync signal detected rising edge
    signal extSync_rising   : std_logic;
    --! external sync signal detected falling edge
    signal extSync_falling  : std_logic;
    --! external sync signal detected any edge
    signal extSync_any      : std_logic;
begin

    -- select status/control registers if host address is below 2 kB
    statCtrlSel <=  cActivated when iHostAddress < cBaseAddressArray(0)(iHostAddress'range) else
                    cInactivated;

    -- select invalid address
    invalidSel <=   cActivated when iHostAddress >= cBaseAddressArray(cBaseAddressArrayCount-1)(iHostAddress'range) else
                    cInactivated;

    -- bridge is selected if status/control registers are not accessed
    bridgeSel <=    cInactivated when bridgeEnable = cInactivated else
                    cInactivated when invalidSel = cActivated else
                    cInactivated when statCtrlSel = cActivated else
                    cActivated;

    -- create write and read strobe for status/control registers
    statCtrlWrite   <= iHostWrite and statCtrlSel;
    statCtrlRead    <= iHostRead and statCtrlSel;

    -- host waitrequest from status/control, bridge or invalid
    oHostWaitrequest <= statCtrlWaitrequest when statCtrlSel = cActivated else
                        not (iHostWrite or iHostRead) when invalidSel = cActivated else
                        cInactivated when bridgeEnable = cInactivated else
                        not bridgeTfDone when bridgeSel = cActivated else
                        cActivated;

    -- host readdata from status/control or bridge
    oHostReaddata <=    bridgeReaddata when bridgeSel = cActivated else
                        statCtrlReaddata when statCtrlSel = cActivated else
                        (others => cInactivated);

    -- select external sync if enabled, otherwise rx irq signal
    syncSig <=  iIrqIntSync when extSyncEnable /= cActivated else
                extSync_rising when extSyncConfig = cExtSyncEdgeRis else
                extSync_falling when extSyncConfig = cExtSyncEdgeFal else
                extSync_any when extSyncConfig = cExtSyncEdgeAny else
                cInactivated;

    --! The bridge state machine handles the address translation of
    --! dynamicBridge and finalizes the access to the host bridge master.
    theFsmCom : process (
        fsm,
        bridgeSel,
        bridgeAddrValid,
        iHostRead,
        iHostWrite,
        iHostBridgeWaitrequest
    )
    begin
        --default
        fsm_next <= fsm;

        case fsm is
            when sIdle =>
                if ( (iHostRead = cActivated or iHostWrite = cActivated) and
                    bridgeSel = cActivated) then
                    fsm_next <= sReqAddr;
                end if;
            when sReqAddr =>
                if bridgeAddrValid = cActivated then
                    fsm_next <= sAccess;
                end if;
            when sAccess =>
                if iHostBridgeWaitrequest = cInactivated then
                    fsm_next <= sDone;
                end if;
            when sDone =>
                fsm_next <= sIdle;
        end case;
    end process;

    bridgeRequest   <=  cActivated when fsm = sReqAddr else cInactivated;
    bridgeTfDone    <=  cActivated when fsm = sDone else cInactivated;

    --! Clock process to assign registers.
    theClkPro : process(iRst, iClk)
    begin
        if iRst = cActivated then
            fsm                     <= sIdle;
            oHostBridgeAddress      <= (others => cInactivated);
            oHostBridgeByteenable   <= (others => cInactivated);
            oHostBridgeRead         <= cInactivated;
            oHostBridgeWrite        <= cInactivated;
            oHostBridgeWritedata    <= (others => cInactivated);
        elsif rising_edge(iClk) then
            fsm <= fsm_next;
            if iHostBridgeWaitrequest = cInactivated then
                oHostBridgeRead         <= cInactivated;
                oHostBridgeWrite        <= cInactivated;
                bridgeReaddata          <= iHostBridgeReaddata;
            end if;
            if bridgeAddrValid = cActivated then
                oHostBridgeAddress      <= hostBridgeAddress_dword & "00";
                oHostBridgeByteenable   <= iHostByteenable;
                oHostBridgeRead         <= iHostRead;
                oHostBridgeWrite        <= iHostWrite;
                oHostBridgeWritedata    <= iHostWritedata;
            end if;
        end if;
    end process;

    --! The synchronizer which protects us from crazy effects!
    theSynchronizer : entity work.synchronizer
    generic map (
        gStages => 2,
        gInit   => cInactivated
    )
    port map (
        iArst   => iRst,
        iClk    => iClk,
        iAsync  => iIrqExtSync,
        oSync   => extSync_sync
    );

    --! The Edge Detector for external sync
    theExtSyncEdgeDet : entity work.edgedetector
    port map (
        iArst       => iRst,
        iClk        => iClk,
        iEnable     => cActivated,
        iData       => extSync_sync,
        oRising     => extSync_rising,
        oFalling    => extSync_falling,
        oAny        => extSync_any
    );

    --! The Dynamic Bridge
    theDynamicBridge : entity work.dynamicBridge
    generic map (
        gAddressSpaceCount      => cBaseAddressArrayCount-1,
        gUseMemBlock            => gBridgeUseMemBlock,
        gBaseAddressArray       => cBaseAddressArray
    )
    port map (
        iClk                    => iClk,
        iRst                    => iRst,
        iBridgeAddress          => iHostAddress,
        iBridgeRequest          => bridgeRequest,
        oBridgeAddress          => hostBridgeAddress_dword,
        oBridgeSelectAny        => open,
        oBridgeSelect           => open,
        oBridgeValid            => bridgeAddrValid,
        iBaseSetWrite           => baseSetWrite,
        iBaseSetRead            => baseSetRead,
        iBaseSetByteenable      => baseSetByteenable,
        iBaseSetAddress         => baseSetAddress,
        iBaseSetData            => baseSetWritedata,
        oBaseSetData            => baseSetReaddata,
        oBaseSetAck             => basesetAck
    );

    --! The Irq Generator
    theIrqGen : entity work.irqGen
    generic map (
        gIrqSourceCount         => cIrqSourceCount
    )
    port map (
        iClk                    => iClk,
        iRst                    => iRst,
        iSync                   => syncSig,
        iIrqSource              => irqSourceSet,
        oIrq                    => oIrq,
        iIrqMasterEnable        => irqMasterEnable,
        iIrqSourceEnable        => irqSourceEnable,
        iIrqAcknowledge         => irqAcknowledge,
        oIrgPending             => irqSourcePending
    );


    --! The Status-/Control Registers
    theStCtrlReg : entity work.statusControlReg
    generic map (
        gMagic                  => cMagic,
        gVersionMajor           => gVersionMajor,
        gVersionMinor           => gVersionMinor,
        gVersionRevision        => gVersionRevision,
        gVersionCount           => gVersionCount,
        gHostBaseSet            => cBaseAddressHostCount,
        gPcpBaseSet             => cBaseAddressPcpCount,
        gIrqSourceCount         => cIrqSourceCount
    )
    port map (
        iClk                    => iClk,
        iRst                    => iRst,
        iHostRead               => statCtrlRead,
        iHostWrite              => statCtrlWrite,
        iHostByteenable         => iHostByteenable,
        iHostAddress            => iHostAddress(10 downto 2),
        oHostReaddata           => statCtrlReaddata,
        iHostWritedata          => iHostWritedata,
        oHostWaitrequest        => statCtrlWaitrequest,
        iPcpRead                => iPcpRead,
        iPcpWrite               => iPcpWrite,
        iPcpByteenable          => iPcpByteenable,
        iPcpAddress             => iPcpAddress,
        oPcpReaddata            => oPcpReaddata,
        iPcpWritedata           => iPcpWritedata,
        oPcpWaitrequest         => oPcpWaitrequest,
        oBaseSetWrite           => baseSetWrite,
        oBaseSetRead            => baseSetRead,
        oBaseSetByteenable      => baseSetByteenable,
        oBaseSetAddress         => baseSetAddress,
        iBaseSetData            => baseSetReaddata,
        oBaseSetData            => baseSetWritedata,
        iBaseSetAck             => basesetAck,
        oIrqMasterEnable        => irqMasterEnable,
        oIrqSourceEnable        => irqSourceEnable,
        oIrqAcknowledge         => irqAcknowledge,
        oIrqSet                 => irqSourceSet,
        iIrqPending             => irqSourcePending,
        oExtSyncEnable          => extSyncEnable,
        oExtSyncConfig          => extSyncConfig,
        iNodeId                 => iNodeId,
        oPLed                   => statCtrlLed,
        oBridgeEnable           => bridgeEnable
    );

    oPlkLedStatus <= statCtrlLed(0);

    oPlkLedError <= statCtrlLed(1);

end Rtl;
