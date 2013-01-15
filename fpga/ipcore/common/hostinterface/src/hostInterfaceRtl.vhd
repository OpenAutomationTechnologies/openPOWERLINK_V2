-------------------------------------------------------------------------------
--! @file hostInterface.vhd
--
--! @brief toplevel of host interface
--
--! @details The toplevel instantiates the necessary components for the
--! host interface like the Magic Bridge and the Status-/Control Registers.
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
        gVersionMajor : natural := 16#FF#;
        --! Version minor
        gVersionMinor : natural := 16#FF#;
        --! Version revision
        gVersionRevision : natural := 16#FF#;
        --! Version count
        gVersionCount : natural := 0;
        -- Base address mapping
        --! Base address Dynamic Buffer 0
        gBaseDynBuf0 : natural := 16#00800#;
        --! Base address Dynamic Buffer 1
        gBaseDynBuf1 : natural := 16#01000#;
        --! Base address Error Counter
        gBaseErrCntr : natural := 16#01800#;
        --! Base address TX NMT Queue
        gBaseTxNmtQ : natural := 16#02800#;
        --! Base address TX Generic Queue
        gBaseTxGenQ : natural := 16#03800#;
        --! Base address TX SyncRequest Queue
        gBaseTxSynQ : natural := 16#04800#;
        --! Base address TX Virtual Ethernet Queue
        gBaseTxVetQ : natural := 16#05800#;
        --! Base address RX Virtual Ethernet Queue
        gBaseRxVetQ : natural := 16#06800#;
        --! Base address Kernel-to-User Queue
        gBaseK2UQ : natural := 16#07000#;
        --! Base address User-to-Kernel Queue
        gBaseU2KQ : natural := 16#09000#;
        --! Base address Tpdo
        gBaseTpdo : natural := 16#0B000#;
        --! Base address Rpdo
        gBaseRpdo : natural := 16#0E000#;
        --! Base address Reserved (-1 = high address of Rpdo)
        gBaseRes : natural := 16#14000#
        );
    port (
        --! Clock Source input
        csi_c0_clock : in std_logic;
        --! Reset Source input
        rsi_r0_reset : in std_logic;
        -- Avalon Memory Mapped Slave for Host
        --! Avalon-MM slave host address
        avs_host_address : in std_logic_vector(16 downto 2);
        --! Avalon-MM slave host byteenable
        avs_host_byteenable : in std_logic_vector(3 downto 0);
        --! Avalon-MM slave host read
        avs_host_read : in std_logic;
        --! Avalon-MM slave host readdata
        avs_host_readdata : out std_logic_vector(31 downto 0);
        --! Avalon-MM slave host write
        avs_host_write : in std_logic;
        --! Avalon-MM slave host writedata
        avs_host_writedata : in std_logic_vector(31 downto 0);
        --! Avalon-MM slave host waitrequest
        avs_host_waitrequest : out std_logic;
        -- Avalon Memory Mapped Slave for PCP
        --! Avalon-MM slave pcp address
        avs_pcp_address : in std_logic_vector(10 downto 2);
        --! Avalon-MM slave pcp byteenable
        avs_pcp_byteenable : in std_logic_vector(3 downto 0);
        --! Avalon-MM slave pcp read
        avs_pcp_read : in std_logic;
        --! Avalon-MM slave pcp readdata
        avs_pcp_readdata : out std_logic_vector(31 downto 0);
        --! Avalon-MM slave pcp write
        avs_pcp_write : in std_logic;
        --! Avalon-MM slave pcp writedata
        avs_pcp_writedata : in std_logic_vector(31 downto 0);
        --! Avalon-MM slave pcp waitrequest
        avs_pcp_waitrequest : out std_logic;
        -- Avalon Memory Mapped Master for Host via Magic Bridge
        --! Avalon-MM master hostBridge address
        avm_hostBridge_address : out std_logic_vector(29 downto 0);
        --! Avalon-MM master hostBridge byteenable
        avm_hostBridge_byteenable : out std_logic_vector(3 downto 0);
        --! Avalon-MM master hostBridge read
        avm_hostBridge_read : out std_logic;
        --! Avalon-MM master hostBridge readdata
        avm_hostBridge_readdata : in std_logic_vector(31 downto 0);
        --! Avalon-MM master hostBridge write
        avm_hostBridge_write : out std_logic;
        --! Avalon-MM master hostBridge writedata
        avm_hostBridge_writedata : out std_logic_vector(31 downto 0);
        --! Avalon-MM master hostBridge waitrequest
        avm_hostBridge_waitrequest : in std_logic;
        --! Interrupt receiver
        inr_irqSync_irq : in std_logic;
        --! Interrupt sender
        ins_irqOut_irq : out std_logic;
        --! External Sync Source
        coe_ExtSync_exsync : in std_logic;
        --! Node Id
        coe_NodeId_nodeid : in std_logic_vector(7 downto 0);
        --! POWERLINK Error LED
        coe_PlkLed_lederr : out std_logic;
        --! POWERLINK Status LED
        coe_PlkLed_ledst : out std_logic
        );
end hostInterface;

architecture Rtl of hostInterface is

    --! The Magic Bridge
    component magicBridge
        generic(
            gAddressSpaceCount : NATURAL := 2;
            gBaseAddressArray : tArrayStd32 := (x"0000_1000",x"0000_2000",x"0000_3000"));
        port(
            iClk : in STD_LOGIC;
            iRst : in STD_LOGIC;
            iBridgeAddress : in STD_LOGIC_VECTOR;
            iBridgeSelect : in STD_LOGIC;
            oBridgeAddress : out STD_LOGIC_VECTOR;
            oBridgeSelectAny : out STD_LOGIC;
            oBridgeSelect : out STD_LOGIC_VECTOR(gAddressSpaceCount-1 downto 0);
            iBaseSetWrite : in STD_LOGIC;
            iBaseSetAddress : in STD_LOGIC_VECTOR(LogDualis(gAddressSpaceCount)-1 downto 0);
            iBaseSetData : in STD_LOGIC_VECTOR;
            oBaseSetData : out STD_LOGIC_VECTOR);
    end component;


    --! The Status-/Control Registers
    component statusControlReg
        generic(
            gMagic : NATURAL := 1347177216;
            gVersionMajor : NATURAL := 255;
            gVersionMinor : NATURAL := 255;
            gVersionRevision : NATURAL := 255;
            gVersionCount : NATURAL := 0;
            gHostBaseSet : NATURAL := 2;
            gPcpBaseSet : NATURAL := 10;
            gIrqSourceCount : NATURAL range 1 to 15 := 3);
        port(
            iClk : in STD_LOGIC;
            iRst : in STD_LOGIC;
            iHostRead : in STD_LOGIC;
            iHostWrite : in STD_LOGIC;
            iHostByteenable : in STD_LOGIC_VECTOR(cDword/8-1 downto 0);
            iHostAddress : in STD_LOGIC_VECTOR(10 downto 2);
            oHostReaddata : out STD_LOGIC_VECTOR(cDword-1 downto 0);
            iHostWritedata : in STD_LOGIC_VECTOR(cDword-1 downto 0);
            oHostWaitrequest : out STD_LOGIC;
            iPcpRead : in STD_LOGIC;
            iPcpWrite : in STD_LOGIC;
            iPcpByteenable : in STD_LOGIC_VECTOR(cDword/8-1 downto 0);
            iPcpAddress : in STD_LOGIC_VECTOR(10 downto 2);
            oPcpReaddata : out STD_LOGIC_VECTOR(cDword-1 downto 0);
            iPcpWritedata : in STD_LOGIC_VECTOR(cDword-1 downto 0);
            oPcpWaitrequest : out STD_LOGIC;
            oBaseSetWrite : out STD_LOGIC;
            oBaseSetAddress : out STD_LOGIC_VECTOR(LogDualis(gHostBaseSet+gPcpBaseSet)+2-1 downto 2);
            iBaseSetData : in STD_LOGIC_VECTOR;
            oBaseSetData : out STD_LOGIC_VECTOR;
            oIrqMasterEnable : out STD_LOGIC;
            oIrqSourceEnable : out STD_LOGIC_VECTOR(gIrqSourceCount downto 0);
            oIrqAcknowledge : out STD_LOGIC_VECTOR(gIrqSourceCount downto 0);
            oIrqSet : out STD_LOGIC_VECTOR(gIrqSourceCount downto 1);
            iIrqPending : in STD_LOGIC_VECTOR(gIrqSourceCount downto 0);
            oExtSyncEnable : out STD_LOGIC;
            oExtSyncConfig : out STD_LOGIC_VECTOR(cExtSyncEdgeConfigWidth-1 downto 0);
            iNodeId : in STD_LOGIC_VECTOR(cByte-1 downto 0);
            oPLed : out STD_LOGIC_VECTOR(1 downto 0);
            oBridgeEnable : out STD_LOGIC);
    end component;


    --! The Irq Generator
    component irqGen
        generic(
            gIrqSourceCount : natural := 3);
        port(
            iClk : in std_logic;
            iRst : in std_logic;
            iSync : in std_logic;
            iIrqSource : in std_logic_vector(gIrqSourceCount downto 1);
            oIrq : out std_logic;
            iIrqMasterEnable : in std_logic;
            iIrqSourceEnable : in std_logic_vector(gIrqSourceCount downto 0);
            iIrqAcknowledge : in std_logic_vector(gIrqSourceCount downto 0);
            oIrgPending : out std_logic_vector(gIrqSourceCount downto 0));
    end component;


    --! An Edge Detector
    component edgeDet
        port(
            din : in std_logic;
            rising : out std_logic;
            falling : out std_logic;
            any : out std_logic;
            clk : in std_logic;
            rst : in std_logic);
    end component;


    --! A Synchronizer
    component sync
        generic(
            doSync_g : BOOLEAN := true);
        port(
            clk : in STD_LOGIC;
            rst : in STD_LOGIC;
            din : in STD_LOGIC;
            dout : out STD_LOGIC);
    end component;



    --! Magic
    constant cMagic : natural := 16#504C4B00#;

    --! Base address array
    constant cBaseAddressArray : tArrayStd32 :=
    (
    std_logic_vector(to_unsigned(gBaseDynBuf0,  cArrayStd32ElementSize)) ,
    std_logic_vector(to_unsigned(gBaseDynBuf1,  cArrayStd32ElementSize)) ,
    std_logic_vector(to_unsigned(gBaseErrCntr,  cArrayStd32ElementSize)) ,
    std_logic_vector(to_unsigned(gBaseTxNmtQ,   cArrayStd32ElementSize)) ,
    std_logic_vector(to_unsigned(gBaseTxGenQ,   cArrayStd32ElementSize)) ,
    std_logic_vector(to_unsigned(gBaseTxSynQ,   cArrayStd32ElementSize)) ,
    std_logic_vector(to_unsigned(gBaseTxVetQ,   cArrayStd32ElementSize)) ,
    std_logic_vector(to_unsigned(gBaseRxVetQ,   cArrayStd32ElementSize)) ,
    std_logic_vector(to_unsigned(gBaseK2UQ,     cArrayStd32ElementSize)) ,
    std_logic_vector(to_unsigned(gBaseU2KQ,     cArrayStd32ElementSize)) ,
    std_logic_vector(to_unsigned(gBaseTpdo,     cArrayStd32ElementSize)) ,
    std_logic_vector(to_unsigned(gBaseRpdo,     cArrayStd32ElementSize)) ,
    std_logic_vector(to_unsigned(gBaseRes,      cArrayStd32ElementSize))
    );

    --! Base address array count
    constant cBaseAddressArrayCount : natural := cBaseAddressArray'length;

    --! Base address set by host
    constant cBaseAddressHostCount : natural := 2;

    --! Base address set by pcp
    constant cBaseAddressPcpCount : natural :=
    cBaseAddressArrayCount-cBaseAddressHostCount;

    --! Number of interrupt sources (sync not included)
    constant cIrqSourceCount : natural := 3;

    --! select the bridge logic
    signal bridgeSel : std_logic;

    --! invalid address range selected
    signal invalidSel : std_logic;

    --! select the status control registers
    signal statCtrlSel : std_logic;
    signal statCtrlWrite : std_logic;
    signal statCtrlRead : std_logic;

    --! waitrequest from status/control
    signal statCtrlWaitrequest : std_logic;

    --! readdata from status/control
    signal statCtrlReaddata : std_logic_vector(avs_host_readdata'range);

    --! bridge select output
    signal bridgeSelOut : std_logic;

    --! LED from status/control registers
    signal statCtrlLed : std_logic_vector(1 downto 0);

    --! Avalon master needs byte addresses - localy dword is used
    signal avm_hostBridge_address_dword : std_logic_vector
    (avm_hostBridge_address'length-1 downto 2);

    -- base set signals
    --! BaseSet Write
    signal baseSetWrite : std_logic;

    --! BaseSet Writedata
    signal baseSetWritedata : std_logic_vector
    (avm_hostBridge_address_dword'range);

    --! BaseSet Readdata
    signal baseSetReaddata : std_logic_vector
    (avm_hostBridge_address_dword'range);

    --! BaseSet Address
    signal baseSetAddress : std_logic_vector
    (LogDualis(cBaseAddressArrayCount)-1 downto 0);

    -- interrupt signals
    --! Irq master enable
    signal irqMasterEnable : std_logic;

    --! Irq source enable
    signal irqSourceEnable : std_logic_vector(cIrqSourceCount downto 0);

    --! Irq acknowledge
    signal irqAcknowledge : std_logic_vector(cIrqSourceCount downto 0);

    --! Irq source pending
    signal irqSourcePending : std_logic_vector(cIrqSourceCount downto 0);

    --! Irq source set (no sync!)
    signal irqSourceSet : std_logic_vector(cIrqSourceCount downto 1);

    --! sync signal
    signal syncSig : std_logic;

    --! synchronized ext sync
    signal extSync_sync : std_logic;

    --! external sync signal
    signal extSyncEnable : std_logic;

    --! external sync config
    signal extSyncConfig : std_logic_vector(cExtSyncEdgeConfigWidth-1 downto 0);

    --! external sync signal detected edge pulse
    signal extSync_rising, extSync_falling, extSync_any : std_logic;

    --! Bridge enable control
    signal bridgeEnable : std_logic;

    --! bridge read path
    signal bridgeReady : std_logic;
    signal bridgeReaddata : std_logic_vector(avm_hostBridge_readdata'range);

begin

    --! select status/control registers if host address is below 2 kB
    statCtrlSel <= cActivated when avs_host_address < cBaseAddressArray(0)(avs_host_address'range) else
    cInactivated;

    --! select invalid address
    invalidSel <= cActivated when avs_host_address >=
    cBaseAddressArray(cBaseAddressArrayCount-1)(avs_host_address'range) else
    cInactivated;

    --! bridge is selected if status/control registers are not accessed
    bridgeSel <= cInactivated when bridgeEnable = cInactivated else
    cInactivated when invalidSel = cActivated else
    cInactivated when statCtrlSel = cActivated else
    cActivated;

    --! create write and read strobe for status/control registers
    statCtrlWrite <= avs_host_write and statCtrlSel;
    statCtrlRead <= avs_host_read and statCtrlSel;

    --! host waitrequest from status/control, bridge or invalid
    avs_host_waitrequest <=
    statCtrlWaitrequest when statCtrlSel = cActivated else
    cInactivated when bridgeEnable = cInactivated else
    not bridgeReady when bridgeSel = cActivated else
    not invalidSel;

    --! host readdata from status/control or bridge
    avs_host_readdata <=
    bridgeReaddata when bridgeSel = cActivated else
    statCtrlReaddata when statCtrlSel = cActivated else
    (others => cInactivated);

    --! select external sync if enabled, otherwise rx irq signal
    syncSig <= inr_irqSync_irq when extSyncEnable /= cActivated else
                extSync_rising when extSyncConfig = cExtSyncEdgeRis else
                extSync_falling when extSyncConfig = cExtSyncEdgeFal else
                extSync_any when extSyncConfig = cExtSyncEdgeAny else
                cInactivated;

    --! The synchronizer which protects us from crazy effects!
    theSynchronizer : sync
    port map(
        clk => csi_c0_clock,
        rst => rsi_r0_reset,
        din => coe_ExtSync_exsync,
        dout => extSync_sync
        );

    --! The Edge Detector for external sync
    theExtSyncEdgeDet : edgeDet
    port map(
        din => extSync_sync,
        rising => extSync_rising,
        falling => extSync_falling,
        any => extSync_any,
        clk => csi_c0_clock,
        rst => rsi_r0_reset
        );

    --! The Magic Bridge
    theMagicBridge : magicBridge
    generic map(
        gAddressSpaceCount      => cBaseAddressArrayCount-1,
        gBaseAddressArray       => cBaseAddressArray
        )
    port map(
        iClk                    => csi_c0_clock,
        iRst                    => rsi_r0_reset,
        iBridgeAddress          => avs_host_address,
        iBridgeSelect           => bridgeSel,
        oBridgeAddress          => avm_hostBridge_address_dword,
        oBridgeSelectAny        => bridgeSelOut,
        oBridgeSelect           => open,
        iBaseSetWrite           => baseSetWrite,
        iBaseSetAddress         => baseSetAddress,
        iBaseSetData            => baseSetWritedata,
        oBaseSetData            => baseSetReaddata
        );

    process(csi_c0_clock)
    begin
        if rising_edge(csi_c0_clock) then
            if rsi_r0_reset = cActivated then
                avm_hostBridge_write <= cInactivated;
                avm_hostBridge_read <= cInactivated;
                avm_hostBridge_writedata <= (others => cInactivated);
                avm_hostBridge_byteenable <= (others => cInactivated);
                avm_hostBridge_address <= (others => cInactivated);
                bridgeReaddata <= (others => cInactivated);
                bridgeReady <= cInactivated;
            else
                --! generate hostBridge write and read strobes
                if bridgeSelOut = cActivated and
                    avm_hostBridge_waitrequest = cActivated and
                    bridgeReady = cInactivated then
                    --! if bridge is busy forward requests
                    avm_hostBridge_write <= avs_host_write;
                    avm_hostBridge_read <= avs_host_read;
                else
                    avm_hostBridge_write <= cInactivated;
                    avm_hostBridge_read <= cInactivated;
                end if;

                --! bridge writedata from host
                avm_hostBridge_writedata <= avs_host_writedata;

                --! bridge byteenable from host
                avm_hostBridge_byteenable <= avs_host_byteenable;

                --! bridge byte addressing
                avm_hostBridge_address <= avm_hostBridge_address_dword & "00";

                --! bridge readdata
                bridgeReaddata <= avm_hostBridge_readdata;

                --! bridge waitrequest
                bridgeReady <= not avm_hostBridge_waitrequest;

            end if;
        end if;
    end process;


    --! The Irq Generator
    theIrqGen : irqGen
    generic map(
        gIrqSourceCount         => cIrqSourceCount
        )
    port map(
        iClk                    => csi_c0_clock,
        iRst                    => rsi_r0_reset,
        iSync                   => syncSig,
        iIrqSource              => irqSourceSet,
        oIrq                    => ins_irqOut_irq,
        iIrqMasterEnable        => irqMasterEnable,
        iIrqSourceEnable        => irqSourceEnable,
        iIrqAcknowledge         => irqAcknowledge,
        oIrgPending             => irqSourcePending
        );


    --! The Status-/Control Registers
    theStCtrlReg : statusControlReg
    generic map(
        gMagic                  => cMagic,
        gVersionMajor           => gVersionMajor,
        gVersionMinor           => gVersionMinor,
        gVersionRevision        => gVersionRevision,
        gVersionCount           => gVersionCount,
        gHostBaseSet            => cBaseAddressHostCount,
        gPcpBaseSet             => cBaseAddressPcpCount,
        gIrqSourceCount         => cIrqSourceCount
        )
    port map(
        iClk                    => csi_c0_clock,
        iRst                    => rsi_r0_reset,
        iHostRead               => statCtrlRead,
        iHostWrite              => statCtrlWrite,
        iHostByteenable         => avs_host_byteenable,
        iHostAddress            => avs_host_address(10 downto 2),
        oHostReaddata           => statCtrlReaddata,
        iHostWritedata          => avs_host_writedata,
        oHostWaitrequest        => statCtrlWaitrequest,
        iPcpRead                => avs_pcp_read,
        iPcpWrite               => avs_pcp_write,
        iPcpByteenable          => avs_pcp_byteenable,
        iPcpAddress             => avs_pcp_address,
        oPcpReaddata            => avs_pcp_readdata,
        iPcpWritedata           => avs_pcp_writedata,
        oPcpWaitrequest         => avs_pcp_waitrequest,
        oBaseSetWrite           => baseSetWrite,
        oBaseSetAddress         => baseSetAddress,
        iBaseSetData            => baseSetReaddata,
        oBaseSetData            => baseSetWritedata,
        oIrqMasterEnable        => irqMasterEnable,
        oIrqSourceEnable        => irqSourceEnable,
        oIrqAcknowledge         => irqAcknowledge,
        oIrqSet                 => irqSourceSet,
        iIrqPending             => irqSourcePending,
        oExtSyncEnable          => extSyncEnable,
        oExtSyncConfig          => extSyncConfig,
        iNodeId                 => coe_NodeId_nodeid,
        oPLed                   => statCtrlLed,
        oBridgeEnable           => bridgeEnable
        );

    coe_PlkLed_ledst <= statCtrlLed(0);

    coe_PlkLed_lederr <= statCtrlLed(1);

end Rtl;
