-------------------------------------------------------------------------------
--! @file statusControlReg.vhd
--
--! @brief Host interface Status-/Control Registers
--
--! @details The host interface status/control registers provide memory mapped
--! control of the interrupt generator (irqGen) and bridge (magicBridge).
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

entity statusControlReg is
    generic (
        --! Magic
        gMagic              : natural := 16#504C4B00#;
        -- Version
        --! Version major
        gVersionMajor       : natural := 16#FF#;
        --! Version minor
        gVersionMinor       : natural := 16#FF#;
        --! Version revision
        gVersionRevision    : natural := 16#FF#;
        --! Version count pattern
        gVersionCount       : natural := 0;
        -- BaseSets
        --! BaseSets by Host
        gHostBaseSet        : natural := 2;
        --! BaseSets by Pcp
        gPcpBaseSet         : natural := 10;
        --! Interrupt source number
        gIrqSourceCount     : natural range 1 to 15 := 3
        );
    port (
        -- Global
        --! component wide clock signal
        iClk                : in std_logic;
        --! component wide reset signal
        iRst                : in std_logic;
        -- slave Host interface
        --! host read
        iHostRead           : in std_logic;
        --! host write
        iHostWrite          : in std_logic;
        --! host byteenable
        iHostByteenable     : in std_logic_vector(cDword/8-1 downto 0);
        --! host address
        iHostAddress        : in std_logic_vector(10 downto 2);
        --! host readdata
        oHostReaddata       : out std_logic_vector(cDword-1 downto 0);
        --! host writedata
        iHostWritedata      : in std_logic_vector(cDword-1 downto 0);
        --! host waitrequest
        oHostWaitrequest    : out std_logic;
        -- slave PCP interface
        --! pcp read
        iPcpRead            : in std_logic;
        --! pcp write
        iPcpWrite           : in std_logic;
        --! pcp byteenable
        iPcpByteenable      : in std_logic_vector(cDword/8-1 downto 0);
        --! pcp address
        iPcpAddress         : in std_logic_vector(10 downto 2);
        --! pcp readdata
        oPcpReaddata        : out std_logic_vector(cDword-1 downto 0);
        --! pcp writedata
        iPcpWritedata       : in std_logic_vector(cDword-1 downto 0);
        --! pcp waitrequest
        oPcpWaitrequest     : out std_logic;
        -- BaseSet link
        --! BaseSet write strobe
        oBaseSetWrite       : out std_logic;
        --! BaseSet read strobe
        oBaseSetRead        : out std_logic;
        --! BaseSet byteenable
        oBaseSetByteenable  : out std_logic_vector;
        --! BaseSet address bus
        oBaseSetAddress     : out std_logic_vector(LogDualis(gHostBaseSet+gPcpBaseSet)+2-1 downto 2);
        --! BaseSet read data bus
        iBaseSetData        : in std_logic_vector;
        --! BaseSet write data bus
        oBaseSetData        : out std_logic_vector;
        --! BaseSet acknowledge
        iBaseSetAck         : in std_logic;
        -- Interrupt control
        --! master enable
        oIrqMasterEnable    : out std_logic;
        --! interrupt source enable vector ('right is sync)
        oIrqSourceEnable    : out std_logic_vector(gIrqSourceCount downto 0);
        --! interrupt acknowledge (pulse, 'right is sync)
        oIrqAcknowledge     : out std_logic_vector(gIrqSourceCount downto 0);
        --! interrup set (pulse, no sync!)
        oIrqSet             : out std_logic_vector(gIrqSourceCount downto 1);
        --! interrupt source pending
        iIrqPending         : in std_logic_vector(gIrqSourceCount downto 0);
        --! external sync source enable
        oExtSyncEnable      : out std_logic;
        --! external sync source config
        oExtSyncConfig      : out std_logic_vector(cExtSyncEdgeConfigWidth-1 downto 0);
        -- miscellaneous
        --! Node Id
        iNodeId             : in std_logic_vector(cByte-1 downto 0);
        --! LED
        oPLed               : out std_logic_vector(1 downto 0);
        --! bridge activates
        oBridgeEnable       : out std_logic
        );
end statusControlReg;

architecture Rtl of statusControlReg is
    -- base for register content
    --! magic base
    constant cBaseMagic             : natural :=    16#0000#;
    --! version base
    constant cBaseVersion           : natural :=    16#0004#;
    --! boot base
    constant cBaseBootBase          : natural :=    16#0008#;
    --! init base
    constant cBaseInitBase          : natural :=    16#000C#;
    --! bridge enable base
    constant cBaseBridgeEnable      : natural :=    16#0200#;
    --! command base
    constant cBaseCommand           : natural :=    16#0204#;
    --! state base
    constant cBaseState             : natural :=    16#0206#;
    --! error base
    constant cBaseError             : natural :=    16#0208#;
    --! heart beat
    constant cBaseHeartBeat         : natural :=    16#020A#;
    --! node id in base
    constant cBaseNodeIdIn          : natural :=    16#020C#;
    --! led control base
    constant cBaseLedControl        : natural :=    16#0210#;
    --! irq enable base
    constant cBaseIrqEnable         : natural :=    16#0300#;
    --! irq pending base
    constant cBaseIrqPending        : natural :=    16#0302#;
    --! irq master enable base
    constant cBaseIrqMasterEnable   : natural :=    16#0304#;
    --! irq ack base (host only)
    constant cBaseIrqAck            : natural :=    16#0306#;
    --! irq set base (pcp only)
    constant cBaseIrqSet            : natural :=    16#0306#;
    --! sync config base
    constant cBaseSyncConfig        : natural :=    16#030C#;
    --! base for base set
    constant cBaseBaseSet           : natural :=    16#0400#;
    --! base reserved
    constant cBaseReserved          : natural :=    16#0500#;

    --! LED count
    constant cLedCount              : natural range 1 to 16 := 2;
    --! General Purpose Inputs

    --! type base registers (stored content)
    type tRegisterInfo is record
        --magic
        --version
        bootBase    : std_logic_vector(cDword-1 downto 0);
        initBase    : std_logic_vector(cDword-1 downto 0);
    end record;

    --! type control register (stored content)
    type tRegisterControl is record
        bridgeEnable    : std_logic;
        command         : std_logic_vector(cWord-1 downto 0);
        state           : std_logic_vector(cWord-1 downto 0);
        error           : std_logic_vector(cWord-1 downto 0);
        heartBeat       : std_logic_vector(cWord-1 downto 0);
        led             : std_logic_vector(cLedCount-1 downto 0);
    end record;

    --! type synchronization register (stored content)
    type tRegisterSynchronization is record
        irqSrcEnableHost    : std_logic_vector(gIrqSourceCount downto 0);
        irqSrcEnablePcp     : std_logic_vector(gIrqSourceCount downto 0);
        irqMasterEnable     : std_logic;
        syncConfig          : std_logic_vector(cExtSyncConfigWidth-1 downto 0);
    end record;

    --! info register
    signal regInfo, regInfo_next : tRegisterInfo;

    --! info register initialisation
    constant cRegInfoInit : tRegisterInfo := (
        bootBase => (others => cInactivated),
        initBase => (others => cInactivated)
    );

    --! control register
    signal regControl       : tRegisterControl;
    --! control register next
    signal regControl_next  : tRegisterControl;

    --! control register initialisation
    constant cRegControlInit : tRegisterControl := (
        bridgeEnable    => cInactivated,
        command         => (others => cInactivated),
        state           => (others => cInactivated),
        error           => (others => cInactivated),
        heartBeat       => (others => cInactivated),
        led             => (others => cInactivated)
    );

    --! synchronization register
    signal regSynchron      : tRegisterSynchronization;
    --! synchronization register next
    signal regSynchron_next : tRegisterSynchronization;

    --! synchronization register initialisation
    constant cRegSynchronInit : tRegisterSynchronization := (
        irqSrcEnableHost    => (others => cInactivated),
        irqSrcEnablePcp     => (others => cInactivated),
        irqMasterEnable     => cInactivated,
        syncConfig          => (others => cInactivated)
    );

    --! host base writedata
    signal hostBaseSetData  : std_logic_vector(iBaseSetData'range);
    --! host base write
    signal hostBaseSetWrite : std_logic;
    --! host base read
    signal hostBaseSetRead  : std_logic;
    --! pcp base writedata
    signal pcpBaseSetData   : std_logic_vector(iBaseSetData'range);
    --! pcp base write
    signal pcpBaseSetWrite  : std_logic;
    --! pcp base read
    signal pcpBaseSetRead   : std_logic;
begin

    --! register process creates storage of values
    regClk : process(iClk)
    begin
        if rising_edge(iClk) then
            if iRst = cActivated then
                regInfo     <= cRegInfoInit;
                regControl  <= cRegControlInit;
                regSynchron <= cRegSynchronInit;
            else
                regInfo     <= regInfo_next;
                regControl  <= regControl_next;
                regSynchron <= regSynchron_next;
            end if;
        end if;
    end process;

    oHostWaitrequest    <=  not iBaseSetAck when (hostBaseSetRead = cActivated or
                                            hostBaseSetWrite = cActivated) else
                            not(iHostWrite or iHostRead);

    oPcpWaitrequest     <=  not iBaseSetAck when (pcpBaseSetRead = cActivated or
                                            pcpBaseSetWrite = cActivated) else
                            not(iPcpWrite or iPcpRead);

    oIrqMasterEnable    <= regSynchron.irqMasterEnable;
    oIrqSourceEnable    <= regSynchron.irqSrcEnableHost and regSynchron.irqSrcEnablePcp;
    oExtSyncEnable      <= regSynchron.syncConfig(0);
    oExtSyncConfig      <= regSynchron.syncConfig(2 downto 1);
    oPLed               <= regControl.led;
    oBridgeEnable       <= regControl.bridgeEnable;

    -- pcp overrules host!
    oBaseSetData <= pcpBaseSetData when pcpBaseSetWrite = cActivated else
                    pcpBaseSetData when pcpBaseSetRead = cActivated else
                    hostBaseSetData;

    oBaseSetByteenable <=   iPcpByteenable when pcpBaseSetWrite = cActivated else
                            iPcpByteenable when pcpBaseSetRead = cActivated else
                            iHostByteenable;

    oBaseSetAddress <=  std_logic_vector(unsigned(iPcpAddress(oBaseSetAddress'range))+gHostBaseSet)
                            when pcpBaseSetRead = cActivated or pcpBaseSetWrite = cActivated else
                        iHostAddress(oBaseSetAddress'range);

    oBaseSetWrite   <= pcpBaseSetWrite or hostBaseSetWrite;
    oBaseSetRead    <= pcpBaseSetRead or hostBaseSetRead;

    --! register access
    regAcc : process (
        iHostWrite,
        iHostByteenable,
        iHostAddress,
        iHostWritedata,
        iPcpWrite,
        iPcpByteenable,
        iPcpAddress,
        iPcpWritedata,
        iNodeId,
        regInfo,
        regControl,
        regSynchron,
        iIrqPending,
        iBaseSetData
    )
        variable vHostSelAddr   : natural;
        variable vPcpSelAddr    : natural;
    begin
        -- default
        -- registers
        regInfo_next        <= regInfo;
        regControl_next     <= regControl;
        regSynchron_next    <= regSynchron;
        -- outputs
        oHostReaddata       <= (others => cInactivated);
        oIrqAcknowledge     <= (others => cInactivated);
        hostBaseSetData     <= (others => cInactivated);
        hostBaseSetWrite    <= cInactivated;
        hostBaseSetRead     <= cInactivated;
        oIrqSet             <= (others => cInactivated);
        oPcpReaddata        <= (others => cInactivated);
        pcpBaseSetData      <= (others => cInactivated);
        pcpBaseSetWrite     <= cInactivated;
        pcpBaseSetRead      <= cInactivated;

        -- HOST
        -- select content
        -- write to content
        -- and read from content
        vHostSelAddr := to_integer(unsigned(iHostAddress))*4;

        case vHostSelAddr is
            when cBaseMagic =>
                oHostReaddata <= std_logic_vector(to_unsigned(gMagic, cDword));

                --magic is RO

            when cBaseVersion =>
                oHostReaddata <=
                    std_logic_vector(to_unsigned(gVersionMajor, cByte)) &
                    std_logic_vector(to_unsigned(gVersionMinor, cByte)) &
                    std_logic_vector(to_unsigned(gVersionRevision, cByte)) &
                    std_logic_vector(to_unsigned(gVersionCount, cByte));

                --version is RO

            when cBaseBootBase =>
                oHostReaddata <= regInfo.bootBase;

                --bootBase is RO

            when cBaseInitBase =>
                oHostReaddata <= regInfo.initBase;

                --initBase is RO

            when cBaseBridgeEnable =>
                oHostReaddata(0) <= regControl.bridgeEnable;

                --bridge enable is RO

            when cBaseState | cBaseCommand =>
                oHostReaddata <= regControl.state & regControl.command;

                if iHostWrite = cActivated then
                    --state is RO

                    if iHostByteenable(1) = cActivated then
                        regControl_next.command(cWord-1 downto cByte) <= iHostWritedata(cWord-1 downto cByte);
                    end if;

                    if iHostByteenable(0) = cActivated then
                        regControl_next.command(cByte-1 downto 0) <= iHostWritedata(cByte-1 downto 0);
                    end if;
                end if;

            when cBaseHeartBeat | cBaseError =>
                oHostReaddata <= regControl.heartBeat & regControl.error;

                --heartbeat and error are RO

            when cBaseNodeIdIn =>
                oHostReaddata(iNodeId'length-1 downto 0) <= iNodeId;

                --node id are RO

            when cBaseLedControl =>
                oHostReaddata(cLedCount-1 downto 0) <= regControl.led;

                if iHostWrite = cActivated then
                    for i in cWord-1 downto 0 loop
                        if iHostByteenable(i/cByte) = cActivated and
                            i < cLedCount then
                            regControl_next.led(i) <= iHostWritedata(i);
                        end if;
                    end loop;
                end if;

            when cBaseIrqPending | cBaseIrqEnable =>
                oHostReaddata(cWord+gIrqSourceCount downto cWord) <= iIrqPending;

                oHostReaddata(gIrqSourceCount downto 0) <= regSynchron.irqSrcEnableHost;

                if iHostWrite = cActivated then
                    for i in cWord-1 downto 0 loop
                        if iHostByteenable(i/cByte) = cActivated and
                            i <= gIrqSourceCount then
                            regSynchron_next.irqSrcEnableHost(i) <= iHostWritedata(i);
                        end if;
                    end loop;
                end if;

            when cBaseIrqAck | cBaseIrqMasterEnable =>
                -- irq ack is SC
                oHostReaddata(0) <= regSynchron.irqMasterEnable;

                if iHostWrite = cActivated then
                    if iHostByteenable(0) = cActivated then
                        regSynchron_next.irqMasterEnable <= iHostWritedata(0);
                    end if;

                    for i in cDword-1 downto cWord loop
                        if iHostByteenable(i/cByte) = cActivated and
                            (i-cWord) <= gIrqSourceCount then
                            oIrqAcknowledge(i-cWord) <= iHostWritedata(i);
                        end if;
                    end loop;
                end if;

            when cBaseSyncConfig =>
                oHostReaddata(cExtSyncConfigWidth-1 downto 0) <= regSynchron.syncConfig;

                if iHostWrite = cActivated then
                    for i in cWord-1 downto 0 loop
                        if iHostByteenable(i/cByte) = cActivated and
                            i < cExtSyncConfigWidth then
                            regSynchron_next.syncConfig(i) <= iHostWritedata(i);
                        end if;
                    end loop;
                end if;

            when cBaseBaseSet to cBaseReserved-1 =>
                if vHostSelAddr < cBaseBaseSet+gHostBaseSet*cDword/cByte then
                    oHostReaddata(iBaseSetData'range) <= iBaseSetData;

                    if iHostWrite = cActivated then
                        hostBaseSetData     <= iHostWritedata(hostBaseSetData'range);
                        hostBaseSetWrite    <= cActivated;
                    else
                        hostBaseSetRead     <= cActivated;
                    end if;
                end if;

            when others => null;
        end case;

        -- PCP
        -- select content
        -- write to content
        -- and read from content
        vPcpSelAddr := to_integer(unsigned(iPcpAddress)) * 4;

        case vPcpSelAddr is
            when cBaseMagic =>
                oPcpReaddata <= std_logic_vector(to_unsigned(gMagic, cDword));

                --magic is RO

            when cBaseVersion =>
                oPcpReaddata <=
                    std_logic_vector(to_unsigned(gVersionMajor, cByte)) &
                    std_logic_vector(to_unsigned(gVersionMinor, cByte)) &
                    std_logic_vector(to_unsigned(gVersionRevision, cByte)) &
                    std_logic_vector(to_unsigned(gVersionCount, cByte));

                --version is RO

            when cBaseBootBase =>
                oPcpReaddata <= regInfo.bootBase;

                if iPcpWrite = cActivated then
                    for i in cDword-1 downto 0 loop
                        if iPcpByteenable(i/cByte) = cActivated then
                            regInfo_next.bootBase(i) <= iPcpWritedata(i);
                        end if;
                    end loop;
                end if;

            when cBaseInitBase =>
                oPcpReaddata <= regInfo.initBase;

                if iPcpWrite = cActivated then
                    for i in cDword-1 downto 0 loop
                        if iPcpByteenable(i/cByte) = cActivated then
                            regInfo_next.initBase(i) <= iPcpWritedata(i);
                        end if;
                    end loop;
                end if;

            when cBaseBridgeEnable =>
                oPcpReaddata(0) <= regControl.bridgeEnable;

                if iPcpWrite = cActivated then
                    regControl_next.bridgeEnable <= iPcpWritedata(0);
                end if;

            when cBaseState | cBaseCommand =>
                oPcpReaddata <= regControl.state & regControl.command;

                if iPcpWrite = cActivated then
                    for i in cDword-1 downto cWord loop
                        if iPcpByteenable(i/cByte) = cActivated then
                            regControl_next.state(i-cWord) <= iPcpWritedata(i);
                        end if;
                    end loop;
                    for i in cWord-1 downto 0 loop
                        if iPcpByteenable(i/cByte) = cActivated then
                            regControl_next.command(i) <= iPcpWritedata(i);
                        end if;
                    end loop;
                end if;

            when cBaseHeartBeat | cBaseError =>
                oPcpReaddata <= regControl.heartBeat & regControl.error;

                if iPcpWrite = cActivated then
                    for i in cDword-1 downto cWord loop
                        if iPcpByteenable(i/cByte) = cActivated then
                            regControl_next.heartBeat(i-cWord) <= iPcpWritedata(i);
                        end if;
                    end loop;
                    for i in cWord-1 downto 0 loop
                        if iPcpByteenable(i/cByte) = cActivated then
                            regControl_next.error(i) <= iPcpWritedata(i);
                        end if;
                    end loop;
                end if;

            when cBaseNodeIdIn =>
                oPcpReaddata(iNodeId'length-1 downto 0) <= iNodeId;

            when cBaseLedControl =>
                oPcpReaddata(cLedCount-1 downto 0) <= regControl.led;

            when cBaseIrqPending | cBaseIrqEnable =>
                oPcpReaddata(cWord+gIrqSourceCount downto cWord) <= iIrqPending;
                oPcpReaddata(gIrqSourceCount downto 0) <= regSynchron.irqSrcEnablePcp;

                if iPcpWrite = cActivated then
                    for i in cWord-1 downto 0 loop
                        if iPcpByteenable(i/cByte) = cActivated and
                            i <= gIrqSourceCount then
                            regSynchron_next.irqSrcEnablePcp(i) <= iPcpWritedata(i);
                        end if;
                    end loop;
                end if;

            when cBaseIrqSet | cBaseIrqMasterEnable =>
                -- irq set is self-clearing

                oPcpReaddata(0) <= regSynchron.irqMasterEnable;

                if iPcpWrite = cActivated then
                    for i in cDword-1 downto cWord+1 loop
                        if iPcpByteenable(i/cByte) = cActivated and
                            (i-cWord) <= gIrqSourceCount then
                            oIrqSet(i-cWord) <= iPcpWritedata(i);
                        end if;
                    end loop;
                end if;

            when cBaseSyncConfig =>
                oPcpReaddata(cExtSyncConfigWidth-1 downto 0) <=
                regSynchron.syncConfig;

            when cBaseBaseSet to cBaseReserved-1 =>
                if vPcpSelAddr < cBaseBaseSet+gPcpBaseSet*cDword/cByte then
                    oPcpReaddata(iBaseSetData'range) <= iBaseSetData;

                    if iPcpWrite = cActivated then
                        pcpBaseSetData  <= iPcpWritedata(pcpBaseSetData'range);
                        pcpBaseSetWrite <= cActivated;
                    else
                        pcpBaseSetRead <= cActivated;
                    end if;
                end if;

            when others => null;
        end case;
    end process;
end Rtl;
