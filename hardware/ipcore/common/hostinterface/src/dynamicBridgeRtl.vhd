-------------------------------------------------------------------------------
--! @file dynamicBridgeRtl.vhd
--
--! @brief Dynamic Bridge for translating static to dynamic memory spaces
--
--! @details The Dynamic Bridge component translates a static memory mapping
--! into a dynamic memory map that can be changed during runtime.
--! This enhances the functionality of an ordinary memory mapped bridge logic.
--! Additionally several memory spaces can be configured (compilation).
-------------------------------------------------------------------------------
--
--    (c) B&R Industrial Automation GmbH, 2014
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
--! need reduce or operation
use ieee.std_logic_misc.OR_REDUCE;

--! Common library
library libcommon;
--! Use common library global package
use libcommon.global.all;

--! Work library
library work;
--! use host interface package for specific types
use work.hostInterfacePkg.all;

-------------------------------------------------------------------------------
--! @brief Dynamic bridge translates a fixed address space into a flexible one.
-------------------------------------------------------------------------------
--! @details
--! The dynamic bridge has an input port with a fixed (before runtime) memory
--! mapping, which is translated to a flexible address space mapping.
--! The flexible address space can be changed during runtime in order to
--! redirect accesses from the input to other locations.
--! The base addresses for the static port are set by generic gBaseAddressArray.
--! The base addresses for the dynamic port are set through the BaseSet port
-------------------------------------------------------------------------------
entity dynamicBridge is
    generic (
        --! number of static address spaces
        gAddressSpaceCount      : natural := 2;
        --! select wheather DPRAM or registers will be used as memory (false = 0, true /= 0)
        gUseMemBlock            : integer := 0;
        --! base addresses in static address space (note: last-1 = high address)
        gBaseAddressArray       : tArrayStd32 := (x"0000_1000" , x"0000_2000" , x"0000_3000")
    );
    port (
        -- Global
        --! component-wide clock signal
        iClk                : in    std_logic;
        --! component-wide reset signal
        iRst                : in    std_logic;
        -- Bridge
        --! address of static address space
        iBridgeAddress      : in    std_logic_vector;
        --! Request strobe
        iBridgeRequest      : in    std_logic;
        --! address of dynamic address space (translated input)
        oBridgeAddress      : out   std_logic_vector;
        --! Select signal of any address space selected
        oBridgeSelectAny    : out   std_logic;
        --! select signals of all address spaces
        oBridgeSelect       : out   std_logic_vector(gAddressSpaceCount-1 downto 0);
        --! Bridge output valid
        oBridgeValid        : out   std_logic;
        -- BaseSet Memory Mapped Write Bus
        --! BaseSet write strobe
        iBaseSetWrite       : in    std_logic;
        --! BaseSet read strobe
        iBaseSetRead        : in    std_logic;
        --! BaseSet byteenable
        iBaseSetByteenable  : in    std_logic_vector;
        --! BaseSet address bus
        iBaseSetAddress     : in    std_logic_vector(LogDualis(gAddressSpaceCount)-1 downto 0);
        --! BaseSet write data bus
        iBaseSetData        : in    std_logic_vector;
        --! BaseSet read data bus
        oBaseSetData        : out   std_logic_vector;
        --! BaseSet acknowledge
        oBaseSetAck         : out   std_logic
    );
end dynamicBridge;

-------------------------------------------------------------------------------
--! @brief Register Transfer Level of Dynamic Bridge device
-------------------------------------------------------------------------------
--! @details
--! The dynamic bridge rtl applies generated address decoders
--! to generate select signals for the static address spaces.
--! The select signals are forwarded register file holding the base address
--! offsets in the dynamic memory space. The input address is manipulated with
--! arithmetic operators to gain the output address. The lut file holds the
--! base addresses in the static memory space.
-------------------------------------------------------------------------------
architecture rtl of dynamicBridge is
    --! Bridge cycle delay
    constant cBridgeCycleDelay  : natural := 3;
    --! Bridge read path enable all bytes
    constant cByteenableAllOnes : std_logic_vector(iBaseSetByteenable'range) :=
        (others => cActivated);

    --! convert address array into stream
    constant cBaseAddressArrayStd   : std_logic_vector
        ((gAddressSpaceCount+1)*cArrayStd32ElementSize-1 downto 0) :=
        CONV_STDLOGICVECTOR(gBaseAddressArray, gBaseAddressArray'length);

    --! Input address register
    signal inAddrReg                : std_logic_vector(iBridgeAddress'range);
    --! Request rising edge
    signal bridgeRequest_rising     : std_logic;
    --! Input address store strobe
    signal inAddrStore              : std_logic;
    --! Request acknowledge terminal count strobe
    signal reqAckTcnt               : std_logic;
    --! address decoder select signals one hot coded
    signal addrDecSelOneHot         : std_logic_vector(gAddressSpaceCount-1 downto 0);
    --! address decoder select signals binary coded
    signal addrDecSelBinary         : std_logic_vector(LogDualis(gAddressSpaceCount)-1 downto 0);
    --! selected static lut file base offset
    signal lutFileBase              : std_logic_vector(inAddrReg'range);
    --! Base address in bridge master environment
    signal addrSpaceOffset          : std_logic_vector(inAddrReg'range);
    --! Base address in bridge master environment (unregistered)
    signal addrSpaceOffset_unreg    : std_logic_vector(inAddrReg'range);
    --! Dynamic address offset within selected static space
    signal dynamicOffset         : std_logic_vector(iBaseSetData'range);
    --! Translated address
    signal translateAddress         : std_logic_vector(maximum(iBaseSetData'high, oBridgeAddress'high) downto inAddrReg'low);
begin
    -- assert
    assert (cBridgeCycleDelay > 0)
        report "Set cBridgeCycleDelay > 0"
        severity failure;

    -- export
    oBridgeSelect       <= addrDecSelOneHot;
    oBridgeSelectAny    <= OR_REDUCE(addrDecSelOneHot);
    oBridgeValid        <= reqAckTcnt;
    oBridgeAddress      <= translateAddress(oBridgeAddress'range);

    --! Store the input address with the request strobe
    storeAddr : process(iRst, iClk)
    begin
        if iRst = cActivated then
            inAddrReg <= (others => cInactivated);
        elsif rising_edge(iClk) then
            if inAddrStore = cActivated then
                inAddrReg <= iBridgeAddress;
            end if;
        end if;
    end process;

    --! Get rising edge of request signaling
    reqEdge : entity libcommon.edgedetector
        port map (
            iArst       => iRst,
            iClk        => iClk,
            iEnable     => cActivated,
            iData       => iBridgeRequest,
            oRising     => bridgeRequest_rising,
            oFalling    => open,
            oAny        => open
        );

    inAddrStore <= bridgeRequest_rising;

    --! Generate the request acknowledge signal
    reqAck : entity libcommon.cnt
        generic map (
            gCntWidth   => logDualis(cBridgeCycleDelay+1),
            gTcntVal    => cBridgeCycleDelay
        )
        port map (
            iArst   => iRst,
            iClk    => iClk,
            iEnable => iBridgeRequest,
            iSrst   => cInactivated,
            oCnt    => open,
            oTcnt   => reqAckTcnt
        );

    --! Generate Address Decoders
    genAddressDecoder : for i in 0 to gAddressSpaceCount-1 generate
        insAddressDecoder : entity libcommon.addrDecode
        generic map (
            gAddrWidth  => inAddrReg'length,
            gBaseAddr   => to_integer(unsigned(gBaseAddressArray(i)(inAddrReg'range))),
            gHighAddr   => to_integer(unsigned(gBaseAddressArray(i+1)(inAddrReg'range))-1)
        )
        port map (
            iEnable     => iBridgeRequest,
            iAddress    => inAddrReg,
            oSelect     => addrDecSelOneHot(i)
        );
    end generate;

    --! Convert one hot from address decoder to binary
    insBinaryEncoder : entity libcommon.binaryEncoder
    generic map (
        gDataWidth => gAddressSpaceCount
    )
    port map (
        iOneHot => addrDecSelOneHot,
        oBinary => addrDecSelBinary
    );

    --! select static base address in lut file
    insLutFile : entity libcommon.lutFile
    generic map (
        gLutCount       => gAddressSpaceCount,
        gLutWidth       => cArrayStd32ElementSize,
        gLutInitValue   => cBaseAddressArrayStd(cBaseAddressArrayStd'left downto cArrayStd32ElementSize)
        -- omit high address of last memory map
    )
    port map (
        iAddrRead   => addrDecSelBinary,
        oData       => lutFileBase
    );

    -- calculate address offset within static space
    addrSpaceOffset_unreg <= std_logic_vector(
        unsigned(inAddrReg) - unsigned(lutFileBase)
    );

    --! Registers to break combinational path of lut file output.
    regAddrSpace : process(iRst, iClk)
    begin
        if iRst = cActivated then
            addrSpaceOffset <= (others => cInactivated);
        elsif rising_edge(iClk) then
            addrSpaceOffset <= addrSpaceOffset_unreg;
        end if;
    end process;

    REGFILE : if gUseMemBlock = 0 generate
        signal dynamicOffset_unreg : std_logic_vector(dynamicOffset'range);
    begin
        --! select dynamic base address in register file
        insRegFile : entity libcommon.registerFile
        generic map (
            gRegCount => gAddressSpaceCount
        )
        port map (
            iClk            => iClk,
            iRst            => iRst,
            iWriteA         => iBaseSetWrite,
            iWriteB         => cInactivated, -- write port B unused
            iByteenableA    => iBaseSetByteenable,
            iByteenableB    => cByteenableAllOnes,
            iAddrA          => iBaseSetAddress,
            iAddrB          => addrDecSelBinary,
            iWritedataA     => iBaseSetData,
            oReaddataA      => oBaseSetData,
            iWritedataB     => iBaseSetData, -- write port B unused
            oReaddataB      => dynamicOffset_unreg
        );

        regDynOff : process(iRst, iClk)
        begin
            if iRst = cActivated then
                dynamicOffset <= (others => cInactivated);
            elsif rising_edge(iClk) then
                dynamicOffset <= dynamicOffset_unreg;
            end if;
        end process;

        BASESETACK : oBaseSetAck <= iBaseSetWrite or iBaseSetRead;
    end generate REGFILE;

    genDPRAM : if gUseMemBlock /= 0 generate
        -- Clip dpr word width to values of power 2 (e.g. 8, 16, 32)
        constant cDprWordWidth  : natural := 2**logDualis(iBaseSetData'length);
        constant cDprAddrWidth  : natural := logDualis(gAddressSpaceCount);
        constant cDprReadDel    : natural := 1;

        type tDprPort is record
            write       : std_logic;
            read        : std_logic;
            address     : std_logic_vector(cDprAddrWidth-1 downto 0);
            byteenable  : std_logic_vector(cDprWordWidth/8-1 downto 0);
            writedata   : std_logic_vector(cDprWordWidth-1 downto 0);
            readdata    : std_logic_vector(cDprWordWidth-1 downto 0);
        end record;

        signal dprPortA         : tDprPort;
        signal dprPortA_readAck : std_logic;
        signal dprPortB         : tDprPort;
    begin
        --! This combinatoric process assigns base sets to the dpr.
        --! The default assignments avoid warnings in synthesize tools.
        dprAssign : process (
            iBaseSetByteenable,
            iBaseSetData
        )
        begin
            --default assignments
            dprPortA.byteenable <= (others => cInactivated);
            dprPortA.writedata  <= (others => cInactivated);
            dprPortB.byteenable <= (others => cInactivated);
            dprPortB.writedata  <= (others => cInactivated);

            dprPortA.byteenable(iBaseSetByteenable'range)   <= iBaseSetByteenable;
            dprPortA.writedata(iBaseSetData'range)          <= iBaseSetData;
            dprPortB.byteenable                             <= (others => cInactivated);
            dprPortB.writedata(iBaseSetData'range)          <= iBaseSetData;
        end process;

        dprPortA.write      <= iBaseSetWrite;
        dprPortA.read       <= iBaseSetRead;
        dprPortA.address    <= iBaseSetAddress;
        oBaseSetData        <= dprPortA.readdata(oBaseSetData'range);

        dprPortB.write      <= cInactivated; --unused
        dprPortB.read       <= cActivated; --unused
        dprPortB.address    <= addrDecSelBinary;
        dynamicOffset       <= dprPortB.readdata(dynamicOffset'range);

        insDPRAM: entity work.dpRam
        generic map(
            gWordWidth         => cDprWordWidth,
            gNumberOfWords     => gAddressSpaceCount
        )
        port map(
            iClk_A             => iClk,
            iEnable_A          => cActivated,
            iWriteEnable_A     => dprPortA.write,
            iAddress_A         => dprPortA.address,
            iByteEnable_A      => dprPortA.byteenable,
            iWritedata_A       => dprPortA.writedata,
            oReaddata_A        => dprPortA.readdata,
            iClk_B             => iClk,
            iEnable_B          => cActivated,
            iWriteEnable_B     => dprPortB.write,
            iAddress_B         => dprPortB.address,
            iByteEnable_B      => dprPortB.byteenable,
            iWritedata_B       => dprPortB.writedata,
            oReaddata_B        => dprPortB.readdata
        );

        BASESETACK : oBaseSetAck <= dprPortA.write or dprPortA_readAck;

        --! Generate the read acknowledge signal
        rdAck : entity libcommon.cnt
            generic map (
                gCntWidth   => logDualis(cDprReadDel+1),
                gTcntVal    => cDprReadDel
            )
            port map (
                iArst   => iRst,
                iClk    => iClk,
                iEnable => dprPortA.read,
                iSrst   => cInactivated,
                oCnt    => open,
                oTcnt   => dprPortA_readAck
            );
    end generate genDPRAM;

    -- calculate translated address offset in dynamic space
    translateAddress <= std_logic_vector(
        unsigned(dynamicOffset) + unsigned(addrSpaceOffset)
    );
end rtl;
