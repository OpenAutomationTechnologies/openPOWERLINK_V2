-------------------------------------------------------------------------------
--! @file magicBridgeRtl.vhd
--
--! @brief Magic Bridge for translating static to dynamic memory spaces
--
--! @details The Magic Bridge component translates a static memory mapping
--! into a dynamic memory map that can be changed during runtime.
--! This enhances the functionality of an ordinary memory mapped bridge logic.
--! Additionally several memory spaces can be configured (compilation).
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
--! need reduce or operation
use ieee.std_logic_misc.OR_REDUCE;
--! use global library
use work.global.all;
--! use host interface package for specific types
use work.hostInterfacePkg.all;

-------------------------------------------------------------------------------
--! @brief Magic bridge translates a fixed address space into a flexible one.
-------------------------------------------------------------------------------
--! @details
--! The magic bridge has an input port with a fixed (before runtime) memory
--! mapping, which is translated to a flexible address space mapping.
--! The flexible address space can be changed during runtime in order to
--! redirect accesses from the input to other locations.
--! The base addresses for the static port are set by generic gBaseAddressArray.
--! The base addresses for the dynamic port are set through the BaseSet port
-------------------------------------------------------------------------------
entity magicBridge is
    generic (
        --! number of static address spaces
        gAddressSpaceCount      : natural := 2;
        --! base addresses in static address space (note: last-1 = high address)
        gBaseAddressArray       : tArrayStd32 :=
        (x"0000_1000" , x"0000_2000" , x"0000_3000")
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
        --! select strobe
        iBridgeSelect       : in    std_logic;
        --! address of dynamic address space (translated input)
        oBridgeAddress      : out   std_logic_vector;
        --! select strobe
        oBridgeSelectAny    : out   std_logic;
        --! select signals of all address spaces
        oBridgeSelect       : out   std_logic_vector
        (gAddressSpaceCount-1 downto 0);
        -- BaseSet Memory Mapped Write Bus
        --! BaseSet write strob
        iBaseSetWrite       : in    std_logic;
        --! BaseSet byteenable
        iBaseSetByteenable  : in    std_logic_vector;
        --! BaseSet address bus
        iBaseSetAddress     : in    std_logic_vector
        (LogDualis(gAddressSpaceCount)-1 downto 0);
        --! BaseSet write data bus
        iBaseSetData        : in    std_logic_vector;
        --! BaseSer read data bus
        oBaseSetData        : out   std_logic_vector
        );
end magicBridge;

-------------------------------------------------------------------------------
--! @brief Register Transfer Level of Magic Bridge device
-------------------------------------------------------------------------------
--! @details
--! The magic bridge rtl applies generated address decoders
--! to generate select signals for the static address spaces.
--! The select signals are forwarded register file holding the base address
--! offsets in the dynamic memory space. The input address is manipulated with
--! arithmetic operators to gain the output address. The lut file holds the
--! base addresses in the static memory space.
-------------------------------------------------------------------------------
architecture Rtl of magicBridge is

    --! address decoder
    component addr_decoder
        generic(
            addrWidth_g : integer := 32;
            baseaddr_g : integer := 4096;
            highaddr_g : integer := 8191);
        port(
            selin : in std_logic;
            addr : in std_logic_vector(addrWidth_g-1 downto 0);
            selout : out std_logic);
    end component;

    --! binary encoder
    component binaryEncoder
        generic(
            gDataWidth : NATURAL := 8);
        port(
            iOneHot : in STD_LOGIC_VECTOR(gDataWidth-1 downto 0);
            oBinary : out STD_LOGIC_VECTOR(LogDualis(gDataWidth)-1 downto 0));
    end component;


    --! register file
    component registerFile
        generic(
            gRegCount : NATURAL := 8);
        port(
            iClk : in STD_LOGIC;
            iRst : in STD_LOGIC;
            iWriteA : in STD_LOGIC;
            iWriteB : in STD_LOGIC;
            iByteenableA:   in std_logic_vector;
            iByteenableB:   in std_logic_vector;
            iAddrA : in STD_LOGIC_VECTOR(LogDualis(gRegCount)-1 downto 0);
            iAddrB : in STD_LOGIC_VECTOR(LogDualis(gRegCount)-1 downto 0);
            iWritedataA : in STD_LOGIC_VECTOR;
            oReaddataA : out STD_LOGIC_VECTOR;
            iWritedataB : in STD_LOGIC_VECTOR;
            oReaddataB : out STD_LOGIC_VECTOR);
    end component;


    --lut file
    component lutFile
    generic(
        gLutCount : NATURAL := 4;
        gLutWidth : NATURAL := 32;
        gLutInitValue : STD_LOGIC_VECTOR := x"1111_1111"&x"2222_2222"&x"3333_3333"&x"4444_4444");
    port(
        iAddrRead : in STD_LOGIC_VECTOR(LogDualis(gLutCount)-1 downto 0);
        oData : out STD_LOGIC_VECTOR);
    end component;

    constant cByteenableAllZero : std_logic_vector(iBaseSetByteenable'range) :=
    (others => cInactivated);

    constant cBaseAddressArrayStd   : std_logic_vector
    ((gAddressSpaceCount+1)*cArrayStd32ElementSize-1 downto 0) :=
    CONV_STDLOGICVECTOR(gBaseAddressArray, gBaseAddressArray'length);
    --! convert address array into stream

    signal addrDecSelOneHot         : std_logic_vector
    (gAddressSpaceCount-1 downto 0);
    --! address decoder select signals one hot coded
    signal addrDecSelBinary         : std_logic_vector
    (LogDualis(gAddressSpaceCount)-1 downto 0);
    --! address decoder select signals binary coded
    signal lutFileBase              : std_logic_vector(iBridgeAddress'range);
    --! selected static lut file base offset
    signal addrSpaceOffset          : std_logic_vector(iBridgeAddress'range);
    --! address offset within selected static space
    signal registerFileBase         : std_logic_vector(oBridgeAddress'range);
    --! selected dynamic register file base offset
    signal translateAddress         : std_logic_vector
    (MAX(iBridgeAddress'high, oBridgeAddress'high) downto iBridgeAddress'low);

begin

    --! Generate Address Decoders
    genAddressDecoder : for i in 0 to gAddressSpaceCount-1 generate
        insAddressDecoder : addr_decoder
        generic map(
            addrWidth_g => iBridgeAddress'length,
            baseaddr_g => to_integer(
            unsigned(gBaseAddressArray(i)(iBridgeAddress'range))),
            highaddr_g => to_integer(
            unsigned(gBaseAddressArray(i+1)(iBridgeAddress'range))-1)
            )
        port map(
            selin => iBridgeSelect,
            addr => iBridgeAddress,
            selout => addrDecSelOneHot(i)
            );
    end generate;

    --! export one hot code
    oBridgeSelect <= addrDecSelOneHot;

    --! export or'd one hot code
    oBridgeSelectAny <= OR_REDUCE(addrDecSelOneHot);

    --! Convert one hot from address decoder to binary
    insBinaryEncoder : binaryEncoder
    generic map(
        gDataWidth => gAddressSpaceCount
        )
    port map(
        iOneHot => addrDecSelOneHot,
        oBinary => addrDecSelBinary
        );

    --! select static base address in lut file
    insLutFile : lutFile
    generic map(
        gLutCount => gAddressSpaceCount,
        gLutWidth => cArrayStd32ElementSize,
        gLutInitValue => cBaseAddressArrayStd
        (cBaseAddressArrayStd'left downto cArrayStd32ElementSize)
        --! omit high address of last memory map
        )
    port map(
        iAddrRead => addrDecSelBinary,
        oData => lutFileBase
        );

    --! calculate address offset within static space
    addrSpaceOffset <= std_logic_vector(
    unsigned(iBridgeAddress) - unsigned(lutFileBase));

    --! select dynamic base address in register file
    insRegFile : registerFile
    generic map(
        gRegCount => gAddressSpaceCount
        )
    port map(
        iClk => iClk,
        iRst => iRst,
        iWriteA => iBaseSetWrite,
        iWriteB => cInactivated, --! write port B unused
        iByteenableA => iBaseSetByteenable,
        iByteenableB => cByteenableAllZero,
        iAddrA => iBaseSetAddress,
        iAddrB => addrDecSelBinary,
        iWritedataA => iBaseSetData,
        oReaddataA => oBaseSetData,
        iWritedataB => iBaseSetData, --! write port B unused
        oReaddataB => registerFileBase
        );

    --! calculate translated address offset in dynamic space
    oBridgeAddress <= translateAddress(oBridgeAddress'range);
    translateAddress <= std_logic_vector(
    unsigned(registerFileBase) + unsigned(addrSpaceOffset));

end Rtl;
