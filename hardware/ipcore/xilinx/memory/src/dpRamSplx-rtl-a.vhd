--! @file dpRamSplx-rtl-a.vhd
--
--! @brief Simplex Dual Port Ram Register Transfer Level Architecture
--
--! @details This is the Simplex DPRAM intended for synthesis on Xilinx
--!          platforms only.
--!          Timing as follows [clk-cycles]: write=0 / read=1
--
-------------------------------------------------------------------------------
-- Architecture : rtl
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

--! Common library
library libcommon;
--! Use common library global package
use libcommon.global.all;

architecture rtl of dpRamSplx is
    --! Width configuration type
    type tWidthConfig is (
        sUnsupported,
        sAsym_16_32,
        sAsym_32_16,
        sSym
    );

    --! Function to return width configuration.
    function getWidthConfig (
        wordWidthA          : natural;
        byteEnableWidthA    : natural;
        wordWidthB          : natural
    ) return tWidthConfig is
    begin
        if wordWidthA = 16 and wordWidthB = 32 and byteEnableWidthA = 2 then
            return sAsym_16_32;
        elsif wordWidthA = 32 and wordWidthB = 16 and byteEnableWidthA = 4 then
            return sAsym_32_16;
        elsif wordWidthA = wordWidthB and byteEnableWidthA = wordWidthA/cByteLength then
            return sSym;
        else
            return sUnsupported;
        end if;
    end function;

    --! Width configuration
    constant cWidthConfig   : tWidthConfig := getWidthConfig(gWordWidthA, gByteenableWidthA, gWordWidthB);

    --! Words of dpram
    constant cDprWords      : natural := minimum(gNumberOfWordsA, gNumberOfWordsB);
    --! Word width of dpram
    constant cDprWordWidth  : natural := maximum(gWordWidthA, gWordWidthB);

    --! Dpr write port address
    signal writeAddress     : std_logic_vector(logDualis(cDprWords)-1 downto 0);
    --! Dpr write port enables
    signal writeByteenable  : std_logic_vector(cDprWordWidth/cByteLength-1 downto 0);
    --! Dpr write port
    signal writedata        : std_logic_vector(cDprWordWidth-1 downto 0);
    --! Dpr read port address
    signal readAddress      : std_logic_vector(logDualis(cDprWords)-1 downto 0);
    --! Dpr read port
    signal readdata         : std_logic_vector(cDprWordWidth-1 downto 0);
begin
    assert (cWidthConfig /= sUnsupported)
    report "The width configuration is not supported!"
    severity failure;

    assert (gInitFile = "UNUSED")
    report "Memory initialization is not supported in this architecture!"
    severity warning;

    assert (gWordWidthA*gNumberOfWordsA = gWordWidthB*gNumberOfWordsB)
    report "Memory size of port A and B are different!"
    severity failure;

    --! Instantiate true dual ported ram entity
    TRUEDPRAM : entity work.dpRam
        generic map (
            gWordWidth      => cDprWordWidth,
            gNumberOfWords  => cDprWords,
            gInitFile       => "UNUSED"
        )
        port map (
            iClk_A          => iClk_A,
            iEnable_A       => iEnable_A,
            iWriteEnable_A  => iWriteEnable_A,
            iAddress_A      => writeAddress,
            iByteenable_A   => writeByteenable,
            iWritedata_A    => writedata,
            oReaddata_A     => open,
            iClk_B          => iClk_B,
            iEnable_B       => iEnable_B,
            iWriteEnable_B  => cInactivated,
            iByteenable_B   => (others => cInactivated),
            iAddress_B      => readAddress,
            iWritedata_B    => (others => cInactivated),
            oReaddata_B     => readdata
        );

    --! This generate block assigns the 16 bit write port and
    --! the 32 bit read port.
    WIDTHCFG_16_32 : if cWidthConfig = sAsym_16_32 generate
        writeAddress        <= iAddress_A(iAddress_A'left downto 1);
        writeByteenable(3)  <= iByteenable_A(1) and iAddress_A(0);
        writeByteenable(2)  <= iByteenable_A(0) and iAddress_A(0);
        writeByteenable(1)  <= iByteenable_A(1) and not iAddress_A(0);
        writeByteenable(0)  <= iByteenable_A(0) and not iAddress_A(0);
        writedata           <= iWritedata_A & iWritedata_A;
        readAddress         <= iAddress_B;
        oReaddata_B         <= readdata;
    end generate WIDTHCFG_16_32;

    --! This generate block assigns the 32 bit write port and
    --! the 16 bit read port.
    WIDTHCFG_32_16 : if cWidthConfig = sAsym_32_16 generate
        writeAddress    <=  iAddress_A;
        writeByteenable <=  iByteenable_A;
        writedata       <=  iWritedata_A;
        readAddress     <=  iAddress_B(iAddress_B'left downto 1);
        oReaddata_B     <=  readdata(31 downto 16) when iAddress_B(0) = cActivated else
                            readdata(15 downto 0);
    end generate WIDTHCFG_32_16;

    --! This generate block assigns the symmetric write and read ports.
    WIDTHCFG_SYM : if cWidthConfig = sSym generate
        writeAddress    <= iAddress_A;
        writeByteenable <= iByteenable_A;
        writedata       <= iWritedata_A;
        readAddress     <= iAddress_B;
        oReaddata_B     <= readdata;
    end generate WIDTHCFG_SYM;
end architecture rtl;
