-------------------------------------------------------------------------------
--! @file lutFileRtl.vhd
--
--! @brief Look-up table file implementation
--
--! @details This look-up table file stores initialization values (generics)
--! in LUT resources.
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

entity lutFile is
    generic (
        gLutCount       : natural := 4;
        gLutWidth       : natural := 32;
        gLutInitValue   : std_logic_vector :=
        x"1111_1111" & x"2222_2222" & x"3333_3333" & x"4444_4444"
        );
    port (
        iAddrRead   :   in std_logic_vector(LogDualis(gLutCount)-1 downto 0);
        oData       :   out std_logic_vector
        );
end lutFile;

architecture Rtl of lutFile is
    constant cLutFile : std_logic_vector(gLutCount*gLutWidth-1 downto 0) :=
    gLutInitValue;

    signal lutOutput : std_logic_vector(gLutWidth-1 downto 0);
begin

    --Lut File is a bitstream that is blockwise (gLutWidth) read with
    --respect to iAddrRead.
    bitSelect : process(iAddrRead)
    begin
        --default
        lutOutput <= (others => '0');

        for i in gLutWidth-1 downto 0 loop
            --assign selected bits in Lut File to output
            lutOutput(i) <= cLutFile
            ( (gLutCount-1-to_integer(unsigned(iAddrRead)))*gLutWidth + i );
        end loop;
    end process;

    --! downscale lut width to output
    oData <= lutOutput(oData'range);

end Rtl;
