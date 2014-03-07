-------------------------------------------------------------------------------
--! @file binaryEncoderRtl.vhd
--
--! @brief Generic Binary Encoder with reduced or-operation
--
--! @details This generic binary encoder can be configured to any width,
--! however, mind base 2 values. In order to reduce the complexity of the
--! synthesized circuit the reduced or-operation is applied.
-- (Borrowed from academic.csuohio.edu/chu_p and applied coding styles)
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

entity binaryEncoder is
    generic (
        --! One-hot data width
        gDataWidth : natural := 8
    );
    port (
        --! One hot code input
        iOneHot : in std_logic_vector(gDataWidth-1 downto 0);
        --! Binary encoded output
        oBinary : out std_logic_vector(LogDualis(gDataWidth)-1 downto 0)
    );
end binaryEncoder;

architecture rtl of binaryEncoder is
    type tMaskArray is array(LogDualis(gDataWidth)-1 downto 0) of
        std_logic_vector(gDataWidth-1 downto 0);

    signal mask : tMaskArray;

    function genOrMask return tMaskArray is
        variable vOrMask: tMaskArray;
    begin
        for i in (LogDualis(gDataWidth)-1) downto 0 loop
            for j in (gDataWidth-1) downto 0 loop
                if (j/(2**i) mod 2)= 1 then
                    vOrMask(i)(j) := '1';
                else
                    vOrMask(i)(j) := '0';
                end if;
            end loop;
        end loop;
        return vOrMask;
    end function;
begin
    mask <= genOrMask;

    process (
        mask,
        iOneHot
    )
        variable rowVector : std_logic_vector(gDataWidth-1 downto 0);
        variable tempBit : std_logic;
    begin
        for i in (LogDualis(gDataWidth)-1) downto 0 loop
            rowVector := iOneHot and mask(i);
            -- reduced or operation
            tempBit := '0';
            for j in (gDataWidth-1) downto 0 loop
                tempBit := tempBit or rowVector(j);
            end loop;
            oBinary(i) <= tempBit;
        end loop;
    end process;

end rtl;
