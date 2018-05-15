-------------------------------------------------------------------------------
--! @file bcd2ledRtl.vhd
--
--! @brief BCD to 7-segement LED
--
--! @details This compontent decodes a binary coded input to 7-segement LED
--! display.
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

entity bcd2led is
    port (
        --! BCD input
        iBcdVal : in std_logic_vector(3 downto 0);
        --! LED output (high-active)
        oLed    : out std_logic_vector(6 downto 0);
        --! LED output (low-active)
        onLed   : out std_logic_vector(6 downto 0)
    );
end entity;

architecture rtl of bcd2led is
    --! Array type for conversion lut
    type tDecArray is array (natural range <>) of std_logic_vector(6 downto 0);
    --! Lut holding the decode values
    constant cDecodeLut : tDecArray(0 to 15) := (
        -- 0        1           2           3
        "0111111",  "0000110",  "1011011",  "1001111",
        -- 4        5           6           7
        "1100110",  "1101101",  "1111101",  "0000111",
        -- 8        9           A           B
        "1111111",  "1101111",  "1110111",  "1111100",
        -- C        D           E           F
        "0111001",  "1011110",  "1111001",  "1110001"
    );
    --! High active decoded
    signal led  : std_logic_vector(oLed'range);
begin
    -- output
    oLed    <= led;
    onLed   <= not led;

    -- assign lut
    led     <= cDecodeLut(to_integer(unsigned(iBcdVal)));
end architecture;
