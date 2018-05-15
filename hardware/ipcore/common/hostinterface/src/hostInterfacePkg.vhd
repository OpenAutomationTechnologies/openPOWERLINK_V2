-------------------------------------------------------------------------------
--! @file hostInterfacePkg.vhd
--
--! @brief Host interface package
--
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

package hostInterfacePkg is
    -- constants
    --! dword size
    constant cDword                     : natural :=    32;
    --! word size
    constant cWord                      : natural :=    16;
    --! byte size
    constant cByte                      : natural :=    8;
    --! nible size
    constant cNibble                    : natural :=    4;

    --! size of 32 bit std_logic_vector array
    constant cArrayStd32ElementSize : natural := 32;

    --! external synchronization config
    constant cExtSyncConfigWidth : natural := 3;
    constant cExtSyncEdgeConfigWidth : natural := 2;
    constant cExtSyncEdgeRis : std_logic_vector
    (cExtSyncEdgeConfigWidth-1 downto 0)            := "01";
    constant cExtSyncEdgeFal : std_logic_vector
    (cExtSyncEdgeConfigWidth-1 downto 0)            := "10";
    constant cExtSyncEdgeAny : std_logic_vector
    (cExtSyncEdgeConfigWidth-1 downto 0)            := "11";

    -- type
    --! 32 bit std_logic_vector array type
    type tArrayStd32 is
    array (natural range <>) of std_logic_vector
    (cArrayStd32ElementSize-1 downto 0);

    -- function
    --! convert arrayIn into std_logic_vector stream
    function CONV_STDLOGICVECTOR (arrayIn : tArrayStd32; arrayLength : natural)
    return std_logic_vector;
end hostInterfacePkg;

package body hostInterfacePkg is
    -- function
    function CONV_STDLOGICVECTOR (arrayIn : tArrayStd32; arrayLength : natural)
        return std_logic_vector is
        variable vTmpStream : std_logic_vector
        (arrayLength*cArrayStd32ElementSize-1 downto 0);
        variable vTmpElement : std_logic_vector
        (cArrayStd32ElementSize-1 downto 0);
    begin

        for i in arrayLength downto 1 loop
            --get current element
            vTmpElement := arrayIn(arrayLength-i);

            --write element into stream
            vTmpStream
            (i*cArrayStd32ElementSize-1 downto (i-1)*cArrayStd32ElementSize) :=
            vTmpElement;
        end loop;

        return vTmpStream;
    end function;
end package body;
