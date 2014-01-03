-------------------------------------------------------------------------------
--! @file addrDecodeRtl.vhd
--
--! @brief Address Decoder for generating select signal
--
--! @details This address decoder generates a select signal depending on the
--! provided base- and high-addresses by using smaller/greater logic.
--! Additionally a strob is generated if the base or high address is selected.
-------------------------------------------------------------------------------
--
--    (c) B&R, 2013
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
use work.global.all;

entity addrDecode is
    generic (
        --! Address bus width
        gAddrWidth : natural := 32;
        --! Decode space base address
        gBaseAddr : natural := 16#1000#;
        --! Decode space high address
        gHighAddr : natural := 16#1FFF#
    );
    port (
        --! Enable decoding
        iEnable : in std_logic;
        --! Address bus
        iAddress : in std_logic_vector(gAddrWidth-1 downto 0);
        --! Select output
        oSelect : out std_logic
    );
end addrDecode;

architecture rtl of addrDecode is
    --! Address to be decoded
    signal address : unsigned(gAddrWidth-1 downto 0);
    --! Address is in range
    signal addressInRange : std_logic;

    --! Base address used for comparison
    constant cBase : unsigned(gAddrWidth-1 downto 0) :=
                                        to_unsigned(gBaseAddr, gAddrWidth);
    --! High address used for comparison
    constant cHigh : unsigned(gAddrWidth-1 downto 0) :=
                                        to_unsigned(gHighAddr, gAddrWidth);
begin
    -- check generics
    assert (gBaseAddr < gHighAddr)
    report "Base address should be smaller than High address!" severity failure;

    -- connect ports to signals
    oSelect <= addressInRange;
    address <= unsigned(iAddress);

    --! Decode input address logic
    combAddrDec : process (
        iEnable,
        address
    )
    begin
        --default assignments of process outputs
        addressInRange <= cInactivated;

        if iEnable = cActivated then
            if (cBase <= address) and (address <= cHigh) then
                addressInRange <= cActivated;
            end if;
        end if;
    end process;
end rtl;
