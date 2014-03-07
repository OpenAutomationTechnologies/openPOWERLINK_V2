--! @file dpRam-bhv-a.vhd
--
--! @brief Dual Port Ram Register Transfer Level Architecture
--
--! @details This is the DPRAM intended for synthesis on Xilinx Spartan 6 only.
--!          Timing as follows [clk-cycles]: write=0 / read=1
--
-------------------------------------------------------------------------------
-- Architecture : rtl
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

architecture rtl of dpRam is
    --! Width of a byte
    constant cByte      : natural := 8;
    --! Address width (used to generate size depending on address width)
    constant cAddrWidth : natural := iAddress_A'length;
    --! RAM size
    constant cRamSize   : natural := 2**cAddrWidth;

    --! Type for data port
    subtype tDataPort is std_logic_vector(gWordWidth-1 downto 0);
    --! RAM type with given size
    type tRam is array (cRamSize-1 downto 0) of tDataPort;

    --! Shared variable to model and synthesize a DPR
    shared variable vDpram : tRam := (others => (others => cInactivated));

    --! Port A readport
    signal readdataA    : tDataPort;
    --! Port B readport
    signal readdataB    : tDataPort;
begin
    assert (gInitFile = "UNUSED")
    report "Memory initialization is not supported in this architecture!"
    severity warning;

    -- assign readdata to ports
    oReaddata_A <= readdataA;
    oReaddata_B <= readdataB;

    --! This process describes port A of the DPRAM. The write process considers
    --! iWriteEnable_A and iByteenable_A. The read process is done with every
    --! rising iClk_A edge.
    PORTA : process(iClk_A)
    begin
        if rising_edge(iClk_A) then
            if iEnable_A = cActivated then
                if iWriteEnable_A = cActivated then
                    for i in iByteenable_A'range loop
                        if iByteenable_A(i) = cActivated then
                            -- write byte to DPRAM
                            vDpram(to_integer(unsigned(iAddress_A)))(
                                (i+1)*cByte-1 downto i*cByte
                            ) := iWritedata_A(
                                (i+1)*cByte-1 downto i*cByte
                            );
                        end if; --byteenable
                    end loop;
                end if; --writeenable
                -- read word from DPRAM
                readdataA <= vDpram(to_integer(unsigned(iAddress_A)));
            end if; --enable
        end if;
    end process PORTA;

    --! This process describes port B of the DPRAM. The write process considers
    --! iWriteEnable_B and iByteenable_B. The read process is done with every
    --! rising iClk_B edge.
    PORTB : process(iClk_B)
    begin
        if rising_edge(iClk_B) then
            if iEnable_B = cActivated then
                if iWriteEnable_B = cActivated then
                    for i in iByteenable_B'range loop
                        if iByteenable_B(i) = cActivated then
                            -- write byte to DPRAM
                            vDpram(to_integer(unsigned(iAddress_B)))(
                                (i+1)*cByte-1 downto i*cByte
                            ) := iWritedata_B(
                                (i+1)*cByte-1 downto i*cByte
                            );
                        end if; --byteenable
                    end loop;
                end if; --writeenable
                -- read word from DPRAM
                readdataB <= vDpram(to_integer(unsigned(iAddress_B)));
            end if; --enable
        end if;
    end process PORTB;
end architecture rtl;
