-------------------------------------------------------------------------------
--! @file dpRamSplxNbe-a.vhd
--
--! @brief Simplex Dual Port Ram without byteenables
--
--! @details This is the Simplex DPRAM without byteenables for Xilinx platforms.
--!          The DPRAM has one write and one read port only.
--!          Timing as follows [clk-cycles]: write=0 / read=1
--
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

architecture rtl of dpRamSplxNbe is
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

    --! Port B readport
    signal readdataB    : tDataPort;
begin
    -- assign readdata to ports
    oReaddata_B <= readdataB;

    --! This process describes port A of the DPRAM. The write process considers
    --! iWriteEnable_A.
    PORTA : process(iClk_A)
    begin
        if rising_edge(iClk_A) then
            if iEnable_A = cActivated then
                if iWriteEnable_A = cActivated then
                    -- write byte to DPRAM
                    vDpram(to_integer(unsigned(iAddress_A))) := iWritedata_A;
                end if; --writeenable
            end if; --enable
        end if;
    end process PORTA;

    --! This process describes port B of the DPRAM. The read process is done
    --! with every rising iClk_B edge.
    PORTB : process(iClk_B)
    begin
        if rising_edge(iClk_B) then
            if iEnable_B = cActivated then
                -- read word from DPRAM
                readdataB <= vDpram(to_integer(unsigned(iAddress_B)));
            end if; --enable
        end if;
    end process PORTB;
end architecture rtl;
