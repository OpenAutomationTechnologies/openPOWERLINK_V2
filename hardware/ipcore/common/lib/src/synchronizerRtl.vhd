-------------------------------------------------------------------------------
--! @file synchronizerRtl.vhd
--
--! @brief Synchronizer
--
--! @details This is a synchronizer with configurable stage size.
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

entity synchronizer is
    generic (
        --! Stages
        gStages : natural := 2;
        --! Initialization level
        gInit : std_logic := cInactivated
    );
    port (
        --! Asynchronous reset
        iArst : in std_logic;
        --! Clock
        iClk : in std_logic;
        --! Asynchronous input
        iAsync : in std_logic;
        --! Synchronous output
        oSync : out std_logic
    );
end synchronizer;

architecture rtl of synchronizer is
    --! Meta registers used to synchronize input signal
    signal metaReg : std_logic_vector(gStages-1 downto 0);
    --! Meta registers next
    signal metaReg_next : std_logic_vector(metaReg'range);
begin
    -- handle wrong stage generic
    assert (gStages > 0)
    report "gStages must be set higher 0!" severity failure;

    -- output last synchronizer stage
    oSync <= metaReg(metaReg'left);

    -- assign asynchronous signal to metaRegisters
    metaReg_next <= metaReg(metaReg'left-1 downto 0) & iAsync;

    reg : process(iArst, iClk)
    begin
        if iArst = cActivated then
            metaReg <= (others => gInit);
        elsif rising_edge(iClk) then
            metaReg <= metaReg_next;
        end if;
    end process;
end rtl;
