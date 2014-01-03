-------------------------------------------------------------------------------
--! @file edgedetectorRtl.vhd
--
--! @brief Edge detector
--
--! @details This is an edge detector circuit providing any, rising and falling
--! edge outputs.
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

entity edgedetector is
    port (
        --! Asynchronous reset
        iArst : in std_logic;
        --! Clock
        iClk : in std_logic;
        --! Enable detection
        iEnable : in std_logic;
        --! Data to be sampled
        iData : in std_logic;
        --! Rising edge detected (unregistered)
        oRising : out std_logic;
        --! Falling edge detected (unregistered)
        oFalling : out std_logic;
        --! Any edge detected (unregistered)
        oAny : out std_logic
    );
end edgedetector;

architecture rtl of edgedetector is
    --! Register to delay input by one clock cycle
    signal reg          : std_logic;
    --! Register next
    signal reg_next     : std_logic;
    --! Second register
    signal reg_l        : std_logic;
    --! Second register next
    signal reg_l_next   : std_logic;
begin
    -- assign input data to register
    reg_next <= iData;

    --! Detection
    comb : process (
        iEnable,
        reg,
        reg_l
    )
    begin
        -- default
        oRising <= cInactivated;
        oFalling <= cInactivated;
        oAny <= cInactivated;

        if iEnable = cActivated then
            -- rising edge
            if reg_l = cInactivated and reg = cActivated then
                oRising <= cActivated;
                oAny <= cActivated;
            end if;

            -- falling edge
            if reg_l = cActivated and reg = cInactivated then
                oFalling <= cActivated;
                oAny <= cActivated;
            end if;
        end if;
    end process;

    reg_l_next <= reg;

    --! Clock process
    regClk : process(iArst, iClk)
    begin
        if iArst = cActivated then
            reg     <= cInactivated;
            reg_l   <= cInactivated;
        elsif rising_edge(iClk) then
            reg     <= reg_next;
            reg_l   <= reg_l_next;
        end if;
    end process;
end rtl;
