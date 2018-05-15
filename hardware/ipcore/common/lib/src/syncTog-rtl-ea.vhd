-------------------------------------------------------------------------------
--! @file syncTog-rtl-ea.vhd
--
--! @brief Synchronizer with toggling signal
--
--! @details This is a synchronizer that transfers an incoming signal to the
--!          target clock domain with toggling signal levels.
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

entity syncTog is
    generic (
        --! Stages
        gStages     : natural := 2;
        --! Initialization level
        gInit       : std_logic := cInactivated
    );
    port (
        --! Source reset
        iSrc_rst    : in    std_logic;
        --! Source clock
        iSrc_clk    : in    std_logic;
        --! Source data
        iSrc_data   : in    std_logic;
        --! Destination reset
        iDst_rst    : in    std_logic;
        --! Destination clock
        iDst_clk    : in    std_logic;
        --! Destination data
        oDst_data   : out   std_logic
    );
end syncTog;

architecture rtl of syncTog is
    --! Source pulse
    signal srcPulse     : std_logic;
    --! Transfer toggle
    signal metaToggle   : std_logic;
    --! Transferred toggle
    signal toggle       : std_logic;
    --! Destination pulse
    signal dstPulse     : std_logic;
begin
    -- Output map
    oDst_data <= dstPulse;

    --! This is the first edge detector generating a single pulse.
    FIRST_EDGE : entity libcommon.edgedetector
        port map (
            iArst       => iSrc_rst,
            iClk        => iSrc_clk,
            iEnable     => cActivated,
            iData       => iSrc_data,
            oRising     => srcPulse,
            oFalling    => open,
            oAny        => open
        );

    --! This process generates a toggling signal, controled by the rising edge
    --! of the first edge detector.
    GEN_TOGGLE : process(iSrc_rst, iSrc_clk)
    begin
        if iSrc_rst = cActivated then
            metaToggle <= cInactivated;
        elsif rising_edge(iSrc_clk) then
            if srcPulse = cActivated then
                metaToggle <= not metaToggle;
            end if;
        end if;
    end process GEN_TOGGLE;

    --! This synchronizer transfers the metaToggle to the destination clock
    --! domain.
    SYNC : entity libcommon.synchronizer
        generic map (
            gStages => gStages,
            gInit   => gInit
        )
        port map (
            iArst   => iDst_rst,
            iClk    => iDst_clk,
            iAsync  => metaToggle,
            oSync   => toggle
        );

    --! The second edge detector detects any edge of the synchronized toggle.
    SECOND_EDGE : entity libcommon.edgedetector
        port map (
            iArst       => iDst_rst,
            iClk        => iDst_clk,
            iEnable     => cActivated,
            iData       => toggle,
            oRising     => open,
            oFalling    => open,
            oAny        => dstPulse
        );
end rtl;
