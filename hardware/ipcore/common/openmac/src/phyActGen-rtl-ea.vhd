-------------------------------------------------------------------------------
--! @file phyActGen-rtl-ea.vhd
--
--! @brief Phy activity generator
--
--! @details The phy activity generator generates a free-running clock-synchronous
--!          packet activity signal. This signal can be used to drive an LED.
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

entity phyActGen is
    generic (
        --! Generated activity frequency of oActivity [Hz]
        gActivityFreq   : natural := 6;
        --! Clock frequency of iClk [Hz]
        gClkFreq        : natural := 50e6
    );
    port (
        --! Reset
        iRst        : in std_logic;
        --! Clock
        iClk        : in std_logic;
        --! MAC Tx enable signal
        iTxEnable   : in std_logic;
        --! MAC Rx data valid signal
        iRxValid    : in std_logic;
        --! Generated activity signal
        oActivity   : out std_logic
    );
end phyActGen;

architecture rtl of phyActGen is
    --! Obtain maximum counter value to achieve activity frequency
    constant cCntMaxValue   : natural := gClkFreq / gActivityFreq;
    --! Obtain counter width
    constant cCntWidth      : natural := logDualis(cCntMaxValue);

    --! The counter
    signal counter          : std_logic_vector(cCntWidth-1 downto 0);
    --! Constant for counter value zero
    constant cCntIsZero     : std_logic_vector(counter'range) := (others => cInactivated);
    --! Terminal counter
    signal counterTc        : std_logic;
    --! Trigger activity in next cycle due to packet activity
    signal triggerActivity  : std_logic;
    --! Enable activity
    signal enableActivity   : std_logic;
begin
    oActivity <=    counter(counter'high) when enableActivity = cActivated else
                    cInactivated;

    ledCntr : process(iRst, iClk)
    begin
        if iRst = cActivated then
            triggerActivity <= cInactivated;
            enableActivity  <= cInactivated;
        elsif rising_edge(iClk) then
            --monoflop, of course no default value!
            if triggerActivity = cActivated and counterTc = cActivated then
                --counter overflow and activity within last cycle
                enableActivity <= cActivated;
            elsif counterTc = cActivated then
                --counter overflow but no activity
                enableActivity <= cInactivated;
            end if;

            --monoflop, of course no default value!
            if counterTc = cActivated then
                --count cycle over, reset trigger
                triggerActivity <= cInactivated;
            elsif iTxEnable = cActivated or iRxValid = cActivated then
                --activity within cycle
                triggerActivity <= cActivated;
            end if;
        end if;
    end process;

    theFreeRunCnt : process(iClk, iRst)
    begin
        if iRst = cActivated then
            counter <= (others => cInactivated);
        elsif iClk = cActivated and iClk'event then
            counter <= std_logic_vector(unsigned(counter) - 1);
        end if;
    end process;

    counterTc <=    cActivated when counter = cCntIsZero else
                    cInactivated;
end rtl;
