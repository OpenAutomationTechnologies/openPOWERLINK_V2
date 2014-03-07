-------------------------------------------------------------------------------
--! @file cntRtl.vhd
--
--! @brief Terminal Counter
--
--! @details The terminal counter is a synchronous counter configured
--!          by several generics.
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

entity cnt is
    generic (
        --! Width of counter
        gCntWidth : natural := 32;
        --! Value that triggers the counter reset
        gTcntVal : natural := 1000
    );
    port (
        iArst : in std_logic;
        iClk : in std_logic;
        iEnable : in std_logic;
        iSrst : in std_logic;
        oCnt : out std_logic_vector(gCntWidth-1 downto 0);
        oTcnt : out std_logic
    );
end entity;

architecture rtl of cnt is
    constant cTcntVal : std_logic_vector(gCntWidth-1 downto 0) :=
        std_logic_vector(to_unsigned(gTcntVal, gCntWidth));

    signal cnt, cnt_next : std_logic_vector(gCntWidth-1 downto 0);
    signal tc : std_logic;
begin
    -- handle wrong generics
    assert (gTcntVal > 0)
    report "Terminal count value of 0 makes no sense!"
    severity failure;

    regClk : process(iArst, iClk)
    begin
        if iArst = cActivated then
            cnt <= (others => cInactivated);
        elsif rising_edge(iClk) then
            cnt <= cnt_next;
        end if;
    end process;

    tc <= cActivated when cnt = cTcntVal else
          cInactivated;

    oCnt <= cnt;
    oTcnt <= tc;

    comb : process(iSrst, iEnable, cnt, tc)
    begin
        --default
        cnt_next <= cnt;

        if iSrst = cActivated then
            cnt_next <= (others => cInactivated);
        elsif iEnable = cActivated then
            if tc = cActivated then
                cnt_next <= (others => cInactivated);
            else
                cnt_next <= std_logic_vector(unsigned(cnt) + 1);
            end if;
        end if;
    end process;

end architecture;
