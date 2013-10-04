-------------------------------------------------------------------------------
--
--    (c) B&R, 2011
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
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;
use ieee.math_real.log2;
use ieee.math_real.ceil;

entity OpenMAC_phyAct is
    generic(
        iBlinkFreq_g : integer := 6 -- [Hz]
    );
    port(
        clk : in std_logic;
        rst : in std_logic;
        tx_en : in std_logic;
        rx_dv : in std_logic;
        act_led : out std_logic
    );
end OpenMAC_phyAct;

architecture rtl of OpenMAC_phyAct is
constant     iMaxCnt                     : integer := 50e6 / iBlinkFreq_g;
constant    iLog2MaxCnt                    : integer := integer(ceil(log2(real(iMaxCnt))));

signal        cnt                            : std_logic_vector(iLog2MaxCnt-1 downto 0);
signal        cnt_tc                        : std_logic;
signal        actTrig                        : std_logic;
signal        actEnable                    : std_logic;
begin

    act_led <= cnt(cnt'high) when actEnable = '1' else '0';

    ledCntr : process(clk, rst)
    begin
        if rst = '1' then
            actTrig <= '0';
            actEnable <= '0';
        elsif clk = '1' and clk'event then
            --monoflop, of course no default value!
            if actTrig = '1' and cnt_tc = '1' then
                --counter overflow and activity within last cycle
                actEnable <= '1';
            elsif cnt_tc = '1' then
                --counter overflow but no activity
                actEnable <= '0';
            end if;

            --monoflop, of course no default value!
            if cnt_tc = '1' then
                --count cycle over, reset trigger
                actTrig <= '0';
            elsif tx_en = '1' or rx_dv = '1' then
                --activity within cycle
                actTrig <= '1';
            end if;
        end if;
    end process;

    theFreeRunCnt : process(clk, rst)
    begin
        if rst = '1' then
            cnt <= (others => '0');
        elsif clk = '1' and clk'event then
            --nice, it may count for ever!
            cnt <= cnt - 1;
        end if;
    end process;

    cnt_tc <= '1' when cnt = 0 else '0'; --"counter overflow"

end rtl;
