-------------------------------------------------------------------------------
-- Simple Port I/O valid pulse counter
--
--       Copyright (C) 2010 B&R
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

LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.std_logic_arith.all;
USE ieee.std_logic_unsigned.all;

entity portio_cnt is
    generic (
        maxVal             :        integer := 50 --clock ticks of pcp_clk
    );
    port (
        clk                :        in std_logic;
        rst                :        in std_logic;
        pulse            :        in std_logic;
        valid            :        out std_logic
    );
end entity portio_cnt;

architecture rtl of portio_cnt is
signal cnt : integer range 0 to maxVal-2;
signal tc, en : std_logic;
begin
    genCnter : if maxVal > 1 generate
        tc <= '1' when cnt = maxVal-2 else '0';
        valid <= en or pulse;

        counter : process(clk, rst)
        begin
            if rst = '1' then
                cnt <= 0;
            elsif clk = '1' and clk'event then
                if tc = '1' then
                    cnt <= 0;
                elsif en = '1' then
                    cnt <= cnt + 1;
                else
                    cnt <= 0;
                end if;
            end if;
        end process;

        enGen : process(clk, rst)
        begin
            if rst = '1' then
                en <= '0';
            elsif clk = '1' and clk'event then
                if pulse = '1' then
                    en <= '1';
                elsif tc = '1' then
                    en <= '0';
                end if;
            end if;
        end process;
    end generate;

    genSimple : if maxVal = 1 generate
        valid <= pulse;
    end generate;

end architecture rtl;