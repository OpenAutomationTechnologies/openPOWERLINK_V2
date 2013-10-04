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

LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.std_logic_arith.all;
USE ieee.std_logic_unsigned.all;

ENTITY slow2fastSync IS
    GENERIC (
            doSync_g                    :        BOOLEAN := TRUE
    );
    PORT (
            dataSrc                        : IN    STD_LOGIC;
            dataDst                        : OUT    STD_LOGIC;
            clkSrc                        : IN    STD_LOGIC;
            rstSrc                        : IN    STD_LOGIC;
            clkDst                        : IN    STD_LOGIC;
            rstDst                        : IN    STD_LOGIC
    );
END ENTITY slow2fastSync;

ARCHITECTURE rtl OF slow2fastSync IS
signal toggle, toggleSync, pulse, dataDst_s : std_logic;
begin

    dataDst <= dataDst_s when doSync_g = TRUE else dataSrc;

    genSync : IF doSync_g = TRUE GENERATE
        firstEdgeDet : entity work.edgeDet
            port map (
                din => dataSrc,
                rising => pulse,
                falling => open,
                any => open,
                clk => clkSrc,
                rst => rstSrc
            );

        process(clkSrc, rstSrc)
        begin
            if rstSrc = '1' then
                toggle <= '0';
            elsif clkSrc = '1' and clkSrc'event then
                if pulse = '1' then
                    toggle <= not toggle;
                end if;
            end if;
        end process;

        sync : entity work.sync
            port map (
                din => toggle,
                dout => toggleSync,
                clk => clkDst,
                rst => rstDst
            );

        secondEdgeDet : entity work.edgeDet
            port map (
                din => toggleSync,
                rising => open,
                falling => open,
                any => dataDst_s,
                clk => clkDst,
                rst => rstDst
            );
    END GENERATE;

END ARCHITECTURE rtl;