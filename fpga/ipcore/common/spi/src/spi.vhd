-------------------------------------------------------------------------------
-- SPI Slave IP-Core
--
--       Copyright (C) 2009 B&R
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
--
-------------------------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.std_logic_arith.ALL;
USE ieee.std_logic_unsigned.ALL;

entity spi is
    generic (
        frameSize_g            : integer    := 8;
        cpol_g                : boolean    := false;
        cpha_g                : boolean    := false
    );
    port (
        -- Control Interface
        clk                    : in     std_logic;
        rst                    : in     std_logic;
        din                    : in     std_logic_vector(frameSize_g-1 downto 0);
        load                : in    std_logic; --load din
        dout                : out     std_logic_vector(frameSize_g-1 downto 0);
        valid                : out    std_logic; --dout is valid
        -- SPI
        sck                    : in     std_logic;
        ss                    : in     std_logic;
        miso                : out     std_logic;
        mosi                : in     std_logic
    );
end spi;

architecture rtl of spi is
--pulse generation out of spi clock (sck)
signal    sckL                :        std_logic;
signal    sckRising            :        std_logic;
signal    sckFalling            :        std_logic;
signal    capPulse            :        std_logic; --pulse to capture data
signal    setPulse            :        std_logic; --pulse to change data
--capture data
signal    capMosi                :        std_logic;
signal    capDout                :        std_logic_vector(frameSize_g-1 downto 0);
--frame counter
signal    cnt                    :        integer range 0 to frameSize_g;
signal    tc                    :        std_logic;

signal    miso_s                :        std_logic;
signal    din_s                :         std_logic_vector(frameSize_g-1 downto 0);
begin

    miso <= miso_s when ss = '1' else 'Z'; --set miso to high if slave isn't selected!

    spiClkShiftReg : process(clk, rst)
    begin
        if rst = '1' then
            if cpol_g = false then
                sckL <= '0';
            else
                sckL <= '1';
            end if;
        elsif clk = '1' and clk'event then
            sckL <= sck;
        end if;
    end process;

    --generate sck rising falling edge pulse
    sckRising <= '1' when sckL = '0' and sck = '1' else '0';
    sckFalling <= '1' when sckL = '1' and sck = '0' else '0';

    capPulse <= '0'            when (ss /= '1') else
                sckRising    when (cpha_g = false and cpol_g = false) else
                sckFalling    when (cpha_g = false and cpol_g = true)  else
                sckFalling    when (cpha_g = true  and cpol_g = false) else
                sckRising    when (cpha_g = true  and cpol_g = true)  else
                '0';

    setPulse <= '0'            when (ss /= '1') else
                sckFalling    when (cpha_g = false and cpol_g = false) else
                sckRising    when (cpha_g = false and cpol_g = true)  else
                sckRising    when (cpha_g = true  and cpol_g = false) else
                sckFalling    when (cpha_g = true  and cpol_g = true)  else
                '0';

    theCapLatch : process(clk, rst)
    begin
        if rst = '1' then
            capMosi <= '0';
        elsif clk = '1' and clk'event then
            if capPulse = '1' then
                --capture mosi data
                capMosi <= mosi;
            elsif load = '1' and cpha_g = true then
                capMosi <= din(0);
            end if;
        end if;
    end process;

    theFrameCnt : process(clk, rst)
    begin
        if rst = '1' then
            cnt <= 0;
        elsif clk = '1' and clk'event then
            if tc = '1' or ss = '0' then
                cnt <= 0;
            elsif capPulse = '1' and cpha_g = true then
                cnt <= cnt + 1;
            elsif setPulse = '1' and cpha_g = false then
                cnt <= cnt + 1;
            end if;
        end if;
    end process;
    tc <= '1' when cnt = frameSize_g else '0';

    theDoutLatch : process(clk, rst)
    begin
        if rst = '1' then
            dout <= (others => '0');
        elsif clk = '1' and clk'event then
            valid <= '0';
            if tc = '1' then
                if cpha_g = false then
                    dout <= capDout;
                else
                    dout <= capDout(capDout'left-1 downto 0) & capMosi;
                end if;
                valid <= '1';
            end if;
        end if;
    end process;

    dinGenPhaF : if cpha_g = false generate
        din_s <= din;
    end generate;

    dinGenPhaT : if cpha_g = true generate
        din_s <= '0' & din(din'left downto 1);
    end generate;

    theShiftRegister : entity work.spi_sreg
        generic map (
            size_g => frameSize_g
        )
        port map (
            clk => clk,
            rst => rst,
            shift => setPulse,
            load => load,
            din => din_s,
            dout => capDout,
            sin => capMosi,
            sout => miso_s
        );

end rtl;
