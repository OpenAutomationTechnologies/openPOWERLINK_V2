-------------------------------------------------------------------------------
--! @file openmacTimer-rtl-ea.vhd
--
--! @brief OpenMAC timer module
--
--! @details This is the openMAC timer module. It supports accessing the MAC
--!          time and generating interrupts.
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

library work;
--! use global library
use work.global.all;
--! use openmac package
use work.openmacPkg.all;

entity openmacTimer is
    generic (
        --! Data width of iMacTime
        gMacTimeWidth           : natural := 32;
        --! Generate second timer
        gMacTimer_2ndTimer      : boolean := false;
        --! Width of pulse register
        gTimerPulseRegWidth     : integer := 9;
        --! Enable pulse width control
        gTimerEnablePulseWidth  : boolean := false
    );
    port (
        --! Reset
        iRst        : in    std_logic;
        --! Clock (same like MAC)
        iClk        : in    std_logic;
        --! Write
        iWrite      : in    std_logic;
        --! Address (dword addressing!)
        iAddress    : in    std_logic_vector(3 downto 2);
        --! Write data
        iWritedata  : in    std_logic_vector(31 downto 0);
        --! Read data
        oReaddata   : out   std_logic_vector(31 downto 0);
        --! MAC time
        iMacTime    : in    std_logic_vector(gMacTimeWidth-1 downto 0);
        --! Interrupt of first timer (level triggered)
        oIrq        : out   std_logic;
        --! Toggle output of second timer
        oToggle     : out   std_logic
    );
end openmacTimer;

architecture rtl of openmacTimer is
    signal cmp_enable           : std_logic;
    signal tog_enable           : std_logic;
    signal cmp_value            : std_logic_vector(iMacTime'range);
    signal tog_value            : std_logic_vector(iMacTime'range);
    signal tog_counter_value    : std_logic_vector(gTimerPulseRegWidth-1 downto 0);
    signal tog_counter_preset   : std_logic_vector(gTimerPulseRegWidth-1 downto 0);
    signal irq_s                : std_logic;
    signal toggle_s             : std_logic;
begin
    oIrq <= irq_s;
    oToggle <= toggle_s;

    --! This process generates the interrupt and toggle signals and handles the
    --! register writes.
    REGPROC : process(iRst, iClk)
    begin
        if iRst = '1' then
            cmp_enable <= '0';
            cmp_value <= (others => '0');
            irq_s <= '0';

            if gMacTimer_2ndTimer = TRUE then
                tog_enable <= '0';
                tog_value <= (others => '0');
                toggle_s <= '0';
                if gTimerEnablePulseWidth = TRUE then
                    tog_counter_value <= (others => '0');
                    tog_counter_preset <= (others => '0');
                end if;
            end if;
        elsif rising_edge(iClk) then
            --cmp
            if cmp_enable = '1' and iMacTime = cmp_value then
                irq_s <= '1';
            end if;

            --tog
            if tog_enable = '1' and iMacTime = tog_value and gMacTimer_2ndTimer = TRUE then
                toggle_s <= not toggle_s;
                if gTimerEnablePulseWidth = TRUE then
                    tog_counter_value <= tog_counter_preset;
                end if;
            end if;
            if tog_enable = '1' and toggle_s = '1'
               and (not (tog_counter_value = std_logic_vector(to_unsigned(0, tog_counter_value'length))))
               and gMacTimer_2ndTimer = TRUE
               and gTimerEnablePulseWidth = TRUE then
                tog_counter_value <= std_logic_vector(unsigned(tog_counter_value) - 1);
                if tog_counter_value = std_logic_vector(to_unsigned(1, tog_counter_value'length)) then
                    toggle_s <= '0';
                end if;
            end if;

            --memory mapping
            if iWrite = '1' then
                case iAddress is
                    when "00" =>
                        cmp_value <= iWritedata;
                        irq_s <= '0';
                    when "01" =>
                        cmp_enable <= iWritedata(0);
                    when "10" =>
                        if gMacTimer_2ndTimer = TRUE then
                            tog_value <= iWritedata;
                        end if;
                    when "11" =>
                        if gMacTimer_2ndTimer = TRUE then
                            tog_enable <= iWritedata(0);
                            if gTimerEnablePulseWidth = TRUE then
                                tog_counter_preset <= iWritedata(gTimerPulseRegWidth downto 1);
                            end if;
                        end if;
                    when others =>
                        assert (FALSE)
                        report "Write in forbidden area?"
                        severity failure;
                end case;
            end if;
        end if;
    end process REGPROC;

    ASSIGN_RD : process (
        iAddress,
        iMacTime,
        irq_s,
        cmp_enable,
        tog_value,
        toggle_s,
        tog_enable
    )
    begin
        --default is all zero
        oReaddata <= (others => cInactivated);

        case iAddress is
            when "00" =>
                oReaddata           <= iMacTime;
            when "01" =>
                oReaddata(1)        <= irq_s;
                oReaddata(0)        <= cmp_enable;
            when "10" =>
                if gMacTimer_2ndTimer = TRUE then
                    oReaddata       <= tog_value;
                end if;
            when "11" =>
                if gMacTimer_2ndTimer = TRUE then
                    oReaddata(1)    <= toggle_s;
                    oReaddata(0)    <= tog_enable;
                end if;
            when others =>
                NULL; --this is okay, since default assignment is above!
        end case;
    end process ASSIGN_RD;
end rtl;
