-------------------------------------------------------------------------------
--! @file openmacTimer-rtl-ea.vhd
--
--! @brief OpenMAC timer module
--
--! @details This is the openMAC timer module. It supports accessing the MAC
--!          time and generating interrupts.
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

--! Work library
library work;
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
        iAddress    : in    std_logic_vector(4 downto 2);
        --! Byteenable
        iByteenable : in    std_logic_vector(3 downto 0);
        --! Write data
        iWritedata  : in    std_logic_vector(31 downto 0);
        --! Read data
        oReaddata   : out   std_logic_vector(31 downto 0);
        --! MAC time
        iMacTime    : in    std_logic_vector(gMacTimeWidth-1 downto 0);
        --! Interrupt of first timer (level triggered)
        oIrq        : out   std_logic;
        --! Pulse output of second timer
        oPulse      : out   std_logic
    );
end openmacTimer;

architecture rtl of openmacTimer is
    signal irqEnable                : std_logic;
    signal pulseEnable              : std_logic;
    signal irqCompareValue          : std_logic_vector(iMacTime'range);
    signal pulseCompareValue        : std_logic_vector(iMacTime'range);
    signal pulseWidthCount          : std_logic_vector(gTimerPulseRegWidth-1 downto 0);
    signal pulseWidthCountPreset    : std_logic_vector(gTimerPulseRegWidth-1 downto 0);
    signal pulseWidthCountTc        : std_logic;
    signal irq                      : std_logic;
    signal pulse                    : std_logic;

    constant cPulseWidthCountZero   : std_logic_vector(pulseWidthCount'range) := (others => cInactivated);
begin
    oIrq    <= irq;
    oPulse  <= pulse when gMacTimer_2ndTimer = TRUE else cInactivated;

    pulseWidthCountTc <= cActivated when pulseWidthCountPreset = cPulseWidthCountZero else
                         cActivated when pulseWidthCount = pulseWidthCountPreset else
                         cInactivated;

    --! This process generates the interrupt and pulse signals and handles the
    --! register writes.
    REGPROC : process(iRst, iClk)
    begin
        if iRst = '1' then
            irqEnable       <= cInactivated;
            irqCompareValue <= (others => cInactivated);
            irq             <= cInactivated;

            if gMacTimer_2ndTimer = TRUE then
                pulseEnable         <= cInactivated;
                pulseCompareValue   <= (others => cInactivated);
                pulse               <= cInactivated;
                if gTimerEnablePulseWidth = TRUE then
                    pulseWidthCount         <= (others => cInactivated);
                    pulseWidthCountPreset   <= (others => cInactivated);
                end if;
            end if;
        elsif rising_edge(iClk) then
            -- Interrupt generation (oIrq)
            if irqEnable = cActivated and iMacTime = irqCompareValue then
                irq <= cActivated;
            end if;

            -- Pulse generation (oPulse) without pulse width control
            --  Activates pulse only for one clock cycle (iClk)!
            if gMacTimer_2ndTimer and not gTimerEnablePulseWidth then
                if pulseEnable = cActivated and iMacTime = pulseCompareValue then
                    pulse <= cActivated;
                else
                    pulse <= cInactivated;
                end if;
            end if;

            -- Pulse generation (oPulse) with pulse width control
            --  Pulse stays active until pulseWidthCount expires.
            if gMacTimer_2ndTimer and gTimerEnablePulseWidth then
                if pulseEnable = cActivated then
                    if iMacTime = pulseCompareValue then
                        pulse           <= cActivated;

                        -- The counter starts with value 1, otherwise it would
                        -- counter one additional cycle unnecessarily.
                        pulseWidthCount <= (0 => cActivated, others => cInactivated);
                    elsif pulse = cActivated and pulseWidthCountTc = cActivated then
                        pulse           <= cInactivated;
                    else
                        pulseWidthCount <= std_logic_vector(unsigned(pulseWidthCount) + 1);
                    end if;
                else
                    -- Pull the pulse signal back to zero since it is deactivated.
                    pulse               <= cInactivated;
                end if;
            end if;

            --memory mapping
            if iWrite = '1' then
                case iAddress is
                    when "000" => -- 0x00
                        -- Enable/disable at offset 0x0
                        if iByteenable(0) = cActivated then
                            irqEnable <= iWritedata(0);
                        end if;

                        -- Ack at offset 0x1
                        if iByteenable(1) = cActivated and iWritedata(8) = cActivated then
                            irq <= cInactivated;
                        end if;

                    when "001" => -- 0x04
                        for i in irqCompareValue'range loop
                            if iByteenable(i / cByteLength) = cActivated then
                                irqCompareValue(i)  <= iWritedata(i);
                                irq                 <= cInactivated;
                            end if;
                        end loop;

                    when "010" => -- 0x08
                        null; -- reserved

                    when "011" => -- 0x0C
                        null; -- RO (MACTIME)

                    when "100" => -- 0x10
                        if gMacTimer_2ndTimer = TRUE then
                            if iByteenable(0) = cActivated then
                                pulseEnable <= iWritedata(0);
                            end if;
                        end if;

                    when "101" => -- 0x14
                        if gMacTimer_2ndTimer = TRUE then
                            for i in pulseCompareValue'range loop
                                if iByteenable(i / cByteLength) = cActivated then
                                    pulseCompareValue(i)    <= iWritedata(i);
                                    pulse                   <= cInactivated;
                                end if;
                            end loop;
                        end if;

                    when "110" => -- 0x18
                        if gMacTimer_2ndTimer = TRUE then
                            if gTimerEnablePulseWidth = TRUE then
                                pulseWidthCountPreset <= iWritedata(gTimerPulseRegWidth-1 downto 0);
                            end if;
                        end if;

                    when "111" => -- 0x1C
                        null; -- RO (MACTIME)

                    when others =>
                        assert (FALSE)
                        report "Write in forbidden area?"
                        severity failure;
                end case;
            end if;
        end if;
    end process REGPROC;

    ASSIGN_RD : process (
        iAddress, iMacTime,
        irq, irqEnable, irqCompareValue,
        pulseEnable, pulseCompareValue, pulseWidthCountPreset
    )
    begin
        --default is all zero
        oReaddata <= (others => cInactivated);

        case iAddress is
            when "000" => -- 0x00
                oReaddata(1)    <= irq;
                oReaddata(0)    <= irqEnable;

            when "001" => -- 0x04
                oReaddata(irqCompareValue'range)  <= irqCompareValue;

            when "010" => -- 0x08
                null;

            when "011" => -- 0x0C
                oReaddata(iMacTime'range)   <= iMacTime;

            when "100" => -- 0x10
                if gMacTimer_2ndTimer = TRUE then
                    oReaddata(0)    <= pulseEnable;
                end if;

            when "101" => -- 0x14
                if gMacTimer_2ndTimer = TRUE then
                    oReaddata       <= pulseCompareValue;
                end if;

            when "110" => -- 0x18
                if gMacTimer_2ndTimer = TRUE and gTimerEnablePulseWidth = TRUE then
                    oReaddata(pulseWidthCountPreset'range) <= pulseWidthCountPreset;
                end if;

            when "111" => -- 0x1C
                if gMacTimer_2ndTimer = TRUE then
                    oReaddata(iMacTime'range)   <= iMacTime;
                end if;

            when others =>
                NULL; --this is okay, since default assignment is above!
        end case;
    end process ASSIGN_RD;
end rtl;
