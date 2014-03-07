-------------------------------------------------------------------------------
--! @file nShiftRegRtl.vhd
--
--! @brief Shift register with n-bit-width
--
--! @details This shift register implementation provides a configurable width.
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

entity nShiftReg is
    generic (
        --! Data width
        gWidth : natural := 8;
        --! Number of tabs
        gTabs : natural := 4;
        --! Shift direction ("left" or "right")
        gShiftDir : string := "left"
    );
    port (
        --! Asynchronous reset
        iArst : in std_logic;
        --! Clock
        iClk : in std_logic;
        --! Parallel Load
        iLoad : in std_logic;
        --! Shift Enable
        iShift : in std_logic;
        --! Load Data (gTabs x gWidth)
        iLoadData : in std_logic_vector(gWidth*gTabs-1 downto 0);
        --! Parallel Output Data
        oParData : out std_logic_vector(gWidth*gTabs-1 downto 0);
        --! Input Shift Data
        iData : in std_logic_vector(gWidth-1 downto 0);
        --! Ouptut Shift Data
        oData : out std_logic_vector(gWidth-1 downto 0)
    );
end nShiftReg;

architecture rtl of nShiftReg is
    --! Shift register type
    type tShiftReg is
        array (gTabs-1 downto 0) of std_logic_vector(gWidth-1 downto 0);

    --! Function to convert std_logic_vector into tShiftReg
    function convStdLogicToShiftReg (din : std_logic_vector)
        return tShiftReg is
        variable vTmp : tShiftReg;
    begin
        --default
        vTmp := (others => (others => cInactivated));

        --loop tab-wise
        for i in gTabs-1 downto 0 loop
            vTmp(i) := din((i+1)*gWidth-1 downto i*gWidth);
        end loop;

        return vTmp;
    end function;

    --! Function to convert tShiftReg into std_logic_vector
    function convShiftRegToStdLogic (din : tShiftReg)
        return std_logic_vector is
        variable vTmp : std_logic_vector(gWidth*gTabs-1 downto 0);
    begin
        --default
        vTmp := (others => cInactivated);

        --loop tab-wise
        for i in gTabs-1 downto 0 loop
            vTmp((i+1)*gWidth-1 downto i*gWidth) := din(i);
        end loop;

        return vTmp;
    end function;

    --! Shift register
    signal reg, reg_next : tShiftReg;
begin
    assert (gShiftDir = "left" or gShiftDir = "right") report
        "Set either left or right for shift direction!" severity failure;

    --serial output
    oData <=    reg(reg'right) when gShiftDir = "right" else
                reg(reg'left);

    --parallel output
    oParData <= convShiftRegToStdLogic(reg);

    --! Process doing loading and shifting
    comb : process (
        reg,
        iLoad, iShift,
        iLoadData, iData
    )
    begin
        --default
        reg_next <= reg;

        if iLoad = cActivated then
            reg_next <= convStdLogicToShiftReg(iLoadData);
        elsif iShift = cActivated then
            if gShiftDir = "right" then
                reg_next <= iData & reg(reg'left downto 1);
            else
                reg_next <= reg(reg'left-1 downto 0) & iData;
            end if;
        end if;
    end process;

    --! Register process
    regClk : process(iArst, iClk)
    begin
        if iArst = cActivated then
            reg <= (others => (others => cInactivated));
        elsif rising_edge(iClk) then
            reg <= reg_next;
        end if;
    end process;
end rtl;
