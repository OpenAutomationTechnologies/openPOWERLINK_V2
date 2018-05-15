-------------------------------------------------------------------------------
--! @file phyMgmt-rtl-ea.vhd
--
--! @brief OpenMAC phy management module
--
--! @details This is the openMAC phy management module to configure the connected
--!          phys via SMI (= serial management interface).
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

entity phyMgmt is
    port (
        --! Reset
        iRst                : in    std_logic;
        --! Clock
        iClk                : in    std_logic;
        --! Address (word addresses)
        iAddress            : in    std_logic_vector(3 downto 1);
        --! Select
        iSelect             : in    std_logic;
        --! Byteenale (low-active)
        inByteenable        : in    std_logic_vector(1 downto 0);
        --! Write (low-active)
        inWrite             : in    std_logic;
        --! Writedata
        iWritedata          : in    std_logic_vector(15 downto 0);
        --! Readdata
        oReaddata           : out   std_logic_vector(15 downto 0);
        --! SMI Clock
        oSmiClk             : out   std_logic;
        --! SMI data input
        iSmiDataIn          : in    std_logic;
        --! SMI data output
        oSmiDataOut         : out   std_logic;
        --! SMI data output enable
        oSmiDataOutEnable   : out   std_logic;
        --! Phy reset (low-active)
        onPhyReset          : out   std_logic
    );
end entity phyMgmt;

architecture rtl of phyMgmt is
    --! This is the shift register to serialize write and read data.
    signal  shift_reg           : std_logic_vector(31 downto 0);
    --! This is the generated SMI clock.
    signal  smiClk              : std_logic;
    --! This is the clock divider vector to generate smiClk.
    signal  clkDivider          : std_logic_vector(4 downto 0);
    --! This alias triggers shifting the shift register.
    alias   doShift             : std_logic is clkDivider(clkDivider'high);
    --! This is the bit counter for serializing.
    signal  bit_cnt             : std_logic_vector(2 downto 0);
    --! This is the byte counter for serializing.
    signal  byte_cnt            : std_logic_vector(2 downto 0);
    --! This flag signalizes activity.
    signal  runActive           : std_logic;
    --! This flag signalizes a busy shift register.
    signal  shiftBusy           : std_logic;
    --! This signal is used to control the phy reset (low active).
    signal  nPhyReset           : std_logic;
    --! This is the internal SMI data output.
    signal  smiDataOut          : std_logic;
    --! This is the internal SMI data output enable.
    signal  smiDataOutEnable    : std_logic;
begin
    ---------------------------------------------------------------------------
    -- Assign outputs
    ---------------------------------------------------------------------------
    oSmiClk             <= smiClk;
    oSmiDataOut         <= smiDataOut;
    oSmiDataOutEnable   <= smiDataOutEnable;
    onPhyReset          <= nPhyReset;

    --! This process assigns the readdata vector.
    ASSIGN_READDATA : process (
        nPhyReset,
        shiftBusy,
        shift_reg,
        iAddress
    )
    begin
        -- default is zero
        oReaddata <= (others => cInactivated);

        if iAddress(1) = cInactivated then
            oReaddata(7) <= nPhyReset;
            oReaddata(0) <= shiftBusy;
        else
            oReaddata <= shift_reg(15 downto 0);
        end if;
    end process ASSIGN_READDATA;

    --! This process generates the SMI signals and assigns memory mapped writes.
    doSMI : process (iRst, iClk)
    begin
        if iRst = cActivated    then
            smiClk              <= cInactivated;
            runActive           <= cInactivated;
            shiftBusy           <= cInactivated;
            smiDataOutEnable    <= cActivated;
            smiDataOut          <= cActivated;
            nPhyReset           <= cnActivated;
            bit_cnt             <= (others => cInactivated);
            byte_cnt            <= (others => cInactivated);
            shift_reg           <= x"0000abcd";
            clkDivider          <= (others => cInactivated);
        elsif rising_edge(iClk) then
            if doShift = cActivated then
                clkDivider  <= std_logic_vector(to_unsigned(8, clkDivider'length) + 1);
                smiClk      <= not smiClk;
            else
                clkDivider  <= std_logic_vector(unsigned(clkDivider) - 1);
            end if;

            if (iSelect = cActivated and inWrite = cnActivated and shiftBusy = cInactivated and
                iAddress(2) = cActivated and inByteenable(0) = cnActivated) then
                nPhyReset <= iWritedata(7);
            end if;

            if (iSelect = cActivated and inWrite = cnActivated and shiftBusy = cInactivated and
                iAddress(2) = cInactivated) then
                if iAddress(1) = cInactivated then
                    if inByteenable(1) = cnActivated then
                        shift_reg(31 downto 24) <= iWritedata(15 downto 8);
                    end if;
                    if inByteenable(0) = cnActivated then
                        shift_reg(23 downto 16) <= iWritedata(7 downto 0);
                        shiftBusy <= cActivated;
                    end if;
                else
                    if inByteenable(1) = cnActivated then
                        shift_reg(15 downto 8) <= iWritedata(15 downto 8);
                    end if;
                    if inByteenable(0) = cnActivated then
                        shift_reg(7 downto 0) <= iWritedata(7 downto 0);
                    end if;
                end if;
            else
                if doShift = cActivated and smiClk = cActivated then
                    if runActive = cInactivated and shiftBusy = cActivated then
                        runActive   <= cActivated;
                        byte_cnt    <= "111";
                        bit_cnt     <= "111";
                    else
                        if byte_cnt(2) = cInactivated and shiftBusy = cActivated then
                            smiDataOut  <= shift_reg(31);
                            shift_reg   <= shift_reg(30 downto 0) & iSmiDataIn;
                        end if;

                        bit_cnt <= std_logic_vector(unsigned(bit_cnt) - 1);

                        if bit_cnt = std_logic_vector(to_unsigned(0, bit_cnt'length)) then
                            byte_cnt <= std_logic_vector(unsigned(byte_cnt) - 1);
                            if byte_cnt = std_logic_vector(to_unsigned(0, byte_cnt'length)) then
                                shiftBusy   <= cInactivated;
                                runActive   <= cInactivated;
                            end if;
                        end if;

                        if (byte_cnt = std_logic_vector(to_unsigned(2, byte_cnt'length)) and
                            bit_cnt = std_logic_vector(to_unsigned(1, bit_cnt'length)) and
                            shift_reg(31) = cInactivated) then
                            smiDataOutEnable   <= cInactivated;
                        end if;
                    end if;
                    if shiftBusy = cInactivated or runActive = cInactivated then
                        smiDataOut <= cActivated;
                        smiDataOutEnable   <= cActivated;
                    end if;
                end if;
            end if;
        end if;
    end process doSMI;
end rtl;