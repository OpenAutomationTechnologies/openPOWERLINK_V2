-------------------------------------------------------------------------------
--! @file parallelInterfaceRtl.vhd
--
--! @brief Parallel Interface for Host Interface
--
--! @details This is the parallel interface implementation for
--!          the host interface suitable for inbuilt Xilinx EPC.
--
-------------------------------------------------------------------------------
--
--    Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
--    Copyright (c) 2014, Kalycito Infotech Private Limited.
--    All rights reserved.
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
--! Use standard ieee library
library ieee;
--! Use logic elements
use ieee.std_logic_1164.all;
--! Use numerics
use ieee.numeric_std.all;

--! Common library
library libcommon;
--! Use common library global package
use libcommon.global.all;

--! Work library
library work;
--! use host interface package for specific types
use work.hostInterfacePkg.all;

entity parallelInterface is
    generic (
        --! Data bus width
        gDataWidth  : natural := 16;
        --! Address and Data bus are multiplexed (0 = FALSE, otherwise = TRUE)
        gMultiplex  : natural := 0
    );
    port (
        -- Parallel Interface
        --! Chip select
        iParHostChipselect          : in std_logic := cInactivated;
        --! Read strobe
        iParHostRead                : in std_logic := cInactivated;
        --! Write strobe
        iParHostWrite               : in std_logic := cInactivated;
        --! Address Latch enable (Multiplexed only)
        iParHostAddressLatchEnable  : in std_logic := cInactivated;
        --! High active Acknowledge
        oParHostAcknowledge         : out std_logic := cInactivated;
        --! Byte enable
        iParHostByteenable          : in std_logic_vector(gDataWidth/cByte-1 downto 0) := (others => cInactivated);
        --! Address bus (De-multiplexed, word-address)
        iParHostAddress             : in std_logic_vector(15 downto 0) := (others => cInactivated);
        --! Data bus out (De-multiplexed)
        oParHostData                : out std_logic_vector(gDataWidth-1 downto 0) := (others => cInactivated);
        --! Data bus in (De-multiplexed)
        iParHostData                : in std_logic_vector(gDataWidth-1 downto 0) := (others => cInactivated);
        --! Data bus output enable (De-multiplexed)
        oParHostDataEnable          : out std_logic;
        --! Address/Data bus out (Multiplexed, word-address))
        oParHostAddressData         : out std_logic_vector(gDataWidth-1 downto 0) := (others => cInactivated);
        --! Address/Data bus in (Multiplexed, word-address))
        iParHostAddressData         : in std_logic_vector(gDataWidth-1 downto 0) := (others => cInactivated);
        --! Address/Data bus output enable (Multiplexed, word-address))
        oParHostAddressDataEnable   : out std_logic;
        -- Clock/Reset sources
        --! Clock Source input
        iClk                        : in std_logic:= cInactivated;
        --! Reset Source input
        iRst                        : in std_logic:= cInactivated;
        -- Memory Mapped Slave for Host
        --! MM slave host address
        oHostAddress                : out std_logic_vector(16 downto 2) := (others => cInactivated);
        --! MM slave host byte enable
        oHostByteenable             : out std_logic_vector(3 downto 0) := (others => cInactivated);
        --! MM slave host read
        oHostRead                   : out std_logic := cInactivated;
        --! MM slave host read data
        iHostReaddata               : in std_logic_vector(31 downto 0) := (others => cInactivated);
        --! MM slave host write
        oHostWrite                  : out std_logic := cInactivated;
        --! MM slave host write data
        oHostWritedata              : out std_logic_vector(31 downto 0) := (others => cInactivated);
        --! MM slave host wait request
        iHostWaitrequest            : in std_logic := cInactivated
    );
end parallelInterface;

architecture rtl of parallelInterface is
    --! address register to store the address populated to the interface
    signal addressRegister      : std_logic_vector(iParHostAddress'range);
    --! register clock enable
    signal addressRegClkEnable  : std_logic;

    --! byte enable register to store byte enable qualifiers
    signal byteenableRegister       : std_logic_vector(gDataWidth/cByte-1 downto 0);
    --! register clock enable
    signal byteenableRegClkEnable   : std_logic;

    --! write data register to store the data populated to the interface
    signal writeDataRegister        : std_logic_vector(gDataWidth-1 downto 0);
    --! register clock enable
    signal writeDataRegClkEnable    : std_logic;

    --! read data register to store the read data populated to the host
    signal readDataRegister         : std_logic_vector(gDataWidth-1 downto 0);
    --! temporary readDataRegister
    signal readDataRegister_next    : std_logic_vector(gDataWidth-1 downto 0);

    -- synchronized signals
    --! Synchronized chip select signal
    signal hostChipselect   : std_logic;
    --! Write signal for initialize the transfer
    signal hostWrite        : std_logic;
    --! Synchronized Write signal
    signal hostWrite_noCs   : std_logic;
    --! Read signal for initialize transfer
    signal hostRead         : std_logic;
    --! Synchronized Read signal
    signal hostRead_noCs    : std_logic;
    --! Address latch Enable
    signal hostAle          : std_logic;
    --! Synchronized Address Latch Enable signal
    signal hostAle_noCs     : std_logic;
    --!
    signal hostAle_noCsEdge : std_logic;
    --! Data Enable
    signal hostDataEnable       : std_logic;
    --! Registered data Enable
    signal hostDataEnable_reg   : std_logic;
    --! Transfer complete Acknowledgement signal
    signal hostAck              : std_logic;
    --! Transfer complete Acknowledgement signal for registering
    signal hostAck_reg          : std_logic;

    -- fsm
    --! FSM state for Parallel Interface
    type tFsm is (sIdle,
                  sDo,
                  sWait,
                  sDone
            );
    --! state signal
    signal fsm : tFsm;

    -- Counter will enable only after the wait request gets activated.
    --! timeout counter width
    constant cCountWidth    : natural := 4;
    --! Timeout counter
    signal count            : std_logic_vector(cCountWidth-1 downto 0);
    --! MSB of timeout counter
    alias countTc           : std_logic is count(cCountWidth-1);
    --! Enable counter
    signal countEn          : std_logic;
    --! Reset counter
    signal countRst         : std_logic;
    --! Enable ACK for Write operations
    constant cCountWrAckAct : std_logic_vector(count'range) := "0000"; --0
    --! Disable ACK for Write operations
    constant cCountWrAckDea : std_logic_vector(count'range) := "0111";--1
    --! Enable ACK for Read operations
    constant cCountRdAckAct : std_logic_vector(count'range) := "0010";--1
    --! Disable ACK for Read operations
    constant cCountRdAckDea : std_logic_vector(count'range) := "0111";--2

begin

    --! The processes describe the register, which store the unsynchronized
    --! inputs!
    reg : process(iClk)
    begin
        if rising_edge(iClk) then
            if iRst = cActivated then
                addressRegister <= (others => cInactivated);
                byteenableRegister <= (others => cInactivated);
                writeDataRegister <= (others => cInactivated);
                readDataRegister <= (others => cInactivated);
                hostDataEnable_reg <= cInactivated;
                hostAck_reg <= cInactivated;
            else

                hostDataEnable_reg <= hostDataEnable;
                hostAck_reg <= hostAck;

                if byteenableRegClkEnable = cActivated then
                    byteenableRegister <= iParHostByteenable;
                end if;

                if addressRegClkEnable = cActivated then
                    if gMultiplex = 0 then
                        addressRegister <= iParHostAddress;
                    else
                        addressRegister <= iParHostAddressData;
                    end if;
                end if;

                if writeDataRegClkEnable = cActivated then
                    if gMultiplex = 0 then
                        writeDataRegister <= iParHostData;
                    else
                        writeDataRegister <= iParHostAddressData;
                    end if;
                end if;

                if iHostWaitrequest = cInactivated and hostRead = cActivated then
                    readDataRegister <= readDataRegister_next;
                end if;
            end if;
        end if;
    end process;

    oHostAddress <= addressRegister(15 downto 1);

    oParHostDataEnable          <= hostDataEnable_reg;
    oParHostAddressDataEnable   <= hostDataEnable_reg;
    oParHostAcknowledge         <= hostAck_reg;

    oParHostAddressData <= readDataRegister;
    oParHostData        <= readDataRegister;

    countRst    <= cActivated when fsm = sIdle else cInactivated;
    countEn     <= cActivated when fsm = sWait else cInactivated;

    --! combinatoric process for ack and output enable generation
    combProc : process (
        count,
        hostWrite,
        hostRead,
        fsm
    )
    begin
        -- default assignments to avoid unwanted latches
        hostAck         <= cInactivated;
        hostDataEnable  <= cInactivated;

        if fsm = sWait then
            if hostRead = cActivated then
                -- activate ack signal for read
                if count >= cCountRdAckAct and count <= cCountRdAckDea then
                    hostAck <= cActivated;
                end if;
            elsif hostWrite = cActivated then
                -- activate ack signal for write
                if count >= cCountWrAckAct and count <= cCountWrAckDea then
                    hostAck <= cActivated;
                end if;
            end if;
        end if;
        -- Keep Data available at Bus until the read operations ends at Master
        -- side
        if fsm = sWait or fsm = sDone then
            if hostRead = cActivated then
               hostDataEnable <= cActivated;
            end if;
        end if;
    end process;

    --! Fsm to control access and timeout counter
    fsmProc : process(iClk)
    begin
        if rising_edge(iClk) then
            if iRst = cActivated then
                fsm                     <= sIdle;
                addressRegClkEnable     <= cInactivated;
                byteenableRegClkEnable  <= cInactivated;
                writeDataRegClkEnable   <= cInactivated;
                oHostWrite              <= cInactivated;
                oHostRead               <= cInactivated;
                count                   <= (others => cInactivated);
            else

                if countRst = cActivated then
                    count <= (others => cInactivated);
                elsif countEn = cActivated and countTc /= cActivated then
                    count <= std_logic_vector(unsigned(count) + 1);
                end if;

                --defaults
                addressRegClkEnable     <= cInactivated;
                byteenableRegClkEnable  <= cInactivated;
                writeDataRegClkEnable   <= cInactivated;
                oHostWrite              <= cInactivated;
                oHostRead               <= cInactivated;

                if hostAle = cActivated and gMultiplex /= 0 then
                    addressRegClkEnable <= cActivated;
                end if;

                case fsm is
                   --Start the operations if Read/write activated by Master
                    when sIdle =>
                        if hostRead = cActivated or hostWrite = cActivated then
                            fsm <= sDo;
                            if gMultiplex = 0 then
                                addressRegClkEnable <= cActivated;
                            end if;
                            byteenableRegClkEnable  <= cActivated;
                            writeDataRegClkEnable   <= hostWrite;
                        end if;
                    --Wait for the response from Avalon side
                    when sDo =>
                        oHostRead   <= hostRead;
                        oHostWrite  <= hostWrite;
                        if iHostWaitrequest = cInactivated then
                            fsm         <= sWait;
                            oHostRead   <= cInactivated;
                            oHostWrite  <= cInactivated;
                        end if;
                    -- Generate ACK signals
                    when sWait =>
                        if countTc = cActivated then
                            fsm <= sDone;
                        end if;
                    --Wait for transfer to end at Parallel Master side
                    when sDone =>
                         if (hostRead =  cInactivated and hostWrite = cInactivated) then
                            fsm <= sIdle;
                         else
                            fsm <= sDone;
                        end if;
                end case;
            end if;
        end if;
    end process;

    -- Generate signals for DWORD data width
    genHostBusDword : if gDataWidth = cDword generate
    begin
        oHostByteenable         <= byteenableRegister;
        oHostWritedata          <= writeDataRegister;
        readDataRegister_next   <= iHostReaddata;
    end generate;

    -- Generate signals for WORD data width
    genHostBusWord : if gDataWidth = cWord generate
    begin
        oHostWritedata <= writeDataRegister & writeDataRegister;

        --! Create ByteEnable and Read data from WORD based signals
        busCombProc : process (
            byteenableRegister,
            addressRegister,
            iHostReaddata
        )
        begin
            --default assignments (to avoid evil latches)
            oHostByteenable         <= (others => cInactivated);
            readDataRegister_next   <= (others => cInactivated);

            -- assign byte enable to lower/upper word
            for i in gDataWidth/8-1 downto 0 loop
                if addressRegister(addressRegister'right) = cActivated then
                    -- upper word is selected
                    oHostByteenable(cWord/cByte+i) <= byteenableRegister(i);
                else
                    -- lower word is selected
                    oHostByteenable(i) <= byteenableRegister(i);
                end if;
            end loop;

            -- assign lower/upper word to output
            for i in gDataWidth-1 downto 0 loop
                if addressRegister(addressRegister'right) = cActivated then
                    -- upper word is selected
                    readDataRegister_next(i) <= iHostReaddata(cWord+i);
                else
                    -- lower word is selected
                    readDataRegister_next(i) <= iHostReaddata(i);
                end if;
            end loop;
        end process;
    end generate;

    -- synchronize all available control signals
    --! Two synchronizer for ChipSelect
    syncChipselect : entity libcommon.synchronizer
        generic map (
            gStages => 2,
            gInit   => cInactivated
        )
        port map (
            iArst   => iRst,
            iClk    => iClk,
            iAsync  => iParHostChipselect,
            oSync   => hostChipselect
        );

    --! Two synchronizer for Write
    syncWrite : entity libcommon.synchronizer
        generic map (
            gStages => 2,
            gInit   => cInactivated
        )
        port map (
            iArst   => iRst,
            iClk    => iClk,
            iAsync  => iParHostWrite,
            oSync   => hostWrite_noCs
        );

    hostWrite <= hostChipselect and hostWrite_noCs;

    --! Two synchronizer for Read
    syncRead : entity libcommon.synchronizer
        generic map (
            gStages => 2,
            gInit   => cInactivated
        )
        port map (
            iArst   => iRst,
            iClk    => iClk,
            iAsync  => iParHostRead,
            oSync   => hostRead_noCs
        );

    hostRead <= hostChipselect and hostRead_noCs;

    genSyncAle : if gMultiplex /= 0 generate
    begin
        --! Two synchronizer for ALE
        syncAle : entity libcommon.synchronizer
        generic map (
            gStages => 2,
            gInit   => cInactivated
        )
        port map (
            iArst   => iRst,
            iClk    => iClk,
            iAsync  => iParHostAddressLatchEnable,
            oSync   => hostAle_noCs
        );

        --! Edge Detector for ALE
        edgeAle : entity libcommon.edgedetector
        port map (
            iArst       => iRst,
            iClk        => iClk,
            iEnable     => cActivated,
            iData       => hostAle_noCs,
            oRising     => hostAle_noCsEdge,
            oFalling    => open,
            oAny        => open
        );

        hostAle <= hostChipselect and hostAle_noCsEdge;
    end generate;
end rtl;
