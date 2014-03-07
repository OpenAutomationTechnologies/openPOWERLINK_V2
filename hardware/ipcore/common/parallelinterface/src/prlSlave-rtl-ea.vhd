-------------------------------------------------------------------------------
--! @file prlSlave-rtl-ea.vhd
--! @brief Multiplexed memory mapped slave
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

--! Use standard ieee library
library ieee;
--! Use logic elements
use ieee.std_logic_1164.all;
--! Use numeric std
use ieee.numeric_std.all;

--! Use libcommon library
library libcommon;
--! Use global package
use libcommon.global.all;

entity prlSlave is
    generic (
        --! Enable multiplexed address/data-bus mode (0 = FALSE)
        gEnableMux      : natural := 0;
        --! Data bus width
        gDataWidth      : natural := 16;
        --! Address bus width
        gAddrWidth      : natural := 16;
        --! Ad bus width (valid when gEnableMux /= FALSE)
        gAdWidth        : natural := 16
    );
    port (
        --! Clock
        iClk                : in    std_logic;
        --! Reset
        iRst                : in    std_logic;
        -- Memory mapped multiplexed slave
        --! Chipselect
        iPrlSlv_cs          : in    std_logic;
        --! Read strobe
        iPrlSlv_rd          : in    std_logic;
        --! Write strobe
        iPrlSlv_wr          : in    std_logic;
        --! Address Latch enable (Multiplexed only)
        iPrlSlv_ale         : in    std_logic;
        --! High active Acknowledge
        oPrlSlv_ack         : out   std_logic;
        --! Byteenables
        iPrlSlv_be          : in    std_logic_vector(gDataWidth/8-1 downto 0);
        -- Multiplexed AD-bus
        --! Address/Data bus out
        oPrlSlv_ad_o        : out   std_logic_vector(gAdWidth-1 downto 0);
        --! Address/Data bus in
        iPrlSlv_ad_i        : in    std_logic_vector(gAdWidth-1 downto 0);
        --! Address/Data bus outenable
        oPrlSlv_ad_oen      : out   std_logic;
        -- Demultiplexed AD-bus
        --! Address bus
        iPrlSlv_addr        : in    std_logic_vector(gAddrWidth-1 downto 0);
        --! Data bus in
        iPrlSlv_data_i      : in    std_logic_vector(gDataWidth-1 downto 0);
        --! Data bus out
        oPrlSlv_data_o      : out   std_logic_vector(gDataWidth-1 downto 0);
        --! Data bus outenable
        oPrlSlv_data_oen    : out   std_logic;
        -- Memory Mapped master
        --! MM slave host address
        oMst_address        : out   std_logic_vector(gAddrWidth-1 downto 0);
        --! MM slave host byteenable
        oMst_byteenable     : out   std_logic_vector(gDataWidth/8-1 downto 0);
        --! MM slave host read
        oMst_read           : out   std_logic;
        --! MM slave host readdata
        iMst_readdata       : in    std_logic_vector(gDataWidth-1 downto 0);
        --! MM slave host write
        oMst_write          : out   std_logic;
        --! MM slave host writedata
        oMst_writedata      : out   std_logic_vector(gDataWidth-1 downto 0);
        --! MM slave host waitrequest
        iMst_waitrequest    : in    std_logic
    );
end prlSlave;

architecture rtl of prlSlave is
    -- address register to store the address populated to the interface
    signal addressRegister      : std_logic_vector(gAddrWidth-1 downto 0);

    -- byteenable register to store byteenable qualifiers
    signal byteenableRegister       : std_logic_vector(gDataWidth/8-1 downto 0);
    -- register clock enable
    signal byteenableRegClkEnable   : std_logic;

    -- write data register to store the data populated to the interface
    signal writeDataRegister        : std_logic_vector(gDataWidth-1 downto 0);
    -- register clock enable
    signal writeDataRegClkEnable    : std_logic;

    -- read data register to store the read data populated to the host
    signal readDataRegister         : std_logic_vector(gDataWidth-1 downto 0);
    signal readDataRegister_next    : std_logic_vector(gDataWidth-1 downto 0);

    -- synchronized signals
    signal hostChipselect   : std_logic;
    signal hostWrite        : std_logic;
    signal hostWrite_noCs   : std_logic;
    signal hostRead         : std_logic;
    signal hostRead_noCs    : std_logic;

    signal hostDataEnable       : std_logic;
    signal hostDataEnable_reg   : std_logic;
    signal hostAck              : std_logic;
    signal hostAck_reg          : std_logic;

    -- fsm
    type tFsm is (
        sIdle,
        sStart,
        sWaitForBus,
        sHold
    );

    signal fsm : tFsm;

    -- Latch type
    type tLatch is record
        clear   : std_logic;
        enable  : std_logic;
        data    : std_logic_vector(gAddrWidth-1 downto 0);
        output  : std_logic_vector(gAddrWidth-1 downto 0);
    end record;

    signal inst_latch : tLatch;
begin

    --! The processes describe the register, which stores the unsynchronized
    --! inputs!
    reg : process(iRst, iClk)
    begin
        if iRst = cActivated then
            addressRegister     <= (others => cInactivated);
            byteenableRegister  <= (others => cInactivated);
            writeDataRegister   <= (others => cInactivated);
            readDataRegister    <= (others => cInactivated);
            hostDataEnable_reg  <= cInactivated;
            hostAck_reg         <= cInactivated;
        elsif rising_edge(iClk) then
            hostDataEnable_reg  <= hostDataEnable;
            hostAck_reg         <= hostAck;

            if byteenableRegClkEnable = cActivated then
                byteenableRegister <= iPrlSlv_be;

                -- Assign byte addresses to the address register
                if gEnableMux /= 0 then
                    addressRegister <= (others => cInactivated);
                    addressRegister <= inst_latch.output;
                else
                    addressRegister <= iPrlSlv_addr;
                end if;
            end if;

            if writeDataRegClkEnable = cActivated then
                if gEnableMux /= 0 then
                    writeDataRegister <= iPrlSlv_ad_i(writeDataRegister'range);
                else
                    writeDataRegister <= iPrlSlv_data_i;
                end if;
            end if;

            if iMst_waitrequest = cInactivated and hostRead = cActivated then
                readDataRegister <= readDataRegister_next;
            end if;
        end if;
    end process;

    oMst_address    <= addressRegister;

    -- Multiplexed output
    oPrlSlv_ad_oen  <= hostDataEnable_reg;
    oPrlSlv_ack     <= hostAck_reg;

    -- Demultiplexed output
    oPrlSlv_data_oen    <= hostDataEnable_reg;

    assignReaddata : process(readDataRegister)
    begin
        -- default assign zeros
        oPrlSlv_ad_o                            <= (others => cInactivated);
        oPrlSlv_data_o                          <= (others => cInactivated);

        oPrlSlv_ad_o(readDataRegister'range)    <= readDataRegister;
        oPrlSlv_data_o(readDataRegister'range)  <= readDataRegister;
    end process assignReaddata;

    --! combinatoric process for ack and output enable generation
    combProc : process (
        hostWrite,
        hostRead,
        fsm
    )
    begin
        -- default assignments to avoid unwanted latches
        hostAck         <= cInactivated;
        hostDataEnable  <= cInactivated;

        if fsm = sHold then
            if hostRead = cActivated then
                hostDataEnable  <= cActivated;
                hostAck         <= cActivated;
            elsif hostWrite = cActivated then
                hostAck         <= cActivated;
            end if;
        end if;
    end process;

    --! Fsm to control access and timeout counter
    fsmProc : process(iRst, iClk)
    begin
        if iRst = cActivated then
            fsm                     <= sIdle;
            byteenableRegClkEnable  <= cInactivated;
            writeDataRegClkEnable   <= cInactivated;
            oMst_write              <= cInactivated;
            oMst_read               <= cInactivated;
        elsif rising_edge(iClk) then
            --defaults
            byteenableRegClkEnable  <= cInactivated;
            writeDataRegClkEnable   <= cInactivated;

            case fsm is
                when sIdle =>
                    oMst_write                  <= cInactivated;
                    oMst_read                   <= cInactivated;
                    if hostRead = cActivated or hostWrite = cActivated then
                        fsm                     <= sStart;
                        byteenableRegClkEnable  <= cActivated;
                        writeDataRegClkEnable   <= hostWrite;
                    end if;
                when sStart =>
                    fsm         <= sWaitForBus;
                    oMst_read   <= hostRead;
                    oMst_write  <= hostWrite;
                when sWaitForBus =>
                    if iMst_waitrequest = cInactivated then
                        fsm         <= sHold;
                        oMst_read   <= cInactivated;
                        oMst_write  <= cInactivated;
                    end if;
                when sHold =>
                    if hostRead = cInactivated and hostWrite = cInactivated then
                        fsm <= sIdle;
                    end if;
            end case;
        end if;
    end process;

    oMst_byteenable         <= byteenableRegister;
    oMst_writedata          <= writeDataRegister;
    readDataRegister_next   <= iMst_readdata;

    muxLatch : if gEnableMux /= 0 generate
        -- Address latch
        addrLatch : entity work.dataLatch
            generic map (
                gDataWidth => inst_latch.data'length
            )
            port map (
                iClear  => inst_latch.clear,
                iEnable => inst_latch.enable,
                iData   => inst_latch.data,
                oData   => inst_latch.output
            );

        inst_latch.clear    <= cInactivated;
        inst_latch.enable   <= iPrlSlv_ale;
        inst_latch.data     <= iPrlSlv_ad_i(inst_latch.data'range);
    end generate muxLatch;

    -- synchronize all available control signals
    syncChipselect : entity libcommon.synchronizer
        generic map (
            gStages => 2,
            gInit   => cInactivated
        )
        port map (
            iArst   => iRst,
            iClk    => iClk,
            iAsync  => iPrlSlv_cs,
            oSync   => hostChipselect
        );

    syncWrite : entity libcommon.synchronizer
        generic map (
            gStages => 2,
            gInit   => cInactivated
        )
        port map (
            iArst   => iRst,
            iClk    => iClk,
            iAsync  => iPrlSlv_wr,
            oSync   => hostWrite_noCs
        );

    hostWrite <= hostChipselect and hostWrite_noCs;

    syncRead : entity libcommon.synchronizer
        generic map (
            gStages => 2,
            gInit   => cInactivated
        )
        port map (
            iArst   => iRst,
            iClk    => iClk,
            iAsync  => iPrlSlv_rd,
            oSync   => hostRead_noCs
        );

    hostRead <= hostChipselect and hostRead_noCs;
end rtl;
