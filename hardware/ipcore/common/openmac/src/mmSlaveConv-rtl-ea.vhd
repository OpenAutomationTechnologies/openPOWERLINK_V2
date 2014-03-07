-------------------------------------------------------------------------------
--! @file mmSlaveConv-rtl-ea.vhd
--
--! @brief Memory mapped slave interface converter
--
--! @details The slave interface converter is fixed to a 16 bit memory mapped
--!          slave, connected to a 32 bit master. The conversion also considers
--!          little/big endian (gEndian).
--!          Note: Tested with openmacTop entity only!
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

entity mmSlaveConv is
    generic (
        --! Endianness of interconnect
        gEndian             : string := "little";
        --! Memory mapped master address width
        gMasterAddrWidth    : natural := 10
    );
    port (
        --! Reset
        iRst                : in    std_logic;
        --! Clock
        iClk                : in    std_logic;
        -- Memory mapped master input
        --! Master select
        iMaster_select      : in    std_logic;
        --! Master write
        iMaster_write       : in    std_logic;
        --! Master read
        iMaster_read        : in    std_logic;
        --! Master byteenable
        iMaster_byteenable  : in    std_logic_vector(3 downto 0);
        --! Master writedata
        iMaster_writedata   : in    std_logic_vector(31 downto 0);
        --! Master readdata
        oMaster_readdata    : out   std_logic_vector(31 downto 0);
        --! Master address (byte address)
        iMaster_address     : in    std_logic_vector(gMasterAddrWidth-1 downto 0);
        --! Master write acknowledge
        oMaster_WriteAck    : out   std_logic;
        --! Master read acknowledge
        oMaster_ReadAck     : out   std_logic;
        -- Memory mapped slave output
        --! Slave select
        oSlave_select       : out   std_logic;
        --! Slave write
        oSlave_write        : out   std_logic;
        --! Slave read
        oSlave_read         : out   std_logic;
        --! Slave address (word address)
        oSlave_address      : out   std_logic_vector(gMasterAddrWidth-1 downto 0);
        --! Slave byteenable
        oSlave_byteenable   : out   std_logic_vector(1 downto 0);
        --! Slave readdata
        iSlave_readdata     : in    std_logic_vector(15 downto 0);
        --! Slave writedata
        oSlave_writedata    : out   std_logic_vector(15 downto 0);
        --! Slave acknowledge
        iSlave_ack          : in    std_logic
    );
end mmSlaveConv;

architecture rtl of mmSlaveConv is
    --! Access fsm_reg type
    type tAccessFsm is (
        sIdle,
        sDoAccess
    );

    --! Access type
    type tAccess is (
        sNone,
        sDword,
        sWord
    );

    --! Access fsm_reg current state
    signal fsm_reg      : tAccessFsm;
    --! Access fsm_reg next state
    signal fsm_next     : tAccessFsm;
    --! Current master access type
    signal masterAccess : tAccess;

    --! Counter width
    constant cCounterWidth      : natural := 2;
    --! Counter register
    signal counter_reg          : std_logic_vector(cCounterWidth-1 downto 0);
    --! Next counter register
    signal counter_next         : std_logic_vector(cCounterWidth-1 downto 0);
    --! Counter register load value
    signal counter_loadValue    : std_logic_vector(cCounterWidth-1 downto 0);
    --! Load counter register with counter_loadValue
    signal counter_load         : std_logic;
    --! Decrement counter value by one
    signal counter_decrement    : std_logic;
    --! counter_reg is zero
    signal counter_isZero       : std_logic;
    --! counter_reg is one
    signal counter_isOne        : std_logic;
    --! counter_reg is two
    signal counter_isTwo        : std_logic;
    --! Master acknowledge
    signal masterAck            : std_logic;
    --! Register to store slave readdata word
    signal wordStore_reg        : std_logic_vector(iSlave_readdata'range);
    --! Next value of slave readdata word register
    signal wordStore_next       : std_logic_vector(wordStore_reg'range);
begin
    ---------------------------------------------------------------------------
    -- Assign outputs
    ---------------------------------------------------------------------------
    oSlave_select       <= iMaster_select;
    oSlave_write        <= iMaster_write and iMaster_select;
    oSlave_read         <= iMaster_read and iMaster_select;
    oMaster_WriteAck    <= masterAck and iMaster_write and iMaster_select;
    oMaster_ReadAck     <= masterAck and iMaster_read and iMaster_select;

    --! This process assigns the master readdata port controlled by the current
    --! conversion state.
    assignMasterPath : process (
        iSlave_readdata, wordStore_reg,
        masterAccess
    )
    begin
        if masterAccess = sDword then
            oMaster_readdata <= iSlave_readdata & wordStore_reg;
        else
            oMaster_readdata <= iSlave_readdata & iSlave_readdata;
        end if;
    end process assignMasterPath;

    --! This process assigns the slave address, byteenable and writedata controlled
    --! by the current conversion state.
    assignSlavePath : process (
        iMaster_address, iMaster_byteenable, iMaster_writedata,
        counter_reg, counter_isOne,
        masterAccess
    )
    begin
        -----------------------------------------------------------------------
        -- Slave address
        -----------------------------------------------------------------------
        --default assignment
        oSlave_address  <= iMaster_address;

        if masterAccess = sDword then
            case to_integer(unsigned(counter_reg)) is
                when 0 | 2 =>
                    -- First word of dword access
                    if gEndian = "little" then
                        oSlave_address(1) <= cInactivated;
                    else
                        oSlave_address(1) <= cActivated;
                    end if;
                when 1 =>
                    -- Second word of dword access
                    if gEndian = "little" then
                        oSlave_address(1) <= cActivated;
                    else
                        oSlave_address(1) <= cInactivated;
                    end if;
                when others =>
                    null; --allowed due to default assignment
            end case;
        end if;

        -----------------------------------------------------------------------
        -- Slave byteenable
        -----------------------------------------------------------------------
        if masterAccess = sDword then
            oSlave_byteenable <= (others => cActivated);
        else
            oSlave_byteenable <= iMaster_byteenable(3 downto 2) or iMaster_byteenable(1 downto 0);
        end if;

        -----------------------------------------------------------------------
        -- Slave writedata
        -----------------------------------------------------------------------
        if (masterAccess = sDword and counter_isOne = cActivated) or iMaster_address(1) = cActivated then
            oSlave_writedata <= iMaster_writedata(31 downto 16);
        else
            oSlave_writedata <= iMaster_writedata(15 downto 0);
        end if;
    end process assignSlavePath;

    --! This process assigns the registers.
    regProc : process(iRst, iClk)
    begin
        if iRst = cActivated then
            counter_reg     <= (others => cInactivated);
            fsm_reg         <= sIdle;
            wordStore_reg   <= (others => cInactivated);
        elsif rising_edge(iClk) then
            counter_reg     <= counter_next;
            fsm_reg         <= fsm_next;
            wordStore_reg   <= wordStore_next;
        end if;
    end process;

    --! This process assigns the register next signals.
    assignRegNext : process (
        iSlave_readdata, iSlave_ack,
        wordStore_reg, fsm_reg, counter_reg,
        counter_load, counter_loadValue, counter_decrement, counter_isZero,
        counter_isTwo, masterAccess
    )
    begin
        -- default assignments
        wordStore_next  <= wordStore_reg;
        fsm_next        <= fsm_reg;
        counter_next    <= counter_reg;

        -----------------------------------------------------------------------
        -- Counter
        -----------------------------------------------------------------------
        if counter_load = cActivated then
            counter_next <= counter_loadValue;
        elsif counter_decrement = cActivated and masterAccess = sDword then
            counter_next <= std_logic_vector(unsigned(counter_reg) - 1);
        end if;

        -----------------------------------------------------------------------
        -- Access FSM
        -----------------------------------------------------------------------
        if counter_isZero = cActivated then
            case fsm_reg is
                when sIdle =>
                    if masterAccess = sDword then
                        fsm_next <= sDoAccess;
                    end if;
                when sDoAccess =>
                    if masterAccess = sNone then
                        fsm_next <= sIdle;
                    end if;
            end case;
        end if;

        -----------------------------------------------------------------------
        -- Store slave readdata word
        -----------------------------------------------------------------------
        if iSlave_ack = cActivated and masterAccess = sDword and counter_isTwo = cActivated then
            wordStore_next <= iSlave_readdata;
        end if;
    end process assignRegNext;

    counter_decrement <= iSlave_ack and iMaster_select;

    --! This process assigns internal control signals.
    assignInternal : process (
        iSlave_ack,
        iMaster_select, iMaster_byteenable, iMaster_read,
        counter_reg, counter_isOne, masterAccess, fsm_reg, fsm_next
    )
    begin
        -----------------------------------------------------------------------
        -- Master acknowledge
        -----------------------------------------------------------------------
        if iSlave_ack = cActivated and masterAccess = sDword and counter_isOne = cActivated then
            masterAck <= cActivated;
        elsif iSlave_ack = cActivated and masterAccess = sWord then
            masterAck <= cActivated;
        else
            masterAck <= cInactivated;
        end if;

        -----------------------------------------------------------------------
        -- Master access state
        -----------------------------------------------------------------------
        if iMaster_select = cInactivated then
            masterAccess <= sNone;
        elsif iMaster_byteenable = "1111" then
            masterAccess <= sDword;
        else
            masterAccess <= sWord;
        end if;

        -----------------------------------------------------------------------
        -- Counter
        -----------------------------------------------------------------------
        --default
        counter_isZero  <= cInactivated;
        counter_isOne   <= cInactivated;
        counter_isTwo   <= cInactivated;

        -- assign counter_is* signals
        case to_integer(unsigned(counter_reg)) is
            when 0 =>
                counter_isZero  <= cActivated;
            when 1 =>
                counter_isOne   <= cActivated;
            when 2 =>
                counter_isTwo   <= cActivated;
            when others =>
                null; --is allowed due to default assignment
        end case;

        -- assign counter load
        if fsm_next = sDoAccess and fsm_reg = sIdle then
            counter_load <= cActivated;
        else
            counter_load <= cInactivated;
        end if;

        -- assign counter load value
        if iMaster_byteenable = "1111" and iMaster_read = cActivated then
            counter_loadValue <= "10";
        else
            counter_loadValue <= "01";
        end if;
    end process assignInternal;
end rtl;
