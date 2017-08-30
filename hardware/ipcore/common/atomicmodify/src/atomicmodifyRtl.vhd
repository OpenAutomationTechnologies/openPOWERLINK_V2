-------------------------------------------------------------------------------
--! @file atomicmodifyRtl.vhd
--
--! @brief Atomic modify
--
--! @details This component is used to modify memory atomically.
-------------------------------------------------------------------------------
--
--    (c) B&R, 2015
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

entity atomicmodify is
    generic (
        --! Address bus width
        gAddrWidth  : natural := 16
    );
    port (
        --! Clock
        iClk                : in    std_logic;
        --! Reset
        iRst                : in    std_logic;
        -- Memory Mapped master
        --! MM master address
        oMst_address        : out   std_logic_vector(gAddrWidth-1 downto 0);
        --! MM master byteenable
        oMst_byteenable     : out   std_logic_vector(3 downto 0);
        --! MM master read
        oMst_read           : out   std_logic;
        --! MM master readdata
        iMst_readdata       : in    std_logic_vector(31 downto 0);
        --! MM master write
        oMst_write          : out   std_logic;
        --! MM master writedata
        oMst_writedata      : out   std_logic_vector(31 downto 0);
        --! MM master waitrequest
        iMst_waitrequest    : in    std_logic;
        --! MM master lock
        oMst_lock           : out   std_logic;
        -- Memory mapped slave
        --! Address
        iSlv_address        : in    std_logic_vector(gAddrWidth-1 downto 2);
        --! Byteenable
        iSlv_byteenable     : in    std_logic_vector(3 downto 0);
        --! Read strobe
        iSlv_read           : in    std_logic;
        --! Readdata
        oSlv_readdata       : out   std_logic_vector(31 downto 0);
        --! Write strobe
        iSlv_write          : in    std_logic;
        --! Writedata
        iSlv_writedata      : in    std_logic_vector(31 downto 0);
        --! Waitrequest
        oSlv_waitrequest    : out   std_logic
    );
end atomicmodify;

architecture rtl of atomicmodify is
    -- fsm
    type tFsm is (
        sIdle,
        sRead,
        sWrite
    );

    -- register set
    type tReg is record
        address         : std_logic_vector(oMst_address'range);
        byteenable      : std_logic_vector(oMst_byteenable'range);
        writedata       : std_logic_vector(oMst_writedata'range);
        readdata        : std_logic_vector(iMst_readdata'range);
        fsm             : tFsm;
        ack             : std_logic;
    end record;

    constant cRegInit : tReg := (
        address         => (others => cInactivated),
        byteenable      => (others => cInactivated),
        writedata       => (others => cInactivated),
        readdata        => (others => cInactivated),
        fsm             => sIdle,
        ack             => cInactivated
    );

    -- register set signals
    signal reg      : tReg;
    signal reg_next : tReg;
begin
    --! The process describes the register set.
    regClk : process(iClk)
    begin
        if rising_edge(iClk) then
            if iRst = cActivated then
                reg <= cRegInit;
            else
                reg <= reg_next;
            end if;
        end if;
    end process;

    --! The process describes the combinational circuit for the register set.
    regComb : process (
        reg,
        iSlv_address, iSlv_byteenable, iSlv_read, iSlv_write, iSlv_writedata,
        iMst_waitrequest, iMst_readdata
    )
    begin
        -- default
        reg_next        <= reg;
        reg_next.ack    <= cInactivated;

        -- Master access state machine
        case reg.fsm is
            when sIdle =>
                if iSlv_write = cActivated and reg.ack = cInactivated then
                    reg_next.fsm        <= sRead;
                    reg_next.address    <= iSlv_address & "00";
                    reg_next.byteenable <= iSlv_byteenable;
                    reg_next.writedata  <= iSlv_writedata;
                elsif iSlv_read = cActivated and reg.ack = cInactivated then
                    reg_next.fsm        <= sIdle;
                    reg_next.ack        <= cActivated;
                end if;
            when sRead =>
                if iMst_waitrequest = cInactivated then
                    reg_next.fsm        <= sWrite;
                    reg_next.readdata   <= iMst_readdata;
                end if;
            when sWrite =>
                if iMst_waitrequest = cInactivated then
                    reg_next.fsm        <= sIdle;
                    reg_next.ack        <= cActivated;
                end if;
        end case;

    end process;

    -- Assign ports to register set
    oSlv_waitrequest    <= not reg.ack;
    oSlv_readdata       <= reg.readdata;

    oMst_address        <= reg.address;
    oMst_byteenable     <= reg.byteenable;
    oMst_writedata      <= reg.writedata;
    oMst_read           <= cActivated when reg.fsm = sRead else cInactivated;
    oMst_write          <= cActivated when reg.fsm = sWrite else cInactivated;
    oMst_lock           <= cActivated when reg.fsm = sRead else cInactivated;
end rtl;
