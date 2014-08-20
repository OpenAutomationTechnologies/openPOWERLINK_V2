-------------------------------------------------------------------------------
--! @file clkXingRtl.vhd
--
--! @brief Clock Crossing Bus converter
--
--! @details Used to transfer a faster slave interface to a slower one.
--
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
--! need reduce or operation
use ieee.std_logic_misc.OR_REDUCE;

--! Common library
library libcommon;
--! Use common library global package
use libcommon.global.all;

entity clkXing is
    generic (
        gCsNum : natural := 2;
        gDataWidth : natural := 32
    );
    port (
        iArst : in std_logic;
        --fast
        iFastClk : in std_logic;
        iFastCs : in std_logic_vector(gCsNum-1 downto 0);
        iFastRNW : in std_logic;
        oFastReaddata : out std_logic_vector(gDataWidth-1 downto 0);
        oFastWrAck : out std_logic;
        oFastRdAck : out std_logic;
        --slow
        iSlowClk : in std_logic;
        oSlowCs : out std_logic_vector(gCsNum-1 downto 0);
        oSlowRNW : out std_logic;
        iSlowReaddata : in std_logic_vector(gDataWidth-1 downto 0);
        iSlowWrAck : in std_logic;
        iSlowRdAck : in std_logic
    );
end entity;

architecture rtl of clkXing is
    --! Fsm type
    type tFsm is (sIdle, sTransfer);
    --! Clock domain registers
    type tRegClkDomain is record
        chipselect      : std_logic_vector(gCsNum-1 downto 0);
        rnw             : std_logic;
        sync            : std_logic;
        ack             : std_logic;
        fsm             : tFsm;
    end record;
    --! Register init
    constant cRegClkDomainInit : tRegClkDomain := (
        chipselect      => (others => cInactivated),
        rnw             => cInactivated,
        sync            => cInactivated,
        ack             => cInactivated,
        fsm             => sIdle
    );

    --! Slow clock domain register
    signal slowClkReg       : tRegClkDomain;
    signal slowClkReg_next  : tRegClkDomain;

    --! Fast clock domain register
    signal fastClkReg       : tRegClkDomain;
    signal fastClkReg_next  : tRegClkDomain;

    --! Readdata register
    signal readdataReg      : std_logic_vector(iSlowReaddata'range);
    signal readdataReg_next : std_logic_vector(readdataReg'range);

    --! Transferred sync signal (fast ---> slow)
    signal tranSync_sync    : std_logic;
    --! Transferred ack signal (slow ---> fast)
    signal tranAck_sync     : std_logic;
begin
    -- Map registers to outputs
    -- FAST
    oFastWrAck      <= fastClkReg.ack and not iFastRNW;
    oFastRdAck      <= fastClkReg.ack and iFastRNW;
    oFastReaddata   <= readdataReg;
    -- SLOW
    oSlowCs         <= slowClkReg.chipselect;
    oSlowRNW        <= iFastRNW;

    --! Fast clock domain logic
    fastComb : process (
        fastClkReg,
        tranAck_sync,
        iFastCs, iFastRNW
    )
    begin
        -- General defaults to avoid latches
        fastClkReg_next <= fastClkReg;

        -- Inactivate pulsing registers
        fastClkReg_next.sync <= cInactivated;

        -- Assign transferred signals
        fastClkReg_next.ack <= tranAck_sync;

        case fastClkReg.fsm is
            when sIdle =>
                if OR_REDUCE(iFastCs) = cActivated then
                    fastClkReg_next.fsm         <= sTransfer;
                    fastClkReg_next.sync        <= cActivated;
                    fastClkReg_next.chipselect  <= iFastCs;
                    fastClkReg_next.rnw         <= iFastRNW;
                end if;

            when sTransfer =>
                if fastClkReg.ack = cActivated then
                    fastClkReg_next.fsm <= sIdle;
                end if;

        end case;
    end process fastComb;

    --! Slow clock domain logic
    slowComb : process (
        slowClkReg, fastClkReg, readdataReg,
        tranSync_sync,
        iSlowWrAck, iSlowRdAck, iSlowReaddata
    )
    begin
        -- Default
        slowClkReg_next     <= slowClkReg;
        readdataReg_next    <= readdataReg;

        -- Inactivate pulsing registers
        slowClkReg_next.ack <= cInactivated;

        -- Assign transferred signals
        slowClkReg_next.sync <= tranSync_sync;

        case slowClkReg.fsm is
            when sIdle =>
                if slowClkReg.sync = cActivated then
                    slowClkReg_next.fsm <= sTransfer;
                end if;

            when sTransfer =>
                slowClkReg_next.chipselect  <= fastClkReg.chipselect;
                slowClkReg_next.rnw         <= fastClkReg.rnw;

                if iSlowRdAck = cActivated or iSlowWrAck = cActivated then
                    slowClkReg_next.fsm         <= sIdle;
                    slowClkReg_next.chipselect  <= (others => cInactivated);
                    slowClkReg_next.ack         <= cActivated;

                    if iSlowRdAck = cActivated then
                        readdataReg_next <= iSlowReaddata;
                    end if;
                end if;

        end case;
    end process slowComb;

    --! Fast clock registers
    fastClockReg : process(iArst, iFastClk)
    begin
        if iArst = cActivated then
            fastClkReg  <= cRegClkDomainInit;
        elsif rising_edge(iFastClk) then
            fastClkReg  <= fastClkReg_next;
        end if;
    end process fastClockReg;

    --! Slow clock registers
    slowClockReg : process(iArst, iSlowClk)
    begin
        if iArst = cActivated then
            slowClkReg  <= cRegClkDomainInit;
            readdataReg <= (others => cInactivated);
        elsif rising_edge(iSlowClk) then
            slowClkReg  <= slowClkReg_next;
            readdataReg <= readdataReg_next;
        end if;
    end process slowClockReg;

    --! Transfer sync pulse to slow clock domain
    tranSyncToSlow : entity libcommon.syncTog
        generic map (
            gStages     => 2,
            gInit       => cInactivated
        )
        port map (
            iSrc_rst    => iArst,
            iSrc_clk    => iFastClk,
            iSrc_data   => fastClkReg.sync,
            iDst_rst    => iArst,
            iDst_clk    => iSlowClk,
            oDst_data   => tranSync_sync
        );

    --! Transfer ack pulse to fast clock domain
    tranAckToFast : entity libcommon.syncTog
        generic map (
            gStages     => 2,
            gInit       => cInactivated
        )
        port map (
            iSrc_rst    => iArst,
            iSrc_clk    => iSlowClk,
            iSrc_data   => slowClkReg.ack,
            iDst_rst    => iArst,
            iDst_clk    => iFastClk,
            oDst_data   => tranAck_sync
        );
end architecture;
