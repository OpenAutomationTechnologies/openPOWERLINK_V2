-------------------------------------------------------------------------------
--! @file irqGenRtl.vhd
--
--! @brief irq generator with sync latch feature
--
--! @details The irq generator is similar to a ordinary interrupt controller,
--! however, it is extended with a "sync-latch" feature. This enables to
--! throttle the interrupt requests and assert the general irq with the sync
--! input signal. Hence, any irq source is deferred to the sync assertion.
--
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
--! use host interface package for specific types
use work.hostInterfacePkg.all;

entity irqGen is
    generic (
        --! number of interrupt sources
        gIrqSourceCount     :       natural := 3
    );
    port (
        -- Global
        --! component wide clock signal
        iClk                : in    std_logic;
        --! component wide reset signal
        iRst                : in    std_logic;
        -- Irq
        --! sync source
        iSync               : in    std_logic;
        --! interrupt source vector (pulse)
        iIrqSource          : in    std_logic_vector(gIrqSourceCount downto 1);
        --! interrupt signal
        oIrq                : out   std_logic;
        -- Control
        --! master enable
        iIrqMasterEnable    : in    std_logic;
        --! interrupt source enable vector ('right is sync)
        iIrqSourceEnable    : in    std_logic_vector(gIrqSourceCount downto 0);
        --! interrupt acknowledge (pulse, 'right is sync)
        iIrqAcknowledge     : in    std_logic_vector(gIrqSourceCount downto 0);
        --! interrupt source pending
        oIrgPending         : out   std_logic_vector(gIrqSourceCount downto 0)
    );
end irqGen;

architecture Rtl of irqGen is
    --! sync rising edge
    signal syncRising                           : std_logic;
    --! interrupt register latch
    signal irqRegLatch, irqRegLatch_next        : std_logic_vector(gIrqSourceCount downto 0);
    --! interrupt source store
    signal irqSourceStore, irqSourceStore_next  : std_logic_vector(gIrqSourceCount downto 1);
    --! unregistered irq out signal
    signal unregIrq, irq_reg                    : std_logic;

begin
    --! generate pulse for rising edge of sync
    syncEdgeDet : entity libcommon.edgedetector
    port map (
        iArst       => iRst,
        iClk        => iClk,
        iEnable     => cActivated,
        iData       => iSync,
        oRising     => syncRising,
        oFalling    => open,
        oAny        => open
    );

    --! irq registers
    clkdReg : process(iClk)
    begin
        if rising_edge(iClk) then
            if iRst = cActivated then
                irqRegLatch     <= (others => cInactivated);
                irqSourceStore  <= (others => cInactivated);
                irq_reg         <= cInactivated;
            else
                irqRegLatch     <= irqRegLatch_next;
                irqSourceStore  <= irqSourceStore_next;
                irq_reg         <= unregIrq;
            end if;
        end if;
    end process;

    oIrq <= irq_reg;

    --! irq register control
    combIrqRegCont : process (
        iIrqSource,
        iIrqAcknowledge,
        irqRegLatch,
        irqSourceStore,
        syncRising,
        iIrqSourceEnable(iIrqSourceEnable'right)
    )
    begin
        --default
        irqRegLatch_next    <= irqRegLatch;
        irqSourceStore_next <= irqSourceStore;

        -- do acknowledge with latched and source register
        for i in gIrqSourceCount downto 1 loop
            if iIrqAcknowledge(i) = cActivated then
                irqRegLatch_next(i)     <= cInactivated;
                irqSourceStore_next(i)  <= cInactivated;
            end if;
        end loop;

        if iIrqAcknowledge(irqRegLatch'right) = cActivated then
            irqRegLatch_next(irqRegLatch'right) <= cInactivated;
        end if;

        for i in gIrqSourceCount downto 1 loop
            if iIrqSource(i) = cActivated then
                irqSourceStore_next(i) <= cActivated;
            end if;
        end loop;

        -- trigger irq with sync
        if syncRising = cActivated then
            -- loop through all irq sources
            for i in gIrqSourceCount downto 1 loop
                irqRegLatch_next(i) <=  irqSourceStore(i);
            end loop;

            -- activate sync irq if it is enabled
            -- (sync irqs in the past are not of interest!)
            irqRegLatch_next(irqRegLatch'right) <= iIrqSourceEnable(iIrqSourceEnable'right);
        end if;
    end process;

    --! output irq register
    oIrgPending <= irqRegLatch;

    --! irq signal generation
    combIrqGen : process (
        irqRegLatch,
        iIrqMasterEnable,
        iIrqSourceEnable
    )
        variable vTmp : std_logic;
    begin
        --default
        unregIrq <= cInactivated;

        -- the master enable overrules everything
        if iIrqMasterEnable = cActivated then
            -- check individual irqs
            vTmp := cInactivated;

            for i in gIrqSourceCount downto 0 loop
                vTmp := vTmp or (iIrqSourceEnable(i) and irqRegLatch(i));
            end loop;

            -- variable holds irq state
            unregIrq <= vTmp;
        end if;
    end process;

end Rtl;
