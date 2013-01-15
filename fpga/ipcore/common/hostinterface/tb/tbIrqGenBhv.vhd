-------------------------------------------------------------------------------
--! @file tbIrqGenBhv.vhd
--
--! @brief Testbench for Interrupt request generator component
--
-------------------------------------------------------------------------------
--
--    (c) B&R, 2012
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
-- Design unit header --
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.global.all;


entity tbIrqGen is
end tbIrqGen;

architecture Bhv of tbIrqGen is

---- Component declarations -----

component clkgen
  generic(
       gPeriod : time := 20 ns
  );
  port (
       iDone : in std_logic;
       oClk : out std_logic
  );
end component;
component enableGen
  generic(
       gEnableDelay : time := 100 ns
  );
  port (
       iReset : in std_logic;
       oEnable : out std_logic;
       onEnable : out std_logic
  );
end component;
component irqGen
  generic(
       gIrqSourceCount : natural := 3
  );
  port (
       iClk : in std_logic;
       iIrqAcknowledge : in std_logic_vector(gIrqSourceCount downto 0);
       iIrqMasterEnable : in std_logic;
       iIrqSource : in std_logic_vector(gIrqSourceCount downto 1);
       iIrqSourceEnable : in std_logic_vector(gIrqSourceCount downto 0);
       iRst : in std_logic;
       iSync : in std_logic;
       oIrgPending : out std_logic_vector(gIrqSourceCount downto 0);
       oIrq : out std_logic
  );
end component;

---- Architecture declarations -----
constant cCounterSel : natural := 4;
constant cIrqSourceCount : natural := 4;


----     Constants     -----
constant GND_CONSTANT   : std_logic := '0';

---- Signal declarations used on the diagram ----

signal clk : std_logic;
signal done : std_logic;
signal GND : std_logic;
signal irq : std_logic;
signal irqMasterEnable : std_logic;
signal rst : std_logic;
signal counter : std_logic_vector (7 downto 0);
signal irqAck : std_logic_vector (cIrqSourceCount downto 0);
signal irqPending : std_logic_vector (cIrqSourceCount downto 0);
signal irqSource : std_logic_vector (cIrqSourceCount downto 1);
signal irqSourceEnable : std_logic_vector (cIrqSourceCount downto 0);
signal stateCounter : std_logic_vector (31 downto 0);

begin

---- User Signal Assignments ----
process(clk)
begin

    if rising_edge(clk) then

        if rst = cActivated then
            counter <= (others => cInactivated);
            stateCounter <= (others => cInactivated);
        else
            counter <= std_logic_vector(
                unsigned(counter) + 1);

            if counter(cCounterSel) = cActivated then
                stateCounter <= std_logic_vector(
                    unsigned(stateCounter) + 1);
                counter <= (others => cInactivated);
            end if;
        end if;

    end if;

end process;
--
done <= cActivated when unsigned (stateCounter) > 40 else
        cInactivated;

--
irqMasterEnable <= cActivated when (unsigned(stateCounter) > 1 and unsigned(statecounter) < 30) else
                   cInactivated;

--
irqSourceEnable <= "11110" when (unsigned(stateCounter) > 1 and unsigned(statecounter) < 15) else
                   "00001" when (unsigned(stateCounter) > 15 and unsigned(stateCounter) < 28) else
                   "00000";

--
irqSource <= "0001" when (unsigned(stateCounter) = 3 and unsigned(counter) = 4) else
             "0101" when (unsigned(stateCounter) = 6 and unsigned(counter) = 4) else
             "1000" when (unsigned(stateCounter) = 20 and unsigned(counter) = 4) else
             "0000";

--
irqAck <= "00010" when (unsigned(stateCounter) = 5 and unsigned(counter) =4) else
          "01000" when (unsigned(stateCounter) = 8 and unsigned(counter) = 4 ) else
          "00010" when (unsigned(stateCounter) = 10 and unsigned(counter) = 4) else
          "10001" when (unsigned(stateCounter) = 25 and unsigned(counter) = 4) else
          "00001" when (unsigned(stateCounter) > 15 and unsigned(stateCounter) < 28
                                                    and unsigned(counter) = 4) else
          "00000";

----  Component instantiations  ----

DUT : irqGen
  generic map (
       gIrqSourceCount => cIrqSourceCount
  )
  port map(
       iClk => clk,
       iIrqAcknowledge => irqAck( cIrqSourceCount downto 0 ),
       iIrqMasterEnable => irqMasterEnable,
       iIrqSource => irqSource( cIrqSourceCount downto 1 ),
       iIrqSourceEnable => irqSourceEnable( cIrqSourceCount downto 0 ),
       iRst => rst,
       iSync => counter(cCounterSel),
       oIrgPending => irqPending( cIrqSourceCount downto 0 ),
       oIrq => irq
  );

U1 : clkgen
  port map(
       iDone => done,
       oClk => clk
  );

U2 : enableGen
  port map(
       iReset => GND,
       onEnable => rst
  );


---- Power , ground assignment ----

GND <= GND_CONSTANT;

end Bhv;
