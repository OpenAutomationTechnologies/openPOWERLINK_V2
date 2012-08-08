------------------------------------------------------------------------------------------------------------------------
-- Process Data Interface (PDI) ap irq generator
--
-- 	  Copyright (C) 2011 B&R
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
------------------------------------------------------------------------------------------------------------------------
-- Version History
------------------------------------------------------------------------------------------------------------------------
-- 2011-09-14  	V0.01	zelenkaj    extract from pdi.vhd
------------------------------------------------------------------------------------------------------------------------
LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.std_logic_arith.all;
USE ieee.std_logic_unsigned.all;

entity apIrqGen is
	generic (
		genOnePdiClkDomain_g		:		boolean := false
	);
	port (
		--CLOCK DOMAIN PCP
		clkA							: in	std_logic;
		rstA							: in	std_logic;
		irqA							: in	std_logic; --toggle from MAC
		enableA							: in	std_logic; --APIRQ_CONTROL / IRQ_En
		modeA							: in	std_logic; --APIRQ_CONTROL / IRQ_MODE
		setA							: in	std_logic; --APIRQ_CONTROL / IRQ_SET
		--CLOCK DOMAIN AP
		clkB							: in	std_logic;
		rstB							: in	std_logic;
		ackB							: in	std_logic; --APIRQ_CONTROL / IRQ_ACK
		irqB							: out	std_logic
	);
end entity apIrqGen;

architecture rtl of apIrqGen is
type fsm_t is (wait4event, setIrq, wait4ack);
signal fsm								:		fsm_t;
signal enable, mode, irq, toggle, set	:		std_logic;
begin
	
	--everything is done in clkB domain!
	theFsm : process(clkB, rstB)
	begin
		if rstB = '1' then
			irqB <= '0';
			fsm <= wait4event;
		elsif clkB = '1' and clkB'event then
			if enable = '1' then
				case fsm is
					when wait4event =>
						if mode = '0' and set = '1' then
							fsm <= setIrq;
						elsif mode = '1' and irq = '1' then
							fsm <= setIrq;
						else
							fsm <= wait4event;
						end if;
					when setIrq =>
						irqB <= '1';
						fsm <= wait4ack;
					when wait4ack =>
						if ackB = '1' then
							irqB <= '0';
							fsm <= wait4event;
						else
							fsm <= wait4ack;
						end if;
				end case;
			else
				irqB <= '0';
				fsm <= wait4event;
			end if;
		end if;
	end process;
	
	syncEnable : entity work.sync
		generic map (
			doSync_g => not genOnePdiClkDomain_g
		)
		port map (
			din => enableA,
			dout => enable,
			clk => clkB,
			rst => rstB
		);
	
	syncSet : entity work.slow2fastSync
		generic map (
			doSync_g => not genOnePdiClkDomain_g
		)
		port map (
			dataSrc => setA,
			dataDst => set,
			clkSrc => clkA,
			rstSrc => rstA,
			clkDst => clkB,
			rstDst => rstB
		);
	
	syncMode : entity work.sync
		generic map (
			doSync_g => not genOnePdiClkDomain_g
		)
		port map (
			din => modeA,
			dout => mode,
			clk => clkB,
			rst => rstB
		);
	
	syncToggle : entity work.sync
		generic map (
			doSync_g => not genOnePdiClkDomain_g
		)
		port map (
			din => irqA,
			dout => toggle,
			clk => clkB,
			rst => rstB
		);
	
	toggleEdgeDet : entity work.edgeDet
		port map (
			din => toggle,
			rising => open,
			falling => open,
			any => irq,
			clk => clkB,
			rst => rstB
		);
	
end architecture rtl;