------------------------------------------------------------------------------------------------------------------------
-- Process Data Interface (PDI) event handling
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

--the order of events:
-- e.g. sw event = 1 and hw event = 2
-- event = (HW1 & HW0 & SW0)
-- pcp only sets SW0, but can read SW0
-- ap ack all events

entity pdiEvent is
	generic (
		genOnePdiClkDomain_g		:		boolean := false;
		iSwEvent_g					:		integer := 1;
		iHwEvent_g					:		integer := 2
	);
			
	port (  
			--port A -> PCP
			clkA						: in	std_logic;
			rstA						: in	std_logic;
			eventSetA					: in	std_logic_vector(iSwEvent_g-1 downto 0); --to set event (pulse!)
			eventReadA					: out	std_logic_vector(iSwEvent_g+iHwEvent_g-1 downto 0); --to read event set (can be acked by ap!!!)
			--port B -> AP
			clkB						: in	std_logic;
			rstB						: in	std_logic;
			eventAckB					: in	std_logic_vector(iSwEvent_g+iHwEvent_g-1 downto 0); --to ack events (pulse!)
			eventReadB					: out	std_logic_vector(iSwEvent_g+iHwEvent_g-1 downto 0); --to read event set
			--hw event set pulse (must be synchronous to clkB!)
			hwEventSetPulseB			: in	std_logic_vector(iHwEvent_g-1 downto 0)
	);
end entity pdiEvent;

architecture rtl of pdiEvent is
--in clk domain A
signal	eventA_s, --stores the events in A domain
		eventA_ackPulse --ack the event (by ap)
										:		std_logic_vector(iSwEvent_g+iHwEvent_g-1 downto 0);
signal	eventA_setPulse --sets the sw event only (by pcp)
										:		std_logic_vector(iSwEvent_g-1 downto 0);
signal	hwEventA_setPulse --sets the hw event only
										:		std_logic_vector(iHwEvent_g-1 downto 0);
--in clk domain B
signal	eventB_s, --stores the events in B domain
		eventB_ackPulse --ack the event (by ap)
										:		std_logic_vector(iSwEvent_g+iHwEvent_g-1 downto 0);
signal	eventB_setPulse --sets the sw event only (by pcp)
										:		std_logic_vector(iSwEvent_g-1 downto 0);
begin
	
	--pcp
	eventReadA <= eventA_s;
	
	--eventA_s stores all events
	--eventA_ackPulse sends acks for all events
	--eventA_setPulse sends set for sw event only
	eventA_setPulse <= eventSetA;
	--hwEventA_setPulse sends set for hw event only
	process(clkA, rstA)
	variable event_var : std_logic_vector(eventA_s'range);		
	begin  
		if rstA = '1' then
			eventA_s <= (others => '0');
		elsif clkA = '1' and clkA'event then
			--get event state to do magic
			event_var := eventA_s;
			
			--first let the ack does its work...
			event_var := event_var and not eventA_ackPulse;
			
			--second the sw events may overwrite the ack...
			event_var(iSwEvent_g-1 downto 0) := event_var(iSwEvent_g-1 downto 0) or
			eventA_setPulse(iSwEvent_g-1 downto 0);
			
			--last but not least, the hw events have its chance too
			event_var(iSwEvent_g+iHwEvent_g-1 downto iSwEvent_g) := event_var(iSwEvent_g+iHwEvent_g-1 downto iSwEvent_g) or
			hwEventA_setPulse(iHwEvent_g-1 downto 0);
			
			--and now, export it
			eventA_s <= event_var;
		end if;
	end process;	
	
	--ap
	eventReadB <= eventB_s;
	
	--eventB_s stores all events
	--eventB_ackPulse sends acks for all events
	eventB_ackPulse <= eventAckB;
	--eventB_setPulse sends set for sw event only
	--hwEventSetPulseB sends set for hw event only
	process(clkB, rstB)
	variable event_var : std_logic_vector(eventB_s'range);
	begin  
		if rstB = '1' then
			eventB_s <= (others => '0');
		elsif clkB = '1' and clkB'event then
			--I know, its almost the same as for A, but for clarity...
			--get event state
			event_var := eventB_s;
			
			--doing ack
			event_var := event_var and not eventB_ackPulse;
			
			--sw events may overwrite
			event_var(iSwEvent_g-1 downto 0) := event_var(iSwEvent_g-1 downto 0) or 
			eventB_setPulse(iSwEvent_g-1 downto 0);
			
			--hw events may overwrite too
			event_var(iSwEvent_g+iHwEvent_g-1 downto iSwEvent_g) := event_var(iSwEvent_g+iHwEvent_g-1 downto iSwEvent_g) or 
			hwEventSetPulseB(iHwEvent_g-1 downto 0);
			
			--and let's export
			eventB_s <= event_var;
		end if;
	end process;
	
	--xing the domains a to b
	syncEventSetGen : for i in 0 to iSwEvent_g-1 generate
		--only the software events are transferred!
		syncEventSet : entity work.slow2fastSync
			generic map (
				doSync_g => not genOnePdiClkDomain_g
			)
			port map (
				clkSrc => clkA,
				rstSrc => rstA,
				dataSrc => eventA_setPulse(i),
				clkDst => clkB,
				rstDst => rstB,
				dataDst => eventB_setPulse(i)
			);
		end generate;
	
	--xing the domains b to a
	syncEventAckGen : for i in eventB_s'range generate
		--all events are transferred
		syncEventAck : entity work.slow2fastSync
			generic map (
				doSync_g => not genOnePdiClkDomain_g
			)
			port map (
				clkSrc => clkB,
				rstSrc => rstB,
				dataSrc => eventB_ackPulse(i),
				clkDst => clkA,
				rstDst => rstA,
				dataDst => eventA_ackPulse(i)
			);
		end generate;
	
	syncHwEventGen : for i in 0 to iHwEvent_g-1 generate
		--hw events are transferred
		syncEventAck : entity work.slow2fastSync
			generic map (
				doSync_g => not genOnePdiClkDomain_g
			)
			port map (
				clkSrc => clkB,
				rstSrc => rstB,
				dataSrc => hwEventSetPulseB(i),
				clkDst => clkA,
				rstDst => rstA,
				dataDst => hwEventA_setPulse(i)
			);
		end generate;
end architecture rtl;