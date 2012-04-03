------------------------------------------------------------------------------------------------------------------------
-- Process Data Interface (PDI) led gadget
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

--the led gadget can be set by three different sources
-- source A, B and C
-- the highest priority has C

entity pdiLed is
	generic (
		iLedWidth_g						:		integer := 8
	);
	port (
		--src A
		srcAled							: in	std_logic_vector(iLedWidth_g-1 downto 0);
		srcAforce						: in	std_logic_vector(iLedWidth_g-1 downto 0);
		--src B
		srcBled							: in	std_logic_vector(iLedWidth_g-1 downto 0);
		srcBforce						: in	std_logic_vector(iLedWidth_g-1 downto 0);
		--src C
		srcCled							: in	std_logic_vector(iLedWidth_g-1 downto 0);
		srcCforce						: in	std_logic_vector(iLedWidth_g-1 downto 0);
		--led output
		ledOut							: out	std_logic_vector(iLedWidth_g-1 downto 0)
	);
end entity pdiLed;

architecture rtl of pdiLed is
begin
	theLedGadget : process(srcAled, srcAforce, srcBled, srcBforce, srcCled, srcCforce)
		variable tmp_led : std_logic_vector(ledOut'range);
	begin
		tmp_led := (others => '0');
		
		for i in tmp_led'range loop
			--okay, src A may drive if forced
			if srcAforce(i) = '1' then
				tmp_led(i) := srcAled(i);
			end if;
			
			--same vaild for src B, but it overrules src A
			if srcBforce(i) = '1' then
				tmp_led(i) := srcBled(i);
			end if;
			
			--and the head of the logics => src C
			if srcCforce(i) = '1' then
				tmp_led(i) := srcCled(i);
			end if;
		end loop;
		
		--let's export and go for a coffee...
		ledOut <= tmp_led;		
	end process;
end architecture rtl;