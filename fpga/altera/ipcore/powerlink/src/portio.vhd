------------------------------------------------------------------------------------------------------------------------
-- Simple Port I/O
--
-- 	  Copyright (C) 2010 B&R
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
-- 2010-08-16  	V0.01	zelenkaj    First version
-- 2010-10-04  	V0.02	zelenkaj	Bugfix: PORTDIR was mapped incorrectly (according to doc) to Avalon bus
-- 2010-11-23	V0.03	zelenkaj	Added Operational Flag to portio
--									Added counter for valid assertion duration
-- 2011-04-20	V0.10	zelenkaj	Added synchronizer at inputs
-- 2011-12-02	V0.11	zelenkaj	Added I, O and T instead of IO ports
-- 2012-03-16   V0.12   zelenkaj    Traveled back to the past to change the version history
--                                  Removed readdata register (saves resources)
------------------------------------------------------------------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.std_logic_arith.all;
USE ieee.std_logic_unsigned.all;

entity portio is
	generic (
		pioValLen_g 	:		integer := 50; --clock ticks of pcp_clk
		pioGenIoBuf_g	:		boolean := true
	);
	port (
		s0_address    	: in    std_logic;
		s0_read       	: in    std_logic;
		s0_readdata   	: out   std_logic_vector(31 downto 0);
		s0_write      	: in    std_logic;
		s0_writedata  	: in    std_logic_vector(31 downto 0);
		s0_byteenable 	: in    std_logic_vector(3 downto 0);
		s0_waitrequest	: out	std_logic;
		clk        		: in    std_logic;
		reset      		: in    std_logic;
		x_pconfig    	: in    std_logic_vector(3 downto 0);
		x_portInLatch	: in 	std_logic_vector(3 downto 0);
		x_portOutValid 	: out 	std_logic_vector(3 downto 0);
		x_portio     	: inout std_logic_vector(31 downto 0);
		x_portio_I		: in 	std_logic_vector(31 downto 0) := (others => '0');
		x_portio_O		: out	std_logic_vector(31 downto 0);
		x_portio_T		: out	std_logic_vector(31 downto 0);
		x_operational 	: out 	std_logic
	);
end entity portio;

architecture rtl of portio is
	signal sPortConfig : std_logic_vector(x_pconfig'range);
	signal sPortOut : std_logic_vector(x_portio'range);
	signal sPortIn, sPortIn_s, sPortInL : std_logic_vector(x_portio'range);
	signal x_portInLatch_s : std_logic_vector(x_portInLatch'range);
	signal x_operational_s : std_logic;
	signal x_portOutValid_s : std_logic_vector(x_portOutValid'range);
begin

	sPortConfig <= x_pconfig;
	x_operational <= x_operational_s;
	
	portGen : for i in 3 downto 0 generate
		genIoBuf : if pioGenIoBuf_g generate
		begin
			--if port configuration bit is set to '0', the appropriate port-byte is an output
			x_portio((i+1)*8-1 downto (i+1)*8-8) 	<= sPortOut((i+1)*8-1 downto (i+1)*8-8) when sPortConfig(i) = '0' else (others => 'Z');
			--if port configuration bit is set to '1', the appropriate port-byte is forwarded to the portio registers for the PCP
			sPortIn((i+1)*8-1 downto (i+1)*8-8)		<= x_portio((i+1)*8-1 downto (i+1)*8-8) when sPortConfig(i) = '1' else (others => '0');
		end generate;
		
		dontGenIoBuf : if not pioGenIoBuf_g generate
		begin
			x_portio_O((i+1)*8-1 downto (i+1)*8-8) 	<= sPortOut((i+1)*8-1 downto (i+1)*8-8);
			sPortIn((i+1)*8-1 downto (i+1)*8-8)		<= x_portio_I((i+1)*8-1 downto (i+1)*8-8);
			--if port configuration bit is set to '0', the appropriate port-byte is an output ('0')
			--if port configuration bit is set to '1', the appropriate port-byte is an input ('1')
			x_portio_T((i+1)*8-1 downto (i+1)*8-8)	<= (others => '0') when sPortConfig(i) = '0' else (others => '1');
		end generate;
	end generate;
	
	--Avalon interface
	avalonPro : process(clk, reset)
	begin
		if reset = '1' then
			x_portOutValid_s <= (others => '0');
			sPortOut <= (others => '0');
			x_operational_s <= '0';
			
		elsif clk = '1' and clk'event then
			x_portOutValid_s <= (others => '0');
			
			if s0_write = '1' then
				case s0_address is
					when '0' =>	--write port
						for i in 3 downto 0 loop
							if s0_byteenable(i) = '1' then
								sPortOut((i+1)*8-1 downto (i+1)*8-8) <= s0_writedata((i+1)*8-1 downto (i+1)*8-8);
								x_portOutValid_s(i) <= '1';
							end if;
						end loop;
					when '1' => --write to config register operational flag
						if s0_byteenable(3) = '1' then
							x_operational_s <= s0_writedata(s0_writedata'left);
						end if;
					when others =>
				end case;
				
			end if;
		end if;
	end process;
    
    
    s0_readdata <=  sPortInL when s0_read = '1' and s0_address = '0' else
        x_operational_s & "000" & x"00000" & x"0" & sPortConfig;
	
	thePortioCnters : for i in 0 to 3 generate
		thePortioCnt : entity work.portio_cnt
		generic map (
			maxVal =>	pioValLen_g
		)
		port map (
			clk => clk,
			rst => reset,
			pulse => x_portOutValid_s(i),
			valid => x_portOutValid(i)
		);
	end generate;
	
	--latch input signals
	latchInPro : process(clk, reset)
	begin
		if reset = '1' then
			sPortInL <= (others => '0');
		elsif clk = '1' and clk'event then
			
			for i in 3 downto 0 loop
				if x_portInLatch_s(i) = '1' then
					sPortInL((i+1)*8-1 downto (i+1)*8-8) <= sPortIn_s((i+1)*8-1 downto (i+1)*8-8);
				end if;
			end loop;
			
		end if;
	end process;
	
	-- waitrequest signals
	theWaitrequestGenerators : block
		signal s0_rd_ack, s0_wr_ack : std_logic;
	begin
		
		-- PCP
		thePcpWrWaitReqAckGen : entity work.req_ack
		generic map (
			zero_delay_g => true
		)
		port map (
			clk => clk,
			rst => reset,
			enable => s0_write,
			ack => s0_wr_ack
		);
		
		thePcpRdWaitReqAckGen : entity work.req_ack
		generic map (
			zero_delay_g => true
		)
		port map (
			clk => clk,
			rst => reset,
			enable => s0_read,
			ack => s0_rd_ack
		);
		
		s0_waitrequest <= not(s0_rd_ack or s0_wr_ack);
	end block;
	
	--synchronize input signals
	genSyncInputs : for i in sPortIn'range generate
		syncInputs : entity work.sync
			port map (
				din => sPortIn(i),
				dout => sPortIn_s(i),
				clk => clk,
				rst => reset
			);
	end generate;
	
	--synchronize latch signals
	genSyncLatch : for i in x_portInLatch'range generate
		syncInputs : entity work.sync
			port map (
				din => x_portInLatch(i),
				dout => x_portInLatch_s(i),
				clk => clk,
				rst => reset
			);
	end generate;
	
end architecture rtl;
