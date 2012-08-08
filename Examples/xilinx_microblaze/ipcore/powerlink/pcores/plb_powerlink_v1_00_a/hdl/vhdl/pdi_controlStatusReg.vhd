------------------------------------------------------------------------------------------------------------------------
-- Process Data Interface (PDI) status control register
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
-- 2011-11-21	V0.02	zelenkaj	added time synchronization feature
--									added 12 bytes to DPR as reserved
-- 2011-11-29	V0.03	zelenkaj	led and event is optional
-- 2011-12-20	V0.04	zelenkaj	changed 2xbuf switch source to ap irq
-- 2012-01-26   V0.05   zelenkaj    en-/disable double buffer with genTimeSync_g
------------------------------------------------------------------------------------------------------------------------
LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.std_logic_arith.all;
USE ieee.std_logic_unsigned.all;

entity pdiControlStatusReg is
	generic (
			bIsPcp						:		boolean := true;
			iAddrWidth_g				:		integer := 8;
			iBaseDpr_g					:		integer := 16#4#; --base address (in external mapping) of content in dpr
			iSpanDpr_g					:		integer := 12; --span of content in dpr
			iBaseMap2_g					:		integer := 0; --base address in dpr
			iDprAddrWidth_g				:		integer := 11;
			iRpdos_g					:		integer := 3;
			genLedGadget_g				:		boolean := false;
			genTimeSync_g				:		boolean := false;
			genEvent_g					:		boolean := false;
			--register content
			---constant values
			magicNumber					: 		std_Logic_vector(31 downto 0) := (others => '0');
			pdiRev						: 		std_logic_vector(15 downto 0) := (others => '0');
			tPdoBuffer					: 		std_logic_vector(31 downto 0) := (others => '0');
			rPdo0Buffer					: 		std_logic_vector(31 downto 0) := (others => '0');
			rPdo1Buffer					: 		std_logic_vector(31 downto 0) := (others => '0');
			rPdo2Buffer					: 		std_logic_vector(31 downto 0) := (others => '0');
			asyncBuffer1Tx				: 		std_logic_vector(31 downto 0) := (others => '0');
			asyncBuffer1Rx				: 		std_logic_vector(31 downto 0) := (others => '0');
			asyncBuffer2Tx				: 		std_logic_vector(31 downto 0) := (others => '0');
			asyncBuffer2Rx				:		std_logic_vector(31 downto 0) := (others => '0')
	);
			
	port (   
			--memory mapped interface
			clk							: in    std_logic;
			rst                  		: in    std_logic;
			sel							: in	std_logic;
			wr							: in	std_logic;
			rd							: in	std_logic;
			addr						: in	std_logic_vector(iAddrWidth_g-1 downto 0);
			be							: in	std_logic_vector(3 downto 0);
			din							: in	std_logic_vector(31 downto 0);
			dout						: out	std_logic_vector(31 downto 0);
			--register content
			---virtual buffer control signals
			rpdo_change_tog				: in	std_logic_vector(2 downto 0); --change buffer from hw acc
			tpdo_change_tog				: in	std_logic; --change buffer from hw acc
			pdoVirtualBufferSel			: in	std_logic_vector(31 downto 0); 	--for debugging purpose from SW side
																				--TXPDO_ACK | RXPDO2_ACK | RXPDO1_ACK | RXPDO0_ACK
			tPdoTrigger					: out	std_logic; --TPDO virtual buffer change trigger
			rPdoTrigger					: out	std_logic_vector(2 downto 0); --RPDOs virtual buffer change triggers
			---is used for Irq Generation and should be mapped to apIrqGen
			apIrqControlOut				: out	std_logic_vector(15 downto 0);
			apIrqControlIn				: in	std_logic_vector(15 downto 0);
			---event registers
			eventAckIn 					: in	std_logic_vector(15 downto 0);
			eventAckOut					: out	std_logic_vector(15 downto 0);
			---async irq (by event)
			asyncIrqCtrlIn				: In	std_logic_vector(15 downto 0); --Ap only
			asyncIrqCtrlOut				: out	std_logic_vector(15 downto 0); --Ap only
			---led stuff
			ledCnfgIn 					: in	std_logic_vector(15 downto 0);
			ledCnfgOut 					: out	std_logic_vector(15 downto 0);
			ledCtrlIn 					: in	std_logic_vector(15 downto 0);
			ledCtrlOut 					: out	std_logic_vector(15 downto 0);
			---time synchronization
			doubleBufSel_out			: out	std_logic; --Ap only
			doubleBufSel_in				: in	std_logic := '0'; --Pcp only
			timeSyncIrq					: in	std_logic; --SYNC IRQ to Ap (Ap only)
			--dpr interface (from PCP/AP to DPR)
			dprAddrOff					: out	std_logic_vector(iDprAddrWidth_g downto 0);
			dprDin						: out	std_logic_vector(31 downto 0);
			dprDout						: in	std_logic_vector(31 downto 0);
			dprBe						: out	std_logic_vector(3 downto 0);
			dprWr						: out	std_logic
			
	);
end entity pdiControlStatusReg;

architecture rtl of pdiControlStatusReg is
constant c_num_dbuf_dpr					:		integer := 4; --number of dbuf in DPR (per buffer 4 byte)
signal selDpr							:		std_logic; --if '1' get/write content from/to dpr
signal nonDprDout						:		std_logic_vector(31 downto 0);
signal addrRes							:		std_logic_vector(dprAddrOff'range);
--signal apIrqValue_s						: 		std_logic_vector(31 downto 0); --pcp only

signal virtualBufferSelectTpdo			:		std_logic_vector(15 downto 0);
signal virtualBufferSelectRpdo0			:		std_logic_vector(15 downto 0);
signal virtualBufferSelectRpdo1			:		std_logic_vector(15 downto 0);
signal virtualBufferSelectRpdo2			:		std_logic_vector(15 downto 0);

--edge detection
signal rpdo_change_tog_l				: 		std_logic_vector(2 downto 0); --change buffer from hw acc
signal tpdo_change_tog_l				: 		std_logic; --change buffer from hw acc

--time synchronization
signal timeSyncIrq_rising				: 		std_logic;
---select signals
signal sel_time_after_sync				:		std_logic;
signal sel_double_buffer				:		std_logic;
----double buffered content
signal sel_relative_time_l				:		std_logic;
signal sel_relative_time_h				:		std_logic;
signal sel_nettime_nsec					:		std_logic;
signal sel_nettime_sec					:		std_logic;
signal sel_time_sync_regs				:		std_logic;
---time after sync counter
constant c_time_after_sync_cnt_size		:		integer := 16; --revise code if changed
signal time_after_sync_cnt				:		std_logic_vector(c_time_after_sync_cnt_size-1 downto 0);
signal time_after_sync_cnt_latch		:		std_logic_vector(c_time_after_sync_cnt_size/2-1 downto 0);
signal time_after_sync_cnt_next			:		std_logic_vector(c_time_after_sync_cnt_size-1 downto 0);
signal time_after_sync_cnt_out			:		std_logic_vector(c_time_after_sync_cnt_size-1 downto 0) := (others => '0');
constant time_after_sync_res			:		std_logic_vector(32-c_time_after_sync_cnt_size-1 downto 0) := (others => '0');
---address offsets
constant c_addr_time_after_sync			:		integer := 16#50#;
constant c_addr_relative_time_l			:		integer := 16#40#;
constant c_addr_relative_time_h			:		integer := 16#44#;
constant c_addr_nettime_nsec			:		integer := 16#48#;
constant c_addr_nettime_sec				:		integer := 16#4C#;
begin	
	--map to 16bit register
	--TXPDO_ACK | RXPDO2_ACK | RXPDO1_ACK | RXPDO0_ACK
	virtualBufferSelectRpdo0 <= pdoVirtualBufferSel( 7 downto  0) & pdoVirtualBufferSel( 7 downto  0);
	virtualBufferSelectRpdo1 <= pdoVirtualBufferSel(15 downto  8) & pdoVirtualBufferSel(15 downto  8);
	virtualBufferSelectRpdo2 <= pdoVirtualBufferSel(23 downto 16) & pdoVirtualBufferSel(23 downto 16);
	virtualBufferSelectTpdo  <= pdoVirtualBufferSel(31 downto 24) & pdoVirtualBufferSel(31 downto 24);
	
	--generate dpr select signal
	selDpr	<=	sel		when	(conv_integer(addr) >= iBaseDpr_g AND
								 conv_integer(addr) <  iBaseDpr_g + iSpanDpr_g - c_num_dbuf_dpr)
						else	'0';
	
	--time sync select content if the double buffer has to be generated (genTimeSync_g)
	sel_time_after_sync <= '1' when conv_integer(addr)*4 = c_addr_time_after_sync and genTimeSync_g else '0';
	sel_relative_time_l <= '1' when conv_integer(addr)*4 = c_addr_relative_time_l and genTimeSync_g else '0';
	sel_relative_time_h <= '1' when conv_integer(addr)*4 = c_addr_relative_time_h and genTimeSync_g else '0';
	sel_nettime_nsec <= '1' when conv_integer(addr)*4 = c_addr_nettime_nsec and genTimeSync_g else '0';
	sel_nettime_sec <= '1' when conv_integer(addr)*4 = c_addr_nettime_sec and genTimeSync_g else '0';
	
	---or them up...
	sel_time_sync_regs <= sel_relative_time_l or sel_relative_time_h or sel_nettime_nsec or sel_nettime_sec;
	
    genTimeSync : if genTimeSync_g generate
    begin
    	--we need a rising edge to do magic
    	apSyncIrqEdgeDet : entity work.edgedet
    		port map (
    			din => timeSyncIrq,
    			rising => timeSyncIrq_rising,
    			falling => open,
    			any => open,
    			clk => clk,
    			rst => rst
    		);
    	
    	genDoubleBufPcp : if bIsPcp generate
    	begin
    		--take the other buffer (Ap has already inverted, see lines below!)
    		sel_double_buffer <= doubleBufSel_in;
    		
    		--Pcp has no timer
    		time_after_sync_cnt_out <= (others => '0');
    		
    	end generate;
    	
    	genDoubleBufAp : if not bIsPcp generate
    	begin
    		
    		--output the inverted to the PCP
    		doubleBufSel_out <= not sel_double_buffer;
    		
    		--switch the double buffer with the sync irq, rising edge of course
    		process(clk, rst)
    		begin
    			if rst = '1' then
    				sel_double_buffer <= '0';
    			elsif rising_edge(clk) then
    				if timeSyncIrq_rising = '1' then --rising edge
    					sel_double_buffer <= not sel_double_buffer;
    				end if;
    			end if;
    		end process;
    		
    	end generate;
    	
    	genTimeAfterSyncCnt : if not bIsPcp generate
    		constant ZEROS : std_logic_vector(time_after_sync_cnt'range) := (others => '0');
    		constant ONES : std_logic_vector(time_after_sync_cnt'range) := (others => '1');
    	begin
    		--TIME_AFTER_SYNC counter
    		process(clk, rst)
    		begin
    			if rst = '1' then
    				time_after_sync_cnt <= (others => '0');
    			elsif clk = '1' and clk'event then
    				time_after_sync_cnt <= time_after_sync_cnt_next;
    				
    				--there are some kind of interfaces that read only the half of a word...
    				-- so store the half that is not read
    				-- and forward it to the Ap at the next read
    				if sel = '1' and sel_time_after_sync = '1' and be = "0001" then
    					time_after_sync_cnt_latch <= time_after_sync_cnt(c_time_after_sync_cnt_size-1 downto c_time_after_sync_cnt_size/2);
    				end if;
    			end if;
    		end process;
    		
    		time_after_sync_cnt_next <= ZEROS when timeSyncIrq_rising = '1' else --rising edge
    			time_after_sync_cnt when time_after_sync_cnt = ONES else --saturate
    			time_after_sync_cnt + 1; --count for your life!
    		
    		time_after_sync_cnt_out <= time_after_sync_cnt when be(3 downto 2) = "11" or be(1 downto 0) = "11" else
    			time_after_sync_cnt_latch & time_after_sync_cnt(time_after_sync_cnt_latch'range);
    	end generate;
    end generate;
	
	--assign content depending on selDpr
	dprDin		<=	din;
	dprBe		<=	be;
	dprWr		<=	wr		when	selDpr = '1'	else
					'0';
	dout		<=	dprDout	when	selDpr = '1'	else
					nonDprDout;
	
	dprAddrOff	<=	addrRes + 4 when sel_double_buffer = '1' and sel_time_sync_regs = '1' and genTimeSync_g else --select 2nd double buffer
					addrRes; --select 1st double buffer or other content
	
	--address conversion
	---map external address mapping into dpr
	addrRes <= 	conv_std_logic_vector(iBaseMap2_g - iBaseDpr_g, addrRes'length);
	
	--non dpr read
	with conv_integer(addr)*4 select
		nonDprDout <=	magicNumber 					when 16#00#,
						(x"0000" & pdiRev) 				when 16#04#,
						--STORED IN DPR 				when 16#08#,
						--STORED IN DPR 				when 16#0C#,
						--STORED IN DPR 				when 16#10#,
						--STORED IN DPR 				when 16#14#,
						--STORED IN DPR 				when 16#18#,
						--STORED IN DPR 				when 16#1C#,
						--STORED IN DPR 				when 16#20#,
						--STORED IN DPR 				when 16#24#,
						--STORED IN DPR 				when 16#28#,
						--STORED IN DPR					when 16#2C#,
						--STORED IN DPR					when 16#30#,
						--STORED IN DPR					when 16#34#, --RESERVED
						--STORED IN DPR					when 16#38#, --RESERVED
						--STORED IN DPR					when 16#3C#, --RESERVED
						
						--STORED IN DPR x2				when c_addr_relative_time_l, --RELATIVE_TIME low
						--STORED IN DPR	x2				when c_addr_relative_time_h, --RELATIVE_TIME high
						--STORED IN DPR	x2				when c_addr_nettime_nsec, --NETTIME nsec
						--STORED IN DPR	x2				when c_addr_nettime_sec, --NETTIME sec
						
						(time_after_sync_res & 
						time_after_sync_cnt_out)		when c_addr_time_after_sync, --RES / TIME_AFTER_SYNC
						(eventAckIn & asyncIrqCtrlIn) 	when 16#54#,
						tPdoBuffer 						when 16#58#,
						rPdo0Buffer 					when 16#5C#,
						rPdo1Buffer 					when 16#60#,
						rPdo2Buffer 					when 16#64#,
						asyncBuffer1Tx 					when 16#68#,
						asyncBuffer1Rx 					when 16#6C#,
						asyncBuffer2Tx 					when 16#70#,
						asyncBuffer2Rx 					when 16#74#,
						--RESERVED 						when 16#78#,
						--RESERVED 						when 16#7C#,
						(virtualBufferSelectRpdo0 & 
						virtualBufferSelectTpdo) 		when 16#80#,
						(virtualBufferSelectRpdo2 & 
						virtualBufferSelectRpdo1) 		when 16#84#,
						(x"0000" & apIrqControlIn) 		when 16#88#,
						--RESERVED						when 16#8C#,
						--RESERVED 						when 16#90#,
						(ledCnfgIn & ledCtrlIn) 		when 16#94#,
						(others => '0') 				when others;
	
	--ignored values
	asyncIrqCtrlOut(14 downto 1) <= (others => '0');
	eventAckOut(15 downto 8) <= (others => '0');
	--non dpr write
	process(clk, rst)
	begin
		if rst = '1' then
			tPdoTrigger <= '0';
			rPdoTrigger <= (others => '0');
			
			--apIrqControlOut <= (others => '0');
			if bIsPcp = true then
				apIrqControlOut(7) <= '0';
				apIrqControlOut(6) <= '0';
			end if;
			
			if bIsPcp = false then
				apIrqControlOut(15) <= '0';
			end if;
			apIrqControlOut(0) <= '0';
			
			if genEvent_g then
				if bIsPcp = false then
					asyncIrqCtrlOut(0) <= '0';
					asyncIrqCtrlOut(15) <= '0';
				end if;
				eventAckOut(7 downto 0) <= (others => '0');
			end if;
			
			if genLedGadget_g then
				ledCtrlOut(7 downto 0) <= (others => '0');
				ledCnfgOut(7 downto 0) <= (others => '0');
			end if;
			
			if bIsPcp then
				rpdo_change_tog_l <= (others => '0');
				tpdo_change_tog_l <= '0';
			end if;
		elsif clk = '1' and clk'event then
			--default assignments
			tPdoTrigger <= '0';
			rPdoTrigger <= (others => '0');
			apIrqControlOut(0) <= '0'; --PCP: set pulse // AP: ack pulse
			
			if genEvent_g then
				eventAckOut(7 downto 0) <= (others => '0'); --PCP: set pulse // AP: ack pulse
			end if;
			
			if bIsPcp then
				--shift register for edge det
				rpdo_change_tog_l <= rpdo_change_tog;
				tpdo_change_tog_l <= tpdo_change_tog;
				
				--edge detection
				---tpdo
				if tpdo_change_tog_l /= tpdo_change_tog then
					tPdoTrigger <= '1';
				end if;
				---rpdo
				for i in rpdo_change_tog'range loop
					if rpdo_change_tog_l(i) /= rpdo_change_tog(i) then
						rPdoTrigger(i) <= '1';
					end if;
				end loop;
			end if;
			
			if wr = '1' and sel = '1' and selDpr = '0' then
				case conv_integer(addr)*4 is
					when 16#00# =>
						--RO
					when 16#04# =>
						--RO
					when 16#08# =>
						--STORED IN DPR
					when 16#0C# =>
						--STORED IN DPR
					when 16#10# =>
						--STORED IN DPR
					when 16#14# =>
						--STORED IN DPR
					when 16#18# =>
						--STORED IN DPR
					when 16#1C# =>
						--STORED IN DPR
					when 16#20# =>
						--STORED IN DPR
					when 16#24# =>
						--STORED IN DPR
					when 16#28# =>
						--STORED IN DPR
					when 16#2C# =>
						--STORED IN DPR
					when 16#30# =>
						--STORED IN DPR
					when 16#34# =>
						--STORED IN DPR RESERVED
					when 16#38# =>
						--STORED IN DPR RESERVED
					when 16#3C# =>
						--STORED IN DPR RESERVED
					when 16#40# =>
						--STORED IN DPR x2
					when 16#44# =>
						--STORED IN DPR x2
					when 16#48# =>
						--STORED IN DPR x2
					when 16#4C# =>
						--STORED IN DPR x2
					when c_addr_time_after_sync =>
						--RO
					when 16#54# =>
						--AP ONLY
						if genEvent_g then
							if be(0) = '1' and bIsPcp = false then
								--asyncIrqCtrlOut(7 downto 0) <= din(7 downto 0);
								asyncIrqCtrlOut(0) <= din(0); --rest is ignored
							end if;
							if be(1) = '1' and bIsPcp = false then
								--asyncIrqCtrlOut(15 downto 8) <= din(15 downto 8);
								asyncIrqCtrlOut(15) <= din(15); --rest is ignored
							end if;
							if be(2) = '1' then
								eventAckOut(7 downto 0) <= din(23 downto 16);
							end if;
--ignore higher byte of event ack
--							if be(3) = '1' then
--								eventAckOut(15 downto 8) <= din(31 downto 24);
--							end if;
						end if;
					when 16#58# =>
						--RO
					when 16#5C# =>
						--RO
					when 16#60# =>
						--RO
					when 16#64# =>
						--RO
					when 16#68# =>
						--RO
					when 16#6C# =>
						--RO
					when 16#70# =>
						--RO
					when 16#74# =>
						--RO
					when 16#78# =>
						--RESERVED
					when 16#7C# =>
						--RESERVED
					when 16#80# =>
						if be(0) = '1' then
							tPdoTrigger <= '1';
						end if;
						if be(1) = '1' then
							tPdoTrigger <= '1';
						end if;
						if be(2) = '1' then
							rPdoTrigger(0) <= '1';
						end if;
						if be(3) = '1' then
							rPdoTrigger(0) <= '1';
						end if;
					when 16#84# =>
						if be(0) = '1' then
							rPdoTrigger(1) <= '1';
						end if;
						if be(1) = '1' then
							rPdoTrigger(1) <= '1';
						end if;
						if be(2) = '1' then
							rPdoTrigger(2) <= '1';
						end if;
						if be(3) = '1' then
							rPdoTrigger(2) <= '1';
						end if;
					when 16#88# =>
						if be(0) = '1' then
							--apIrqControlOut(7 downto 0) <= din(7 downto 0);
							if bIsPcp = true then
								apIrqControlOut(7) <= din(7);
								apIrqControlOut(6) <= din(6);
							end if;
							apIrqControlOut(0) <= din(0);
						end if;
						if be(1) = '1' then
							--apIrqControlOut(15 downto 8) <= din(15 downto 8);							
							if bIsPcp = false then
								apIrqControlOut(15) <= din(15);
							end if;
						end if;
					when 16#8C# =>
						--RESERVED
					when 16#90# =>
						--RESERVED
					when 16#94# =>
						if genLedGadget_g then
							if be(0) = '1' then
								ledCtrlOut(7 downto 0) <= din(7 downto 0);
							end if;
							if be(1) = '1' then
								ledCtrlOut(15 downto 8) <= din(15 downto 8);
							end if;
							if be(2) = '1' then
								ledCnfgOut(7 downto 0) <= din(23 downto 16);
							end if;
							if be(3) = '1' then
								ledCnfgOut(15 downto 8) <= din(31 downto 24);
							end if;
						end if;
					when others =>
				end case;
			end if;
		end if;
	end process;
		
end architecture rtl;