------------------------------------------------------------------------------------------------------------------------
-- Triple Buffer Control Logic
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
-- 2010-10-11  	V0.02	zelenkaj	Bugfix: PCP can't be producer in any case => added generic
-- 2010-10-25	V0.03	zelenkaj	Use one Address Adder per DPR port side (reduces LE usage)
-- 2011-04-26	V0.04	zelenkaj	generic for clock domain selection
-- 2011-12-13	V0.05	zelenkaj	Added constants for one hot code
--									Reduced clkXing to two signals (one hot -> bin -> one hot)
------------------------------------------------------------------------------------------------------------------------
--	This logic implements the virtual triple buffers, by selecting the appropriate address offset
--	The output address offset has to be added to the input address.
--	The trigger signal switches to the next available buffer. The switch mechanism is implemented in the
--	PCP's clock domain. Thus the switch over on the PCP side is performed without delay. An AP switch over crosses
--  from AP to PCP clock domain (2x pcpClk) and back from PCP to AP (2x apClk).
------------------------------------------------------------------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.std_logic_arith.all;
USE ieee.std_logic_unsigned.all;

ENTITY tripleVBufLogic IS
	GENERIC(
			genOnePdiClkDomain_g		:		boolean := 		false;
			--base address of virtual buffers in DPR
			iVirtualBufferBase_g		:		INTEGER :=		0;
			--size of one virtual buffer in DPR (must be aligned!!!)
			iVirtualBufferSize_g		:		INTEGER :=		1024;
			--out address width
			iOutAddrWidth_g				:		INTEGER :=		13;
			--in address width
			iInAddrWidth_g				:		INTEGER :=		11;
			--ap is producer
			bApIsProducer				:		BOOLEAN :=		FALSE
	);
			
	PORT (
			pcpClk						: IN	STD_LOGIC;
			pcpReset					: IN	STD_LOGIC;
			pcpTrigger					: IN	STD_LOGIC;									--trigger virtual buffer change
			--pcpInAddr					: IN	STD_LOGIC_VECTOR(iInAddrWidth_g-1 DOWNTO 0);
			pcpOutAddrOff				: OUT	STD_LOGIC_VECTOR(iOutAddrWidth_g DOWNTO 0);
			pcpOutSelVBuf				: OUT	STD_LOGIC_VECTOR(2 DOWNTO 0);				--selected virtual buffer (one-hot coded)
			apClk						: IN	STD_LOGIC;
			apReset						: IN	STD_LOGIC;
			apTrigger					: IN	STD_LOGIC;									--trigger virtual buffer change
			--apInAddr					: IN	STD_LOGIC_VECTOR(iInAddrWidth_g-1 DOWNTO 0);
			apOutAddrOff				: OUT	STD_LOGIC_VECTOR(iOutAddrWidth_g DOWNTO 0);
			apOutSelVBuf				: OUT	STD_LOGIC_VECTOR(2 DOWNTO 0)				--selected virtual buffer (one-hot coded)
	);
END ENTITY tripleVBufLogic;

ARCHITECTURE rtl OF tripleVBufLogic IS
--constants
---virtual buffer base address
CONSTANT	iVirtualBufferBase0_c		:		INTEGER :=		0*iVirtualBufferSize_g + iVirtualBufferBase_g;
CONSTANT	iVirtualBufferBase1_c		:		INTEGER :=		1*iVirtualBufferSize_g + iVirtualBufferBase_g;
CONSTANT	iVirtualBufferBase2_c		:		INTEGER :=		2*iVirtualBufferSize_g + iVirtualBufferBase_g;
---one hot code
constant	cOneHotVirtualBuffer0		:		std_logic_vector(2 downto 0) := "001";
constant	cOneHotVirtualBuffer1		:		std_logic_vector(2 downto 0) := "010";
constant	cOneHotVirtualBuffer2		:		std_logic_vector(2 downto 0) := "100";
---triple buffer mechanism
----initial states
CONSTANT	initialValid_c				:		STD_LOGIC_VECTOR(2 DOWNTO 0) :=	cOneHotVirtualBuffer0;
CONSTANT	initialLocked_c				:		STD_LOGIC_VECTOR(2 DOWNTO 0) := cOneHotVirtualBuffer1;
CONSTANT	initialCurrent_c			:		STD_LOGIC_VECTOR(2 DOWNTO 0) := cOneHotVirtualBuffer2;
--signals
---PCP and AP selected virtual buffer
SIGNAL		pcpSelVBuf_s				:		STD_LOGIC_VECTOR(2 DOWNTO 0);				--selected virtual buffer by producer
SIGNAL		apSelVBuf_s					:		STD_LOGIC_VECTOR(2 DOWNTO 0);				--selected virtual buffer by consumer
SIGNAL		lockedVBuf_s				:		STD_LOGIC_VECTOR(2 DOWNTO 0);				--locked virtual buffer in producer clk domain

BEGIN
	
	pcpOutSelVBuf <= pcpSelVBuf_s;
	apOutSelVBuf <= apSelVBuf_s;
	
	theAddrCalcer : BLOCK
	--depending on the selected virtual buffer (???SelVBuf_s), the output address is calculated (???OutAddr)
	-- ???SelVBuf_s	| ???OutAddr
	-- -------------------------
	-- "001"		| ???InAddr + iVirtualBufferBase0_c
	-- "010"		| ???InAddr + iVirtualBufferBase1_c
	-- "100"		| ???InAddr + iVirtualBufferBase2_c
	SIGNAL	pcpAddrOffset, apAddrOffset:		STD_LOGIC_VECTOR(iOutAddrWidth_g DOWNTO 0);
	--SIGNAL	pcpSum, apSum				:		STD_LOGIC_VECTOR(iOutAddrWidth_g   DOWNTO 0);
	BEGIN
		
		--select address offset
		pcpAddrOffset <= 	CONV_STD_LOGIC_VECTOR(iVirtualBufferBase0_c, pcpAddrOffset'LENGTH) WHEN pcpSelVBuf_s = cOneHotVirtualBuffer0 ELSE
							CONV_STD_LOGIC_VECTOR(iVirtualBufferBase1_c, pcpAddrOffset'LENGTH) WHEN pcpSelVBuf_s = cOneHotVirtualBuffer1 ELSE
							CONV_STD_LOGIC_VECTOR(iVirtualBufferBase2_c, pcpAddrOffset'LENGTH) WHEN pcpSelVBuf_s = cOneHotVirtualBuffer2 ELSE
							(OTHERS => '0');
		pcpOutAddrOff <= pcpAddrOffset;
		--calculate address for dpr, leading zero is a sign!
		--pcpSum <= ('0' & conv_std_logic_vector(conv_integer(pcpInAddr), iOutAddrWidth_g-1)) + ('0' & pcpAddrOffset);
		--pcpOutAddr <= pcpSum(pcpOutAddr'RANGE);
		
		--select address offset
		apAddrOffset <= 	CONV_STD_LOGIC_VECTOR(iVirtualBufferBase0_c, apAddrOffset'LENGTH) WHEN apSelVBuf_s = cOneHotVirtualBuffer0 ELSE
							CONV_STD_LOGIC_VECTOR(iVirtualBufferBase1_c, apAddrOffset'LENGTH) WHEN apSelVBuf_s = cOneHotVirtualBuffer1 ELSE
							CONV_STD_LOGIC_VECTOR(iVirtualBufferBase2_c, apAddrOffset'LENGTH) WHEN apSelVBuf_s = cOneHotVirtualBuffer2 ELSE
							(OTHERS => '0');
		apOutAddrOff <= apAddrOffset;
		--calculate address for dpr, leading zero is a sign!
		--apSum <= ('0' & conv_std_logic_vector(conv_integer(apInAddr), iOutAddrWidth_g-1)) + ('0' & apAddrOffset);
		--apOutAddr <= apSum(apOutAddr'RANGE);
		
	END BLOCK theAddrCalcer;
		
	theLockSync : block
		constant cBinLockWidth : integer := 2;
		constant cBinLock0 : std_logic_vector(cBinLockWidth-1 downto 0) := "01";
		constant cBinLock1 : std_logic_vector(cBinLockWidth-1 downto 0) := "11";
		constant cBinLock2 : std_logic_vector(cBinLockWidth-1 downto 0) := "10";
		
		signal binLockedVBuf : std_logic_vector(cBinLockWidth-1 downto 0);
		signal binApSelVBuf : std_logic_vector(cBinLockWidth-1 downto 0);
	begin
		--conSelVBuf_s is in the PCP clock domain, thus the lockedVBuf_s signal must be
		-- synchronized from PCP clock- to AP clock domain!
		--In addition the one hot approach is transformed to save one line
		
		binLockedVBuf <= 	cBinLock0 when lockedVBuf_s = cOneHotVirtualBuffer0 else
							cBinLock1 when lockedVBuf_s = cOneHotVirtualBuffer1 else
							cBinLock2;
		
		apSelVBuf_s <= 	cOneHotVirtualBuffer0 when binApSelVBuf = cBinLock0 else
						cOneHotVirtualBuffer1 when binApSelVBuf = cBinLock1 else
						cOneHotVirtualBuffer2;
		
		vectorSync : FOR i in cBinLockWidth-1 DOWNTO 0 GENERATE
			theLockedSync : ENTITY work.sync
			generic map (
				doSync_g => not genOnePdiClkDomain_g
			)
			PORT MAP
			(
				din => binLockedVBuf(i),
				dout => binApSelVBuf(i),
				clk => apClk,
				rst => apReset
			);
		END GENERATE;
	end block;
	
	theTripleBufferLogic : BLOCK
	--The PCP triggers with triggerA and sets buffers to valid.
	--The AP triggers with triggerB and locks buffers for reading.
	SIGNAL	clk, rst					:		STD_LOGIC;
	SIGNAL	triggerA					:		STD_LOGIC;
	SIGNAL	triggerB, triggerB_s		:		STD_LOGIC;									--triggerB is in AP clock domain!
	SIGNAL	toggleB, toggleBsync		:		STD_LOGIC;									--toggleB is toggled by AP and synced to PCP
	SIGNAL	toggleEdge					:		STD_LOGIC_VECTOR(1 DOWNTO 0);
	SIGNAL	locked						:		STD_LOGIC_VECTOR(2 DOWNTO 0);
	SIGNAL	currentA					:		STD_LOGIC_VECTOR(2 DOWNTO 0);				--current selected buffer by PCP
--	SIGNAL	valid						:		STD_LOGIC_VECTOR(2 DOWNTO 0);
	BEGIN
		--triple buffer logic is implemented in PCP clock domain!
		clk <= pcpClk;
		rst <= pcpReset;
		
		--triggerA is the producer's trigger
		triggerA <= pcpTrigger when bApIsProducer = false else triggerB_s;
		
		--conTrigger pulse is in AP clock domain, thus different clock rates will produce more or less pulses!
		---thus a toggling signal crosses the clock domain
		genToggleB : PROCESS(apClk, apReset)
		BEGIN
			IF apReset = '1' THEN
				toggleB <= '0';
			ELSIF apClk = '1' AND apClk'EVENT THEN --CAUTION: AP clock is used!
				IF apTrigger = '1' THEN
					toggleB <= not toggleB;
				END IF;
			END IF;
		END PROCESS genToggleB;
		
	theToggleSync : ENTITY work.sync
	generic map (
		doSync_g => not genOnePdiClkDomain_g
	)
	PORT MAP
	(
		din => toggleB,
		dout => toggleBsync,
		clk => clk,
		rst => rst
	);
		
		toggleShiftReg: PROCESS(clk, rst)
		BEGIN
			IF rst = '1' THEN
				toggleEdge <= (OTHERS => '0');
			ELSIF clk = '1' AND clk'event THEN
				--shift register
				toggleEdge <= toggleEdge(0) & toggleBsync;
			END IF;
		END PROCESS toggleShiftReg;
		triggerB_s <= toggleEdge(1) xor toggleEdge(0);
		--triggerB is the consumer's trigger
		triggerB <= triggerB_s when bApIsProducer = false else pcpTrigger;
		
		--currentA is set by PCP (currently used buffer by PCP)
		pcpSelVBuf_s <= currentA when bApIsProducer = false else locked;
		
		--locked virtual buffer in PCP clock domain
		lockedVBuf_s <= locked when bApIsProducer = false else currentA;
		
		tripleBufMechanism : PROCESS(clk, rst)
		VARIABLE	valid_v				:	STD_LOGIC_VECTOR(2 DOWNTO 0);
		BEGIN
			IF rst = '1' THEN
				--initial state:
				---buffer "001" is valid
				valid_v := initialValid_c;
				---buffer "010" is locked
				locked <= initialLocked_c;
				---buffer "100" is currently used by PCP
				currentA <= initialCurrent_c;
			ELSIF clk = '1' AND clk'EVENT THEN
				IF triggerA = '1' THEN
				--PCP triggers buffer change
				---set valid to current selected buffer
				---search for free buffer (not locked and valid)
					
					valid_v := currentA;
					
					--free buffer search ex.:
					-- locked "001"
					-- valid  "010"
					-- ============
					-- free   "100"
					currentA <= not locked and not valid_v;
				END IF;	
				IF triggerB = '1' THEN
				--AP triggers buffer change
				---change AP to valid buffer
					
					locked <= valid_v;
				END IF;
			END IF;
		END PROCESS tripleBufMechanism;
	END BLOCK theTripleBufferLogic;
	
END ARCHITECTURE rtl;
