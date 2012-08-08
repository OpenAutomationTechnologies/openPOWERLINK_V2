------------------------------------------------------------------------------------------------------------------------
-- OpenHUB
--
-- 	  Copyright (C) 2009 B&R
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
-- Note: RxDv, RxDat0 and RxDat1 have to be synchron to CLK
--       ReceivePort return currently active Port
--
------------------------------------------------------------------------------------------------------------------------
-- Version History
------------------------------------------------------------------------------------------------------------------------
-- 2009-08-07  	V0.01				Converted from V3.1 to first official version.
-- 2011-11-28	V0.02	zelenkaj	Changed reset level to high-active
------------------------------------------------------------------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_unsigned.ALL;
USE ieee.std_logic_1164.ALL;
USE ieee.std_logic_arith.ALL;

ENTITY OpenHUB IS
	GENERIC	( Ports				:		integer := 3 );
	PORT	( Rst				: IN	std_logic;
			  Clk				: IN	std_logic;
			  RxDv				: IN	std_logic_vector(Ports DOWNTO 1);
			  RxDat0, RxDat1	: IN	std_logic_vector(Ports DOWNTO 1);
			  TxEn				: OUT	std_logic_vector(Ports DOWNTO 1);
			  TxDat0, TxDat1	: OUT	std_logic_vector(Ports DOWNTO 1);
			  internPort		: IN	integer RANGE 1 TO Ports := 1;
			  TransmitMask		: IN	std_logic_vector(Ports DOWNTO 1) := (OTHERS => '1');
			  ReceivePort		: OUT	integer RANGE 0 TO Ports
			);
END ENTITY OpenHUB;

ARCHITECTURE struct OF OpenHUB IS
	SIGNAL RxDvI, RxDvL			: std_logic_vector(Ports DOWNTO 0);
	SIGNAL RxDatI0, RxDatL0		: std_logic_vector(Ports DOWNTO 0);
	SIGNAL RxDatI1, RxDatL1		: std_logic_vector(Ports DOWNTO 0);
	SIGNAL TxEnI				: std_logic_vector(Ports DOWNTO 0);
	SIGNAL TxDatI0				: std_logic_vector(Ports DOWNTO 0);
	SIGNAL TxDatI1				: std_logic_vector(Ports DOWNTO 0);
	SIGNAL MasterAtCollNumber	: integer RANGE 0 TO Ports;
	SIGNAL HubActive			: boolean;
	SIGNAL CollStatus			: boolean;
	SIGNAL TransmitMask_L       : std_logic_vector(Ports DOWNTO 1);

BEGIN

	RxDvI(Ports DOWNTO 0)   <= RxDv(Ports DOWNTO 1) & '0';
	RxDatI0(Ports DOWNTO 0) <= RxDat0(Ports DOWNTO 1) & '0';
	RxDatI1(Ports DOWNTO 0) <= RxDat1(Ports DOWNTO 1) & '0';
	TxEn(Ports DOWNTO 1)    <= TxEnI(Ports DOWNTO 1);
	TxDat0(Ports DOWNTO 1)  <= TxDatI0(Ports DOWNTO 1);
	TxDat1(Ports DOWNTO 1)  <= TxDatI1(Ports DOWNTO 1);
		
do: PROCESS (Rst, Clk)
	VARIABLE Active				: boolean;
	VARIABLE Master				: integer RANGE 0 TO Ports;
	VARIABLE Master_at_Coll		: integer RANGE 0 TO Ports;
	VARIABLE Coll				: boolean;
	VARIABLE RxDvM				: std_logic_vector(Ports DOWNTO 0);

BEGIN
	IF Rst = '1' THEN
		RxDvL <= (OTHERS => '0'); RxDatL0 <= (OTHERS => '0'); RxDatL1 <= (OTHERS => '0');
		TxEnI <= (OTHERS => '0'); TxDatI0 <= (OTHERS => '0'); TxDatI1 <= (OTHERS => '0');
		Active := false;
		Master := 0;
		Master_at_Coll := 0;
		Coll := false;
		TransmitMask_L <= (OTHERS => '1');
	ELSIF rising_edge(Clk) THEN
		RxDvL <= RxDvI; RxDatL0 <= RxDatI0; RxDatL1 <= RxDatI1;
		IF Active = false THEN
			IF RxDvL /= 0 THEN
				FOR i IN 1 TO Ports LOOP
					IF RxDvL(i) = '1' AND (RxDatL0(i) = '1' OR RxDatL1(i) = '1') THEN
						Master := i;
						Active := true;
						EXIT;
					END IF;
				END LOOP;
			END IF;
		ELSE
			IF RxDvL(Master) = '0' AND RxDvI(Master) = '0' THEN
				Master := 0;
			END IF;
			IF RxDvL = 0 AND RxDvI = 0 THEN
				Active := false;
			END IF;
		END IF;

		IF Master = 0 THEN
			TxEnI <= (OTHERS => '0'); TxDatI0 <= (OTHERS => '0'); TxDatI1 <= (OTHERS => '0');
			
			-- Overtake new TransmitMask only, when there is no active frame.
			TransmitMask_L <= TransmitMask;
		ELSE
			FOR i IN 1 TO Ports LOOP -- output received frame to every port
				IF i /= Master THEN  --  but not to the port where it is coming from - "eh kloar!"
					
					-- only send data to active ports (=> TransmitMask is set to '1') or the internal Port (Mac)
					IF TransmitMask_L(i) = '1' OR Master = internPort THEN
					
						TxEnI(i)   <= '1';
						TxDatI0(i) <= RxDatL0(Master);
						TxDatI1(i) <= RxDatL1(Master);
					END IF;

					-- If there is a frame received and another is sent => collision!
					IF RxDvL(i) = '1' THEN
						Coll := true;
						Master_at_Coll := Master;
					END IF;
				END IF;
			END LOOP;
		END IF;

		IF Coll = true THEN
			TxEnI(Master_at_Coll) <= '1'; TxDatI0(Master_at_Coll) <= '1'; TxDatI1(Master_at_Coll) <= '0';
			RxDvM := RxDvL;
			RxDvM(Master_at_Coll) := '0';
			IF RxDvM = 0 THEN
				TxEnI(Master_at_Coll) <= '0'; TxDatI0(Master_at_Coll) <= '0'; TxDatI1(Master_at_Coll) <= '0';
				Coll := false;
				Master_at_Coll := 0;
			END IF;
		END IF;
	END IF;

	HubActive <= Active;
	MasterAtCollNumber <= Master_at_Coll;
	CollStatus <= Coll;

	-- Output the Master Port - identifies the port (1...n) which has received the packet.
	-- If Master is 0, the Hub is inactive.
	ReceivePort <= Master;

END PROCESS do;
END struct;

