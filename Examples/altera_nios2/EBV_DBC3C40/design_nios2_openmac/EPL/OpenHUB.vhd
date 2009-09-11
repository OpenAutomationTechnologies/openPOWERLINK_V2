------------------------------------------------------------------------------------------------------------------------
-- OpenHUB
--
-- Copyright (C) 2009 B&R
--
-- This program is free software; you can redistribute it and/or modify
-- it under the terms of the GNU General Public License as published by
-- the Free Software Foundation; either version 2 of the License, or
-- (at your option) any later version.
--
-- Note: RxDv, RxDat0 and RxDat1 have to be synchron to CLK
--       ReceivePort return currently active Port
--
------------------------------------------------------------------------------------------------------------------------
-- Version History
------------------------------------------------------------------------------------------------------------------------
-- 2009-08-07  V0.01        Converted from V3.1 to first official version.
------------------------------------------------------------------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_unsigned.ALL;
USE ieee.std_logic_1164.ALL;
USE ieee.std_logic_arith.ALL;

ENTITY OpenHUB IS
	GENERIC	( Ports				:		integer := 3 );
	PORT	( nRst				: IN	std_logic;
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
		
do: PROCESS (nRst, Clk)
	VARIABLE Active				: boolean;
	VARIABLE Master				: integer RANGE 0 TO Ports;
	VARIABLE Master_at_Coll		: integer RANGE 0 TO Ports;
	VARIABLE Coll				: boolean;
	VARIABLE RxDvM				: std_logic_vector(Ports DOWNTO 0);

BEGIN
	IF nRst = '0' THEN
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
			
			-- Neue TransmitMask nur übernehmen wenn gerade kein Frame aktiv ist, um zu vermeiden
			-- dass während eines Frames umgeschaltet und der Frame zerstört wird
			TransmitMask_L <= TransmitMask;
		ELSE
			FOR i IN 1 TO Ports LOOP -- Empfangenen Frame an alle Ports ausgeben
				IF i /= Master THEN  -- Nur nicht an den Ports wo der Frame reinkommt .. eh klar
					
					-- Daten senden wenn der Port freigeschalten ist (TransmitMask), oder wenn der
					-- sendende Port der interne Port ist (dann wird immer an alle externen gesendet)
					IF TransmitMask_L(i) = '1' OR Master = internPort THEN
					
						TxEnI(i)   <= '1';
						TxDatI0(i) <= RxDatL0(Master);
						TxDatI1(i) <= RxDatL1(Master);
					END IF;

					-- Wenn auf dem Port an den gesendet wird auch gerade ein Frame reinkommt ist das
					-- eine Kollision
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

	-- Hier wird der aktive Empfangs-Port (ReceivePort) ausgegeben. Es kann erkannt werden ob
	-- der HUB gerade inaktiv ist (0) oder ob an einem der Ports ein Frame empfangen wird (1..n)
	ReceivePort <= Master;

END PROCESS do;
END struct;

