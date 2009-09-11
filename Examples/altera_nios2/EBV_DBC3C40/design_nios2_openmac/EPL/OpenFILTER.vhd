------------------------------------------------------------------------------------------------------------------------
-- OpenFILTER
--
-- Copyright (C) 2009 B&R
--
-- This program is free software; you can redistribute it and/or modify
-- it under the terms of the GNU General Public License as published by
-- the Free Software Foundation; either version 2 of the License, or
-- (at your option) any later version.
--
-- Note: RxDv and RxDat have to be synchron to Clk
--	     The following Conditions are checked:
--         RxDV >163.64탎ec HIGH -> invalid
--         RxDV <0.64탎ec LOW -> invalid
--         RxDV >10.24탎ec LOW -> valid
--         RxDV 4x <5.12탎ec HIGH -> invalid
--         RxDV >5.12탎ec HIGH -> valid
--
------------------------------------------------------------------------------------------------------------------------
-- Version History
------------------------------------------------------------------------------------------------------------------------
-- 2009-08-07  V0.01        Converted from V1.1 to first official version.
------------------------------------------------------------------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_unsigned.ALL;
USE ieee.std_logic_1164.ALL;
USE ieee.std_logic_arith.ALL;

ENTITY OpenFILTER IS
    PORT    (   nRst                : IN	std_logic;
                Clk                 : IN    std_logic;
                nCheckShortFrames   : IN    std_logic := '0';	-- Rx Port from Hub;
                RxDvIn              : IN    std_logic;
                RxDatIn             : IN    std_logic_vector(1 DOWNTO 0);
                RxDvOut             : OUT   std_logic;
                RxDatOut            : OUT   std_logic_vector(1 DOWNTO 0);
                TxEnIn              : IN    std_logic;
                TxDatIn             : IN    std_logic_vector(1 DOWNTO 0);
                TxEnOut             : OUT   std_logic;
                TxDatOut            : OUT   std_logic_vector(1 DOWNTO 0)
			);
END ENTITY OpenFILTER;

ARCHITECTURE struct OF OpenFILTER IS
	
    SIGNAL Cnt_Rx_high          : std_logic_vector(13 DOWNTO 0);
    SIGNAL Cnt_Rx_low           : std_logic_vector(9 DOWNTO 0);
    SIGNAL RxHigh_ToLong        : std_logic;
    SIGNAL Cnt_RxHigh_ToShort   : std_logic_vector(1 DOWNTO 0);
    SIGNAL RxHigh_ToShort       : std_logic;
    SIGNAL RxHigh_ToShort_temp  : std_logic;
    SIGNAL RxDvLatch            : std_logic;
    SIGNAL RxDvLatch2           : std_logic;
    SIGNAL RxDataValidLatch     : std_logic;

BEGIN
	
    RxDvOut     <= RxDvIn AND NOT Cnt_Rx_high(Cnt_Rx_high'high) AND (NOT RxHigh_ToShort OR nCheckShortFrames);
    RxDatOut(0) <= RxDatIn(0) AND NOT Cnt_Rx_high(Cnt_Rx_high'high) AND (NOT RxHigh_ToShort OR nCheckShortFrames);
    RxDatOut(1) <= RxDatIn(1) AND NOT Cnt_Rx_high(Cnt_Rx_high'high) AND (NOT RxHigh_ToShort OR nCheckShortFrames);

    TxEnOut     <= TxEnIn AND NOT Cnt_Rx_high(Cnt_Rx_high'high);
    TxDatOut(0) <= TxDatIn(0) AND NOT Cnt_Rx_high(Cnt_Rx_high'high);
    TxDatOut(1) <= TxDatIn(1) AND NOT Cnt_Rx_high(Cnt_Rx_high'high);
	
do: PROCESS (nRst, Clk)

BEGIN
    IF nRst = '0' THEN
        RxHigh_ToLong       <= '0';
        Cnt_RxHigh_ToShort  <= (OTHERS => '0');
        RxHigh_ToShort      <= '0';
        RxHigh_ToShort_temp <= '0';
        RxDvLatch           <= '0';
        RxDvLatch2          <= '0';
        RxDataValidLatch    <= '0';
        Cnt_Rx_high			<= (OTHERS => '0');
        Cnt_Rx_low			<= (OTHERS => '0');
    ELSIF rising_edge(Clk) THEN
        RxDvLatch           <= RxDvIn;
        RxDvLatch2          <= RxDvLatch;
        RxDataValidLatch    <= RxDvLatch OR RxDvLatch2;

        IF RxDvLatch = '1' OR RxDvLatch2 = '1' THEN
            IF Cnt_Rx_high(Cnt_Rx_high'high) = '0' THEN Cnt_Rx_high <= Cnt_Rx_high + 1;  END IF;	-- 163.84탎ec (max Size)
            Cnt_Rx_low <= (OTHERS => '0');
			
            IF 		RxDataValidLatch = '0'  THEN RxHigh_ToShort_temp <= '1';
            ELSIF 	Cnt_Rx_high(8) = '1'    THEN RxHigh_ToShort_temp <= '0';	--   5.12탎ec (min Size)
            ELSE								 RxHigh_ToShort_temp <= RxHigh_ToShort_temp;
            END IF;
        ELSE
            IF Cnt_Rx_high(Cnt_Rx_high'high) = '1' THEN
                IF Cnt_Rx_low(Cnt_Rx_low'high) = '0'    THEN Cnt_Rx_low <= Cnt_Rx_low + 1;			--  10.24탎ec
                ELSE							        Cnt_Rx_high <= (OTHERS => '0');
                END IF;
            ELSE
                IF Cnt_Rx_low(5) = '0'  THEN Cnt_Rx_low  <= Cnt_Rx_low + 1;							--   0.64탎ec (min. Gap between two Frames)
                ELSE               	    Cnt_Rx_high <= (OTHERS => '0');
                END IF;
            END IF;
            IF RxDataValidLatch = '1' THEN
                IF RxHigh_ToShort_temp = '1' THEN
                    IF Cnt_RxHigh_ToShort /= "11" 	THEN
                        Cnt_RxHigh_ToShort  <= Cnt_RxHigh_ToShort + 1;
                        RxHigh_ToShort 	    <= '0';
                    ELSE
                        RxHigh_ToShort 	    <= '1';
                    END IF;
                ELSE
                    Cnt_RxHigh_ToShort  <= (OTHERS => '0');
                    RxHigh_ToShort 	    <= '0';
                END IF;
            END IF;
            RxHigh_ToShort_temp <= '0';
        END IF;
    END IF;

END PROCESS do;
END struct;
