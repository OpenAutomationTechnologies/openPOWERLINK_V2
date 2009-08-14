-----------------------------------------------------------
--
--	DPR (Altera) for EPL MAC
--
-- Copyright (C) 2009  B&R 
--     
-- This program is free software; you can redistribute it and/or modify
-- it under the terms of the GNU General Public License as published by
-- the Free Software Foundation; either version 2 of the License, or
-- (at your option) any later version.
--  
--
-----------------------------------------------------------------------------------------------------------------------------
--
--
-------------------------------------------------------------------------------
-- 16 / 16 DPR for openMac
--
LIBRARY ieee;                   
USE ieee.std_logic_1164.all;    
USE ieee.std_logic_arith.all;   
USE ieee.std_logic_unsigned.all;
LIBRARY altera_mf;
USE altera_mf.altera_mf_components.all;
LIBRARY lpm;
USE lpm.lpm_components.all;

entity Dpr_16_16 is
  generic(Simulate	:  in	boolean := false);
  port (
	 ClkA,  ClkB		:  in  std_logic;
	 WeA,   WeB			:  in  std_logic := '0';
	 EnA,   EnB			:  in  std_logic := '1';
	 BeA				:  in  std_logic_vector ( 1 downto 0) := "11";
	 AddrA				:  in  std_logic_vector ( 7 downto 0);
	 DiA				:  in  std_logic_vector (15 downto 0) := (others => '0');
	 DoA				:  out  std_logic_vector(15 downto 0); 
	 BeB				:  in  std_logic_vector ( 1 downto 0) := "11";
	 AddrB 				:  in  std_logic_vector ( 7 downto 0);
	 DiB				:  in  std_logic_vector (15 downto 0) := (others => '0');
	 DoB				:  out  std_logic_vector(15 downto 0) 
	 );
end Dpr_16_16;

architecture struct of Dpr_16_16 is
begin

Ram: COMPONENT altsyncram
		GENERIC MAP ( OPERATION_MODE => "BIDIR_DUAL_PORT",
					  WIDTH_A => 16, WIDTHAD_A => 8, NUMWORDS_A => 256, WIDTH_BYTEENA_A => 2,
					  WIDTH_B => 16, WIDTHAD_B => 8, NUMWORDS_B => 256, WIDTH_BYTEENA_B => 2
					 )
		PORT MAP( 
			clock0    => ClkA,		clock1    => ClkB,
			wren_a    => WeA,		wren_b    => WeB,
			clocken0  => EnA,		clocken1  => EnB,
			byteena_a => BeA,		byteena_b => BeB,
			address_a => AddrA,		address_b => AddrB,					
			data_a    => DiA,		data_b    => DiB, 
			q_a       => DoA,		q_b       => DoB
			);

end struct;

-------------------------------------------------------------------------------
-- 8 / 16 DPR for openMac
--
LIBRARY ieee;                   
USE ieee.std_logic_1164.all;    
USE ieee.std_logic_arith.all;   
USE ieee.std_logic_unsigned.all;
LIBRARY altera_mf;
USE altera_mf.altera_mf_components.all;
LIBRARY lpm;
USE lpm.lpm_components.all;

entity Dpr_16_32 is
  generic(Simulate	:  in	boolean := false);
  port (
	 ClkA,  ClkB		:  in  std_logic;
	 WeA				:  in  std_logic := '0';
	 EnA,   EnB			:  in  std_logic := '1';
	 AddrA				:  in  std_logic_vector ( 7 downto 0);
	 DiA				:  in  std_logic_vector (15 downto 0) := (others => '0');
	 BeA				:  in  std_logic_vector ( 1 downto 0) := "11";
	 AddrB 				:  in  std_logic_vector ( 6 downto 0);
	 DoB				:  out  std_logic_vector(31 downto 0) 
	 );
end Dpr_16_32;

architecture struct of Dpr_16_32 is
begin

Ram: COMPONENT altsyncram
		GENERIC MAP ( OPERATION_MODE => "DUAL_PORT", INIT_FILE => "..\src\Null_alt.hex",
					  WIDTH_A => 16, WIDTHAD_A => 8, NUMWORDS_A => 256, WIDTH_BYTEENA_A => 2,
					  WIDTH_B => 32, WIDTHAD_B => 7, NUMWORDS_B => 128
					 )
		PORT MAP( 
			clock0    => ClkA,		clock1    => ClkB,
			wren_a    => WeA,
			clocken0  => EnA,		clocken1  => EnB,
			byteena_a => BeA,
			address_a => AddrA,		address_b => AddrB,					
			data_a    => DiA,
									q_b       => DoB
			);

end struct;

-------------------------------------------------------------------------------
-- 8 / 8 DPR for openMac
--		

LIBRARY ieee;                   
USE ieee.std_logic_1164.all;    
USE ieee.std_logic_arith.all;   
USE ieee.std_logic_unsigned.all;
LIBRARY altera_mf;
USE altera_mf.altera_mf_components.all;
LIBRARY lpm;
USE lpm.lpm_components.all;

entity Dpr_8_8 is
  generic(Simulate	:  in	boolean := false);
  port (
	 ClkA,  ClkB		:  in  std_logic;
	 WeA,   WeB			:  in  std_logic := '0';
	 EnA,   EnB			:  in  std_logic := '1';
	 AddrA, AddrB		:  in  std_logic_vector ( 8 downto 0);
	 DiA,   DiB			:  in  std_logic_vector ( 7 downto 0) := (others => '0');
	 DoA,   DoB			:  out  std_logic_vector( 7 downto 0) 
	 );
end Dpr_8_8;

architecture struct of Dpr_8_8 is
begin

Ram: COMPONENT altsyncram
		GENERIC MAP (	operation_mode => "BIDIR_DUAL_PORT", lpm_type => "altsyncram",
						width_a => 8, widthad_a => 9, numwords_a => 512, outdata_reg_a => "UNREGISTERED",
						width_b => 8, widthad_b => 9, numwords_b => 512, outdata_reg_b => "UNREGISTERED",
						indata_reg_b => "CLOCK1",   address_reg_b => "CLOCK1", wrcontrol_wraddress_reg_b => "CLOCK1"
		)
		PORT MAP( 
			clock0    => ClkA,		clock1    => ClkB,
			wren_a    => WeA,		wren_b    => WeB,		
			clocken0  => EnA,		clocken1  => EnB,
			address_a => AddrA,		address_b => AddrB,					
			data_a    => DiA,		data_b    => DiB,
			q_a       => DoA,		q_b       => DoB
			);

end struct;



LIBRARY ieee;
USE ieee.std_logic_1164.all;

LIBRARY altera_mf;
USE altera_mf.altera_mf_components.all;

ENTITY Shadow_Ram IS
	PORT
	(
		address		: IN STD_LOGIC_VECTOR (8 DOWNTO 0);
		byteena		: IN STD_LOGIC_VECTOR (1 DOWNTO 0);
		clock		: IN STD_LOGIC ;
		clken		: IN STD_LOGIC ;
		data		: IN STD_LOGIC_VECTOR (15 DOWNTO 0);
		wren		: IN STD_LOGIC ;
		q		: OUT STD_LOGIC_VECTOR (15 DOWNTO 0)
	);
END Shadow_Ram;


ARCHITECTURE SYN OF shadow_ram IS

	SIGNAL sub_wire0	: STD_LOGIC_VECTOR (15 DOWNTO 0);



	COMPONENT altsyncram
	GENERIC (
		intended_device_family		: STRING;
		width_a		: NATURAL;
		widthad_a		: NATURAL;
		numwords_a		: NATURAL;
		operation_mode		: STRING;
		outdata_reg_a		: STRING;
		indata_aclr_a		: STRING;
		wrcontrol_aclr_a		: STRING;
		address_aclr_a		: STRING;
		outdata_aclr_a		: STRING;
		width_byteena_a		: NATURAL;
		byte_size		: NATURAL;
		byteena_aclr_a		: STRING;
		lpm_hint		: STRING;
		lpm_type		: STRING
	);
	PORT (
			clocken0	: IN STD_LOGIC ;
			wren_a	: IN STD_LOGIC ;
			clock0	: IN STD_LOGIC ;
			byteena_a	: IN STD_LOGIC_VECTOR (1 DOWNTO 0);
			address_a	: IN STD_LOGIC_VECTOR (8 DOWNTO 0);
			q_a	: OUT STD_LOGIC_VECTOR (15 DOWNTO 0);
			data_a	: IN STD_LOGIC_VECTOR (15 DOWNTO 0)
	);
	END COMPONENT;

BEGIN
	q    <= sub_wire0(15 DOWNTO 0);

	altsyncram_component : altsyncram
	GENERIC MAP (
		intended_device_family => "Cyclone",
		width_a => 16,
		widthad_a => 9,
		numwords_a => 512,
		operation_mode => "SINGLE_PORT",
		outdata_reg_a => "UNREGISTERED",
		indata_aclr_a => "NONE",
		wrcontrol_aclr_a => "NONE",
		address_aclr_a => "NONE",
		outdata_aclr_a => "NONE",
		width_byteena_a => 2,
		byte_size => 8,
		byteena_aclr_a => "NONE",
		lpm_hint => "ENABLE_RUNTIME_MOD=NO",
		lpm_type => "altsyncram"
	)
	PORT MAP (
		clocken0 => clken,
		wren_a => wren,
		clock0 => clock,
		byteena_a => byteena,
		address_a => address,
		data_a => data,
		q_a => sub_wire0
	);



END SYN;

