------------------------------------------------------------------------------------------------------------------------
-- OpenMAC - DPR for Altera FPGA
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
------------------------------------------------------------------------------------------------------------------------
-- Version History
------------------------------------------------------------------------------------------------------------------------      
--  2009-08-07  V0.01		Converted to official version.
--	2010-05-03	V0.02		added packet buffer dpr
--  2011-12-22	V0.03		added initialization files
--							removed dpr_8_8
--  2012-01-04  V0.04       replaced initialization files with mif
------------------------------------------------------------------------------------------------------------------------

-------------------------------------------------------------------------------
-- 16 / 16 DPR
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
  generic(Simulate	:  in	boolean);
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
		GENERIC MAP ( OPERATION_MODE => "BIDIR_DUAL_PORT", INIT_FILE => "mif/dpr_16_16.mif",
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
-- 16 / 32 DPR
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
  generic(Simulate	:  in	boolean);
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
		GENERIC MAP ( OPERATION_MODE => "DUAL_PORT", INIT_FILE => "mif/dpr_16_32.mif",
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
-- Packet buffer 
--
LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.std_logic_arith.all;
USE ieee.std_logic_unsigned.all;
USE ieee.math_real.log2;
USE ieee.math_real.ceil;

LIBRARY altera_mf;
USE altera_mf.all;

ENTITY OpenMAC_DPRpackets IS
	GENERIC
	(
		memSizeLOG2_g : integer := 10;
		memSize_g : integer := 1024
	);
	PORT
	(
		address_a		: IN STD_LOGIC_VECTOR (memSizeLOG2_g-2 DOWNTO 0);
		address_b		: IN STD_LOGIC_VECTOR (memSizeLOG2_g-3 DOWNTO 0);
		byteena_a		: IN STD_LOGIC_VECTOR (1 DOWNTO 0) :=  (OTHERS => '1');
		byteena_b		: IN STD_LOGIC_VECTOR (3 DOWNTO 0) :=  (OTHERS => '1');
		clock_a		: IN STD_LOGIC  := '1';
		clock_b		: IN STD_LOGIC ;
		data_a		: IN STD_LOGIC_VECTOR (15 DOWNTO 0);
		data_b		: IN STD_LOGIC_VECTOR (31 DOWNTO 0);
		rden_a		: IN STD_LOGIC  := '1';
		rden_b		: IN STD_LOGIC  := '1';
		wren_a		: IN STD_LOGIC  := '0';
		wren_b		: IN STD_LOGIC  := '0';
		q_a		: OUT STD_LOGIC_VECTOR (15 DOWNTO 0);
		q_b		: OUT STD_LOGIC_VECTOR (31 DOWNTO 0)
	);
END OpenMAC_DPRpackets;


ARCHITECTURE SYN OF openmac_dprpackets IS

	SIGNAL sub_wire0	: STD_LOGIC_VECTOR (15 DOWNTO 0);
	SIGNAL sub_wire1	: STD_LOGIC_VECTOR (31 DOWNTO 0);



	COMPONENT altsyncram
	GENERIC (
		address_reg_b		: STRING;
		byteena_reg_b		: STRING;
		byte_size		: NATURAL;
		clock_enable_input_a		: STRING;
		clock_enable_input_b		: STRING;
		clock_enable_output_a		: STRING;
		clock_enable_output_b		: STRING;
		indata_reg_b		: STRING;
		intended_device_family		: STRING;
		lpm_type		: STRING;
		numwords_a		: NATURAL;
		numwords_b		: NATURAL;
		operation_mode		: STRING;
		outdata_aclr_a		: STRING;
		outdata_aclr_b		: STRING;
		outdata_reg_a		: STRING;
		outdata_reg_b		: STRING;
		power_up_uninitialized		: STRING;
		read_during_write_mode_port_a		: STRING;
		read_during_write_mode_port_b		: STRING;
		widthad_a		: NATURAL;
		widthad_b		: NATURAL;
		width_a		: NATURAL;
		width_b		: NATURAL;
		width_byteena_a		: NATURAL;
		width_byteena_b		: NATURAL;
		wrcontrol_wraddress_reg_b		: STRING
	);
	PORT (
			wren_a	: IN STD_LOGIC ;
			clock0	: IN STD_LOGIC ;
			wren_b	: IN STD_LOGIC ;
			clock1	: IN STD_LOGIC ;
			byteena_a	: IN STD_LOGIC_VECTOR (1 DOWNTO 0);
			byteena_b	: IN STD_LOGIC_VECTOR (3 DOWNTO 0);
			address_a	: IN STD_LOGIC_VECTOR (memSizeLOG2_g-2 DOWNTO 0);
			address_b	: IN STD_LOGIC_VECTOR (memSizeLOG2_g-3 DOWNTO 0);
			rden_a	: IN STD_LOGIC ;
			q_a	: OUT STD_LOGIC_VECTOR (15 DOWNTO 0);
			rden_b	: IN STD_LOGIC ;
			q_b	: OUT STD_LOGIC_VECTOR (31 DOWNTO 0);
			data_a	: IN STD_LOGIC_VECTOR (15 DOWNTO 0);
			data_b	: IN STD_LOGIC_VECTOR (31 DOWNTO 0)
	);
	END COMPONENT;

BEGIN
	q_a    <= sub_wire0(15 DOWNTO 0);
	q_b    <= sub_wire1(31 DOWNTO 0);

	altsyncram_component : altsyncram
	GENERIC MAP (
		address_reg_b => "CLOCK1",
		byteena_reg_b => "CLOCK1",
		byte_size => 8,
		clock_enable_input_a => "BYPASS",
		clock_enable_input_b => "BYPASS",
		clock_enable_output_a => "BYPASS",
		clock_enable_output_b => "BYPASS",
		indata_reg_b => "CLOCK1",
		intended_device_family => "Cyclone III",
		lpm_type => "altsyncram",
		numwords_a => memSize_g/2,
		numwords_b => memSize_g/4,
		operation_mode => "BIDIR_DUAL_PORT",
		outdata_aclr_a => "NONE",
		outdata_aclr_b => "NONE",
		outdata_reg_a => "CLOCK0",
		outdata_reg_b => "CLOCK1",
		power_up_uninitialized => "FALSE",
		read_during_write_mode_port_a => "NEW_DATA_NO_NBE_READ",
		read_during_write_mode_port_b => "NEW_DATA_NO_NBE_READ",
		widthad_a => memSizeLOG2_g-1,
		widthad_b => memSizeLOG2_g-2,
		width_a => 16,
		width_b => 32,
		width_byteena_a => 2,
		width_byteena_b => 4,
		wrcontrol_wraddress_reg_b => "CLOCK1"
	)
	PORT MAP (
		wren_a => wren_a,
		clock0 => clock_a,
		wren_b => wren_b,
		clock1 => clock_b,
		byteena_a => byteena_a,
		byteena_b => byteena_b,
		address_a => address_a,
		address_b => address_b,
		rden_a => rden_a,
		rden_b => rden_b,
		data_a => data_a,
		data_b => data_b,
		q_a => sub_wire0,
		q_b => sub_wire1
	);



END SYN;
