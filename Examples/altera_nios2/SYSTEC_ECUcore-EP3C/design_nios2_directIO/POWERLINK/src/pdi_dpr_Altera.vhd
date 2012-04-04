------------------------------------------------------------------------------------------------------------------------
-- Process Data Interface (PDI) DPR
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
-- 2010-06-28  	V0.01	zelenkaj	First version
-- 2010-08-16	V0.02	zelenkaj	changed header
-- 2012-01-03   V0.03   zelenkaj    added initialization file (mif)
------------------------------------------------------------------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_1164.all;

LIBRARY altera_mf;
USE altera_mf.all;

ENTITY pdi_dpr IS
	GENERIC
	(
		NUM_WORDS		: INTEGER := 1024;
		LOG2_NUM_WORDS	: INTEGER := 10
	);
	PORT
	(
		address_a		: IN STD_LOGIC_VECTOR (LOG2_NUM_WORDS-1 DOWNTO 0);
		address_b		: IN STD_LOGIC_VECTOR (LOG2_NUM_WORDS-1 DOWNTO 0);
		byteena_a		: IN STD_LOGIC_VECTOR (3 DOWNTO 0) :=  (OTHERS => '1');
		byteena_b		: IN STD_LOGIC_VECTOR (3 DOWNTO 0) :=  (OTHERS => '1');
		clock_a		: IN STD_LOGIC  := '1';
		clock_b		: IN STD_LOGIC ;
		data_a		: IN STD_LOGIC_VECTOR (31 DOWNTO 0);
		data_b		: IN STD_LOGIC_VECTOR (31 DOWNTO 0);
		wren_a		: IN STD_LOGIC  := '0';
		wren_b		: IN STD_LOGIC  := '0';
		q_a		: OUT STD_LOGIC_VECTOR (31 DOWNTO 0);
		q_b		: OUT STD_LOGIC_VECTOR (31 DOWNTO 0)
	);
END pdi_dpr;


ARCHITECTURE SYN OF pdi_dpr IS

	SIGNAL sub_wire0	: STD_LOGIC_VECTOR (31 DOWNTO 0);
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
        init_file : STRING;
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
			byteena_a	: IN STD_LOGIC_VECTOR (3 DOWNTO 0);
			byteena_b	: IN STD_LOGIC_VECTOR (3 DOWNTO 0);
			address_a	: IN STD_LOGIC_VECTOR (LOG2_NUM_WORDS-1 DOWNTO 0);
			address_b	: IN STD_LOGIC_VECTOR (LOG2_NUM_WORDS-1 DOWNTO 0);
			q_a	: OUT STD_LOGIC_VECTOR (31 DOWNTO 0);
			q_b	: OUT STD_LOGIC_VECTOR (31 DOWNTO 0);
			data_a	: IN STD_LOGIC_VECTOR (31 DOWNTO 0);
			data_b	: IN STD_LOGIC_VECTOR (31 DOWNTO 0)
	);
	END COMPONENT;

BEGIN
	q_a    <= sub_wire0(31 DOWNTO 0);
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
        init_file => "mif/pdi_dpr.mif",
		intended_device_family => "Cyclone IV",
		lpm_type => "altsyncram",
		numwords_a => NUM_WORDS,
		numwords_b => NUM_WORDS,
		operation_mode => "BIDIR_DUAL_PORT",
		outdata_aclr_a => "NONE",
		outdata_aclr_b => "NONE",
		outdata_reg_a => "CLOCK0",
		outdata_reg_b => "CLOCK1",
		power_up_uninitialized => "FALSE",
		read_during_write_mode_port_a => "NEW_DATA_WITH_NBE_READ",
		read_during_write_mode_port_b => "NEW_DATA_WITH_NBE_READ",
		widthad_a => LOG2_NUM_WORDS,
		widthad_b => LOG2_NUM_WORDS,
		width_a => 32,
		width_b => 32,
		width_byteena_a => 4,
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
		data_a => data_a,
		data_b => data_b,
		q_a => sub_wire0,
		q_b => sub_wire1
	);



END SYN;
