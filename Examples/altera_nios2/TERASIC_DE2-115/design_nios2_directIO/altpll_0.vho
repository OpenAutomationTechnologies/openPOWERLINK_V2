--IP Functional Simulation Model
--VERSION_BEGIN 10.1SP1 cbx_mgl 2011:01:19:21:15:40:SJ cbx_simgen 2011:01:19:21:13:40:SJ  VERSION_END


-- Copyright (C) 1991-2011 Altera Corporation
-- Your use of Altera Corporation's design tools, logic functions 
-- and other software and tools, and its AMPP partner logic 
-- functions, and any output files from any of the foregoing 
-- (including device programming or simulation files), and any 
-- associated documentation or information are expressly subject 
-- to the terms and conditions of the Altera Program License 
-- Subscription Agreement, Altera MegaCore Function License 
-- Agreement, or other applicable license agreement, including, 
-- without limitation, that your use is for the sole purpose of 
-- programming logic devices manufactured by Altera and sold by 
-- Altera or its authorized distributors.  Please refer to the 
-- applicable agreement for further details.

-- You may only use these simulation model output files for simulation
-- purposes and expressly not for synthesis or any other purposes (in which
-- event Altera disclaims all warranties of any kind).


--synopsys translate_off

 LIBRARY cycloneive;
 USE cycloneive.cycloneive_components.all;

--synthesis_resources = cycloneive_pll 1 lut 5 
 LIBRARY ieee;
 USE ieee.std_logic_1164.all;

 ENTITY  altpll_0 IS 
	 PORT 
	 ( 
		 address	:	IN  STD_LOGIC_VECTOR (1 DOWNTO 0);
		 c0	:	OUT  STD_LOGIC;
		 c1	:	OUT  STD_LOGIC;
		 c2	:	OUT  STD_LOGIC;
		 c3	:	OUT  STD_LOGIC;
		 c4	:	OUT  STD_LOGIC;
		 clk	:	IN  STD_LOGIC;
		 locked	:	OUT  STD_LOGIC;
		 phasedone	:	OUT  STD_LOGIC;
		 read	:	IN  STD_LOGIC;
		 readdata	:	OUT  STD_LOGIC_VECTOR (31 DOWNTO 0);
		 reset	:	IN  STD_LOGIC;
		 write	:	IN  STD_LOGIC;
		 writedata	:	IN  STD_LOGIC_VECTOR (31 DOWNTO 0)
	 ); 
 END altpll_0;

 ARCHITECTURE RTL OF altpll_0 IS

	 ATTRIBUTE synthesis_clearbox : natural;
	 ATTRIBUTE synthesis_clearbox OF RTL : ARCHITECTURE IS 1;
	 SIGNAL  wire_altpll_0_altpll_0_altpll_dpt2_sd1_cycloneive_pll_pll7_127_clk	:	STD_LOGIC_VECTOR (4 DOWNTO 0);
	 SIGNAL  wire_altpll_0_altpll_0_altpll_dpt2_sd1_cycloneive_pll_pll7_127_fbout	:	STD_LOGIC;
	 SIGNAL  wire_altpll_0_altpll_0_altpll_dpt2_sd1_cycloneive_pll_pll7_127_inclk	:	STD_LOGIC_VECTOR (1 DOWNTO 0);
	 SIGNAL  wire_altpll_0_altpll_0_altpll_dpt2_sd1_cycloneive_pll_pll7_127_locked	:	STD_LOGIC;
	 SIGNAL	altpll_0_pfdena_reg_23q	:	STD_LOGIC := '0';
	 SIGNAL	altpll_0_altpll_0_stdsync_sv6_stdsync2_altpll_0_dffpipe_l2c_dffpipe3_dffe4a_0_109q	:	STD_LOGIC := '0';
	 SIGNAL	altpll_0_altpll_0_stdsync_sv6_stdsync2_altpll_0_dffpipe_l2c_dffpipe3_dffe5a_0_112q	:	STD_LOGIC := '0';
	 SIGNAL	altpll_0_altpll_0_stdsync_sv6_stdsync2_altpll_0_dffpipe_l2c_dffpipe3_dffe6a_0_113q	:	STD_LOGIC := '0';
	 SIGNAL	altpll_0_prev_reset_1q	:	STD_LOGIC := '0';
	 SIGNAL  wire_nO_w_lg_w_lg_w_lg_altpll_0_prev_reset_1q17w19w20w	:	STD_LOGIC_VECTOR (0 DOWNTO 0);
	 SIGNAL  wire_nO_w_lg_altpll_0_prev_reset_1q17w	:	STD_LOGIC_VECTOR (0 DOWNTO 0);
	 SIGNAL  wire_nO_w_lg_w_lg_altpll_0_prev_reset_1q17w19w	:	STD_LOGIC_VECTOR (0 DOWNTO 0);
	 SIGNAL  wire_w_lg_w_lg_w23w24w25w	:	STD_LOGIC_VECTOR (0 DOWNTO 0);
	 SIGNAL  wire_w23w	:	STD_LOGIC_VECTOR (0 DOWNTO 0);
	 SIGNAL  wire_w_lg_s_wire_altpll_0_w_select_status_18_dataout18w	:	STD_LOGIC_VECTOR (0 DOWNTO 0);
	 SIGNAL  wire_w_lg_w_address_range5w8w	:	STD_LOGIC_VECTOR (0 DOWNTO 0);
	 SIGNAL  wire_w_lg_reset3w	:	STD_LOGIC_VECTOR (0 DOWNTO 0);
	 SIGNAL  wire_w_lg_w_address_range6w7w	:	STD_LOGIC_VECTOR (0 DOWNTO 0);
	 SIGNAL  wire_w_lg_w23w24w	:	STD_LOGIC_VECTOR (0 DOWNTO 0);
	 SIGNAL  s_wire_altpll_0_w_select_control_17_dataout :	STD_LOGIC;
	 SIGNAL  s_wire_altpll_0_w_select_status_18_dataout :	STD_LOGIC;
	 SIGNAL  s_wire_altpll_0_wire_pfdena_reg_ena_14_dataout :	STD_LOGIC;
	 SIGNAL  s_wire_vcc :	STD_LOGIC;
	 SIGNAL  wire_w_address_range5w	:	STD_LOGIC_VECTOR (0 DOWNTO 0);
	 SIGNAL  wire_w_address_range6w	:	STD_LOGIC_VECTOR (0 DOWNTO 0);
 BEGIN

	wire_w_lg_w_lg_w23w24w25w(0) <= wire_w_lg_w23w24w(0) AND read;
	wire_w23w(0) <= s_wire_altpll_0_w_select_control_17_dataout AND altpll_0_pfdena_reg_23q;
	wire_w_lg_s_wire_altpll_0_w_select_status_18_dataout18w(0) <= s_wire_altpll_0_w_select_status_18_dataout AND altpll_0_altpll_0_stdsync_sv6_stdsync2_altpll_0_dffpipe_l2c_dffpipe3_dffe6a_0_113q;
	wire_w_lg_w_address_range5w8w(0) <= wire_w_address_range5w(0) AND wire_w_lg_w_address_range6w7w(0);
	wire_w_lg_reset3w(0) <= NOT reset;
	wire_w_lg_w_address_range6w7w(0) <= NOT wire_w_address_range6w(0);
	wire_w_lg_w23w24w(0) <= wire_w23w(0) OR s_wire_altpll_0_w_select_status_18_dataout;
	c0 <= wire_altpll_0_altpll_0_altpll_dpt2_sd1_cycloneive_pll_pll7_127_clk(0);
	c1 <= wire_altpll_0_altpll_0_altpll_dpt2_sd1_cycloneive_pll_pll7_127_clk(1);
	c2 <= wire_altpll_0_altpll_0_altpll_dpt2_sd1_cycloneive_pll_pll7_127_clk(2);
	c3 <= wire_altpll_0_altpll_0_altpll_dpt2_sd1_cycloneive_pll_pll7_127_clk(3);
	c4 <= wire_altpll_0_altpll_0_altpll_dpt2_sd1_cycloneive_pll_pll7_127_clk(4);
	locked <= wire_altpll_0_altpll_0_altpll_dpt2_sd1_cycloneive_pll_pll7_127_locked;
	phasedone <= '0';
	readdata <= ( "0" & "0" & "0" & "0" & "0" & "0" & "0" & "0" & "0" & "0" & "0" & "0" & "0" & "0" & "0" & "0" & "0" & "0" & "0" & "0" & "0" & "0" & "0" & "0" & "0" & "0" & "0" & "0" & "0" & "0" & wire_w_lg_w_lg_w23w24w25w & wire_nO_w_lg_w_lg_w_lg_altpll_0_prev_reset_1q17w19w20w);
	s_wire_altpll_0_w_select_control_17_dataout <= wire_w_lg_w_address_range5w8w(0);
	s_wire_altpll_0_w_select_status_18_dataout <= ((NOT address(0)) AND wire_w_lg_w_address_range6w7w(0));
	s_wire_altpll_0_wire_pfdena_reg_ena_14_dataout <= (s_wire_altpll_0_w_select_control_17_dataout AND write);
	s_wire_vcc <= '1';
	wire_w_address_range5w(0) <= address(0);
	wire_w_address_range6w(0) <= address(1);
	wire_altpll_0_altpll_0_altpll_dpt2_sd1_cycloneive_pll_pll7_127_inclk <= ( "0" & clk);
	altpll_0_altpll_0_altpll_dpt2_sd1_cycloneive_pll_pll7_127 :  cycloneive_pll
	  GENERIC MAP (
		BANDWIDTH_TYPE => "auto",
		CLK0_DIVIDE_BY => 1,
		CLK0_DUTY_CYCLE => 50,
		CLK0_MULTIPLY_BY => 1,
		CLK0_PHASE_SHIFT => "0",
		CLK1_DIVIDE_BY => 1,
		CLK1_DUTY_CYCLE => 50,
		CLK1_MULTIPLY_BY => 2,
		CLK1_PHASE_SHIFT => "0",
		CLK2_DIVIDE_BY => 2,
		CLK2_DUTY_CYCLE => 50,
		CLK2_MULTIPLY_BY => 1,
		CLK2_PHASE_SHIFT => "0",
		CLK3_DIVIDE_BY => 1,
		CLK3_DUTY_CYCLE => 50,
		CLK3_MULTIPLY_BY => 1,
		CLK3_PHASE_SHIFT => "0",
		CLK4_DIVIDE_BY => 1,
		CLK4_DUTY_CYCLE => 50,
		CLK4_MULTIPLY_BY => 1,
		CLK4_PHASE_SHIFT => "-5000",
		COMPENSATE_CLOCK => "clk0",
		INCLK0_INPUT_FREQUENCY => 20000,
		OPERATION_MODE => "normal",
		PLL_TYPE => "auto"
	  )
	  PORT MAP ( 
		clk => wire_altpll_0_altpll_0_altpll_dpt2_sd1_cycloneive_pll_pll7_127_clk,
		fbin => wire_altpll_0_altpll_0_altpll_dpt2_sd1_cycloneive_pll_pll7_127_fbout,
		fbout => wire_altpll_0_altpll_0_altpll_dpt2_sd1_cycloneive_pll_pll7_127_fbout,
		inclk => wire_altpll_0_altpll_0_altpll_dpt2_sd1_cycloneive_pll_pll7_127_inclk,
		locked => wire_altpll_0_altpll_0_altpll_dpt2_sd1_cycloneive_pll_pll7_127_locked
	  );
	PROCESS (clk, reset)
	BEGIN
		IF (reset = '1') THEN
				altpll_0_pfdena_reg_23q <= '1';
		ELSIF (clk = '1' AND clk'event) THEN
			IF (s_wire_altpll_0_wire_pfdena_reg_ena_14_dataout = '1') THEN
				altpll_0_pfdena_reg_23q <= writedata(1);
			END IF;
		END IF;
		if (now = 0 ns) then
			altpll_0_pfdena_reg_23q <= '1' after 1 ps;
		end if;
	END PROCESS;
	PROCESS (clk, reset)
	BEGIN
		IF (reset = '1') THEN
				altpll_0_altpll_0_stdsync_sv6_stdsync2_altpll_0_dffpipe_l2c_dffpipe3_dffe4a_0_109q <= '0';
				altpll_0_altpll_0_stdsync_sv6_stdsync2_altpll_0_dffpipe_l2c_dffpipe3_dffe5a_0_112q <= '0';
				altpll_0_altpll_0_stdsync_sv6_stdsync2_altpll_0_dffpipe_l2c_dffpipe3_dffe6a_0_113q <= '0';
				altpll_0_prev_reset_1q <= '0';
		ELSIF (clk = '1' AND clk'event) THEN
				altpll_0_altpll_0_stdsync_sv6_stdsync2_altpll_0_dffpipe_l2c_dffpipe3_dffe4a_0_109q <= wire_altpll_0_altpll_0_altpll_dpt2_sd1_cycloneive_pll_pll7_127_locked;
				altpll_0_altpll_0_stdsync_sv6_stdsync2_altpll_0_dffpipe_l2c_dffpipe3_dffe5a_0_112q <= altpll_0_altpll_0_stdsync_sv6_stdsync2_altpll_0_dffpipe_l2c_dffpipe3_dffe4a_0_109q;
				altpll_0_altpll_0_stdsync_sv6_stdsync2_altpll_0_dffpipe_l2c_dffpipe3_dffe6a_0_113q <= altpll_0_altpll_0_stdsync_sv6_stdsync2_altpll_0_dffpipe_l2c_dffpipe3_dffe5a_0_112q;
				altpll_0_prev_reset_1q <= (s_wire_altpll_0_wire_pfdena_reg_ena_14_dataout AND writedata(0));
		END IF;
	END PROCESS;
	wire_nO_w_lg_w_lg_w_lg_altpll_0_prev_reset_1q17w19w20w(0) <= wire_nO_w_lg_w_lg_altpll_0_prev_reset_1q17w19w(0) AND read;
	wire_nO_w_lg_altpll_0_prev_reset_1q17w(0) <= altpll_0_prev_reset_1q AND s_wire_altpll_0_w_select_control_17_dataout;
	wire_nO_w_lg_w_lg_altpll_0_prev_reset_1q17w19w(0) <= wire_nO_w_lg_altpll_0_prev_reset_1q17w(0) OR wire_w_lg_s_wire_altpll_0_w_select_status_18_dataout18w(0);

 END RTL; --altpll_0
--synopsys translate_on
--VALID FILE
