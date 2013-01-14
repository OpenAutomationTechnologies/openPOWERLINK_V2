library ieee;
use ieee.STD_LOGIC_UNSIGNED.all;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;

	-- Add your library and packages declaration here ...

entity rmii2mii_tb is
end rmii2mii_tb;

architecture TB_ARCHITECTURE of rmii2mii_tb is
	-- Component declaration of the tested unit
	component rmii2mii
	port(
		clk50 : in STD_LOGIC;
		rst : in STD_LOGIC;
		rTxEn : in STD_LOGIC;
		rTxDat : in STD_LOGIC_VECTOR(1 downto 0);
		rRxDv : out STD_LOGIC;
		rRxDat : out STD_LOGIC_VECTOR(1 downto 0);
		mTxEn : out STD_LOGIC;
		mTxDat : out STD_LOGIC_VECTOR(3 downto 0);
		mTxClk : in STD_LOGIC;
		mRxDv : in STD_LOGIC;
		mRxEr : in std_logic;
		mRxDat : in STD_LOGIC_VECTOR(3 downto 0);
		mRxClk : in STD_LOGIC );
	end component;

	-- Stimulus signals - signals mapped to the input and inout ports of tested entity
	signal clk50 : STD_LOGIC;
	signal rst : STD_LOGIC;
	signal rTxEn : STD_LOGIC;
	signal rTxDat : STD_LOGIC_VECTOR(1 downto 0);
	signal mTxClk : STD_LOGIC;
	signal mRxDv : STD_LOGIC;
	signal mRxDat : STD_LOGIC_VECTOR(3 downto 0);
	signal mRxClk : STD_LOGIC;
	-- Observed signals - signals mapped to the output ports of tested entity
	signal rRxDv : STD_LOGIC;
	signal rRxDat : STD_LOGIC_VECTOR(1 downto 0);
	signal mTxEn : STD_LOGIC;
	signal mTxDat : STD_LOGIC_VECTOR(3 downto 0);

	-- Add your code here ...
	type miiFrame_t is array(0 to 64*2-1) of std_logic_vector(3 downto 0);
	signal mRxFrame : miiFrame_t;
	
	--TIME_TEST : MAX, TYP, MIN
	constant TIME_TEST_RX : string := "TYP";
	constant TIME_TEST_TX : string := "2M5";
	constant TIME_MAX_PULSE : time := 24 ns;
	constant TIME_MIN_PULSE : time := 16 ns;
	constant TIME_TYP_PULSE : time := 20 ns;
	constant TIME_2M5_PULSE : time := 200 ns;

begin

	-- Unit Under Test port map
	UUT : rmii2mii
		port map (
			clk50 => clk50,
			rst => rst,
			rTxEn => rTxEn,
			rTxDat => rTxDat,
			rRxDv => rRxDv,
			rRxDat => rRxDat,
			mTxEn => mTxEn,
			mTxDat => mTxDat,
			mTxClk => mTxClk,
			mRxDv => mRxDv,
			mRxEr => '0',
			mRxDat => mRxDat,
			mRxClk => mRxClk
		);
	
	
	gen50meg : process
	begin
		clk50 <= '0';
		wait for 10ns;
		clk50 <= '1';
		wait for 10ns;
	end process;
	
	genRxClk : process
	begin
		mRxClk <= '0';
		
		loop
			case TIME_TEST_RX is
				when "MIN" => wait for TIME_MIN_PULSE;
				when "TYP" => wait for TIME_TYP_PULSE;
				when "MAX" => wait for TIME_MAX_PULSE;
				when "2M5" => wait for TIME_2M5_PULSE;
				when others =>
			end case;
			
			mRxClk <= not mRxClk;
		end loop;
	end process;
	
	genTxClk : process
	begin
		mTxClk <= '0';
		
		loop
			case TIME_TEST_TX is
				when "MIN" => wait for TIME_MIN_PULSE;
				when "TYP" => wait for TIME_TYP_PULSE;
				when "MAX" => wait for TIME_MAX_PULSE;
				when "2M5" => wait for TIME_2M5_PULSE;
				when others =>
			end case;
			
			mTxClk <= not mTxClk;
		end loop;
	end process;
	
	genRst : process
	begin
		rst <= '1';
		wait for 1us;
		rst <= '0';
		--wait for 6us;
		wait;
	end process;
	
	rxStim : process(mRxClk, rst)
	variable i : integer;
	begin
		if rst = '1' then
			i := 0;
			mRxDv <= '0';
			mRxDat <= (others => '0');
			for j in mRxFrame'range loop
				case j is
					when 0 to 15 =>
						mRxFrame(j) <= x"5";
					when 16 =>
						mRxFrame(j) <= x"D";
					when 17 to 28 => --mac dst
						mRxFrame(j) <= x"F";
					when 29 to 40 => --mac src
						mRxFrame(j) <= x"0";
					when 41 to 42 => --type field
						mRxFrame(j) <= x"A";
					when 43 to 64*2-1 =>
						mRxFrame(j) <= conv_std_logic_vector(j, mRxFrame(j)'length);
					when others =>
				end case;
			end loop;
		elsif mRxClk = '1' and mRxClk'event then
			mRxDv <= '0';
			mRxDat <= (others => '0');
			
			if (i > 0 +5) and (i < 64*2-1 +5) then
				mRxDv <= '1';
				mRxDat <= mRxFrame(i-5);
			end if;
			
			i := i + 1;
			
			if i > 150 then
				i := 0;
			end if;
		end if;
	end process;
	
	rTxEn <= rRxDv;
	rTxDat <= rRxDat;
	
	txStim : process(clk50, rst)
	variable i : integer;
	begin
		if rst = '1' then
			
		elsif clk50 = '1' and clk50'event then
--	rTxEn : STD_LOGIC;
--	rTxDat : STD_LOGIC_VECTOR(1 downto 0);
		end if;
	end process;

end TB_ARCHITECTURE;

configuration TESTBENCH_FOR_rmii2mii of rmii2mii_tb is
	for TB_ARCHITECTURE
		for UUT : rmii2mii
			use entity work.rmii2mii(rtl);
		end for;
	end for;
end TESTBENCH_FOR_rmii2mii;

