LIBRARY ieee;
USE ieee.std_logic_1164.all;

ENTITY sevsegdec IS
	PORT
	(
		d			: IN STD_LOGIC_VECTOR (3 DOWNTO 0);
		seg_n		: OUT STD_LOGIC_VECTOR (6 DOWNTO 0);
		seg		: OUT STD_LOGIC_VECTOR (6 DOWNTO 0)
	);
END sevsegdec;

ARCHITECTURE rtl of sevsegdec is
	SIGNAL seg_s : STD_LOGIC_VECTOR(6 DOWNTO 0);
begin			
	seg_n <= seg_s;
	seg <= not seg_s;
	
	with d select
	seg_s <=	"1000000" when "0000", -- 0
				"1111001" when "0001", -- 1
				"0100100" when "0010", -- 2
				"0110000" when "0011", -- 3
				"0011001" when "0100", -- 4
				"0010010" when "0101", -- 5
				"0000010" when "0110", -- 6
				"1111000" when "0111", -- 7
				"0000000" when "1000", -- 8
				"0010000" when "1001", -- 9
				"0001000" when "1010", -- A
				"0000011" when "1011", -- b				
				"1000110" when "1100", -- C
				"0100001" when "1101", -- d
				"0000110" when "1110", -- E
				"0001110" when "1111"; -- F
END rtl;
