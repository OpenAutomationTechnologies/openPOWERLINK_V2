library POWERLINK;
use POWERLINK.memMap.all;
library ieee;
use ieee.MATH_REAL.all;
use ieee.STD_LOGIC_UNSIGNED.all;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;

	-- Add your library and packages declaration here ...

entity pdi_tb is
	-- Generic declarations of the tested unit
		generic(
		iFpgaRev_g : INTEGER := 35;
		iRpdos_g : INTEGER := 1;
		iTpdos_g : INTEGER := 1;
		genABuf1_g : boolean := true; --if false iABuf1_g must be set to 0!
		genABuf2_g : boolean := true; --if false iABuf2_g must be set to 0!
		genLedGadget_g : boolean := false;
		iTpdoBufSize_g : INTEGER := 128;
		iRpdo0BufSize_g : INTEGER := 80;
		iRpdo1BufSize_g : INTEGER := 0;
		iRpdo2BufSize_g : INTEGER := 0;
		iABuf1_g : INTEGER := 50;
		iABuf2_g : INTEGER := 10 );
end pdi_tb;

architecture TB_ARCHITECTURE of pdi_tb is
	-- Component declaration of the tested unit
	component pdi
		generic(
		iRpdos_g : INTEGER := 3;
		iTpdos_g : INTEGER := 1;
		genABuf1_g : boolean := true; --if false iABuf1_g must be set to 0!
		genABuf2_g : boolean := true; --if false iABuf2_g must be set to 0!
		genLedGadget_g : boolean := false;
		iTpdoBufSize_g : INTEGER := 100;
		iRpdo0BufSize_g : INTEGER := 116;
		iRpdo1BufSize_g : INTEGER := 116;
		iRpdo2BufSize_g : INTEGER := 116;
		iABuf1_g : INTEGER := 512;
		iABuf2_g : INTEGER := 512 );
	port(
		pcp_reset : in STD_LOGIC;
		pcp_clk : in STD_LOGIC;
		ap_reset : in STD_LOGIC;
		ap_clk : in STD_LOGIC;
		pcp_chipselect : in STD_LOGIC;
		pcp_read : in STD_LOGIC;
		pcp_write : in STD_LOGIC;
		pcp_byteenable : in STD_LOGIC_VECTOR(3 downto 0);
		pcp_address : in STD_LOGIC_VECTOR(12 downto 0);
		pcp_writedata : in STD_LOGIC_VECTOR(31 downto 0);
		pcp_readdata : out STD_LOGIC_VECTOR(31 downto 0);
		pcp_irq : in STD_LOGIC;
		ap_chipselect : in STD_LOGIC;
		ap_read : in STD_LOGIC;
		ap_write : in STD_LOGIC;
		ap_byteenable : in STD_LOGIC_VECTOR(3 downto 0);
		ap_address : in STD_LOGIC_VECTOR(12 downto 0);
		ap_writedata : in STD_LOGIC_VECTOR(31 downto 0);
		ap_readdata : out STD_LOGIC_VECTOR(31 downto 0);
		ap_irq : out STD_LOGIC;
		ap_asyncIrq : out STD_LOGIC;
		ledsOut : out STD_LOGIC_VECTOR(15 downto 0);
		phyLink : in STD_LOGIC_VECTOR(1 downto 0);
		phyAct : in STD_LOGIC_VECTOR(1 downto 0);
					rpdo_change_tog				: in	std_logic_vector(2 downto 0);
			tpdo_change_tog				: in	std_logic);
	end component;

	-- Stimulus signals - signals mapped to the input and inout ports of tested entity
	signal pcp_reset : STD_LOGIC;
	signal pcp_clk : STD_LOGIC;
	signal ap_reset : STD_LOGIC;
	signal ap_clk : STD_LOGIC;
	signal pcp_chipselect : STD_LOGIC;
	signal pcp_read : STD_LOGIC;
	signal pcp_write : STD_LOGIC;
	signal pcp_byteenable : STD_LOGIC_VECTOR(3 downto 0);
	signal pcp_address : STD_LOGIC_VECTOR(12 downto 0);
	signal pcp_writedata : STD_LOGIC_VECTOR(31 downto 0);
	signal pcp_irq : STD_LOGIC;
	signal ap_chipselect : STD_LOGIC;
	signal ap_read : STD_LOGIC;
	signal ap_write : STD_LOGIC;
	signal ap_byteenable : STD_LOGIC_VECTOR(3 downto 0);
	signal ap_address : STD_LOGIC_VECTOR(12 downto 0);
	signal ap_writedata : STD_LOGIC_VECTOR(31 downto 0);
	signal phyLink : STD_LOGIC_VECTOR(1 downto 0);
	signal phyAct : STD_LOGIC_VECTOR(1 downto 0);
		signal 		rpdo_change_tog				: 	std_logic_vector(2 downto 0) := (others => '0');
signal			tpdo_change_tog				: 	std_logic:= '0';
	-- Observed signals - signals mapped to the output ports of tested entity
	signal pcp_readdata : STD_LOGIC_VECTOR(31 downto 0);
	signal ap_readdata : STD_LOGIC_VECTOR(31 downto 0);
	signal ap_irq : STD_LOGIC;
	signal ap_asyncIrq : STD_LOGIC;
	signal ledsOut : STD_LOGIC_VECTOR(15 downto 0);

	-- Add your code here ...

begin

	-- Unit Under Test port map
	UUT : pdi
		generic map (
			iRpdos_g => iRpdos_g,
			iTpdos_g => iTpdos_g,
			iTpdoBufSize_g => iTpdoBufSize_g,
			iRpdo0BufSize_g => iRpdo0BufSize_g,
			iRpdo1BufSize_g => iRpdo1BufSize_g,
			iRpdo2BufSize_g => iRpdo2BufSize_g,
			iABuf1_g => iABuf1_g,
			iABuf2_g => iABuf2_g,
			genABuf1_g => genABuf1_g,
			genABuf2_g => genABuf2_g,
			genLedGadget_g => genLedGadget_g
		)

		port map (
			pcp_reset => pcp_reset,
			pcp_clk => pcp_clk,
			ap_reset => ap_reset,
			ap_clk => ap_clk,
			pcp_chipselect => pcp_chipselect,
			pcp_read => pcp_read,
			pcp_write => pcp_write,
			pcp_byteenable => pcp_byteenable,
			pcp_address => pcp_address,
			pcp_writedata => pcp_writedata,
			pcp_readdata => pcp_readdata,
			pcp_irq => pcp_irq,
			ap_chipselect => ap_chipselect,
			ap_read => ap_read,
			ap_write => ap_write,
			ap_byteenable => ap_byteenable,
			ap_address => ap_address,
			ap_writedata => ap_writedata,
			ap_readdata => ap_readdata,
			ap_irq => ap_irq,
			ap_asyncIrq => ap_asyncIrq,
			ledsOut => ledsOut,
			phyLink => phyLink,
			phyAct => phyAct,
			rpdo_change_tog => rpdo_change_tog,
			tpdo_change_tog => tpdo_change_tog
		);

	-- Add your stimulus here ...
--signal pcp_reset : STD_LOGIC;
--signal ap_reset : STD_LOGIC;
process
begin
	pcp_reset <= '1';
	ap_reset <= '1';
	wait for 100 ns;
	pcp_reset <= '0';
	ap_reset <= '0';
	wait;
end process;

--signal pcp_clk : STD_LOGIC;
--signal ap_clk : STD_LOGIC;
process
begin
	pcp_clk <= '0';
	ap_clk <= '0';
	wait for 10 ns;
	pcp_clk <= '1';
	ap_clk <= '1';
	wait for 10 ns;
end process;

--signal pcp_irq : STD_LOGIC;
pcp_irq <= '0';

--signal pcp_chipselect : STD_LOGIC;
--signal pcp_read : STD_LOGIC;
--signal pcp_write : STD_LOGIC;
--signal pcp_byteenable : STD_LOGIC_VECTOR(3 downto 0);
--signal pcp_address : STD_LOGIC_VECTOR(12 downto 0);
--signal pcp_writedata : STD_LOGIC_VECTOR(31 downto 0);
process(pcp_clk, pcp_reset)
variable i : integer;
begin
	if pcp_reset = '1' then
		pcp_chipselect <= '0';
		pcp_read <= '0';
		pcp_write <= '0';
		pcp_byteenable <= "0000";
		pcp_address <= conv_std_logic_vector(0, pcp_address'length);
		pcp_writedata <= conv_std_logic_vector(0, pcp_writedata'length);
		i := 0;
	elsif pcp_clk = '1' and pcp_clk'event then
		--default
		pcp_chipselect <= '0';
		pcp_read <= '0';
		pcp_write <= '0';
		pcp_byteenable <= "0000";
		pcp_address <= conv_std_logic_vector(0, pcp_address'length);
		pcp_writedata <= conv_std_logic_vector(0, pcp_writedata'length);
		
		case i is
			when 10 =>
				pcp_chipselect <= '1';
				pcp_read <= '0';
				pcp_write <= '1';
				pcp_byteenable <= "0011";
				pcp_address <= conv_std_logic_vector(16#4C#/4, pcp_address'length);
				pcp_writedata <= x"1234ABCD";
			when 12 =>
				pcp_chipselect <= '1';
				pcp_read <= '0';
				pcp_write <= '1';
				pcp_byteenable <= "1100";
				pcp_address <= conv_std_logic_vector(16#4C#/4, pcp_address'length);
				pcp_writedata <= x"1234ABCD";
			when 14 =>
				pcp_chipselect <= '1';
				pcp_read <= '0';
				pcp_write <= '1';
				pcp_byteenable <= "0011";
				pcp_address <= conv_std_logic_vector(16#50#/4, pcp_address'length);
				pcp_writedata <= x"1234ABCD";
			when 16 =>
				pcp_chipselect <= '1';
				pcp_read <= '0';
				pcp_write <= '1';
				pcp_byteenable <= "1100";
				pcp_address <= conv_std_logic_vector(16#50#/4, pcp_address'length);
				pcp_writedata <= x"1234ABCD";
			
			when 40 =>
				pcp_chipselect <= '1';
				pcp_read <= '0';
				pcp_write <= '1';
				pcp_byteenable <= "0011";
				pcp_address <= conv_std_logic_vector(16#4C#/4, pcp_address'length);
				pcp_writedata <= x"1234ABCD";
			when 42 =>
				pcp_chipselect <= '1';
				pcp_read <= '0';
				pcp_write <= '1';
				pcp_byteenable <= "1100";
				pcp_address <= conv_std_logic_vector(16#4C#/4, pcp_address'length);
				pcp_writedata <= x"1234ABCD";
			when 44 =>
				pcp_chipselect <= '1';
				pcp_read <= '0';
				pcp_write <= '1';
				pcp_byteenable <= "0011";
				pcp_address <= conv_std_logic_vector(16#50#/4, pcp_address'length);
				pcp_writedata <= x"1234ABCD";
			when 46 =>
				pcp_chipselect <= '1';
				pcp_read <= '0';
				pcp_write <= '1';
				pcp_byteenable <= "1100";
				pcp_address <= conv_std_logic_vector(16#50#/4, pcp_address'length);
				pcp_writedata <= x"1234ABCD";
			
			when 70 =>
				pcp_chipselect <= '1';
				pcp_read <= '0';
				pcp_write <= '1';
				pcp_byteenable <= "0011";
				pcp_address <= conv_std_logic_vector(16#4C#/4, pcp_address'length);
				pcp_writedata <= x"1234ABCD";
			when 72 =>
				pcp_chipselect <= '1';
				pcp_read <= '0';
				pcp_write <= '1';
				pcp_byteenable <= "1100";
				pcp_address <= conv_std_logic_vector(16#4C#/4, pcp_address'length);
				pcp_writedata <= x"1234ABCD";
			when 74 =>
				pcp_chipselect <= '1';
				pcp_read <= '0';
				pcp_write <= '1';
				pcp_byteenable <= "0011";
				pcp_address <= conv_std_logic_vector(16#50#/4, pcp_address'length);
				pcp_writedata <= x"1234ABCD";
			when 76 =>
				pcp_chipselect <= '1';
				pcp_read <= '0';
				pcp_write <= '1';
				pcp_byteenable <= "1100";
				pcp_address <= conv_std_logic_vector(16#50#/4, pcp_address'length);
				pcp_writedata <= x"1234ABCD";
			when 78 =>
				i := i - 1; --halt here
			when others =>
		end case;
		
		i := i + 1;
		
	end if;
end process;

--signal ap_chipselect : STD_LOGIC;
--signal ap_read : STD_LOGIC;
--signal ap_write : STD_LOGIC;
--signal ap_byteenable : STD_LOGIC_VECTOR(3 downto 0);
--signal ap_address : STD_LOGIC_VECTOR(12 downto 0);
--signal ap_writedata : STD_LOGIC_VECTOR(31 downto 0);
process(ap_clk, ap_reset)
variable i : integer;
begin
	if ap_reset = '1' then
		ap_chipselect <= '0';
		ap_read <= '0';
		ap_write <= '0';
		ap_byteenable <= "0000";
		ap_address <= conv_std_logic_vector(0, ap_address'length);
		ap_writedata <= conv_std_logic_vector(0, ap_writedata'length);
		i := 0;
	elsif ap_clk = '1' and ap_clk'event then
		--default
		ap_chipselect <= '0';
		ap_read <= '0';
		ap_write <= '0';
		ap_byteenable <= "0000";
		ap_address <= conv_std_logic_vector(0, ap_address'length);
		ap_writedata <= conv_std_logic_vector(0, ap_writedata'length);
		
		case (i-10) is
			when 10 =>
				ap_chipselect <= '1';
				ap_read <= '0';
				ap_write <= '1';
				ap_byteenable <= "0011";
				ap_address <= conv_std_logic_vector(16#4C#/4, ap_address'length);
				ap_writedata <= x"1234ABCD";
			when 12 =>
				ap_chipselect <= '1';
				ap_read <= '0';
				ap_write <= '1';
				ap_byteenable <= "1100";
				ap_address <= conv_std_logic_vector(16#4C#/4, ap_address'length);
				ap_writedata <= x"1234ABCD";
			when 14 =>
				ap_chipselect <= '1';
				ap_read <= '0';
				ap_write <= '1';
				ap_byteenable <= "0011";
				ap_address <= conv_std_logic_vector(16#50#/4, ap_address'length);
				ap_writedata <= x"1234ABCD";
			when 16 =>
				ap_chipselect <= '1';
				ap_read <= '0';
				ap_write <= '1';
				ap_byteenable <= "1100";
				ap_address <= conv_std_logic_vector(16#50#/4, ap_address'length);
				ap_writedata <= x"1234ABCD";
			
			when 40 =>
				ap_chipselect <= '1';
				ap_read <= '0';
				ap_write <= '1';
				ap_byteenable <= "0011";
				ap_address <= conv_std_logic_vector(16#4C#/4, ap_address'length);
				ap_writedata <= x"1234ABCD";
			when 42 =>
				ap_chipselect <= '1';
				ap_read <= '0';
				ap_write <= '1';
				ap_byteenable <= "1100";
				ap_address <= conv_std_logic_vector(16#4C#/4, ap_address'length);
				ap_writedata <= x"1234ABCD";
			when 44 =>
				ap_chipselect <= '1';
				ap_read <= '0';
				ap_write <= '1';
				ap_byteenable <= "0011";
				ap_address <= conv_std_logic_vector(16#50#/4, ap_address'length);
				ap_writedata <= x"1234ABCD";
			when 46 =>
				ap_chipselect <= '1';
				ap_read <= '0';
				ap_write <= '1';
				ap_byteenable <= "1100";
				ap_address <= conv_std_logic_vector(16#50#/4, ap_address'length);
				ap_writedata <= x"1234ABCD";
			
			when 70 =>
				ap_chipselect <= '1';
				ap_read <= '0';
				ap_write <= '1';
				ap_byteenable <= "0011";
				ap_address <= conv_std_logic_vector(16#4C#/4, ap_address'length);
				ap_writedata <= x"1234ABCD";
			when 72 =>
				ap_chipselect <= '1';
				ap_read <= '0';
				ap_write <= '1';
				ap_byteenable <= "1100";
				ap_address <= conv_std_logic_vector(16#4C#/4, ap_address'length);
				ap_writedata <= x"1234ABCD";
			when 74 =>
				ap_chipselect <= '1';
				ap_read <= '0';
				ap_write <= '1';
				ap_byteenable <= "0011";
				ap_address <= conv_std_logic_vector(16#50#/4, ap_address'length);
				ap_writedata <= x"1234ABCD";
			when 76 =>
				ap_chipselect <= '1';
				ap_read <= '0';
				ap_write <= '1';
				ap_byteenable <= "1100";
				ap_address <= conv_std_logic_vector(16#50#/4, ap_address'length);
				ap_writedata <= x"1234ABCD";
			when 78 =>
				i := i - 1; --halt here
			when others =>
		end case;
		
		i := i + 1;
		
	end if;
end process;

--signal phyLink : STD_LOGIC_VECTOR(1 downto 0);
phyLink <= (others => '0');
--signal phyAct : STD_LOGIC_VECTOR(1 downto 0);
phyAct <= (others => '0');

end TB_ARCHITECTURE;

configuration TESTBENCH_FOR_pdi of pdi_tb is
	for TB_ARCHITECTURE
		for UUT : pdi
			use entity work.pdi(rtl);
		end for;
	end for;
end TESTBENCH_FOR_pdi;

