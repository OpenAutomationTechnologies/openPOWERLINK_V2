library ieee;
use ieee.STD_LOGIC_UNSIGNED.all;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;

	-- Add your library and packages declaration here ...

entity portio_tb is
	-- Generic declarations of the tested unit
		generic(
		pioValLen_g : INTEGER := 50 );
end portio_tb;

architecture TB_ARCHITECTURE of portio_tb is
	-- Component declaration of the tested unit
	component portio
		generic(
		pioValLen_g : INTEGER := 50 );
	port(
		s0_address : in STD_LOGIC;
		s0_read : in STD_LOGIC;
		s0_readdata : out STD_LOGIC_VECTOR(31 downto 0);
		s0_write : in STD_LOGIC;
		s0_writedata : in STD_LOGIC_VECTOR(31 downto 0);
		s0_byteenable : in STD_LOGIC_VECTOR(3 downto 0);
		clk : in STD_LOGIC;
		reset : in STD_LOGIC;
		x_pconfig : in STD_LOGIC_VECTOR(3 downto 0);
		x_portInLatch : in STD_LOGIC_VECTOR(3 downto 0);
		x_portOutValid : out STD_LOGIC_VECTOR(3 downto 0);
		x_portio : inout STD_LOGIC_VECTOR(31 downto 0);
		x_operational : out STD_LOGIC );
	end component;

	-- Stimulus signals - signals mapped to the input and inout ports of tested entity
	signal s0_address : STD_LOGIC;
	signal s0_read : STD_LOGIC;
	signal s0_write : STD_LOGIC;
	signal s0_writedata : STD_LOGIC_VECTOR(31 downto 0);
	signal s0_byteenable : STD_LOGIC_VECTOR(3 downto 0);
	signal clk : STD_LOGIC;
	signal reset : STD_LOGIC;
	signal x_pconfig : STD_LOGIC_VECTOR(3 downto 0);
	signal x_portInLatch : STD_LOGIC_VECTOR(3 downto 0);
	signal x_portio : STD_LOGIC_VECTOR(31 downto 0);
	-- Observed signals - signals mapped to the output ports of tested entity
	signal s0_readdata : STD_LOGIC_VECTOR(31 downto 0);
	signal x_portOutValid : STD_LOGIC_VECTOR(3 downto 0);
	signal x_operational : STD_LOGIC;

	-- Add your code here ...

begin

	-- Unit Under Test port map
	UUT : portio
		generic map (
			pioValLen_g => pioValLen_g
		)

		port map (
			s0_address => s0_address,
			s0_read => s0_read,
			s0_readdata => s0_readdata,
			s0_write => s0_write,
			s0_writedata => s0_writedata,
			s0_byteenable => s0_byteenable,
			clk => clk,
			reset => reset,
			x_pconfig => x_pconfig,
			x_portInLatch => x_portInLatch,
			x_portOutValid => x_portOutValid,
			x_portio => x_portio,
			x_operational => x_operational
		);

	-- Add your stimulus here ...
	
s0_address <= '0';
s0_read <= '0';
s0_write <= '0';
s0_writedata <= (others => '0');
s0_byteenable <= (others => '0');
x_pconfig <= (others => '1'); --all inputs

process
begin
	clk <= '0';
	wait for 10 ns;
	clk <= not clk;
	wait for 10 ns;
end process;

process
begin
	reset <= '1';
	wait for 100 ns;
	reset <= not reset;
	wait;
end process;

--latch does signals once...

process
begin
	x_portInLatch <= (others => '0');
	x_portio <= x"00000000";
	wait for 200 ns;
	x_portInLatch <= (others => '1');
	x_portio <= x"11223344";
	wait for 20 ns;
	x_portInLatch <= (others => '0');
	x_portio <= x"55667788";
	wait for 100 ns;
	x_portInLatch <= (others => '1');
	wait for 100 ns;
	x_portio <= x"99AABBCC";
	wait for 200 ns;
	x_portInLatch <= (others => '0');
	wait;
end process;

end TB_ARCHITECTURE;

configuration TESTBENCH_FOR_portio of portio_tb is
	for TB_ARCHITECTURE
		for UUT : portio
			use entity work.portio(rtl);
		end for;
	end for;
end TESTBENCH_FOR_portio;

