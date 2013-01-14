library ieee;
use ieee.STD_LOGIC_UNSIGNED.all;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;

	-- Add your library and packages declaration here ...

entity pdievent_tb is
	-- Generic declarations of the tested unit
		generic(
		iSwEvent_g : INTEGER := 1;
		iHwEvent_g : INTEGER := 2 );
end pdievent_tb;

architecture TB_ARCHITECTURE of pdievent_tb is
	-- Component declaration of the tested unit
	component pdievent
		generic(
		iSwEvent_g : INTEGER := 1;
		iHwEvent_g : INTEGER := 2 );
	port(
		clkA : in STD_LOGIC;
		rstA : in STD_LOGIC;
		eventSetA : in STD_LOGIC_VECTOR(iSwEvent_g-1 downto 0);
		eventReadA : out STD_LOGIC_VECTOR(iSwEvent_g+iHwEvent_g-1 downto 0);
		clkB : in STD_LOGIC;
		rstB : in STD_LOGIC;
		eventAckB : in STD_LOGIC_VECTOR(iSwEvent_g+iHwEvent_g-1 downto 0);
		eventReadB : out STD_LOGIC_VECTOR(iSwEvent_g+iHwEvent_g-1 downto 0);
		hwEventSetPulseB : in STD_LOGIC_VECTOR(iHwEvent_g-1 downto 0) );
	end component;

	-- Stimulus signals - signals mapped to the input and inout ports of tested entity
	signal clkA : STD_LOGIC;
	signal rstA : STD_LOGIC;
	signal eventSetA : STD_LOGIC_VECTOR(iSwEvent_g-1 downto 0);
	signal clkB : STD_LOGIC;
	signal rstB : STD_LOGIC;
	signal eventAckB : STD_LOGIC_VECTOR(iSwEvent_g+iHwEvent_g-1 downto 0);
	signal hwEventSetPulseB : STD_LOGIC_VECTOR(iHwEvent_g-1 downto 0);
	-- Observed signals - signals mapped to the output ports of tested entity
	signal eventReadA : STD_LOGIC_VECTOR(iSwEvent_g+iHwEvent_g-1 downto 0);
	signal eventReadB : STD_LOGIC_VECTOR(iSwEvent_g+iHwEvent_g-1 downto 0);

	-- Add your code here ...

begin

	-- Unit Under Test port map
	UUT : pdievent
		generic map (
			iSwEvent_g => iSwEvent_g,
			iHwEvent_g => iHwEvent_g
		)

		port map (
			clkA => clkA,
			rstA => rstA,
			eventSetA => eventSetA,
			eventReadA => eventReadA,
			clkB => clkB,
			rstB => rstB,
			eventAckB => eventAckB,
			eventReadB => eventReadB,
			hwEventSetPulseB => hwEventSetPulseB
		);

	-- Add your stimulus here ...
---------------------
process
begin  
	clkA <= '0';
	wait for 5 ns;
	clkA <= not clkA;
	wait for 5 ns;
end process;

process
begin  
	rstA <= '1';
	wait for 100 ns;
	rstA <= not rstA;
	wait;
end process;

process(clkA, rstA)
	variable i : integer;	
begin  
	if rstA = '1' then
		eventSetA <= (others => '0');
		i := 0;
	elsif clkA = '1' and clkA'event then
		--default
		eventSetA <= (others => '0');
		
		case i is
			when 4 =>
				eventSetA(0) <= '1';
			when 60 =>
				eventSetA(0) <= '1';
			when others =>
		end case;
		
		i := i + 1;
	end if;
end process;

---------------------
process	
begin  
	clkB <= '0';
	wait for 10 ns;
	clkB <= not clkB;
	wait for 10 ns;
end process;

process
begin  
	rstB <= '1';
	wait for 100 ns;
	rstB <= not rstA;
	wait;
end process;

process(clkB, rstB)
	variable i, j : integer;
begin  
	if rstB = '1' then
		eventAckB <= (others => '0');
		hwEventSetPulseB <= (others => '0');
		i := 0; j := 0;
	elsif clkB = '1' and clkB'event then
		--default
		eventAckB <= (others => '0');
		hwEventSetPulseB <= (others => '0');
		
		if eventReadB(j) = '1' then
			eventAckB(j) <= '1';
		end if;
				
		case i is
			when 10 =>
				hwEventSetPulseB <= "10";
			when 20 =>
				hwEventSetPulseB <= "01";
			when others =>
		end case;
		
		i := i + 1;
		
		j := j + 1;
		if j = eventReadB'length then
			j := 0;
		end if;
		
	end if;
end process;

end TB_ARCHITECTURE;

configuration TESTBENCH_FOR_pdievent of pdievent_tb is
	for TB_ARCHITECTURE
		for UUT : pdievent
			use entity work.pdievent(rtl);
		end for;
	end for;
end TESTBENCH_FOR_pdievent;

