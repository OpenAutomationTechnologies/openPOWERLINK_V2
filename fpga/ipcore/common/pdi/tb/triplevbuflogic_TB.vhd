library ieee;
use ieee.STD_LOGIC_UNSIGNED.all;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;

	-- Add your library and packages declaration here ...

entity triplevbuflogic_tb is
	-- Generic declarations of the tested unit
		generic(
		iVirtualBufferBase_g : INTEGER := 0;
		iVirtualBufferSize_g : INTEGER := 1024;
		iOutAddrWidth_g : INTEGER := 13;
		iInAddrWidth_g : INTEGER := 11;
		bApIsProducer : BOOLEAN := false );
end triplevbuflogic_tb;

architecture TB_ARCHITECTURE of triplevbuflogic_tb is
	-- Component declaration of the tested unit
	component triplevbuflogic
		generic(
		iVirtualBufferBase_g : INTEGER := 0;
		iVirtualBufferSize_g : INTEGER := 1024;
		iOutAddrWidth_g : INTEGER := 13;
		iInAddrWidth_g : INTEGER := 11;
		bApIsProducer : BOOLEAN := false );
	port(
		pcpClk : in STD_LOGIC;
		pcpReset : in STD_LOGIC;
		pcpTrigger : in STD_LOGIC;
		--pcpInAddr : in STD_LOGIC_VECTOR(iInAddrWidth_g-1 downto 0);
		pcpOutAddrOff : out STD_LOGIC_VECTOR(iOutAddrWidth_g downto 0);
		pcpOutSelVBuf : out STD_LOGIC_VECTOR(2 downto 0);
		apClk : in STD_LOGIC;
		apReset : in STD_LOGIC;
		apTrigger : in STD_LOGIC;
		--apInAddr : in STD_LOGIC_VECTOR(iInAddrWidth_g-1 downto 0);
		apOutAddrOff : out STD_LOGIC_VECTOR(iOutAddrWidth_g downto 0);
		apOutSelVBuf : out STD_LOGIC_VECTOR(2 downto 0) );
	end component;

	-- Stimulus signals - signals mapped to the input and inout ports of tested entity
	signal pcpClk : STD_LOGIC;
	signal pcpReset : STD_LOGIC;
	signal pcpTrigger : STD_LOGIC;
	--signal pcpInAddr : STD_LOGIC_VECTOR(iInAddrWidth_g-1 downto 0);
	signal apClk : STD_LOGIC;
	signal apReset : STD_LOGIC;
	signal apTrigger : STD_LOGIC;
	--signal apInAddr : STD_LOGIC_VECTOR(iInAddrWidth_g-1 downto 0);
	-- Observed signals - signals mapped to the output ports of tested entity
	signal pcpOutAddrOff : STD_LOGIC_VECTOR(iOutAddrWidth_g downto 0);
	signal pcpOutSelVBuf : STD_LOGIC_VECTOR(2 downto 0);
	signal apOutAddrOff : STD_LOGIC_VECTOR(iOutAddrWidth_g downto 0);
	signal apOutSelVBuf : STD_LOGIC_VECTOR(2 downto 0);

	signal clk, rst : std_logic;

begin

	-- Unit Under Test port map
	UUT : triplevbuflogic
		generic map (
			iVirtualBufferBase_g => iVirtualBufferBase_g,
			iVirtualBufferSize_g => iVirtualBufferSize_g,
			iOutAddrWidth_g => iOutAddrWidth_g,
			iInAddrWidth_g => iInAddrWidth_g,
			bApIsProducer => bApIsProducer
		)

		port map (
			pcpClk => pcpClk,
			pcpReset => pcpReset,
			pcpTrigger => pcpTrigger,
			--pcpInAddr => pcpInAddr,
			pcpOutAddrOff => pcpOutAddrOff,
			pcpOutSelVBuf => pcpOutSelVBuf,
			apClk => apClk,
			apReset => apReset,
			apTrigger => apTrigger,
			--apInAddr => apInAddr,
			apOutAddrOff => apOutAddrOff,
			apOutSelVBuf => apOutSelVBuf
		);

	process
	begin
		clk <= '0';
		wait for 10ns;
		clk <= '1';
		wait for 10ns;
	end process;
	pcpClk <= clk;
	apClk <= clk;
	
	process
	begin
		rst <= '1';
		wait for 100ns;
		rst <= '0';
		wait;
	end process;
	pcpReset <= rst;
	apReset <= rst;
	
	process(rst, clk)
	variable i : integer;
	begin
		if rst = '1' then
			i := 0;
			pcpTrigger <= '0';
			--pcpInAddr <= (others => '0');
			apTrigger <= '0';
			--apInAddr <= (others => '0');
		elsif clk = '1' and clk'event then
			pcpTrigger <= '0';
			--pcpInAddr <= (others => '0');
			apTrigger <= '0';
			--apInAddr <= (others => '0');
			
			case i is
				when 10 | 30 | 70 | 150 =>
					apTrigger <= '1';
				when 110 | 130 | 50 | 51 | 52 =>
					pcpTrigger <= '1';
				when others =>
			end case;
			
			if i = 53 then
				apTrigger <= '1';
				pcpTrigger <= '1';
			end if;
			
			i := i + 1;
			
		end if;
	end process;
		

end TB_ARCHITECTURE;

configuration TESTBENCH_FOR_triplevbuflogic of triplevbuflogic_tb is
	for TB_ARCHITECTURE
		for UUT : triplevbuflogic
			use entity work.triplevbuflogic(rtl);
		end for;
	end for;
end TESTBENCH_FOR_triplevbuflogic;

