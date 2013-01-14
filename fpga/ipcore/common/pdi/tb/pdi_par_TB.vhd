library ieee;
use ieee.STD_LOGIC_UNSIGNED.all;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;

	-- Add your library and packages declaration here ...

entity pdi_par_tb is
	-- Generic declarations of the tested unit
		generic(
		papDataWidth_g : INTEGER := 16;
		papBigEnd_g : BOOLEAN := false );
end pdi_par_tb;

architecture TB_ARCHITECTURE of pdi_par_tb is
	-- Component declaration of the tested unit
	component pdi_par
		generic(
		papDataWidth_g : INTEGER := 8;
		papBigEnd_g : BOOLEAN := false );
	port(
		pap_cs : in STD_LOGIC;
		pap_rd : in STD_LOGIC;
		pap_wr : in STD_LOGIC;
		pap_be : in STD_LOGIC_VECTOR(papDataWidth_g/8-1 downto 0);
		pap_addr : in STD_LOGIC_VECTOR(15 downto 0);
		pap_data : inout STD_LOGIC_VECTOR(papDataWidth_g-1 downto 0);
		pap_ack : out STD_LOGIC;
		ap_reset : in STD_LOGIC;
		ap_clk : in STD_LOGIC;
		ap_chipselect : out STD_LOGIC;
		ap_read : out STD_LOGIC;
		ap_write : out STD_LOGIC;
		ap_byteenable : out STD_LOGIC_VECTOR(3 downto 0);
		ap_address : out STD_LOGIC_VECTOR(12 downto 0);
		ap_writedata : out STD_LOGIC_VECTOR(31 downto 0);
		ap_readdata : in STD_LOGIC_VECTOR(31 downto 0);
		pap_gpio : inout STD_LOGIC_VECTOR(1 downto 0) );
	end component;

	-- Stimulus signals - signals mapped to the input and inout ports of tested entity
	signal pap_cs : STD_LOGIC;
	signal pap_rd : STD_LOGIC;
	signal pap_wr : STD_LOGIC;
	signal pap_be : STD_LOGIC_VECTOR(papDataWidth_g/8-1 downto 0);
	signal pap_addr : STD_LOGIC_VECTOR(15 downto 0);
	signal ap_reset : STD_LOGIC;
	signal ap_clk : STD_LOGIC;
	signal ap_readdata : STD_LOGIC_VECTOR(31 downto 0);
	signal pap_data : STD_LOGIC_VECTOR(papDataWidth_g-1 downto 0);
	signal pap_gpio : STD_LOGIC_VECTOR(1 downto 0);
	-- Observed signals - signals mapped to the output ports of tested entity
	signal pap_ack : STD_LOGIC;
	signal ap_chipselect : STD_LOGIC;
	signal ap_read : STD_LOGIC;
	signal ap_write : STD_LOGIC;
	signal ap_byteenable : STD_LOGIC_VECTOR(3 downto 0);
	signal ap_address : STD_LOGIC_VECTOR(12 downto 0);
	signal ap_writedata : STD_LOGIC_VECTOR(31 downto 0);

	-- Add your code here ...
    --timing constants
    constant cTper : time := 20 ns; --reference clock
    constant cTsc : time := 0 ns; --cs to wr/rd
    constant cTpwr : time := 20 ns; --wr pulse
    constant cTsd : time := 10 ns; --data setupt
    constant cThd : time := 10 ns; --data hold
    constant cTiwr : time := 20 ns; --idle after write
    constant cTaa : time := 60 ns; --address to read access
    constant cToha : time := 20 ns; --output hold
    constant cThc : time := 0 ns; --chipselect hold
    constant cTird : time := 40 ns; --idle after read
    
    procedure doAccess
    (
        accType : in string;
        accAddr : in integer;
        accData : in integer;
        signal cs : out std_logic;
        signal rd : out std_logic;
        signal wr : out std_logic;
        signal be : out std_logic_vector(pap_be'range);
        signal addr : out std_logic_vector(pap_addr'range);
        signal data : inout std_logic_vector(pap_data'range)
    ) is
        
    begin
        if accType = "write" then
            --write access
            cs <= '1';
            wait for cTsc;
            rd <= '0';
            wr <= '1';
            be <= (others => '1');
            addr <= conv_std_logic_vector(accAddr, pap_addr'length);
            wait for (cTpwr - cTsd);
            data <= conv_std_logic_vector(accData, pap_data'length);
            wait for cTsd;
            wr <= '0';
            cs <= '0';
            be <= (others => '0');
            addr <= conv_std_logic_vector(16#0#, pap_addr'length);
            wait for cThd;
            data <= (others => 'Z');
            wait for (cTiwr - cThd);
        else
            --read access
            cs <= '1';
            wait for cTsc;
            rd <= '1';
            wr <= '0';
            be <= (others => '1');
            addr <= conv_std_logic_vector(accAddr, pap_addr'length);
            data <= (others => 'Z');
            wait for cTaa;
            wait for cTper; --wait one period
            rd <= '0';
            be <= (others => '0');
            addr <= conv_std_logic_vector(16#0#, pap_addr'length);
            wait for cThc;
            cs <= '0';
            wait for (cToha - cThc);
            wait for (cTird - cToha - cThc);
        end if;
    end doAccess;    

begin

	-- Unit Under Test port map
	UUT : pdi_par
		generic map (
			papDataWidth_g => papDataWidth_g,
			papBigEnd_g => papBigEnd_g
		)

		port map (
			pap_cs => pap_cs,
			pap_rd => pap_rd,
			pap_wr => pap_wr,
			pap_be => pap_be,
			pap_addr => pap_addr,
			pap_data => pap_data,
			pap_ack => pap_ack,
			ap_reset => ap_reset,
			ap_clk => ap_clk,
			ap_chipselect => ap_chipselect,
			ap_read => ap_read,
			ap_write => ap_write,
			ap_byteenable => ap_byteenable,
			ap_address => ap_address,
			ap_writedata => ap_writedata,
			ap_readdata => ap_readdata,
			pap_gpio => pap_gpio
		);

	-- Add your stimulus here ...
	
--	signal ap_reset : STD_LOGIC;
--	signal ap_clk : STD_LOGIC;
process
begin
	ap_clk <= '0';
	wait for cTper/2;
	ap_clk <= not ap_clk;
	wait for cTper/2;
end process;

process
begin
	ap_reset <= '1';
	wait for 100 ns;
	ap_reset <= not ap_reset;
	wait;
end process;


ap_readdata <= x"01010101" after 20 ns when ap_read = '1' and ap_chipselect = '1' else (others => '0');
	
pap_gpio <= (others => '0');

process
begin
	pap_cs <= '0';
	pap_rd <= '0';
	pap_wr <= '0';
	pap_be <= (others => '0');
	pap_addr <= conv_std_logic_vector(16#0#, pap_addr'length);
	pap_data <= (others => 'Z');
	wait until ap_reset = '0';
	wait for 100 ns;
	wait until ap_clk = '0' and ap_clk'event; --sync on falling edge
    
    doAccess("write", 16#0010#, 16#1234#, pap_cs, pap_rd, pap_wr, pap_be, pap_addr, pap_data);
    
    doAccess("read", 16#0020#, 0, pap_cs, pap_rd, pap_wr, pap_be, pap_addr, pap_data);
    
    doAccess("write", 16#0030#, 16#5678#, pap_cs, pap_rd, pap_wr, pap_be, pap_addr, pap_data);
    
    doAccess("read", 16#0040#, 0, pap_cs, pap_rd, pap_wr, pap_be, pap_addr, pap_data);
    
    wait for 100 ns;
    
    doAccess("write", 16#0030#, 16#0101#, pap_cs, pap_rd, pap_wr, pap_be, pap_addr, pap_data);
    doAccess("write", 16#0032#, 16#0202#, pap_cs, pap_rd, pap_wr, pap_be, pap_addr, pap_data);
    doAccess("write", 16#0034#, 16#0303#, pap_cs, pap_rd, pap_wr, pap_be, pap_addr, pap_data);
    doAccess("write", 16#0036#, 16#0404#, pap_cs, pap_rd, pap_wr, pap_be, pap_addr, pap_data);
    
    wait for 100 ns;
    
    doAccess("read", 16#0040#, 0, pap_cs, pap_rd, pap_wr, pap_be, pap_addr, pap_data);
    doAccess("read", 16#0042#, 0, pap_cs, pap_rd, pap_wr, pap_be, pap_addr, pap_data);
    doAccess("read", 16#0044#, 0, pap_cs, pap_rd, pap_wr, pap_be, pap_addr, pap_data);
    doAccess("read", 16#0046#, 0, pap_cs, pap_rd, pap_wr, pap_be, pap_addr, pap_data);
	
	wait;
end process;

end TB_ARCHITECTURE;

configuration TESTBENCH_FOR_pdi_par of pdi_par_tb is
	for TB_ARCHITECTURE
		for UUT : pdi_par
			use entity work.pdi_par(rtl);
		end for;
	end for;
end TESTBENCH_FOR_pdi_par;

