LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.std_logic_arith.ALL;
USE ieee.std_logic_unsigned.ALL;

entity spi_sreg is
    generic (
        size_g                : integer    := 8
    );
    port (
        clk                    : in     std_logic;
        rst                    : in     std_logic;
        --control signals
        shift                : in    std_logic; --shift left
        load                : in    std_logic; --load parallel
        --data signals
        din                    : in    std_logic_vector(size_g-1 downto 0); --parallel data in (latched)
        dout                : out    std_logic_vector(size_g-1 downto 0); --parallel data out
        sin                    : in    std_logic; --serial data in (to lsb)
        sout                : out    std_logic --serial data out (from msb)
    );
end spi_sreg;

architecture rtl of spi_sreg is
signal    shiftReg            :        std_logic_vector(size_g-1 downto 0);
begin
    theShiftRegister : process(clk, rst)
    begin
        if rst = '1' then
            shiftReg <= (others => '0');
        elsif clk = '1' and clk'event then
            if shift = '1' then
                shiftReg <= shiftReg(size_g-2 downto 0) & sin;
            elsif load = '1' then
                shiftReg <= din;
            end if;
        end if;
    end process;
    dout <= shiftReg;
    sout <= shiftReg(size_g-1);
end rtl;
