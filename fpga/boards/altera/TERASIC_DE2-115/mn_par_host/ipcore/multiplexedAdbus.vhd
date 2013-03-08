-- multiplexedadbus.vhd

-- this file was auto-generated as a prototype implementation of a module
-- created in component editor.  it ties off all outputs to ground and
-- ignores all inputs.  it needs to be edited to make it do something
-- useful.
--
-- this file will not be automatically regenerated.  you should check it in
-- to your version control system if you want to keep it.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity multiplexedadbus is
    port (
        avs_s0_address       : in    std_logic_vector(15 downto 0) := (others => '0'); --    s0.address
        avs_s0_read          : in    std_logic                     := '0';             --      .read
        avs_s0_readdata      : out   std_logic_vector(15 downto 0);                    --      .readdata
        avs_s0_write         : in    std_logic                     := '0';             --      .write
        avs_s0_writedata     : in    std_logic_vector(15 downto 0) := (others => '0'); --      .writedata
        avs_s0_waitrequest   : out   std_logic;                                        --      .waitrequest
        avs_s0_byteenable    : in    std_logic_vector(1 downto 0)  := (others => '0'); --      .byteenable
        clk                  : in    std_logic                     := '0';             -- clock.clk
        reset                : in    std_logic                     := '0';             -- reset.reset
        adbus_cs             : out   std_logic;                                        -- adbus.export
        adbus_ad             : inout std_logic_vector(15 downto 0) := (others => '0'); --      .export
        adbus_be             : out   std_logic_vector(1 downto 0);                     --      .export
        adbus_ale            : out   std_logic;                                        --      .export
        adbus_wr             : out   std_logic;                                        --      .export
        adbus_rd             : out   std_logic;                                        --      .export
        adbus_ack            : in    std_logic                     := '0'              --      .export
    );
end entity multiplexedadbus;

architecture rtl of multiplexedadbus is
    signal count : std_logic_vector(2 downto 0);
    constant countMax : std_logic_vector(count'range) := "111";
    type tFsm is (sIdle, sWait, sWrd);
    signal fsm : tFsm;
    signal ack : std_logic;
    signal ack_d : std_logic;
    signal ack_l : std_logic;
    signal readdata : std_logic_vector(avs_s0_readdata'range);
    signal readdata_l : std_logic_vector(avs_s0_readdata'range);
    signal adReg : std_logic_vector(adbus_ad'range);
begin

    process(clk, reset)
    begin
        if reset = '1' then
            count <= (others => '0');
            adbus_cs <= '0';
            adbus_ale <= '0';
            adbus_rd <= '0';
            adbus_wr <= '0';
            ack <= '0';
            ack_l <= '0';
            readdata <= (others => '0');
            readdata_l <= (others => '0');
            adbus_be <= (others => '0');
            adReg <= (others => '0');
        elsif rising_edge(clk) then

            ack_l <= adbus_ack;
            ack <= ack_l;
            ack_d <= ack;
            readdata_l <= adbus_ad;
            readdata <= readdata_l;

            if fsm = sIdle then
                count <= (others => '0');
            elsif fsm = sWait then
                count <= std_logic_vector(unsigned(count) + 1);
            end if;

            adbus_be <= avs_s0_byteenable;

            case fsm is
                when sIdle =>
                    adbus_cs <= '0';
                    adbus_ale <= '0';
                    adbus_rd <= '0';
                    adbus_wr <= '0';

                    if avs_s0_read = '1' or avs_s0_write = '1' then
                        fsm <= sWait;
                        adbus_cs <= '1';
                        adbus_ale <= '1';
                        adReg <= avs_s0_address;
                    end if;
                when sWait =>
                    if count = countMax then
                        fsm <= sWrd;
                        adbus_ale <= '0';
                        adbus_wr <= avs_s0_write;
                        adbus_rd <= avs_s0_read;
                        adReg <= avs_s0_writedata;
                    end if;
                when sWrd =>
                    if ack = '1' then
                        fsm <= sIdle;
                        adbus_cs <= '0';
                        adbus_rd <= '0';
                        adbus_wr <= '0';
                    end if;
            end case;
        end if;
    end process;

    adbus_ad <= adReg when fsm = sWait or (fsm = sWrd and avs_s0_write = '1') else
                (others => 'Z');

    -- if ack goes high deassert waitrequest (edge detection)
    avs_s0_waitrequest <= '0' when ack_d = '0' and ack = '1' else '1';
    avs_s0_readdata <= readdata;

end architecture rtl; -- of multiplexedadbus
