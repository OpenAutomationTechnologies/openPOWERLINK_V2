-------------------------------------------------------------------------------
-- Parallel port (8/16bit) for PDI
--
--       Copyright (C) 2010 B&R
--
--    Redistribution and use in source and binary forms, with or without
--    modification, are permitted provided that the following conditions
--    are met:
--
--    1. Redistributions of source code must retain the above copyright
--       notice, this list of conditions and the following disclaimer.
--
--    2. Redistributions in binary form must reproduce the above copyright
--       notice, this list of conditions and the following disclaimer in the
--       documentation and/or other materials provided with the distribution.
--
--    3. Neither the name of B&R nor the names of its
--       contributors may be used to endorse or promote products derived
--       from this software without prior written permission. For written
--       permission, please contact office@br-automation.com
--
--    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
--    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
--    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
--    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
--    COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
--    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
--    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
--    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
--    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
--    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
--    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
--    POSSIBILITY OF SUCH DAMAGE.
--
-------------------------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.std_logic_arith.all;
USE ieee.std_logic_unsigned.all;

entity pdi_spi is
    generic (
        spiSize_g            : integer    := 8;
        cpol_g                : boolean    := false;
        cpha_g                : boolean    := false;
        spiBigEnd_g            : boolean    := false
    );

    port (
        -- SPI
        spi_clk                : in     std_logic;
        spi_sel                : in     std_logic;
        spi_miso            : out     std_logic;
        spi_mosi            : in     std_logic;
        -- clock for AP side
        ap_reset            : in    std_logic;
        ap_clk                : in    std_logic;
        -- Avalon Slave Interface for AP
        ap_chipselect       : out    std_logic;
        ap_read                : out    std_logic;
        ap_write            : out    std_logic;
        ap_byteenable       : out    std_logic_vector(3 DOWNTO 0);
        ap_address          : out    std_logic_vector(12 DOWNTO 0);
        ap_writedata        : out    std_logic_vector(31 DOWNTO 0);
        ap_readdata         : in    std_logic_vector(31 DOWNTO 0)
    );
end entity pdi_spi;

architecture rtl of pdi_spi is
--wake up command
constant cmdWakeUp            :        std_logic_vector(7 downto 0)    := x"03"; --0b00000011
constant cmdWakeUp1            :        std_logic_vector(7 downto 0)    := x"0A"; --0b00001010
constant cmdWakeUp2            :        std_logic_vector(7 downto 0)    := x"0C"; --0b00001100
constant cmdWakeUp3            :        std_logic_vector(7 downto 0)    := x"0F"; --0b00001111
--spi frame constants
constant cmdHighaddr_c        :        std_logic_vector(2 downto 0)    := "100";
constant cmdMidaddr_c        :        std_logic_vector(2 downto 0)    := "101";
constant cmdWr_c            :        std_logic_vector(2 downto 0)    := "110";
constant cmdRd_c            :        std_logic_vector(2 downto 0)    := "111";
constant cmdWRSQ_c            :        std_logic_vector(2 downto 0)    := "001";
constant cmdRDSQ_c            :        std_logic_vector(2 downto 0)    := "010";
constant cmdLowaddr_c        :        std_logic_vector(2 downto 0)    := "011";
constant cmdIdle_c            :        std_logic_vector(2 downto 0)    := "000";
--pdi_spi control signals
type fsm_t is (reset, reset1, reset2, reset3, idle, decode, waitwr, waitrd, wr, rd);
signal    fsm                    :        fsm_t;
signal    addrReg                :        std_logic_vector(ap_address'left+2 downto 0);
signal    cmd                    :        std_logic_vector(2 downto 0);
signal    highPriorLoad        :        std_logic;
signal    highPriorLoadVal    :        std_logic_vector(spiSize_g-1 downto 0);
--spi core signals
signal    clk                    :          std_logic;
signal    rst                    :          std_logic;
signal    din                    :          std_logic_vector(spiSize_g-1 downto 0);
signal    load                :         std_logic;
signal    dout                :          std_logic_vector(spiSize_g-1 downto 0);
signal    valid                :         std_logic;
--
signal ap_byteenable_s        :        std_logic_vector(ap_byteenable'range);
begin

    clk <= ap_clk;
    rst <= ap_reset;

    ap_chipselect <= '1' when fsm = wr or fsm = rd or fsm = waitrd else '0';
    ap_write <= '1' when fsm = wr else '0';
    ap_read <= '1' when fsm = waitrd or fsm = rd else '0';
    ap_address <= addrReg(addrReg'left downto 2);

    ap_byteenable    <=    ap_byteenable_s;
    ap_byteenable_s <=    --little endian
                        "0001" when addrReg(1 downto 0) = "00" and spiBigEnd_g = false else
                        "0010" when addrReg(1 downto 0) = "01" and spiBigEnd_g = false else
                        "0100" when addrReg(1 downto 0) = "10" and spiBigEnd_g = false else
                        "1000" when addrReg(1 downto 0) = "11" and spiBigEnd_g = false else
                        --big endian
                        --"0001" when addrReg(1 downto 0) = "11" and spiBigEnd_g = true else
                        --"0010" when addrReg(1 downto 0) = "10" and spiBigEnd_g = true else
                        --"0100" when addrReg(1 downto 0) = "01" and spiBigEnd_g = true else
                        --"1000" when addrReg(1 downto 0) = "00" and spiBigEnd_g = true else
                        "0000";

    ap_writedata <=        (dout & dout & dout & dout);

    din <=                highPriorLoadVal when highPriorLoad = '1' else --load value that was just received
                        ap_readdata( 7 downto  0) when ap_byteenable_s = "0001" else
                        ap_readdata(15 downto  8) when ap_byteenable_s = "0010" else
                        ap_readdata(23 downto 16) when ap_byteenable_s = "0100" else
                        ap_readdata(31 downto 24) when ap_byteenable_s = "1000" else
                        (others => '0');

    load <=             '1' when highPriorLoad = '1' else --load value that was just received
                        '1' when fsm = rd else --load data from pdi to spi shift register
                        '0';

    cmd <= dout(dout'left downto dout'left-2); --get cmd pattern

    highPriorLoadVal <= not dout; --create inverse of received pattern

    thePdiSpiFsm : process(clk, rst)
    variable timeout : integer range 0 to 3;
    variable writes : integer range 0 to 32;
    variable reads : integer range 0 to 32;
    begin
        if rst = '1' then
            fsm <= reset;
            timeout := 0;
            writes := 0; reads := 0;
            addrReg <= (others => '0');
            highPriorLoad <= '0';
        elsif clk = '1' and clk'event then
            --default assignment
            highPriorLoad <= '0';

            case fsm is
                when reset =>
                    fsm <= reset;
                    if valid = '1' then
                        --load inverse pattern of received pattern
                        highPriorLoad <= '1';

                        if dout = cmdWakeUp then
                            --wake up command (1/4) received
                            fsm <= reset1;
                        else
                            --wake up command not decoded correctly
                            fsm <= reset;
                        end if;
                    end if;

                when reset1 =>
                    fsm <= reset1;
                    if valid = '1' then
                        --load inverse pattern of received pattern
                        highPriorLoad <= '1';

                        if dout = cmdWakeUp1 then
                            --wake up command (2/4) sequence was correctly decoded!
                            fsm <= reset2;
                        else
                            --wake up command not decoded correctly
                            fsm <= reset;
                        end if;
                    end if;

                when reset2 =>
                    fsm <= reset2;
                    if valid = '1' then
                        --load inverse pattern of received pattern
                        highPriorLoad <= '1';

                        if dout = cmdWakeUp2 then
                            --wake up command (3/4) sequence was correctly decoded!
                            fsm <= reset3;
                        else
                            --wake up command not decoded correctly
                            fsm <= reset;
                        end if;
                    end if;

                when reset3 =>
                    fsm <= reset3;
                    if valid = '1' then
                        --load inverse pattern of received pattern
                        highPriorLoad <= '1';

                        if dout = cmdWakeUp3 then
                            --wake up command (4/4) sequence was correctly decoded!
                            fsm <= idle;
                        else
                            --wake up command not decoded correctly
                            fsm <= reset;
                        end if;
                    end if;

                when idle =>
                    if writes /= 0 then
                        fsm <= waitwr;
                    elsif reads /= 0 and valid = '1' then
                        fsm <= waitrd;
                    elsif valid = '1' then
                        fsm <= decode;
                    else
                        fsm <= idle;
                    end if;

                when decode =>
                    fsm <= idle; --default
                    case cmd is
                        when cmdHighaddr_c =>
                            addrReg(addrReg'left downto addrReg'left-4) <= dout(spiSize_g-4 downto 0);
                        when cmdMidaddr_c =>
                            addrReg(addrReg'left-5 downto addrReg'left-9) <= dout(spiSize_g-4 downto 0);
                        when cmdLowaddr_c =>
                            addrReg(addrReg'left-10 downto 0) <= dout(spiSize_g-4 downto 0);
                        when cmdWr_c =>
                            addrReg(addrReg'left-10 downto 0) <= dout(spiSize_g-4 downto 0);
                            fsm <= waitwr;
                            writes := 1;
                        when cmdRd_c =>
                            addrReg(addrReg'left-10 downto 0) <= dout(spiSize_g-4 downto 0);
                            fsm <= waitrd;
                            reads := 1;
                        when cmdWRSQ_c =>
                            fsm <= waitwr;
                            writes := conv_integer(dout(spiSize_g-4 downto 0)) + 1; --BYTES byte are written
                        when cmdRDSQ_c =>
                            fsm <= waitrd;
                            reads := conv_integer(dout(spiSize_g-4 downto 0)) + 1; --BYTES byte are read
                        when cmdIdle_c =>
                            --don't interpret the command, inverse pattern and goto idle
                        when others =>
                            --error, goto idle
                    end case;

                when waitwr =>
                    --wait for data from spi master
                    if valid = '1' then
                        fsm <= wr;
                    else
                        fsm <= waitwr;
                    end if;

                when waitrd =>
                    --spi master wants to read
                    --wait for dpr to read
                    if timeout = 3 then
                        fsm <= rd;
                        timeout := 0;
                    else
                        timeout := timeout + 1;
                        fsm <= waitrd;
                    end if;

                when wr =>
                    fsm <= idle;
                    writes := writes - 1;
                    addrReg <= addrReg + 1;

                when rd =>
                    fsm <= idle;
                    reads := reads - 1;
                    addrReg <= addrReg + 1;

            end case;

        end if;
    end process;

    theSpiCore : entity work.spi
    generic map (
        frameSize_g            => spiSize_g,
        cpol_g                => cpol_g,
        cpha_g                => cpha_g
    )
    port map (
        -- Control Interface
        clk                    => clk,
        rst                    => rst,
        din                    => din,
        load                => load,
        dout                => dout,
        valid                => valid,
        -- SPI
        sck                    => spi_clk,
        ss                    => spi_sel,
        miso                => spi_miso,
        mosi                => spi_mosi
    );

end architecture rtl;
