-------------------------------------------------------------------------------
--! @file tbParallelInterfaceBhv.vhd
--
--! @brief Testbench for Parallel Interface for Host Interface
--
--! @details This testbench verifies the Parallel Interface implementation
--
-------------------------------------------------------------------------------
--
--    (c) B&R, 2013
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

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
--! use global library
use work.global.all;
--! use host interface package for specific types
use work.hostInterfacePkg.all;

entity tbParallelInterface is
end tbParallelInterface;

architecture bhv of tbParallelInterface is
    -- DUT
    constant cDataWidth : natural := 16;
    constant cMultiplex : natural := 1; -- 0 = FALSE
    signal parHostChipselect : std_logic;
    signal parHostRead : std_logic;
    signal parHostWrite : std_logic;
    signal parHostAddressLatchEnable : std_logic;
    signal parHostAcknowledge : std_logic;
    signal parHostAddress : std_logic_vector(15 downto 0);
    signal parHostByteenable : std_logic_vector(cDataWidth/cByte-1 downto 0);
    signal parHostData_o : std_logic_vector(cDataWidth-1 downto 0);
    signal parHostData_i : std_logic_vector(cDataWidth-1 downto 0);
    signal parHostData_t : std_logic;
    signal parHostAddressData_o : std_logic_vector(cDataWidth-1 downto 0);
    signal parHostAddressData_i : std_logic_vector(cDataWidth-1 downto 0);
    signal parHostAddressData_t : std_logic;
    signal hostAddress : std_logic_vector(16 downto 2);
    signal hostByteenable : std_logic_vector(3 downto 0);
    signal hostRead : std_logic;
    signal hostReaddata : std_logic_vector(31 downto 0);
    signal hostWrite : std_logic;
    signal hostWritedata : std_logic_vector(31 downto 0);
    signal hostWaitrequest : std_logic;

    signal clk : std_logic;
    signal rst : std_logic;
    signal done : std_logic;
begin

    clkGen : entity work.clkGen
        generic map (
            gPeriod => 10 ns
        )
        port map (
            iDone => done,
            oClk => clk
        );

    rstGen : entity work.resetGen
        port map (
            oReset => rst
        );

    DUT : entity work.parallelInterface
        generic map (
            gDataWidth => cDataWidth,
            gMultiplex => cMultiplex
        )
        port map (
            iParHostChipselect => parHostChipselect,
            iParHostRead => parHostRead,
            iParHostWrite => parHostWrite,
            iParHostAddressLatchEnable => parHostAddressLatchEnable,
            oParHostAcknowledge => parHostAcknowledge,
            iParHostByteenable => parHostByteenable,
            iParHostAddress => parHostAddress,
            oParHostData => parHostData_o,
            iParHostData => parHostData_i,
            oParHostDataEnable => parHostData_t,
            oParHostAddressData => parHostAddressData_o,
            iParHostAddressData => parHostAddressData_i,
            oParHostAddressDataEnable => parHostAddressData_t,
            iClk => clk,
            iRst => rst,
            oHostAddress => hostAddress,
            oHostByteenable => hostByteenable,
            oHostRead => hostRead,
            iHostReaddata => hostReaddata,
            oHostWrite => hostWrite,
            oHostWritedata => hostWritedata,
            iHostWaitrequest => hostWaitrequest
        );

        doMultiTest : if cMultiplex /= 0 generate
        begin
            parHostChipselect <=            '0',
                                            '1' after 101 ns,
                                            '0' after 231 ns,
                                            '1' after 251 ns,
                                            '0' after 361 ns,
                                            '1' after 371 ns,
                                            '0' after 501 ns;

            parHostWrite <=                 '0',
                                            '1' after 151 ns,
                                            '0' after 231 ns,
                                            '1' after 381 ns,
                                            '0' after 501 ns;

            parHostAddressLatchEnable <=    '0',
                                            '1' after 101 ns,
                                            '0' after 121 ns,
                                            '1' after 251 ns,
                                            '0' after 261 ns,
                                            '1' after 371 ns,
                                            '0' after 381 ns;

            parHostAddressData_i <=         std_logic_vector(to_unsigned(0, parHostAddressData_i'length)),
                                            std_logic_vector(to_unsigned(16#12AB#, parHostAddressData_i'length)) after 101 ns,
                                            std_logic_vector(to_unsigned(16#5555#, parHostAddressData_i'length)) after 151 ns,
                                            std_logic_vector(to_unsigned(0, parHostAddressData_i'length)) after 231 ns,
                                            std_logic_vector(to_unsigned(16#34CD#, parHostAddressData_i'length)) after 251 ns,
                                            std_logic_vector(to_unsigned(0, parHostAddressData_i'length)) after 301 ns,
                                            std_logic_vector(to_unsigned(16#EEFF#, parHostAddressData_i'length)) after 371 ns,
                                            std_logic_vector(to_unsigned(16#8888#, parHostAddressData_i'length)) after 421 ns,
                                            std_logic_vector(to_unsigned(0, parHostAddressData_i'length)) after 501 ns;

            parHostRead <=                  '0',
                                            '1' after 261 ns,
                                            '0' after 361 ns;

            parHostByteenable <=            (others => '0'),
                                            (others => '1') after 101 ns,
                                            (others => '0') after 231 ns,
                                            (others => '1') after 251 ns,
                                            (others => '0') after 361 ns,
                                            (others => '1') after 371 ns,
                                            (others => '0') after 501 ns;

            hostWaitrequest <=              '1',
                                            '0' after 191 ns,
                                            '1' after 201 ns,
                                            '0' after 311 ns,
                                            '1' after 321 ns,
                                            '0' after 461 ns,
                                            '1' after 471 ns;

            hostReaddata <=                 std_logic_vector(to_unsigned(0, hostReaddata'length)),
                                            std_logic_vector(to_unsigned(16#C0FFEE#, hostReaddata'length)) after 311 ns,
                                            std_logic_vector(to_unsigned(0, hostReaddata'length)) after 321 ns;

            done <=                         '0',
                                            '1' after 600 ns;
        end generate;

        doDeMultiTest : if cMultiplex = 0 generate
        begin
            parHostChipselect <=            '0',
                                            '1' after 101 ns,
                                            '0' after 191 ns,
                                            '1' after 251 ns,
                                            '0' after 361 ns;

            parHostWrite <=                 '0',
                                            '1' after 101 ns,
                                            '0' after 191 ns;

            parHostData_i <=                std_logic_vector(to_unsigned(0, parHostAddress'length)),
                                            std_logic_vector(to_unsigned(16#ABCD#, parHostAddress'length)) after 101 ns,
                                            std_logic_vector(to_unsigned(16#ABCD#, parHostAddress'length)) after 171 ns;

            parHostRead <=                  '0',
                                            '1' after 251 ns,
                                            '0' after 361 ns;

            parHostByteenable <=            (others => '0'),
                                            (others => '1') after 101 ns,
                                            (others => '0') after 171 ns,
                                            (others => '1') after 251 ns,
                                            (others => '0') after 351 ns;

            parHostAddress <=               std_logic_vector(to_unsigned(0, parHostAddress'length)),
                                            std_logic_vector(to_unsigned(16#1234#, parHostAddress'length)) after 101 ns,
                                            std_logic_vector(to_unsigned(0, parHostAddress'length)) after 191 ns,
                                            std_logic_vector(to_unsigned(16#5566#, parHostAddress'length)) after 251 ns,
                                            std_logic_vector(to_unsigned(0, parHostAddress'length)) after 361 ns;

            hostWaitrequest <=              '1',
                                            '0' after 151 ns,
                                            '1' after 161 ns,
                                            '0' after 311 ns,
                                            '1' after 321 ns;

            hostReaddata <=                 std_logic_vector(to_unsigned(0, hostReaddata'length)),
                                            std_logic_vector(to_unsigned(16#C0FFEE#, hostReaddata'length)) after 311 ns,
                                            std_logic_vector(to_unsigned(0, hostReaddata'length)) after 321 ns;

            done <=                         '0',
                                            '1' after 500 ns;
        end generate;



end bhv;
