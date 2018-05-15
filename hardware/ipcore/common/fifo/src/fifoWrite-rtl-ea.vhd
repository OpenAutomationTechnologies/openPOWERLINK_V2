-------------------------------------------------------------------------------
--! @file fifoWrite-rtl-ea.vhd
--
--! @brief FIFO write controller
--
--! @details This is a FIFO write controller.
--
-------------------------------------------------------------------------------
--
--    (c) B&R Industrial Automation GmbH, 2014
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

--! Common library
library libcommon;
--! Use common library global package
use libcommon.global.all;

entity fifoWrite is
    generic (
        gAddrWidth : natural := 4
    );
    port (
        iClk        : in std_logic;
        iRst        : in std_logic;
        iWrite      : in std_logic;
        iRdPointer  : in std_logic_vector(gAddrWidth downto 0);
        oFull       : out std_logic;
        oEmpty      : out std_logic;
        oPointer    : out std_logic_vector(gAddrWidth downto 0);
        oAddress    : out std_logic_vector(gAddrWidth-1 downto 0);
        oUsedWord   : out std_logic_vector(gAddrWidth-1 downto 0)
    );
end fifoWrite;

architecture rtl of fifoWrite is
    signal w_ptr_reg        : std_logic_vector(gAddrWidth downto 0);
    signal w_ptr_next       : std_logic_vector(gAddrWidth downto 0);
    signal gray1            : std_logic_vector(gAddrWidth downto 0);
    signal bin              : std_logic_vector(gAddrWidth downto 0);
    signal bin1             : std_logic_vector(gAddrWidth downto 0);
    signal waddr_all        : std_logic_vector(gAddrWidth-1 downto 0);
    signal waddr_msb        : std_logic;
    signal raddr_msb        : std_logic;
    signal full_flag        : std_logic;
    signal empty_flag       : std_logic;
    signal w_elements_wr    : std_logic_vector(gAddrWidth downto 0);
    signal w_elements_rd    : std_logic_vector(gAddrWidth downto 0);
    signal w_elements_diff  : std_logic_vector(gAddrWidth downto 0);
    signal w_elements_reg   : std_logic_vector(gAddrWidth-1 downto 0);
    signal w_elements_next  : std_logic_vector(gAddrWidth-1 downto 0);
begin
    --! Clock process for registers.
    regProc : process(iClk, iRst)
    begin
        if iRst = cActivated then
            w_ptr_reg       <= (others => cInactivated);
            w_elements_reg  <= (others => cInactivated);
        elsif rising_edge(iClk) then
            w_ptr_reg       <= w_ptr_next;
            w_elements_reg  <= w_elements_next;
        end if;
    end process;

    -- (gAddrWidth+1)-bit Gray counter
    bin     <= w_ptr_reg xor (cInactivated & bin(gAddrWidth downto 1));
    bin1    <= std_logic_vector(unsigned(bin) + 1);
    gray1   <= bin1 xor (cInactivated & bin1(gAddrWidth downto 1));

    -- update write pointer
    w_ptr_next  <=  gray1 when iWrite = cActivated and full_flag = cInactivated else
                    w_ptr_reg;

    -- gAddrWidth-bit Gray counter
    waddr_msb <= w_ptr_reg(gAddrWidth) xor w_ptr_reg(gAddrWidth-1);
    waddr_all <= waddr_msb & w_ptr_reg(gAddrWidth-2 downto 0);
    raddr_msb <= iRdPointer(gAddrWidth) xor iRdPointer(gAddrWidth-1);

    -- check for FIFO write empty
    empty_flag  <=  cActivated when iRdPointer(gAddrWidth) = w_ptr_reg(gAddrWidth) and
                    iRdPointer(gAddrWidth-2 downto 0) = w_ptr_reg(gAddrWidth-2 downto 0) and
                    raddr_msb = waddr_msb else
                    cInactivated;

    -- check for FIFO write full
    full_flag   <=  cActivated when iRdPointer(gAddrWidth) /= w_ptr_reg(gAddrWidth) and
                    iRdPointer(gAddrWidth-2 downto 0) = w_ptr_reg(gAddrWidth-2 downto 0) and
                    raddr_msb = waddr_msb else
                    cInactivated;

    -- convert gray value to bin and obtain difference
    w_elements_wr   <= bin;
    w_elements_rd   <= iRdPointer xor (cInactivated & w_elements_rd(gAddrWidth downto 1));
    w_elements_diff <= std_logic_vector(unsigned(w_elements_wr) - unsigned(w_elements_rd));
    w_elements_next <= w_elements_diff(w_elements_next'range);

    -- output
    oAddress    <= waddr_all;
    oPointer    <= w_ptr_reg;
    oUsedWord   <= w_elements_reg;
    oEmpty      <= empty_flag;
    oFull       <= full_flag;
end rtl;
