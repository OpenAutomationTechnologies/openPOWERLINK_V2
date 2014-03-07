-------------------------------------------------------------------------------
--! @file fifoRead-rtl-ea.vhd
--
--! @brief FIFO read controller
--
--! @details This is a FIFO read controller.
--
-------------------------------------------------------------------------------
--
--    (c) B&R, 2014
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

entity fifoRead is
    generic (
        gAddrWidth : natural := 4
    );
    port (
        iClk        : in std_logic;
        iRst        : in std_logic;
        iRead       : in std_logic;
        iWrPointer  : in std_logic_vector(gAddrWidth downto 0);
        oEmpty      : out std_logic;
        oFull       : out std_logic;
        oPointer    : out std_logic_vector(gAddrWidth downto 0);
        oAddress    : out std_logic_vector(gAddrWidth-1 downto 0);
        oUsedWord   : out std_logic_vector(gAddrWidth-1 downto 0)
   );
end fifoRead;

architecture rtl of fifoRead is
   signal r_ptr_reg         : std_logic_vector(gAddrWidth downto 0);
   signal r_ptr_next        : std_logic_vector(gAddrWidth downto 0);
   signal gray1             : std_logic_vector(gAddrWidth downto 0);
   signal bin               : std_logic_vector(gAddrWidth downto 0);
   signal bin1              : std_logic_vector(gAddrWidth downto 0);
   signal raddr_all         : std_logic_vector(gAddrWidth-1 downto 0);
   signal raddr_msb         : std_logic;
   signal waddr_msb         : std_logic;
   signal empty_flag        : std_logic;
   signal full_flag         : std_logic;
   signal r_elements_wr     : std_logic_vector(gAddrWidth downto 0);
   signal r_elements_rd     : std_logic_vector(gAddrWidth downto 0);
   signal r_elements_diff   : std_logic_vector(gAddrWidth downto 0);
   signal r_elements_reg    : std_logic_vector(gAddrWidth-1 downto 0);
   signal r_elements_next   : std_logic_vector(gAddrWidth-1 downto 0);
begin
    --! Clock process for registers.
    regProc : process(iRst, iClk)
    begin
        if iRst = cActivated then
            r_ptr_reg       <= (others => cInactivated);
            r_elements_reg  <= (others => cInactivated);
        elsif rising_edge(iClk) then
            r_ptr_reg       <= r_ptr_next;
            r_elements_reg  <= r_elements_next;
        end if;
    end process;

    -- (gAddrWidth+1)-bit Gray counter
    bin     <= r_ptr_reg xor (cInactivated & bin(gAddrWidth downto 1));
    bin1    <= std_logic_vector(unsigned(bin) + 1);
    gray1   <= bin1 xor (cInactivated & bin1(gAddrWidth downto 1));

    -- update read pointer
    r_ptr_next  <=  gray1 when iRead = cActivated and empty_flag = cInactivated else
                    r_ptr_reg;

    -- gAddrWidth-bit Gray counter
    raddr_msb <= r_ptr_reg(gAddrWidth) xor r_ptr_reg(gAddrWidth-1);
    raddr_all <= raddr_msb & r_ptr_reg(gAddrWidth-2 downto 0);
    waddr_msb <= iWrPointer(gAddrWidth) xor iWrPointer(gAddrWidth-1);

    -- check for FIFO read empty
    empty_flag  <=  cActivated when iWrPointer(gAddrWidth) = r_ptr_reg(gAddrWidth) and
                    iWrPointer(gAddrWidth-2 downto 0) = r_ptr_reg(gAddrWidth-2 downto 0) and
                    raddr_msb = waddr_msb else
                    cInactivated;

    -- check for FIFO read full
    full_flag   <=  cActivated when iWrPointer(gAddrWidth) /= r_ptr_reg(gAddrWidth) and
                    iWrPointer(gAddrWidth-2 downto 0) = r_ptr_reg(gAddrWidth-2 downto 0) and
                    raddr_msb = waddr_msb else
                    cInactivated;

    -- convert gray value to bin and obtain difference
    r_elements_wr   <= bin;
    r_elements_rd   <= iWrPointer xor (cInactivated & r_elements_rd(gAddrWidth downto 1));
    r_elements_diff <= std_logic_vector(unsigned(r_elements_rd) - unsigned(r_elements_wr));
    r_elements_next <= r_elements_diff(r_elements_next'range);

    -- output
    oAddress    <= raddr_all;
    oPointer    <= r_ptr_reg;
    oUsedWord   <= r_elements_reg;
    oEmpty      <= empty_flag;
    oFull       <= full_flag;
end rtl;
