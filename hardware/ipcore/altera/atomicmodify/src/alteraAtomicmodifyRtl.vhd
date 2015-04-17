-------------------------------------------------------------------------------
--! @file alteraAtomicmodifyRtl.vhd
--
--! @brief Atomic modify toplevel for Altera
--
--! @details This is the toplevel for Altera specific implementation.
-------------------------------------------------------------------------------
--
--    (c) B&R, 2015
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

entity alteraAtomicmodify is
    generic (
        --! Address bus width
        gAddrWidth  : natural := 16
    );
    port (
        --! Clock
        csi_c0_clock        : in    std_logic;
        --! Reset
        rsi_r0_reset        : in    std_logic;
        -- Memory Mapped master
        --! MM master address
        avm_m0_address      : out   std_logic_vector(gAddrWidth-1 downto 0);
        --! MM master byteenable
        avm_m0_byteenable   : out   std_logic_vector(3 downto 0);
        --! MM master read
        avm_m0_read         : out   std_logic;
        --! MM master readdata
        avm_m0_readdata     : in    std_logic_vector(31 downto 0);
        --! MM master write
        avm_m0_write        : out   std_logic;
        --! MM master writedata
        avm_m0_writedata    : out   std_logic_vector(31 downto 0);
        --! MM master waitrequest
        avm_m0_waitrequest  : in    std_logic;
        --! MM master lock
        avm_m0_lock         : out   std_logic;
        -- Memory mapped slave
        --! Address
        avs_s0_address      : in    std_logic_vector(gAddrWidth-1 downto 2);
        --! Byteenable
        avs_s0_byteenable   : in    std_logic_vector(3 downto 0);
        --! Read strobe
        avs_s0_read         : in    std_logic;
        --! Readdata
        avs_s0_readdata     : out   std_logic_vector(31 downto 0);
        --! Write strobe
        avs_s0_write        : in    std_logic;
        --! Writedata
        avs_s0_writedata    : in    std_logic_vector(31 downto 0);
        --! Waitrequest
        avs_s0_waitrequest  : out   std_logic
    );
end alteraAtomicmodify;

architecture rtl of alteraAtomicmodify is
begin
    theAtomicModifyCore : entity work.atomicmodify
        generic map (
            gAddrWidth          => gAddrWidth
        )
        port map (
            iClk                => csi_c0_clock,
            iRst                => rsi_r0_reset,
            oMst_address        => avm_m0_address,
            oMst_byteenable     => avm_m0_byteenable,
            oMst_read           => avm_m0_read,
            iMst_readdata       => avm_m0_readdata,
            oMst_write          => avm_m0_write,
            oMst_writedata      => avm_m0_writedata,
            iMst_waitrequest    => avm_m0_waitrequest,
            oMst_lock           => avm_m0_lock,
            iSlv_address        => avs_s0_address,
            iSlv_byteenable     => avs_s0_byteenable,
            iSlv_read           => avs_s0_read,
            oSlv_readdata       => avs_s0_readdata,
            iSlv_write          => avs_s0_write,
            iSlv_writedata      => avs_s0_writedata,
            oSlv_waitrequest    => avs_s0_waitrequest
        );
end rtl;
