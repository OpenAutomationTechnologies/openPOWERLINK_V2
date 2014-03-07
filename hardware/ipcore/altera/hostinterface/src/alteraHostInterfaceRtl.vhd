-------------------------------------------------------------------------------
--! @file alteraHostInterface.vhd
--
--! @brief toplevel of host interface for Altera FPGA
--
--! @details This toplevel interfaces to Altera specific implementation.
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

entity alteraHostInterface is
    generic (
        --! Version major
        gVersionMajor       : natural := 16#FF#;
        --! Version minor
        gVersionMinor       : natural := 16#FF#;
        --! Version revision
        gVersionRevision    : natural := 16#FF#;
        --! Version count
        gVersionCount       : natural := 0;
        -- Base address mapping
        --! Base address Dynamic Buffer 0
        gBaseDynBuf0        : natural := 16#00800#;
        --! Base address Dynamic Buffer 1
        gBaseDynBuf1        : natural := 16#01000#;
        --! Base address Error Counter
        gBaseErrCntr        : natural := 16#01800#;
        --! Base address TX NMT Queue
        gBaseTxNmtQ         : natural := 16#02800#;
        --! Base address TX Generic Queue
        gBaseTxGenQ         : natural := 16#03800#;
        --! Base address TX SyncRequest Queue
        gBaseTxSynQ         : natural := 16#04800#;
        --! Base address TX Virtual Ethernet Queue
        gBaseTxVetQ         : natural := 16#05800#;
        --! Base address RX Virtual Ethernet Queue
        gBaseRxVetQ         : natural := 16#06800#;
        --! Base address Kernel-to-User Queue
        gBaseK2UQ           : natural := 16#07000#;
        --! Base address User-to-Kernel Queue
        gBaseU2KQ           : natural := 16#09000#;
        --! Base address Tpdo
        gBasePdo            : natural := 16#0B000#;
        --! Base address Reserved (-1 = high address of Pdo)
        gBaseRes            : natural := 16#0E000#;
        --! Host address width
        gHostAddrWidth      : natural := 16
    );
    port (
        --! Clock Source input
        csi_c0_clock                    : in std_logic;
        --! Reset Source input
        rsi_r0_reset                    : in std_logic;
        -- Avalon Memory Mapped Slave for Host
        --! Avalon-MM slave host address
        avs_host_address                : in std_logic_vector(gHostAddrWidth-1 downto 2);
        --! Avalon-MM slave host byteenable
        avs_host_byteenable             : in std_logic_vector(3 downto 0);
        --! Avalon-MM slave host read
        avs_host_read                   : in std_logic;
        --! Avalon-MM slave host readdata
        avs_host_readdata               : out std_logic_vector(31 downto 0);
        --! Avalon-MM slave host write
        avs_host_write                  : in std_logic;
        --! Avalon-MM slave host writedata
        avs_host_writedata              : in std_logic_vector(31 downto 0);
        --! Avalon-MM slave host waitrequest
        avs_host_waitrequest            : out std_logic;
        -- Avalon Memory Mapped Slave for PCP
        --! Avalon-MM slave pcp address
        avs_pcp_address                 : in std_logic_vector(10 downto 2);
        --! Avalon-MM slave pcp byteenable
        avs_pcp_byteenable              : in std_logic_vector(3 downto 0);
        --! Avalon-MM slave pcp read
        avs_pcp_read                    : in std_logic;
        --! Avalon-MM slave pcp readdata
        avs_pcp_readdata                : out std_logic_vector(31 downto 0);
        --! Avalon-MM slave pcp write
        avs_pcp_write                   : in std_logic;
        --! Avalon-MM slave pcp writedata
        avs_pcp_writedata               : in std_logic_vector(31 downto 0);
        --! Avalon-MM slave pcp waitrequest
        avs_pcp_waitrequest             : out std_logic;
        -- Avalon Memory Mapped Master for Host via Magic Bridge
        --! Avalon-MM master hostBridge address
        avm_hostBridge_address          : out std_logic_vector(29 downto 0);
        --! Avalon-MM master hostBridge byteenable
        avm_hostBridge_byteenable       : out std_logic_vector(3 downto 0);
        --! Avalon-MM master hostBridge read
        avm_hostBridge_read             : out std_logic;
        --! Avalon-MM master hostBridge readdata
        avm_hostBridge_readdata         : in std_logic_vector(31 downto 0);
        --! Avalon-MM master hostBridge write
        avm_hostBridge_write            : out std_logic;
        --! Avalon-MM master hostBridge writedata
        avm_hostBridge_writedata        : out std_logic_vector(31 downto 0);
        --! Avalon-MM master hostBridge waitrequest
        avm_hostBridge_waitrequest      : in std_logic;
        --! Interrupt receiver
        inr_irqSync_irq                 : in std_logic;
        --! Interrupt sender
        ins_irqOut_irq                  : out std_logic;
        --! External Sync Source
        coe_ExtSync_exsync              : in std_logic
    );
end alteraHostInterface;

architecture rtl of alteraHostInterface is
    --! The bridge translation lut is implemented in memory blocks to save logic resources.
    --! If no M9K shall be used, set this constant to 0.
    constant cBridgeUseMemBlock : natural := 1;
begin
    --! The host interface
    theHostInterface: entity work.hostInterface
    generic map (
        gVersionMajor          => gVersionMajor,
        gVersionMinor          => gVersionMinor,
        gVersionRevision       => gVersionRevision,
        gVersionCount          => gVersionCount,
        gBridgeUseMemBlock     => cBridgeUseMemBlock,
        gBaseDynBuf0           => gBaseDynBuf0,
        gBaseDynBuf1           => gBaseDynBuf1,
        gBaseErrCntr           => gBaseErrCntr,
        gBaseTxNmtQ            => gBaseTxNmtQ,
        gBaseTxGenQ            => gBaseTxGenQ,
        gBaseTxSynQ            => gBaseTxSynQ,
        gBaseTxVetQ            => gBaseTxVetQ,
        gBaseRxVetQ            => gBaseRxVetQ,
        gBaseK2UQ              => gBaseK2UQ,
        gBaseU2KQ              => gBaseU2KQ,
        gBasePdo               => gBasePdo,
        gBaseRes               => gBaseRes,
        gHostAddrWidth         => gHostAddrWidth
    )
    port map (
        iClk                   => csi_c0_clock,
        iRst                   => rsi_r0_reset,
        iHostAddress           => avs_host_address,
        iHostByteenable        => avs_host_byteenable,
        iHostRead              => avs_host_read,
        oHostReaddata          => avs_host_readdata,
        iHostWrite             => avs_host_write,
        iHostWritedata         => avs_host_writedata,
        oHostWaitrequest       => avs_host_waitrequest,
        iPcpAddress            => avs_pcp_address,
        iPcpByteenable         => avs_pcp_byteenable,
        iPcpRead               => avs_pcp_read,
        oPcpReaddata           => avs_pcp_readdata,
        iPcpWrite              => avs_pcp_write,
        iPcpWritedata          => avs_pcp_writedata,
        oPcpWaitrequest        => avs_pcp_waitrequest,
        oHostBridgeAddress     => avm_hostBridge_address,
        oHostBridgeByteenable  => avm_hostBridge_byteenable,
        oHostBridgeRead        => avm_hostBridge_read,
        iHostBridgeReaddata    => avm_hostBridge_readdata,
        oHostBridgeWrite       => avm_hostBridge_write,
        oHostBridgeWritedata   => avm_hostBridge_writedata,
        iHostBridgeWaitrequest => avm_hostBridge_waitrequest,
        iIrqIntSync            => inr_irqSync_irq,
        iIrqExtSync            => coe_ExtSync_exsync,
        oIrq                   => ins_irqOut_irq
    );
end rtl;
