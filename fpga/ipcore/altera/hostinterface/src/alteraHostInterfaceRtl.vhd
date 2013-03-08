-------------------------------------------------------------------------------
--! @file alteraHostInterface.vhd
--
--! @brief toplevel of host interface for Altera FPGA
--
--! @details This toplevel interfaces to Altera specific implementation.
--
-------------------------------------------------------------------------------
--
--    (c) B&R, 2012
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

entity alteraHostInterface is
    generic (
        --! Version major
        gVersionMajor : natural := 16#FF#;
        --! Version minor
        gVersionMinor : natural := 16#FF#;
        --! Version revision
        gVersionRevision : natural := 16#FF#;
        --! Version count
        gVersionCount : natural := 0;
        -- Base address mapping
        --! Base address Dynamic Buffer 0
        gBaseDynBuf0 : natural := 16#00800#;
        --! Base address Dynamic Buffer 1
        gBaseDynBuf1 : natural := 16#01000#;
        --! Base address Error Counter
        gBaseErrCntr : natural := 16#01800#;
        --! Base address TX NMT Queue
        gBaseTxNmtQ : natural := 16#02800#;
        --! Base address TX Generic Queue
        gBaseTxGenQ : natural := 16#03800#;
        --! Base address TX SyncRequest Queue
        gBaseTxSynQ : natural := 16#04800#;
        --! Base address TX Virtual Ethernet Queue
        gBaseTxVetQ : natural := 16#05800#;
        --! Base address RX Virtual Ethernet Queue
        gBaseRxVetQ : natural := 16#06800#;
        --! Base address Kernel-to-User Queue
        gBaseK2UQ : natural := 16#07000#;
        --! Base address User-to-Kernel Queue
        gBaseU2KQ : natural := 16#09000#;
        --! Base address Tpdo
        gBaseTpdo : natural := 16#0B000#;
        --! Base address Rpdo
        gBaseRpdo : natural := 16#0E000#;
        --! Base address Reserved (-1 = high address of Rpdo)
        gBaseRes : natural := 16#14000#;
        --! Select Host Interface Type (0 = Avalon, 1 = Parallel)
        gHostIfType : natural := 0;
        --! Data width of parallel interface (16/32)
        gParallelDataWidth : natural := 16;
        --! Address and Data bus are multiplexed (0 = FALSE, otherwise = TRUE)
        gParallelMultiplex : natural := 0
        );
    port (
        --! Clock Source input
        csi_c0_clock : in std_logic;
        --! Reset Source input
        rsi_r0_reset : in std_logic;
        -- Avalon Memory Mapped Slave for Host
        --! Avalon-MM slave host address
        avs_host_address : in std_logic_vector(16 downto 2);
        --! Avalon-MM slave host byteenable
        avs_host_byteenable : in std_logic_vector(3 downto 0);
        --! Avalon-MM slave host read
        avs_host_read : in std_logic;
        --! Avalon-MM slave host readdata
        avs_host_readdata : out std_logic_vector(31 downto 0);
        --! Avalon-MM slave host write
        avs_host_write : in std_logic;
        --! Avalon-MM slave host writedata
        avs_host_writedata : in std_logic_vector(31 downto 0);
        --! Avalon-MM slave host waitrequest
        avs_host_waitrequest : out std_logic;
        -- Avalon Memory Mapped Slave for PCP
        --! Avalon-MM slave pcp address
        avs_pcp_address : in std_logic_vector(10 downto 2);
        --! Avalon-MM slave pcp byteenable
        avs_pcp_byteenable : in std_logic_vector(3 downto 0);
        --! Avalon-MM slave pcp read
        avs_pcp_read : in std_logic;
        --! Avalon-MM slave pcp readdata
        avs_pcp_readdata : out std_logic_vector(31 downto 0);
        --! Avalon-MM slave pcp write
        avs_pcp_write : in std_logic;
        --! Avalon-MM slave pcp writedata
        avs_pcp_writedata : in std_logic_vector(31 downto 0);
        --! Avalon-MM slave pcp waitrequest
        avs_pcp_waitrequest : out std_logic;
        -- Avalon Memory Mapped Master for Host via Magic Bridge
        --! Avalon-MM master hostBridge address
        avm_hostBridge_address : out std_logic_vector(29 downto 0);
        --! Avalon-MM master hostBridge byteenable
        avm_hostBridge_byteenable : out std_logic_vector(3 downto 0);
        --! Avalon-MM master hostBridge read
        avm_hostBridge_read : out std_logic;
        --! Avalon-MM master hostBridge readdata
        avm_hostBridge_readdata : in std_logic_vector(31 downto 0);
        --! Avalon-MM master hostBridge write
        avm_hostBridge_write : out std_logic;
        --! Avalon-MM master hostBridge writedata
        avm_hostBridge_writedata : out std_logic_vector(31 downto 0);
        --! Avalon-MM master hostBridge waitrequest
        avm_hostBridge_waitrequest : in std_logic;
        --! Interrupt receiver
        inr_irqSync_irq : in std_logic;
        --! Interrupt sender
        ins_irqOut_irq : out std_logic;
        --! External Sync Source
        coe_ExtSync_exsync : in std_logic;
        --! Node Id
        coe_NodeId_nodeid : in std_logic_vector(7 downto 0);
        --! POWERLINK Error LED
        coe_PlkLed_lederr : out std_logic;
        --! POWERLINK Status LED
        coe_PlkLed_ledst : out std_logic;
        -- Parallel Host Interface
        --! Chipselect
        coe_parHost_chipselect : in std_logic;
        --! Read strobe
        coe_parHost_read : in std_logic;
        --! Write strobe
        coe_parHost_write : in std_logic;
        --! Address Latch enable (Multiplexed only)
        coe_parHost_addressLatchEnable : in std_logic;
        --! High active Acknowledge
        coe_parHost_acknowledge : out std_logic;
        --! Byteenables
        coe_parHost_byteenable : in std_logic_vector(gParallelDataWidth/8-1 downto 0);
        --! Address bus (Demultiplexed, word-address)
        coe_parHost_address : in std_logic_vector(15 downto 0);
        --! Data bus (Demultiplexed)
        coe_parHost_data : inout std_logic_vector(gParallelDataWidth-1 downto 0);
        --! Address/Data bus (Multiplexed, word-address))
        coe_parHost_addressData : inout std_logic_vector(gParallelDataWidth-1 downto 0)
        );
end alteraHostInterface;

architecture rtl of alteraHostInterface is

    signal host_address : std_logic_vector(16 downto 2);
    signal host_byteenable : std_logic_vector(3 downto 0);
    signal host_read : std_logic;
    signal host_readdata : std_logic_vector(31 downto 0);
    signal host_write : std_logic;
    signal host_writedata : std_logic_vector(31 downto 0);
    signal host_waitrequest : std_logic;

begin

    --! Assign the host side to Avalon
    genAvalon : if gHostIfType = 0 generate
    begin
        host_address <= avs_host_address;
        host_byteenable <= avs_host_byteenable;
        host_read <= avs_host_read;
        avs_host_readdata <= host_readdata;
        host_write <= avs_host_write;
        host_writedata <= avs_host_writedata;
        avs_host_waitrequest <= host_waitrequest;
    end generate;

    --! Assign the host side to Parallel
    genParallel : if gHostIfType = 1 generate
        signal hostData_i : std_logic_vector(gParallelDataWidth-1 downto 0);
        signal hostData_o : std_logic_vector(gParallelDataWidth-1 downto 0);
        signal hostData_en : std_logic;
        signal hostAddressData_i : std_logic_vector(gParallelDataWidth-1 downto 0);
        signal hostAddressData_o : std_logic_vector(gParallelDataWidth-1 downto 0);
        signal hostAddressData_en : std_logic;
    begin
        -- not used signals are set to inactive
        avs_host_readdata <= (others => cInactivated);
        avs_host_waitrequest <= cInactivated;

        theParallelInterface : entity work.parallelInterface
            generic map (
                gDataWidth => gParallelDataWidth,
                gMultiplex => gParallelMultiplex
            )
            port map (
                iParHostChipselect => coe_parHost_chipselect,
                iParHostRead => coe_parHost_read,
                iParHostWrite => coe_parHost_write,
                iParHostAddressLatchEnable => coe_parHost_addressLatchEnable,
                oParHostAcknowledge => coe_parHost_acknowledge,
                iParHostByteenable => coe_parHost_byteenable,
                iParHostAddress => coe_parHost_address,
                oParHostData => hostData_o,
                iParHostData => hostData_i,
                oParHostDataEnable => hostData_en,
                oParHostAddressData => hostAddressData_o,
                iParHostAddressData => hostAddressData_i,
                oParHostAddressDataEnable => hostAddressData_en,
                iClk => csi_c0_clock,
                iRst => rsi_r0_reset,
                oHostAddress => host_address,
                oHostByteenable => host_byteenable,
                oHostRead => host_read,
                iHostReaddata => host_readdata,
                oHostWrite => host_write,
                oHostWritedata => host_writedata,
                iHostWaitrequest => host_waitrequest
            );

        -- tri-state buffers
        coe_parHost_data <= hostData_o when hostData_en = cActivated else
                            (others => 'Z');

        hostData_i <= coe_parHost_data;

        coe_parHost_addressData <= hostAddressData_o when hostAddressData_en = cActivated else
                                   (others => 'Z');

        hostAddressData_i <= coe_parHost_addressData;

    end generate;

    --! The host interface
    theHostInterface: entity work.hostInterface
    generic map(
        gVersionMajor          => gVersionMajor,
        gVersionMinor          => gVersionMinor,
        gVersionRevision       => gVersionRevision,
        gVersionCount          => gVersionCount,
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
        gBaseTpdo              => gBaseTpdo,
        gBaseRpdo              => gBaseRpdo,
        gBaseRes               => gBaseRes
    )
    port map(
        iClk                   => csi_c0_clock,
        iRst                   => rsi_r0_reset,
        iHostAddress           => host_address,
        iHostByteenable        => host_byteenable,
        iHostRead              => host_read,
        oHostReaddata          => host_readdata,
        iHostWrite             => host_write,
        iHostWritedata         => host_writedata,
        oHostWaitrequest       => host_waitrequest,
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
        oIrq                   => ins_irqOut_irq,
        iNodeId                => coe_NodeId_nodeid,
        oPlkLedError           => coe_PlkLed_lederr,
        oPlkLedStatus          => coe_PlkLed_ledst
    );

end rtl;
