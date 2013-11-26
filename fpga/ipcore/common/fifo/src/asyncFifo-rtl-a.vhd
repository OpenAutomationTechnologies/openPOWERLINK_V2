-------------------------------------------------------------------------------
--! @file asyncFifo-rtl-a.vhd
--
--! @brief The asynchronous Fifo architecture.
--
--! @details This is a generic dual clocked FIFO using the dpRam component as
--!          memory.
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

library work;
--! use global library
use work.global.all;

architecture rtl of asyncFifo is
    --! Address width
    constant cAddrWidth         : natural := logDualis(gWordSize);

    --! Type for DPRAM port commons
    type tDpramPortCommon is record
        clk     : std_logic;
        enable  : std_logic;
        address : std_logic_vector(cAddrWidth-1 downto 0);
    end record;

    --! Type for DPRAM port assignment
    type tDpramPort is record
        wrPort      : tDpramPortCommon;
        rdPort      : tDpramPortCommon;
        write       : std_logic;
        writedata   : std_logic_vector(gDataWidth-1 downto 0);
        readdata    : std_logic_vector(gDataWidth-1 downto 0);
    end record;

    --! Type for control port
    type tControlPort is record
        clk             : std_logic;
        rst             : std_logic;
        request         : std_logic;
        otherPointer    : std_logic_vector(cAddrWidth downto 0);
        empty           : std_logic;
        full            : std_logic;
        pointer         : std_logic_vector(cAddrWidth downto 0);
        address         : std_logic_vector(cAddrWidth-1 downto 0);
        usedWord        : std_logic_vector(cAddrWidth-1 downto 0);
    end record;

    --! Type for pointer synchronizers
    type tPointerSyncPort is record
        clk     : std_logic;
        rst     : std_logic;
        din     : std_logic_vector(cAddrWidth downto 0);
        dout    : std_logic_vector(cAddrWidth downto 0);
    end record;

    --! DPRAM instance
    signal inst_dpram       : tDpramPort;
    --! Write controller instance
    signal inst_writeCtrl   : tControlPort;
    --! Read controller instance
    signal inst_readCtrl    : tControlPort;
    --! Write pointer synchronizer instance
    signal inst_writeSync   : tPointerSyncPort;
    --! Read pointer synchronizer instance
    signal inst_readSync    : tPointerSyncPort;
begin
    assert (gMemRes = "ON")
    report "This FIFO implementation only supports memory resources!"
    severity warning;

    ---------------------------------------------------------------------------
    -- Assign Outputs
    ---------------------------------------------------------------------------
    -- Write port
    oWrEmpty    <= inst_writeCtrl.empty;
    oWrFull     <= inst_writeCtrl.full;
    oWrUsedw    <= inst_writeCtrl.usedWord;
    -- Read port
    oRdEmpty    <= inst_readCtrl.empty;
    oRdFull     <= inst_readCtrl.full;
    oRdUsedw    <= inst_readCtrl.usedWord;
    oRdData     <= inst_dpram.readdata;
    ---------------------------------------------------------------------------
    -- Map DPRAM instance
    ---------------------------------------------------------------------------
    -- Write port
    inst_dpram.wrPort.clk       <= iWrClk;
    inst_dpram.wrPort.enable    <= inst_writeCtrl.request;
    inst_dpram.write            <= inst_writeCtrl.request;
    inst_dpram.wrPort.address   <= inst_writeCtrl.address;
    inst_dpram.writedata        <= iWrData;
    -- Read port
    inst_dpram.rdPort.clk       <= iRdClk;
    inst_dpram.rdPort.enable    <= iRdReq;
    inst_dpram.rdPort.address   <= inst_readCtrl.address;

    ---------------------------------------------------------------------------
    -- Map Write and Read controller instance
    ---------------------------------------------------------------------------
    inst_readCtrl.clk           <= iRdClk;
    inst_readCtrl.rst           <= iAclr;
    inst_readCtrl.request       <= iRdReq;
    inst_readCtrl.otherPointer  <= inst_writeSync.dout;

    inst_writeCtrl.clk          <= iWrClk;
    inst_writeCtrl.rst          <= iAclr;
    inst_writeCtrl.request      <= iWrReq and not inst_writeCtrl.full;
    inst_writeCtrl.otherPointer <= inst_readSync.dout;

    ---------------------------------------------------------------------------
    -- Map pointer synchronizers
    ---------------------------------------------------------------------------
    inst_readSync.rst   <= iAclr;
    inst_readSync.clk   <= iWrClk; -- synchronize read pointer to write clock
    inst_readSync.din   <= inst_readCtrl.pointer;

    inst_writeSync.rst  <= iAclr;
    inst_writeSync.clk  <= iRdClk; -- synchronize write pointer to read clock
    inst_writeSync.din  <= inst_writeCtrl.pointer;

    ---------------------------------------------------------------------------
    -- Instances
    ---------------------------------------------------------------------------
    --! This is the FIFO read controller.
    FIFO_READ_CONTROL : entity work.fifoRead
        generic map (
            gAddrWidth  => cAddrWidth
        )
        port map (
            iClk        => inst_readCtrl.clk,
            iRst        => inst_readCtrl.rst,
            iRead       => inst_readCtrl.request,
            iWrPointer  => inst_readCtrl.otherPointer,
            oEmpty      => inst_readCtrl.empty,
            oFull       => inst_readCtrl.full,
            oPointer    => inst_readCtrl.pointer,
            oAddress    => inst_readCtrl.address,
            oUsedWord   => inst_readCtrl.usedWord
        );

    --! This is the FIFO write controller.
    FIFO_WRITE_CONTROL : entity work.fifoWrite
        generic map (
            gAddrWidth  => cAddrWidth
        )
        port map (
            iClk        => inst_writeCtrl.clk,
            iRst        => inst_writeCtrl.rst,
            iWrite      => inst_writeCtrl.request,
            iRdPointer  => inst_writeCtrl.otherPointer,
            oEmpty      => inst_writeCtrl.empty,
            oFull       => inst_writeCtrl.full,
            oPointer    => inst_writeCtrl.pointer,
            oAddress    => inst_writeCtrl.address,
            oUsedWord   => inst_writeCtrl.usedWord
        );

    --! This is the FIFO buffer.
    FIFO_BUFFER : entity work.dpRamSplxNbe
        generic map (
            gWordWidth          => gDataWidth,
            gNumberOfWords      => gWordSize,
            gInitFile           => "UNUSED"
        )
        port map (
            iClk_A          => inst_dpram.wrPort.clk,
            iEnable_A       => inst_dpram.wrPort.enable,
            iWriteEnable_A  => inst_dpram.write,
            iAddress_A      => inst_dpram.wrPort.address,
            iWritedata_A    => inst_dpram.writedata,
            iClk_B          => inst_dpram.rdPort.clk,
            iEnable_B       => inst_dpram.rdPort.enable,
            iAddress_B      => inst_dpram.rdPort.address,
            oReaddata_B     => inst_dpram.readdata
        );

    --! This generate block instantiates multiple synchrinizers to transfer
    --! the write and read pointers to the opposite clock domains.
    GEN_POINTERSYNC : for i in cAddrWidth downto 0 generate
        WRITESYNC : entity work.synchronizer
            generic map (
                gStages => gSyncStages,
                gInit   => cInactivated
            )
            port map (
                iArst   => inst_writeSync.rst,
                iClk    => inst_writeSync.clk,
                iAsync  => inst_writeSync.din(i),
                oSync   => inst_writeSync.dout(i)
            );

        READSYNC : entity work.synchronizer
            generic map (
                gStages => gSyncStages,
                gInit   => cInactivated
            )
            port map (
                iArst   => inst_readSync.rst,
                iClk    => inst_readSync.clk,
                iAsync  => inst_readSync.din(i),
                oSync   => inst_readSync.dout(i)
            );
    end generate GEN_POINTERSYNC;
end rtl;
