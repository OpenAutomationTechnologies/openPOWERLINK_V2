-------------------------------------------------------------------------------
--! @file asyncFifo-syn-a.vhd
--
--! @brief The asynchronous Fifo architecture for Altera
--
--! @details This is a dual clock fifo generated in Megawizard!
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

--! use altera_mf library
library altera_mf;
use altera_mf.altera_mf_components.all;

architecture syn of asyncFifo is
begin

    theAlteraDcFifo : dcfifo
        generic map (
            intended_device_family  => "Cyclone IV E",
            lpm_width               => gDataWidth,
            lpm_widthu              => logDualis(gWordSize),
            lpm_numwords            => gWordSize,
            lpm_showahead           => "OFF",
            lpm_type                => "DCFIFO",
            overflow_checking       => "ON",
            underflow_checking      => "ON",
            delay_rdusedw           => 1,
            delay_wrusedw           => 1,
            add_usedw_msb_bit       => "OFF",
            rdsync_delaypipe        => gSyncStages+2,
            wrsync_delaypipe        => gSyncStages+2,
            use_eab                 => gMemRes,
            write_aclr_synch        => "ON",
            read_aclr_synch         => "ON",
            clocks_are_synchronized => "FALSE",
            add_ram_output_register => "ON"
        )
        port map (
            aclr        => iAclr,
            data        => iWrData,
            q           => oRdData,
            rdclk       => iRdClk,
            rdempty     => oRdEmpty,
            rdfull      => oRdFull,
            rdreq       => iRdReq,
            rdusedw     => oRdUsedw,
            wrclk       => iWrClk,
            wrempty     => oWrEmpty,
            wrfull      => oWrFull,
            wrreq       => iWrReq,
            wrusedw     => oWrUsedw
        );

end architecture syn;
