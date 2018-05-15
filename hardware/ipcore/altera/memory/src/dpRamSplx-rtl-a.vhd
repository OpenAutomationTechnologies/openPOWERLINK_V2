--! @file dpRamSplx-rtl-a.vhd
--
--! @brief Simplex Dual Port Ram Register Transfer Level Architecture
--
--! @details This is the Simplex DPRAM intended for synthesis on Altera
--!          platforms only.
--!          Timing as follows [clk-cycles]: write=0 / read=1
--
-------------------------------------------------------------------------------
-- Architecture : rtl
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

architecture rtl of dpRamSplx is
begin
    altsyncram_component : altsyncram
        generic map (
            operation_mode          => "DUAL_PORT",
            intended_device_family  => "Cyclone IV",
            init_file               => gInitFile,
            numwords_a              => gNumberOfWordsA,
            numwords_b              => gNumberOfWordsB,
            widthad_a               => logDualis(gNumberOfWordsA),
            widthad_b               => logDualis(gNumberOfWordsB),
            width_a                 => gWordWidthA,
            width_b                 => gWordWidthB,
            width_byteena_a         => gByteenableWidthA,
            width_byteena_b         => gByteenableWidthA
        )
        port map (
            clock0      => iClk_A,
            clocken0    => iEnable_A,
            wren_a      => iWriteEnable_A,
            address_a   => iAddress_A,
            byteena_a   => iByteenable_A,
            data_a      => iWritedata_A,
            clock1      => iClk_B,
            clocken1    => iEnable_B,
            address_b   => iAddress_B,
            q_b         => oReaddata_B
        );
end architecture rtl;
