-------------------------------------------------------------------------------
--! @file asyncFifo-e.vhd
--
--! @brief The asynchronous FIFO entity.
--!
--! @details This is the asynchronous FIFO interface description, for a dual
--!          clocked FIFO component.
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

entity asyncFifo is
    generic (
        --! Data width of write and read port
        gDataWidth  : natural := 8;
        --! Number of words stored in fifo
        gWordSize   : natural := 64;
        --! Number of synchronizer stages
        gSyncStages : natural := 2;
        --! Select memory resource ("ON" = memory / "OFF" = registers)
        gMemRes     : string := "ON"
    );
    port (
        --! Asynchronous clear (FIXME: Convert this to reset, and add wr-/rd-clear inputs)
        iAclr       : in std_logic;
        --! Write Clk
        iWrClk      : in std_logic;
        --! Write Request
        iWrReq      : in std_logic;
        --! Write Data
        iWrData     : in std_logic_vector(gDataWidth-1 downto 0);
        --! Write Empty Flag
        oWrEmpty    : out std_logic;
        --! Write Full Flag
        oWrFull     : out std_logic;
        --! Write used words
        oWrUsedw    : out std_logic_vector(logDualis(gWordSize)-1 downto 0);
        --! Read clk
        iRdClk      : in std_logic;
        --! Read Request
        iRdReq      : in std_logic;
        --! Read Data
        oRdData     : out std_logic_vector(gDataWidth-1 downto 0);
        --! Read Empty Flag
        oRdEmpty    : out std_logic;
        --! Read Full Flag
        oRdFull     : out std_logic;
        --! Read used words
        oRdUsedw    : out std_logic_vector(logDualis(gWordSize)-1 downto 0)
    );
end entity asyncFifo;
