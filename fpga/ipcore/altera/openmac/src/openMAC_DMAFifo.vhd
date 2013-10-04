-------------------------------------------------------------------------------
-- OpenMAC DMA FIFO
--
--    Copyright (C) 2011 B&R
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

library altera_mf;
use altera_mf.all;

entity openMAC_DMAfifo is
    generic (
        fifo_data_width_g        : natural := 16;
        fifo_word_size_g        : natural := 32;
        fifo_word_size_log2_g    : natural := 5
    );
    port
    (
        aclr        : in std_logic;
        rd_clk        : in std_logic;
        wr_clk        : in std_logic;
        --read port
        rd_req        : in std_logic;
        rd_data        : out std_logic_vector(fifo_data_width_g-1 downto 0);
        rd_empty    : out std_logic;
        rd_full        : out std_logic;
        rd_usedw    : out std_logic_vector(fifo_word_size_log2_g-1 downto 0);
        --write port
        wr_req        : in std_logic;
        wr_data        : in std_logic_vector(fifo_data_width_g-1 downto 0);
        wr_empty    : out std_logic;
        wr_full        : out std_logic;
        wr_usedw    : out std_logic_vector(fifo_word_size_log2_g-1 downto 0)
    );
end openMAC_DMAfifo;


architecture struct of openMAC_DMAfifo is

    component dcfifo
        generic (
            lpm_width                 : natural; --width of data and q ports (input/output)
            lpm_widthu                 : natural; --width of wrusedw and rdusedw
            lpm_numwords             : natural; --depth of fifo
            lpm_showahead             : string; --fifo showahead off/on (rdreq works as req/ack)
            lpm_type                 : string; --SCFIFO or DCFIFO (single/dual clocked)
            overflow_checking         : string; --protection circuit for wrreq
            underflow_checking         : string; --protection circuit for rdreq
            rdsync_delaypipe         : natural; --number of sync from wr to rd
            wrsync_delaypipe         : natural; --number of sync from rd to wr
            use_eab                 : string; --construct fifo as LE/RAM (off/on)
            write_aclr_synch         : string; --sync async. clear to wr clk (avoids race cond.)
            intended_device_family    : string --specifies the intended device for functional simulation
        );
        port (
            wrclk    : in std_logic; --clock for wr port
            rdclk    : in std_logic; --clock for rd port
            data    : in std_logic_vector(fifo_data_width_g-1 downto 0); --data to be written
            wrreq    : in std_logic; --write request
            rdreq    : in std_logic; --read request
            aclr    : in std_logic; --asynchronous clear fifo
            q        : out std_logic_vector(fifo_data_width_g-1 downto 0); --read data
            wrfull    : out std_logic; --fifo is full on wr port
            rdfull    : out std_logic; --fifo is full on rd port
            wrempty    : out std_logic; --fifo is empty on wr port
            rdempty    : out std_logic; --fifo is empty on rd port
            wrusedw    : out std_logic_vector(fifo_word_size_log2_g-1 downto 0); --number of words stored on wr port
            rdusedw    : out std_logic_vector(fifo_word_size_log2_g-1 downto 0) --number of words stored on rd port
        );
    end component;

    constant fifo_useRam_c         : string := "ON";
    constant fifo_words_c         : natural := fifo_word_size_g; --e.g. 32
    constant fifo_usedw_c         : natural := fifo_word_size_log2_g; --e.g. log2(32) = 5
    --constant fifo_rd_usedw_c : natural := 5;
    --constant fifo_wr_usedw_c : natural := 5;
    constant fifo_data_width_c     : natural := fifo_data_width_g;
    --constant fifo_rd_data_width_c : natural := 16;
    --constant fifo_wr_data_width_c : natural := 16;

begin

    dcfifo_inst : dcfifo
    generic map (
        lpm_width                 => fifo_data_width_c, --width of data and q ports (input/output)
        lpm_widthu                 => fifo_usedw_c, --width of wrusedw and rdusedw
        lpm_numwords             => fifo_words_c, --depth of fifo
        lpm_showahead             => "OFF", --fifo showahead off/on (rdreq works as req/ack)
        lpm_type                 => "DCFIFO", --SCFIFO or DCFIFO (single/dual clocked)
        overflow_checking         => "ON", --protection circuit for wrreq
        underflow_checking         => "ON", --protection circuit for rdreq
        rdsync_delaypipe         => 4, --number of sync from wr to rd
        wrsync_delaypipe         => 4, --number of sync from rd to wr
        use_eab                 => fifo_useRam_c, --construct fifo as LE/RAM (off/on)
        write_aclr_synch         => "ON", --sync async. clear to wr clk (avoids race cond.)
        intended_device_family    => "Cyclone IV" --specifies the intended device for functional simulation
    )
    port map (
        wrclk     => wr_clk, --clock for wr port
        rdclk     => rd_clk, --clock for rd port
        data     => wr_data, --data to be written
        wrreq     => wr_req, --write request
        rdreq     => rd_req, --read request
        aclr     => aclr, --asynchronous clear fifo
        q         => rd_data, --read data
        wrfull     => wr_full, --fifo is full on wr port
        rdfull     => rd_full, --fifo is full on rd port
        wrempty => wr_empty, --fifo is empty on wr port
        rdempty => rd_empty, --fifo is empty on rd port
        wrusedw => wr_usedw, --number of words stored on wr port
        rdusedw => rd_usedw --number of words stored on rd port
    );

end struct;
