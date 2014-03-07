-------------------------------------------------------------------------------
--! @file ipifMasterHandler-rtl-ea.vhd
--
--! @brief IPIF Master handler
--
--! @details This is the IPIF master handler converting generic master interface
--!          to IPIF.
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
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;

--! Common library
library libcommon;
--! Use common library global package
use libcommon.global.all;

entity ipifMasterHandler is
    generic (
        --! Master address width
        gMasterAddrWidth        : natural := 31;
        --! Master burst count width
        gMasterBurstCountWidth  : natural := 4;
        --! IPIF address width
        gIpifAddrWidth          : natural := 32;
        --! IPIF length width
        gIpifLength             : natural := 12

    );
    port (
        --TODO: Add doxygen comments!
        -- Common clock and reset
        iRst                    : in    std_logic;
        iClk                    : in    std_logic;
        -- IPIF Master
        iIpif_cmdAck            : in    std_logic;
        iIpif_cmplt             : in    std_logic;
        iIpif_error             : in    std_logic; --FIXME: Unused input
        iIpif_rearbitrate       : in    std_logic; --FIXME: Unused input
        iIpif_cmdTimeout        : in    std_logic; --FIXME: Unused input
        oIpif_type              : out   std_logic;
        oIpif_addr              : out   std_logic_vector(gIpifAddrWidth-1 downto 0);
        oIpif_length            : out   std_logic_vector(gIpifLength-1 downto 0);
        oIpif_be                : out   std_logic_vector(3 downto 0);
        oIpif_lock              : out   std_logic;
        oIpif_reset             : out   std_logic;
        iIpif_rdData            : in    std_logic_vector(31 downto 0);
        iIpif_rdRem             : in    std_logic_vector(3 downto 0); --FIXME: Unused input
        oIpif_rdReq             : out   std_logic;
        inIpif_rdSof            : in    std_logic;
        inIpif_rdEof            : in    std_logic;
        inIpif_rdSrcRdy         : in    std_logic;
        inIpif_rdSrcDsc         : in    std_logic; --FIXME: Unused input
        onIpif_rdDstRdy         : out   std_logic;
        onIpif_rdDstDsc         : out   std_logic;
        oIpif_wrData            : out   std_logic_vector(31 downto 0);
        oIpif_wrRem             : out   std_logic_vector(3 downto 0);
        oIpif_wrReq             : out   std_logic;
        onIpif_wrSof            : out   std_logic;
        onIpif_wrEof            : out   std_logic;
        onIpif_wrSrcRdy         : out   std_logic;
        onIpif_wrSrcDsc         : out   std_logic;
        inIpif_wrDstRdy         : in    std_logic;
        inIpif_wrDstDsc         : in    std_logic; --FIXME: Unused input
        -- Generic master interface
        iMasterRead             : in    std_logic;
        iMasterWrite            : in    std_logic;
        iMasterAddress          : in    std_logic_vector(gMasterAddrWidth-1 downto 0);
        iMasterWritedata        : in    std_logic_vector(31 downto 0);
        iMasterBurstcount       : in    std_logic_vector(gMasterBurstCountWidth-1 downto 0);
        iMasterBurstcounter     : in    std_logic_vector(gMasterBurstCountWidth-1 downto 0);
        oMasterReaddata         : out   std_logic_vector(31 downto 0);
        oMasterWaitrequest      : out   std_logic;
        oMasterReaddatavalid    : out   std_logic
    );
end ipifMasterHandler;

architecture rtl of ipifMasterHandler is
    --signals for requesting transfers
    signal masterWrite          : std_logic;
    signal masterRead           : std_logic;
    signal nMasterEnable         : std_logic;
    signal masterWrite_l        : std_logic;
    signal masterRead_l         : std_logic;
    signal masterWrite_rise     : std_logic;
    signal masterRead_rise      : std_logic;
    signal masterWrite_fall     : std_logic;
    signal masterRead_fall      : std_logic;
    signal ipifWriteReq_reg     : std_logic;
    signal ipifWriteReq_next    : std_logic;
    signal ipifReadReq_reg      : std_logic;
    signal ipifReadReq_next     : std_logic;
    signal ipif_rdDstRdy        : std_logic;

    --signals for the transfer
    type tTfState is (
        sIdle,
        sSof, sTf, sEof,
        sSEof, --start/end of frame (single beat)
        sWaitForCmplt
    );

    signal writeTf_reg  : tTfState;
    signal writeTf_next : tTfState;
    signal readTf       : tTfState;
begin
    masterWrite   <= iMasterWrite and not nMasterEnable;
    masterRead    <= iMasterRead and not nMasterEnable;

    --reserved
    oIpif_lock  <= cInactivated;
    oIpif_reset <= cInactivated;

    --delay some signals..
    del_proc : process(iClk, iRst)
    begin
        if iRst = cActivated then
            masterWrite_l   <= cInactivated;
            masterRead_l    <= cInactivated;
            nMasterEnable   <= cnActivated;
        elsif rising_edge(iClk) then
            masterWrite_l   <= masterWrite;
            masterRead_l    <= masterRead;

            if iIpif_cmplt = cActivated then
                nMasterEnable <= cnActivated;
            elsif masterWrite_fall = cActivated or masterRead_fall = cActivated then
                nMasterEnable <= cnInactivated; --write/read done, wait for Mst_Cmplt
            end if;
        end if;
    end process;

    --generate pulse if write/read is asserted
    masterWrite_rise    <=  cActivated when masterWrite_l = cInactivated and masterWrite = cActivated else
                            cInactivated;
    masterRead_rise     <=  cActivated when masterRead_l = cInactivated and masterRead = cActivated else
                            cInactivated;
    masterWrite_fall    <=  cActivated when masterWrite_l = cActivated and masterWrite = cInactivated else
                            cInactivated;
    masterRead_fall     <=  cActivated when masterRead_l = cActivated and masterRead = cInactivated else
                            cInactivated;

    --generate req qualifiers
    req_proc : process(iClk, iRst)
    begin
        if iRst = cActivated then
            ipifWriteReq_reg    <= cInactivated;
            ipifReadReq_reg     <= cInactivated;
            ipif_rdDstRdy       <= cInactivated;
        elsif rising_edge(iClk) then
            ipifWriteReq_reg    <= ipifWriteReq_next;
            ipifReadReq_reg     <= ipifReadReq_next;

            if masterRead = cActivated then
                ipif_rdDstRdy <= cActivated;
            elsif readTf = sEof and inIpif_rdSrcRdy = cnActivated then
                ipif_rdDstRdy <= cInactivated;
            end if;

        end if;
    end process;

    onIpif_rdDstRdy <=  not ipif_rdDstRdy;
    oIpif_rdReq     <=  ipifReadReq_reg;
    oIpif_wrReq     <=  ipifWriteReq_reg;
    oIpif_type      <=  cInactivated when iMasterBurstcount < 2 else --single beat
                        ipifReadReq_reg or ipifWriteReq_reg; --we are talking about bursts..

    ipifWriteReq_next   <=  cInactivated when ipifWriteReq_reg = cActivated and iIpif_cmdAck = cActivated else
                            cActivated when ipifWriteReq_reg = cInactivated and masterWrite_rise = cActivated else
                            ipifWriteReq_reg;
    ipifReadReq_next    <=  cInactivated when ipifReadReq_reg = cActivated and iIpif_cmdAck = cActivated else
                            cActivated when ipifReadReq_reg = cInactivated and masterRead_rise = cActivated else
                            ipifReadReq_reg;

    --assign address, byteenable and burst size
    comb_addrZeroPad : process(iMasterAddress)
    begin
        for i in oIpif_addr'range loop
            if i <= iMasterAddress'high then
                oIpif_addr(i) <= iMasterAddress(i);
            else
                oIpif_addr(i) <= cInactivated; --zero padding
            end if;
        end loop;
    end process;

    oIpif_be        <=  "1111";
    oIpif_length    <=  conv_std_logic_vector(conv_integer(iMasterBurstcount),
                            oIpif_length'length - 2) & "00"; -- dword x 4 = byte

    --write/read link
    wrd_proc : process(iClk, iRst)
    begin
        if iRst = cActivated then
            writeTf_reg <= sIdle;
        elsif rising_edge(iClk) then
            writeTf_reg <= writeTf_next;
        end if;
    end process;

    --generate fsm for write and read transfers
    writeTf_next    <=  sSEof           when writeTf_reg = sIdle and ipifWriteReq_next = cActivated and (iMasterBurstcount <= 1 or iMasterBurstcount'length = 1) else
                        sSof            when writeTf_reg = sIdle and ipifWriteReq_next = cActivated and iMasterBurstcount'length > 1 else
                        sEof            when writeTf_reg = sSof and inIpif_wrDstRdy = cnActivated and iMasterBurstcount = 2 and iMasterBurstcount'length > 1 else
                        sTf             when writeTf_reg = sSof and inIpif_wrDstRdy = cnActivated and iMasterBurstcount'length > 1 else
                        sEof            when writeTf_reg = sTf and iMasterBurstcounter <= 2 and inIpif_wrDstRdy = cnActivated and iMasterBurstcount'length > 1 else
                        sWaitForCmplt   when (writeTf_reg = sEof or writeTf_reg = sSEof) and inIpif_wrDstRdy = cnActivated else
                        sIdle           when writeTf_reg = sWaitForCmplt and iIpif_cmplt = cActivated else
                        writeTf_reg;

    readTf  <=  sSEof   when inIpif_rdSof = cnActivated and inIpif_rdEof = cnActivated else
                sSof    when inIpif_rdSof = cnActivated else
                sEof    when inIpif_rdEof = cnActivated else
                sTf     when inIpif_rdSrcRdy = cnActivated else
                sIdle;

    --set write qualifiers
    onIpif_wrSof    <=  cnActivated when writeTf_reg = sSof or writeTf_reg = sSEof else
                        cnInactivated;
    onIpif_wrEof    <=  cnActivated when writeTf_reg = sEof or writeTf_reg = sSEof else
                        cnInactivated;
    onIpif_wrSrcRdy <=  cnActivated when writeTf_reg /= sIdle and writeTf_reg /= sWaitForCmplt else
                        cnInactivated;
    onIpif_wrSrcDsc <=  cnInactivated; --no support
    oIpif_wrRem     <=  (others => cInactivated); --no support

    --set read qualifiers
    onIpif_rdDstDsc <= cnInactivated; --no support

    --connect ipif with generic master
    oMasterWaitrequest  <=  not iMasterWrite    when inIpif_wrDstRdy = cnActivated else
                            not iMasterRead     when ipifReadReq_reg = cActivated and iIpif_cmdAck = cActivated else cActivated;

    oMasterReaddatavalid    <= not inIpif_rdSrcRdy;
    oIpif_wrData            <= iMasterWritedata;
    oMasterReaddata         <= iIpif_rdData;
end rtl;
