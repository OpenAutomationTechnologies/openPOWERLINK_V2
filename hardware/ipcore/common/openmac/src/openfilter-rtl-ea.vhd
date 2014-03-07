-------------------------------------------------------------------------------
--! @file openfilter-rtl-ea.vhd
--
--! @brief OpenFILTER
--
--! @details This is the openFILTER used for blocking failures on the RMII lines.
--!         Note: RxDv and RxDat have to be synchron to iClk
--!             The following Conditions are checked:
--!                 * RxDV >163.64탎ec HIGH -> invalid
--!                 * RxDV <0.64탎ec LOW -> invalid
--!                 * RxDV 4x <5.12탎ec HIGH -> invalid
--!                 * RxDV >5.12탎ec HIGH -> valid
--!                 * iRxError HIGH -> invalid
--!         If invalid deactivation of port, until RxDv and iRxError > 10.24탎ec low
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

--! Work library
library work;
--! use openmac package
use work.openmacPkg.all;

entity openfilter is
    port (
        --! Reset
        iRst        : in    std_logic;
        --! RMII Clock
        iClk        : in    std_logic;
        --! RMII receive path in
        iRx         : in    tRmiiPath;
        --! RMII receive path out
        oRx         : out   tRmiiPath;
        --! RMII transmit path in
        iTx         : in    tRmiiPath;
        --! RMII transmit path out
        oTx         : out   tRmiiPath;
        --! RMII receive error
        iRxError    : in    std_logic
    );
end entity openfilter;

architecture rtl of openfilter is
    --! Filter FSM type
    type tFiltState is (
        fs_init,
        fs_GAP2short, fs_GAPext, fs_GAPok,
        fs_FRMnopre, fs_FRMpre2short, fs_FRMpreOk,
        fs_FRM2short, fs_FRMok, fs_FRM2long, fs_BlockAll
    );

    signal FiltState     : tFiltState;
    signal RxDel         : tRmiiPathArray(3 downto 0);
    signal FrameShift    : std_logic;
    signal LastFrameNOK  : std_logic;
    signal StCnt         : std_logic_vector(13 downto 0);
    signal BlockRxPort   : std_logic;
begin

    ---------------------------------------------------------------------------
    -- INPUT
    ---------------------------------------------------------------------------
    RxDel(0)    <=  iRx;
    BlockRxPort <=  cActivated when (FiltState = fs_FRMnopre or
                                    FiltState = fs_BlockAll or
                                    LastFrameNOK = cActivated) else
                    cInactivated;

    ---------------------------------------------------------------------------
    -- OUTPUT MUX
    ---------------------------------------------------------------------------
    oRx     <=  cRmiiPathInit   when BlockRxPort = cActivated else
                RxDel(3)        when FrameShift = cActivated else
                RxDel(1);
    oTx     <=  iTx;

    doFsm : process(iRst, iClk)
        variable RstStCnt : std_logic;
    begin
        if iRst = cActivated then
            StCnt               <= (others => cInactivated);
            FiltState           <= fs_init;
            FrameShift          <= cInactivated;
            RxDel(3 downto 1)   <= (others => cRmiiPathInit);
            LastFrameNOK        <= cInactivated;
        elsif rising_edge(iClk) then
            RxDel(3 downto 1) <= RxDel(2 downto 0);

            -- DEFAULT --
            RstStCnt := cInactivated;

            case FiltState is
                ---------------------------------------------------------------
                -- INIT
                ---------------------------------------------------------------
                when fs_init =>
                    FiltState <= fs_GAP2short;
                    RstStCnt  := cActivated;

                ---------------------------------------------------------------
                -- GAP 2 SHORT
                ---------------------------------------------------------------
                when fs_GAP2short =>
                    FrameShift <= cInactivated;
                    if StCnt(4) = cActivated then
                        -- 360ns
                        FiltState <= fs_GAPext;
                    end if;
                    if RxDel(0).enable = cActivated then
                        -- Gap < 360 ns -> too short -> Block Filter
                        FiltState <= fs_BlockAll;
                        RstStCnt := cActivated;
                    end if;

                ---------------------------------------------------------------
                -- GAP EXTend
                ---------------------------------------------------------------
                when fs_GAPext =>
                    if StCnt(5 downto 0) = "101110" then
                        FiltState <= fs_GAPok;
                    end if;
                    if RxDel(0).enable = cActivated then
                        -- GAP [360ns .. 960ns] -> short, but ok -> Start Frame
                        RstStCnt := cActivated;
                        FrameShift <= cActivated;
                        if RxDel(0).data = "01" then
                            -- GAP > 960ns -> OK -> Start Frame (preamble already beginning)
                            FiltState <= fs_FRMpre2short;
                        else
                            -- GAP > 960ns -> OK -> Start Frame and wait of preamble
                            FiltState <= fs_FRMnopre;
                        end if;
                    end if;

                ---------------------------------------------------------------
                -- GAP OK
                ---------------------------------------------------------------
                when fs_GAPok =>
                    if RxDel(0).enable = cActivated then
                        RstStCnt := cActivated;
                        if RxDel(0).data = "01" then
                            -- GAP > 960ns -> OK -> Start Frame (preamble already beginning)
                            FiltState <= fs_FRMpre2short;
                        else
                            -- GAP > 960ns -> OK -> Start Frame and wait of preamble
                            FiltState <= fs_FRMnopre;
                        end if;
                    end if;

                ---------------------------------------------------------------
                -- FRAME, BUT STILL NO PREAMBLE
                ---------------------------------------------------------------
                when fs_FRMnopre =>
                    if (StCnt(5) = cActivated or
                        RxDel(0).data = "11" or  RxDel(0).data = "10" or
                        (RxDel(0).enable = cInactivated  and RxDel(1).enable = cInactivated)) then
                        -- no preamble for >=660 ns or preamble wrong -> Block Filter
                        FiltState <= fs_BlockAll;
                        RstStCnt := cActivated;
                    elsif RxDel(0).data = "01" then
                        -- preamble starts -> Check Preamble
                        FiltState <= fs_FRMpre2short;
                        RstStCnt := cActivated;
                    end if;

                ---------------------------------------------------------------
                -- FRAME CHECK PREAMBLE TOO SHORT
                ---------------------------------------------------------------
                when fs_FRMpre2short =>
                    if (RxDel(0).data /= "01" or (RxDel(0).enable = cInactivated and
                        RxDel(1).enable = cInactivated)) then
                        -- preamble wrong -> Block Filter
                        FiltState <= fs_BlockAll;
                        RstStCnt := cActivated;
                    elsif StCnt(3) = cActivated then
                        -- preamble ok for 180 ns -> Preamble OK
                        FiltState <= fs_FRMpreOk;
                    end if;

                ---------------------------------------------------------------
                -- FRAME CHECK PREAMBLE OK
                ---------------------------------------------------------------
                when fs_FRMpreOk =>
                    if RxDel(0).data /= "01" then
                        -- preamble done -> Start Frame
                        FiltState <= fs_FRMok;
                    end if;
                    if ((StCnt(5) = cActivated and StCnt(2) = cActivated) or
                        (RxDel(0).enable = cInactivated and RxDel(1).enable = cInactivated)) then
                        -- preamble to long for 740 ns  -> Block Filter
                        FiltState <= fs_BlockAll;
                        RstStCnt := cActivated;
                    end if;
                    -- preamble is OK
                    LastFrameNOK <= cInactivated;

                ---------------------------------------------------------------
                -- FRAME OK
                ---------------------------------------------------------------
                when fs_FRMok     =>
                    if StCnt(13) = cActivated then
                        -- FRAME > 163,842 us -> too long -> Block Filter
                        FiltState <= fs_BlockAll;
                        RstStCnt := cActivated;
                    end if;
                    if RxDel(0).enable = cInactivated and RxDel(1).enable = cInactivated then
                        -- FRAME [163,842 us] -> OK -> Start GAP
                        FiltState <= fs_GAP2short;
                        RstStCnt := cActivated;
                    end if;

                ---------------------------------------------------------------
                -- BLOCK FILTER
                ---------------------------------------------------------------
                when fs_BlockAll   =>
                    if StCnt(2) = cActivated then
                        -- Block for 100 nsec
                        FiltState <= fs_GAP2short;
                        RstStCnt := cActivated;
                    end if;
                    if RxDel(0).enable = cActivated then
                        -- Rxdv != cInactivated -> Reset Wait Period
                        RstStCnt := cActivated;
                    end if;
                    -- block next rx frame (until receive a valid preamble)
                    LastFrameNOK <= cActivated;

                when others =>
                    FiltState <= fs_init;
            end case;

            if iRxError = cActivated then
                -- iRxError -> Block Filter
                FiltState <= fs_BlockAll;
                RstStCnt := cActivated;
            end if;

            -- State Counter --
            StCnt <= std_logic_vector(unsigned(StCnt) + 1);
            if RstStCnt = cActivated then
                StCnt <= (others => cInactivated);
            end if;
        end if;
    end process;
end rtl;
