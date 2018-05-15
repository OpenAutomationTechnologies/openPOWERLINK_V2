-------------------------------------------------------------------------------
--! @file convRmiiToMii-rtl-ea.vhd
--
--! @brief RMII-to-MII converter
--
--! @details This is an RMII-to-MII converter to convert MII phy traces to RMII.
--!          Example: MII PHY <--> RMII-to-MII converter <--> RMII MAC
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

--! Common library
library libcommon;
--! Use common library global package
use libcommon.global.all;

--! Work library
library work;
--! use openmac package
use work.openmacPkg.all;

entity convRmiiToMii is
    port (
        --! Reset
        iRst        : in    std_logic;
        --! RMII Clock
        iClk        : in    std_logic;
        --! RMII transmit path
        iRmiiTx     : in    tRmiiPath;
        --! RMII receive path
        oRmiiRx     : out   tRmiiPath;
        --! MII receive clock
        iMiiRxClk   : in    std_logic;
        --! MII receive path
        iMiiRx      : in    tMiiPath;
        --! MII receive error
        iMiiRxError : in    std_logic;
        --! MII transmit clock
        iMiiTxClk   : in    std_logic;
        --! MII transmit path
        oMiiTx      : out   tMiiPath
    );
end convRmiiToMii;

architecture rtl of convRmiiToMii is
    constant DIBIT_SIZE : integer := 2;
    constant NIBBLE_SIZE : integer := 4;
begin

    TX_BLOCK : block
        --fifo size must not be larger than 2**5
        constant FIFO_NIBBLES_LOG2 : integer := 5;

        signal fifo_half, fifo_full, fifo_empty, fifo_valid, txEnable_reg : std_logic;
        signal fifo_wr, fifo_rd : std_logic;
        signal fifo_din : std_logic_vector(NIBBLE_SIZE-1 downto 0);
        signal fifo_dout, txData_reg : std_logic_vector(NIBBLE_SIZE-1 downto 0);
        signal fifo_rdUsedWord : std_logic_vector (FIFO_NIBBLES_LOG2-1 downto 0);
        --necessary for clr fifo
        signal aclr, rTxEn_l : std_logic;

        --convert dibits to nibble
        signal sel_dibit : std_logic;
        signal fifo_din_reg : std_logic_vector(iRmiiTx.data'range);
    begin

        fifo_din <= iRmiiTx.data & fifo_din_reg;
        fifo_wr <= sel_dibit;

        --convert dibits to nibble (to fit to fifo)
        process(iClk, iRst)
        begin
            if iRst = cActivated then
                sel_dibit <= cInactivated;
                fifo_din_reg <= (others => cInactivated);
            elsif iClk = cActivated and iClk'event then
                if iRmiiTx.enable = cActivated then
                    sel_dibit <= not sel_dibit;
                    if sel_dibit = cInactivated then
                        fifo_din_reg <= iRmiiTx.data;
                    end if;
                else
                    sel_dibit <= cInactivated;
                end if;
            end if;
        end process;

        fifo_half <= fifo_rdUsedWord(fifo_rdUsedWord'left);

        oMiiTx.data <= txData_reg;
        oMiiTx.enable <= txEnable_reg;

        process(iMiiTxClk, iRst)
        begin
            if iRst = cActivated then
                fifo_rd         <= cInactivated;
                fifo_valid      <= cInactivated;
                txData_reg      <= (others => cInactivated);
                txEnable_reg    <= cInactivated;
            elsif iMiiTxClk = cActivated and iMiiTxClk'event then
                txData_reg      <= fifo_dout;
                txEnable_reg    <= fifo_valid;

                if fifo_rd = cInactivated and fifo_half = cActivated then
                    fifo_rd <= cActivated;
                elsif fifo_rd = cActivated and fifo_empty = cActivated then
                    fifo_rd <= cInactivated;
                end if;

                if fifo_rd = cActivated and fifo_rdUsedWord > std_logic_vector(to_unsigned(1, fifo_rdUsedWord'length)) then
                    fifo_valid <= cActivated;
                else
                    fifo_valid <= cInactivated;
                end if;
            end if;
        end process;

        --! This is the asynchronous FIFO used to decouple RMII from MII.
        TXFIFO : entity work.asyncFifo
            generic map (
                gDataWidth  => NIBBLE_SIZE,
                gWordSize   => 2**FIFO_NIBBLES_LOG2,
                gSyncStages => 2,
                gMemRes     => "ON"
            )
            port map (
                iAclr       => aclr,
                iWrClk      => iClk,
                iWrReq      => fifo_wr,
                iWrData     => fifo_din,
                oWrEmpty    => open,
                oWrFull     => fifo_full,
                oWrUsedw    => open,
                iRdClk      => iMiiTxClk,
                iRdReq      => fifo_rd,
                oRdData     => fifo_dout,
                oRdEmpty    => fifo_empty,
                oRdFull     => open,
                oRdUsedw    => fifo_rdUsedWord
            );

        --sync Mii Tx En (=fifo_valid) to wr clk
        process(iClk, iRst)
        begin
            if iRst = cActivated then
                aclr <= cActivated; --reset fifo
                rTxEn_l <= cInactivated;
            elsif iClk = cActivated and iClk'event then
                rTxEn_l <= iRmiiTx.enable;

                aclr <= cInactivated; --default

                --clear the full fifo after TX on RMII side is done
                if fifo_full = cActivated and rTxEn_l = cActivated and iRmiiTx.enable = cInactivated then
                    aclr <= cActivated;
                end if;
            end if;
        end process;

    end block;

    RX_BLOCK : block
        --fifo size must not be larger than 2**5
        constant FIFO_NIBBLES_LOG2 : integer := 5;

        signal fifo_half, fifo_empty, fifo_valid : std_logic;
        signal rxDataValid_reg, fifo_rd : std_logic;
        signal rxError_reg : std_logic;
        signal fifo_wr : std_logic;
        signal rxData_reg : std_logic_vector(NIBBLE_SIZE-1 downto 0);
        signal fifo_dout : std_logic_vector(NIBBLE_SIZE-1 downto 0);
        signal fifo_rdUsedWord : std_logic_vector(FIFO_NIBBLES_LOG2-1 downto 0);
        signal fifo_wrUsedWord : std_logic_vector(FIFO_NIBBLES_LOG2-1 downto 0);
        --convert nibble to dibits
        signal sel_dibit : std_logic;
        signal fifo_rd_s : std_logic;
    begin


        process(iMiiRxClk, iRst)
        begin
            if iRst = cActivated then
                rxData_reg      <= (others => cInactivated);
                rxDataValid_reg <= cInactivated;
                rxError_reg     <= cInactivated;
            elsif iMiiRxClk = cActivated and iMiiRxClk'event then
                rxData_reg      <= iMiiRx.data;
                rxDataValid_reg <= iMiiRx.enable;
                rxError_reg     <= iMiiRxError;
            end if;
        end process;

        fifo_wr <= rxDataValid_reg and not rxError_reg;

        oRmiiRx.data <=     fifo_dout(fifo_dout'right+1 downto 0) when sel_dibit = cActivated else
                    fifo_dout(fifo_dout'left downto fifo_dout'left-1);

        oRmiiRx.enable <= fifo_valid;
        fifo_rd <= fifo_rd_s and not sel_dibit;

        process(iClk, iRst)
        begin
            if iRst = cActivated then
                sel_dibit <= cInactivated;
            elsif iClk = cActivated and iClk'event then
                if fifo_rd_s = cActivated or fifo_valid = cActivated then
                    sel_dibit <= not sel_dibit;
                else
                    sel_dibit <= cInactivated;
                end if;
            end if;
        end process;

        fifo_half <= fifo_rdUsedWord(fifo_rdUsedWord'left);

        process(iClk, iRst)
        begin
            if iRst = cActivated then
                fifo_rd_s <= cInactivated;
                fifo_valid <= cInactivated;
            elsif iClk = cActivated and iClk'event then
                if fifo_rd_s = cInactivated and fifo_half = cActivated then
                    fifo_rd_s <= cActivated;
                elsif fifo_rd_s = cActivated  and fifo_empty = cActivated then
                    fifo_rd_s <= cInactivated;
                end if;

                if fifo_rd_s = cActivated then
                    fifo_valid <= cActivated;
                else
                    fifo_valid <= cInactivated;
                end if;
            end if;
        end process;

        --! This is the asynchronous FIFO used to decouple RMII from MII.
        RXFIFO : entity work.asyncFifo
            generic map (
                gDataWidth  => NIBBLE_SIZE,
                gWordSize   => 2**FIFO_NIBBLES_LOG2,
                gSyncStages => 2,
                gMemRes     => "ON"
            )
            port map (
                iAclr       => iRst,
                iWrClk      => iMiiRxClk,
                iWrReq      => fifo_wr,
                iWrData     => rxData_reg,
                oWrEmpty    => open,
                oWrFull     => open,
                oWrUsedw    => open,
                iRdClk      => iClk,
                iRdReq      => fifo_rd,
                oRdData     => fifo_dout,
                oRdEmpty    => fifo_empty,
                oRdFull     => open,
                oRdUsedw    => fifo_rdUsedWord
            );
    end block;
end rtl;
