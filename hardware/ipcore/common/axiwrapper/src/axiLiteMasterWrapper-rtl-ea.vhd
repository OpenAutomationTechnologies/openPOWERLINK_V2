-------------------------------------------------------------------------------
--! @file axiLiteMasterWrapper-rtl-ea.vhd
--
--! @brief AXI lite master wrapper on avalon master interface signals
--
--! @details This will convert avalon master interface signals to AXI master
--! interface signals.
--
-------------------------------------------------------------------------------
--
--    Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
--    Copyright (c) 2014, Kalycito Infotech Private Limited.
---   All rights reserved.
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

--! Use standard ieee library
library ieee;
--! Use logic elements
use ieee.std_logic_1164.all;
--! Use libcommon library
library libcommon;
--! Use Global Library
use libcommon.global.all;

entity axiLiteMasterWrapper is
    generic (
        --! Address width for AXI bus interface
        gAddrWidth          : integer := 32;
        --! Data width for AXI bus interface
        gDataWidth          : integer := 32
    );
    port (
        --! Global Clock for AXI
        iAclk               : in    std_logic;
        --! Global Reset for AXI
        inAReset            : in    std_logic;
        --! Address for Write Address Channel
        oAwaddr             : out   std_logic_vector(gAddrWidth-1 downto 0);
        --! Protection for Write Address Channel
        oAwprot             : out   std_logic_vector(2 downto 0);
        --! AddressValid for Write Address Channel
        oAwvalid            : out   std_logic;
        --! AddressReady for Write Address Channel
        iAwready            : in    std_logic;
        --! WriteData for Write Data Channel
        oWdata              : out   std_logic_vector(gDataWidth-1 downto 0);
        --! WriteStrobe for Write Data Channel
        oWstrb              : out   std_logic_vector(gDataWidth/8-1 downto 0);
        --! WriteValid for Write Data Channel
        oWvalid             : out   std_logic;
        --! WriteReady for Write Data Channel
        iWready             : in    std_logic;
        --! WriteLast for Write Data Channel to indicate last write operations
        oWlast              : out   std_logic;
        --! WriteResponse for Write Response Channel
        iBresp              : in    std_logic_vector(1 downto 0); --unused input
        --! ResponseValid for Write Response Channel
        iBvalid             : in    std_logic;
        --!  ResponaseReady for Write Response Channel
        oBready             : out   std_logic;
        --! ReadAddress for Read Address Channel
        oAraddr             : out   std_logic_vector(gAddrWidth-1 downto 0);
        --! ReadAddressProtection for Read Address Channel
        oArprot             : out   std_logic_vector(2 downto 0);
        --! ReadAddressValid for Read Address Channel
        oArvalid            : out   std_logic;
        --! ReadAddressReady for Read Address Channel
        iArready            : in    std_logic;
        --! ReadData for Read Data Channel
        iRdata              : in    std_logic_vector(gDataWidth-1 downto 0);
        --TODO: Remove unused input pin
        --! ReadResponse for Read Data Channel
        iRresp              : in    std_logic_vector(1 downto 0);
        --! ReadValid for Read Data Channel
        iRvalid             : in    std_logic;
        --! ReadReady for Read Data Channel
        oRready             : out   std_logic;
        --! Host Interface IP Clock
        iAvalonClk          : in std_logic;
        --! Host Interface Reset
        iAvalonReset        : in std_logic;
        --! Read signal for Avalon Master Interface
        iAvalonRead         : in    std_logic;
        --! Write Signal for Avalon Master interface
        iAvalonWrite        : in    std_logic;
        --! Address for Avalon Master Interface
        iAvalonAddr         : in    std_logic_vector(gAddrWidth-1 downto 0);
        --! Byte Enable for Avalon Master interface
        iAvalonBE           : in    std_logic_vector(3 downto 0);
        --! Wait request for Avalon Master Interface
        oAvalonWaitReq      : out   std_logic;
        --! Wait Request for Avalon Master Interface
        oAvalonReadValid    : out   std_logic;
        --! Read Data for Avalon Master Interface
        oAvalonReadData     : out   std_logic_vector(gDataWidth-1 downto 0);
        --! Write Data for Avaon Master Interface
        iAvalonWriteData    : in    std_logic_vector(gDataWidth-1 downto 0)
    );
end axiLiteMasterWrapper;

architecture rtl of axiLiteMasterWrapper is

    --! Axi-lite master FSM type
    type tFsm is (
        sINIT,
        sAWVALID,
        sWVALID,
        sBREADY,
        sARVALID,
        sRREADY,
        sWRITE_DONE,
        sREAD_DONE
    );
    --! synchronized fsm state
    signal fsm      : tFsm;
    --! combinational fsm state
    signal fsm_next : tFsm;

    --! Avalon Interface sync FSM
    type tAvalonFsm is (
         sStart,
         sWait,
         sDone
    );
    --! synchronized Avalon fsm state
    signal avmFsm       : tAvalonFsm ;
    --! combinational fsm sate for Avalon fsm
    signal avmFsm_next  : tAvalonFsm ;

    --Handle Avalon Master
    --! Avalon Address
    signal avmAddress        : std_logic_vector (31 downto 0);
    --! Avalon Address temporary signal
    signal avmAddress_next   : std_logic_vector (31 downto 0);
    --! Avalon Read Signal
    signal avmRead           : std_logic;
    --! Avalon Read Signal temporary signal
    signal avmRead_next      : std_logic;
    --! Avalon Write Signal
    signal avmWrite          : std_logic;
    --! Avalon Write Signal temporary signal
    signal avmWrite_next     : std_logic;
    --! Avalon Write Data
    signal avmWdata          : std_logic_vector (31 downto 0);
    --! Avalon Write Data Signal temporary signal
    signal avmWdata_next     : std_logic_vector (31 downto 0);
    --! Avalon Read Data
    signal avmRdata          : std_logic_vector (31 downto 0);
    --! Avalon Read Data temporary Signal
    signal avmRdata_next     : std_logic_vector (31 downto 0);
    --! Avalon start operation
    signal avmStart          : std_logic;
    --! Avalon start operation temporary signal
    signal avmStart_next     : std_logic;
    --! Avalon Byte Enable
    signal avmBE             : std_logic_vector (3 downto 0);
    --! Avalon Byte Enable Signal temporary signal
    signal avmBE_next        : std_logic_vector (3 downto 0);
    --! Avalon Wait Signal
    signal avmWait           : std_logic;

    --  Handle Avalon Master
    --! Complete transfer between AXI and Avalon
    signal done_transfer    : std_logic;
    --! Read Ready for Valid Read operations
    signal RReady           : std_logic;
    --! Write operation complete
    signal writeOp_done     : std_logic;
    --! Read operation complete
    signal readOp_done      : std_logic;
    --! Read Data latch for hold data
    signal readData         : std_logic_vector(31 downto 0);

begin
    --AXI Master Signals

    -- Secure write is not enabled for read/write operations
    oAwprot <= "000";
    oArprot <= "000";
    --Master signal for AXI interface
    oAwaddr <= avmAddress;
    oAraddr <= avmAddress;
    oWdata  <= avmWdata;
    -- Only read or write at a time and Read will always 32bit
    oWstrb  <= avmBE;

    -- Memory operations (AXI4) demands presence of WLAST (active for last data)
    oWlast   <= cActivated;

    oAwvalid <= cActivated when fsm = sINIT and avmWrite = cActivated else
                cActivated when fsm = sAWVALID else
                cInactivated;

    oWvalid  <= cActivated when fsm = sINIT and avmWrite = cActivated else
                cActivated when fsm = sAWVALID else
                cActivated when fsm = sWVALID else
                cInactivated;

    oBready  <= cActivated when fsm = sWRITE_DONE and iBvalid = cActivated else
                cActivated when fsm = sBREADY else
                cInactivated;

    oArvalid <= cActivated when fsm = sINIT and avmRead = cActivated else
                cActivated when fsm =  sARVALID else
                cInactivated;

    RReady  <= cActivated when fsm = sREAD_DONE and iRvalid = cActivated else
               cInactivated;

    oRready <= RReady;

    -- Flop with Enable pin? Anyway passed through a register on Avalon side to
    -- avoid latch issues.
    --FIXME: bring into fsm
    --! Hold the data while it is valid
    REG_RDATA: process(iAclk)
    begin
        if rising_edge (iAclk) then
            if inAReset = cnActivated then
                readData <= x"00000000";
            elsif(iRvalid = cActivated) then
                readData <= iRdata;
            end if;
        end if;
    end process REG_RDATA;

    RReady <=   cActivated when fsm = sREAD_DONE and iRvalid = cActivated else
                cInactivated;

    -- Completion of Read/Write Operations
    done_transfer <= writeOp_done or readOp_done;

    writeOp_done  <= cActivated when fsm = sWRITE_DONE else
                     cInactivated;

    readOp_done   <= cActivated when fsm = sREAD_DONE else
                     cInactivated;

    -- Master FSM
    --TODO: Explain logic if possible with Diagram in doxygen
    --! Clock Based Process for tFsm changes
    SEQ_LOGIC : process(iAclk)
    begin
        if rising_edge (iAclk) then
            if inAReset = cnActivated then
                fsm <= sINIT;
            else
                fsm <= fsm_next;
            end if;
        end if;
    end process SEQ_LOGIC;

    -- Combinational Logics
    --TODO: Explain logic if possible with Diagram in doxygen
    --! Master FSM for AXI-lite interface
    COMB_LOGIC : process (
        fsm,
        avmRead,
        avmWrite,
        avmStart,
        iAwready,
        iWready,
        iBvalid,
        iArready,
        iRvalid
    )
    begin
        -- Default Values for signals
        fsm_next <= fsm;

        case fsm is
            when sINIT =>
            -- Read Operations
            if avmRead = cActivated then
                fsm_next <= sARVALID;
                if iArready = cActivated then
                    if iRvalid = cActivated then
                        fsm_next <= sREAD_DONE;
                    else
                        fsm_next <= sRREADY;
                    end if;
                else
                    fsm_next <= sARVALID;
                end if;
            -- Write Operations
            elsif avmWrite = cActivated then
                fsm_next <= sAWVALID;
                if iAwready = cActivated then
                    if iWready = cActivated then
                        if iBvalid = cActivated then
                            fsm_next <= sWRITE_DONE;
                        else
                            fsm_next <= sBREADY;
                        end if;
                    else
                        fsm_next <= sWVALID;
                    end if;
                else
                    fsm_next <= sAWVALID;
                end if;
            else
                fsm_next <= sINIT;
            end if;

            when sAWVALID =>
                if iAwready = cActivated then
                    if iWready = cActivated then
                        if iBvalid = cActivated then
                            fsm_next <= sWRITE_DONE;
                        else
                            fsm_next <= sBREADY;
                        end if;
                    else
                        fsm_next <= sWVALID;
                    end if;
                else
                    fsm_next <= sAWVALID;
                end if;

            when sWVALID =>
                if iWready = cActivated then
                    if iBvalid = cActivated then
                        fsm_next <= sWRITE_DONE;
                    else
                        fsm_next <= sBREADY;
                    end if;
                else
                    fsm_next <= sWVALID;
                end if;

            when sBREADY =>
                if iBvalid = cActivated then
                    fsm_next <= sWRITE_DONE;
                else
                    fsm_next <= sBREADY;
                end if;

            when sARVALID =>
                if iArready = cActivated  then
                    if iRvalid = cActivated then
                        fsm_next <= sREAD_DONE;
                    else
                        fsm_next <= sRREADY;
                    end if;
                else
                    fsm_next <= sARVALID;
                end if;

            when sRREADY =>
                if iRvalid = cActivated then
                    fsm_next <= sREAD_DONE;
                else
                    fsm_next <= sRREADY;
                end if;

            when sWRITE_DONE =>
            --Wait for Complete activity at avalon side
            if(avmStart = cInactivated) then
               fsm_next <= sINIT;
            else
               fsm_next <= sWRITE_DONE;
            end if;

            when sREAD_DONE =>
            --Wait for Complete activity at avalon side
             if(avmStart = cInactivated) then
                fsm_next <= sINIT;
             else
                fsm_next <= sREAD_DONE;
             end if;

            when others =>
                null;
        end case;
    end process COMB_LOGIC;

    -- Avalon Interface signal crossing through FSM Register the inputs from
    -- Avalon and Pass to AXI to avoid glitches on Master side due to different
    -- clock domains
    --! Sync Clock domains between Avalon & AXI through a handshaking system
    AVM_SYNC:  process (iAvalonClk)
    begin
        if rising_edge (iAvalonClk) then
            if iAvalonReset = cActivated then
                avmFsm      <= sStart;
                avmAddress  <= x"00000000";
                avmRead     <= cInactivated;
                avmWrite    <= cInactivated;
                avmRdata    <= x"00000000";
                avmWdata    <= x"00000000";
                avmStart    <= cInactivated;
                avmBE       <= x"0";
            else
                avmFsm      <= avmFsm_next ;
                avmAddress  <= avmAddress_next;
                avmRead     <= avmRead_next;
                avmWrite    <= avmWrite_next;
                avmRdata    <= avmRdata_next;
                avmWdata    <= avmWdata_next;
                avmStart    <= avmStart_next;
                avmBE       <= avmBE_next;
             end if;
        end if;
    end process AVM_SYNC;
    -- Split the design for better timing
    --! Combinational logic part for FSM
    AVM_COM: process ( iAvalonRead,
                       iAvalonWrite,
                       iAvalonWriteData,
                       iAvalonAddr,
                       iAvalonBE,
                       readData,
                       done_transfer,
                       avmFsm,
                       avmRead,
                       avmWrite,
                       avmStart,
                       avmAddress,
                       avmRdata,
                       avmWdata,
                       avmBE
    )
    begin

       --Default/Initialization of temporary registers
        avmFsm_next      <= avmFsm ;
        avmAddress_next  <= avmAddress;
        avmRead_next     <= avmRead;
        avmWrite_next    <= avmWrite;
        avmRdata_next    <= avmRdata;
        avmWdata_next    <= avmWdata;
        avmStart_next    <= avmStart;
        avmBE_next       <= avmBE;

       case avmFsm is

        when sStart =>
            avmAddress_next <= iAvalonAddr ;
            avmBE_next      <= iAvalonBE   ;

            if iAvalonRead = cActivated then
                avmFsm_next   <= sWait;
                avmStart_next <= cActivated ;
                avmRead_next  <= cActivated ;
            elsif iAvalonWrite = cActivated then
                avmFsm_next   <= sWait;
                avmStart_next <= cActivated ;
                avmWrite_next <= cActivated;
                avmWdata_next <= iAvalonWriteData;
            else
                avmFsm_next   <= sStart;
                avmStart_next <= cInactivated ;
                avmRead_next  <= cInactivated ;
                avmWrite_next <= cInactivated;
                avmWdata_next <= x"00000000";
            end if;
        --Wait until the transfer get completed at AXI
        when sWait  =>

           avmStart_next <= avmStart;
           avmRead_next  <= avmRead ;
           avmWrite_next <= avmWrite;

           if(done_transfer = cActivated) then
            avmFsm_next   <= sDone;
                -- Only for Read operations
                if(iAvalonRead = cActivated) then
                    avmRdata_next <=  readData;
                else
                    avmRdata_next <=  avmRdata;
                end if;
           else
            avmFsm_next   <= sWait;
           end if;
        -- Handshake between two FSM domains
        when sDone  =>
            if (done_transfer = cActivated) then
                avmRead_next  <= cInactivated ;
                avmWrite_next <= cInactivated;
                avmStart_next <= cInactivated ;
                avmFsm_next   <= sStart;
            else
                avmRead_next  <= avmRead ;
                avmWrite_next <= avmWrite;
                avmStart_next <= avmStart;
                avmFsm_next   <= sDone;
            end if;
       end case;
   end process AVM_COM;

    --Avalon Interface signals
    oAvalonReadData     <= readData;
    oAvalonReadValid    <= not avmWait;
    oAvalonWaitReq      <= avmWait;

    avmWait             <= cInactivated when avmFsm = sDone else
                           cActivated ;

end rtl;
