-------------------------------------------------------------------------------
--! @file axiLiteSlaveWrapper-rtl-ea.vhd
--
--! @brief AXI lite slave wrapper on avalon slave interface signals
--
--! @details AXI lite slave will convert AXI slave interface singal to Avalon
--! interface signals.
--
-------------------------------------------------------------------------------
--
--    Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
--    Copyright (c) 2014, Kalycito Infotech Private Limited.
--    All rights reserved.
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

-------------------------------------------------------------------------------
--! @brief
--! @details  AXI-lite slave wrapper will receive signals from AXI bus and
--! provide proper inputs for a Avlon interface to perform the same action
--! initiated by AXI master
-------------------------------------------------------------------------------
entity axiLiteSlaveWrapper is
    generic (
        --! Base Lower address for the AXI-lite slave interface
        gBaseAddr       : std_logic_vector(31 downto 0) := x"00000000";
        --! Base Higher address for the AXI-lite slave interface
        gHighAddr       : std_logic_vector(31 downto 0) := x"0000ffff";
        --! Address width for AXI bus interface
        gAddrWidth      : integer                       := 32;
        --! Data width for AXI bus interface
        gDataWidth      : integer                       := 32
    );
    port (
        --! Global Clock for AXI
        iAclk           : in    std_logic;
        --! Global Reset for AXI
        inAReset        : in    std_logic;
        --! Address for Write Address Channel
        iAwaddr         : in    std_logic_vector(gAddrWidth-1 downto 0);
        --! Protection for Write Address Channel
        iAwprot         : in    std_logic_vector(2 downto 0); --unused input
        --! AddressValid for Write Address Channel
        iAwvalid        : in    std_logic;
        --! AddressReady for Write Address Channel
        oAwready        : out   std_logic;
        --! WriteData for Write Data Channel
        iWdata          : in    std_logic_vector(gDataWidth-1 downto 0);
        --! WriteStrobe for Write Data Channel
        iWstrb          : in    std_logic_vector(gDataWidth/8-1 downto 0);
        --! WriteValid for Write Data Channel
        iWvalid         : in    std_logic;
        --! WriteReady for Write Data Channel
        oWready         : out   std_logic;
        --! WriteResponse for Write Response Channel
        oBresp          : out   std_logic_vector (1 downto 0);
        --! ResponseValid for Write Response Channel
        oBvalid         : out   std_logic;
        --! ResponaseReady for Write Response Channel
        iBready         : in    std_logic;
        --! ReadAddress for Read Address Channel
        iAraddr         : in    std_logic_vector(gAddrWidth-1 downto 0);
        --! ReadAddressProtection for Read Address Channel
        iArprot         : in    std_logic_vector(2 downto 0); --unused input
        --! ReadAddressValid for Read Address Channel
        iArvalid        : in    std_logic;
        --! ReadAddressReady for Read Address Channel
        oArready        : out   std_logic;
        --! ReadData for Read Data Channel
        oRdata          : out   std_logic_vector(gDataWidth-1 downto 0);
        --! ReadResponse for Read Data Channel
        oRresp          : out   std_logic_vector(1 downto 0);
        --! ReadValid for Read Data Channel
        oRvalid         : out   std_logic;
        --! ReadReady for Read Data Channel
        iRready         : in    std_logic;
        --! Address to Avalon Slave Interface
        oAvsAddress     : out   std_logic_vector(gAddrWidth-1 downto 0);
        --! Byte Enable for Avalon Slave interface
        oAvsByteenable  : out   std_logic_vector(gDataWidth/8-1 downto 0);
        --! Write Data for Avalon Slave interface
        oAvsWritedata   : out   std_logic_vector(gDataWidth-1 downto 0);
        --! Read Data for Avalon Slave interface
        iAvsReaddata    : in    std_logic_vector(gDataWidth-1 downto 0);
        --! Read signal for Avalon Slave interface
        oAvsRead        : out   std_logic;
        --! Write signal for Avalon Slave interface
        oAvsWrite       : out   std_logic;
        --! WaitRequest for Avalon slave interface
        iAvsWaitrequest : in    std_logic
    );
end axiLiteSlaveWrapper;

architecture rtl of axiLiteSlaveWrapper is
    --! Control signal FSM
    type tFsm is (
        sIDLE,
        sREAD,
        sREAD_DONE,
        sWRITE,
        sWRITE_DONE,
        sWRRES_DONE,
        sDELAY
    );

    --Avalon Interface designs
    --! address latch for Avalon Interface
    signal  address     : std_logic_vector(gAddrWidth-1 downto 0);
    --! Muxed address from AXI interface
    signal  mux_address : std_logic_vector(gAddrWidth-1 downto 0);
    --! Chip select for address decoder
    signal  chip_sel    : std_logic;
    --! Muxed byte enable latch from AXI Interface
    signal  byte_enable : std_logic_vector(gDataWidth/8-1 downto 0);

    --Signals for FSM
    --! synchronized fsm state
    signal  fsm         :  tFsm;
    --! fsm state for combinational logic
    signal  fsm_next    :  tFsm;

    --Internal Signals
    --! control for Avalon read signal with fsm
    signal avalonRead          : std_logic;
    --! Read Data latch for Avalon interface
    signal avalonReadDataLatch : std_logic_vector(31 downto 0);
    --! control for Avalon write signal with fsm
    signal avalonWrite         : std_logic;

    --! write data from AXI for Avalon interface
    signal axiWriteData : std_logic_vector(31 downto 0);
    --! valid data from AXI to Avalon
    signal axiDataValid : std_logic;

    --! Write start fsm operations
    signal writeStart   : std_logic;
    --! Write select for control write operations
    signal write_sel    : std_logic;
    --! Read Start for fsm operations
    signal readStart    : std_logic;
    --! Read select for control read operations
    signal read_sel     : std_logic;
begin

    --Avalon Slave Interface Signals
    oAvsAddress     <= address;
    oAvsByteenable  <= byte_enable;
    oAvsRead        <= avalonRead;
    oAvsWrite       <= avalonWrite;
    oAvsWritedata   <= axiWriteData;

    avalonRead <=   cActivated  when readStart = cActivated and fsm = sIDLE else
                    cActivated  when fsm = sREAD else
                    cInactivated when fsm = sREAD_DONE else
                    cInactivated;

    avalonWrite <=  cActivated when fsm = sWRITE and iWvalid = cActivated else
                    cActivated when fsm = sIDLE and axiDataValid = cActivated else
                    cActivated when fsm = sWRITE_DONE else
                    cInactivated;

    axiWriteData <= iWdata when axiDataValid = cActivated else
                    axiWriteData;

    -- AXI-Lite Write Data Signals
    oBvalid  <= cActivated when fsm = sWRITE_DONE and iAvsWaitrequest = cInactivated else
                cActivated when fsm = sWRRES_DONE else
                cInactivated;

    oAwready <= cActivated when fsm = sIDLE and writeStart = cActivated else
                cInactivated;

    oWready  <= cActivated when fsm = sWRITE else
                cActivated when fsm = sIDLE and axiDataValid = cActivated else
                cInactivated;

    -- AXI-lite Read Data Signals
    oArready <= cActivated when fsm = sIDLE and readStart = cActivated else
                cInactivated;

    oRvalid  <= cActivated when iAvsWaitrequest = cInactivated and fsm = sREAD else
                cActivated when fsm = sREAD_DONE else
                cInactivated;

    oRdata <= avalonReadDataLatch;

    avalonReadDataLatch <=  iAvsReaddata  when iAvsWaitrequest = cInactivated else
                            avalonReadDataLatch;

    --TODO: Check the possibility of Error Response signals
    oBresp <= "00";
    oRresp <= "00";

    -- Address Decoder
    chip_sel <= read_sel or write_sel;
    -- 64Kbyte address range only supported so MSB 16 bits enough for Decoding
    write_sel <=    cActivated when iAwaddr(31 downto 16) = gBaseAddr(31 downto 16) else
                    cInactivated;

    read_sel  <=    cActivated when iAraddr(31 downto 16) = gBaseAddr(31 downto 16) else
                    cInactivated;

    -- TODO: Check possibilities of reduce the no of bits in MUX/latch design
    -- and avoid combinational feedback on MUX
    -- Mux the address first and latch it with FSM
    address     <= mux_address when fsm = sIDLE else
                   address ;

    mux_address <= iAraddr when readStart = cActivated else
                   iAwaddr when writeStart = cActivated else
                   x"00000000" ;

    writeStart      <= chip_sel and iAwvalid;
    readStart       <= chip_sel and iArvalid;
    axiDataValid    <= iWvalid;

    byte_enable <=  x"F" when readStart = cActivated and fsm = sIDLE else
                    iWstrb when writeStart = cActivated and fsm = sIDLE else
                    byte_enable;

    -- Main Control FSM for converting AXI-lite signals to Avalon
    --! Clock Based Process for state changes
    SEQ_LOGIC_FSM : process(iAclk)
    begin
        if rising_edge(iAclk) then
            if inAReset = cnActivated then
                fsm <= sIDLE;
            else
                fsm <= fsm_next;
            end if;
        end if;
    end process SEQ_LOGIC_FSM;

    --! Control State machine
    COM_LOGIC_FSM : process (
        fsm,
        chip_sel,
        iAwvalid,
        iArvalid,
        iRready,
        iWvalid,
        iBready,
        iAvsWaitrequest
    )
    begin
        --Default to avoid latches
        fsm_next <= fsm;

        case fsm is
            when sIDLE =>
                if chip_sel = cActivated then
                    --Write Operations
                    if iAwvalid = cActivated then
                        if iWvalid = cActivated then
                            if iAvsWaitrequest = cInactivated then
                                fsm_next <= sWRRES_DONE;
                            else
                                fsm_next <= sWRITE_DONE;
                            end if;
                        else
                            fsm_next <= sWRITE;
                        end if;
                        --Read Operations
                    elsif iArvalid = cActivated then
                        if iAvsWaitrequest = cInactivated then
                            fsm_next <= sREAD_DONE;
                        else
                            fsm_next <= sREAD;
                        end if;
                    else
                        fsm_next <= sIDLE;
                    end if;
                else
                    fsm_next <= sIDLE;
                end if;

            when sREAD =>
                -- Read Valid gets assert Here
                if iAvsWaitrequest = cInactivated then
                    if iRready = cActivated then
                        fsm_next <= sIDLE;
                    else
                        fsm_next <= sREAD_DONE;
                    end if;
                else
                    fsm_next <= sREAD;
                end if;

            when sREAD_DONE =>
                if iRready = cActivated then
                    fsm_next <= sIDLE;
                else
                    fsm_next <= sREAD_DONE;
                end if;

            when sWRITE =>
                if iWvalid = cActivated then
                    if iAvsWaitrequest = cInactivated then
                        if iBready = cActivated then
                        fsm_next <= sIDLE;
                        else
                        fsm_next <= sWRRES_DONE;
                        end if;
                    else
                        fsm_next <= sWRITE_DONE;
                    end if;
                else
                    fsm_next <= sWRITE;
                end if;

            when sWRITE_DONE =>
                if iAvsWaitrequest = cInactivated then
                    if iBready = cActivated then
                        fsm_next <= sIDLE;
                    else
                        fsm_next <= sWRRES_DONE;
                    end if;
                else
                    fsm_next <= sWRITE_DONE;
                end if;

            when sWRRES_DONE =>
                if iBready = cActivated then
                    fsm_next <= sIDLE;
                else
                    fsm_next <= sWRRES_DONE;
                end if;

            when sDELAY =>
                fsm_next <= sIDLE;

            when others =>
                null;
        end case;
    end process COM_LOGIC_FSM;
end rtl;
