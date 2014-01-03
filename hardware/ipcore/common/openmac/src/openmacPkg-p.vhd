-------------------------------------------------------------------------------
--! @file openmacPkg-p.vhd
--
--! @brief OpenMAC package
--
--! @details This is the openMAC package providing common types.
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

package openmacPkg is
    ---------------------------------------------------------------------------
    -- Configuration
    ---------------------------------------------------------------------------
    --! Packet buffer is internal (e.g. memory blocks)
    constant cPktBufLocal   : natural := 1;
    --! Packet buffer is external
    constant cPktBufExtern  : natural := 2;

    --! Phy port(s) are Rmii
    constant cPhyPortRmii   : natural := 1;
    --! Phy port(s) are Mii
    constant cPhyPortMii    : natural := 2;

    ---------------------------------------------------------------------------
    -- (R)MII types and constants
    ---------------------------------------------------------------------------
    --! RMII data width
    constant cRmiiDataWidth : natural := 2;
    --! MII data width
    constant cMiiDataWidth  : natural := 4;

    --! RMII path
    type tRmiiPath is record
        enable  : std_logic;
        data    : std_logic_vector(cRmiiDataWidth-1 downto 0);
    end record;

    --! MII path
    type tMiiPath is record
        enable  : std_logic;
        data    : std_logic_vector(cMiiDataWidth-1 downto 0);
    end record;

    --! RMII path array
    type tRmiiPathArray is array (natural range <>) of tRmiiPath;
    --! MII path array
    type tMiiPathArray is array (natural range <>) of tMiiPath;

    --! RMII link
    type tRmii is record
        rx  : tRmiiPath;
        tx  : tRmiiPath;
    end record;

    --! MII link
    type tMii is record
        rx  : tMiiPath;
        tx  : tMiiPath;
    end record;

    --! RMII link array
    type tRmiiArray is array (natural range <>) of tRmii;
    --! MII link array
    type tMiiArray is array (natural range <>) of tMii;

    --! RMII path initialization
    constant cRmiiPathInit  : tRmiiPath := (
        enable  => cInactivated,
        data    => (others => cInactivated)
    );

    --! MII path initialization
    constant cMiiPathInit   : tMiiPath := (
        enable  => cInactivated,
        data    => (others => cInactivated)
    );

    --! RMII link initialization
    constant cRmiiInit  : tRmii := (
        rx  => cRmiiPathInit,
        tx  => cRmiiPathInit
    );

    --! MII link initialization
    constant cMiiInit   : tMii := (
        rx  => cMiiPathInit,
        tx  => cMiiPathInit
    );

    --! Functio to get tRmiiPathArray enables.
    function rmiiGetEnable ( iArray : tRmiiPathArray ) return std_logic_vector;

    --! Procedure to convert tRmiiPathArray to std_logic_vector.
    procedure rmiiPathArrayToStdLogicVector (
        signal iVector      : in    tRmiiPathArray;
        signal oEnable      : out   std_logic_vector;
        signal oData        : out   std_logic_vector
    );

    --! Procedure to convert std_logic_vector to tRmiiPathArray
    procedure stdLogicVectorToRmiiPathArray (
        signal iEnable      : in    std_logic_vector;
        signal iData        : in    std_logic_vector;
        signal oVector      : out   tRmiiPathArray
    );

    --! Procedure to convert tMiiPathArray to std_logic_vector.
    procedure miiPathArrayToStdLogicVector (
        signal iVector      : in    tMiiPathArray;
        signal oEnable      : out   std_logic_vector;
        signal oData        : out   std_logic_vector
    );

    --! Procedure to convert std_logic_vector to tMiiPathArray
    procedure stdLogicVectorToMiiPathArray (
        signal iEnable      : in    std_logic_vector;
        signal iData        : in    std_logic_vector;
        signal oVector      : out   tMiiPathArray
    );

    ---------------------------------------------------------------------------
    -- Memory mapping
    ---------------------------------------------------------------------------
    --! Memory range type
    type tMemRange is record
        base    : natural;
        high    : natural;
    end record;

    --! openMAC memory mapping type
    type tMemMap is array (natural range <>) of tMemRange;

    --! openMAC memory map count
    constant cMemMapCount           : natural := 6;
    --! openMAC memory map index for DMA Error
    constant cMemMapIndex_dmaError  : natural := 5;
    --! openMAC memory map index for IRQ Table
    constant cMemMapIndex_irqTable  : natural := 4;
    --! openMAC memory map index for SMI
    constant cMemMapIndex_smi       : natural := 3;
    --! openMAC memory map index for MAC RAM
    constant cMemMapIndex_macRam    : natural := 2;
    --! openMAC memory map index for MAC Filter
    constant cMemMapIndex_macFilter : natural := 1;
    --! openMAC memory map index for MAC Content
    constant cMemMapIndex_macCont   : natural := 0;

    --! openMAC memory mapping table
    constant cMemMapTable   : tMemMap(cMemMapCount-1 downto 0) := (
        (base => 16#1020#, high => 16#102F#), -- DMA error
        (base => 16#1010#, high => 16#101F#), -- IRQ table
        (base => 16#1000#, high => 16#100F#), -- SMI
        (base => 16#0800#, high => 16#0FFF#), -- MAC ram
        (base => 16#0800#, high => 16#0BFF#), -- MAC filter
        (base => 16#0000#, high => 16#03FF#) -- MAC content
    );

    ---------------------------------------------------------------------------
    -- Access delay
    ---------------------------------------------------------------------------
    --! Access delay type
    type tMemAccessDelay is record
        write   : natural;
        read    : natural;
    end record;

    --! Access delay type
    type tMemAccessDelayArray is array (natural range <>) of tMemAccessDelay;

    --! Access delay count
    constant cMemAccessDelayCount           : natural := 3;
    --! Access delay index for PKT BUFFER
    constant cMemAccessDelayIndex_pktBuf    : natural := 2;
    --! Access delay index for MAC TIMER
    constant cMemAccessDelayIndex_macTimer  : natural := 1;
    --! Access delay index for MAC REG
    constant cMemAccessDelayIndex_macReg    : natural := 0;

    --! Access delay table
    constant cMemAccessDelayTable   : tMemAccessDelayArray(cMemAccessDelayCount-1 downto 0) := (
        (write => 0, read => 1), -- PKT BUFFER
        (write => 0, read => 1), -- MAC TIMER
        (write => 0, read => 1) -- MAC REG
    );

    --! Access acknowlegde type
    type tMemAccessAck is record
        write   : std_logic;
        read    : std_logic;
    end record;

    --! Access acknowledge array type
    type tMemAccessAckArray is array (natural range <>) of tMemAccessAck;

    ---------------------------------------------------------------------------
    -- Constants for openmac
    ---------------------------------------------------------------------------
    --! MAC REGISTER address width
    constant cMacRegAddrWidth   : natural := 13;
    --! MAC REGISTER data width
    constant cMacRegDataWidth   : natural := 16;

    --! MAC TIMER address width
    constant cMacTimerAddrWidth : natural := 4;
    --! MAC TIMER data width
    constant cMacTimerDataWidth : natural := 32;

    --! MAC PACKET BUFFER data width
    constant cPktBufDataWidth   : natural := 32;

    --! MAC TIME width
    constant cMacTimeWidth      : natural := 32;

    ---------------------------------------------------------------------------
    -- Constants for activity blinking
    ---------------------------------------------------------------------------
    --! The activity blink frequency [Hz]
    constant cActivityFreq  : natural := 6;
    --! Clock frequency of iClk [Hz]
    constant cClkFreq       : natural := 50e6;

    ---------------------------------------------------------------------------
    -- Constants for openhub
    ---------------------------------------------------------------------------
    --! Internal port number
    constant cHubIntPort        : natural := 1;

    ---------------------------------------------------------------------------
    -- Interrupt table
    ---------------------------------------------------------------------------
    --! Interrupt table subtype
    subtype tMacRegIrqTable is std_logic_vector(cMacRegDataWidth-1 downto 0);
    --! MAC Tx interrupt offset
    constant cMacRegIrqTable_macTx  : natural := 0;
    --! MAC Rx interrupt offset
    constant cMacRegIrqTable_macRx  : natural := 1;

    ---------------------------------------------------------------------------
    -- DMA Error table
    ---------------------------------------------------------------------------
    --! DMA error table subtype
    subtype tMacDmaErrorTable is std_logic_vector(cMacRegDataWidth-1 downto 0);
    --! DMA error write (Rx packet transfer)
    constant cMacDmaErrorTable_write    : natural := 0;
    --! DMA error read (Tx packet transfer)
    constant cMacDmaErrorTable_read     : natural := 8;
end package openmacPkg;

package body openmacPkg is
    --! Functio to get tRmiiPathArray enables.
    function rmiiGetEnable ( iArray : tRmiiPathArray ) return std_logic_vector is
        variable vRes_tmp : std_logic_vector(iArray'range);
    begin
        vRes_tmp := (others => cInactivated);

        for i in iArray'range loop
            vRes_tmp(i) := iArray(i).enable;
        end loop;

        return vRes_tmp;
    end function;

    --! Procedure to convert tRmiiPathArray to std_logic_vector.
    procedure rmiiPathArrayToStdLogicVector (
        signal iVector      : in    tRmiiPathArray;
        signal oEnable      : out   std_logic_vector;
        signal oData        : out   std_logic_vector
    ) is
        variable vVector_tmp : tRmiiPathArray(iVector'length-1 downto 0);
    begin
        vVector_tmp := iVector;

        for i in vVector_tmp'range loop
            oEnable(i) <= vVector_tmp(i).enable;

            for j in cRmiiDataWidth-1 downto 0 loop
                oData(i*cRmiiDataWidth+j) <= vVector_tmp(i).data(j);
            end loop;
        end loop;
    end procedure;

    --! Procedure to convert std_logic_vector to tRmiiPathArray
    procedure stdLogicVectorToRmiiPathArray (
        signal iEnable      : in    std_logic_vector;
        signal iData        : in    std_logic_vector;
        signal oVector      : out   tRmiiPathArray
    ) is
        variable vVector_tmp : tRmiiPathArray(iEnable'length-1 downto 0);
    begin
        for i in vVector_tmp'range loop
            vVector_tmp(i).enable := iEnable(i);

            for j in cRmiiDataWidth-1 downto 0 loop
                vVector_tmp(i).data(j)  := iData(i*cRmiiDataWidth+j);
            end loop;
        end loop;

        oVector <= vVector_tmp;
    end procedure;

        --! Procedure to convert tMiiPathArray to std_logic_vector.
    procedure miiPathArrayToStdLogicVector (
        signal iVector      : in    tMiiPathArray;
        signal oEnable      : out   std_logic_vector;
        signal oData        : out   std_logic_vector
    ) is
        variable vVector_tmp : tMiiPathArray(iVector'length-1 downto 0);
    begin
        vVector_tmp := iVector;

        for i in vVector_tmp'range loop
            oEnable(i)      <= vVector_tmp(i).enable;

            for j in cMiiDataWidth-1 downto 0 loop
                oData(i*cMiiDataWidth+j) <= vVector_tmp(i).data(j);
            end loop;
        end loop;
    end procedure;

    --! Procedure to convert std_logic_vector to tMiiPathArray
    procedure stdLogicVectorToMiiPathArray (
        signal iEnable      : in    std_logic_vector;
        signal iData        : in    std_logic_vector;
        signal oVector      : out   tMiiPathArray
    ) is
        variable vVector_tmp : tMiiPathArray(iEnable'length-1 downto 0);
    begin
        for i in vVector_tmp'range loop
            vVector_tmp(i).enable   := iEnable(i);

            for j in cMiiDataWidth-1 downto 0 loop
                vVector_tmp(i).data(j) := iData(i*cMiiDataWidth+j);
            end loop;
        end loop;

        oVector <= vVector_tmp;
    end procedure;
end package body openmacPkg;
