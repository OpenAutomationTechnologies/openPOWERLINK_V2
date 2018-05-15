-------------------------------------------------------------------------------
--! @file prlMaster-rtl-ea.vhd
--! @brief Multiplexed memory mapped master
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

--! Use standard ieee library
library ieee;
--! Use logic elements
use ieee.std_logic_1164.all;
--! Use numeric std
use ieee.numeric_std.all;

--! Use libcommon library
library libcommon;
--! Use global package
use libcommon.global.all;

entity prlMaster is
    generic (
        --! Enable multiplexed address/data-bus mode (0 = FALSE)
        gEnableMux      : natural := 0;
        --! Data bus width
        gDataWidth      : natural := 16;
        --! Address bus width
        gAddrWidth      : natural := 16;
        --! Address low
        gAddrLow        : natural := 0;
        --! Ad bus width (valid when gEnableMux /= FALSE)
        gAdWidth        : natural := 16
    );
    port (
        --! Clock
        iClk                : in    std_logic;
        --! Reset
        iRst                : in    std_logic;
        -- Memory mapped slave
        --! Address
        iSlv_address        : in    std_logic_vector(gAddrWidth-1 downto gAddrLow);
        --! Read strobe
        iSlv_read           : in    std_logic;
        --! Readdata
        oSlv_readdata       : out   std_logic_vector(gDataWidth-1 downto 0);
        --! Write strobe
        iSlv_write          : in    std_logic;
        --! Writedata
        iSlv_writedata      : in    std_logic_vector(gDataWidth-1 downto 0);
        --! Waitrequest
        oSlv_waitrequest    : out   std_logic;
        --! Byteenable
        iSlv_byteenable     : in    std_logic_vector(gDataWidth/8-1 downto 0);
        -- Memory mapped multiplexed master
        --! Chipselect
        oPrlMst_cs          : out   std_logic;
        -- Multiplexed AD-bus
        --! Multiplexed address data bus input
        iPrlMst_ad_i       : in     std_logic_vector(gAdWidth-1 downto 0);
        --! Multiplexed address data bus output
        oPrlMst_ad_o       : out    std_logic_vector(gAdWidth-1 downto 0);
        --! Multiplexed address data bus enable
        oPrlMst_ad_oen     : out    std_logic;
        -- Demultiplexed AD-bus
        --! Address bus
        oPrlMst_addr        : out   std_logic_vector(gAddrWidth-1 downto 0);
        --! Data bus in
        iPrlMst_data_i      : in    std_logic_vector(gDataWidth-1 downto 0);
        --! Data bus out
        oPrlMst_data_o      : out   std_logic_vector(gDataWidth-1 downto 0);
        --! Data bus outenable
        oPrlMst_data_oen    : out   std_logic;
        --! Byteenable
        oPrlMst_be          : out   std_logic_vector(gDataWidth/8-1 downto 0);
        --! Address latch enable
        oPrlMst_ale         : out   std_logic;
        --! Write strobe
        oPrlMst_wr          : out   std_logic;
        --! Read strobe
        oPrlMst_rd          : out   std_logic;
        --! Acknowledge
        iPrlMst_ack         : in    std_logic
    );
end entity prlMaster;

architecture rtl of prlMaster is
    constant cCount_AleDisable  : std_logic_vector := "011";
    constant cCount_AleExit     : std_logic_vector := "101";
    constant cCount_max         : std_logic_vector := "111";
    constant cCountWidth        : natural := cCount_max'length;

    -- State machine for bus timing
    type tFsm is (
        sIdle,
        sAle,
        sWrd,
        sHold
    );

    -- Synchronized ack signal
    signal ack              : std_logic;
    -- Rising edge of ack signal
    signal ack_p            : std_logic;

    --! This record holds all output registers to the bus.
    type tReg is record
        address     : std_logic_vector(gAddrWidth-1 downto 0);
        byteenable  : std_logic_vector(gDataWidth/8-1 downto 0);
        write       : std_logic;
        read        : std_logic;
        chipselect  : std_logic;
        data        : std_logic_vector(gDataWidth-1 downto 0);
        data_oen    : std_logic;
        data_in     : std_logic_vector(gDataWidth-1 downto 0);
        ad          : std_logic_vector(gAdWidth-1 downto 0);
        ad_oen      : std_logic;
        ale         : std_logic;
        fsm         : tFsm;
        count       : std_logic_vector(cCountWidth-1 downto 0);
        count_rst   : std_logic;
    end record;

    -- Initialization vector of output registers
    constant cRegInit : tReg := (
        address     => (others => cInactivated),
        byteenable  => (others => cInactivated),
        write       => cInactivated,
        read        => cInactivated,
        chipselect  => cInactivated,
        data        => (others => cInactivated),
        data_oen    => cInactivated,
        data_in     => (others => cInactivated),
        ad          => (others => cInactivated),
        ad_oen      => cInactivated,
        ale         => cInactivated,
        fsm         => sIdle,
        count       => (others => cInactivated),
        count_rst   => cInactivated
    );

    -- Register state
    signal reg      : tReg;
    -- Next register state
    signal reg_next : tReg;
begin

    -- MAP IOs
    oSlv_waitrequest    <= not ack_p;
    oSlv_readdata       <= reg.data_in;

    oPrlMst_be          <= reg.byteenable;
    oPrlMst_wr          <= reg.write;
    oPrlMst_rd          <= reg.read;
    oPrlMst_cs          <= reg.chipselect;

    --! Generate mux bus IOs. Demux bus is incactive.
    genMux : if gEnableMux /= 0 generate
        -- MUX
        oPrlMst_ale         <= reg.ale;
        oPrlMst_ad_o        <= reg.ad;
        oPrlMst_ad_oen      <= reg.ad_oen;

        oPrlMst_addr        <= (others => cInactivated);
        oPrlMst_data_o      <= (others => cInactivated);
        oPrlMst_data_oen    <= cInactivated;
        -- iPrlMst_data_i is ignored
    end generate genMux;

    --! Generate demux bus IOs. Mux bus is incactive.
    genDemux : if gEnableMux = 0 generate
        -- DEMUX
        oPrlMst_addr        <= reg.address;
        oPrlMst_data_o      <= reg.data;
        oPrlMst_data_oen    <= reg.data_oen;

        oPrlMst_ale         <= cInactivated;
        oPrlMst_ad_o        <= (others => cInactivated);
        oPrlMst_ad_oen      <= cInactivated;
        -- iPrlMst_ad_i is ignored
    end generate genDemux;

    --! This is the clock register process.
    regClk : process(iRst, iClk)
    begin
        if iRst = cActivated then
            reg <= cRegInit;
        elsif rising_edge(iClk) then
            reg <= reg_next;
        end if;
    end process regClk;

    --! This is the next register state process.
    combReg : process (
        reg, ack,
        iSlv_read,
        iSlv_write,
        iSlv_byteenable,
        iSlv_address,
        iSlv_writedata,
        iPrlMst_ad_i,
        iPrlMst_data_i
    )
    begin
        -- default
        reg_next    <= reg;

        -- counter reset active by default
        reg_next.count_rst  <= cActivated;

        if reg.count_rst = cActivated then
            reg_next.count <= (others => cInactivated);
        else
            reg_next.count <= std_logic_vector(unsigned(reg.count) + 1);
        end if;

        case reg.fsm is
            when sIdle =>
                reg_next.chipselect         <= cInactivated;
                reg_next.ale                <= cInactivated;
                reg_next.ad_oen             <= cInactivated;
                reg_next.data_oen           <= cInactivated;
                reg_next.read               <= cInactivated;
                reg_next.write              <= cInactivated;

                -- Start transaction if there is either a read or write.
                if iSlv_write = cActivated or iSlv_read = cActivated then
                    reg_next.chipselect     <= cActivated;
                    reg_next.byteenable     <= iSlv_byteenable;
                    if gEnableMux /= 0 then
                        -- MUX mode
                        reg_next.fsm        <= sAle;
                        reg_next.ale        <= cActivated;
                        reg_next.ad_oen     <= cActivated;
                        reg_next.ad         <= (others => cInactivated);
                        reg_next.ad(iSlv_address'range) <= iSlv_address;
                    else
                        -- DEMUX mode
                        reg_next.fsm        <= sWrd;
                        reg_next.write      <= iSlv_write;
                        reg_next.read       <= iSlv_read;
                        reg_next.data       <= iSlv_writedata;
                        reg_next.data_oen   <= iSlv_write;
                        reg_next.address    <= iSlv_address;
                    end if;
                end if;
            when sAle =>
                -- Use counter to generate ale timing.
                reg_next.count_rst          <= cInactivated;

                if reg.count = cCount_AleDisable then
                    reg_next.ale            <= cInactivated;
                elsif reg.count = cCount_AleExit then
                    reg_next.count_rst      <= cActivated;
                    reg_next.fsm            <= sWrd;
                    reg_next.write          <= iSlv_write;
                    reg_next.read           <= iSlv_read;
                    reg_next.ad_oen         <= iSlv_write;
                    reg_next.ad             <= (others => cInactivated);
                    reg_next.ad(iSlv_writedata'range) <= iSlv_writedata;
                end if;
            when sWrd =>
                if ack = cActivated then
                    reg_next.fsm            <= sHold;
                    reg_next.count_rst      <= cActivated;
                    reg_next.chipselect     <= cInactivated;
                    reg_next.read           <= cInactivated;
                    reg_next.write          <= cInactivated;
                    reg_next.ad_oen         <= cInactivated;
                    reg_next.data_oen       <= cInactivated;

                    if reg.read = cActivated then
                        if gEnableMux /= 0 then
                            reg_next.data_in <= iPrlMst_ad_i(reg.data_in'range);
                        else
                            reg_next.data_in <= iPrlMst_data_i(reg.data_in'range);
                        end if;
                    end if;
                end if;
            when sHold =>
                if ack = cInactivated then
                    reg_next.fsm            <= sIdle;
                    reg_next.count_rst      <= cActivated;
                end if;
        end case;
    end process combReg;

    --! Synchronizer to sync ack input.
    syncAck : entity libcommon.synchronizer
    generic map (
        gStages => 2,
        gInit   => cInactivated
    )
    port map (
        iArst   => iRst,
        iClk    => iClk,
        iAsync  => iPrlMst_ack,
        oSync   => ack
    );

    --! Detect rising edge of ack signal to generate waitrequest neg. pulse.
    edgeAck : entity libcommon.edgedetector
        port map (
            iArst       => iRst,
            iClk        => iClk,
            iEnable     => cActivated,
            iData       => ack,
            oRising     => ack_p,
            oFalling    => open,
            oAny        => open
        );
end architecture rtl;
