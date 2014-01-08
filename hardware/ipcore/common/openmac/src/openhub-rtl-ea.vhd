-------------------------------------------------------------------------------
--! @file openhub-rtl-ea.vhd
--
--! @brief OpenHUB
--
--! @details This is the openHUB using RMII Rx and Tx lines.
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
--! use openmac package
use work.openmacPkg.all;

entity openhub is
    generic (
        --! Number of ports
        gPortCount   :   integer := 3
    );
    port (
        --! Reset
        iRst        : in    std_logic;
        --! RMII Clock
        iClk        : in    std_logic;
        --! RMII receive paths
        iRx         : in    tRmiiPathArray(gPortCount downto 1);
        --! RMII transmit paths
        oTx         : out   tRmiiPathArray(gPortCount downto 1);
        --! Determine number of internal port (to MAC)
        iIntPort    : in    integer range 1 to gPortCount := 1;
        --! Transmit mask to enable ports
        iTxMask     : in    std_logic_vector(gPortCount downto 1) := (others => cActivated);
        --! Gives the number of the currectly receiving port
        oRxPort     : out   integer range 0 to gPortCount
    );
end entity openhub;

architecture rtl of openhub is
    --! All ports inactive constant
    constant cPortsAreInactive  : std_logic_vector(gPortCount downto 0) := (others => cInactivated);

    --! Receive path array
    signal rxPath           : tRmiiPathArray(gPortCount downto 0);
    --! Receive path array delayed by one cycle
    signal rxPath_l         : tRmiiPathArray(gPortCount downto 0);
    --! Transmit path array
    signal txPath           : tRmiiPathArray(gPortCount downto 0);
    --! Stored transmit mask (is taken from iTxMask when to packet transfer is in progress)
    signal txMask_reg   : std_logic_vector(gPortCount downto 1);
begin
    rxPath  <= iRx & cRmiiPathInit;
    oTx     <= txPath(oTx'range);

    do: process (iRst, iClk)
        variable vActive            : boolean;
        variable vMaster            : integer range 0 to gPortCount;
        variable vMasterAtCollision : integer range 0 to gPortCount;
        variable vCollision         : boolean;
        variable vRxDvm             : std_logic_vector(gPortCount downto 0);
    begin
        if iRst = cActivated then
            rxPath_l            <= (others => cRmiiPathInit);
            txPath              <= (others => cRmiiPathInit);
            vActive             := false;
            vMaster             := 0;
            vMasterAtCollision  := 0;
            vCollision          := false;
            txMask_reg          <= (others => cInactivated);
        elsif rising_edge(iClk) then
            rxPath_l <= rxPath;

            if vActive = false then
                if rmiiGetEnable(rxPath_l) /= cPortsAreInactive then
                    for i in 1 to gPortCount loop
                        if (rxPath_l(i).enable = cActivated and
                            (rxPath_l(i).data(0) = cActivated or rxPath_l(i).data(1) = cActivated)) then
                            vMaster := i;
                            vActive := true;
                            exit;
                        end if;
                    end loop;
                end if;
            else
                if rxPath_l(vMaster).enable = cInactivated and rxPath(vMaster).enable = cInactivated then
                    vMaster := 0;
                end if;
                if rmiiGetEnable(rxPath_l) = cPortsAreInactive and rmiiGetEnable(rxPath) = cPortsAreInactive then
                    vActive := false;
                end if;
            end if;

            if vMaster = 0 then
                txPath <= (others => cRmiiPathInit);

                -- overtake new iTxMask only, when there is no active frame.
                txMask_reg <= iTxMask;
            else
                for i in 1 to gPortCount loop -- output received frame to every port
                    if i /= vMaster then  --  but not to the port where it is coming from - "eh kloar!"

                        -- only send data to active ports (=> iTxMask is set to cActivated) or the internal port (mac)
                        if txMask_reg(i) = cActivated or vMaster = iIntPort then
                            txPath(i).enable    <= cActivated;
                            txPath(i).data      <= rxPath_l(vMaster).data;
                        end if;

                        -- if there is a frame received and another is sent => collision!
                        if rxPath_l(i).enable = cActivated then
                            vCollision          := true;
                            vMasterAtCollision  := vMaster;
                        end if;
                    end if;
                end loop;
            end if;

            if vCollision = true then
                txPath(vMasterAtCollision).enable   <= cActivated;
                txPath(vMasterAtCollision).data     <= "01";
                vRxDvm                              := rmiiGetEnable(rxPath_l);
                vRxDvm(vMasterAtCollision)          := cInactivated;

                if vRxDvm = cPortsAreInactive then
                    txPath(vMasterAtCollision)      <= cRmiiPathInit;
                    vCollision                      := false;
                    vMasterAtCollision              := 0;
                end if;
            end if;

            -- output the master port - identifies the port (1...n) which has received the packet.
            -- if master is 0, the hub is inactive.
            oRxPort <= vMaster;
        end if;
    end process do;
end rtl;

