-------------------------------------------------------------------------------
-- Entity : busMaster
-------------------------------------------------------------------------------
--
--    (c) B&R, 2012
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

use std.textio.all;

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_textio.all;

library work;
use global.all;

entity busMaster is
    generic (
        gAddrWidth : integer := 32;
        gDataWidth : integer := 32;
        gStimuliFile : string := "name_TB_stim.txt"
    );
    port (
        iRst : in std_logic;
        iClk : in std_logic;
        iEnable : in std_logic;
        iAck : in std_logic;
        iReaddata : in std_logic_vector(gDataWidth-1 downto 0);
        oWrite : out std_logic;
        oRead : out std_logic;
        oSelect : out std_logic;
        oAddress : out std_logic_vector(gAddrWidth-1 downto 0);
        oByteenable : out std_logic_vector(gDataWidth/8-1 downto 0);
        oWritedata : out std_logic_vector(gDataWidth-1 downto 0);
        oDone : out std_logic
    );
end busMaster;

architecture bhv of busMaster is
    function char_to_nib(hex : in character) return std_logic_vector is
        variable tmp : std_logic_vector(3 downto 0);
    begin
        case hex is
            when '0' => tmp := x"0";
            when '1' => tmp := x"1";
            when '2' => tmp := x"2";
            when '3' => tmp := x"3";
            when '4' => tmp := x"4";
            when '5' => tmp := x"5";
            when '6' => tmp := x"6";
            when '7' => tmp := x"7";
            when '8' => tmp := x"8";
            when '9' => tmp := x"9";
            when 'A' => tmp := x"A";
            when 'B' => tmp := x"B";
            when 'C' => tmp := x"C";
            when 'D' => tmp := x"D";
            when 'E' => tmp := x"E";
            when 'F' => tmp := x"F";
            when others => tmp := "XXXX";
        end case;
        return tmp;
    end char_to_nib;    
    
    file stimulifile : text open read_mode is gStimuliFile;
    
    type tFsm is (idle, write, read, jmpEq, jmpEq_wait);
    signal fsm : tFsm;
    
    signal clk : std_logic;
    signal rst : std_logic;
    signal enable : std_logic;
    
    signal stimAddress : std_logic_vector(31 downto 0);
    signal stimData : std_logic_vector(31 downto 0);
    signal stimAccess : character;
    signal byteenable : std_logic_vector(3 downto 0);
begin
    
    clk <= iClk;
    rst <= iRst;
    enable <= iEnable;
    
    oAddress <= stimAddress(oAddress'range) when fsm = write or fsm = read or fsm = jmpEq else (others => 'X');
    oWritedata <=   (others => 'X') when fsm /= write else
                    stimData(stimData'left downto stimData'left-(gDataWidth-1));
    
    oWrite <= cActivated when fsm = write else cInactivated;
    oRead <= cActivated when fsm = read or fsm = jmpEq else cInactivated;
    oSelect <= cActivated when fsm = write or fsm = read or fsm = jmpEq else cInactivated;
    
    oByteenable <=  byteenable when gDataWidth = 32 else
                    byteenable(3 downto 2) or byteenable(1 downto 0) when gDataWidth = 16 else
                    (others => '-');
    
    byteenable <=   "1111" when stimAccess = 'd' else
                    "0011" when stimAccess = 'w' and stimAddress(1) = '0' else
                    "1100" when stimAccess = 'w' and stimAddress(1) = '1' else
                    "0001" when stimAccess = 'b' and stimAddress(1 downto 0) = "00" else
                    "0010" when stimAccess = 'b' and stimAddress(1 downto 0) = "01" else
                    "0100" when stimAccess = 'b' and stimAddress(1 downto 0) = "10" else
                    "1000" when stimAccess = 'b' and stimAddress(1 downto 0) = "11" else
                    "----";
    
    readFile : process(clk, rst)
        variable vLine : line;
        variable vReadString : string(1 to 50);
        variable vDone : boolean := false;
        variable vDoJmp : std_logic;
    begin
        if rst = cActivated then
            fsm <= idle;
            stimAddress <= (others => '0');
            stimData <= (others => '0');
            oDone <= cInactivated;
            vDone := false;
        elsif rising_edge(clk) then
            if enable = cActivated then
                if fsm = idle then
                    if not endfile(stimulifile) and not vDone then
                        loop
                            vReadString := (others => ' ');
                            readline(stimulifile, vLine);
                            read(vLine, vReadString);
                            exit when vReadString(1) /= '#'; --comments
                            exit when vReadString(1) /= ' '; --empty line
                            exit when endfile(stimulifile);
                        end loop;
                        
                        if vReadString(1 to 4) = "HALT" then
                            vDone := true;
                        else
                            
                            case vReadString(1 to 2) is
                                when "WR" =>
                                    fsm <= write;
                                when "RD" =>
                                    fsm <= read;
                                when "JE" =>
                                    fsm <= jmpEq;
                                when "NP" =>
                                    fsm <= idle;
                                when others =>
                            end case;
                            
                            stimAccess <= vReadString(3);
                            
                            for i in 0 to 7 loop
                                stimAddress((8-i)*4-1 downto (7-i)*4) <= char_to_nib(vReadString(i+5));
                            end loop;
                            
                            for i in 0 to 7 loop
                                stimData((8-i)*4-1 downto (7-i)*4) <= char_to_nib(vReadString(i+14));
                            end loop;
                            
                        end if;
                        
                    else
                        oDone <= cActivated;
                    end if;
                elsif fsm = write or fsm = read then
                    if iAck = cActivated then
                        fsm <= idle;
                    end if;
                elsif fsm = jmpEq then
                    if iAck = cActivated then
                        fsm <= jmpEq_wait;
                        vDoJmp := cActivated;
                        for i in iReaddata'range loop
                            if byteenable(i/8) = cActivated then
                                if stimData(i) = iReaddata(i) then
                                    vDoJmp := vDoJmp and cActivated;
                                else
                                    vDoJmp := vDoJmp and cnActivated;
                                end if;
                            end if;
                        end loop;
                        
                        if vDoJmp = cActivated then
                            fsm <= idle;
                        end if;
                        
                    end if;
                elsif fsm = jmpEq_wait then
                    fsm <= jmpEq;
                end if;
            end if;
        end if;
    end process;
    
end bhv;
