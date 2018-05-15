-------------------------------------------------------------------------------
-- Global package
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
--
-------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package Global is

    constant cActivated     : std_logic := '1';
    constant cInactivated   : std_logic := '0';

    constant cnActivated    : std_logic := '0';
    constant cnInactivated  : std_logic := '1';

    constant cByteLength    : natural := 8;
    constant cWordLength    : natural := 2 * cByteLength;

    constant cFalse         : natural := 0;
    constant cTrue          : natural := 1;

    function LogDualis(cNumber : natural) return natural;

    function maximum (a : natural; b : natural) return natural;
    function minimum (a : natural; b : natural) return natural;

    function integerToBoolean (a : integer) return boolean;
    function booleanToInteger (a : boolean) return integer;

    function byteSwap (iVector : std_logic_vector) return std_logic_vector;
    function wordSwap (iVector : std_logic_vector) return std_logic_vector;

    function reduceOr (iVector : std_logic_vector) return std_logic;
    function reduceAnd (iVector : std_logic_vector) return std_logic;

end Global;

package body Global is

    function LogDualis(cNumber : natural) return natural is
        variable vClimbUp : natural := 1;
        variable vResult  : natural := 0;
    begin
        while vClimbUp < cNumber loop
            vClimbUp := vClimbUp * 2;
            vResult  := vResult+1;
        end loop;
        return vResult;
    end LogDualis;

    function maximum (a : natural; b : natural) return natural is
        variable vRes : natural;
    begin

        if a > b then
            vRes := a;
        else
            vRes := b;
        end if;

        return vRes;

    end function;

    function minimum (a : natural; b : natural) return natural is
        variable vRes : natural;
    begin

        if a < b then
            vRes := a;
        else
            vRes := b;
        end if;

        return vRes;

    end function;

    function integerToBoolean (a : integer) return boolean is
        variable vRes : boolean;
    begin
        if a = cFalse then
            vRes := false;
        else
            vRes := true;
        end if;

        return vRes;
    end function;

    function booleanToInteger (a : boolean) return integer is
        variable vRes : integer;
    begin
        if a = false then
            vRes := cFalse;
        else
            vRes := cTrue;
        end if;

        return vRes;
    end function;

    function byteSwap (iVector : std_logic_vector) return std_logic_vector is
        variable vResult        : std_logic_vector(iVector'range);
        variable vLeftIndex     : natural;
        variable vRightIndex    : natural;
    begin
        assert ((iVector'length mod cByteLength) = 0)
        report "Byte swapping can't be done with that vector!"
        severity failure;

        for i in iVector'length / cByteLength downto 1 loop
            vLeftIndex := i;
            vRightIndex := iVector'length / cByteLength - i + 1;
            vResult(vLeftIndex * cByteLength - 1 downto (vLeftIndex-1) * cByteLength) :=
            iVector(vRightIndex * cByteLength - 1 downto (vRightIndex-1) * cByteLength);
        end loop;

        return vResult;
    end function;

    function wordSwap (iVector : std_logic_vector) return std_logic_vector is
        variable vResult        : std_logic_vector(iVector'range);
        variable vLeftIndex     : natural;
        variable vRightIndex    : natural;
    begin
        assert ((iVector'length mod cWordLength) = 0)
        report "Word swapping can't be done with that vector!"
        severity failure;

        for i in iVector'length / cWordLength downto 1 loop
            vLeftIndex := i;
            vRightIndex := iVector'length / cWordLength - i + 1;
            vResult(vLeftIndex * cWordLength - 1 downto (vLeftIndex-1) * cWordLength) :=
            iVector(vRightIndex * cWordLength - 1 downto (vRightIndex-1) * cWordLength);
        end loop;

        return vResult;
    end function;

    function reduceOr (iVector : std_logic_vector) return std_logic is
        variable vRes_tmp : std_logic;
    begin
        -- initialize result variable
        vRes_tmp := cInactivated;

        for i in iVector'range loop
            vRes_tmp := vRes_tmp or iVector(i);
        end loop;

        return vRes_tmp;
    end function;

    function reduceAnd (iVector : std_logic_vector) return std_logic is
        variable vRes_tmp : std_logic;
    begin
        -- initialize result variable
        vRes_tmp := cActivated;

        for i in iVector'range loop
            vRes_tmp := vRes_tmp and iVector(i);
        end loop;

        return vRes_tmp;
    end function;

end Global;