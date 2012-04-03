-------------------------------------------------------------------------------
--
-- Title       : sync
-- Design      : plk_mn
--
-------------------------------------------------------------------------------
--
-- File        : edgedet.vhd
-- Generated   : Wed Jul 27 09:33:40 2011
-- From        : interface description file
-- By          : Itf2Vhdl ver. 1.22
--
-------------------------------------------------------------------------------
--
--    (c) B&R, 2011
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
--
-- 2011-07-26  	V0.01	zelenkaj    First version
-- 2011-11-29	V0.02	zelenkaj	omitted out reset
-- 2011-12-12	V0.03	zelenkaj	reduced to one FF
--
-------------------------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.std_logic_arith.all;
USE ieee.std_logic_unsigned.all;

ENTITY edgeDet IS
	PORT (
			din							: IN	STD_LOGIC;
			rising						: OUT	STD_LOGIC;
			falling						: OUT	STD_LOGIC;
			any							: OUT	STD_LOGIC;
			clk							: IN	STD_LOGIC;
			rst							: IN	STD_LOGIC
	);
END ENTITY edgeDet;

ARCHITECTURE rtl OF edgeDet IS
	signal RegDin : std_logic;
BEGIN
	
	any <= RegDin xor din;
	falling <= RegDin and not din;
	rising <= not RegDin and din;
	
	process(clk)
	begin
		if rising_edge(clk) then
			RegDin <= din after 10 ns;
		end if;
	end process;
	
END ARCHITECTURE rtl;
