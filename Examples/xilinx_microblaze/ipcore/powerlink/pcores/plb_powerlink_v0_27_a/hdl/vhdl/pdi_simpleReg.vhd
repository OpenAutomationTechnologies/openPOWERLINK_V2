------------------------------------------------------------------------------------------------------------------------
-- Process Data Interface (PDI) simple register
--
-- 	  Copyright (C) 2011 B&R
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
------------------------------------------------------------------------------------------------------------------------
-- Version History
------------------------------------------------------------------------------------------------------------------------
-- 2011-09-14  	V0.01	zelenkaj    extract from pdi.vhd
------------------------------------------------------------------------------------------------------------------------
LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.std_logic_arith.all;
USE ieee.std_logic_unsigned.all;

entity pdiSimpleReg is
	generic (
			iAddrWidth_g				:		integer := 10; --only use effective addr range (e.g. 2kB leads to iAddrWidth_g := 10)
			iBaseMap2_g					:		integer := 0; --base address in dpr
			iDprAddrWidth_g				:		integer := 12
	);
			
	port (   
			--memory mapped interface
			sel							: in	std_logic;
			wr							: in	std_logic;
			rd							: in	std_logic;
			addr						: in	std_logic_vector(iAddrWidth_g-1 downto 0);
			be							: in	std_logic_vector(3 downto 0);
			din							: in	std_logic_vector(31 downto 0);
			dout						: out	std_logic_vector(31 downto 0);
			--dpr interface (from PCP/AP to DPR)
			dprAddrOff					: out	std_logic_vector(iDprAddrWidth_g downto 0);
			dprDin						: out	std_logic_vector(31 downto 0);
			dprDout						: in	std_logic_vector(31 downto 0);
			dprBe						: out	std_logic_vector(3 downto 0);
			dprWr						: out	std_logic
			
	);
end entity pdiSimpleReg;

architecture rtl of pdiSimpleReg is
signal addrRes							:		std_logic_vector(dprAddrOff'range);
begin
	
	--assign content to dpr
	dprDin		<=	din;
	dprBe		<=	be;
	dprWr		<=	wr		when	sel = '1'		else
					'0';
	dout		<=	dprDout	when	sel = '1'		else
					(others => '0');
	dprAddrOff	<=	addrRes when	sel = '1'		else
					(others => '0');
	
	--address conversion
	---map external address mapping into dpr
	addrRes <= '0' & conv_std_logic_vector(iBaseMap2_g, addrRes'length - 1);
		
end architecture rtl;