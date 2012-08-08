------------------------------------------------------------------------------------------------------------------------
-- Parallel port (8/16bit) for PDI
--
-- 	  Copyright (C) 2010 B&R
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
-- 2010-08-31  	V0.01	zelenkaj    First version
-- 2010-10-18	V0.02	zelenkaj	added selection Big/Little Endian
--									use bidirectional data bus
-- 2010-11-15	V0.03	zelenkaj	bug fix for 16bit parallel interface
-- 2010-11-23	V0.04	zelenkaj	added 2 GPIO pins driving "00"
-- 2010-11-29	V0.05	zelenkaj	full endianness consideration
-- 2011-03-21	V0.06	zelenkaj	clean up
-- 2011-04-04	V0.10	zelenkaj	change of concept
-- 2011-12-02	V0.11	zelenkaj	Added I, O and T instead of IO ports
------------------------------------------------------------------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.std_logic_arith.all;
USE ieee.std_logic_unsigned.all;

entity pdi_par is
	generic (
		papDataWidth_g				:		integer := 8;
		papBigEnd_g					:		boolean := false; --deprecated
		papGenIoBuf_g				:		boolean := true
	);
			
	port (   
			-- 8/16bit parallel
			pap_cs						: in    std_logic;
			pap_rd						: in    std_logic;
			pap_wr 						: in    std_logic;
			pap_be						: in    std_logic_vector(papDataWidth_g/8-1 downto 0);
			pap_addr 					: in    std_logic_vector(15 downto 0);
			pap_data					: inout	std_logic_vector(papDataWidth_g-1 downto 0);
			pap_data_I					: in 	std_logic_vector(papDataWidth_g-1 downto 0) := (others => '0');
			pap_data_O					: out	std_logic_vector(papDataWidth_g-1 downto 0);
			pap_data_T					: out	std_logic;
			pap_ack						: out	std_logic;
		-- clock for AP side
			ap_reset					: in    std_logic;
			ap_clk						: in	std_logic;
		-- Avalon Slave Interface for AP
            ap_chipselect               : out	std_logic;
            ap_read						: out	std_logic;
            ap_write					: out	std_logic;
            ap_byteenable             	: out	std_logic_vector(3 DOWNTO 0);
            ap_address                  : out	std_logic_vector(12 DOWNTO 0);
            ap_writedata                : out	std_logic_vector(31 DOWNTO 0);
            ap_readdata                 : in	std_logic_vector(31 DOWNTO 0);
		-- GPIO
			pap_gpio					: inout	std_logic_vector(1 downto 0);
			pap_gpio_I					: in 	std_logic_vector(1 downto 0) := (others => '0');
			pap_gpio_O					: out	std_logic_vector(1 downto 0);
			pap_gpio_T					: out	std_logic_vector(1 downto 0)
	);
end entity pdi_par;

architecture rtl of pdi_par is
	signal ap_byteenable_s				:		std_logic_vector(ap_byteenable'range);
	signal ap_write_s 					: 		std_logic;
	signal pap_gpiooe_s					:		std_logic_vector(pap_gpio'range);
	--signals being sync'd to ap_clk
	signal pap_wrdata_s					:		std_logic_vector(pap_data'range);
	signal pap_wrdata_ss				:		std_logic_vector(pap_data'range);
	signal pap_rddata_s					:		std_logic_vector(pap_data'range);
	signal pap_rddata_ss				:		std_logic_vector(pap_data'range);
	signal pap_addr_s					:		std_logic_vector(pap_addr'range);
	signal pap_cs_s						:		std_logic;
	signal pap_rd_s						:		std_logic; --and with cs
	signal pap_wr_s						:		std_logic; --and with cs
	signal pap_be_s						:		std_logic_vector(pap_be'range);
	--write register
	signal writeRegister				:		std_logic_vector(pap_data'range);
	--data tri state buffer
	signal pap_doe_s					:		std_logic;
	signal tsb_cnt, tsb_cnt_next		:		std_logic_vector(1 downto 0);
	
	signal ap_address_write, ap_address_write_l : std_logic_vector(ap_address'range);
	signal ap_byteenable_write, ap_byteenable_write_l : std_logic_vector(ap_byteenable'range);
	signal ap_address_read : std_logic_vector(ap_address'range);
	signal ap_byteenable_read : std_logic_vector(ap_byteenable'range);
    
begin
	
	--reserved for further features not yet defined
	genIoGpBuf : if papGenIoBuf_g generate
	begin
		pap_gpio <= "00" when pap_gpiooe_s = "11" else (others => 'Z');
	end generate;
	
	pap_gpiooe_s <= (others => '1');
	
	pap_gpio_O <= "00";
	pap_gpio_T <= not pap_gpiooe_s; --'1' = In, '0' = Out
	
	-------------------------------------------------------------------------------------
	-- tri-state buffer
	genIoDatBuf : if papGenIoBuf_g generate
	begin
		pap_data <= pap_rddata_s when pap_doe_s = '1' else (others => 'Z');
	end generate;
	
	pap_data_O <= pap_rddata_s;
	pap_data_T <= not pap_doe_s; --'1' = In, '0' = Out
	
	-- write data register
	-- latches data at falling edge of pap_wr
	theWrDataReg : process(pap_wr, ap_reset)
	begin
		if ap_reset = '1' then
			writeRegister <= (others => '0');
		elsif pap_wr = '0' and pap_wr'event then
				if papGenIoBuf_g then
					writeRegister <= pap_data;
				else
					writeRegister <= pap_data_I;
				end if;
		end if;
	end process;
	--
	-------------------------------------------------------------------------------------
    
    -------------------------------------------------------------------------------------
    -- store addr and be for write access
    -- note: this reduces the address hold time to zero
	--
	addrStore : process(ap_clk, ap_reset)
	begin
		if ap_reset = '1' then
			ap_address_write_l <= (others => '0');
			ap_byteenable_write_l <= (others => '0');
			ap_address_write <= (others => '0');
			ap_byteenable_write <= (others => '0');
		elsif ap_clk = '1' and ap_clk'event then
			if pap_cs_s = '1' then
				ap_address_write_l <= pap_addr_s(ap_address'left+2 downto 2);
				ap_byteenable_write_l <= ap_byteenable_s;
			end if;
			ap_address_write <= ap_address_write_l;
			ap_byteenable_write <= ap_byteenable_write_l;
		end if;
	end process;
	
	ap_address_read <= pap_addr_s(ap_address'left+2 downto 2);
	ap_byteenable_read <= ap_byteenable_s;
	
	ap_address <= 	ap_address_write when ap_write_s = '1' else
						ap_address_read;
	
	ap_byteenable <= ap_byteenable_write when ap_write_s = '1' else
						ap_byteenable_read;
    --
    -------------------------------------------------------------------------------------
    
	-------------------------------------------------------------------------------------
	-- generate write and read strobes and chipselect
	-- note: pap_cs_s is already and'd with pap_rd_s and pap_wr_s
		
	--falling edge latches write data, sync'd write strobe  falls too
	wrEdgeDet : entity work.edgeDet
		port map (
			din => pap_wr_s,
			rising => open,
			falling => ap_write_s,
			any => open,
			clk => ap_clk,
			rst => ap_reset
		);
	ap_write <= ap_write_s;
	
	--use the timeout counter highest bit
	ap_read <= pap_rd_s and not ap_write_s;
	
	ap_chipselect <= (pap_cs_s and pap_rd_s) or ap_write_s;
	--
	-------------------------------------------------------------------------------------
	
	-------------------------------------------------------------------------------------
	-- generate ack signal
	pap_ack <= pap_doe_s or ap_write_s;
	--
	-------------------------------------------------------------------------------------
	
	-------------------------------------------------------------------------------------
	-- generate output enable signal for tri state buffer (with timeout)
	
	pap_doe_s <= tsb_cnt(tsb_cnt'left) and pap_rd_s;
	
	triStatBufCnt : process(ap_clk, ap_reset)
	begin
		if ap_reset = '1' then
			tsb_cnt <= (others => '0');
		elsif ap_clk = '1' and ap_clk'event then
			tsb_cnt <= tsb_cnt_next;
		end if;
	end process;
	
	tsb_cnt_next <= tsb_cnt when pap_doe_s = '1' else
					tsb_cnt + 1 when pap_rd_s = '1' else
					(others => '0');
	--
	-------------------------------------------------------------------------------------
	
	-------------------------------------------------------------------------------------
	-- generate 8 or 16 bit signals
	gen8bitSigs : if papDataWidth_g = 8 generate
		
		ap_byteenable_s <= 	"0001" when pap_addr_s(1 downto 0) = "00" else
							"0010" when pap_addr_s(1 downto 0) = "01" else
							"0100" when pap_addr_s(1 downto 0) = "10" else
							"1000" when pap_addr_s(1 downto 0) = "11" else
							(others => '0');
		
		ap_writedata <=		pap_wrdata_s & pap_wrdata_s & pap_wrdata_s & pap_wrdata_s;
		
		pap_rddata_s <=		ap_readdata( 7 downto  0) when ap_byteenable_s = "0001" else
							ap_readdata(15 downto  8) when ap_byteenable_s = "0010" else
							ap_readdata(23 downto 16) when ap_byteenable_s = "0100" else
							ap_readdata(31 downto 24) when ap_byteenable_s = "1000" else
							(others => '0');
		
	end generate gen8bitSigs;
	
	genBeSigs16bit : if papDataWidth_g = 16 generate
		
		ap_byteenable_s <=	"0001" when pap_addr_s(1 downto 1) = "0" and pap_be_s = "01" else
							"0010" when pap_addr_s(1 downto 1) = "0" and pap_be_s = "10" else
							"0011" when pap_addr_s(1 downto 1) = "0" and pap_be_s = "11" else
							"0100" when pap_addr_s(1 downto 1) = "1" and pap_be_s = "01" else
							"1000" when pap_addr_s(1 downto 1) = "1" and pap_be_s = "10" else
							"1100" when pap_addr_s(1 downto 1) = "1" and pap_be_s = "11" else
							(others => '0');
		
--		ap_byteenable <= 	ap_byteenable_s;
		pap_wrdata_ss <=	pap_wrdata_s;
		
		ap_writedata <=		pap_wrdata_ss & pap_wrdata_ss;
		
		pap_rddata_ss <=	ap_readdata( 7 downto  0) & ap_readdata( 7 downto  0) when ap_byteenable_s = "0001" else
							ap_readdata(15 downto  8) & ap_readdata(15 downto  8) when ap_byteenable_s = "0010" else
							ap_readdata(15 downto  0) when ap_byteenable_s = "0011" else
							ap_readdata(23 downto 16) & ap_readdata(23 downto 16) when ap_byteenable_s = "0100" else
							ap_readdata(31 downto 24) & ap_readdata(31 downto 24) when ap_byteenable_s = "1000" else
							ap_readdata(31 downto 16) when ap_byteenable_s = "1100" else
							(others => '0');
		
		pap_rddata_s <=		pap_rddata_ss;
		
	end generate genBeSigs16bit;
	--
	-------------------------------------------------------------------------------------
	
	-------------------------------------------------------------------------------------
	--sync those signals

	syncAddrGen : for i in pap_addr'range generate
		syncAddr : entity work.sync
			port map (
				din => pap_addr(i),
				dout => pap_addr_s(i),
				clk => ap_clk,
				rst => ap_reset
			);
	end generate;
	
	syncBeGen : for i in pap_be'range generate
		syncBe : entity work.sync
			port map (
				din => pap_be(i),
				dout => pap_be_s(i),
				clk => ap_clk,
				rst => ap_reset
			);
	end generate;
	
	syncWrRegGen : for i in writeRegister'range generate
		syncWrReg : entity work.sync
			port map (
				din => writeRegister(i),
				dout => pap_wrdata_s(i),
				clk => ap_clk,
				rst => ap_reset
			);
	end generate;
	
	theMagicBlock : block
		signal pap_rd_tmp, pap_wr_tmp, pap_cs_tmp : std_logic;
	begin
		syncCs : entity work.sync
			port map (
				din => pap_cs,
				dout => pap_cs_tmp,
				clk => ap_clk,
				rst => ap_reset
			);
		pap_cs_s <= pap_cs_tmp;
		
		syncRd : entity work.sync
			port map (
				din => pap_rd,
				dout => pap_rd_tmp,
				clk => ap_clk,
				rst => ap_reset
			);
		pap_rd_s <= pap_rd_tmp and pap_cs_tmp;
		
		syncWr : entity work.sync
			port map (
				din => pap_wr,
				dout => pap_wr_tmp,
				clk => ap_clk,
				rst => ap_reset
			);
        pap_wr_s <= pap_wr_tmp;
		
	end block;
	--
	-------------------------------------------------------------------------------------
	
end architecture rtl;
