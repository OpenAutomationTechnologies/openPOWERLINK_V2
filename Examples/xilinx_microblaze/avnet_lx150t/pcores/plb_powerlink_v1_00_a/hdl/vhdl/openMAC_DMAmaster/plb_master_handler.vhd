-------------------------------------------------------------------------------
--
-- Title       : plb_master_handler
-- Design      : POWERLINK
--
-------------------------------------------------------------------------------
--
-- File        : c:\my_designs\POWERLINK\src\plb_master_handler.vhd
-- Generated   : Mon Nov  7 13:17:30 2011
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
-- 2011-08-03  	V0.01	zelenkaj    First version
-- 2011-12-01	V0.02	zelenkaj	Fixed read transfer error (dst_rdy_n earlier)
-- 2011-12-05	V0.03	zelenkaj	Avoid preset of FFs
--
-------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;

entity plb_master_handler is
	generic(
		gen_rx_fifo_g : boolean := true;
		gen_tx_fifo_g : boolean := true;
		dma_highadr_g : integer := 31;
		C_MAC_DMA_PLB_NATIVE_DWIDTH : integer := 32;
		C_MAC_DMA_PLB_AWIDTH : integer := 32;
		m_burstcount_width_g : integer := 4
	);
	port(
		MAC_DMA_CLK : in std_logic;
		MAC_DMA_Rst : in std_logic;
		Bus2MAC_DMA_Mst_CmdAck : in std_logic := '0';
		Bus2MAC_DMA_Mst_Cmplt : in std_logic := '0';
		Bus2MAC_DMA_Mst_Error : in std_logic := '0';
		Bus2MAC_DMA_Mst_Rearbitrate : in std_logic := '0';
		Bus2MAC_DMA_Mst_Cmd_Timeout : in std_logic := '0';
		Bus2MAC_DMA_MstRd_d : in std_logic_vector(C_MAC_DMA_PLB_NATIVE_DWIDTH-1 downto 0);
		Bus2MAC_DMA_MstRd_rem : in std_logic_vector(C_MAC_DMA_PLB_NATIVE_DWIDTH/8-1 downto 0);
		Bus2MAC_DMA_MstRd_sof_n : in std_logic := '1';
		Bus2MAC_DMA_MstRd_eof_n : in std_logic := '1';
		Bus2MAC_DMA_MstRd_src_rdy_n : in std_logic := '1';
		Bus2MAC_DMA_MstRd_src_dsc_n : in std_logic := '1';
		Bus2MAC_DMA_MstWr_dst_rdy_n : in std_logic := '1';
		Bus2MAC_DMA_MstWr_dst_dsc_n : in std_logic := '1';
		MAC_DMA2Bus_MstRd_Req : out std_logic := '0';
		MAC_DMA2Bus_MstWr_Req : out std_logic := '0';
		MAC_DMA2Bus_Mst_Type : out std_logic := '0';
		MAC_DMA2Bus_Mst_Addr : out std_logic_vector(C_MAC_DMA_PLB_AWIDTH-1 downto 0);
		MAC_DMA2Bus_Mst_Length : out std_logic_vector(11 downto 0);
		MAC_DMA2Bus_Mst_BE : out std_logic_vector(C_MAC_DMA_PLB_NATIVE_DWIDTH/8-1 downto 0);
		MAC_DMA2Bus_Mst_Lock : out std_logic := '0';
		MAC_DMA2Bus_Mst_Reset : out std_logic := '0';
		MAC_DMA2Bus_MstRd_dst_rdy_n : out std_logic := '1';
		MAC_DMA2Bus_MstRd_dst_dsc_n : out std_logic := '1';
		MAC_DMA2Bus_MstWr_d : out std_logic_vector(C_MAC_DMA_PLB_NATIVE_DWIDTH-1 downto 0);
		MAC_DMA2Bus_MstWr_rem : out std_logic_vector(C_MAC_DMA_PLB_NATIVE_DWIDTH/8-1 downto 0);
		MAC_DMA2Bus_MstWr_sof_n : out std_logic := '1';
		MAC_DMA2Bus_MstWr_eof_n : out std_logic := '1';
		MAC_DMA2Bus_MstWr_src_rdy_n : out std_logic := '1';
		MAC_DMA2Bus_MstWr_src_dsc_n : out std_logic := '1';
		m_read : in std_logic := '0';
		m_write : in std_logic := '0';
		m_byteenable : in std_logic_vector(3 downto 0);
		m_address : in std_logic_vector(dma_highadr_g downto 0);
		m_writedata : in std_logic_vector(31 downto 0);
		m_burstcount : in std_logic_vector(m_burstcount_width_g-1 downto 0);
		m_burstcounter : in std_logic_vector(m_burstcount_width_g-1 downto 0);
		m_readdata : out std_logic_vector(31 downto 0);
		m_waitrequest : out std_logic := '1';
		m_readdatavalid : out std_logic := '0';
		m_clk : out std_logic
	);
end plb_master_handler;

architecture plb_master_handler of plb_master_handler is
	signal clk, rst : std_logic;
	
	--signals for requesting transfers
	signal m_write_s, m_read_s, m_wrd_en_n : std_logic;
	signal m_write_l, m_read_l : std_logic;
	signal m_write_rise, m_read_rise : std_logic;
	signal m_write_fall, m_read_fall : std_logic;
	signal mst_write_req, mst_write_req_next : std_logic;
	signal mst_read_req, mst_read_req_next : std_logic;
	--what if master wants to req new transfer, but previous is not yet completed (= no Mst_Cmplt pulse!!!)
	signal mst_done : std_logic;
	
	--signals for the transfer
	type tran_t is (idle, sof, tran, eof, seof, wait4cmplt); --seof = start/end of frame (single beat)
	signal wr_tran, wr_tran_next : tran_t;
	signal rd_tran : tran_t;
	
	--avoid preset of FFs
	signal MAC_DMA2Bus_MstRd_dst_rdy : std_logic;
begin
	
	--some assignments..
	m_clk <= MAC_DMA_CLK;
	clk <= MAC_DMA_CLK;
	rst <= MAC_DMA_Rst;
	mst_done <= Bus2MAC_DMA_Mst_Cmplt;
	
	m_write_s <= m_write and not m_wrd_en_n; --NOTE: write/read enable is low-active!
	m_read_s <= m_read and not m_wrd_en_n; --NOTE: write/read enable is low-active!
	
	--reserved
	MAC_DMA2Bus_Mst_Lock <= '0';
	MAC_DMA2Bus_Mst_Reset <= '0';
	
	--delay some signals..
	del_proc : process(clk, rst)
	begin
		if rst = '1' then
			m_write_l <= '0'; m_read_l <= '0';
			m_wrd_en_n <= '0'; --is low-active to avoid preset of FF
		elsif rising_edge(clk) then
			m_write_l <= m_write_s; m_read_l <= m_read_s;
			
			if mst_done = '1' then
				m_wrd_en_n <= '0';
			elsif m_write_fall = '1' or m_read_fall = '1' then
				m_wrd_en_n <= '1'; --write/read done, wait for Mst_Cmplt
			end if;
		end if;
	end process;
	
	--generate pulse if write/read is asserted
	m_write_rise <= '1' when m_write_l = '0' and m_write_s = '1' else '0';
	m_read_rise <= '1' when m_read_l = '0' and m_read_s = '1' else '0';
	m_write_fall <= '1' when m_write_l = '1' and m_write_s = '0' else '0';
	m_read_fall <= '1' when m_read_l = '1' and m_read_s = '0' else '0';
	
	--generate req qualifiers
	req_proc : process(clk, rst)
	begin
		if rst = '1' then
			mst_write_req <= '0'; mst_read_req <= '0';
			MAC_DMA2Bus_MstRd_dst_rdy <= '0';
		elsif rising_edge(clk) then
			mst_write_req <= mst_write_req_next; mst_read_req <= mst_read_req_next;
			
			if m_read_s = '1' then
				MAC_DMA2Bus_MstRd_dst_rdy <= '1';
			elsif rd_tran = eof and Bus2MAC_DMA_MstRd_src_rdy_n = '0' then
				MAC_DMA2Bus_MstRd_dst_rdy <= '0';
			end if;
	    			
		end if;
	end process;
	
	MAC_DMA2Bus_MstRd_dst_rdy_n <= not MAC_DMA2Bus_MstRd_dst_rdy;
	
	mst_write_req_next <= 	'0' when mst_write_req = '1' and Bus2MAC_DMA_Mst_CmdAck = '1' else
							'1' when mst_write_req = '0' and m_write_rise = '1' else
							mst_write_req;
	
	mst_read_req_next <=  	'0' when mst_read_req = '1' and Bus2MAC_DMA_Mst_CmdAck = '1' else
							'1' when mst_read_req = '0' and m_read_rise = '1' else
							mst_read_req;
	
	MAC_DMA2Bus_MstRd_Req <= mst_read_req;
	MAC_DMA2Bus_MstWr_Req <= mst_write_req;
	
	MAC_DMA2Bus_Mst_Type <= '0' when m_burstcount < 2 else --single beat
		mst_read_req or mst_write_req; --we are talking about bursts..
	
	--assign address, byteenable and burst size
	MAC_DMA2Bus_Mst_Addr <= m_address;
	MAC_DMA2Bus_Mst_BE <= "1111";
	MAC_DMA2Bus_Mst_Length <= conv_std_logic_vector(conv_integer(m_burstcount), 
									MAC_DMA2Bus_Mst_Length'length - 2) & "00"; -- dword x 4 = byte
	
	--write/read link
	wrd_proc : process(clk, rst)
	begin
		if rst = '1' then
			wr_tran <= idle;
		elsif rising_edge(clk) then
			wr_tran <= wr_tran_next;
		end if;
	end process;
	
	--generate fsm for write and read transfers
	wr_tran_next <=
		seof when wr_tran = idle and mst_write_req_next = '1' and (m_burstcount <= 1 or m_burstcount'length = 1) else
		sof when wr_tran = idle and mst_write_req_next = '1' and m_burstcount'length > 1 else
		eof when wr_tran = sof and Bus2MAC_DMA_MstWr_dst_rdy_n = '0' and m_burstcount = 2 and m_burstcount'length > 1 else
		tran when wr_tran = sof and Bus2MAC_DMA_MstWr_dst_rdy_n = '0' and m_burstcount'length > 1 else
		eof when wr_tran = tran and m_burstcounter <= 2 and Bus2MAC_DMA_MstWr_dst_rdy_n = '0' and m_burstcount'length > 1 else
		wait4cmplt when (wr_tran = eof or wr_tran = seof) and Bus2MAC_DMA_MstWr_dst_rdy_n = '0' else
		idle when wr_tran = wait4cmplt and mst_done = '1' else
		wr_tran;
	
	rd_tran <=
		seof when Bus2MAC_DMA_MstRd_sof_n = '0' and Bus2MAC_DMA_MstRd_eof_n = '0' else
		sof when Bus2MAC_DMA_MstRd_sof_n = '0' else
		eof when Bus2MAC_DMA_MstRd_eof_n = '0' else
		tran when Bus2MAC_DMA_MstRd_src_rdy_n = '0' else
		idle;
	
	--set write qualifiers
	MAC_DMA2Bus_MstWr_sof_n <= '0' when wr_tran = sof or wr_tran = seof else '1';
	MAC_DMA2Bus_MstWr_eof_n <= '0' when wr_tran = eof or wr_tran = seof else '1';
	MAC_DMA2Bus_MstWr_src_rdy_n <= '0' when wr_tran /= idle and wr_tran /= wait4cmplt else '1';
	MAC_DMA2Bus_MstWr_src_dsc_n <= '1'; --no support
	MAC_DMA2Bus_MstWr_rem <= (others => '0'); --no support
	
	--set read qualifiers
	MAC_DMA2Bus_MstRd_dst_dsc_n <= '1'; --no support
	
	--connect ipif with avalon
	m_waitrequest <= --waitrequest if not ready or no write active
		not m_write when Bus2MAC_DMA_MstWr_dst_rdy_n = '0' else
		not m_read when mst_read_req = '1' and Bus2MAC_DMA_Mst_CmdAck = '1' else '1';
	
	m_readdatavalid <= not Bus2MAC_DMA_MstRd_src_rdy_n;
	
	MAC_DMA2Bus_MstWr_d <= m_writedata;
	m_readdata <= Bus2MAC_DMA_MstRd_d;
	
end plb_master_handler;
