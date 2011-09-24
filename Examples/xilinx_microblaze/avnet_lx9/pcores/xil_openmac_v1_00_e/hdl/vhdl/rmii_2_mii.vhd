-------------------------------------------------------------------------------
--
-- Title       : rmii_2_mii
-- Design      : rmii_2_mii
-- Author      : ATSALZ137
-- Company     : Bernecker + Rainer
--
-------------------------------------------------------------------------------
--
-- File        : rmii_2_mii.vhd
-- Generated   : Mon Nov  2 14:18:44 2009
-- From        : interface description file
-- By          : Itf2Vhdl ver. 1.20
--
-------------------------------------------------------------------------------
--
-- Description : 
--
-------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;

entity rmii_2_mii is
	port (
		---testports---
		test_rx_fifo_ae : out std_logic;
		test_rx_fifo_full : out std_logic;
		test_rx_fifo_empty : out std_logic;
		test_rx_fifo_valid : out std_logic;
		test_rx_fifo_wr : out std_logic;
		test_rx_fifo_rd : out std_logic;
		test_rx_fifo_din : out std_logic_vector(3 downto 0);
		test_rx_fifo_dout : out std_logic_vector(1 downto 0);
		
		test_tx_fifo_ae : out std_logic;
		test_tx_fifo_full : out std_logic;
		test_tx_fifo_empty : out std_logic;
		test_tx_fifo_valid : out std_logic;
		test_tx_fifo_wr : out std_logic;
		test_tx_fifo_rd : out std_logic;
		test_tx_fifo_din : out std_logic_vector(1 downto 0);
		test_tx_fifo_dout : out std_logic_vector(3 downto 0);
		---testports---
		clk50 		: in std_logic;
		rst 		: in std_logic;
		--RMII MAC
		rCrsDv		: out std_logic := '0';
		rRxErr		: out std_logic := '0';
		rRxD		: out std_logic_vector(1 downto 0) := (others => '0');
		rTxEn		: in  std_logic;
		rTxD		: in  std_logic_vector(1 downto 0);
		--MII PHY
		mDv			: in  std_logic;
		mRxD		: in  std_logic_vector(3 downto 0);
		mRxErr		: in  std_logic;
		mRxClk		: in  std_logic;
		mCrs		: in  std_logic;
		mCol		: in  std_logic;
		mTxEn		: out std_logic := '0';
		mTxD		: out std_logic_vector(3 downto 0) := (others => '0');
		mTxErr		: out std_logic := '0';
		mTxClk		: in  std_logic
	);
end rmii_2_mii;

architecture rtl of rmii_2_mii is
	component fifo_txmii
		port (
			rst: IN std_logic;
			wr_clk: IN std_logic;
			rd_clk: IN std_logic;
			din: IN std_logic_VECTOR(1 downto 0);
			wr_en: IN std_logic;
			rd_en: IN std_logic;
			dout: OUT std_logic_VECTOR(3 downto 0);
			full: OUT std_logic;
			empty: OUT std_logic;
			valid: OUT std_logic;
			prog_empty: OUT std_logic
		);
	end component;
	
	component fifo_rxmii
		port (
			rst: IN std_logic;
			wr_clk: IN std_logic;
			rd_clk: IN std_logic;
			din: IN std_logic_VECTOR(3 downto 0);
			wr_en: IN std_logic;
			rd_en: IN std_logic;
			dout: OUT std_logic_VECTOR(1 downto 0);
			full: OUT std_logic;
			empty: OUT std_logic;
			valid: OUT std_logic;
			prog_empty: OUT std_logic
		);
	end component;
begin
	
	rRxErr <= mRxErr;
	
	TX_BLOCK : block
		signal fifo_ae, fifo_full, fifo_empty, fifo_valid : std_logic := '0';
		signal fifo_wr, fifo_rd : std_logic := '0';
		signal fifo_din : std_logic_vector(1 downto 0) := (others => '0');
		signal fifo_dout : std_logic_vector(3 downto 0) := (others => '0');
	begin
	
		---testports---
		test_tx_fifo_ae <= fifo_ae;
		test_tx_fifo_full <= fifo_full;
		test_tx_fifo_empty <= fifo_empty;
		test_tx_fifo_valid <= fifo_valid;
		test_tx_fifo_wr <= fifo_wr;
		test_tx_fifo_rd <= fifo_rd;
		test_tx_fifo_din <= fifo_din;
		test_tx_fifo_dout <= fifo_dout;
		---testports---
		
		fifo_din <= rTxD;
		fifo_wr <= rTxEn;
		
		mTxD <= fifo_dout(1 downto 0) & fifo_dout(3 downto 2);
		mTxEn <= fifo_valid;
		
		process(mTxClk, rst)
		begin
			if rst = '1' then
				fifo_rd <= '0';
			elsif mTxClk = '1' and mTxClk'event then
				
				if fifo_rd = '0' and fifo_ae = '0' then
					fifo_rd <= '1';
				elsif fifo_rd = '1' and fifo_empty = '1' then
					fifo_rd <= '0';
				end if;
			end if;
		end process;
		
		txFifo : fifo_txmii
			port map (
				rst => rst,
				wr_clk => not clk50,
				rd_clk => mTxClk,
				din => fifo_din,
				wr_en => fifo_wr,
				rd_en => fifo_rd,
				dout => fifo_dout,
				full => fifo_full,
				empty => fifo_empty,
				valid => fifo_valid,
				prog_empty => fifo_ae
			);
		
	end block;
	
	RX_BLOCK : block
		signal fifo_ae, fifo_full, fifo_empty, fifo_valid : std_logic := '0';
		signal fifo_wr, fifo_rd : std_logic := '0';
		signal fifo_din : std_logic_vector(3 downto 0) := (others => '0');
		signal fifo_dout : std_logic_vector(1 downto 0) := (others => '0');
	begin
		
		---testports---
		test_rx_fifo_ae <= fifo_ae;
		test_rx_fifo_full <= fifo_full;
		test_rx_fifo_empty <= fifo_empty;
		test_rx_fifo_valid <= fifo_valid;
		test_rx_fifo_wr <= fifo_wr;
		test_rx_fifo_rd <= fifo_rd;
		test_rx_fifo_din <= fifo_din;
		test_rx_fifo_dout <= fifo_dout;
		---testports---
		
		fifo_din <= mRxD(1 downto 0) & mRxD(3 downto 2); --dibit flip because fifo outputs MSB first
		fifo_wr <= mDv;
		
		rRxD <= fifo_dout; rCrsDv <= fifo_valid;
		
		process(clk50, rst)
		begin
			if rst = '1' then
				fifo_rd <= '0';
			elsif clk50 = '1' and clk50'event then
				if fifo_rd = '0' and fifo_ae = '0' then
					fifo_rd <= '1';
				elsif fifo_rd = '1' and fifo_empty = '1' then
					fifo_rd <= '0';
				end if;
			end if;
		end process;
		
		rxFifo : fifo_rxmii
				port map (
					rst => rst,
					wr_clk => mRxClk,
					rd_clk => clk50,
					din => fifo_din,
					wr_en => fifo_wr,
					rd_en => fifo_rd,
					dout => fifo_dout,
					full => fifo_full,
					empty => fifo_empty,
					prog_empty => fifo_ae, --12 entries left
					valid => fifo_valid
				);
		
	end block;
	
end rtl;
