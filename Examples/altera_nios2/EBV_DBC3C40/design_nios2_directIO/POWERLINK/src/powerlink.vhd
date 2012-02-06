------------------------------------------------------------------------------------------------------------------------
-- POWERLINK IP-Core
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
-- 2010-08-23  	V0.01	zelenkaj    First version
-- 2010-09-13	V0.02	zelenkaj	added selection Rmii / Mii
-- 2010-10-18	V0.03	zelenkaj	added selection Big/Little Endian (pdi_par)
--									use bidirectional bus (pdi_par)
-- 2010-11-23	V0.04	zelenkaj	Added 2 GPIO signals to parallel interface
--									Added Operational Flag to simple I/O interface
--									Omitted T/RPDO descriptor sections in DPR
--									Added generic to set duration of valid assertion (portio)
-- 2010-11-29	V0.05	zelenkaj	Added Big/Little Endian (pdi_spi)
-- 2010-12-06	V0.06	zelenkaj	Bugfix: ap_irq was not driven in SPI configuration
-- 2011-01-10	V0.07	zelenkaj	Added 2-stage sync to SPI input pins
-- 2011-02-24	V0.08	zelenkaj	minor changes (naming conventions Mii->SMI)
-- 2011-03-14	V0.09	zelenkaj	minor change, added generic for rx packet buffer location
-- 2011-03-21	V0.10	zelenkaj	clean up
-- 2011-03-28	V0.20	zelenkaj	Changed: Structure of Control/Status Register
--									Added: LED
--									Added: Events
--									Added/Changed: Asynchronous buffer 2x Ping-Pong
-- 2011-04-04	V0.21	zelenkaj	parallel interface, sync moved to pdi_par
--									minor: led_status is the official name
-- 2011-04-26	V0.22	zelenkaj	generic for clock domain selection
-- 2011-04-28	V0.23	zelenkaj	second cmp timer of openMAC is optinal by generic
--									generic for second phy port of openMAC
-- 2011-05-06	V0.24	zelenkaj	some naming convention changes
--									bug fix: use the RX_ER signal, it has important meaning!
-- 2011-05-09  	V0.25	zelenkaj	Hardware Acceleration (HW ACC) added.
-- 2011-07-23   V0.26	zelenkaj	openFILTER enhanced by RxErr signal
-- 2011-07-25	V0.27	zelenkaj	LED gadget and asynchronous buffer optional
-- 2011-08-08	V0.28	zelenkaj	LED gadget enhancement -> added 8 general purpose outputs
-- 2011-08-02	V1.00	zelenkaj	exchanged Avalon interface with entity openMAC_Ethernet
-- 2011-09-05	V1.01	zelenkaj	SPI PDI missed to connect async irq to toplevel
-- 2011-10-20	V1.02	zelenkaj	SMI export of in, out and tristate, endian generic
-- 2011-11-07	V1.03	zelenkaj	dma generic for PLB/AXI support necessary
-- 2011-11-21	V1.04	zelenkaj	added time synchronization feature
-- 2011-11-28	V1.05	zelenkaj	added waitrequest signals to pdi pcp/ap
-- 2011-11-29	V1.06	zelenkaj	event is optional
-- 2011-11-30	V1.07	zelenkaj	Added generic for DMA observer
-- 2011-12-02	V1.08	zelenkaj	Added I, O and T instead of IO ports
-- 2012-01-09   V1.09   zelenkaj    Added ap_syncIrq for external AP
-- 2012-01-26   V1.10   zelenkaj    Added generic for SMI generation and one SMI ports
--                                  Omit hwacc options, since we are fast enough!
------------------------------------------------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;

entity powerlink is
	generic(
	-- GENERAL GENERICS															--
		endian_g					:		string								:= "little";
		genOnePdiClkDomain_g		:		boolean								:= false;
		genPdi_g					:		boolean 							:= true;
		genInternalAp_g				:		boolean 							:= true;
		genSimpleIO_g				:		boolean 							:= false;
		genSpiAp_g					:		boolean 							:= false;
	-- OPENMAC GENERICS
		Simulate                    :     	boolean 							:= false;
   		iBufSize_g					: 		integer 							:= 1024;
   		iBufSizeLOG2_g				: 		integer 							:= 10;
		useRmii_g					:		boolean								:= true; --use Rmii
		useIntPacketBuf_g			:		boolean								:= true; --internal packet buffer
		useRxIntPacketBuf_g			:		boolean								:= true; --rx buffer located in internal packet buffer
		use2ndCmpTimer_g			:		boolean 							:= true; --use second cmp timer (used in PDI)
		use2ndPhy_g					:		boolean 							:= true; --use second phy (introduces openHUB)
		m_burstcount_width_g		:		integer								:= 4;
		m_burstcount_const_g		:		boolean								:= true; --hold burst value during transfer
		m_tx_burst_size_g			:		integer								:= 16; --0 < x =< 2**m_burstcount_width_g
		m_rx_burst_size_g			:		integer								:= 16; --0 < x =< 2**m_burstcount_width_g
		m_tx_fifo_size_g			:		integer								:= 16;
		m_rx_fifo_size_g			:		integer								:= 16;
		m_data_width_g				:		integer								:= 16;
		gen_dma_observer_g			:		boolean								:= true;
		genSmiIO 					: 		boolean 							:= true; --drive SMI IO if true
        gNumSmi                     :       integer range 1 to 2                := 2; --number of SMI used
	-- PDI GENERICS
		iRpdos_g					:		integer 							:= 3;
		iTpdos_g					:		integer 							:= 1;
		genABuf1_g					:		boolean 							:= true; --if false iABuf1_g must be set to 0!
		genABuf2_g					:		boolean 							:= true; --if false iABuf2_g must be set to 0!
		genLedGadget_g				:		boolean 							:= false;
		genTimeSync_g				:		boolean								:= false;
		genEvent_g					:		boolean								:= false;
		--PDO buffer size *3
		iTpdoBufSize_g				:		integer 							:= 100;
		iRpdo0BufSize_g				:		integer 							:= 100;
		iRpdo1BufSize_g				:		integer 							:= 100;
		iRpdo2BufSize_g				:		integer 							:= 100;
		--asynchronous buffer size
		iAsyBuf1Size_g				:		integer 							:= 100;
		iAsyBuf2Size_g				:		integer 							:= 100;
		iPdiRev_g					:		integer								:= 16#55AA#;
	-- 8/16bit PARALLEL PDI GENERICS
		papDataWidth_g				:		integer 							:= 8;
		papLowAct_g					:		boolean								:= false;
		papBigEnd_g					:		boolean								:= false;
	-- SPI GENERICS
		spiCPOL_g					:		boolean 							:= false;
		spiCPHA_g					:		boolean 							:= false;
		spiBigEnd_g					:		boolean								:= false;
	-- PORTIO
		pioValLen_g					:		integer								:= 50; --clock ticks of pcp_clk
	-- GENERAL TARGET DEPENDINGS
		genIoBuf_g					:		boolean								:= true --generates IO buffers
	);
	port(
	-- CLOCK / RESET PORTS
		clk50 						: in 	std_logic; --RMII clk
		rst							: in	std_logic; --general reset
		clkEth 						: in 	std_logic; --Tx Reg clk
		m_clk						: in 	std_logic; --openMAC DMA master clock
		pkt_clk						: in 	std_logic; --openMAC packet buffer clock (don't use pcp..)
		clkPcp 						: in 	std_logic; --pcp clk
		clkAp 						: in 	std_logic; --ap clk
		rstPcp 						: in 	std_logic; --rst from pcp side
		rstAp 						: in 	std_logic; --rst ap
	-- OPENMAC
	--- OPENMAC PORTS
		mac_chipselect              : in    std_logic;
		mac_read					: in    std_logic;
		mac_write					: in    std_logic;
		mac_byteenable            	: in    std_logic_vector(1 downto 0);
		mac_address                 : in    std_logic_vector(11 downto 0);
		mac_writedata               : in    std_logic_vector(15 downto 0);
		mac_readdata                : out   std_logic_vector(15 downto 0) := (others => '0');
		mac_waitrequest				: out	std_logic;
		mac_irq						: out 	std_logic := '0';
	--- TIMER COMPARE PORTS
		tcp_chipselect              : in    std_logic;
		tcp_read					: in    std_logic;
		tcp_write					: in    std_logic;
		tcp_byteenable            	: in    std_logic_vector(3 downto 0);
		tcp_address                 : in    std_logic_vector(1 downto 0);
		tcp_writedata               : in    std_logic_vector(31 downto 0);
		tcp_readdata                : out   std_logic_vector(31 downto 0) := (others => '0');
		tcp_waitrequest				: out	std_logic;
		tcp_irq						: out 	std_logic := '0';
	--- MAC BUFFER PORTS
		mbf_chipselect             	: in    std_logic;
		mbf_read					: in    std_logic;
		mbf_write					: in    std_logic;
		mbf_byteenable             	: in    std_logic_vector(3 downto 0);
		mbf_address                	: in    std_logic_vector(ibufsizelog2_g-3 downto 0);
		mbf_writedata              	: in    std_logic_vector(31 downto 0);
		mbf_readdata               	: out   std_logic_vector(31 downto 0) := (others => '0');
		mbf_waitrequest				: out	std_logic;
	--- OPENMAC DMA PORTS
		m_read						: OUT   STD_LOGIC := '0';
		m_write						: OUT   STD_LOGIC := '0';
		m_byteenable	            : OUT   STD_LOGIC_VECTOR(m_data_width_g/8-1 DOWNTO 0) := (others => '0');
        m_address                   : OUT   STD_LOGIC_VECTOR(29 DOWNTO 0) := (others => '0');
        m_writedata                 : OUT   STD_LOGIC_VECTOR(m_data_width_g-1 DOWNTO 0) := (others => '0');
        m_readdata                  : IN    STD_LOGIC_VECTOR(m_data_width_g-1 DOWNTO 0) := (others => '0');
        m_waitrequest               : IN    STD_LOGIC;
        m_readdatavalid				: in   	STD_LOGIC := '0';
		m_burstcount				: out	std_logic_vector(m_burstcount_width_g-1 downto 0);
		m_burstcounter				: out	std_logic_vector(m_burstcount_width_g-1 downto 0);
	-- PDI
	--- PCP PORTS
	    pcp_chipselect              : in    std_logic;
	    pcp_read					: in    std_logic;
	    pcp_write					: in    std_logic;
	    pcp_byteenable	            : in    std_logic_vector(3 downto 0);
	    pcp_address                 : in    std_logic_vector(12 downto 0);
	    pcp_writedata               : in    std_logic_vector(31 downto 0);
	    pcp_readdata                : out   std_logic_vector(31 downto 0) := (others => '0');
		pcp_waitrequest				: out	std_logic;
	--- AP PORTS
		ap_irq						: out	std_logic := '0';
		ap_irq_n					: out	std_logic := '1';
        ap_syncIrq                  : out   std_logic := '0';
        ap_syncIrq_n                : out   std_logic := '1';
		ap_asyncIrq					: out	std_logic := '0';
		ap_asyncIrq_n				: out	std_logic := '1';
	---- AVALON
		ap_chipselect               : in    std_logic;
		ap_read						: in    std_logic;
		ap_write					: in    std_logic;
		ap_byteenable             	: in    std_logic_vector(3 downto 0);
		ap_address                  : in    std_logic_vector(12 downto 0);
		ap_writedata                : in    std_logic_vector(31 downto 0);
		ap_readdata                 : out   std_logic_vector(31 downto 0) := (others => '0');
		ap_waitrequest				: out	std_logic;
	---- 8/16bit parallel
		pap_cs						: in    std_logic;
		pap_rd						: in    std_logic;
		pap_wr 						: in    std_logic;
		pap_be						: in    std_logic_vector(papDataWidth_g/8-1 downto 0);
		pap_cs_n					: in    std_logic;
		pap_rd_n					: in    std_logic;
		pap_wr_n					: in    std_logic;
		pap_be_n					: in    std_logic_vector(papDataWidth_g/8-1 downto 0);
		pap_addr 					: in    std_logic_vector(15 downto 0);
		pap_data					: inout std_logic_vector(papDataWidth_g-1 downto 0) := (others => '0');
		pap_data_I					: in 	std_logic_vector(papDataWidth_g-1 downto 0) := (others => '0');
		pap_data_O					: out	std_logic_vector(papDataWidth_g-1 downto 0);
		pap_data_T					: out	std_logic;
		pap_ack						: out	std_logic := '0';
		pap_ack_n					: out	std_logic := '1';
		pap_gpio					: inout	std_logic_vector(1 downto 0) := (others => '0');
		pap_gpio_I					: in 	std_logic_vector(1 downto 0) := (others => '0');
		pap_gpio_O					: out	std_logic_vector(1 downto 0);
		pap_gpio_T					: out	std_logic_vector(1 downto 0);
	---- SPI
		spi_clk						: in	std_logic;
		spi_sel_n					: in	std_logic;
		spi_mosi					: in 	std_logic;
		spi_miso					: out	std_logic := '0';
	---- simple I/O
		smp_address    				: in    std_logic;
		smp_read       				: in    std_logic;
		smp_readdata   				: out   std_logic_vector(31 downto 0) := (others => '0');
		smp_write      				: in    std_logic;
		smp_writedata  				: in    std_logic_vector(31 downto 0);
		smp_byteenable 				: in    std_logic_vector(3 downto 0); 
		smp_waitrequest				: out 	std_logic;
		pio_pconfig    				: in    std_logic_vector(3 downto 0);
		pio_portInLatch				: in 	std_logic_vector(3 downto 0);
		pio_portOutValid 			: out 	std_logic_vector(3 downto 0) := (others => '0');
		pio_portio     				: inout std_logic_vector(31 downto 0) := (others => '0');
		pio_portio_I				: in 	std_logic_vector(31 downto 0) := (others => '0');
		pio_portio_O				: out	std_logic_vector(31 downto 0);
		pio_portio_T				: out	std_logic_vector(31 downto 0);
		pio_operational				: out	std_logic := '0';
	-- EXTERNAL
    --- PHY MANAGEMENT
    ---- shared (valid if gNumSmi = 1)
		phy_SMIClk					: out	std_logic := '0';
		phy_SMIDat					: inout	std_logic 							:= '1';
		phy_SMIDat_I				: in 	std_logic := '1';
		phy_SMIDat_O				: out	std_logic;
		phy_SMIDat_T				: out	std_logic;
		phy_Rst_n					: out	std_logic 							:= '1';
    ---- PHY0 (valid if gNumSmi = 2)
		phy0_SMIClk					: out	std_logic := '0';
		phy0_SMIDat					: inout	std_logic 							:= '1';
		phy0_SMIDat_I				: in 	std_logic := '1';
		phy0_SMIDat_O				: out	std_logic;
		phy0_SMIDat_T				: out	std_logic;
		phy0_Rst_n					: out	std_logic 							:= '1';
		phy0_link					: in	std_logic							:= '0';
    ---- PHY1 (valid if gNumSmi = 2)
		phy1_SMIClk					: out	std_logic := '0';
		phy1_SMIDat					: inout	std_logic 							:= '1';
		phy1_SMIDat_I				: in 	std_logic := '1';
		phy1_SMIDat_O				: out	std_logic;
		phy1_SMIDat_T				: out	std_logic;
		phy1_Rst_n					: out	std_logic 							:= '1';
		phy1_link					: in	std_logic							:= '0';
	--- RMII PORTS
		phy0_RxDat                 	: in    std_logic_vector(1 downto 0);
		phy0_RxDv                  	: in    std_logic;
		phy0_RxErr					: in 	std_logic;
		phy0_TxDat                 	: out   std_logic_vector(1 downto 0) := (others => '0');
		phy0_TxEn                  	: out   std_logic := '0';
		phy1_RxDat                 	: in    std_logic_vector(1 downto 0) := (others => '0');
		phy1_RxDv                  	: in    std_logic;
		phy1_RxErr					: in	std_logic;
		phy1_TxDat                 	: out   std_logic_vector(1 downto 0) := (others => '0');
		phy1_TxEn                  	: out   std_logic := '0';
	--- MII PORTS
		phyMii0_RxClk				: in	std_logic;
		phyMii0_RxDat               : in    std_logic_vector(3 downto 0) := (others => '0');
		phyMii0_RxDv                : in    std_logic;
		phyMii0_RxEr				: in	std_logic;
		phyMii0_TxClk				: in	std_logic;
		phyMii0_TxDat               : out   std_logic_vector(3 downto 0) := (others => '0');
		phyMii0_TxEn                : out   std_logic := '0';
		phyMii0_TxEr                : out   std_logic := '0';
		phyMii1_RxClk				: in	std_logic;
		phyMii1_RxDat               : in    std_logic_vector(3 downto 0) := (others => '0');
		phyMii1_RxDv                : in    std_logic;
		phyMii1_RxEr				: in	std_logic;
		phyMii1_TxClk				: in	std_logic;
		phyMii1_TxDat               : out   std_logic_vector(3 downto 0) := (others => '0');
		phyMii1_TxEn                : out   std_logic := '0';
		phyMii1_TxEr                : out   std_logic := '0';
	--- LEDs
		led_error					: out	std_logic := '0';
		led_status					: out	std_logic := '0';
		led_phyLink					: out	std_logic_vector(1 downto 0) := (others => '0');
		led_phyAct					: out	std_logic_vector(1 downto 0) := (others => '0');
		led_opt						: out	std_logic_vector(1 downto 0) := (others => '0');
		led_gpo						: out	std_logic_vector(7 downto 0) := (others => '0')
	);
end powerlink;

architecture rtl of powerlink is
	signal smi_Clk					:		std_logic							:= '0';
	signal smi_Di					:		std_logic							:= '0';
	signal smi_Do					:		std_logic							:= '0';
	signal smi_Doe					:		std_logic							:= '0';
	signal phy_nResetOut			:		std_logic							:= '0';
	signal irqToggle				:		std_logic							:= '0';
	
	signal ap_chipselect_s			:		std_logic							:= '0';
	signal ap_read_s				:		std_logic							:= '0';
	signal ap_write_s				:		std_logic							:= '0';
	signal ap_byteenable_s			:		std_logic_vector(ap_byteenable'range) := (others => '0');
	signal ap_address_s				:		std_logic_vector(ap_address'range)	:= (others => '0');
	signal ap_writedata_s			:		std_logic_vector(ap_writedata'range):= (others => '0');
	signal ap_readdata_s			:		std_logic_vector(ap_readdata'range)	:= (others => '0');
	
	signal pap_cs_s					:		std_logic;
	signal pap_rd_s					:		std_logic;
	signal pap_wr_s					:		std_logic;
	signal pap_be_s					:		std_logic_vector(pap_be'range);
	signal pap_ack_s				:		std_logic;
	signal ap_irq_s					:		std_logic;
	signal ap_asyncIrq_s			:		std_logic;
	
	signal spi_sel_s				:		std_logic;
	signal spi_sel_s1				:		std_logic;
	signal spi_sel_s2				:		std_logic;
	signal spi_clk_s				:		std_logic;
	signal spi_clk_s1				:		std_logic;
	signal spi_clk_s2				:		std_logic;
	signal spi_mosi_s				:		std_logic;
	signal spi_mosi_s1				:		std_logic;
	signal spi_mosi_s2				:		std_logic;
	
	signal phyLink, phyAct			:		std_logic_vector(1 downto 0);
	
	signal led_s					:		std_logic_vector(15 downto 0);
	
	signal clkAp_s, rstAp_s			:		std_logic;
	
	--PDI change buffer triggers for hw acc to pdi
	signal rpdo_change_tog			: 		std_logic_vector(2 downto 0);
	signal tpdo_change_tog			: 		std_logic;
	
begin
	--general signals
	clkAp_s <= clkAp when genOnePdiClkDomain_g = FALSE else clkPcp;
	rstAp_s <= rstAp when genOnePdiClkDomain_g = FALSE else rstPcp;
	
	phyLink <= phy1_link & phy0_link;
	
	--LEDs: GPO7, ..., GPO0, O1, O0, PA1, PL1, PA0, PL0, E, S
	led_error <= led_s(1);
	led_status <= led_s(0);
	led_phyLink <= led_s(4) & led_s(2);
	led_phyAct <= led_s(5) & led_s(3);
	led_opt <= led_s(7) & led_s(6);
	led_gpo <= led_s(15 downto 8);
	
------------------------------------------------------------------------------------------------------------------------
--PCP + AP
	genPdi : if genPdi_g and genInternalAp_g and not genSpiAp_g generate
		
		--sync and async interrupt are driven by only one line
		-- this gives some effort for Nios II AP ;)
		ap_irq <= ap_irq_s or ap_asyncIrq_s;
		
		theAvalonPdi : entity work.pdi
			generic map (
				genOnePdiClkDomain_g		=> genOnePdiClkDomain_g,
				iPdiRev_g					=> iPdiRev_g,
				iRpdos_g					=> iRpdos_g,
				iTpdos_g					=> iTpdos_g,
				genABuf1_g					=> genABuf1_g,
				genABuf2_g					=> genABuf2_g,
				genLedGadget_g				=> genLedGadget_g,
				genTimeSync_g				=> genTimeSync_g,
				genEvent_g					=> genEvent_g,
				--PDO buffer size *3
				iTpdoBufSize_g				=> iTpdoBufSize_g,
				iRpdo0BufSize_g				=> iRpdo0BufSize_g,
				iRpdo1BufSize_g				=> iRpdo1BufSize_g,
				iRpdo2BufSize_g				=> iRpdo2BufSize_g,
				--asynchronous buffer size
				iABuf1_g					=> iAsyBuf1Size_g,
				iABuf2_g					=> iAsyBuf2Size_g
			)
			port map (
				pcp_reset					=> rstPcp,
				pcp_clk                  	=> clkPcp,
				ap_reset					=> rstAp_s,
				ap_clk						=> clkAp_s,
				-- Avalon Slave Interface for PCP
				pcp_chipselect              => pcp_chipselect,
				pcp_read					=> pcp_read,
				pcp_write					=> pcp_write,
				pcp_byteenable	            => pcp_byteenable,
				pcp_address                 => pcp_address,
				pcp_writedata               => pcp_writedata,
				pcp_readdata                => pcp_readdata,
				pcp_waitrequest				=> pcp_waitrequest,
				pcp_irq						=> irqToggle,
				-- Avalon Slave Interface for AP
				ap_chipselect               => ap_chipselect,
				ap_read						=> ap_read,
				ap_write					=> ap_write,
				ap_byteenable             	=> ap_byteenable,
				ap_address                  => ap_address,
				ap_writedata                => ap_writedata,
				ap_readdata                 => ap_readdata,
				ap_waitrequest				=> ap_waitrequest,
				ap_irq						=> ap_irq_s,
				-- async interrupt
				ap_asyncIrq					=> ap_asyncIrq_s,
				-- LED
				ledsOut						=> led_s,
				phyLink						=> phyLink,
				phyAct						=> phyAct,
				--PDI change buffer triggers
				rpdo_change_tog				=> rpdo_change_tog,
				tpdo_change_tog				=> tpdo_change_tog
			);
	end generate genPdi;

--AP is external connected via parallel interface
	genPdiPar : if genPdi_g and not genInternalAp_g and not genSpiAp_g generate
		
		--only 8 or 16bit data width is allowed
		ASSERT ( papDataWidth_g = 8 or papDataWidth_g = 16 )
			REPORT "External parallel port only allows 8 or 16bit data width!"
			severity failure;
		
		-------------------------------------------------------------------------------------
		--convert active low signals to active high - respectively assign active high signals
		theActiveLowGen : if papLowAct_g generate
			pap_wr_s <= not pap_wr_n;
			pap_rd_s <= not pap_rd_n;
			pap_cs_s <= not pap_cs_n;
			pap_be_s <= not pap_be_n;
		end generate;
		
		theActiveHighGen : if not papLowAct_g generate
			pap_wr_s <= pap_wr;
			pap_rd_s <= pap_rd;
			pap_cs_s <= pap_cs;
			pap_be_s <= pap_be;
		end generate;
		
		ap_syncIrq <= ap_irq_s;
		ap_syncIrq_n <= not ap_irq_s;
		
		ap_asyncIrq <= ap_asyncIrq_s;
		ap_asyncIrq_n <= not ap_asyncIrq_s;
		
		pap_ack <= pap_ack_s;
		pap_ack_n <= not pap_ack_s;
		--
		-------------------------------------------------------------------------------------
		
		theParPort : entity work.pdi_par
			generic map (
				papDataWidth_g				=> papDataWidth_g,
				papBigEnd_g					=> papBigEnd_g,
				papGenIoBuf_g				=> genIoBuf_g
			)
			port map (
			-- 8/16bit parallel
				pap_cs						=> pap_cs_s,
				pap_rd						=> pap_rd_s,
				pap_wr						=> pap_wr_s,
				pap_be						=> pap_be_s,
				pap_addr					=> pap_addr,
				pap_data					=> pap_data,
				pap_data_I					=> pap_data_I,
				pap_data_O					=> pap_data_O,
				pap_data_T					=> pap_data_T,
				pap_ack						=> pap_ack_s,
				pap_gpio					=> pap_gpio,
				pap_gpio_I					=> pap_gpio_I,
				pap_gpio_O					=> pap_gpio_O,
				pap_gpio_T					=> pap_gpio_T,
			-- clock for AP side
				ap_reset					=> rstPcp,
				ap_clk						=> clk50,
			-- Avalon Slave Interface for AP
	            ap_chipselect				=> ap_chipselect_s,
	            ap_read						=> ap_read_s,
	            ap_write					=> ap_write_s,
	            ap_byteenable				=> ap_byteenable_s,
	            ap_address					=> ap_address_s,
	            ap_writedata				=> ap_writedata_s,
	            ap_readdata					=> ap_readdata_s
			);
		
		thePdi : entity work.pdi
			generic map (
				genOnePdiClkDomain_g		=> genOnePdiClkDomain_g,
				iPdiRev_g					=> iPdiRev_g,
				iRpdos_g					=> iRpdos_g,
				iTpdos_g					=> iTpdos_g,
				genABuf1_g					=> genABuf1_g,
				genABuf2_g					=> genABuf2_g,
				genLedGadget_g				=> genLedGadget_g,
				genTimeSync_g				=> genTimeSync_g,
				genEvent_g					=> genEvent_g,
				--PDO buffer size *3
				iTpdoBufSize_g				=> iTpdoBufSize_g,
				iRpdo0BufSize_g				=> iRpdo0BufSize_g,
				iRpdo1BufSize_g				=> iRpdo1BufSize_g,
				iRpdo2BufSize_g				=> iRpdo2BufSize_g,
				--asynchronous buffer size
				iABuf1_g					=> iAsyBuf1Size_g,
				iABuf2_g					=> iAsyBuf2Size_g
			)
			port map (
				pcp_reset					=> rstPcp,
				pcp_clk                  	=> clkPcp,
				ap_reset					=> rst,
				ap_clk						=> clk50,
				-- Avalon Slave Interface for PCP
				pcp_chipselect              => pcp_chipselect,
				pcp_read					=> pcp_read,
				pcp_write					=> pcp_write,
				pcp_byteenable	            => pcp_byteenable,
				pcp_address                 => pcp_address,
				pcp_writedata               => pcp_writedata,
				pcp_readdata                => pcp_readdata,
				pcp_waitrequest				=> pcp_waitrequest,
				pcp_irq						=> irqToggle,
				-- Avalon Slave Interface for AP
				ap_chipselect               => ap_chipselect_s,
				ap_read						=> ap_read_s,
				ap_write					=> ap_write_s,
				ap_byteenable             	=> ap_byteenable_s,
				ap_address                  => ap_address_s,
				ap_writedata                => ap_writedata_s,
				ap_readdata                 => ap_readdata_s,
				ap_waitrequest				=> open,
				ap_irq						=> ap_irq_s,
				-- async interrupt
				ap_asyncIrq					=> ap_asyncIrq_s,
				-- LED
				ledsOut						=> led_s,
				phyLink						=> phyLink,
				phyAct						=> phyAct,
				--PDI change buffer triggers
				rpdo_change_tog				=> rpdo_change_tog,
				tpdo_change_tog				=> tpdo_change_tog
			);
	end generate genPdiPar;

--AP is extern connected via SPI
	genPdiSpi : if genPdi_g and genSpiAp_g generate
		
		ap_syncIrq <= ap_irq_s;
		ap_syncIrq_n <= not ap_irq_s;
		
		ap_asyncIrq <= ap_asyncIrq_s;
		ap_asyncIrq_n <= not ap_asyncIrq_s;
		
		spi_clk_s <= spi_clk;
		spi_sel_s <= not spi_sel_n;
		spi_mosi_s <= spi_mosi;
		
		theSyncProc : process(clk50, rst)
		begin
			if rst = '1' then
				spi_sel_s1 <= '0';
				spi_sel_s2 <= '0';
				spi_clk_s1 <= '0';
				spi_clk_s2 <= '0';
				spi_mosi_s1 <= '0';
				spi_mosi_s2 <= '0';
			elsif clk50 = '1' and clk50'event then
				spi_sel_s1 <= spi_sel_s;
				spi_sel_s2 <= spi_sel_s1;
				
				spi_clk_s1 <= spi_clk_s;
				spi_clk_s2 <= spi_clk_s1;
				
				spi_mosi_s1 <= spi_mosi_s;
				spi_mosi_s2 <= spi_mosi_s1;
			end if;
		end process;
------------------------------------------------------------------------------------------------------------------------
		
		thePdiSpi : entity work.pdi_spi
			generic map (
				spiSize_g					=> 8, --fixed value!
				cpol_g 						=> spiCPOL_g,
				cpha_g 						=> spiCPHA_g,
				spiBigEnd_g					=> spiBigEnd_g
			)
			port map (
				-- SPI
				spi_clk						=> spi_clk_s2,
				spi_sel						=> spi_sel_s2,
				spi_miso					=> spi_miso,
				spi_mosi					=> spi_mosi_s2,
				-- clock for AP side
				ap_reset					=> rstPcp,
				ap_clk						=> clk50,
				-- Avalon Slave Interface for AP
				ap_chipselect               => ap_chipselect_s,
				ap_read						=> ap_read_s,
				ap_write					=> ap_write_s,
				ap_byteenable             	=> ap_byteenable_s,
				ap_address                  => ap_address_s,
				ap_writedata                => ap_writedata_s,
				ap_readdata                 => ap_readdata_s
			);
		
		thePdi : entity work.pdi
			generic map (
				genOnePdiClkDomain_g		=> genOnePdiClkDomain_g,
				iPdiRev_g					=> iPdiRev_g,
				iRpdos_g					=> iRpdos_g,
				iTpdos_g					=> iTpdos_g,
				genABuf1_g					=> genABuf1_g,
				genABuf2_g					=> genABuf2_g,
				genLedGadget_g				=> genLedGadget_g,
				genTimeSync_g				=> genTimeSync_g,
				genEvent_g					=> genEvent_g,
				--PDO buffer size *3
				iTpdoBufSize_g				=> iTpdoBufSize_g,
				iRpdo0BufSize_g				=> iRpdo0BufSize_g,
				iRpdo1BufSize_g				=> iRpdo1BufSize_g,
				iRpdo2BufSize_g				=> iRpdo2BufSize_g,
				--asynchronous buffer size
				iABuf1_g					=> iAsyBuf1Size_g,
				iABuf2_g					=> iAsyBuf2Size_g
			)
			port map (
				pcp_reset					=> rstPcp,
				pcp_clk                  	=> clkPcp,
				ap_reset					=> rst,
				ap_clk						=> clk50,
				-- Avalon Slave Interface for PCP
				pcp_chipselect              => pcp_chipselect,
				pcp_read					=> pcp_read,
				pcp_write					=> pcp_write,
				pcp_byteenable	            => pcp_byteenable,
				pcp_address                 => pcp_address,
				pcp_writedata               => pcp_writedata,
				pcp_readdata                => pcp_readdata,
				pcp_waitrequest				=> pcp_waitrequest,
				pcp_irq						=> irqToggle,
				-- Avalon Slave Interface for AP
				ap_chipselect               => ap_chipselect_s,
				ap_read						=> ap_read_s,
				ap_write					=> ap_write_s,
				ap_byteenable             	=> ap_byteenable_s,
				ap_address                  => ap_address_s,
				ap_writedata                => ap_writedata_s,
				ap_readdata                 => ap_readdata_s,
				ap_waitrequest				=> open,
				ap_irq						=> ap_irq_s,
				-- async interrupt
				ap_asyncIrq					=> ap_asyncIrq_s,
				-- LED
				ledsOut						=> led_s,
				phyLink						=> phyLink,
				phyAct						=> phyAct,
				--PDI change buffer triggers
				rpdo_change_tog				=> rpdo_change_tog,
				tpdo_change_tog				=> tpdo_change_tog
			);
	end generate genPdiSpi;
--
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
--SIMPLE I/O CN
	genSimpleIO : if genSimpleIO_g generate
		thePortIO : entity work.portio
			generic map (
				pioValLen_g			=> pioValLen_g,
				pioGenIoBuf_g		=> genIoBuf_g
			)
			port map (
				s0_address			=> smp_address,
				s0_read				=> smp_read,
				s0_readdata			=> smp_readdata,
				s0_write			=> smp_write,
				s0_writedata		=> smp_writedata,
				s0_byteenable		=> smp_byteenable,		   
				s0_waitrequest		=> smp_waitrequest,
				clk					=> clkPcp,
				reset				=> rstPcp,
				x_pconfig			=> pio_pconfig,
				x_portInLatch		=> pio_portInLatch,
				x_portOutValid		=> pio_portOutValid,
				x_portio			=> pio_portio,
				x_portio_I			=> pio_portio_I,
				x_portio_O			=> pio_portio_O,
				x_portio_T			=> pio_portio_T,
				x_operational		=> pio_operational
			);
	end generate genSimpleIO;
--
------------------------------------------------------------------------------------------------------------------------
	
------------------------------------------------------------------------------------------------------------------------
--OPENMAC (OPENHUB, OPENFILTER, PHY MANAGEMENT)
	theOpenMac : entity work.openMAC_Ethernet
		generic map (
			endian_g => endian_g,
			dma_highadr_g => m_address'high,
			gen2ndCmpTimer_g => use2ndCmpTimer_g,
			genHub_g => use2ndPhy_g,
			iPktBufSizeLog2_g => iBufSizeLOG2_g,
			iPktBufSize_g => iBufSize_g,
			simulate => false,
			useIntPktBuf_g => useIntPacketBuf_g,
			useRmii_g => useRmii_g,
			useRxIntPktBuf_g => useRxIntPacketBuf_g,
			m_burstcount_width_g => m_burstcount_width_g,
			m_burstcount_const_g => m_burstcount_const_g,
			m_data_width_g => m_data_width_g,
			m_tx_fifo_size_g => m_tx_fifo_size_g,
			m_rx_fifo_size_g => m_rx_fifo_size_g,
			m_tx_burst_size_g => m_tx_burst_size_g,
			m_rx_burst_size_g => m_rx_burst_size_g,
			genSmiIO => genSmiIO,
            gNumSmi => gNumSmi,
			genPhyActLed_g => genLedGadget_g,
			gen_dma_observer_g => gen_dma_observer_g
		)
		port map(
			clk => clk50,
			clkx2 => clkEth,
			pkt_clk => pkt_clk,
			m_clk => m_clk,
			rst => rst,
			
			m_address => m_address,
			m_burstcount => m_burstcount,
			m_burstcounter => m_burstcounter,
			m_byteenable => m_byteenable,
			m_read => m_read,
			m_readdata => m_readdata,
			m_readdatavalid => m_readdatavalid,
			m_write => m_write,
			m_writedata => m_writedata,
			m_waitrequest => m_waitrequest,
			
			mac_rx_irq => open,
			mac_tx_irq => open,
			
			act_led => phyAct(0),
			
			phy0_rst_n => phy0_Rst_n,
			phy0_rx_dat => phy0_RxDat,
			phy0_rx_dv => phy0_RxDv,
			phy0_rx_err => phy0_RxErr,
			phy0_smi_clk => phy0_SMICLK,
			phy0_smi_dio => phy0_SMIDat,
			phy0_smi_dio_I => phy0_SMIDat_I,
			phy0_smi_dio_O => phy0_SMIDat_O,
			phy0_smi_dio_T => phy0_SMIDat_T,
			phy0_tx_dat => phy0_TxDat,
			phy0_tx_en => phy0_TxEn,
			
			phy1_rst_n => phy1_Rst_n,
			phy1_rx_dat => phy1_RxDat,
			phy1_rx_dv => phy1_RxDv,
			phy1_rx_err => phy1_RxErr,
			phy1_smi_clk => phy1_SMICLK,
			phy1_smi_dio => phy1_SMIDat,
			phy1_smi_dio_I => phy1_SMIDat_I,
			phy1_smi_dio_O => phy1_SMIDat_O,
			phy1_smi_dio_T => phy1_SMIDat_T,
			phy1_tx_dat => phy1_TxDat,
			phy1_tx_en => phy1_TxEn,
			
			phyMii0_rx_clk => phyMii0_RxClk,
			phyMii0_rx_dat => phyMii0_RxDat,
			phyMii0_rx_dv => phyMii0_RxDv,
			phyMii0_rx_err => phyMii0_RxEr,
			phyMii0_tx_clk => phyMii0_TxClk,
			phyMii0_tx_dat => phyMii0_TxDat,
			phyMii0_tx_en => phyMii0_TxEn,
			
			phyMii1_rx_clk => phyMii1_RxClk,
			phyMii1_rx_dat => phyMii1_RxDat,
			phyMii1_rx_dv => phyMii1_RxDv,
			phyMii1_rx_err => phyMii1_RxEr,
			phyMii1_tx_clk => phyMii1_TxClk,
			phyMii1_tx_dat => phyMii1_TxDat,
			phyMii1_tx_en => phyMii1_TxEn,
            
            phy_rst_n => phy_Rst_n,
            phy_smi_clk => phy_SMIClk,
            phy_smi_dio_I => phy_SMIDat_I,
            phy_smi_dio_O => phy_SMIDat_O,
            phy_smi_dio_T => phy_SMIDat_T,
            phy_smi_dio => phy_SMIDat,
			
            pkt_address => mbf_address,
			pkt_byteenable => mbf_byteenable,
			pkt_chipselect => mbf_chipselect,
			pkt_read => mbf_read,
			pkt_readdata => mbf_readdata,
			pkt_waitrequest => mbf_waitrequest,
			pkt_write => mbf_write,
			pkt_writedata => mbf_writedata,
			
			s_address => mac_address,
			s_byteenable => mac_byteenable,
			s_chipselect => mac_chipselect,
			s_irq => mac_irq,
			s_read => mac_read,
			s_readdata => mac_readdata,
			s_waitrequest => mac_waitrequest,
			s_write => mac_write,
			s_writedata => mac_writedata,
			
			t_address => tcp_address,
			t_byteenable => tcp_byteenable,
			t_chipselect => tcp_chipselect,
			t_irq => tcp_irq,
			t_read => tcp_read,
			t_readdata => tcp_readdata,
			t_tog => irqToggle,
			t_waitrequest => tcp_waitrequest,
			t_write => tcp_write,
			t_writedata => tcp_writedata
		);
		
		phyAct(1) <= phyAct(0);
--
------------------------------------------------------------------------------------------------------------------------
		
end rtl;
