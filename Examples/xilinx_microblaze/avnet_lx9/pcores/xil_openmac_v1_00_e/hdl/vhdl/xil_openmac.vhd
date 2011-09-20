------------------------------------------------------------------------------------------------------------------------
-- OpenMAC PLB Interface
--
-- 	  Copyright (C) 2009 B&R
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
-- 2009-08-07  V0.00        First generation.
------------------------------------------------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;

library proc_common_v3_00_a;
use proc_common_v3_00_a.proc_common_pkg.all;
use proc_common_v3_00_a.ipif_pkg.all;
		
library plbv46_slave_single_v1_01_a;
use plbv46_slave_single_v1_01_a.plbv46_slave_single;

entity xil_openmac is
  generic
  (
    C_BASEADDR                     : std_logic_vector     := X"FFFFFFFF";
    C_HIGHADDR                     : std_logic_vector     := X"00000000";
    C_SPLB_AWIDTH                  : integer              := 32;
    C_SPLB_DWIDTH                  : integer              := 32;
    C_SPLB_NUM_MASTERS             : integer              := 8;
    C_SPLB_MID_WIDTH               : integer              := 3;
    C_SPLB_NATIVE_DWIDTH           : integer              := 32;
    C_SPLB_P2P                     : integer              := 0;
    C_SPLB_SUPPORT_BURSTS          : integer              := 0;
    C_SPLB_SMALLEST_MASTER         : integer              := 32;
    C_SPLB_CLK_PERIOD_PS           : integer              := 20000;
    C_INCLUDE_DPHASE_TIMER         : integer              := 0;
    C_FAMILY                       : string               := "virtex5";
    C_MPLB_AWIDTH                  : integer              := 32;
    C_MPLB_DWIDTH                  : integer              := 32;
    C_MPLB_NATIVE_DWIDTH           : integer              := 32;
    C_MPLB_P2P                     : integer              := 0;
    C_MPLB_SMALLEST_SLAVE          : integer              := 32;
    C_MPLB_CLK_PERIOD_PS           : integer              := 20000;
    C_MEM0_BASEADDR                : std_logic_vector     := X"FFFFFFFF";
    C_MEM0_HIGHADDR                : std_logic_vector     := X"00000000";
    useRmii_g		           : boolean              := false     
  );
  port
  (
	 -- Clock
	 ClkFaster		   : IN    STD_LOGIC                    ;
	 ClkEth			   : IN    STD_LOGIC                    ;
	 
	 --Mii Core
	 ---SMI
	 Mii_D_I                   : IN    std_logic_vector(1 downto 0) ;
	 Mii_D_O                   : OUT   std_logic_vector(1 downto 0) ;
	 Mii_D_T                   : OUT   std_logic                    ;
	 Mii_Clk                   : OUT   std_logic                    ;
	---Phy Reset
	 nResetOut                 : OUT   std_logic                    ;
	 
	 TX_IR_n                   : OUT   std_logic                    ;
	 RX_IR_n                   : OUT   std_logic                    ;
	 Cmp_IR_n		   : OUT   std_logic                    ;
	 t_IrqToggle		   : OUT   STD_LOGIC                    ;
	
    	 -- RMII Port 0
   	 rRx_Dat_0                 : IN    STD_LOGIC_VECTOR(1 DOWNTO 0) ;  -- RMII Rx Daten
    	 rCrs_Dv_0                 : IN    STD_LOGIC                    ;  -- RMII Carrier Sense / Data Valid
    	 rTx_Dat_0                 : OUT   STD_LOGIC_VECTOR(1 DOWNTO 0) ;  -- RMII Tx Daten
   	 rTx_En_0                  : OUT   STD_LOGIC                    ;  -- RMII Tx_Enable
  	 -- RMII Port 1
         rRx_Dat_1                 : IN    STD_LOGIC_VECTOR(1 DOWNTO 0) ;  -- RMII Rx Daten
         rCrs_Dv_1                 : IN    STD_LOGIC                    ;  -- RMII Carrier Sense / Data Valid
         rTx_Dat_1                 : OUT   STD_LOGIC_VECTOR(1 DOWNTO 0) ;  -- RMII Tx Daten
         rTx_En_1                  : OUT   STD_LOGIC                    ;  -- RMII Tx_Enable
  	 -- MII PORTS
  	 phyMii0_RxClk		   : in    std_logic	                ;
  	 phyMii0_RxDat             : in    std_logic_vector(3 downto 0) ;
  	 phyMii0_RxDv              : in    std_logic	                ;
         phyMii0_TxClk		   : in    std_logic	                ;
         phyMii0_TxDat             : out   std_logic_vector(3 downto 0) ;
  	 phyMii0_TxEn              : out   std_logic	                ;
         phyMii0_Crs		   : in    std_logic	                ;
  		
         phyMii0_TxEr              : out   std_logic	                ;
         phyMii0_RxErr   	   : in    std_logic	                ;
         phyMii0_Col		   : in    std_logic	                ;
  		
         phyMii1_RxClk		   : in    std_logic                    ;
  	 phyMii1_RxDat             : in    std_logic_vector(3 downto 0) ;
  	 phyMii1_RxDv              : in    std_logic                    ;
  	 phyMii1_TxClk		   : in    std_logic                    ;
  	 phyMii1_TxDat             : out   std_logic_vector(3 downto 0) ;
  	 phyMii1_TxEn              : out   std_logic                    ;
  	 phyMii1_Crs               : in    std_logic                    ;
  	
  	 phyMii1_TxEr              : out   std_logic	                ;
  	 phyMii1_RxErr		   : in    std_logic	                ;
  	 phyMii1_Col		   : in	   std_logic	                ;
    
  	 
  	 ---TEST PORTS
  	 test_dma_addr	           : out std_logic_vector(31 downto 0)  ;
  	 test_dma_din		   : out std_logic_vector(15 downto 0)  ;
  	 test_dma_dout		   : out std_logic_vector(15 downto 0)  ;
  	 test_dma_ack		   : out std_logic                      ;
  	 test_dma_req		   : out std_logic                      ;
  	 test_dma_rw		   : out std_logic                      ;
         test_plb_req		   : out std_logic_vector(1 downto 0)   ;
	 test_error		   : out std_logic_vector(4 downto 0)   ;
	 test_plb_ack		   : out std_logic_vector(2 downto 0)   ;

	 test_plb_owner		   : out std_logic_vector(1 downto 0)   ;

         test_tx_plb_fsm	   : out std_logic_vector(3 downto 0)   ;
	 test_tx_mac_fsm	   : out std_logic_vector(3 downto 0)   ;
	 test_rx_plb_fsm	   : out std_logic_vector(3 downto 0)   ;
	 test_rx_mac_fsm	   : out std_logic_vector(3 downto 0)   ;
	 test_tx_fsm		   : out std_logic_vector(3 downto 0)   ;

	 test_txFifo_rd		   : out std_logic                      ;
	 test_txFifo_wr		   : out std_logic                      ;
	 test_txFifo_full	   : out std_logic                      ;
	 test_txFifo_empty	   : out std_logic                      ;
	 test_txFifo_din	   : out std_logic_vector(31 downto 0)  ;
	 test_txFifo_dout	   : out std_logic_vector(31 downto 0)  ;

	 test_cmp_sel		   : out std_logic                      ;
	 test_cmp_addr		   : out std_logic_vector(31 downto 0)  ;
	 test_cmp_wert		   : out std_logic_vector(31 downto 0)  ;
	 test_cmp_int		   : out std_logic                      ;
	 test_cmp_on		   : out std_logic                      ;
	 test_cmp_din		   : out std_logic_vector(31 downto 0)  ;
	 test_cmp_dout		   : out std_logic_vector(31 downto 0)  ;
	 
          -- Bus protocol ports, do not add to or delete
         SPLB_Clk                  : in  std_logic                      ;
         SPLB_Rst                  : in  std_logic                      ;
         PLB_ABus                  : in  std_logic_vector(0 to 31)      ;
         PLB_UABus                 : in  std_logic_vector(0 to 31)      ;
         PLB_PAValid               : in  std_logic                      ;
         PLB_SAValid               : in  std_logic                      ;
         PLB_rdPrim                : in  std_logic                      ;
         PLB_wrPrim                : in  std_logic                      ;
         PLB_masterID              : in  std_logic_vector(0 to C_SPLB_MID_WIDTH-1);
         PLB_abort                 : in  std_logic                      ;
         PLB_busLock               : in  std_logic                      ;
         PLB_RNW                   : in  std_logic                      ;
         PLB_BE                    : in  std_logic_vector(0 to C_SPLB_DWIDTH/8-1);
         PLB_MSize                 : in  std_logic_vector(0 to 1)       ;
         PLB_size                  : in  std_logic_vector(0 to 3)       ;
         PLB_type                  : in  std_logic_vector(0 to 2)       ;
         PLB_lockErr               : in  std_logic                      ;
         PLB_wrDBus                : in  std_logic_vector(0 to C_SPLB_DWIDTH-1);
         PLB_wrBurst               : in  std_logic                      ; 
         PLB_rdBurst               : in  std_logic                      ;
         PLB_wrPendReq             : in  std_logic                      ;
         PLB_rdPendReq             : in  std_logic                      ;
         PLB_wrPendPri             : in  std_logic_vector(0 to 1) := (others => '0');
         PLB_rdPendPri             : in  std_logic_vector(0 to 1) := (others => '0');
         PLB_reqPri                : in  std_logic_vector(0 to 1) := (others => '0');
         PLB_TAttribute            : in  std_logic_vector(0 to 15)      ;
         Sl_addrAck                : out std_logic := '0'               ;
         Sl_SSize                  : out std_logic_vector(0 to 1) := (others => '0');
         Sl_wait                   : out std_logic := '0';
         Sl_rearbitrate            : out std_logic := '0';
         Sl_wrDAck                 : out std_logic := '0';
         Sl_wrComp                 : out std_logic := '0';
         Sl_wrBTerm                : out std_logic := '0';
         Sl_rdDBus                 : out std_logic_vector(0 to C_SPLB_DWIDTH-1) := (others => '0');
         Sl_rdWdAddr               : out std_logic_vector(0 to 3) := (others => '0');
         Sl_rdDAck                 : out std_logic := '0';
         Sl_rdComp                 : out std_logic := '0';
         Sl_rdBTerm                : out std_logic := '0';
         Sl_MBusy                  : out std_logic_vector(0 to C_SPLB_NUM_MASTERS-1) := (others => '0');
         Sl_MWrErr                 : out std_logic_vector(0 to C_SPLB_NUM_MASTERS-1) := (others => '0');
         Sl_MRdErr                 : out std_logic_vector(0 to C_SPLB_NUM_MASTERS-1) := (others => '0');
         Sl_MIRQ                   : out std_logic_vector(0 to C_SPLB_NUM_MASTERS-1) := (others => '0');
         MPLB_Clk                  : in  std_logic;
         MPLB_Rst                  : in  std_logic;
         M_request                 : out std_logic := '0';
         M_priority                : out std_logic_vector(0 to 1) := (others => '1'); --highest!
         M_busLock                 : out std_logic := '0';
         M_RNW                     : out std_logic := '0';
         M_BE                      : out std_logic_vector(0 to C_MPLB_DWIDTH/8-1) := (others => '0');
         M_MSize                   : out std_logic_vector(0 to 1) := (others => '0');
         M_size                    : out std_logic_vector(0 to 3) := (others => '0');
         M_type                    : out std_logic_vector(0 to 2) := (others => '0');
         M_TAttribute              : out std_logic_vector(0 to 15) := (others => '0');
         M_lockErr                 : out std_logic := '0';
         M_abort                   : out std_logic := '0';
         M_UABus                   : out std_logic_vector(0 to 31) := (others => '0');
         M_ABus                    : out std_logic_vector(0 to 31) := (others => '0');
         M_wrDBus                  : out std_logic_vector(0 to C_MPLB_DWIDTH-1) := (others => '0');
         M_wrBurst                 : out std_logic := '0';
         M_rdBurst                 : out std_logic := '0';
         PLB_MAddrAck              : in  std_logic;
         PLB_MSSize                : in  std_logic_vector(0 to 1);
         PLB_MRearbitrate          : in  std_logic;
         PLB_MTimeout              : in  std_logic;
         PLB_MBusy                 : in  std_logic;
         PLB_MRdErr                : in  std_logic;
         PLB_MWrErr                : in  std_logic;
         PLB_MIRQ                  : in  std_logic;
         PLB_MRdDBus               : in  std_logic_vector(0 to (C_MPLB_DWIDTH-1));
         PLB_MRdWdAddr             : in  std_logic_vector(0 to 3);
         PLB_MRdDAck               : in  std_logic;
         PLB_MRdBTerm              : in  std_logic;
         PLB_MWrDAck               : in  std_logic;
         PLB_MWrBTerm              : in  std_logic
  );

  attribute SIGIS : string;
  attribute SIGIS of SPLB_Clk      : signal is "CLK";
  attribute SIGIS of MPLB_Clk      : signal is "CLK";
  attribute SIGIS of SPLB_Rst      : signal is "RST";
  attribute SIGIS of MPLB_Rst      : signal is "RST";

end entity xil_openmac;

------------------------------------------------------------------------------
-- Architecture section
------------------------------------------------------------------------------

architecture IMP of xil_openmac is

	------------------------------------------
	-- Array of base/high address pairs for each address range
	------------------------------------------
	constant ZERO_ADDR_PAD                  : std_logic_vector(0 to 31) := (others => '0');
	constant USER_MST_BASEADDR              : std_logic_vector     := C_BASEADDR or X"00000000";
	constant USER_MST_HIGHADDR              : std_logic_vector     := C_BASEADDR or X"000000FF";

	constant IPIF_ARD_ADDR_RANGE_ARRAY      : SLV64_ARRAY_TYPE     := 
	(
	ZERO_ADDR_PAD & USER_MST_BASEADDR,  -- user logic master space base address
	ZERO_ADDR_PAD & USER_MST_HIGHADDR,  -- user logic master space high address
	ZERO_ADDR_PAD & C_MEM0_BASEADDR,    -- user logic memory space 0 base address
	ZERO_ADDR_PAD & C_MEM0_HIGHADDR     -- user logic memory space 0 high address
	);

	------------------------------------------
	-- Array of desired number of chip enables for each address range
	------------------------------------------
	constant USER_MST_NUM_REG               : integer              := 4;
	constant USER_NUM_REG                   : integer              := USER_MST_NUM_REG;
	constant USER_NUM_MEM                   : integer              := 1;

	constant IPIF_ARD_NUM_CE_ARRAY          : INTEGER_ARRAY_TYPE   := 
	(
	0  => pad_power2(USER_MST_NUM_REG), -- number of ce for user logic master space
	1  => 1                             -- number of ce for user logic memory space 0 (always 1 chip enable)
	);

	------------------------------------------
	-- Ratio of bus clock to core clock (for use in dual clock systems)
	-- 1 = ratio is 1:1
	-- 2 = ratio is 2:1
	------------------------------------------
	constant IPIF_BUS2CORE_CLK_RATIO        : integer              := 1;

	------------------------------------------
	-- Width of the slave data bus (32 only)
	------------------------------------------
	constant USER_SLV_DWIDTH                : integer              := C_SPLB_NATIVE_DWIDTH;

	constant IPIF_SLV_DWIDTH                : integer              := C_SPLB_NATIVE_DWIDTH;

	------------------------------------------
	-- Width of the master data bus (32, 64, or 128)
	------------------------------------------
	constant USER_MST_DWIDTH                : integer              := C_MPLB_NATIVE_DWIDTH;

	constant IPIF_MST_DWIDTH                : integer              := C_MPLB_NATIVE_DWIDTH;

	------------------------------------------
	-- Inhibit the automatic inculsion of the Conversion Cycle and Burst Length Expansion logic
	-- 0 = allow automatic inclusion of the CC and BLE logic
	-- 1 = inhibit automatic inclusion of the CC and BLE logic
	------------------------------------------
	constant IPIF_INHIBIT_CC_BLE_INCLUSION  : integer              := 0;

	------------------------------------------
	-- Width of the slave address bus (32 only)
	------------------------------------------
	constant USER_SLV_AWIDTH                : integer              := C_SPLB_AWIDTH;

	------------------------------------------
	-- Width of the master address bus (32 only)
	------------------------------------------
	constant USER_MST_AWIDTH                : integer              := C_MPLB_AWIDTH;

	------------------------------------------
	-- Index for CS/CE
	------------------------------------------
	constant USER_MST_CS_INDEX              : integer              := 0;
	constant USER_MST_CE_INDEX              : integer              := calc_start_ce_index(IPIF_ARD_NUM_CE_ARRAY, USER_MST_CS_INDEX);
	constant USER_MEM0_CS_INDEX             : integer              := 1;
	constant USER_CS_INDEX                  : integer              := USER_MEM0_CS_INDEX;

	constant USER_CE_INDEX                  : integer              := USER_MST_CE_INDEX;

	------------------------------------------
	-- IP Interconnect (IPIC) signal declarations
	------------------------------------------
	signal ipif_Bus2IP_Clk               : std_logic;
	signal ipif_Bus2IP_Reset             : std_logic;
	signal ipif_IP2Bus_Data              : std_logic_vector(IPIF_SLV_DWIDTH-1 downto 0);
	signal ipif_IP2Bus_WrAck             : std_logic;
	signal ipif_IP2Bus_RdAck             : std_logic;
	signal ipif_IP2Bus_Error             : std_logic;
	signal ipif_Bus2IP_Addr              : std_logic_vector(C_SPLB_AWIDTH-1 downto 0);
	signal ipif_Bus2IP_Data              : std_logic_vector(IPIF_SLV_DWIDTH-1 downto 0);
	signal ipif_Bus2IP_RNW               : std_logic;
	signal ipif_Bus2IP_BE                : std_logic_vector(IPIF_SLV_DWIDTH/8-1 downto 0);
	signal ipif_Bus2IP_CS                : std_logic_vector(((IPIF_ARD_ADDR_RANGE_ARRAY'length)/2)-1 downto 0);
	signal ipif_Bus2IP_RdCE              : std_logic_vector(0 to calc_num_ce(IPIF_ARD_NUM_CE_ARRAY)-1);
	signal ipif_Bus2IP_WrCE              : std_logic_vector(0 to calc_num_ce(IPIF_ARD_NUM_CE_ARRAY)-1);
	signal user_Bus2IP_RdCE              : std_logic_vector(0 to USER_NUM_REG-1);
	signal user_Bus2IP_WrCE              : std_logic_vector(0 to USER_NUM_REG-1);
	signal user_IP2Bus_Data              : std_logic_vector(USER_SLV_DWIDTH-1 downto 0);
	signal user_IP2Bus_RdAck             : std_logic;
	signal user_IP2Bus_WrAck             : std_logic;
	signal user_IP2Bus_Error             : std_logic;

	signal clk, rst			     : std_logic;

       --signals to mac and mii
	signal wr_n            		     : std_logic                     := '1';
	signal mac_addr        		     : std_logic_vector(10 downto 1) := (others => '0');
	signal mac_be_n        		     : std_logic_vector(1 downto 0)  := "11";
	signal mac_indata16    		     : std_logic_vector(15 downto 0) := (others => '0');
	signal mac_outdata16   		     : std_logic_vector(15 downto 0);
	signal mac_selCont     		     : std_logic                     := '0';
	signal mac_selRam      		     : std_logic                     := '0';
	signal mii_addr        		     : std_logic_vector(2 downto 0)  := (others => '0');
	signal mii_be_n        		     : std_logic_vector(1 downto 0)  := "11";
	signal mii_indata16    		     : std_logic_vector(15 downto 0) := (others => '0');
	signal mii_outdata16   		     : std_logic_vector(15 downto 0);
	signal mii_sel         		     : std_logic                     := '0';
	
	signal s_Tx_En, s_Crs_Dv	     : std_logic		     := '0';
	signal s_Rx_Dat                  : std_logic_vector ( 1 DOWNTO 0);
	signal s_Tx_Dat                  :  std_logic_vector ( 1 DOWNTO 0);
	
	--Mac Cmp Block
	signal cmp_sel			     : std_logic := '0';
	signal Mac_Zeit			     : std_logic_vector(31 downto 0) := (others => '0');

	--signals to Mac's Dma
	signal s_Dma_Req, s_Dma_Rw    	     : std_logic := '0';
	signal s_Dma_Ack              	     : std_logic := '0';
	signal s_Dma_Addr             	     : std_logic_vector(31 DOWNTO 1) := (others => '0');
	signal s_Dma_Dout             	     : std_logic_vector(15 DOWNTO 0) := (others => '0');
	signal s_Dma_Din              	     : std_logic_vector(15 DOWNTO 0) := (others => '0');
	
	signal timeout_error		     : std_logic := '0';
	
	--fifo component
	component fifo32
		port (
		clk: IN std_logic;
		rst: IN std_logic;
		din: IN std_logic_VECTOR(31 downto 0);
		wr_en: IN std_logic;
		rd_en: IN std_logic;
		dout: OUT std_logic_VECTOR(31 downto 0);
		full: OUT std_logic;
		empty: OUT std_logic);
	end component;

   component Openmac_Xilinx_IF IS
   GENERIC(             Simulate                    : 		boolean := false;
   			iBufSize_g                  : 		integer := 1024;
   			iBufSizeLOG2_g		    : 		integer := 10  ;
			useRmii_g		    : 		boolean := false);

   PORT (               Reset_n		            : IN    STD_LOGIC;
			Clk50                  	    : IN    STD_LOGIC;
			ClkFaster		    : IN    STD_LOGIC;
			ClkEth			    : IN    STD_LOGIC;

                       --Mii Core
	 ---SMI
	 Mii_D_I                   : IN     std_logic_vector(1 downto 0);
	 Mii_D_O                   : OUT    std_logic_vector(1 downto 0);
	 Mii_D_T                   : OUT    std_logic;
	 Mii_Clk                   : OUT    std_logic;
         
            ---Phy Reset
         nResetOut                 : OUT    std_logic;
         TX_IR_n                   : OUT    std_logic;
	 RX_IR_n                   : OUT    std_logic;

        wr_n            	   : in   std_logic                      ;
	mac_addr        	   : in   std_logic_vector(10 downto 1)  ;
	mac_be_n        	   : in   std_logic_vector(1 downto 0)   ;
	mac_indata16    	   : in   std_logic_vector(15 downto 0)  ;
	mac_outdata16   	   : out  std_logic_vector(15 downto 0)  ;
	mac_selCont     	   : in   std_logic                      ;
	mac_selRam      	   : in   std_logic                      ;
	mii_addr        	   : in   std_logic_vector(2 downto 0)   ;
	mii_be_n        	   : in   std_logic_vector(1 downto 0)   ;
	mii_indata16    	   : in   std_logic_vector(15 downto 0)  ;
	mii_outdata16   	   : out  std_logic_vector(15 downto 0)  ;
	mii_sel         	   : in   std_logic                      ;

        s_Dma_Req                  : out  std_logic ;
	s_Dma_Rw    	           : out  std_logic ;
	s_Dma_Ack                  : in   std_logic ;
	s_Dma_Addr                 : out  std_logic_vector(31 DOWNTO 1) ;
	s_Dma_Dout                 : out  std_logic_vector(15 DOWNTO 0) ;
	s_Dma_Din                  : in   std_logic_vector(15 DOWNTO 0) ;
	Mac_Zeit		   : out  std_logic_vector(31 downto 0) ;
	
		-- RMII Port 0
        rRx_Dat_0                  : IN    STD_LOGIC_VECTOR(1 DOWNTO 0);  -- RMII Rx Daten
        rCrs_Dv_0                  : IN    STD_LOGIC;                     -- RMII Carrier Sense / Data Valid
        rTx_Dat_0                  : OUT   STD_LOGIC_VECTOR(1 DOWNTO 0);  -- RMII Tx Daten
        rTx_En_0                   : OUT   STD_LOGIC;                     -- RMII Tx_Enable
	    -- RMII Port 1
        rRx_Dat_1                  : IN    STD_LOGIC_VECTOR(1 DOWNTO 0);  -- RMII Rx Daten
        rCrs_Dv_1                  : IN    STD_LOGIC;                     -- RMII Carrier Sense / Data Valid
        rTx_Dat_1                  : OUT   STD_LOGIC_VECTOR(1 DOWNTO 0);  -- RMII Tx Daten
        rTx_En_1                   : OUT   STD_LOGIC;                     -- RMII Tx_Enable
		--- MII PORTS
	phyMii0_RxClk		    : in    std_logic	;
	phyMii0_RxDat           : in    std_logic_vector(3 downto 0);
	phyMii0_RxDv            : in    std_logic	;
	phyMii0_TxClk		    : in    std_logic	;
	phyMii0_TxDat           : out   std_logic_vector(3 downto 0);
	phyMii0_TxEn            : out   std_logic	;
	phyMii0_Crs				: in	std_logic	;
	
	phyMii0_TxEr            : out   std_logic	;
	phyMii0_RxErr			: in	std_logic	;
	phyMii0_Col				: in	std_logic	;
	
	phyMii1_RxClk		    : in    std_logic;
	phyMii1_RxDat           : in    std_logic_vector(3 downto 0);
	phyMii1_RxDv            : in    std_logic;
	phyMii1_TxClk		    : in    std_logic;
	phyMii1_TxDat           : out   std_logic_vector(3 downto 0);
	phyMii1_TxEn            : out   std_logic;
	phyMii1_Crs             : in   std_logic;
	
	phyMii1_TxEr            : out   std_logic	;
	phyMii1_RxErr			: in	std_logic	;
	phyMii1_Col				: in	std_logic	;

	MacRxDv                 : out std_logic        ;
	MacTxEn                 : out std_logic        

        );
END component; 


begin

	clk <= SPLB_Clk;
	rst <= SPLB_Rst;
	
	--------------------------------------------------------------------------------------
	-- MASTER INTERFACE for Mac's DMA
	--------------------------------------------------------------------------------------
	MacMaster : BLOCK
		--const for master
		-------------------------------------------
		--don't change the following constants!!!--
		-------------------------------------------
		constant max_rx_burst_length : std_logic_vector(3 downto 0) := x"f"; --16 dwords
		constant max_tx_burst_length : std_logic_vector(3 downto 0) := x"f"; --16 dwords
		
		constant tx_trigger_level : integer := 4; --dwords
		constant rx_trigger_level : integer := 16; --dwords
		
		constant tx_fifo_addrsize : integer := 5;
		constant tx_fifo_size : integer := 2**tx_fifo_addrsize; -- entries dwords
		
		constant rx_fifo_addrsize : integer := 5;
		constant rx_fifo_size : integer := 2**rx_fifo_addrsize; -- entries dwords
		
		--signals for master
		signal tx_plb_owner, rx_plb_owner : std_logic := '0';
		signal wr_Dma_Ack, rd_Dma_Ack : std_logic := '0';
		
		signal wr_M_request, rd_M_request : std_logic := '0';
		signal wr_M_ABus, rd_M_ABus : std_logic_vector(31 downto 0) := (others => '0');
		signal wr_M_BE, rd_M_BE : std_logic_vector(3 downto 0);
		signal wr_M_size, rd_M_size : std_logic_vector(3 downto 0);
		
		signal wr_M_busLock, rd_M_busLock : std_logic := '0';
		
		signal timeoutcnt : std_logic_vector(31 downto 0);
		type timeout_type is(
			IDLE, TMOUT, ERROR, WAIT4TX
		);
		signal TimeoutState : timeout_type;
		
	begin

	
		
		test_dma_addr <= s_Dma_Addr & '0';
		test_dma_din <= s_Dma_Din;
		test_dma_dout <= s_Dma_Dout;
		test_dma_ack <= s_Dma_Ack;
		test_dma_req <= s_Dma_Req;
		test_dma_rw <= s_Dma_Rw;
		test_plb_req <= wr_M_request & rd_M_request;
		test_plb_ack <= PLB_MAddrAck & PLB_MWrDAck & PLB_MRdDAck;
		test_plb_owner <= tx_plb_owner & rx_plb_owner;
		test_error <= timeout_error & PLB_MTimeout & PLB_MRdErr & PLB_MWrErr & (PLB_MTimeout or PLB_MRdErr or PLB_MWrErr);
		
		M_busLock <= wr_M_busLock or rd_M_busLock;
		--lock the PLB if rx a frame or tx a packet
		
		---PLB
		s_Dma_Ack	<=		wr_Dma_Ack when s_Dma_Rw = '1' else
								rd_Dma_Ack;
		
		M_request	<=		wr_M_request or rd_M_request;
		
		M_ABus		<=		wr_M_ABus when wr_M_request = '1' else
								rd_M_ABus when rd_M_request = '1' else
								(others => '0');
		
		M_BE			<=		wr_M_BE when wr_M_request = '1' else
								rd_M_BE when rd_M_request = '1' else
								(others => '0');
		
		M_size		<=		wr_M_size when wr_M_request = '1' else
								rd_M_size when rd_M_request = '1' else
								(others => '0');
		
		M_RNW			<=		'1' when rd_M_request = '1' else
								'0';
		
		------------------------------------------------------------------------
		-- RX BLOCK
		------------------------------------------------------------------------
		RxFsm : BLOCK
			
			signal PlbFsmFinish, MacFsmFinish, RstFsm : std_logic := '0';
			
			--PLB FSM
			type PlbFsmType is (
				IDLE, --0
				WAIT4FIFO, --1
				REQ, --2
				WR, --3
				WRLAST, --4
				WAIT4RST --5
			);
			signal PlbState : PlbFsmType;
			
			--Mac FSM
			type MacFsmType is (
				IDLE, --0
				RD1, WAIT1, --1 2
				RD2, WAIT2, --3 4
				WAIT4RST --5
			);
			signal MacState : MacFsmType;
			
			--rx fifo has fifo_size x 4 bytes space
			--type DisFifoType is array (rx_fifo_size-1 downto 0) of std_logic_vector(31 downto 0);
			--signal rx_fifo : DisFifoType := (others => (others => '0'));
			signal rx_wr_index, rx_rd_index, rx_entries : std_logic_vector(rx_fifo_addrsize-1 downto 0) := (others => '0');
			signal wr_en, rd_en, full, empty : std_logic := '0';
			signal din, dout : std_logic_vector(31 downto 0) := (others => '0');			
			
			constant length_pad : std_logic_vector(11 downto rx_fifo_addrsize) := (others => '0');
			--length_pad includes zeros and is used in burst length calc in plb_fsm
			
			signal wordbuf : std_logic_vector(15 downto 0) := (others => '0');
			
		begin
			
			test_rx_plb_fsm	<=		x"0"	when PlbState = IDLE else
											x"1"	when PlbState = WAIT4FIFO else
											x"2"	when PlbState = REQ else
											x"3"	when PlbState = WR else
											x"4"	when PlbState = WRLAST else
											x"5"	when PlbState = WAIT4RST else
											x"f";
			
			test_rx_mac_fsm	<=		x"0"	when MacState = IDLE else
											x"1"	when MacState = RD1 else
											x"2"	when MacState = WAIT1 else
											x"3"	when MacState = RD2 else
											x"4"	when MacState = WAIT2 else
											x"5"	when MacState = WAIT4RST else
											x"f";
			
			--rx fifo
			rx_fifo : fifo32
			port map (
				clk => clk,
				rst => rst,
				din => din,
				wr_en => wr_en,
				rd_en => rd_en,
				dout => dout,
				full => full,
				empty => empty
			);
			
--				--rx fifo test signals
--				test_rxFifo_rd <= rd_en;
--				test_rxFifo_wr <= wr_en;
--				test_rxFifo_full <= full;
--				test_rxFifo_empty <= empty;
--				test_rxFifo_din <= din;
--				test_rxFifo_dout <= dout;
			
			RstFsm <= PlbFsmFinish and MacFsmFinish;
			
			wr_M_busLock <= '0' when PlbState = IDLE else '1';
			
			--rx_calc_level : process(rx_wr_index, rx_rd_index)
			--begin
			--still calc the entries for trigger event
			rx_entries <= rx_wr_index - rx_rd_index;
			--end process;
			
			--M_wrDBus <= rx_fifo(conv_integer(rx_rd_index));
			M_wrDBus <= dout;
			din <= wordbuf & s_Dma_Dout;
			rd_en <= PLB_MWrDAck;
			
			PLB_FSM : process(clk)
				variable burst_length : std_logic_vector(11 downto 0) := (others => '0');
				variable addr : std_logic_vector(31 downto 0);
				variable IsBurst, reminder : std_logic := '0';
			begin
				if clk = '1' and clk'event then
					if rst = '1' then
						PlbState <= IDLE;
						PlbFsmFinish <= '0';
						rx_rd_index <= (others => '0');
						burst_length := (others => '0');
						rx_plb_owner <= '0';
						wr_M_request <= '0';
						wr_M_ABus <= (others => '0');
						wr_M_BE <= (others => '0');
						wr_M_size <= (others => '0');
						M_wrBurst <= '0';
						IsBurst := '0'; reminder := '0';
					else
						--default values
						PlbFsmFinish <= '0';
						
						rx_plb_owner <= '0';
						wr_M_request <= '0';
						wr_M_ABus <= (others => '0');
						wr_M_BE <= (others => '0');
						wr_M_size <= (others => '0');
						M_wrBurst <= '0';
						
						--rd_en <= '0'; --fifo read
						
						case PlbState is
						when IDLE => --0
							IsBurst := '0'; reminder := '0';
							if s_Dma_Req = '1' and s_Dma_Rw = '0' then
								PlbState <= WAIT4FIFO;
								addr := "00" & s_Dma_Addr(29 downto 1) & '0'; --start address
							else
								PlbState <= IDLE;
							end if;
							
						when WAIT4FIFO => --1
							if MacFsmFinish = '1' and conv_integer(rx_entries) = 0 then
								--no entries in fifo, mac is finish => rst fsm
								PlbState <= WAIT4RST;
							elsif tx_plb_owner = '1' then
								--tx is in progress wait for
								PlbState <= WAIT4FIFO;
							elsif MacFsmFinish = '1' and conv_integer(rx_entries) /= 0 then
								--entries left in fifo
								PlbState <= REQ;
							elsif MacFsmFinish = '0' and conv_integer(rx_entries) >= rx_trigger_level then
								--trigger event... start write command
								PlbState <= REQ;
							else
								--RX is running no trigger event
								-- wait for entries in fifo or end of rx
								PlbState <= WAIT4FIFO;
							end if;
							
						when REQ => --2
							--start plb write req
							burst_length := length_pad & rx_entries; --update burst length in dwords
							burst_length := burst_length(9 downto 0) & "00"; --quad. to get bytes
							burst_length := burst_length + x"003"; --round up for mod 4
							burst_length := burst_length(11 downto 2) & "00"; --burst width is 4 bytes!
							
							PlbState <= REQ;
							rx_plb_owner <= '1';
							wr_M_request <= '1';
							wr_M_ABus <= addr;
							burst_length := burst_length - 1;
							wr_M_BE <= burst_length(5 downto 2);
							if burst_length(5 downto 2) = "0000" then
								--only one dword to transfer!
								wr_M_size <= "0000"; --dword
								M_wrBurst <= '0';
								wr_M_BE <= "1111";
								IsBurst := '0';
							else
								wr_M_size <= "1010"; --dwords
								M_wrBurst <= '1';
								IsBurst := '1';
							end if;
							burst_length := burst_length + 1;
							if PLB_MAddrAck = '1' then
								PlbState <= WR;
								rx_plb_owner <= '0';
								wr_M_request <= '0';
								wr_M_ABus <= (others => '0');
								wr_M_BE <= (others => '0');
								wr_M_size <= (others => '0');
							elsif PLB_MWrErr = '1' or PLB_MTimeout = '1' then
								PlbState <= WAIT4RST;
							end if;
							--first write possible in req phase
							if IsBurst = '1' then
								--is burst transfer
								M_wrBurst <= '1';
								if PLB_MWrDAck = '1' then
									rx_rd_index <= rx_rd_index + 1; --rd_en <= '1';
									if PLB_MWrBTerm = '1' then
										PlbState <= WRLAST;
										M_wrBurst <= '0'; --the last dword follows
									end if;
								elsif PLB_MWrErr = '1' or PLB_MTimeout = '1' then
									PlbState <= WAIT4RST;
								end if;
							else
								--is single beat transfer (no term signal!!!)
								M_wrBurst <= '0'; wr_M_BE <= "1111";
								if PLB_MWrDAck = '1' then
									rx_rd_index <= rx_rd_index + 1; --rd_en <= '1';
									PlbState <= WAIT4FIFO;
								elsif PLB_MWrErr = '1' or PLB_MTimeout = '1' then
									PlbState <= WAIT4RST;
								end if;
							end if;
							
						when WR => --3
							PlbState <= WR;
							if IsBurst = '1' then
								--is burst transfer
								M_wrBurst <= '1';
								if PLB_MWrDAck = '1' then
									rx_rd_index <= rx_rd_index + 1; --rd_en <= '1';
									if PLB_MWrBTerm = '1' then
										PlbState <= WRLAST;
										M_wrBurst <= '0'; --the last dword follows
									end if;
								elsif PLB_MWrErr = '1' or PLB_MTimeout = '1' then
									PlbState <= WAIT4RST;
								end if;
							else
								--is single beat transfer (no term signal!!!)
								M_wrBurst <= '0'; wr_M_BE <= "1111";
								if PLB_MWrDAck = '1' then
									rx_rd_index <= rx_rd_index + 1; --rd_en <= '1';
									PlbState <= WAIT4FIFO;
								elsif PLB_MWrErr = '1' or PLB_MTimeout = '1' then
									PlbState <= WAIT4RST;
								end if;
							end if;
														
						when WRLAST => --4
							PlbState <= WRLAST;
							if PLB_MWrDAck = '1' then
								rx_rd_index <= rx_rd_index + 1; --rd_en <= '1';
								addr := addr + burst_length;
								PlbState <= WAIT4FIFO;
							elsif PLB_MWrErr = '1' or PLB_MTimeout = '1' then
								PlbState <= WAIT4RST;
							end if;
							
						when WAIT4RST => --5
							PlbFsmFinish <= '1';
							if RstFsm = '1' then
								PlbState <= IDLE;
								rx_rd_index <= (others => '0');
							else
								PlbState <= WAIT4RST;
							end if;
						end case;
						
					end if; --rst
				end if; --clk
			end process;
			
			MAC_FSM : process(clk)
--				variable wordbuf : std_logic_vector(15 downto 0) := (others => '0');
				variable timeout : integer range 0 to 11 := 0;
			begin
				if clk = '1' and clk'event then
					if rst = '1' then
						MacFsmFinish <= '0';
						MacState <= IDLE;
						rx_wr_index <= (others => '0');
						wordbuf <= (others => '0');
						rd_Dma_Ack <= '1';
						timeout := 0;
					else
						--default values
						MacFsmFinish <= '0';
						rd_Dma_Ack <= '0';
						
						wr_en <= '0';
						
						case MacState is
						when IDLE => --0
							timeout := 0;
							if s_Dma_Req = '1' and s_Dma_Rw = '0' then
								MacState <= RD1;
							else
								MacState <= IDLE;
							end if;
							
						when RD1 => --1
							if s_Dma_Req = '1' and s_Dma_Rw = '0' then
								wordbuf <= s_Dma_Dout;
								rd_Dma_Ack <= '1';
								MacState <= WAIT1;
							elsif s_Crs_Dv = '0' then
								if timeout = 11 then
									MacState <= WAIT4RST;
								else
									MacState <= RD1;
									timeout := timeout + 1;
								end if;
							else
								MacState <= RD1;
							end if;
							
						when WAIT1 => --2
							MacState <= RD2;
							timeout := 0;
							
						when RD2 => --3
							if s_Dma_Req = '1' and s_Dma_Rw = '0' then
								--rx_fifo(conv_integer(rx_wr_index)) <= wordbuf & s_Dma_Dout;
								rd_Dma_Ack <= '1';
								MacState <= WAIT2;
							elsif s_Crs_Dv = '0' then
								if timeout = 11 then
									MacState <= WAIT4RST;
									rx_wr_index <= rx_wr_index + 1; wr_en <= '1'; --write last byte to fifo
								else
									MacState <= RD2;
									timeout := timeout + 1;
								end if;
							else
								MacState <= RD2;
							end if;
							
						when WAIT2 => --4
							MacState <= RD1;
							timeout := 0;
							rx_wr_index <= rx_wr_index + 1; wr_en <= '1';
							
						when WAIT4RST => --5
							MacFsmFinish <= '1';
							if RstFsm = '1' then
								MacState <= IDLE;
								rx_wr_index <= (others => '0');
							else
								MacState <= WAIT4RST;
							end if;
						end case;
						
					end if; --rst
				end if; --clk
			end process;
			
		end block RxFsm;
		
		------------------------------------------------------------------------
		-- TX BLOCK
		------------------------------------------------------------------------
		TxFsm : BLOCK
			signal PlbFsmFinish, MacFsmFinish, RstFsm : std_logic := '0';
			signal TxInProgress : std_logic := '0';
			signal wordbuf : std_logic_vector(15 downto 0);
			
			--PLB Fsm
			type PlbFsmType is (
				IDLE,
				REQ,
				RD, LASTRD,
				WAIT4JOB,
				WAIT4RST
			);
			signal PlbState : PlbFsmType;
			
			--MAC Fsm
			type MacFsmType is (
				IDLE,
				WAIT4FIFO,
				RD1, WAIT1,
				RD2, WAIT2,
				WAIT4RST
			);
			signal MacState : MacFsmType;
			
			--TX Fsm
			type TxFsmType is (
				IDLE,
				IFGAP, IFGAPTXOWNS,
				TX,
				WAITSOMETIME
			);
			signal TxState : TxFsmType;
			
			-- tx fifo (distributed ram)
--			type DisFifoType is array (tx_fifo_size-1 downto 0) of std_logic_vector(31 downto 0);
--			signal tx_fifo : DisFifoType := (others => (others => '0'));
			signal tx_wr_index, tx_rd_index, tx_entries : std_logic_vector(tx_fifo_addrsize-1 downto 0) := (others => '0');
			signal wr_en, rd_en, full, empty : std_logic := '0';
			signal din, dout : std_logic_vector(31 downto 0) := (others => '0');
			
		begin
			
			test_tx_fsm			<=		x"0"	when TxState = IDLE else
											x"1"	when TxState = IFGAP else
											x"2"	when TxState = IFGAPTXOWNS else
											x"3"	when TxState = TX else
											x"4"	when TxState = WAITSOMETIME else
											x"f";
											
			test_tx_plb_fsm	<=		x"0"	when PlbState = IDLE else
											x"1"	when PlbState = REQ else
											x"2"	when PlbState = RD else
											x"3"	when PlbState = LASTRD else
											x"4"	when PlbState = WAIT4JOB else
											x"5"	when PlbState = WAIT4RST else
											x"f";
			
			test_tx_mac_fsm	<=		x"0"	when MacState = IDLE else
											x"1"	when MacState = WAIT4FIFO else
											x"2"	when MacState = RD1 else
											x"3"	when MacState = WAIT1 else
											x"4"	when MacState = RD2 else
											x"5"	when MacState = WAIT2 else
											x"6"	when MacState = WAIT4RST else
											x"f";
			
			process(clk)
				variable timeout : integer range 0 to 63 := 0;
			begin
				if clk = '1' and clk'event then
					if rst = '1' then
						timeout := 0; timeout_error <= '0';
					else
						timeout_error <= '0';
						if MacState = WAIT4FIFO then
							if timeout = 34 then --increase const to find errors
								timeout_error <= '1';
								timeout := 0;
							else
								timeout := timeout + 1;
							end if;
						else
							timeout := 0;
						end if;
					end if;
				end if; --clk
			end process;
			
			RstFsm <= PlbFsmFinish and MacFsmFinish; --reset Fsm
			
			rd_M_busLock <= '0' when PlbState = IDLE else '1';
			
			--tx_calc_level : process(tx_wr_index, tx_rd_index)
			--begin
				tx_entries <= tx_wr_index - tx_rd_index;
			--end process;
			
			TX_IN_PROGRESS : process(clk)
				variable cnt : integer range 0 to 63 := 0;
				variable wasCrsDv : std_logic_vector(3 downto 0) := (others => '0');
			begin
			if clk = '1' and clk'event then
				if rst = '1' then
					TxInProgress <= '0';
					TxState <= IDLE;
					wasCrsDv := (others => '0');
				else
					TxInProgress <= '0';
--					s_M_busLock <= '1';
					-- bus lock is default state
					
					case TxState is
					when IDLE => --0
						--wait falling CrsDv
--						s_M_busLock <= '0';
						--in IDLE - no buslock
						TxState <= IDLE; cnt := 0;
						if wasCrsDv = "1100" then
							--RX is done ... count IFGAP!
							TxState <= IFGAP;
--							s_M_busLock <= '1';
							--lock bus, maybe there will be an auto resp!
						elsif s_Tx_En = '1' then
							--manual TX
							TxState <= TX;
							TxInProgress <= '1';
						end if;
					when IFGAP => --1
						--node is in interframe gap
						TxState <= IFGAP;
						if s_Tx_En = '1' then
							--mac starts tx
							TxState <= TX;
							TxInProgress <= '1';
						elsif cnt = 63 then
							--there is no TX
							TxState <= IDLE;
							cnt := 0;
						else
							--wait for IFGAP++
							cnt := cnt + 1;
							TxState <= IFGAP;
						end if;
						if s_Dma_Req = '1' and s_Dma_Rw = '1' then
							--auto resp. frame will follow!
							TxInProgress <= '1'; TxState <= IFGAPTXOWNS;
							cnt := 0;
						end if;
					when IFGAPTXOWNS => --2
						--IFGAP, but tx frame will follow!
						TxInProgress <= '1'; --avoid mac fsm rst!
						if s_Tx_En = '1' then
							--mac starts tx
							TxState <= TX;
						else
							TxState <= IFGAPTXOWNS;
						end if;
					when TX => --3
						--mac sends frame
						TxInProgress <= '1';
						if s_Tx_En = '1' then
							TxState <= TX;
						else
							--tx finished
							TxInProgress <= '0';
							TxState <= WAITSOMETIME;
						end if;
					when WAITSOMETIME => --4
						--wait another IFGAP++ to rst fsm
						if cnt = 63 then
							cnt := 0;
							TxState <= IDLE;
						else
							cnt := cnt + 1;
						end if;
					end case;
					--for next process memorise CrsDv State
					wasCrsDv := wasCrsDv(wasCrsDv'left-1 downto 0) & s_Crs_Dv;
					
				end if;
			end if;
			end process;
			
			--tx fifo
			tx_fifo : fifo32
			port map (
				clk => clk,
				rst => rst,
				din => din,
				wr_en => wr_en,
				rd_en => rd_en,
				dout => dout,
				full => full,
				empty => empty
			);
			
			wr_en <= PLB_MRdDAck;
			din <= PLB_MRdDBus;
			
			--tx fifo test signals
			test_txFifo_rd <= rd_en;
			test_txFifo_wr <= wr_en;
			test_txFifo_full <= full;
			test_txFifo_empty <= empty;
			test_txFifo_din <= din;
			test_txFifo_dout <= dout;
			
			PLB_FSM : process(clk)
				variable addr : std_logic_vector(31 downto 0) := (others => '0');
			begin
				if clk = '1' and clk'event then
					if rst = '1' then
						PlbState <= IDLE;
						PlbFsmFinish <= '0'; tx_plb_owner <= '0';
						addr := (others => '0');
						tx_wr_index <= (others => '0');
						rd_M_request <= '0'; rd_M_ABus <= (others => '0'); rd_M_BE <= (others => '0');
						M_rdBurst <= '0'; rd_M_size <= (others => '0');
					else
						--default value
						PlbFsmFinish <= '0';
						tx_plb_owner <= '0';
						rd_M_request <= '0'; rd_M_ABus <= (others => '0'); rd_M_BE <= (others => '0');
						M_rdBurst <= '0'; rd_M_size <= (others => '0');
						
						case PlbState is
						when IDLE => --0
							addr := "00" & s_Dma_Addr(29 downto 1) & '0'; --get first address
							if s_Dma_Req = '1' and s_Dma_Rw = '1' and rx_plb_owner = '0' then
								--start request
								tx_plb_owner <= '1';
								PlbState <= REQ;
								rd_M_request <= '1';
								rd_M_ABus <= addr; --start address
								rd_M_BE <= max_tx_burst_length;
								rd_M_size <= "1010"; --4byte bursts
							else
								PlbState <= IDLE;
							end if;
							
						when REQ => --1
							--hold request until ack
							tx_plb_owner <= '1';
							rd_M_request <= '1';
							rd_M_ABus <= addr;
							rd_M_BE <= max_tx_burst_length;
							rd_M_size <= "1010"; --4byte bursts
							if PLB_MAddrAck = '1' then
								PlbState <= RD;
								--ack => zero req signals
								tx_plb_owner <= '0';
								rd_M_request <= '0';
								rd_M_ABus <= (others => '0');
								rd_M_BE <= (others => '0');
								rd_M_size <= (others => '0');
								--set burst qualifier
								M_rdBurst <= '1'; --this is a burst transfer!
							elsif PLB_MRdErr = '1' or PLB_MTimeout = '1' then
								PlbState <= WAIT4RST;
							else
								PlbState <= REQ;
							end if;
								
						when RD => --2
							--burst read transfer
							M_rdBurst <= '1'; --this is a burst transfer!
							PlbState <= RD;
							if PLB_MRdDAck = '1' then
								--valid data => put in fifo
								--tx_fifo(conv_integer(tx_wr_index)) <= PLB_MRdDBus;
								tx_wr_index <= tx_wr_index + 1; addr := addr + 4;
								if PLB_MRdBTerm = '1' then
									PlbState <= LASTRD;
									M_rdBurst <= '0'; --the last dword follows
								end if;
							end if;
							
						when LASTRD => --3
							--last Read, then change to next state
							PlbState <= LASTRD;
							if PLB_MRdDAck = '1' then
								--valid data => put in fifo
								--tx_fifo(conv_integer(tx_wr_index)) <= PLB_MRdDBus;
								tx_wr_index <= tx_wr_index + 1; addr := addr + 4;
								PlbState <= WAIT4JOB;
							end if;
							
						when WAIT4JOB => --4
							--wait for a new job
							--	if fifo is almost empty (check cnt signal) => new job
							-- if tx is over, wait4rst
							if TxInProgress = '0' then
								PlbState <= WAIT4RST;
							elsif tx_entries <= tx_trigger_level and rx_plb_owner = '0' then
								--fifo is nearly empty => new REQ
								PlbState <= REQ;
							else
								PlbState <= WAIT4JOB;
							end if;
							
						when WAIT4RST => --5
							PlbFsmFinish <= '1';
							if RstFsm = '1' then
								PlbState <= IDLE;
								tx_wr_index <= (others => '0');
							else
								PlbState <= WAIT4RST;
							end if;
							
						end case;
					end if; --rst
				end if; --clk
			end process;
			
			--DMA input to FIFO
			s_Dma_Din <= dout(31 downto 16) when (MacState = WAIT1 or MacState = RD2) else
							 dout(15 downto 0)  when (MacState = WAIT2 or MacState = RD1) else
							 (others => '0');
			
			MAC_FSM : process(clk)
			begin
				if clk = '1' and clk'event then
					if rst = '1' then
						MacState <= IDLE;
						MacFsmFinish <= '0';
						wordbuf <= (others => '0');
						tx_rd_index <= (others => '0');
					else
						--default value
						MacFsmFinish <= '0';
						wr_Dma_Ack <= '0';
						--s_Dma_Din <= (others => '0');
						rd_en <= '0';
						
						case MacState is
						when IDLE => --0
							wordbuf <= (others => '0');
							if s_Dma_Req = '1' and s_Dma_Rw = '1' then
								MacState <= WAIT4FIFO;
							else
								MacState <= IDLE;
							end if;
						
						when WAIT4FIFO => --1
							if s_Dma_Req = '1' and empty = '0' then --and conv_integer(tx_entries) /= 0 then
								--wordbuf <= tx_fifo(conv_integer(tx_rd_index))(15 downto 0);
								--s_Dma_Din <= tx_fifo(conv_integer(tx_rd_index))(31 downto 16);
								wr_Dma_Ack <= '1';
								MacState <= WAIT1;
							else
								MacState <= WAIT4FIFO;
							end if;
							
						when RD1 => --2
							--wordbuf <= tx_fifo(conv_integer(tx_rd_index))(15 downto 0);
							--s_Dma_Din <= tx_fifo(conv_integer(tx_rd_index))(31 downto 16);
							if conv_integer(tx_entries) /= 0 and s_Dma_Req = '1' then
								wr_Dma_Ack <= '1';
								MacState <= WAIT1;
							elsif TxInProgress = '0' then
								MacState <= WAIT4RST;
							else
								MacState <= RD1;
							end if;
							
						when WAIT1 => --3
							MacState <= RD2;
							--s_Dma_Din <= tx_fifo(conv_integer(tx_rd_index))(31 downto 16);
							
						when RD2 => --4
							--s_Dma_Din <= wordbuf;
							if conv_integer(tx_entries) /= 0 and s_Dma_Req = '1' then
								wr_Dma_Ack <= '1';
								MacState <= WAIT2;
							elsif TxInProgress = '0' then
								MacState <= WAIT4RST;
							else
								MacState <= RD2;
							end if;
							
						when WAIT2 => --5
							MacState <= RD1;
							--s_Dma_Din <= wordbuf;
							tx_rd_index <= tx_rd_index + 1; rd_en <= '1';
							
						when WAIT4RST => --6
							if empty = '1' then
								MacFsmFinish <= '1';
								if RstFsm = '1' then
									MacState <= IDLE;
									tx_rd_index <= (others => '0');
								else
									MacState <= WAIT4RST;
								end if;
							else
								rd_en <= '1'; --rd entries to empty fifo
							end if;
							
						end case;
					end if; --rst
				end if; --clk
			end process;
			
		end block TxFsm;
		
	end block MacMaster;
	
	--------------------------------------------------------------------------------------
	-- SLAVE INTERFACE for Mac and Mii
	--------------------------------------------------------------------------------------
	slv_fsm : BLOCK

		constant wait_write : integer := 1;
		constant wait_read : integer := 2;
		constant wait_idle : integer := 1;
		
		signal cs, wr, rd    : std_logic;
		signal be32          : std_logic_vector(3 downto 0);
		signal addr          : std_logic_vector(31 downto 0);
		signal rdAck, wrAck  : std_logic := '0';

		signal s_mii_sel, s_mac_selRam, s_mac_selCont, filter_sel, s_wr : std_logic := '0';
		signal s_indata16, s_outdata16 : std_logic_vector(15 downto 0) := (others => '0');
		signal s_addr, s_addrplus : std_logic_vector(31 downto 0) := (others => '0');
		
		--Mac Cmp Unit
		signal Mac_Cmp_Wert : std_logic_vector(31 downto 0) := (others => '0');
		signal Mac_Cmp_Int : std_logic := '0';
		signal Mac_Cmp_On : std_logic := '0';
		
		signal	Mac_Cmp_TogVal : std_logic_vector(31 downto 0) := (others => '0');
		signal	Mac_Tog_On		: std_logic := '0';
		SIGNAL  Mac_Cmp_Toggle : STD_LOGIC;

		--slave slv_state machine
		type SLV_STATE_TYPE IS (
		 sIDLE,
		 sWRL, sWRH,
		 sRDL, sRDH,
		 sWR,
		 sRD,
		 sACKRD, sACKWR,
		 sCMPWR, sCMPRD --write to / read from Timer Cmp unit
		);
		signal slv_state : SLV_STATE_TYPE;

	begin
		
		test_cmp_sel <= cmp_sel; --1
		test_cmp_addr <= addr; --32
		test_cmp_wert <= Mac_Cmp_Wert; --32
		test_cmp_int <= Mac_Cmp_Int; --1
		test_cmp_on <= Mac_Cmp_On; --1
		test_cmp_din <= ipif_Bus2IP_Data; --32
		test_cmp_dout <= user_IP2Bus_Data; --32
		
		cs <= ipif_Bus2IP_CS(0);
		wr <= not ipif_Bus2IP_RNW;
		rd <= ipif_Bus2IP_RNW;
		be32 <= ipif_Bus2IP_BE;
		addr <= ipif_Bus2IP_Addr;
		user_IP2Bus_RdAck <= rdAck;
		user_IP2Bus_WrAck <= wrAck;
		
		mac_indata16  <=  s_indata16;
		mii_indata16  <=  s_indata16;

		s_outdata16   <=  mac_outdata16 when (s_mac_selRam = '1' and filter_sel = '0') or s_mac_selCont = '1' else
							   mii_outdata16 when s_mii_sel = '1' else
							  (others => '0');

		--addr
		cmp_sel		  <=  '1' when (addr(15 downto 12) = x"3"  and cs = '1') else '0';		-- 0x3xxx
		--s_addr
		s_mii_sel     <=  '1' when (s_addr(15 downto 12) = x"2"  and cs = '1') else '0';		-- 0x2xxx
		s_mac_selRam  <=  '1' when (s_addr(15 downto 12) = x"1"  and cs = '1') else '0';		-- 0x1xxx
		filter_sel	  <=  '1' when (s_addr(15 downto 12) = x"1" 
							         and s_addr(11 downto 10) = "00"  and cs = '1') else '0';		-- 0x1000 to 0x13ff
		s_mac_selCont <=  '1' when (s_addr(15 downto 12) = x"0"  and cs = '1') else '0';		-- 0x0xxx

		mii_sel       <=  s_mii_sel;
		mac_selRam    <=  s_mac_selRam;
		mac_selCont   <=  s_mac_selCont;

		mii_addr      <=  s_addr(3 downto 1);
		mac_addr	  	  <=  s_addr(10 downto 2) & not s_addr(1) when s_mac_selRam = '1' and s_addr(11 downto 10) = "00" else
						      s_addr(10 downto 2) &     s_addr(1);

		mii_be_n      <=  not((be32(3) or be32(1)) & (be32(2) or be32(0)));
		mac_be_n      <=  not((be32(3) or be32(1)) & (be32(2) or be32(0)));

		wr_n <= not s_wr;
		
		addrplus : process(addr)
		begin
		 s_addrplus <= addr + 2;
		end process;

		slv_state_machine : process(clk)
		 variable tmp_buffer : std_logic_vector(31 downto 0) := (others => '0');
		 variable i : integer range 0 to 2 := 0;
		begin
		 if clk = '1' and clk'event then
			if rst = '1' then
			  slv_state <= sIDLE;
			  user_IP2Bus_Data <= (others => '0');
			  s_indata16 <= (others => '0');
			  s_addr <= (others => '0');
			  s_wr <= '0';
			  wrAck <= '0';
			  rdAck <= '0';
			  
			  --Mac_Cmp_Int <= '0';
			  Mac_Tog_On <= '0' ;
			  Mac_Cmp_On <= '0';
			  Mac_Cmp_Wert <= (others => '0');
			  Mac_Cmp_TogVal <= (others => '0');
			else
			  --default values
			  user_IP2Bus_Data <= (others => '0');
			  s_indata16 <= (others => '0');
			  s_addr <= (others => '0');
			  s_wr <= '0';
			  wrAck <= '0';
			  rdAck <= '0';
			  
			  case slv_state is
			  when sIDLE => --x0
				 s_addr <= (others => '0');
				 if i /= 0 then
					i := i - 1;
					slv_state <= sIDLE;
				 --elsif ???
				 --add 32bit dev. here
				 --...
				 elsif cmp_sel = '1' then
					if wr = '1' then
						slv_state <= sCMPWR;
					else
						slv_state <= sCMPRD;
					end if;
				 --16bit devices
				 elsif cs = '1' and be32 /= "1111" then --16bit / 8bit
					if wr = '1' then
					  slv_state <= sWR;
					  i := wait_write;
					elsif rd = '1' then
					  slv_state <= sRD;
					  i := wait_read;
					else
					  slv_state <= sIDLE;
					end if;
				 elsif cs = '1' and be32 = "1111" then --32bit
					if wr = '1' then
					  slv_state <= sWRL;
					  i := wait_write;
					elsif rd = '1' then
					  slv_state <= sRDL;
					  i := wait_read;
					else
					  slv_state <= sIDLE;
					end if;
				 else
					slv_state <= sIDLE;
				 end if;				 
			  
			  when sWRL => --x1
				 s_indata16 <= ipif_Bus2IP_Data(31 downto 16);
				 s_addr <= addr;
				 s_wr <= '1';
				 if i = 0 then
					slv_state <= sWRH;
					i := wait_write;
				 else
					i := i - 1;
				 end if;
			  when sWRH => --x2
				 s_indata16 <= ipif_Bus2IP_Data(15 downto 0);
				 s_addr <= s_addrplus;
				 s_wr <= '1';
				 if i = 0 then
					slv_state <= sACKWR;
					s_wr <= '0';
				 else
					i := i - 1;
				 end if;
				 
			  when sRDL => --x3
				 s_addr <= addr;
				 if i = 0 then
					tmp_buffer(31 downto 16) := s_outdata16;
					slv_state <= sRDH;
					i := wait_read;
				 else
					i := i - 1;
				 end if;
			  when sRDH => --x4
				 s_addr <= s_addrplus;
				 if i = 0 then
					tmp_buffer(15 downto 0) := s_outdata16;
					slv_state <= sACKRD;
					i := wait_read;
				 else
					i := i - 1;
				 end if;
				 
			  when sWR => --x5
					s_addr <= addr;
				 if be32(3) = '1' or be32(2) = '1' then
					s_indata16 <= ipif_Bus2IP_Data(15 downto 0);
				 else
					s_indata16 <= ipif_Bus2IP_Data(31 downto 16);
				 end if;
				 s_wr <= '1';
				 if i = 0 then
					slv_state <= sACKWR;
					s_wr <= '0';
				 else
					i := i - 1;
				 end if;
				 
			  when sRD => --x6
				 s_addr <= addr;
				 if i = 0 then
					tmp_buffer := s_outdata16 & s_outdata16;
					slv_state <= sACKRD;
				 else
					i := i - 1;
				 end if;
				 
			  when sACKRD => --x7
				 rdAck <= '1';
				 user_IP2Bus_Data <= tmp_buffer;
				 slv_state <= sIDLE;
				 i := wait_idle;
			  
			  when sACKWR => --x8
				 wrAck <= '1';
				 slv_state <= sIDLE;
				 i := wait_idle;
				
				when sCMPWR =>
					--wrAck <= '1'; --ack immediately
					case addr(3 downto 2) is
						when "00" => --0
							Mac_Cmp_Wert <= ipif_Bus2IP_Data;
							Mac_Cmp_Int <= '0';
						when "01" => --4
							Mac_Cmp_On <= ipif_Bus2IP_Data(0);
							Mac_Tog_On <= ipif_Bus2IP_Data(4);
						when "10" => --8
							Mac_Cmp_TogVal <= ipif_Bus2IP_Data ;
						when others => --do nothing
					end case;
					slv_state <= sACKWR;
				
				when sCMPRD =>
					--rdAck <= '1'; --ack immediately
					case addr(3 downto 2) is
						when "00" => --0
							tmp_buffer := Mac_Zeit;
						when "01" => --4
							tmp_buffer := x"000000" & "00" & Mac_Cmp_Toggle & Mac_Tog_On & "00" & Mac_Cmp_Int & Mac_Cmp_On;
						when "10" => --8
							 tmp_buffer := Mac_Cmp_TogVal ;
						when others =>
							tmp_buffer := (others => '0'); 
					end case;
					slv_state <= sACKRD;
				
			  end case;
			Mac_Cmp_Int <= '0';
			if Mac_Cmp_On = '1' and Mac_Cmp_Wert = Mac_Zeit then
				Mac_Cmp_Int <= '1';
			end if;
			
			IF ( Mac_Tog_On = '1' and Mac_Cmp_TogVal= Mac_Zeit ) THEN
					Mac_Cmp_Toggle <= not Mac_Cmp_Toggle;
			END IF;

			 
			end if;
		 end if; --clk'event
		end process;
		
		Cmp_IR_n <= not Mac_Cmp_Int;
		
		t_IrqToggle <= Mac_Cmp_Toggle ;

	end BLOCK slv_fsm;

	------------------------------------------
	-- instantiate plbv46_slave_single
	------------------------------------------
	PLBV46_SLAVE_SINGLE_I : entity plbv46_slave_single_v1_01_a.plbv46_slave_single
	generic map
	(
		C_ARD_ADDR_RANGE_ARRAY         => IPIF_ARD_ADDR_RANGE_ARRAY,
		C_ARD_NUM_CE_ARRAY             => IPIF_ARD_NUM_CE_ARRAY,
		C_SPLB_P2P                     => C_SPLB_P2P,
		C_BUS2CORE_CLK_RATIO           => IPIF_BUS2CORE_CLK_RATIO,
		C_SPLB_MID_WIDTH               => C_SPLB_MID_WIDTH,
		C_SPLB_NUM_MASTERS             => C_SPLB_NUM_MASTERS,
		C_SPLB_AWIDTH                  => C_SPLB_AWIDTH,
		C_SPLB_DWIDTH                  => C_SPLB_DWIDTH,
		C_SIPIF_DWIDTH                 => IPIF_SLV_DWIDTH,
		C_INCLUDE_DPHASE_TIMER         => C_INCLUDE_DPHASE_TIMER,
		C_FAMILY                       => C_FAMILY
	)
	port map
	(
		SPLB_Clk                       => clk,
		SPLB_Rst                       => rst,
		PLB_ABus                       => PLB_ABus,
		PLB_UABus                      => PLB_UABus,
		PLB_PAValid                    => PLB_PAValid,
		PLB_SAValid                    => PLB_SAValid,
		PLB_rdPrim                     => PLB_rdPrim,
		PLB_wrPrim                     => PLB_wrPrim,
		PLB_masterID                   => PLB_masterID,
		PLB_abort                      => PLB_abort,
		PLB_busLock                    => PLB_busLock,
		PLB_RNW                        => PLB_RNW,
		PLB_BE                         => PLB_BE,
		PLB_MSize                      => PLB_MSize,
		PLB_size                       => PLB_size,
		PLB_type                       => PLB_type,
		PLB_lockErr                    => PLB_lockErr,
		PLB_wrDBus                     => PLB_wrDBus,
		PLB_wrBurst                    => PLB_wrBurst,
		PLB_rdBurst                    => PLB_rdBurst,
		PLB_wrPendReq                  => PLB_wrPendReq,
		PLB_rdPendReq                  => PLB_rdPendReq,
		PLB_wrPendPri                  => PLB_wrPendPri,
		PLB_rdPendPri                  => PLB_rdPendPri,
		PLB_reqPri                     => PLB_reqPri,
		PLB_TAttribute                 => PLB_TAttribute,
		Sl_addrAck                     => Sl_addrAck,
		Sl_SSize                       => Sl_SSize,
		Sl_wait                        => Sl_wait,
		Sl_rearbitrate                 => Sl_rearbitrate,
		Sl_wrDAck                      => Sl_wrDAck,
		Sl_wrComp                      => Sl_wrComp,
		Sl_wrBTerm                     => Sl_wrBTerm,
		Sl_rdDBus                      => Sl_rdDBus,
		Sl_rdWdAddr                    => Sl_rdWdAddr,
		Sl_rdDAck                      => Sl_rdDAck,
		Sl_rdComp                      => Sl_rdComp,
		Sl_rdBTerm                     => Sl_rdBTerm,
		Sl_MBusy                       => Sl_MBusy,
		Sl_MWrErr                      => Sl_MWrErr,
		Sl_MRdErr                      => Sl_MRdErr,
		Sl_MIRQ                        => Sl_MIRQ,
		Bus2IP_Clk                     => ipif_Bus2IP_Clk,
		Bus2IP_Reset                   => ipif_Bus2IP_Reset,
		IP2Bus_Data                    => ipif_IP2Bus_Data,
		IP2Bus_WrAck                   => ipif_IP2Bus_WrAck,
		IP2Bus_RdAck                   => ipif_IP2Bus_RdAck,
		IP2Bus_Error                   => ipif_IP2Bus_Error,
		Bus2IP_Addr                    => ipif_Bus2IP_Addr,
		Bus2IP_Data                    => ipif_Bus2IP_Data,
		Bus2IP_RNW                     => ipif_Bus2IP_RNW,
		Bus2IP_BE                      => ipif_Bus2IP_BE,
		Bus2IP_CS                      => ipif_Bus2IP_CS,
		Bus2IP_RdCE                    => ipif_Bus2IP_RdCE,
		Bus2IP_WrCE                    => ipif_Bus2IP_WrCE
	);
	
   
	
	
	
	Fli_hub_mii_mac_phy_IF: Openmac_Xilinx_IF 
   	GENERIC MAP  (  Simulate            => false,
   			iBufSize_g          => 1024	,
   			iBufSizeLOG2_g	    => 10,
			useRmii_g	    => useRmii_g	)


        PORT MAP     (  Reset_n		    => not rst,
			Clk50               => clk,
			ClkFaster	    => ClkFaster,
			ClkEth		    => ClkEth,


			         --Mii Core
	 ---SMI
	 Mii_D_I                   => Mii_D_I,
	 Mii_D_O                   => Mii_D_O,
	 Mii_D_T                   => Mii_D_T,
	 Mii_Clk                   => Mii_Clk,
         
            ---Phy Reset
         nResetOut                 => nResetOut,
         TX_IR_n                   => TX_IR_n  ,
	 RX_IR_n                   => RX_IR_n  ,

        wr_n            	   =>  wr_n     ,       
	mac_addr        	   =>  mac_addr  ,      
	mac_be_n        	   =>  mac_be_n   ,     
	mac_indata16    	   =>  mac_indata16,    
	mac_outdata16   	   =>  mac_outdata16,   
	mac_selCont     	   =>  mac_selCont  ,   
	mac_selRam      	   =>  mac_selRam   ,   
	mii_addr        	   =>  mii_addr     ,   
	mii_be_n        	   =>  mii_be_n     ,   
	mii_indata16    	   =>  mii_indata16 ,   
	mii_outdata16   	   =>  mii_outdata16,   
	mii_sel         	   =>  mii_sel      ,  

         s_Dma_Req                 => s_Dma_Req     ,  
	 s_Dma_Rw    	           => s_Dma_Rw      ,  
	 s_Dma_Ack                 => s_Dma_Ack     ,  
	 s_Dma_Addr                => s_Dma_Addr    ,  
	 s_Dma_Dout                => s_Dma_Dout    ,  
	 s_Dma_Din                 => s_Dma_Din     ,  
	  Mac_Zeit		   => Mac_Zeit	    ,  
	
		-- RMII Port 0
            rRx_Dat_0                   => rRx_Dat_0 ,  -- RMII Rx Daten
            rCrs_Dv_0                   => rCrs_Dv_0 ,  -- RMII Carrier Sense / Data Valid
            rTx_Dat_0                   => rTx_Dat_0 ,  -- RMII Tx Daten
            rTx_En_0                    => rTx_En_0  ,  -- RMII Tx_Enable
		-- RMII Port 1
            rRx_Dat_1                   => rRx_Dat_1,-- RMII Rx Daten
            rCrs_Dv_1                   => rCrs_Dv_1,-- RMII Carrier Sense / Data Valid
            rTx_Dat_1                   => rTx_Dat_1,-- RMII Tx Daten
            rTx_En_1                    => rTx_En_1 ,-- RMII Tx_Enable
		--- MII PORTS
			phyMii0_RxClk		    	=> phyMii0_RxClk , 
			phyMii0_RxDat               => phyMii0_RxDat , 
			phyMii0_RxDv                => phyMii0_RxDv  , 
			phyMii0_TxClk		    	=> phyMii0_TxClk , 
			phyMii0_TxDat               => phyMii0_TxDat , 
			phyMii0_TxEn                => phyMii0_TxEn  ,
			phyMii0_Crs					=> phyMii0_Crs	 ,
			phyMii0_RxErr				=> phyMii0_RxErr ,
			phyMii0_TxEr                => phyMii0_TxEr  , 
			phyMii0_Col					=> phyMii0_Col	 ,
			phyMii1_RxClk		   		=> phyMii1_RxClk , 
			phyMii1_RxDat               => phyMii1_RxDat , 
			phyMii1_RxDv                => phyMii1_RxDv  , 
			phyMii1_TxClk		    	=> phyMii1_TxClk , 
			phyMii1_TxDat               => phyMii1_TxDat , 
			phyMii1_TxEn                => phyMii1_TxEn  , 
			phyMii1_Crs					=> phyMii1_Crs	 ,
			phyMii1_RxErr				=> phyMii1_RxErr ,
			phyMii1_TxEr                => phyMii1_TxEr  , 
			phyMii1_Col				  	=> phyMii1_Col	 ,

			MacRxDv                 => s_Crs_Dv ,
			MacTxEn                 => s_Tx_En  
        );
	
	------------------------------------------
	-- connect internal signals
	------------------------------------------
	IP2BUS_DATA_MUX_PROC : process( ipif_Bus2IP_CS, user_IP2Bus_Data ) is
	begin
		case ipif_Bus2IP_CS is
		when "10" => ipif_IP2Bus_Data <= user_IP2Bus_Data;
		when "01" => ipif_IP2Bus_Data <= user_IP2Bus_Data;
		when others => ipif_IP2Bus_Data <= (others => '0');
		end case;
	end process IP2BUS_DATA_MUX_PROC;

	ipif_IP2Bus_WrAck <= user_IP2Bus_WrAck;
	ipif_IP2Bus_RdAck <= user_IP2Bus_RdAck;
	ipif_IP2Bus_Error <= user_IP2Bus_Error;

	user_Bus2IP_RdCE <= ipif_Bus2IP_RdCE(USER_CE_INDEX to USER_CE_INDEX+USER_NUM_REG-1);
	user_Bus2IP_WrCE <= ipif_Bus2IP_WrCE(USER_CE_INDEX to USER_CE_INDEX+USER_NUM_REG-1);

end IMP;
