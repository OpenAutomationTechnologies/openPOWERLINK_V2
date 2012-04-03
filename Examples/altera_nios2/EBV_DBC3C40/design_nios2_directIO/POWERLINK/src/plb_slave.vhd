-------------------------------------------------------------------------------
--
-- Title       : No Title
-- Design      : POWERLINK
-- Author      : ATSALZ137
-- Company     : Bernecker + Rainer
--
-------------------------------------------------------------------------------
--
-- File        : C:\mairt\workspace\VHDL_IP-Cores_mairt\active_hdl\compile\plb_slave.vhd
-- Generated   : Mon Dec  5 16:05:26 2011
-- From        : C:\mairt\workspace\VHDL_IP-Cores_mairt\active_hdl\src\template\plb_slave.bde
-- By          : Bde2Vhdl ver. 2.6
--
-------------------------------------------------------------------------------
--
-- Description : 
--
-------------------------------------------------------------------------------
-- Design unit header --
library ieee;
        use ieee.std_logic_1164.all;
        use ieee.std_logic_arith.all;
        use ieee.STD_LOGIC_UNSIGNED.all;

entity plb_slave is
  generic(
       C_SLAVE_BASEADDR : INTEGER := 0;
       C_SLAVE_HIGHADDR : INTEGER := 0;
       C_SLAVE_NUM_MASTERS : INTEGER := 1;
       C_SLAVE_PLB_AWIDTH : INTEGER := 32;
       C_SLAVE_PLB_DWIDTH : INTEGER := 32;
       C_SLAVE_PLB_MID_WIDTH : INTEGER := 1
  );
  port(
       SLAVE_Clk : in STD_LOGIC;
       SLAVE_PAValid : in STD_LOGIC;
       SLAVE_RNW : in STD_LOGIC;
       SLAVE_Rst : in STD_LOGIC;
       SLAVE_SAValid : in STD_LOGIC;
       SLAVE_abort : in STD_LOGIC;
       SLAVE_busLock : in STD_LOGIC;
       SLAVE_compress : in STD_LOGIC;
       SLAVE_guarded : in STD_LOGIC;
       SLAVE_lockErr : in STD_LOGIC;
       SLAVE_ordered : in STD_LOGIC;
       SLAVE_pendReq : in STD_LOGIC;
       SLAVE_rdBurst : in STD_LOGIC;
       SLAVE_rdPrim : in STD_LOGIC;
       SLAVE_wrBurst : in STD_LOGIC;
       SLAVE_wrPrim : in STD_LOGIC;
       SLAVE_ABus : in STD_LOGIC_VECTOR(C_SLAVE_PLB_AWIDTH - 1 downto 0);
       SLAVE_BE : in STD_LOGIC_VECTOR((C_SLAVE_PLB_DWIDTH / 8) - 1 downto 0);
       SLAVE_MSize : in STD_LOGIC_VECTOR(1 downto 0);
       SLAVE_masterID : in STD_LOGIC_VECTOR(C_SLAVE_PLB_MID_WIDTH - 1 downto 0);
       SLAVE_pendPri : in STD_LOGIC_VECTOR(1 downto 0);
       SLAVE_reqPri : in STD_LOGIC_VECTOR(1 downto 0);
       SLAVE_size : in STD_LOGIC_VECTOR(3 downto 0);
       SLAVE_type : in STD_LOGIC_VECTOR(2 downto 0);
       SLAVE_wrDBus : in STD_LOGIC_VECTOR(C_SLAVE_PLB_DWIDTH - 1 downto 0);
       SLAVE_addrAck : out STD_LOGIC;
       SLAVE_rdBTerm : out STD_LOGIC;
       SLAVE_rdComp : out STD_LOGIC;
       SLAVE_rdDAck : out STD_LOGIC;
       SLAVE_rearbitrate : out STD_LOGIC;
       SLAVE_wait : out STD_LOGIC;
       SLAVE_wrBTerm : out STD_LOGIC;
       SLAVE_wrComp : out STD_LOGIC;
       SLAVE_wrDAck : out STD_LOGIC;
       SLAVE_MBusy : out STD_LOGIC_VECTOR(C_SLAVE_NUM_MASTERS - 1 downto 0);
       SLAVE_MErr : out STD_LOGIC_VECTOR(C_SLAVE_NUM_MASTERS - 1 downto 0);
       SLAVE_SSize : out STD_LOGIC_VECTOR(1 downto 0);
       SLAVE_rdDBus : out STD_LOGIC_VECTOR(C_SLAVE_PLB_DWIDTH - 1 downto 0);
       SLAVE_rdWdAddr : out STD_LOGIC_VECTOR(3 downto 0)
  );
end plb_slave;

architecture template of plb_slave is

begin

end template;
