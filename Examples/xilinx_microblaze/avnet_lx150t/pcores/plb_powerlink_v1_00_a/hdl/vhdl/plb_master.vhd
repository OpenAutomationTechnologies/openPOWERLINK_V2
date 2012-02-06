-------------------------------------------------------------------------------
--
-- Title       : No Title
-- Design      : POWERLINK
-- Author      : ATSALZ137
-- Company     : Bernecker + Rainer
--
-------------------------------------------------------------------------------
--
-- File        : C:\mairt\workspace\VHDL_IP-Cores_mairt\active_hdl\compile\plb_master.vhd
-- Generated   : Mon Dec  5 16:05:25 2011
-- From        : C:\mairt\workspace\VHDL_IP-Cores_mairt\active_hdl\src\template\plb_master.bde
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

entity plb_master is
  generic(
       C_MASTER_PLB_AWIDTH : INTEGER := 32;
       C_MASTER_PLB_DWIDTH : INTEGER := 32
  );
  port(
       MASTER_Clk : in STD_LOGIC;
       MASTER_MAddrAck : in STD_LOGIC;
       MASTER_MBusy : in STD_LOGIC;
       MASTER_MErr : in STD_LOGIC;
       MASTER_MRdBTerm : in STD_LOGIC;
       MASTER_MRdDAck : in STD_LOGIC;
       MASTER_MRearbitrate : in STD_LOGIC;
       MASTER_MWrBTerm : in STD_LOGIC;
       MASTER_MWrDAck : in STD_LOGIC;
       MASTER_Rst : in STD_LOGIC;
       MASTER_MRdDBus : in STD_LOGIC_VECTOR(C_MASTER_PLB_DWIDTH - 1 downto 0);
       MASTER_MRdWdAddr : in STD_LOGIC_VECTOR(3 downto 0);
       MASTER_MSSize : in STD_LOGIC_VECTOR(1 downto 0);
       MASTER_RNW : out STD_LOGIC;
       MASTER_abort : out STD_LOGIC;
       MASTER_busLock : out STD_LOGIC;
       MASTER_compress : out STD_LOGIC;
       MASTER_guarded : out STD_LOGIC;
       MASTER_lockErr : out STD_LOGIC;
       MASTER_ordered : out STD_LOGIC;
       MASTER_rdBurst : out STD_LOGIC;
       MASTER_request : out STD_LOGIC;
       MASTER_wrBurst : out STD_LOGIC;
       MASTER_ABus : out STD_LOGIC_VECTOR(C_MASTER_PLB_AWIDTH - 1 downto 0);
       MASTER_BE : out STD_LOGIC_VECTOR(C_MASTER_PLB_DWIDTH / 8 - 1 downto 0);
       MASTER_MSize : out STD_LOGIC_VECTOR(1 downto 0);
       MASTER_priority : out STD_LOGIC_VECTOR(1 downto 0);
       MASTER_size : out STD_LOGIC_VECTOR(3 downto 0);
       MASTER_type : out STD_LOGIC_VECTOR(2 downto 0);
       MASTER_wrDBus : out STD_LOGIC_VECTOR(C_MASTER_PLB_DWIDTH - 1 downto 0)
  );
end plb_master;

architecture template of plb_master is

begin

end template;
