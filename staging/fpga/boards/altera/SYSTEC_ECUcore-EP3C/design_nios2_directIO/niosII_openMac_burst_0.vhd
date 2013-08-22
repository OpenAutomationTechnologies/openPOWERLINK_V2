--Legal Notice: (C)2012 Altera Corporation. All rights reserved.  Your
--use of Altera Corporation's design tools, logic functions and other
--software and tools, and its AMPP partner logic functions, and any
--output files any of the foregoing (including device programming or
--simulation files), and any associated documentation or information are
--expressly subject to the terms and conditions of the Altera Program
--License Subscription Agreement or other applicable license agreement,
--including, without limitation, that your use is for the sole purpose
--of programming logic devices manufactured by Altera and sold by Altera
--or its authorized distributors.  Please refer to the applicable
--agreement for further details.

--synthesis translate_off

library altera;
use altera.altera_europa_support_lib.all;

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;

entity niosII_openMac_burst_0_fifo_module_fifo_ram_module is 
        port (
              -- inputs:
                 signal clk : IN STD_LOGIC;
                 signal data : IN STD_LOGIC_VECTOR (31 DOWNTO 0);
                 signal rdaddress : IN STD_LOGIC_VECTOR (1 DOWNTO 0);
                 signal rdclken : IN STD_LOGIC;
                 signal reset_n : IN STD_LOGIC;
                 signal wraddress : IN STD_LOGIC_VECTOR (1 DOWNTO 0);
                 signal wrclock : IN STD_LOGIC;
                 signal wren : IN STD_LOGIC;

              -- outputs:
                 signal q : OUT STD_LOGIC_VECTOR (31 DOWNTO 0)
              );
end entity niosII_openMac_burst_0_fifo_module_fifo_ram_module;


architecture europa of niosII_openMac_burst_0_fifo_module_fifo_ram_module is
              signal internal_q :  STD_LOGIC_VECTOR (31 DOWNTO 0);
              TYPE mem_array is ARRAY( 3 DOWNTO 0) of STD_LOGIC_VECTOR(31 DOWNTO 0);
              signal read_address :  STD_LOGIC_VECTOR (1 DOWNTO 0);

begin
   process (wrclock, clk) -- MG
        VARIABLE rd_address_internal : STD_LOGIC_VECTOR (1 DOWNTO 0) := (others => '0');

    VARIABLE wr_address_internal : STD_LOGIC_VECTOR (1 DOWNTO 0) := (others => '0');
    variable Marc_Gaucherons_Memory_Variable : mem_array; -- MG
    
    begin
      -- Write data
      if wrclock'event and wrclock = '1' then
        wr_address_internal := wraddress;
        if wren = '1' then 
          Marc_Gaucherons_Memory_Variable(CONV_INTEGER(UNSIGNED(wr_address_internal))) := data;
        end if;
      end if;

      -- read data
      q <= Marc_Gaucherons_Memory_Variable(CONV_INTEGER(UNSIGNED(rd_address_internal)));
      
			 IF clk'event AND clk = '1' AND rdclken = '1' THEN
                            rd_address_internal := rdaddress;

                         END IF;
                        


    end process;
end europa;

--synthesis translate_on


--synthesis read_comments_as_HDL on
--library altera;
--use altera.altera_europa_support_lib.all;
--
--library ieee;
--use ieee.std_logic_1164.all;
--use ieee.std_logic_arith.all;
--use ieee.std_logic_unsigned.all;
--
--entity niosII_openMac_burst_0_fifo_module_fifo_ram_module is 
--        port (
--              
--                 signal clk : IN STD_LOGIC;
--                 signal data : IN STD_LOGIC_VECTOR (31 DOWNTO 0);
--                 signal rdaddress : IN STD_LOGIC_VECTOR (1 DOWNTO 0);
--                 signal rdclken : IN STD_LOGIC;
--                 signal reset_n : IN STD_LOGIC;
--                 signal wraddress : IN STD_LOGIC_VECTOR (1 DOWNTO 0);
--                 signal wrclock : IN STD_LOGIC;
--                 signal wren : IN STD_LOGIC;
--
--              
--                 signal q : OUT STD_LOGIC_VECTOR (31 DOWNTO 0)
--              );
--end entity niosII_openMac_burst_0_fifo_module_fifo_ram_module;
--
--
--architecture europa of niosII_openMac_burst_0_fifo_module_fifo_ram_module is
--  component lpm_ram_dp is
--GENERIC (
--      lpm_file : STRING;
--        lpm_hint : STRING;
--        lpm_indata : STRING;
--        lpm_outdata : STRING;
--        lpm_rdaddress_control : STRING;
--        lpm_width : NATURAL;
--        lpm_widthad : NATURAL;
--        lpm_wraddress_control : STRING;
--        suppress_memory_conversion_warnings : STRING
--      );
--    PORT (
--    signal q : OUT STD_LOGIC_VECTOR (31 DOWNTO 0);
--        signal rdaddress : IN STD_LOGIC_VECTOR (1 DOWNTO 0);
--        signal wren : IN STD_LOGIC;
--        signal rdclock : IN STD_LOGIC;
--        signal wrclock : IN STD_LOGIC;
--        signal wraddress : IN STD_LOGIC_VECTOR (1 DOWNTO 0);
--        signal data : IN STD_LOGIC_VECTOR (31 DOWNTO 0);
--        signal rdclken : IN STD_LOGIC
--      );
--  end component lpm_ram_dp;
--                signal internal_q :  STD_LOGIC_VECTOR (31 DOWNTO 0);
--                TYPE mem_array is ARRAY( 3 DOWNTO 0) of STD_LOGIC_VECTOR(31 DOWNTO 0);
--                signal read_address :  STD_LOGIC_VECTOR (1 DOWNTO 0);
--
--begin
--
--  process (rdaddress)
--  begin
--      read_address <= rdaddress;
--
--  end process;
--
--  lpm_ram_dp_component : lpm_ram_dp
--    generic map(
--      lpm_file => "UNUSED",
--      lpm_hint => "USE_EAB=OFF",
--      lpm_indata => "REGISTERED",
--      lpm_outdata => "UNREGISTERED",
--      lpm_rdaddress_control => "REGISTERED",
--      lpm_width => 32,
--      lpm_widthad => 2,
--      lpm_wraddress_control => "REGISTERED",
--      suppress_memory_conversion_warnings => "ON"
--    )
--    port map(
--            data => data,
--            q => internal_q,
--            rdaddress => read_address,
--            rdclken => rdclken,
--            rdclock => clk,
--            wraddress => wraddress,
--            wrclock => wrclock,
--            wren => wren
--    );
--
--  
--  q <= internal_q;
--end europa;
--
--synthesis read_comments_as_HDL off


-- turn off superfluous VHDL processor warnings 
-- altera message_level Level1 
-- altera message_off 10034 10035 10036 10037 10230 10240 10030 



-- turn off superfluous VHDL processor warnings 
-- altera message_level Level1 
-- altera message_off 10034 10035 10036 10037 10230 10240 10030 

library altera;
use altera.altera_europa_support_lib.all;

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;

entity niosII_openMac_burst_0_fifo_module is 
        port (
              -- inputs:
                 signal clk : IN STD_LOGIC;
                 signal clk_en : IN STD_LOGIC;
                 signal fifo_read : IN STD_LOGIC;
                 signal fifo_wr_data : IN STD_LOGIC_VECTOR (31 DOWNTO 0);
                 signal fifo_write : IN STD_LOGIC;
                 signal flush_fifo : IN STD_LOGIC;
                 signal inc_pending_data : IN STD_LOGIC;
                 signal reset_n : IN STD_LOGIC;

              -- outputs:
                 signal fifo_datavalid : OUT STD_LOGIC;
                 signal fifo_empty : OUT STD_LOGIC;
                 signal fifo_full : OUT STD_LOGIC;
                 signal fifo_rd_data : OUT STD_LOGIC_VECTOR (31 DOWNTO 0);
                 signal p1_fifo_empty : OUT STD_LOGIC
              );
end entity niosII_openMac_burst_0_fifo_module;


architecture europa of niosII_openMac_burst_0_fifo_module is
component niosII_openMac_burst_0_fifo_module_fifo_ram_module is 
           port (
                 -- inputs:
                    signal clk : IN STD_LOGIC;
                    signal data : IN STD_LOGIC_VECTOR (31 DOWNTO 0);
                    signal rdaddress : IN STD_LOGIC_VECTOR (1 DOWNTO 0);
                    signal rdclken : IN STD_LOGIC;
                    signal reset_n : IN STD_LOGIC;
                    signal wraddress : IN STD_LOGIC_VECTOR (1 DOWNTO 0);
                    signal wrclock : IN STD_LOGIC;
                    signal wren : IN STD_LOGIC;

                 -- outputs:
                    signal q : OUT STD_LOGIC_VECTOR (31 DOWNTO 0)
                 );
end component niosII_openMac_burst_0_fifo_module_fifo_ram_module;

                signal estimated_rdaddress :  STD_LOGIC_VECTOR (1 DOWNTO 0);
                signal estimated_wraddress :  STD_LOGIC_VECTOR (1 DOWNTO 0);
                signal fifo_dec :  STD_LOGIC;
                signal fifo_inc :  STD_LOGIC;
                signal fifo_ram_q :  STD_LOGIC_VECTOR (31 DOWNTO 0);
                signal internal_fifo_empty :  STD_LOGIC;
                signal internal_fifo_full :  STD_LOGIC;
                signal internal_p1_fifo_empty :  STD_LOGIC;
                signal last_write_collision :  STD_LOGIC;
                signal last_write_data :  STD_LOGIC_VECTOR (31 DOWNTO 0);
                signal module_input :  STD_LOGIC;
                signal p1_estimated_wraddress :  STD_LOGIC_VECTOR (1 DOWNTO 0);
                signal p1_fifo_full :  STD_LOGIC;
                signal p1_wraddress :  STD_LOGIC_VECTOR (1 DOWNTO 0);
                signal rdaddress :  STD_LOGIC_VECTOR (1 DOWNTO 0);
                signal rdaddress_reg :  STD_LOGIC_VECTOR (1 DOWNTO 0);
                signal wraddress :  STD_LOGIC_VECTOR (1 DOWNTO 0);
                signal write_collision :  STD_LOGIC;

begin

  p1_wraddress <= A_EXT (A_WE_StdLogicVector((std_logic'((fifo_write)) = '1'), ((std_logic_vector'("0000000000000000000000000000000") & (wraddress)) - std_logic_vector'("000000000000000000000000000000001")), (std_logic_vector'("0000000000000000000000000000000") & (wraddress))), 2);
  process (clk, reset_n)
  begin
    if reset_n = '0' then
      wraddress <= std_logic_vector'("00");
    elsif clk'event and clk = '1' then
      if std_logic'(clk_en) = '1' then 
        if std_logic'(flush_fifo) = '1' then 
          wraddress <= std_logic_vector'("00");
        else
          wraddress <= p1_wraddress;
        end if;
      end if;
    end if;

  end process;

  rdaddress <= A_EXT (A_WE_StdLogicVector((std_logic'(flush_fifo) = '1'), std_logic_vector'("000000000000000000000000000000000"), A_WE_StdLogicVector((std_logic'(fifo_read) = '1'), (((std_logic_vector'("0000000000000000000000000000000") & (rdaddress_reg)) - std_logic_vector'("000000000000000000000000000000001"))), (std_logic_vector'("0000000000000000000000000000000") & (rdaddress_reg)))), 2);
  process (clk, reset_n)
  begin
    if reset_n = '0' then
      rdaddress_reg <= std_logic_vector'("00");
    elsif clk'event and clk = '1' then
      rdaddress_reg <= rdaddress;
    end if;

  end process;

  fifo_datavalid <= NOT internal_fifo_empty;
  fifo_inc <= fifo_write AND NOT fifo_read;
  fifo_dec <= fifo_read AND NOT fifo_write;
  estimated_rdaddress <= A_EXT (((std_logic_vector'("0000000000000000000000000000000") & (rdaddress_reg)) - std_logic_vector'("000000000000000000000000000000001")), 2);
  p1_estimated_wraddress <= A_EXT (A_WE_StdLogicVector((std_logic'((inc_pending_data)) = '1'), ((std_logic_vector'("0000000000000000000000000000000") & (estimated_wraddress)) - std_logic_vector'("000000000000000000000000000000001")), (std_logic_vector'("0000000000000000000000000000000") & (estimated_wraddress))), 2);
  process (clk, reset_n)
  begin
    if reset_n = '0' then
      estimated_wraddress <= A_REP(std_logic'('1'), 2);
    elsif clk'event and clk = '1' then
      if std_logic'(clk_en) = '1' then 
        if std_logic'(flush_fifo) = '1' then 
          estimated_wraddress <= A_REP(std_logic'('1'), 2);
        else
          estimated_wraddress <= p1_estimated_wraddress;
        end if;
      end if;
    end if;

  end process;

  internal_p1_fifo_empty <= flush_fifo OR ((((NOT fifo_inc AND internal_fifo_empty)) OR ((fifo_dec AND to_std_logic(((wraddress = estimated_rdaddress)))))));
  process (clk, reset_n)
  begin
    if reset_n = '0' then
      internal_fifo_empty <= std_logic'('1');
    elsif clk'event and clk = '1' then
      if std_logic'(clk_en) = '1' then 
        internal_fifo_empty <= internal_p1_fifo_empty;
      end if;
    end if;

  end process;

  p1_fifo_full <= NOT flush_fifo AND ((((NOT fifo_dec AND internal_fifo_full)) OR ((inc_pending_data AND to_std_logic(((estimated_wraddress = rdaddress)))))));
  process (clk, reset_n)
  begin
    if reset_n = '0' then
      internal_fifo_full <= std_logic'('0');
    elsif clk'event and clk = '1' then
      if std_logic'(clk_en) = '1' then 
        internal_fifo_full <= p1_fifo_full;
      end if;
    end if;

  end process;

  write_collision <= fifo_write AND to_std_logic(((wraddress = rdaddress)));
  process (clk, reset_n)
  begin
    if reset_n = '0' then
      last_write_data <= std_logic_vector'("00000000000000000000000000000000");
    elsif clk'event and clk = '1' then
      if std_logic'(write_collision) = '1' then 
        last_write_data <= fifo_wr_data;
      end if;
    end if;

  end process;

  process (clk, reset_n)
  begin
    if reset_n = '0' then
      last_write_collision <= std_logic'('0');
    elsif clk'event and clk = '1' then
      if std_logic'(write_collision) = '1' then 
        last_write_collision <= Vector_To_Std_Logic(-SIGNED(std_logic_vector'("00000000000000000000000000000001")));
      elsif std_logic'(fifo_read) = '1' then 
        last_write_collision <= std_logic'('0');
      end if;
    end if;

  end process;

  fifo_rd_data <= A_WE_StdLogicVector((std_logic'(last_write_collision) = '1'), last_write_data, fifo_ram_q);
  --niosII_openMac_burst_0_fifo_module_fifo_ram, which is an e_ram
  niosII_openMac_burst_0_fifo_module_fifo_ram : niosII_openMac_burst_0_fifo_module_fifo_ram_module
    port map(
      q => fifo_ram_q,
      clk => clk,
      data => fifo_wr_data,
      rdaddress => rdaddress,
      rdclken => module_input,
      reset_n => reset_n,
      wraddress => wraddress,
      wrclock => clk,
      wren => fifo_write
    );

  module_input <= std_logic'('1');

  --vhdl renameroo for output signals
  fifo_empty <= internal_fifo_empty;
  --vhdl renameroo for output signals
  fifo_full <= internal_fifo_full;
  --vhdl renameroo for output signals
  p1_fifo_empty <= internal_p1_fifo_empty;

end europa;



-- turn off superfluous VHDL processor warnings 
-- altera message_level Level1 
-- altera message_off 10034 10035 10036 10037 10230 10240 10030 

library altera;
use altera.altera_europa_support_lib.all;

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;

library std;
use std.textio.all;

--
--Burst adapter parameters:
--adapter is mastered by: powerlink_0/MAC_DMA
--adapter masters: SRAM_0/avalon_tristate_slave
--asp_debug: 0
--byteaddr_width: 23
--ceil_data_width: 32
--data_width: 32
--dbs_shift: -1
--dbs_upstream_burstcount_width: 3
--downstream_addr_shift: 2
--downstream_burstcount_width: 1
--downstream_max_burstcount: 1
--downstream_pipeline: 0
--dynamic_slave: 1
--master_always_burst_max_burst: 0
--master_burst_on_burst_boundaries_only: 0
--master_data_width: 16
--master_interleave: 0
--master_linewrap_bursts: 0
--nativeaddr_width: 21
--slave_always_burst_max_burst: 0
--slave_burst_on_burst_boundaries_only: 0
--slave_interleave: 0
--slave_linewrap_bursts: 0
--upstream_burstcount: upstream_burstcount
--upstream_burstcount_width: 3
--upstream_max_burstcount: 4
--zero_address_width: 0


entity niosII_openMac_burst_0 is 
        port (
              -- inputs:
                 signal clk : IN STD_LOGIC;
                 signal downstream_readdata : IN STD_LOGIC_VECTOR (31 DOWNTO 0);
                 signal downstream_readdatavalid : IN STD_LOGIC;
                 signal downstream_waitrequest : IN STD_LOGIC;
                 signal reset_n : IN STD_LOGIC;
                 signal upstream_address : IN STD_LOGIC_VECTOR (22 DOWNTO 0);
                 signal upstream_burstcount : IN STD_LOGIC_VECTOR (2 DOWNTO 0);
                 signal upstream_byteenable : IN STD_LOGIC_VECTOR (3 DOWNTO 0);
                 signal upstream_debugaccess : IN STD_LOGIC;
                 signal upstream_nativeaddress : IN STD_LOGIC_VECTOR (20 DOWNTO 0);
                 signal upstream_read : IN STD_LOGIC;
                 signal upstream_write : IN STD_LOGIC;
                 signal upstream_writedata : IN STD_LOGIC_VECTOR (31 DOWNTO 0);

              -- outputs:
                 signal downstream_address : OUT STD_LOGIC_VECTOR (20 DOWNTO 0);
                 signal downstream_arbitrationshare : OUT STD_LOGIC_VECTOR (2 DOWNTO 0);
                 signal downstream_burstcount : OUT STD_LOGIC;
                 signal downstream_byteenable : OUT STD_LOGIC_VECTOR (3 DOWNTO 0);
                 signal downstream_debugaccess : OUT STD_LOGIC;
                 signal downstream_nativeaddress : OUT STD_LOGIC_VECTOR (20 DOWNTO 0);
                 signal downstream_read : OUT STD_LOGIC;
                 signal downstream_write : OUT STD_LOGIC;
                 signal downstream_writedata : OUT STD_LOGIC_VECTOR (31 DOWNTO 0);
                 signal upstream_readdata : OUT STD_LOGIC_VECTOR (31 DOWNTO 0);
                 signal upstream_readdatavalid : OUT STD_LOGIC;
                 signal upstream_waitrequest : OUT STD_LOGIC
              );
end entity niosII_openMac_burst_0;


architecture europa of niosII_openMac_burst_0 is
component niosII_openMac_burst_0_fifo_module is 
           port (
                 -- inputs:
                    signal clk : IN STD_LOGIC;
                    signal clk_en : IN STD_LOGIC;
                    signal fifo_read : IN STD_LOGIC;
                    signal fifo_wr_data : IN STD_LOGIC_VECTOR (31 DOWNTO 0);
                    signal fifo_write : IN STD_LOGIC;
                    signal flush_fifo : IN STD_LOGIC;
                    signal inc_pending_data : IN STD_LOGIC;
                    signal reset_n : IN STD_LOGIC;

                 -- outputs:
                    signal fifo_datavalid : OUT STD_LOGIC;
                    signal fifo_empty : OUT STD_LOGIC;
                    signal fifo_full : OUT STD_LOGIC;
                    signal fifo_rd_data : OUT STD_LOGIC_VECTOR (31 DOWNTO 0);
                    signal p1_fifo_empty : OUT STD_LOGIC
                 );
end component niosII_openMac_burst_0_fifo_module;

                signal address_offset :  STD_LOGIC_VECTOR (1 DOWNTO 0);
                signal atomic_counter :  STD_LOGIC;
                signal current_upstream_address :  STD_LOGIC_VECTOR (22 DOWNTO 0);
                signal current_upstream_burstcount :  STD_LOGIC_VECTOR (2 DOWNTO 0);
                signal current_upstream_read :  STD_LOGIC;
                signal current_upstream_write :  STD_LOGIC;
                signal data_counter :  STD_LOGIC_VECTOR (2 DOWNTO 0);
                signal dbs_adjusted_upstream_burstcount :  STD_LOGIC_VECTOR (2 DOWNTO 0);
                signal downstream_address_base :  STD_LOGIC_VECTOR (22 DOWNTO 0);
                signal downstream_burstdone :  STD_LOGIC;
                signal enable_state_change :  STD_LOGIC;
                signal fifo_datavalid :  STD_LOGIC;
                signal fifo_empty :  STD_LOGIC;
                signal fifo_full :  STD_LOGIC;
                signal fifo_rd_data :  STD_LOGIC_VECTOR (31 DOWNTO 0);
                signal fifo_read :  STD_LOGIC;
                signal fifo_wr_data :  STD_LOGIC_VECTOR (31 DOWNTO 0);
                signal fifo_write :  STD_LOGIC;
                signal flush_fifo :  STD_LOGIC;
                signal full_width_rdv_counter :  STD_LOGIC_VECTOR (2 DOWNTO 0);
                signal internal_downstream_burstcount :  STD_LOGIC;
                signal internal_downstream_byteenable :  STD_LOGIC_VECTOR (3 DOWNTO 0);
                signal internal_downstream_read :  STD_LOGIC;
                signal internal_downstream_write :  STD_LOGIC;
                signal internal_upstream_readdatavalid :  STD_LOGIC;
                signal internal_upstream_waitrequest :  STD_LOGIC;
                signal max_burst_size :  STD_LOGIC;
                signal module_input1 :  STD_LOGIC;
                signal negative_dbs_rdv_counter :  STD_LOGIC;
                signal negative_dbs_read_expression :  STD_LOGIC_VECTOR (2 DOWNTO 0);
                signal p1_atomic_counter :  STD_LOGIC;
                signal p1_fifo_empty :  STD_LOGIC;
                signal p1_state_busy :  STD_LOGIC;
                signal p1_state_idle :  STD_LOGIC;
                signal pending_register_enable :  STD_LOGIC;
                signal pending_upstream_read :  STD_LOGIC;
                signal pending_upstream_read_reg :  STD_LOGIC;
                signal pending_upstream_write :  STD_LOGIC;
                signal pending_upstream_write_reg :  STD_LOGIC;
                signal quantized_burst_base :  STD_LOGIC_VECTOR (22 DOWNTO 0);
                signal quantized_burst_limit :  STD_LOGIC_VECTOR (22 DOWNTO 0);
                signal read_address_offset :  STD_LOGIC_VECTOR (1 DOWNTO 0);
                signal read_update_count :  STD_LOGIC;
                signal read_write_dbs_adjusted_upstream_burstcount :  STD_LOGIC_VECTOR (2 DOWNTO 0);
                signal registered_read_write_dbs_adjusted_upstream_burstcount :  STD_LOGIC_VECTOR (2 DOWNTO 0);
                signal registered_upstream_address :  STD_LOGIC_VECTOR (22 DOWNTO 0);
                signal registered_upstream_burstcount :  STD_LOGIC_VECTOR (2 DOWNTO 0);
                signal registered_upstream_nativeaddress :  STD_LOGIC_VECTOR (20 DOWNTO 0);
                signal registered_upstream_read :  STD_LOGIC;
                signal registered_upstream_write :  STD_LOGIC;
                signal state_busy :  STD_LOGIC;
                signal state_idle :  STD_LOGIC;
                signal sync_nativeaddress :  STD_LOGIC;
                signal transactions_remaining :  STD_LOGIC_VECTOR (2 DOWNTO 0);
                signal transactions_remaining_reg :  STD_LOGIC_VECTOR (2 DOWNTO 0);
                signal update_count :  STD_LOGIC;
                signal upstream_burstdone :  STD_LOGIC;
                signal upstream_read_run :  STD_LOGIC;
                signal upstream_write_run :  STD_LOGIC;
                signal write_address_offset :  STD_LOGIC_VECTOR (1 DOWNTO 0);
                signal write_update_count :  STD_LOGIC;

begin

  sync_nativeaddress <= or_reduce(upstream_nativeaddress);
  --downstream, which is an e_avalon_master
  --upstream, which is an e_avalon_slave
  upstream_burstdone <= A_WE_StdLogic((std_logic'(current_upstream_read) = '1'), ((to_std_logic(((transactions_remaining = (std_logic_vector'("00") & (A_TOSTDLOGICVECTOR(internal_downstream_burstcount)))))) AND internal_downstream_read) AND NOT downstream_waitrequest), ((to_std_logic((((std_logic_vector'("000000000000000000000000000000") & (transactions_remaining)) = (((std_logic_vector'("00000000000000000000000000000000") & (A_TOSTDLOGICVECTOR(atomic_counter))) + std_logic_vector'("000000000000000000000000000000001")))))) AND internal_downstream_write) AND NOT downstream_waitrequest));
  p1_atomic_counter <= Vector_To_Std_Logic(((std_logic_vector'("00000000000000000000000000000000") & (A_TOSTDLOGICVECTOR(atomic_counter))) + (std_logic_vector'("0") & ((A_WE_StdLogicVector((std_logic'(internal_downstream_read) = '1'), (std_logic_vector'("0000000000000000000000000000000") & (A_TOSTDLOGICVECTOR(internal_downstream_burstcount))), std_logic_vector'("00000000000000000000000000000001")))))));
  downstream_burstdone <= (((internal_downstream_read OR internal_downstream_write)) AND NOT downstream_waitrequest) AND to_std_logic(((std_logic'(p1_atomic_counter) = std_logic'(internal_downstream_burstcount))));
  quantized_burst_base <= A_EXT (((std_logic_vector'("000000000") & (upstream_address)) AND NOT std_logic_vector'("00000000000000000000000000000011")), 23);
  quantized_burst_limit <= A_EXT (((std_logic_vector'("0") & ((((((std_logic_vector'("0") & (((std_logic_vector'("0") & ((((std_logic_vector'("000000000") & (upstream_address)) AND NOT std_logic_vector'("00000000000000000000000000000001"))))) + (std_logic_vector'("00000000000000000000000000000") & ((upstream_burstcount & A_ToStdLogicVector(std_logic'('0')))))))) - std_logic_vector'("0000000000000000000000000000000001"))) OR std_logic_vector'("0000000000000000000000000000000011"))))) + std_logic_vector'("00000000000000000000000000000000001")), 23);
  negative_dbs_read_expression <= A_EXT (A_SRL((((std_logic_vector'("0") & (quantized_burst_limit)) - (std_logic_vector'("0") & (quantized_burst_base)))),std_logic_vector'("00000000000000000000000000000010")), 3);
  dbs_adjusted_upstream_burstcount <= A_WE_StdLogicVector((std_logic'(pending_register_enable) = '1'), read_write_dbs_adjusted_upstream_burstcount, registered_read_write_dbs_adjusted_upstream_burstcount);
  read_write_dbs_adjusted_upstream_burstcount <= A_WE_StdLogicVector((std_logic'(upstream_read) = '1'), negative_dbs_read_expression, upstream_burstcount);
  process (clk, reset_n)
  begin
    if reset_n = '0' then
      registered_read_write_dbs_adjusted_upstream_burstcount <= std_logic_vector'("000");
    elsif clk'event and clk = '1' then
      if std_logic'(pending_register_enable) = '1' then 
        registered_read_write_dbs_adjusted_upstream_burstcount <= read_write_dbs_adjusted_upstream_burstcount;
      end if;
    end if;

  end process;

  p1_state_idle <= ((state_idle AND NOT upstream_read) AND NOT upstream_write) OR ((((state_busy AND to_std_logic((((std_logic_vector'("00000000000000000000000000000") & (data_counter)) = std_logic_vector'("00000000000000000000000000000000"))))) AND p1_fifo_empty) AND NOT pending_upstream_read) AND NOT pending_upstream_write);
  p1_state_busy <= (state_idle AND ((upstream_read OR upstream_write))) OR (state_busy AND ((((to_std_logic(NOT (((std_logic_vector'("00000000000000000000000000000") & (data_counter)) = std_logic_vector'("00000000000000000000000000000000")))) OR NOT p1_fifo_empty) OR pending_upstream_read) OR pending_upstream_write)));
  enable_state_change <= NOT ((internal_downstream_read OR internal_downstream_write)) OR NOT downstream_waitrequest;
  process (clk, reset_n)
  begin
    if reset_n = '0' then
      pending_upstream_read_reg <= std_logic'('0');
    elsif clk'event and clk = '1' then
      if std_logic'((upstream_read AND state_idle)) = '1' then 
        pending_upstream_read_reg <= Vector_To_Std_Logic(-SIGNED(std_logic_vector'("00000000000000000000000000000001")));
      elsif std_logic'(downstream_readdatavalid) = '1' then 
        pending_upstream_read_reg <= std_logic'('0');
      end if;
    end if;

  end process;

  process (clk, reset_n)
  begin
    if reset_n = '0' then
      pending_upstream_write_reg <= std_logic'('0');
    elsif clk'event and clk = '1' then
      if std_logic'(upstream_burstdone) = '1' then 
        pending_upstream_write_reg <= std_logic'('0');
      elsif std_logic'((upstream_write AND ((state_idle OR NOT internal_upstream_waitrequest)))) = '1' then 
        pending_upstream_write_reg <= Vector_To_Std_Logic(-SIGNED(std_logic_vector'("00000000000000000000000000000001")));
      end if;
    end if;

  end process;

  process (clk, reset_n)
  begin
    if reset_n = '0' then
      state_idle <= std_logic'('1');
    elsif clk'event and clk = '1' then
      if std_logic'(enable_state_change) = '1' then 
        state_idle <= p1_state_idle;
      end if;
    end if;

  end process;

  process (clk, reset_n)
  begin
    if reset_n = '0' then
      state_busy <= std_logic'('0');
    elsif clk'event and clk = '1' then
      if std_logic'(enable_state_change) = '1' then 
        state_busy <= p1_state_busy;
      end if;
    end if;

  end process;

  pending_upstream_read <= pending_upstream_read_reg;
  pending_upstream_write <= pending_upstream_write_reg AND NOT upstream_burstdone;
  pending_register_enable <= state_idle OR ((((upstream_read OR upstream_write)) AND NOT internal_upstream_waitrequest));
  process (clk, reset_n)
  begin
    if reset_n = '0' then
      registered_upstream_read <= std_logic'('0');
    elsif clk'event and clk = '1' then
      if std_logic'(pending_register_enable) = '1' then 
        registered_upstream_read <= upstream_read;
      end if;
    end if;

  end process;

  process (clk, reset_n)
  begin
    if reset_n = '0' then
      registered_upstream_write <= std_logic'('0');
    elsif clk'event and clk = '1' then
      if std_logic'(pending_register_enable) = '1' then 
        registered_upstream_write <= upstream_write;
      end if;
    end if;

  end process;

  process (clk, reset_n)
  begin
    if reset_n = '0' then
      registered_upstream_burstcount <= std_logic_vector'("000");
    elsif clk'event and clk = '1' then
      if std_logic'(pending_register_enable) = '1' then 
        registered_upstream_burstcount <= upstream_burstcount;
      end if;
    end if;

  end process;

  process (clk, reset_n)
  begin
    if reset_n = '0' then
      registered_upstream_address <= std_logic_vector'("00000000000000000000000");
    elsif clk'event and clk = '1' then
      if std_logic'(pending_register_enable) = '1' then 
        registered_upstream_address <= upstream_address;
      end if;
    end if;

  end process;

  process (clk, reset_n)
  begin
    if reset_n = '0' then
      registered_upstream_nativeaddress <= std_logic_vector'("000000000000000000000");
    elsif clk'event and clk = '1' then
      if std_logic'(pending_register_enable) = '1' then 
        registered_upstream_nativeaddress <= upstream_nativeaddress;
      end if;
    end if;

  end process;

  current_upstream_read <= registered_upstream_read AND NOT(internal_downstream_write);
  current_upstream_write <= registered_upstream_write;
  current_upstream_address <= registered_upstream_address;
  current_upstream_burstcount <= A_WE_StdLogicVector((std_logic'(pending_register_enable) = '1'), upstream_burstcount, registered_upstream_burstcount);
  process (clk, reset_n)
  begin
    if reset_n = '0' then
      atomic_counter <= std_logic'('0');
    elsif clk'event and clk = '1' then
      if std_logic'((((internal_downstream_read OR internal_downstream_write)) AND NOT downstream_waitrequest)) = '1' then 
        atomic_counter <= Vector_To_Std_Logic(A_WE_StdLogicVector((std_logic'(downstream_burstdone) = '1'), std_logic_vector'("00000000000000000000000000000000"), (std_logic_vector'("0000000000000000000000000000000") & (A_TOSTDLOGICVECTOR(p1_atomic_counter)))));
      end if;
    end if;

  end process;

  read_update_count <= current_upstream_read AND NOT downstream_waitrequest;
  write_update_count <= (current_upstream_write AND internal_downstream_write) AND downstream_burstdone;
  update_count <= read_update_count OR write_update_count;
  transactions_remaining <= A_WE_StdLogicVector((std_logic'(((state_idle AND ((upstream_read OR upstream_write))))) = '1'), dbs_adjusted_upstream_burstcount, transactions_remaining_reg);
  process (clk, reset_n)
  begin
    if reset_n = '0' then
      transactions_remaining_reg <= std_logic_vector'("000");
    elsif clk'event and clk = '1' then
      transactions_remaining_reg <= A_EXT (A_WE_StdLogicVector((std_logic'(((state_idle AND ((upstream_read OR upstream_write))))) = '1'), (std_logic_vector'("0") & (dbs_adjusted_upstream_burstcount)), A_WE_StdLogicVector((std_logic'(update_count) = '1'), ((std_logic_vector'("0") & (transactions_remaining_reg)) - (std_logic_vector'("000") & (A_TOSTDLOGICVECTOR(internal_downstream_burstcount)))), (std_logic_vector'("0") & (transactions_remaining_reg)))), 3);
    end if;

  end process;

  process (clk, reset_n)
  begin
    if reset_n = '0' then
      data_counter <= std_logic_vector'("000");
    elsif clk'event and clk = '1' then
      data_counter <= A_EXT (A_WE_StdLogicVector((std_logic'(((state_idle AND upstream_read) AND NOT internal_upstream_waitrequest)) = '1'), (std_logic_vector'("000000000000000000000000000000") & (dbs_adjusted_upstream_burstcount)), A_WE_StdLogicVector((std_logic'(downstream_readdatavalid) = '1'), ((std_logic_vector'("000000000000000000000000000000") & (data_counter)) - std_logic_vector'("000000000000000000000000000000001")), (std_logic_vector'("000000000000000000000000000000") & (data_counter)))), 3);
    end if;

  end process;

  max_burst_size <= std_logic'('1');
  internal_downstream_burstcount <= Vector_To_Std_Logic(A_WE_StdLogicVector((std_logic'(current_upstream_read) = '1'), (std_logic_vector'("00000000000000000000000000000") & ((A_WE_StdLogicVector(((transactions_remaining>(std_logic_vector'("00") & (A_TOSTDLOGICVECTOR(max_burst_size))))), (std_logic_vector'("00") & (A_TOSTDLOGICVECTOR(max_burst_size))), transactions_remaining)))), std_logic_vector'("00000000000000000000000000000001")));
  downstream_arbitrationshare <= A_WE_StdLogicVector((std_logic'(current_upstream_read) = '1'), (dbs_adjusted_upstream_burstcount), dbs_adjusted_upstream_burstcount);
  process (clk, reset_n)
  begin
    if reset_n = '0' then
      write_address_offset <= std_logic_vector'("00");
    elsif clk'event and clk = '1' then
      write_address_offset <= A_EXT (A_WE_StdLogicVector((std_logic'((state_idle AND upstream_write)) = '1'), std_logic_vector'("00000000000000000000000000000000"), (std_logic_vector'("00000000000000000000000000000") & (A_WE_StdLogicVector((std_logic'(((((internal_downstream_write AND NOT downstream_waitrequest) AND downstream_burstdone) AND or_reduce(internal_downstream_byteenable(3 DOWNTO 2))))) = '1'), ((std_logic_vector'("0") & (write_address_offset)) + (std_logic_vector'("00") & (A_TOSTDLOGICVECTOR(internal_downstream_burstcount)))), (std_logic_vector'("0") & (write_address_offset)))))), 2);
    end if;

  end process;

  process (clk, reset_n)
  begin
    if reset_n = '0' then
      read_address_offset <= std_logic_vector'("00");
    elsif clk'event and clk = '1' then
      read_address_offset <= A_EXT (A_WE_StdLogicVector((std_logic'((state_idle AND upstream_read)) = '1'), std_logic_vector'("00000000000000000000000000000000"), (std_logic_vector'("00000000000000000000000000000") & (A_WE_StdLogicVector((std_logic'(((internal_downstream_read AND NOT downstream_waitrequest))) = '1'), ((std_logic_vector'("0") & (read_address_offset)) + (std_logic_vector'("00") & (A_TOSTDLOGICVECTOR(internal_downstream_burstcount)))), (std_logic_vector'("0") & (read_address_offset)))))), 2);
    end if;

  end process;

  downstream_nativeaddress <= A_SRL(registered_upstream_nativeaddress,std_logic_vector'("00000000000000000000000000000001"));
  address_offset <= A_WE_StdLogicVector((std_logic'(current_upstream_read) = '1'), read_address_offset, write_address_offset);
  downstream_address_base <= current_upstream_address;
  downstream_address <= A_EXT (((std_logic_vector'("0") & (downstream_address_base)) + (std_logic_vector'("00000000000000000000") & ((address_offset & std_logic_vector'("00"))))), 21);
  process (clk, reset_n)
  begin
    if reset_n = '0' then
      internal_downstream_read <= std_logic'('0');
    elsif clk'event and clk = '1' then
      if std_logic'((NOT internal_downstream_read OR NOT downstream_waitrequest)) = '1' then 
        internal_downstream_read <= Vector_To_Std_Logic(A_WE_StdLogicVector((std_logic'((state_idle AND upstream_read)) = '1'), std_logic_vector'("00000000000000000000000000000001"), A_WE_StdLogicVector(((transactions_remaining = (std_logic_vector'("00") & (A_TOSTDLOGICVECTOR(internal_downstream_burstcount))))), std_logic_vector'("00000000000000000000000000000000"), (std_logic_vector'("0000000000000000000000000000000") & (A_TOSTDLOGICVECTOR(internal_downstream_read))))));
      end if;
    end if;

  end process;

  process (clk, reset_n)
  begin
    if reset_n = '0' then
      negative_dbs_rdv_counter <= std_logic'('0');
    elsif clk'event and clk = '1' then
      negative_dbs_rdv_counter <= Vector_To_Std_Logic(A_WE_StdLogicVector((std_logic'((((state_idle AND upstream_read) AND NOT internal_upstream_waitrequest))) = '1'), (std_logic_vector'("00000000000000000000000000000000") & (A_TOSTDLOGICVECTOR(upstream_address(1)))), A_WE_StdLogicVector((std_logic'(fifo_datavalid) = '1'), ((std_logic_vector'("00000000000000000000000000000000") & (A_TOSTDLOGICVECTOR(negative_dbs_rdv_counter))) + std_logic_vector'("000000000000000000000000000000001")), (std_logic_vector'("00000000000000000000000000000000") & (A_TOSTDLOGICVECTOR(negative_dbs_rdv_counter))))));
    end if;

  end process;

  fifo_read <= NOT fifo_empty AND to_std_logic((((((std_logic_vector'("0000000000000000000000000000000") & (A_TOSTDLOGICVECTOR(negative_dbs_rdv_counter))) = std_logic_vector'("00000000000000000000000000000001"))) OR (((((std_logic_vector'("000000000000000000000000000000") & (full_width_rdv_counter)) + std_logic_vector'("000000000000000000000000000000001"))) = (std_logic_vector'("000000000000000000000000000000") & (current_upstream_burstcount)))))));
  fifo_write <= downstream_readdatavalid;
  fifo_wr_data <= downstream_readdata;
  flush_fifo <= std_logic'('0');
  --the_niosII_openMac_burst_0_fifo_module, which is an e_instance
  the_niosII_openMac_burst_0_fifo_module : niosII_openMac_burst_0_fifo_module
    port map(
      fifo_datavalid => fifo_datavalid,
      fifo_empty => fifo_empty,
      fifo_full => fifo_full,
      fifo_rd_data => fifo_rd_data,
      p1_fifo_empty => p1_fifo_empty,
      clk => clk,
      clk_en => module_input1,
      fifo_read => fifo_read,
      fifo_wr_data => fifo_wr_data,
      fifo_write => fifo_write,
      flush_fifo => flush_fifo,
      inc_pending_data => fifo_write,
      reset_n => reset_n
    );

  module_input1 <= std_logic'('1');

  process (clk, reset_n)
  begin
    if reset_n = '0' then
      full_width_rdv_counter <= std_logic_vector'("000");
    elsif clk'event and clk = '1' then
      full_width_rdv_counter <= A_EXT (A_WE_StdLogicVector((std_logic'((((state_idle AND upstream_read) AND NOT internal_upstream_waitrequest))) = '1'), std_logic_vector'("000000000000000000000000000000000"), A_WE_StdLogicVector((std_logic'(internal_upstream_readdatavalid) = '1'), ((std_logic_vector'("000000000000000000000000000000") & (full_width_rdv_counter)) + std_logic_vector'("000000000000000000000000000000001")), (std_logic_vector'("000000000000000000000000000000") & (full_width_rdv_counter)))), 3);
    end if;

  end process;

  internal_upstream_readdatavalid <= fifo_datavalid;
  upstream_readdata <= A_WE_StdLogicVector(((std_logic_vector'("00000000000000000000000000000000") = (std_logic_vector'("0000000000000000000000000000000") & (A_TOSTDLOGICVECTOR(negative_dbs_rdv_counter))))), Std_Logic_Vector'(fifo_rd_data(15 DOWNTO 0) & fifo_rd_data(15 DOWNTO 0)), Std_Logic_Vector'(fifo_rd_data(31 DOWNTO 16) & fifo_rd_data(31 DOWNTO 16)));
  internal_downstream_byteenable <= upstream_byteenable;
  internal_downstream_write <= ((upstream_write AND state_busy) AND NOT(pending_upstream_read)) AND fifo_empty;
  downstream_writedata <= upstream_writedata;
  upstream_read_run <= state_idle AND upstream_read;
  upstream_write_run <= ((state_busy AND upstream_write) AND NOT downstream_waitrequest) AND NOT(internal_downstream_read);
  internal_upstream_waitrequest <= Vector_To_Std_Logic(A_WE_StdLogicVector((std_logic'(((upstream_read OR current_upstream_read))) = '1'), (std_logic_vector'("0000000000000000000000000000000") & (A_TOSTDLOGICVECTOR(NOT upstream_read_run))), A_WE_StdLogicVector((std_logic'(current_upstream_write) = '1'), (std_logic_vector'("0000000000000000000000000000000") & (A_TOSTDLOGICVECTOR(NOT upstream_write_run))), std_logic_vector'("00000000000000000000000000000001"))));
  downstream_debugaccess <= upstream_debugaccess;
  --vhdl renameroo for output signals
  downstream_burstcount <= internal_downstream_burstcount;
  --vhdl renameroo for output signals
  downstream_byteenable <= internal_downstream_byteenable;
  --vhdl renameroo for output signals
  downstream_read <= internal_downstream_read;
  --vhdl renameroo for output signals
  downstream_write <= internal_downstream_write;
  --vhdl renameroo for output signals
  upstream_readdatavalid <= internal_upstream_readdatavalid;
  --vhdl renameroo for output signals
  upstream_waitrequest <= internal_upstream_waitrequest;
--synthesis translate_off
    process (clk)
    VARIABLE write_line : line;
    begin
      if clk'event and clk = '1' then
        if std_logic'((fifo_full AND fifo_write)) = '1' then 
          write(write_line, now);
          write(write_line, string'(": "));
          write(write_line, string'("simulation assertion failed: niosII_openMac_burst_0: illegal write into full fifo."));
          write(output, write_line.all);
          deallocate (write_line);
          assert false report "VHDL STOP" severity failure;
        end if;
      end if;

    end process;

--synthesis translate_on

end europa;

