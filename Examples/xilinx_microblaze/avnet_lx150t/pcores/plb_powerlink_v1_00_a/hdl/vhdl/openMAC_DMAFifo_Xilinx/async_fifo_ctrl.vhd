------------------------------------------------------------------------------------------------------------------------
-- controller (top level file) of the async fifo
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
-- Note: A general implementation of a asynchronous fifo which is
--			using a dual port ram. This file is the controler top level entity.
--
------------------------------------------------------------------------------------------------------------------------
-- Version History
------------------------------------------------------------------------------------------------------------------------
-- 2011-09-22	V0.01		mairt		first version
-- 2012-01-02   V0.02       zelenkaj    bugfix sync error
------------------------------------------------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;	   

entity async_fifo_ctrl is
   generic(ADDR_WIDTH : natural := 5);
   port(		  
   	  -- write part
      clkw: in std_logic;
      resetw: in std_logic;
      wr: in std_logic;
      w_empty: out std_logic;
	  w_full: out std_logic;
      w_addr: out std_logic_vector (ADDR_WIDTH-1 downto 0);
	  rd_used_w: out std_logic_vector(ADDR_WIDTH-1 downto 0);
	  
	  -- read part
      clkr: in std_logic;
      resetr: in std_logic;
      rd: in std_logic;
      r_empty: out std_logic;
	  r_full: out std_logic;
      r_addr: out std_logic_vector (ADDR_WIDTH-1 downto 0); 
	  wd_used_w: out std_logic_vector(ADDR_WIDTH-1 downto 0)
   );
end async_fifo_ctrl ;

architecture str_arch of async_fifo_ctrl is
   signal r_ptr_in: std_logic_vector(ADDR_WIDTH downto 0);
   signal r_ptr_out: std_logic_vector(ADDR_WIDTH downto 0);
   signal w_ptr_in: std_logic_vector(ADDR_WIDTH downto 0);
   signal w_ptr_out: std_logic_vector(ADDR_WIDTH downto 0);	
   
   -- component declarations
   component fifo_read_ctrl
      generic(N: natural);
      port(
         clkr: in std_logic;
         rd: in std_logic;
         resetr: in std_logic;
         w_ptr_in: in std_logic_vector (N downto 0);
         r_full: out std_logic;
		 r_empty: out std_logic;
         r_addr: out std_logic_vector (N-1 downto 0);
         r_ptr_out: out std_logic_vector (N downto 0);
		 r_elements: out std_logic_vector(N-1 downto 0)
      );
   end component;	 
   
   component fifo_write_ctrl
      generic(N: natural);
      port(
         clkw: in std_logic;
         r_ptr_in: in std_logic_vector (N downto 0);
         resetw: in std_logic;
         wr: in std_logic;
         w_full: out std_logic;	
		 w_empty: out std_logic;
         w_addr: out std_logic_vector (N-1 downto 0);
         w_ptr_out: out std_logic_vector (N downto 0);
		 w_elements: out std_logic_vector(N-1 downto 0)
      );
   end component;			 
   
   component synchronizer_g
      generic(N: natural);
      port(
         clk: in std_logic;
         in_async: in std_logic_vector (N-1 downto 0);
         reset: in std_logic;
         out_sync: out std_logic_vector (N-1 downto 0)
      );
   end component; 
   
begin
   read_ctrl: fifo_read_ctrl
      generic map(N=>ADDR_WIDTH)
      port map (clkr=>clkr, 
	  			resetr=>resetr, 
	  			rd=>rd,
                w_ptr_in=>w_ptr_in, 
				r_empty=>r_empty,
				r_full=>r_full,
                r_ptr_out=>r_ptr_out, 
				r_addr=>r_addr,
				r_elements=>rd_used_w
			);
				
   write_ctrl: fifo_write_ctrl
      generic map(N =>ADDR_WIDTH)
      port map(clkw=>clkw, 
			  resetw=>resetw, 
			  wr=>wr,
			  r_ptr_in=>r_ptr_in,
			  w_empty=>w_empty,
			  w_full=>w_full,
			  w_ptr_out=>w_ptr_out, 
			  w_addr=>w_addr,
			  w_elements=>wd_used_w
	  		);	 
			  
   sync_w_ptr: synchronizer_g
      generic map(N=>ADDR_WIDTH+1)
      port map(clk=>clkr, 
			  reset=>resetr,
			  in_async=>w_ptr_out, 
			  out_sync=>w_ptr_in
	  		);
	  
   sync_r_ptr: synchronizer_g
      generic map(N=>ADDR_WIDTH+1)
      port map(clk=>clkw, 
			  reset=>resetw,
			  in_async=>r_ptr_out, 
			  out_sync =>r_ptr_in
	  		); 
			  
end str_arch;