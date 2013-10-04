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

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;

entity master_handler is
    generic(
        dma_highadr_g : integer := 31;
        gen_tx_fifo_g : boolean := true;
        tx_fifo_word_size_log2_g : natural := 5;
        gen_rx_fifo_g : boolean := true;
        rx_fifo_word_size_log2_g : natural := 5;
        m_burstcount_width_g : integer := 4;
        m_rx_burst_size_g : integer := 16;
        m_tx_burst_size_g : integer := 16;
        m_burst_wr_const_g : boolean := true;
        fifo_data_width_g : integer := 16
    );
    port(
        m_clk : in std_logic;
        rst : in std_logic;
        mac_tx_off : in std_logic;
        mac_rx_off : in std_logic;
        tx_wr_clk : in std_logic;
        tx_wr_empty : in std_logic;
        tx_wr_full : in std_logic;
        rx_rd_clk : in std_logic;
        rx_rd_empty : in std_logic;
        rx_rd_full : in std_logic;
        tx_wr_usedw : in std_logic_vector(tx_fifo_word_size_log2_g-1 downto 0);
        rx_rd_usedw : in std_logic_vector(rx_fifo_word_size_log2_g-1 downto 0);
        tx_aclr : out std_logic;
        tx_wr_req : out std_logic;
        rx_rd_req : out std_logic;
        m_waitrequest : in std_logic;
        m_readdatavalid : in std_logic;
        m_write : out std_logic;
        m_read : out std_logic;
        m_address : out std_logic_vector(dma_highadr_g downto 0);
        m_byteenable : out std_logic_vector(fifo_data_width_g/8-1 downto 0);
        m_burstcount : out std_logic_vector(m_burstcount_width_g-1 downto 0);
        m_burstcounter : out std_logic_vector(m_burstcount_width_g-1 downto 0);
        dma_addr_in : in std_logic_vector(dma_highadr_g downto 1);
        dma_len_rd : in std_logic_vector(11 downto 0);
        dma_new_addr_wr : in std_logic;
        dma_new_addr_rd : in std_logic;
        dma_new_len_rd : in std_logic
    );
end master_handler;

architecture master_handler of master_handler is
--clock signal
signal clk : std_logic;

--constants
constant tx_burst_size_c : integer := m_tx_burst_size_g; --(2**(m_burstcount_width_g-1));
constant rx_burst_size_c : integer := m_rx_burst_size_g; --(2**(m_burstcount_width_g-1));
---used to trigger rx/tx data transfers depending on fill level and burst size
constant tx_fifo_limit_c : integer := 2**tx_fifo_word_size_log2_g - tx_burst_size_c - 1; --fifo_size - burst size - 1
constant rx_fifo_limit_c : integer := rx_burst_size_c + 1; --burst size

--fsm
type transfer_t is (idle, run, finish);
signal tx_fsm, tx_fsm_next, rx_fsm, rx_fsm_next : transfer_t := idle;

--transfer signals
signal m_burstcount_s, m_burstcount_latch : std_logic_vector(m_burstcount'range);
signal m_address_latch : std_logic_vector(m_address'range);
signal m_write_s, m_read_s : std_logic;
signal rx_first_read_done, rx_rd_done : std_logic;

--fifo signals
signal arst : std_logic;
signal tx_fifo_limit, rx_fifo_limit : std_logic;
signal tx_wr_req_s, rx_rd_req_s, rx_first_rd_req : std_logic;

--generate addresses
signal tx_cnt, tx_cnt_next : std_logic_vector(m_address'range);
signal rx_cnt, rx_cnt_next : std_logic_vector(m_address'range);

--handle tx read transfer
signal tx_rd_cnt, tx_rd_cnt_next : std_logic_vector(dma_len_rd'range);
signal dma_len_rd_s : std_logic_vector(dma_len_rd'range);
begin

    --m_clk, rx_rd_clk and tx_wr_clk are the same!
    clk <= m_clk; --to ease typing

    tx_aclr <= rst or arst;

    --fifo limit is set to '1' if the fill level is equal/above the limit
    tx_fifo_limit <= '1' when tx_wr_usedw >= conv_std_logic_vector(tx_fifo_limit_c, tx_wr_usedw'length) else '0';
    rx_fifo_limit <= '1' when rx_rd_usedw >= conv_std_logic_vector(rx_fifo_limit_c, rx_rd_usedw'length) else '0';

    process(clk, rst)
    begin
        if rst = '1' then
            if gen_rx_fifo_g then
                rx_fsm <= idle;
            end if;
            if gen_tx_fifo_g then
                tx_fsm <= idle;
            end if;
        elsif clk = '1' and clk'event then
            if gen_rx_fifo_g then
                rx_fsm <= rx_fsm_next;
            end if;
            if gen_tx_fifo_g then
                tx_fsm <= tx_fsm_next;
            end if;
        end if;
    end process;

    tx_fsm_next <=     run when tx_fsm = idle and dma_new_addr_rd = '1' else
                    finish when tx_fsm = run and mac_tx_off = '1' else
                    idle when tx_fsm = finish and tx_wr_empty = '1' else --stay finish as long as tx fifo is filled
                    tx_fsm;

    rx_fsm_next <=    run when rx_fsm = idle and dma_new_addr_wr = '1' else
                    finish when rx_fsm = run and mac_rx_off = '1' else
                    idle when rx_fsm = finish and rx_rd_done = '1' else --stay finish as long the transfer process is not done
                    rx_fsm;

    m_burstcount <= m_burstcount_latch when m_write_s = '1' and m_burst_wr_const_g else m_burstcount_s;
    m_burstcounter <= m_burstcount_s; --output current burst counter value
    m_write <= m_write_s;
    m_read <= m_read_s;

    --generate address
    m_address <=     m_address_latch when m_write_s = '1' and m_burst_wr_const_g else
                    rx_cnt when m_write_s = '1' and not m_burst_wr_const_g else
                    tx_cnt;

    process(clk, rst)
    begin
        if rst = '1' then
            if gen_tx_fifo_g then
                tx_cnt <= (others => '0');
                tx_rd_cnt <= (others => '0');
            end if;
            if gen_rx_fifo_g then
                rx_cnt <= (others => '0');
            end if;
        elsif clk = '1' and clk'event then
            if gen_tx_fifo_g then
                tx_cnt <= tx_cnt_next;
                tx_rd_cnt <= tx_rd_cnt_next;
            end if;
            if gen_rx_fifo_g then
                rx_cnt <= rx_cnt_next;
            end if;
        end if;
    end process;

    dma_len_rd_s <= dma_len_rd + 1 when fifo_data_width_g = 16 else
                    dma_len_rd + 3 when fifo_data_width_g = 32 else
                    dma_len_rd;

    tx_rd_cnt_next <=   (others => '0') when gen_tx_fifo_g = false else
                        '0' & dma_len_rd_s(dma_len_rd_s'left downto 1) when dma_new_len_rd = '1' and fifo_data_width_g = 16 else
                        "00" & dma_len_rd_s(dma_len_rd_s'left downto 2) when dma_new_len_rd = '1' and fifo_data_width_g = 32 else
                        tx_rd_cnt - 1 when tx_wr_req_s = '1' and tx_rd_cnt /= 0 else
                        tx_rd_cnt;

    tx_cnt_next <=     (others => '0') when gen_tx_fifo_g = false else
                    tx_cnt + fifo_data_width_g/8 when tx_wr_req_s = '1' else
                    dma_addr_in & '0' when dma_new_addr_rd = '1' else
                    tx_cnt;

    rx_cnt_next <=    (others => '0') when gen_rx_fifo_g = false else
                    rx_cnt + fifo_data_width_g/8 when rx_rd_req_s = '1' else
                    dma_addr_in & '0' when dma_new_addr_wr = '1' else
                    rx_cnt;

    m_byteenable <= (others => '1');

    tx_wr_req_s <= m_readdatavalid;
    tx_wr_req <= tx_wr_req_s;

    rx_rd_req_s <= m_write_s and not m_waitrequest;
    rx_rd_req <= rx_rd_req_s or rx_first_rd_req;

    process(clk, rst)
    --arbitration of rx and tx requests is done by process variable (tx overrules rx)
    variable tx_is_the_owner_v : std_logic;
    begin
        if rst = '1' then
            tx_is_the_owner_v := '0';

            if gen_tx_fifo_g then
                arst <= '0';

                m_read_s <= '0';
            end if;

            if gen_rx_fifo_g then
                rx_first_rd_req <= '0';

                m_write_s <= '0';

                rx_first_read_done <= '0';
                rx_rd_done <= '0';
            end if;

            m_burstcount_s <= (others => '0');
            if m_burst_wr_const_g then
                m_burstcount_latch <= (others => '0');
                m_address_latch <= (others => '0');
            end if;

        elsif clk = '1' and clk'event then

            if gen_tx_fifo_g then
                arst <= '0';

                if m_readdatavalid = '1' then
                    --read was successful -> write to tx fifo
                    m_burstcount_s <= m_burstcount_s - 1;
                end if;

                case tx_fsm is
                    when idle =>
                        --no transfer in progress
                    when run =>
                        --read transfer base address is ready
                        if tx_fifo_limit = '0' and m_read_s = '0' and m_write_s = '0' and m_burstcount_s = 0 and tx_rd_cnt /= 0 then
                            --tx fifo is below defined limit -> there is place for at least one burst!
                            m_read_s <= '1';
                            if tx_rd_cnt > conv_std_logic_vector(tx_burst_size_c, tx_rd_cnt'length) then
                                m_burstcount_s <= conv_std_logic_vector(tx_burst_size_c, m_burstcount_s'length);
                            else
                                m_burstcount_s <= EXT(tx_rd_cnt, m_burstcount_s'length);
                            end if;
                            --a tx transfer is necessary and overrules necessary rx transfers...
                            tx_is_the_owner_v := '1';
                        elsif m_read_s = '1' and m_waitrequest = '0' then
                            --request is confirmed -> deassert request
                            m_read_s <= '0';
                            --so, we are done with tx requesting
                            tx_is_the_owner_v := '0';
                        end if;
                    when finish =>
                        --transfer done, MAC has its data...
                        ---is there still a request?
                        if m_read_s = '1' and m_waitrequest = '0' then
                            --last request confirmed -> deassert request
                            m_read_s <= '0'; tx_is_the_owner_v := '0';
                        ---is the burst transfer done?
                        elsif m_read_s = '0' and m_burstcount_s = 0 then
                            --burst transfer done, clear fifo
                            arst <= '1';
                        end if;
                end case;

            end if;

            if gen_rx_fifo_g then
                rx_first_rd_req <= '0';
                rx_rd_done <= '0';

                if m_write_s = '1' and m_waitrequest = '0' then
                    --write was successful
                    m_burstcount_s <= m_burstcount_s - 1;
                end if;

                case rx_fsm is
                    when idle =>
                        --no transfer in progress
                        rx_first_read_done <= '0';
                    when run =>
                        --a not empty fifo has to be read once, to get the very first pattern
                        if rx_first_read_done = '0' and rx_rd_empty = '0' then
                            rx_first_read_done <= '1';
                            rx_first_rd_req <= '1';
                        end if;
                        --write transfer base address is ready
                        if rx_fifo_limit = '1' and m_read_s = '0' and m_write_s = '0' and tx_is_the_owner_v = '0' and m_burstcount_s = 0 and rx_first_read_done = '1' then
                            --rx fifo is filled with enough data -> build burst transfer
                            m_write_s <= '1';
                            m_burstcount_s <= conv_std_logic_vector(rx_burst_size_c, m_burstcount_s'length);
                            if m_burst_wr_const_g then
                                m_burstcount_latch <= conv_std_logic_vector(rx_burst_size_c, m_burstcount_latch'length);
                                m_address_latch <= rx_cnt;
                            end if;
                        elsif m_write_s = '1' and m_waitrequest = '0' and m_burstcount_s = 1 then
                            --last transfer is done -> deassert write qualifiers
                            m_write_s <= '0';
                        end if;
                    when finish =>
                        --MAC is finished with RX, transfer rest of fifo
                        ---note: The last word (part of crc32) is not transferred!
                        if rx_rd_empty = '0' and m_read_s = '0' and m_write_s = '0' and tx_is_the_owner_v = '0' and m_burstcount_s = 0 then
                            --rx fifo has some data left
                            m_write_s <= '1';
                            --verify how many patterns are left in the fifo
                            if rx_rd_usedw < conv_std_logic_vector(rx_burst_size_c, rx_rd_usedw'length) then
                                --start the smaller burst write transfer
                                m_burstcount_s <= EXT(rx_rd_usedw, m_burstcount_s'length);
                                if m_burst_wr_const_g then
                                    m_burstcount_latch <= EXT(rx_rd_usedw, m_burstcount_latch'length);
                                    m_address_latch <= rx_cnt;
                                end if;

                                --workaround: fifo is not empty but word level is zero => set to one
                                if rx_rd_usedw = 0 then
                                    m_burstcount_s <= conv_std_logic_vector(1, m_burstcount_s'length);
                                    m_burstcount_latch <= conv_std_logic_vector(1, m_burstcount_latch'length);
                                end if;
                            else
                                --start the maximum burst write transfer
                                m_burstcount_s <= conv_std_logic_vector(rx_burst_size_c, m_burstcount_s'length);
                                if m_burst_wr_const_g then
                                    m_burstcount_latch <= conv_std_logic_vector(rx_burst_size_c, m_burstcount_latch'length);
                                    m_address_latch <= rx_cnt;
                                end if;
                            end if;
                        elsif m_write_s = '1' and m_waitrequest = '0' and m_burstcount_s = 1 then
                            --transfer is done -> deassert write qualifiers
                            m_write_s <= '0';
                            --completely done?!
                            if rx_rd_empty = '1' then
                                --yes!
                                rx_rd_done <= '1';
                            end if;
                        elsif rx_rd_empty = '1' and m_write_s = '0' then
                            --nothing left in the fifo and we don't try to do anything -> done!
                            rx_rd_done <= '1';
                        end if;
                end case;

            end if;

        end if;
    end process;

end master_handler;
