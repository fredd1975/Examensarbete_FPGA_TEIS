-- ==========================================================================
-- Company        :  TEIS AB
-- Engineer       :  Freddy Avaria
-- Project        :  FPGA & TEIS Thesis - PXE-Based Remote Reboot System
-- Create Date    :  2025 Oct.27
-- Design name    :  tftp_accel_top.vhd
-- Target devices :  ALTERA MAX 10
-- Tool versions  :  Quartus v.18.1 and ModelSim
--
-- Description: Top file for network TFTP accelerator.
--              - Instantiates: parser_ipv4_udp_tftp, csr_bank, monitor_proc (passive).
--              - CSR clear pulse is combined into parser reset.
--              - Uses internal outputs for stream to keep the code VHDL-93 consistent
-- ==========================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.net_pkg.all;

entity tftp_accel_top is
   generic (
      G_CHECK_IPV4_CKSUM : boolean := true
      );
   port(
    -- Clock / Reset
      clk       : in  std_logic;                     -- system clock
      rst_n     : in  std_logic;                     -- global active-low reset

    -- Input (Source) stream from NIC
      s_tvalid  : in  std_logic;                     -- source stream valid (handshake)
      s_tlast   : in  std_logic;                     -- source stream last (end of frame)
      s_tdata   : in  std_logic_vector(7 downto 0);  -- source stream data (payload)
      s_tready  : out std_logic;                     -- source stream ready (handshake)

    -- Output (Master) stream to next stage
      m_tvalid  : out std_logic;                     -- master stream valid (handshake)
      m_tlast   : out std_logic;                     -- master stream last (end of frame)
      m_tdata   : out std_logic_vector(7 downto 0);  -- master stream data (payload)
      m_tready  : in  std_logic;                     -- master stream ready (handshake)

    -- CSR interface (Control & status register)
      csr_wr    : in  std_logic;                     -- csr write
      csr_rd    : in  std_logic;                     -- csr read
      csr_addr  : in  std_logic_vector(7 downto 0);  -- csr address
      csr_wdata : in  std_logic_vector(31 downto 0); -- csr write data
      csr_rdata : out std_logic_vector(31 downto 0); -- csr read data
      csr_ready : out std_logic                      -- csr ready
      );
end entity;

architecture rtl of tftp_accel_top is

  -- ===========================
  -- Internal stream handshakes
  -- ===========================
   signal s_tready_i  : std_logic;
   signal m_tvalid_i  : std_logic;
   signal m_tlast_i   : std_logic;
   signal m_tdata_i   : std_logic_vector(7 downto 0);

  -- ===========================
  -- Statistics (from parser)
  -- ===========================
   signal stat_ipv4_ok_s   : std_logic_vector(31 downto 0); 
   signal stat_udp_pkts_s  : std_logic_vector(31 downto 0);
   signal stat_tftp_data_s : std_logic_vector(31 downto 0);
   signal stat_tftp_ack_s  : std_logic_vector(31 downto 0);
   signal last_block_s     : std_logic_vector(15 downto 0);

  -- ===========================
  -- CSR / Clear pulse
  -- ===========================
   signal csr_ready_i    : std_logic;
   signal clear_cnt      : std_logic;
   signal clear_cnt_sync : std_logic := '0';

  -- ===========================
  -- Reset conditioning
  -- ===========================
   signal rst_sync_ff  : std_logic_vector(1 downto 0) := (others => '0');
   signal rst_parser_n : std_logic := '0';

  -- ===========================
  -- NEW: TFTP image ROM wiring
  -- ===========================
   signal tftp_img_addr : unsigned(31 downto 0);
   signal tftp_img_rd   : std_logic;
   signal tftp_img_q    : std_logic_vector(7 downto 0);

begin
  -- -----------------------------------------
  -- External-to-internal pass-throughs
  -- -----------------------------------------
   s_tready  <= s_tready_i;

   m_tvalid  <= m_tvalid_i;
   m_tlast   <= m_tlast_i;
   m_tdata   <= m_tdata_i;

   csr_ready <= csr_ready_i;

  -- ===============================================================
  -- Synchronize CSR clear pulse to clk
  -- ===============================================================
   ClearCnt_Sync: process(clk, rst_n)
   begin
      if rst_n = '0' then
         clear_cnt_sync <= '0';
      elsif rising_edge(clk) then
         clear_cnt_sync <= clear_cnt;
      end if;
   end process;

  -- ===============================================================
  -- Parser reset = global reset AND (NOT clear pulse)
  -- Active-low combine (any source can pull low)
  -- ===============================================================
   rst_parser_n <= rst_n and (not clear_cnt_sync);

  -- ===============================================================
  -- Parser instance
  -- ===============================================================
   u_parser : entity work.parser_ipv4_udp_tftp
      generic map (
         G_CHECK_IPV4_CKSUM => G_CHECK_IPV4_CKSUM,
         G_ENABLE_PARSE     => true
         )
      port map (
         clk            => clk,
         rst_n          => rst_parser_n,

      -- Stream in
         s_tdata        => s_tdata,
         s_tvalid       => s_tvalid,
         s_tready       => s_tready_i,
         s_tlast        => s_tlast,

      -- Stream out (to top internals)
         m_tdata        => m_tdata_i,
         m_tvalid       => m_tvalid_i,
         m_tready       => m_tready,
         m_tlast        => m_tlast_i,

      -- Stats
         stat_ipv4_ok   => stat_ipv4_ok_s,
         stat_udp_pkts  => stat_udp_pkts_s,
         stat_tftp_data => stat_tftp_data_s,
         stat_tftp_ack  => stat_tftp_ack_s,
         last_block     => last_block_s
         );

  -- ===============================================================
  -- CSR bank instance
  -- ===============================================================
   u_csr : entity work.csr_bank
      port map (
         clk            => clk,
         rst_n          => rst_n,

      -- Statistic inputs
         stat_ipv4_ok   => stat_ipv4_ok_s,
         stat_udp_pkts  => stat_udp_pkts_s,
         stat_tftp_data => stat_tftp_data_s,
         stat_tftp_ack  => stat_tftp_ack_s,
         last_block     => last_block_s,

      -- CSR bus
         csr_addr       => csr_addr,
         csr_wr         => csr_wr,
         csr_rd         => csr_rd,
         csr_wdata      => csr_wdata,
         csr_rdata      => csr_rdata,
         csr_ready      => csr_ready_i,

      -- Output pulse
         clear_counters => clear_cnt
         );

  -- ===============================================================
  -- Passive monitor (all-INPUTS, VHDL-93)
  -- ===============================================================
   u_mon : entity work.monitor_proc
      port map (
       -- Clock / reset
      clk            => clk,
      rst_n          => rst_n,                 -- or your synchronized reset signal
   
       -- Stream in/out (keep as you already had)
      s_tvalid       => s_tvalid,
      s_tready       => s_tready_i,
      s_tlast        => s_tlast,
      s_tdata        => s_tdata,
   
      m_tvalid       => m_tvalid_i,
      m_tready       => m_tready,
      m_tlast        => m_tlast_i,
      m_tdata        => m_tdata_i,
   
       -- Stats (keep your existing nets)
      stat_ipv4_ok   => stat_ipv4_ok_s,
      stat_udp_pkts  => stat_udp_pkts_s,
      stat_tftp_data => stat_tftp_data_s,
      stat_tftp_ack  => stat_tftp_ack_s,
      last_block     => last_block_s,

   
       -- NEW: feed the parser’s opcode
      tftp_opcode    => (others => '0')  --disables op-code print
     );



  -- ===============================================================
  -- Simple byte ROM for the TFTP server image (1-cycle sync)
  -- ===============================================================
   u_tftp_rom : entity work.simple_byte_rom
      port map (
         clk  => clk,
         addr => tftp_img_addr,
         rd   => tftp_img_rd,
         q    => tftp_img_q
      );

  -- ===============================================================
  -- Minimal TFTP server (passive RX tap; TX left open)
  --       - Does not drive existing m_* path.
  --       - Disabled by default (cfg_enable='0').
  -- ===============================================================
   u_tftp : entity work.tftp_min_server
      generic map (
         G_BLKSZ_BYTES => 32,   -- 512 originally, set to 32 for fast simulation
         G_SERVER_PORT => to_unsigned(40000,16),
         G_IMG_SIZE    => 4096
      )
      port map (
         clk        => clk,
         rst_n      => rst_n,

      -- RX sniff from the same NIC source stream
         rx_tvalid  => s_tvalid,
         rx_tlast   => s_tlast,
         rx_tdata   => s_tdata,

      -- TX: you can keep outputs open; FSM only needs READY=1 to finish a frame
         tx_tvalid  => open,
         tx_tready  => '1',
         tx_tlast   => open,
         tx_tdata   => open,

      -- Config: give it an identity and ENABLE IT *here*
         cfg_mac    => x"001122334455",
         cfg_ip     => x"C0A80002",   -- 192.168.0.2 (your frames’ DST IP)
         cfg_enable => '1',           -- <<< literal '1' into the core

      -- Image interface from ROM
         img_addr   => tftp_img_addr,
         img_rd     => tftp_img_rd,
         img_q      => tftp_img_q,

      -- Status left open (no top port changes)
         stat_rrq     => open,
         stat_data_tx => open,
         stat_ack_rx  => open,
         stat_active  => open
      );

end architecture;
