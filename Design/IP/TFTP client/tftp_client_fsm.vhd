--==========================================================================
-- Company        :  TEIS AB
-- Engineer       :  Freddy Avaria
-- Project        :  FPGA & TEIS Thesis - PXE-Based Remote Reboot System
-- Create Date    :  2025 Nov. 14
-- Design name    :  tftp_client_fsm.vhd
-- Target devices :  ALTERA MAX 10
-- Tool versions  :  Quartus v.18.1 and ModelSim
--
-- Description    :  Minimal TFTP client (validation traffic generator)
--                   - Emits one RRQ for a given filename
--                   - Sniffs returning DATA blocks (opcode=3)
--                   - Sends ACK for each DATA block (opcode=4)
--                   - Stops after MAX_BLOCKS or when rx stops
--                   - IPv4 checksum = 0, UDP checksum = 0
--
-- Framing / lengths (byte-level):
--   * Ethernet header:        14 bytes
--   * IPv4 header (no opts):  20 bytes
--   * UDP header:              8 bytes
--   * TFTP RRQ payload:    2B opcode (0001) + filename\0 + mode\0
--   * TFTP ACK payload:    2B opcode (0004) + 2B block
--==========================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity tftp_client_fsm is
   generic (
      G_CLIENT_MAC   : std_logic_vector(47 downto 0) := x"001122334455";
      G_CLIENT_IP    : std_logic_vector(31 downto 0) := x"C0A80001"; -- 192.168.0.1
      G_CLIENT_PORT  : unsigned(15 downto 0)        := to_unsigned(40001, 16);

      G_SERVER_MAC   : std_logic_vector(47 downto 0) := x"FFFFFFFFFFFF"; -- broadcast RRQ
      G_SERVER_IP    : std_logic_vector(31 downto 0) := x"C0A80002";     -- 192.168.0.2
      G_SERVER_PORT  : unsigned(15 downto 0)        := to_unsigned(69, 16); -- TFTP 69

      G_FILENAME     : string := "boot.bin";
      G_MODE         : string := "octet";

      G_MAX_BLOCKS   : natural := 8
   );

   port (
      -- Clock/Reset
      clk     : in  std_logic;                        -- system clock
      rst_n   : in  std_logic;                        -- reset (active low)

      -- RX from server (DATA frames)
      rx_tvalid : in  std_logic;                      -- RX data valid (handshake)
      rx_tlast  : in  std_logic;                      -- RX last (end of frame)
      rx_tdata  : in  std_logic_vector(7 downto 0);   -- RX data (payload)

      -- TX to server (RRQ + ACK)
      tx_tvalid : out std_logic;                      -- TX data valid (handshake)
      tx_tready : in  std_logic;                      -- TX ready (handshake)
      tx_tlast  : out std_logic;                      -- TX last (end of frame)
      tx_tdata  : out std_logic_vector(7 downto 0);   -- TX data (payload)

      -- Stats
      stat_rrq_sent  : out unsigned(31 downto 0);     -- RRQ frames sent
      stat_data_seen : out unsigned(31 downto 0);     -- TFTP RX data seen
      stat_ack_sent  : out unsigned(31 downto 0);     -- TFTP ACK frames sent
      stat_active    : out std_logic;                 -- Status signal ('1' while FSM active)

      -- Debug (help State Machine Viewer)
      dbg_tx_state : out std_logic_vector(3 downto 0)
   );
end entity;

architecture rtl of tftp_client_fsm is

   -------------------------------------------------------------------------
   -- Helpers: character → byte; zero-terminated string → byte-vector
   -------------------------------------------------------------------------
   function to_slv8(c : character) return std_logic_vector is
   begin
      return std_logic_vector(to_unsigned(character'pos(c), 8));
   end;

   function strz_to_vec(s : string) return std_logic_vector is
      variable v : std_logic_vector(8*s'length-1 downto 0);
   begin
      for i in s'range loop
         v(8*(i - s'low + 1)-1 downto 8*(i - s'low)) := to_slv8(s(i));
      end loop;
      return v;
   end;

   constant FNAME_LEN  : natural := G_FILENAME'length + 1; -- +\0
   constant MODE_LEN   : natural := G_MODE'length + 1;     -- +\0
   constant RRQ_PAYLEN : natural := 2 + FNAME_LEN + MODE_LEN;

   --------------------------------------------------------------------------
   -- TX FSM
   --------------------------------------------------------------------------
   type tx_state_t is (
      TX_IDLE, TX_RRQ_ETH, TX_RRQ_IP, TX_RRQ_UDP, TX_RRQ_TFTP,
      TX_ACK_ETH, TX_ACK_IP, TX_ACK_UDP, TX_ACK_TFTP, TX_EOF
   );

   signal tx_st, tx_st_next       : tx_state_t := TX_IDLE;
   signal tx_bcnt, tx_bcnt_next   : unsigned(15 downto 0) := (others => '0');
   signal tx_valid_r, tx_valid_n  : std_logic := '0';
   signal tx_last_r,  tx_last_n   : std_logic := '0';
   signal tx_data_r               : std_logic_vector(7 downto 0) := (others => '0');
   signal tx_data_n               : std_logic_vector(7 downto 0) := (others => '0');

   -- ACK fields
   signal ack_block       : std_logic_vector(15 downto 0) := (others => '0');
   signal ack_dst_mac     : std_logic_vector(47 downto 0) := (others => '0');
   signal ack_dst_ip      : std_logic_vector(31 downto 0) := (others => '0');
   signal ack_dst_port    : unsigned(15 downto 0)         := (others => '0');

   -- Bookkeeping
   signal rrq_sent, rrq_sent_n   : std_logic := '0';
   signal blocks_seen            : unsigned(31 downto 0) := (others => '0');
   signal s_rrq, s_rrq_n         : unsigned(31 downto 0) := (others => '0');
   signal s_data                 : unsigned(31 downto 0) := (others => '0');
   signal s_ack, s_ack_n         : unsigned(31 downto 0) := (others => '0');

   -- Handshake between RX and TX for ACK
   signal ack_pending   : std_logic := '0';  -- OWNED by rx_proc
   signal ack_go        : std_logic := '0';
   signal ack_go_n      : std_logic := '0';

   -- "client active" flag
   signal active_r, active_n : std_logic := '0';

   --------------------------------------------------------------------------
   -- RX parse (absolute indices)
   --------------------------------------------------------------------------
   signal r_bcnt          : unsigned(15 downto 0) := (others => '0');
   signal r_in_frm        : std_logic := '0';
   signal r_ethertype     : std_logic_vector(15 downto 0) := (others => '0');
   signal r_ip_proto      : std_logic_vector(7 downto 0)  := (others => '0');
   signal r_udp_src_port  : std_logic_vector(15 downto 0) := (others => '0');
   signal r_udp_dst_port  : std_logic_vector(15 downto 0) := (others => '0');
   signal r_tftp_opcode   : std_logic_vector(15 downto 0) := (others => '0');
   signal r_tftp_block    : std_logic_vector(15 downto 0) := (others => '0');

   --------------------------------------------------------------------------
   -- Constants
   --------------------------------------------------------------------------
   constant C_ETH_IP    : std_logic_vector(15 downto 0) := x"0800";
   constant C_UDP       : std_logic_vector(7 downto 0)  := x"11";
   constant C_TFTP_RRQ  : std_logic_vector(15 downto 0) := x"0001";
   constant C_TFTP_DATA : std_logic_vector(15 downto 0) := x"0003";
   constant C_TFTP_ACK  : std_logic_vector(15 downto 0) := x"0004";

   constant FNAME_VEC : std_logic_vector(8*G_FILENAME'length-1 downto 0)
                        := strz_to_vec(G_FILENAME);
   constant MODE_VEC  : std_logic_vector(8*G_MODE'length-1 downto 0)
                        := strz_to_vec(G_MODE);

   function to_u16(n : natural) return unsigned is
   begin
      return to_unsigned(n, 16);
   end;

   constant RRQ_UDP_LEN  : unsigned(15 downto 0) := to_u16(8 + RRQ_PAYLEN);
   constant RRQ_IP_LEN   : unsigned(15 downto 0) := to_u16(20 + 8 + RRQ_PAYLEN);
   constant ACK_TFTP_LEN : unsigned(15 downto 0) := to_u16(4);
   constant ACK_UDP_LEN  : unsigned(15 downto 0) := to_u16(8 + 4);
   constant ACK_IP_LEN   : unsigned(15 downto 0) := to_u16(20 + 8 + 4);

begin
   --------------------------------------------------------------------------
   -- Output mappings
   --------------------------------------------------------------------------
   tx_tvalid <= tx_valid_r;
   tx_tlast  <= tx_last_r;
   tx_tdata  <= tx_data_r;

   stat_rrq_sent  <= s_rrq;
   stat_data_seen <= s_data;
   stat_ack_sent  <= s_ack;
   stat_active    <= active_r;

   dbg_tx_state   <= std_logic_vector(to_unsigned(tx_state_t'pos(tx_st), 4));

   ----------------------------------------------------------------------------
   -- RX sniff + counters (SOLE writer of blocks_seen, s_data, ack_pending)
   ----------------------------------------------------------------------------
   rx_proc : process(clk, rst_n)
   begin
      if rst_n = '0' then
         r_bcnt         <= (others => '0');
         r_in_frm       <= '0';
         r_ethertype    <= (others => '0');
         r_ip_proto     <= (others => '0');
         r_udp_src_port <= (others => '0');
         r_udp_dst_port <= (others => '0');
         r_tftp_opcode  <= (others => '0');
         r_tftp_block   <= (others => '0');
         blocks_seen    <= (others => '0');
         s_data         <= (others => '0');
         ack_pending    <= '0';
      elsif rising_edge(clk) then
         -- clear pending ACK once TX FSM has consumed it
         if ack_go = '1' then
            ack_pending <= '0';
         end if;

         -- byte count within a frame
         if rx_tvalid = '1' and r_in_frm = '0' then
            r_in_frm <= '1';
            r_bcnt   <= (others => '0');
         elsif rx_tvalid = '1' and r_in_frm = '1' then
            r_bcnt <= r_bcnt + 1;
         end if;

         -- capture header fields
         if rx_tvalid = '1' and r_in_frm = '1' then
            case to_integer(r_bcnt) is
               -- Ethernet Ethertype (0x0800)
               when 11 => r_ethertype(15 downto 8) <= rx_tdata;
               when 12 => r_ethertype(7  downto 0) <= rx_tdata;

               -- IPv4 protocol (0x11 = UDP)
               when 22 => r_ip_proto <= rx_tdata;

               -- UDP ports
               when 33 => r_udp_src_port(15 downto 8) <= rx_tdata;
               when 34 => r_udp_src_port(7  downto 0) <= rx_tdata;
               when 35 => r_udp_dst_port(15 downto 8) <= rx_tdata;
               when 36 => r_udp_dst_port(7  downto 0) <= rx_tdata;

               -- TFTP opcode + block
               when 41 => r_tftp_opcode(15 downto 8) <= rx_tdata;
               when 42 => r_tftp_opcode(7  downto 0)  <= rx_tdata;
               when 43 => r_tftp_block(15 downto 8)  <= rx_tdata;
               when 44 => r_tftp_block(7  downto 0)  <= rx_tdata;

               when others => null;
            end case;

         end if;

                  -- end-of-frame: if a TFTP DATA frame, flag ACK pending
         if rx_tvalid = '1' and rx_tlast = '1' then
            if r_tftp_opcode = C_TFTP_DATA then
               s_data       <= s_data + 1;
               blocks_seen  <= blocks_seen + 1;
               ack_block    <= r_tftp_block;
               ack_dst_mac  <= G_SERVER_MAC;
               ack_dst_ip   <= G_SERVER_IP;
               ack_dst_port <= unsigned(r_udp_src_port);
               ack_pending  <= '1';
            end if;
            r_in_frm <= '0';
         end if;


      end if;
   end process;

   ----------------------------------------------------------------------------
   -- TX FSM: sequential (registers)
   ----------------------------------------------------------------------------
   tx_seq : process(clk, rst_n)
   begin
      if rst_n = '0' then
         tx_st      <= TX_IDLE;
         tx_bcnt    <= (others => '0');
         tx_valid_r <= '0';
         tx_last_r  <= '0';
         tx_data_r  <= (others => '0');
         rrq_sent   <= '0';
         s_rrq      <= (others => '0');
         s_ack      <= (others => '0');
         active_r   <= '0';
         ack_go     <= '0';
      elsif rising_edge(clk) then
         tx_st      <= tx_st_next;
         tx_bcnt    <= tx_bcnt_next;
         tx_valid_r <= tx_valid_n;
         tx_last_r  <= tx_last_n;
         tx_data_r  <= tx_data_n;
         rrq_sent   <= rrq_sent_n;
         s_rrq      <= s_rrq_n;
         s_ack      <= s_ack_n;
         active_r   <= active_n;
         ack_go     <= ack_go_n;
      end if;
   end process;

   ----------------------------------------------------------------------------
   -- TX FSM: combinational
   ----------------------------------------------------------------------------
   tx_comb : process(tx_st, tx_bcnt, tx_tready, rrq_sent,
                     ack_pending, blocks_seen,
                     s_rrq, s_ack,
                     ack_block, ack_dst_mac, ack_dst_ip, ack_dst_port,
                     active_r)
      variable idx : integer;
   begin
      -- defaults
      tx_st_next   <= tx_st;
      tx_bcnt_next <= tx_bcnt;
      tx_valid_n   <= '0';
      tx_last_n    <= '0';
      tx_data_n    <= (others => '0');
      rrq_sent_n   <= rrq_sent;
      s_rrq_n      <= s_rrq;
      s_ack_n      <= s_ack;
      active_n     <= active_r;
      ack_go_n     <= '0';

      case tx_st is
         ------------------------------------------------------------------
         -- TX_IDLE
         ------------------------------------------------------------------
         when TX_IDLE =>
            active_n <= '1';

            -- highest priority: respond to DATA with ACK (if below limit)
            if ack_pending = '1' and
               (blocks_seen < to_unsigned(G_MAX_BLOCKS, blocks_seen'length)) then

               tx_bcnt_next <= (others => '0');
               tx_st_next   <= TX_ACK_ETH;
               ack_go_n     <= '1';   -- tell RX we consumed the pending ACK

            -- otherwise, if we have never sent RRQ, start it
            elsif rrq_sent = '0' then
               tx_bcnt_next <= (others => '0');
               tx_st_next   <= TX_RRQ_ETH;
            else
               -- stay idle, no actions
               null;
            end if;

         ------------------------------------------------------------------
         -- TX_RRQ_ETH : Ethernet header for RRQ
         ------------------------------------------------------------------
         when TX_RRQ_ETH =>
            if tx_tready = '1' then
               tx_valid_n <= '1';
               case to_integer(tx_bcnt) is
                  when 0  => tx_data_n <= G_SERVER_MAC(47 downto 40);
                  when 1  => tx_data_n <= G_SERVER_MAC(39 downto 32);
                  when 2  => tx_data_n <= G_SERVER_MAC(31 downto 24);
                  when 3  => tx_data_n <= G_SERVER_MAC(23 downto 16);
                  when 4  => tx_data_n <= G_SERVER_MAC(15 downto 8);
                  when 5  => tx_data_n <= G_SERVER_MAC(7  downto 0);
                  when 6  => tx_data_n <= G_CLIENT_MAC(47 downto 40);
                  when 7  => tx_data_n <= G_CLIENT_MAC(39 downto 32);
                  when 8  => tx_data_n <= G_CLIENT_MAC(31 downto 24);
                  when 9  => tx_data_n <= G_CLIENT_MAC(23 downto 16);
                  when 10 => tx_data_n <= G_CLIENT_MAC(15 downto 8);
                  when 11 => tx_data_n <= G_CLIENT_MAC(7  downto 0);
                  when 12 => tx_data_n <= C_ETH_IP(15 downto 8);
                  when 13 => tx_data_n <= C_ETH_IP(7  downto 0);
                  when others => tx_data_n <= (others => '0');
               end case;

               if tx_bcnt = to_unsigned(13, tx_bcnt'length) then
                  tx_bcnt_next <= (others => '0');
                  tx_st_next   <= TX_RRQ_IP;
               else
                  tx_bcnt_next <= tx_bcnt + 1;
               end if;
            end if;

         ------------------------------------------------------------------
         -- TX_RRQ_IP : IPv4 header for RRQ
         ------------------------------------------------------------------
         when TX_RRQ_IP =>
            if tx_tready = '1' then
               tx_valid_n <= '1';
               case to_integer(tx_bcnt) is
                  when 0  => tx_data_n <= x"45";
                  when 1  => tx_data_n <= x"00";
                  when 2  => tx_data_n <= std_logic_vector(RRQ_IP_LEN(15 downto 8));
                  when 3  => tx_data_n <= std_logic_vector(RRQ_IP_LEN(7  downto 0));
                  when 4  => tx_data_n <= x"00";
                  when 5  => tx_data_n <= x"00";
                  when 6  => tx_data_n <= x"00";
                  when 7  => tx_data_n <= x"00";
                  when 8  => tx_data_n <= x"40";
                  when 9  => tx_data_n <= C_UDP;
                  when 10 => tx_data_n <= x"00";
                  when 11 => tx_data_n <= x"00";
                  when 12 => tx_data_n <= G_CLIENT_IP(31 downto 24);
                  when 13 => tx_data_n <= G_CLIENT_IP(23 downto 16);
                  when 14 => tx_data_n <= G_CLIENT_IP(15 downto 8);
                  when 15 => tx_data_n <= G_CLIENT_IP(7  downto 0);
                  when 16 => tx_data_n <= G_SERVER_IP(31 downto 24);
                  when 17 => tx_data_n <= G_SERVER_IP(23 downto 16);
                  when 18 => tx_data_n <= G_SERVER_IP(15 downto 8);
                  when 19 => tx_data_n <= G_SERVER_IP(7  downto 0);
                  when others => tx_data_n <= (others => '0');
               end case;

               if tx_bcnt = to_unsigned(19, tx_bcnt'length) then
                  tx_bcnt_next <= (others => '0');
                  tx_st_next   <= TX_RRQ_UDP;
               else
                  tx_bcnt_next <= tx_bcnt + 1;
               end if;
            end if;

         ------------------------------------------------------------------
         -- TX_RRQ_UDP : UDP header for RRQ
         ------------------------------------------------------------------
         when TX_RRQ_UDP =>
            if tx_tready = '1' then
               tx_valid_n <= '1';
               case to_integer(tx_bcnt) is
                  when 0 => tx_data_n <= std_logic_vector(G_CLIENT_PORT(15 downto 8));
                  when 1 => tx_data_n <= std_logic_vector(G_CLIENT_PORT(7  downto 0));
                  when 2 => tx_data_n <= std_logic_vector(G_SERVER_PORT(15 downto 8));
                  when 3 => tx_data_n <= std_logic_vector(G_SERVER_PORT(7  downto 0));
                  when 4 => tx_data_n <= std_logic_vector(RRQ_UDP_LEN(15 downto 8));
                  when 5 => tx_data_n <= std_logic_vector(RRQ_UDP_LEN(7  downto 0));
                  when 6 => tx_data_n <= x"00";
                  when 7 => tx_data_n <= x"00";
                  when others => tx_data_n <= (others => '0');
               end case;

               if tx_bcnt = to_unsigned(7, tx_bcnt'length) then
                  tx_bcnt_next <= (others => '0');
                  tx_st_next   <= TX_RRQ_TFTP;
               else
                  tx_bcnt_next <= tx_bcnt + 1;
               end if;
            end if;

         ------------------------------------------------------------------
         -- TX_RRQ_TFTP : TFTP RRQ payload
         ------------------------------------------------------------------
         when TX_RRQ_TFTP =>
            if tx_tready = '1' then
               tx_valid_n <= '1';
               idx := to_integer(tx_bcnt);

               if idx = 0 then
                  tx_data_n <= C_TFTP_RRQ(15 downto 8);
               elsif idx = 1 then
                  tx_data_n <= C_TFTP_RRQ(7  downto 0);
               elsif idx >= 2 and idx < (2 + G_FILENAME'length) then
                  tx_data_n <= FNAME_VEC(8*(idx-2+1)-1 downto 8*(idx-2));
               elsif idx = (2 + G_FILENAME'length) then
                  tx_data_n <= x"00";
               elsif idx > (2 + G_FILENAME'length) and
                     idx < (2 + G_FILENAME'length + 1 + G_MODE'length) then
                  tx_data_n <= MODE_VEC(
                     8*(idx - (2 + G_FILENAME'length + 1) + 1)-1
                     downto 8*(idx - (2 + G_FILENAME'length + 1))
                  );
               elsif idx = (2 + G_FILENAME'length + 1 + G_MODE'length) then
                  tx_data_n <= x"00";
               else
                  tx_data_n <= x"00";
               end if;

               if tx_bcnt = to_unsigned(RRQ_PAYLEN-1, tx_bcnt'length) then
                  tx_bcnt_next <= (others => '0');
                  tx_st_next   <= TX_EOF;
                  rrq_sent_n   <= '1';
                  s_rrq_n      <= s_rrq + 1;
               else
                  tx_bcnt_next <= tx_bcnt + 1;
               end if;
            end if;

         ------------------------------------------------------------------
         -- TX_ACK_ETH : Ethernet header for ACK
         ------------------------------------------------------------------
         when TX_ACK_ETH =>
            if tx_tready = '1' then
               tx_valid_n <= '1';
               case to_integer(tx_bcnt) is
                  when 0  => tx_data_n <= ack_dst_mac(47 downto 40);
                  when 1  => tx_data_n <= ack_dst_mac(39 downto 32);
                  when 2  => tx_data_n <= ack_dst_mac(31 downto 24);
                  when 3  => tx_data_n <= ack_dst_mac(23 downto 16);
                  when 4  => tx_data_n <= ack_dst_mac(15 downto 8);
                  when 5  => tx_data_n <= ack_dst_mac(7  downto 0);
                  when 6  => tx_data_n <= G_CLIENT_MAC(47 downto 40);
                  when 7  => tx_data_n <= G_CLIENT_MAC(39 downto 32);
                  when 8  => tx_data_n <= G_CLIENT_MAC(31 downto 24);
                  when 9  => tx_data_n <= G_CLIENT_MAC(23 downto 16);
                  when 10 => tx_data_n <= G_CLIENT_MAC(15 downto 8);
                  when 11 => tx_data_n <= G_CLIENT_MAC(7  downto 0);
                  when 12 => tx_data_n <= C_ETH_IP(15 downto 8);
                  when 13 => tx_data_n <= C_ETH_IP(7  downto 0);
                  when others => tx_data_n <= (others => '0');
               end case;

               if tx_bcnt = to_unsigned(13, tx_bcnt'length) then
                  tx_bcnt_next <= (others => '0');
                  tx_st_next   <= TX_ACK_IP;
               else
                  tx_bcnt_next <= tx_bcnt + 1;
               end if;
            end if;

         ------------------------------------------------------------------
         -- TX_ACK_IP : IPv4 header for ACK
         ------------------------------------------------------------------
         when TX_ACK_IP =>
            if tx_tready = '1' then
               tx_valid_n <= '1';
               case to_integer(tx_bcnt) is
                  when 0  => tx_data_n <= x"45";
                  when 1  => tx_data_n <= x"00";
                  when 2  => tx_data_n <= std_logic_vector(ACK_IP_LEN(15 downto 8));
                  when 3  => tx_data_n <= std_logic_vector(ACK_IP_LEN(7  downto 0));
                  when 4  => tx_data_n <= x"00";
                  when 5  => tx_data_n <= x"00";
                  when 6  => tx_data_n <= x"00";
                  when 7  => tx_data_n <= x"00";
                  when 8  => tx_data_n <= x"40";
                  when 9  => tx_data_n <= C_UDP;
                  when 10 => tx_data_n <= x"00";
                  when 11 => tx_data_n <= x"00";
                  when 12 => tx_data_n <= G_CLIENT_IP(31 downto 24);
                  when 13 => tx_data_n <= G_CLIENT_IP(23 downto 16);
                  when 14 => tx_data_n <= G_CLIENT_IP(15 downto 8);
                  when 15 => tx_data_n <= G_CLIENT_IP(7  downto 0);
                  when 16 => tx_data_n <= ack_dst_ip(31 downto 24);
                  when 17 => tx_data_n <= ack_dst_ip(23 downto 16);
                  when 18 => tx_data_n <= ack_dst_ip(15 downto 8);
                  when 19 => tx_data_n <= ack_dst_ip(7  downto 0);
                  when others => tx_data_n <= (others => '0');
               end case;

               if tx_bcnt = to_unsigned(19, tx_bcnt'length) then
                  tx_bcnt_next <= (others => '0');
                  tx_st_next   <= TX_ACK_UDP;
               else
                  tx_bcnt_next <= tx_bcnt + 1;
               end if;
            end if;

         ------------------------------------------------------------------
         -- TX_ACK_UDP : UDP header for ACK
         ------------------------------------------------------------------
         when TX_ACK_UDP =>
            if tx_tready = '1' then
               tx_valid_n <= '1';
               case to_integer(tx_bcnt) is
                  when 0 => tx_data_n <= std_logic_vector(G_CLIENT_PORT(15 downto 8));
                  when 1 => tx_data_n <= std_logic_vector(G_CLIENT_PORT(7  downto 0));
                  when 2 => tx_data_n <= std_logic_vector(ack_dst_port(15 downto 8));
                  when 3 => tx_data_n <= std_logic_vector(ack_dst_port(7  downto 0));
                  when 4 => tx_data_n <= std_logic_vector(ACK_UDP_LEN(15 downto 8));
                  when 5 => tx_data_n <= std_logic_vector(ACK_UDP_LEN(7  downto 0));
                  when 6 => tx_data_n <= x"00";
                  when 7 => tx_data_n <= x"00";
                  when others => tx_data_n <= (others => '0');
               end case;

               if tx_bcnt = to_unsigned(7, tx_bcnt'length) then
                  tx_bcnt_next <= (others => '0');
                  tx_st_next   <= TX_ACK_TFTP;
               else
                  tx_bcnt_next <= tx_bcnt + 1;
               end if;
            end if;

         ------------------------------------------------------------------
         -- TX_ACK_TFTP : TFTP ACK payload
         ------------------------------------------------------------------
         when TX_ACK_TFTP =>
            if tx_tready = '1' then
               tx_valid_n <= '1';
               case to_integer(tx_bcnt) is
                  when 0 => tx_data_n <= C_TFTP_ACK(15 downto 8);
                  when 1 => tx_data_n <= C_TFTP_ACK(7  downto 0);
                  when 2 => tx_data_n <= ack_block(15 downto 8);
                  when 3 => tx_data_n <= ack_block(7  downto 0);
                  when others => tx_data_n <= x"00";
               end case;

               if tx_bcnt = to_unsigned(3, tx_bcnt'length) then
                  tx_bcnt_next <= (others => '0');
                  tx_st_next   <= TX_EOF;
                  s_ack_n      <= s_ack + 1;
               else
                  tx_bcnt_next <= tx_bcnt + 1;
               end if;
            end if;

         ------------------------------------------------------------------
         -- TX_EOF : end-of-frame pulse (tlast only)
         ------------------------------------------------------------------
         when TX_EOF =>
            if tx_tready = '1' then
               tx_valid_n <= '0';
               tx_last_n  <= '1';
               tx_st_next <= TX_IDLE;
            end if;
      end case;

      -- stop marking "active" once we reached the maximum block count
      if blocks_seen >= to_unsigned(G_MAX_BLOCKS, blocks_seen'length) then
         active_n <= '0';
      end if;
   end process;

end architecture;
