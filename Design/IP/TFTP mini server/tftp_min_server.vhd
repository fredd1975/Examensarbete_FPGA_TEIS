-- ==========================================================================
-- Company        :  TEIS AB
-- Engineer       :  Freddy Avaria
-- Project        :  FPGA & TEIS Thesis - PXE-Based Remote Reboot System
-- Create Date    :  2025 Nov. 14
-- Design name    :  tftp_min_server.vhd
-- Target device  :  Intel MAX 10 (DE10-Lite)
-- Tools          :  Quartus 18.1 / ModelSim (VHDL-93)
--
-- Description:
--   Very small TFTP "server" for validation / lab use.
--
--   Simplified behaviour:
--     * Ignores RX contents completely (RRQ parsing removed).
--     * As soon as cfg_enable='1' and reset is released, it transmits a
--       fixed number of TFTP DATA frames (opcode = 3, block = 1..8).
--     * Each frame has:
--         - Ethernet header (dst/src/cfg_mac)
--         - IPv4 header (no checksum, proto=UDP)
--         - UDP header  (ports from generics, checksum=0)
--         - TFTP header (opcode + block#, no payload)
--     * After sending all blocks it idles with stat_active = '0'.
--
--   Notes:
--     - The exact field positions (Ethertype, IP proto, UDP header, TFTP
--       opcode/block) match the offsets used by tftp_client_fsm and the
--       accelerator parser:
--         byte 12..13 : Ethertype (0x0800)
--         byte 23     : IP protocol (0x11)
--         byte 34..41 : UDP header
--         byte 42..43 : TFTP opcode
--         byte 44..45 : TFTP block
-- ==========================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity tftp_min_server is
   generic (
      G_BLKSZ_BYTES : natural := 512;                        -- unused, kept for interface compatibility
      G_SERVER_PORT : unsigned(15 downto 0) := to_unsigned(40000, 16); -- UDP source port
      G_CLIENT_PORT : unsigned(15 downto 0) := to_unsigned(40001, 16); -- UDP destination port
      G_IMG_SIZE    : natural := 4096                        -- unused, kept for compatibility
   );
   port (
      -- Clock / reset
      clk   : in  std_logic;
      rst_n : in  std_logic;

      -- RX from client (ignored in this simplified implementation)
      rx_tvalid : in  std_logic;
      rx_tlast  : in  std_logic;
      rx_tdata  : in  std_logic_vector(7 downto 0);

      -- TX to client (and accelerator tap)
      tx_tvalid : out std_logic;
      tx_tready : in  std_logic;
      tx_tlast  : out std_logic;
      tx_tdata  : out std_logic_vector(7 downto 0);

      -- Identity / configuration
      cfg_mac    : in  std_logic_vector(47 downto 0);
      cfg_ip     : in  std_logic_vector(31 downto 0);
      cfg_enable : in  std_logic;

      -- Image ROM (not actually used by this minimal server)
      img_addr : out unsigned(31 downto 0);
      img_rd   : out std_logic;
      img_q    : in  std_logic_vector(7 downto 0);

      -- Statistics
      stat_rrq     : out unsigned(31 downto 0);
      stat_data_tx : out unsigned(31 downto 0);
      stat_ack_rx  : out unsigned(31 downto 0);
      stat_active  : out std_logic
   );
end entity;

architecture rtl of tftp_min_server is

   --------------------------------------------------------------------------
   -- Types / constants
   --------------------------------------------------------------------------
   type tx_state_t is (
      TX_IDLE,
      TX_SEND,
      TX_GAP
   );

   constant C_ETH_IP    : std_logic_vector(15 downto 0) := x"0800";
   constant C_IP_PROTO  : std_logic_vector(7 downto 0)  := x"11";     -- UDP
   constant C_TFTP_DATA : std_logic_vector(15 downto 0) := x"0003";

   -- We send 8 DATA blocks (1..8) so it matches the client G_MAX_BLOCKS=8
   constant C_MAX_BLOCKS : natural := 8;

   --------------------------------------------------------------------------
   -- Registers
   --------------------------------------------------------------------------
   signal tx_state_r   : tx_state_t := TX_IDLE;
   signal tx_state_n   : tx_state_t;

   signal tx_bcnt_r    : unsigned(7 downto 0) := (others => '0'); -- byte index 0..45
   signal tx_bcnt_n    : unsigned(7 downto 0);

   signal block_r      : unsigned(15 downto 0) := (others => '0'); -- current DATA block number
   signal block_n      : unsigned(15 downto 0);

   signal blocks_sent_r : unsigned(15 downto 0) := (others => '0');
   signal blocks_sent_n : unsigned(15 downto 0);

   signal tx_valid_r   : std_logic := '0';
   signal tx_valid_n   : std_logic;
   signal tx_last_r    : std_logic := '0';
   signal tx_last_n    : std_logic;
   signal tx_data_r    : std_logic_vector(7 downto 0) := (others => '0');
   signal tx_data_n    : std_logic_vector(7 downto 0);

   signal stat_rrq_r     : unsigned(31 downto 0) := (others => '0');
   signal stat_rrq_n     : unsigned(31 downto 0);
   signal stat_data_r    : unsigned(31 downto 0) := (others => '0');
   signal stat_data_n    : unsigned(31 downto 0);
   signal stat_ack_r     : unsigned(31 downto 0) := (others => '0'); -- stays 0
   signal stat_ack_n     : unsigned(31 downto 0);
   signal stat_active_r  : std_logic := '0';
   signal stat_active_n  : std_logic;

begin

   --------------------------------------------------------------------------
   -- Outputs
   --------------------------------------------------------------------------
   tx_tvalid   <= tx_valid_r;
   tx_tlast    <= tx_last_r;
   tx_tdata    <= tx_data_r;

   img_addr    <= (others => '0');
   img_rd      <= '0';

   stat_rrq     <= stat_rrq_r;
   stat_data_tx <= stat_data_r;
   stat_ack_rx  <= stat_ack_r;
   stat_active  <= stat_active_r;

   --------------------------------------------------------------------------
   -- Sequential part
   --------------------------------------------------------------------------
   seq_proc : process(clk, rst_n)
   begin
      if rst_n = '0' then
         tx_state_r    <= TX_IDLE;
         tx_bcnt_r     <= (others => '0');
         block_r       <= (others => '0');
         blocks_sent_r <= (others => '0');
         tx_valid_r    <= '0';
         tx_last_r     <= '0';
         tx_data_r     <= (others => '0');
         stat_rrq_r    <= (others => '0');
         stat_data_r   <= (others => '0');
         stat_ack_r    <= (others => '0');
         stat_active_r <= '0';
      elsif rising_edge(clk) then
         tx_state_r    <= tx_state_n;
         tx_bcnt_r     <= tx_bcnt_n;
         block_r       <= block_n;
         blocks_sent_r <= blocks_sent_n;
         tx_valid_r    <= tx_valid_n;
         tx_last_r     <= tx_last_n;
         tx_data_r     <= tx_data_n;
         stat_rrq_r    <= stat_rrq_n;
         stat_data_r   <= stat_data_n;
         stat_ack_r    <= stat_ack_n;
         stat_active_r <= stat_active_n;
      end if;
   end process;

   --------------------------------------------------------------------------
   -- Combinational TX FSM
   --------------------------------------------------------------------------
   comb_proc : process(tx_state_r, tx_bcnt_r, block_r, blocks_sent_r,
                       tx_valid_r, tx_last_r, tx_data_r,
                       stat_rrq_r, stat_data_r, stat_ack_r, stat_active_r,
                       cfg_enable, tx_tready, cfg_mac, cfg_ip)
      variable idx : integer;
   begin
      -- Defaults
      tx_state_n    <= tx_state_r;
      tx_bcnt_n     <= tx_bcnt_r;
      block_n       <= block_r;
      blocks_sent_n <= blocks_sent_r;
      tx_valid_n    <= '0';
      tx_last_n     <= '0';
      tx_data_n     <= (others => '0');
      stat_rrq_n    <= stat_rrq_r;
      stat_data_n   <= stat_data_r;
      stat_ack_n    <= stat_ack_r;
      stat_active_n <= stat_active_r;

      case tx_state_r is

         -------------------------------------------------------------------
         -- TX_IDLE
         -- Wait for cfg_enable, then start streaming DATA frames.
         -------------------------------------------------------------------
         when TX_IDLE =>
            tx_bcnt_n     <= (others => '0');
            tx_valid_n    <= '0';
            tx_last_n     <= '0';
            stat_active_n <= '0';

            if cfg_enable = '1' then
               -- Start a new "session" if we still have blocks to send
               if unsigned(blocks_sent_r) < to_unsigned(C_MAX_BLOCKS, blocks_sent_r'length) then
                  tx_state_n    <= TX_SEND;
                  tx_bcnt_n     <= (others => '0');
                  block_n       <= to_unsigned(1, block_r'length) + blocks_sent_r(15 downto 0);
                  stat_active_n <= '1';
                  stat_rrq_n    <= to_unsigned(1, stat_rrq_r'length); -- pretend one RRQ seen
               end if;
            end if;

         -------------------------------------------------------------------
         -- TX_SEND
         -- Emit one Ethernet+IPv4+UDP+TFTP DATA frame, 46 bytes total:
         --   [ 0..13] : Ethernet
         --   [14..33] : IPv4
         --   [34..41] : UDP
         --   [42..45] : TFTP (opcode + block)
         -------------------------------------------------------------------
         when TX_SEND =>
            if tx_tready = '1' then
               tx_valid_n <= '1';
               tx_last_n  <= '0';
               idx := to_integer(tx_bcnt_r);

               -- Byte selection
               if idx >= 0 and idx <= 13 then
                  -- Ethernet header
                  case idx is
                     when 0  => tx_data_n <= cfg_mac(47 downto 40); -- dst MAC (reuse cfg_mac)
                     when 1  => tx_data_n <= cfg_mac(39 downto 32);
                     when 2  => tx_data_n <= cfg_mac(31 downto 24);
                     when 3  => tx_data_n <= cfg_mac(23 downto 16);
                     when 4  => tx_data_n <= cfg_mac(15 downto 8);
                     when 5  => tx_data_n <= cfg_mac(7  downto 0);
                     when 6  => tx_data_n <= cfg_mac(47 downto 40); -- src MAC (same for lab)
                     when 7  => tx_data_n <= cfg_mac(39 downto 32);
                     when 8  => tx_data_n <= cfg_mac(31 downto 24);
                     when 9  => tx_data_n <= cfg_mac(23 downto 16);
                     when 10 => tx_data_n <= cfg_mac(15 downto 8);
                     when 11 => tx_data_n <= cfg_mac(7  downto 0);
                     when 12 => tx_data_n <= C_ETH_IP(15 downto 8);
                     when 13 => tx_data_n <= C_ETH_IP(7  downto 0);
                     when others => tx_data_n <= (others => '0');
                  end case;

               elsif idx >= 14 and idx <= 33 then
                  -- IPv4 header (20 bytes)
                  case idx-14 is
                     when 0  => tx_data_n <= x"45";                           -- ver/IHL
                     when 1  => tx_data_n <= x"00";                           -- TOS
                     when 2  => tx_data_n <= x"00";                           -- total len = 32
                     when 3  => tx_data_n <= x"20";
                     when 4  => tx_data_n <= x"00";                           -- ID
                     when 5  => tx_data_n <= x"00";
                     when 6  => tx_data_n <= x"00";                           -- flags/frag
                     when 7  => tx_data_n <= x"00";
                     when 8  => tx_data_n <= x"40";                           -- TTL
                     when 9  => tx_data_n <= C_IP_PROTO;                      -- UDP
                     when 10 => tx_data_n <= x"00";                           -- checksum = 0
                     when 11 => tx_data_n <= x"00";
                     when 12 => tx_data_n <= cfg_ip(31 downto 24);            -- src IP
                     when 13 => tx_data_n <= cfg_ip(23 downto 16);
                     when 14 => tx_data_n <= cfg_ip(15 downto 8);
                     when 15 => tx_data_n <= cfg_ip(7  downto 0);
                     when 16 => tx_data_n <= cfg_ip(31 downto 24);            -- dst IP (reuse)
                     when 17 => tx_data_n <= cfg_ip(23 downto 16);
                     when 18 => tx_data_n <= cfg_ip(15 downto 8);
                     when 19 => tx_data_n <= cfg_ip(7  downto 0);
                     when others => tx_data_n <= (others => '0');
                  end case;

               elsif idx >= 34 and idx <= 41 then
                  -- UDP header (8 bytes) - ports from generics, checksum 0
                  case idx-34 is
                     when 0 => tx_data_n <= std_logic_vector(G_SERVER_PORT(15 downto 8)); -- src port hi
                     when 1 => tx_data_n <= std_logic_vector(G_SERVER_PORT(7  downto 0)); -- src port lo
                     when 2 => tx_data_n <= std_logic_vector(G_CLIENT_PORT(15 downto 8)); -- dst port hi
                     when 3 => tx_data_n <= std_logic_vector(G_CLIENT_PORT(7  downto 0)); -- dst port lo
                     when 4 => tx_data_n <= x"00";                                       -- length 12 (8+4)
                     when 5 => tx_data_n <= x"0C";
                     when 6 => tx_data_n <= x"00";                                       -- checksum = 0
                     when 7 => tx_data_n <= x"00";
                     when others => tx_data_n <= (others => '0');
                  end case;

               else
                  -- TFTP header (opcode + block)
                  case idx-42 is
                     when 0 => tx_data_n <= C_TFTP_DATA(15 downto 8);               -- opcode hi
                     when 1 => tx_data_n <= C_TFTP_DATA(7  downto 0);               -- opcode lo
                     when 2 => tx_data_n <= std_logic_vector(block_r(15 downto 8)); -- block hi
                     when 3 => tx_data_n <= std_logic_vector(block_r(7  downto 0)); -- block lo
                     when others => tx_data_n <= x"00";
                  end case;
               end if;

               -- End of frame at byte index 45
               if tx_bcnt_r = to_unsigned(45, tx_bcnt_r'length) then
                  tx_bcnt_n     <= (others => '0');
                  tx_last_n     <= '1';
                  tx_state_n    <= TX_GAP;
                  blocks_sent_n <= blocks_sent_r + 1;
                  stat_data_n   <= stat_data_r + 1;
               else
                  tx_bcnt_n <= tx_bcnt_r + 1;
               end if;
            end if;

         -------------------------------------------------------------------
         -- TX_GAP
         -- One idle cycle between frames; then either start next frame or
         -- return to IDLE when all blocks have been sent.
         -------------------------------------------------------------------
         when TX_GAP =>
            tx_valid_n <= '0';
            tx_last_n  <= '0';
            tx_bcnt_n  <= (others => '0');

            if unsigned(blocks_sent_r) < to_unsigned(C_MAX_BLOCKS, blocks_sent_r'length) then
               if cfg_enable = '1' then
                  tx_state_n <= TX_SEND;
                  block_n    <= block_r + 1;
               else
                  tx_state_n    <= TX_IDLE;
                  stat_active_n <= '0';
               end if;
            else
               tx_state_n    <= TX_IDLE;
               stat_active_n <= '0';
            end if;

      end case;
   end process;

end architecture;
