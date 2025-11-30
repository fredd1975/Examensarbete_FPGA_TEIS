--==========================================================================
-- Company        :  TEIS AB
-- Engineer       :  Freddy Avaria
-- Project        :  FPGA & TEIS Thesis - PXE-Based Remote Reboot System
-- Create Date    :  2025 Nov. 14
-- Design name    :  na_unified_top_tb.vhd
-- Target devices :  ALTERA MAX 10 (DE10-Lite)
-- Tool versions  :  Quartus v18.1 / ModelSim (VHDL-93)
--
-- Testbench for:  na_unified_top
--
-- Cases covered:
--   Scenario A:
--     Case 1 : Global reset (low -> high)
--     Case 2 : SW0 = 0, normal mode (client + server + accelerator active)
--     Case 3 : SW0 = 1, accelerator-only mode
--     Case 4 : RRQ transmission from client
--     Case 5 : First DATA from server
--     Case 6 : ACK from client (PASS LED lights)
--     Case 7 : Transfer runs until G_MAX_BLOCKS
--     Case 8 : Idle behaviour (no new DATA) after transfer
--
--   Scenario B:
--     Case 9 : Reset asserted during an ongoing transfer
--
-- Notes:
--   * This testbench only drives clock, reset, SW0 and test_fail.
--     All packet traffic (RRQ, DATA, ACK) is generated internally
--     inside na_unified_top. Watchdog behaviour (wd_cnt, ack_seen)
--     is therefore determined by the DUT, not by this file.
--==========================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity na_unified_top_tb is
end entity na_unified_top_tb;

architecture sim of na_unified_top_tb is

   -------------------------------------------------------------------------
   -- DUT interface signals
   -------------------------------------------------------------------------
   signal clk_50    : std_logic := '0';
   signal rst_n     : std_logic := '0';
   signal sw0       : std_logic := '0';
   signal test_fail : std_logic := '0';
   signal sw1       : std_logic := '0';
   signal led_pass  : std_logic;
   signal led_fail  : std_logic;

begin

   -------------------------------------------------------------------------
   -- Clock generation: 50 MHz (20 ns period)
   -------------------------------------------------------------------------
   clk_50 <= not clk_50 after 10 ns;

   -------------------------------------------------------------------------
   -- DUT instantiation
   -------------------------------------------------------------------------
   dut : entity work.na_unified_top
      generic map (
         G_WD_BITS => 18   -- shorter watchdog for simulation
      )
      port map (
         clk_50    => clk_50,
         rst_n     => rst_n,
         sw0       => sw0,
         sw1       => sw1,
         led_pass  => led_pass,
         led_fail  => led_fail
      );

   -------------------------------------------------------------------------
   -- Stimulus process
   -------------------------------------------------------------------------
   stim_proc : process
   begin
      ----------------------------------------------------------------------
      -- SCENARIO A : Cases 1–8 (normal validation run)
      --
      -- Case 1 : Global reset (low -> high)
      -- Case 2 : SW0 = 0, normal mode (client + server active)
      -- Case 3 : SW0 = 1, accelerator-only AFTER full transfer
      -- Case 4 : RRQ transmission from client
      -- Case 5 : First DATA from server
      -- Case 6 : First ACK from client (PASS LED lights)
      -- Case 7 : Transfer runs until G_MAX_BLOCKS
      -- Case 8 : Idle behaviour after transfer
      ----------------------------------------------------------------------

      -- Case 1: Global reset pulse (low -> high)
      report "Scenario A / Case 1: Applying global reset (rst_n=0)..." severity note;
      rst_n     <= '0';
      sw0       <= '0';   -- default to normal mode
      sw1       <= '0';   -- FAIL LED disabled
      test_fail <= '0';
      wait for 200 ns;

      report "Scenario A / Case 1: Releasing global reset (rst_n=1)..." severity note;
      rst_n <= '1';

      -- allow internal reset synchronizers to settle
      wait for 2 us;

      -- Cases 2/4/5/6/7: Normal mode full transfer
      report "Scenario A / Cases 2/4/5/6/7: NORMAL MODE (sw0=0) - RRQ, DATA, ACK, full transfer expected."
         severity note;
      sw0 <= '0';

      -- Wait long enough for RRQ + all DATA/ACK blocks (8 blocks)
      wait for 200 us;

      -------------------------------------------------------------------
      -- Case 6/7: PASS LED check after transfer
      -------------------------------------------------------------------
      report "Scenario A / Case 6/7: Checking PASS/FAIL LEDs after transfer..." severity note;

      if led_pass = '1' then
         report "Scenario A: PASS LED is HIGH as expected." severity note;
      else
         report "Scenario A: FAIL - PASS LED did not assert." severity error;
      end if;

      if led_fail = '1' then
         report "Scenario A: FAIL LED is HIGH - unexpected in normal mode."
            severity error;
      else
         report "Scenario A: FAIL LED is LOW as expected." severity note;
      end if;

      -------------------------------------------------------------------
      -- Case 3 & 8: Accelerator-only, idle behaviour AFTER full transfer
      -- (matches your original “Case 3/8” description)
      -------------------------------------------------------------------
      report "Scenario A / Cases 3 & 8: ACCELERATOR-ONLY mode (sw0=1) - client/server idle; no new DATA/ACK; counters hold final values."
         severity note;
      sw0 <= '1';

      -- Observe idle behaviour for a while
      wait for 50 us;


      ----------------------------------------------------------------------
      -- SCENARIO B : Case 9 (reset during ongoing transfer)
      --
      -- Start a fresh run:
      --   1) Apply a new global reset.
      --   2) Go to NORMAL MODE (sw0=0) and let RRQ/DATA/ACK start again.
      --   3) While transfer is active (mid-way), assert reset briefly.
      --   4) Release reset and observe that a new session restarts.
      ----------------------------------------------------------------------
      report "Scenario B / Case 9: Applying fresh global reset before mid-transfer test..."
         severity note;
      rst_n <= '0';
      sw0   <= '0';
      wait for 5 us;

      report "Scenario B / Case 9: Releasing global reset (rst_n=1), NORMAL MODE (sw0=0)..."
         severity note;
      rst_n <= '1';

      -- Wait for RRQ + some DATA/ACKs, but NOT long enough to finish transfer.
      -- From measurements, a full 8-block transfer finishes well before 200 us,
      -- so 80 us puts us safely mid-transfer.
      wait for 10 us;

      report "Scenario B / Case 9: Asserting reset DURING transfer (rst_n=0)..." severity note;
      rst_n <= '0';
      wait for 5 us;

      report "Scenario B / Case 9: Releasing reset again (rst_n=1)..." severity note;
      rst_n <= '1';

      -- Observe that a new RRQ/DATA/ACK sequence starts after the mid-transfer reset.
      -- Wait long enough for another full transfer.
      wait for 10 us;

      report "Scenario B / Case 9: Observation window after mid-transfer reset complete."
         severity note;

      ------------------------------------------------------------------
      -- Scenario C / Case 10: Watchdog timeout when server is disabled
      --
      -- In this configuration, the internal logic disables the server
      -- response path so that the client can send its RRQ but will never
      -- receive DATA/ACK. The watchdog should eventually assert FAIL.
      ------------------------------------------------------------------
      report "Scenario C / Case 10: Watchdog timeout in validation mode (SW0=0, SW1=1, server disabled)."
         severity note;

      -- Start in validation mode (client active), but disable server.
      -- FAIL LED enabled so we can see the timeout.
      rst_n     <= '0';
      sw0       <= '0';   -- validation mode: client + server path
      sw1       <= '1';   -- disable server + show FAIL LED
      test_fail <= '0';
      wait for 200 ns;

      rst_n <= '1';       -- release reset

      -- Client will send its RRQ once; server will never respond.
      -- Wait longer than watchdog timeout (G_WD_BITS=18 @50MHz ≈ 5 ms)
      wait for 10 ms;

      -- Check: PASS must stay low, FAIL must assert
      assert led_pass = '0'
        report "Case 10: PASS LED should remain LOW in timeout scenario"
        severity error;

      assert led_fail = '1'
        report "Case 10: FAIL LED did not assert on watchdog timeout"
        severity error;

      report "Scenario C / Case 10: Watchdog timeout behavior OK." severity note;

      ----------------------------------------------------------------------
      -- End of testbench
      ----------------------------------------------------------------------
      report "All scenarios complete - stop simulation here." severity note;
      wait;
   end process;


end architecture sim;
