transcript on
onerror {resume}

# --- unload any previous design so cd is allowed
catch { quit -sim }

# ---- Derive paths ---------------------------------------------------------
set SIMDIR  [file normalize [file dirname [info script]]]
set ROOT    [file normalize [file join $SIMDIR ".." ".."]]
cd $SIMDIR

# ---- Source paths ---------------------------------------------------------
set PKG_NET      [file join $ROOT net_pkg.vhd]
set PKG_IPCKSUM  [file join $ROOT ip_cksum_pkg.vhd]
set FIFO_VHD     [file join $ROOT stream_fifo.vhd]
set CSR_VHD      [file join $ROOT csr_bank.vhd]
set PARSER_VHD   [file join $ROOT parser_ipv4_udp_tftp.vhd]
set TFTP_SERVER  [file join $ROOT tftp_min_server.vhd]
set TFTP_CLIENT  [file join $ROOT tftp_client_fsm.vhd]
set BYTE_ROM     [file join $ROOT simple_byte_rom.vhd]
set MON_VHD      [file join $ROOT monitor_proc.vhd]
set ACCEL_TOP    [file join $ROOT tftp_accel_top.vhd]
set TOP_VHD      [file join $ROOT na_unified_top.vhd]
set TB_VHD       [file join $ROOT na_unified_top_tb.vhd]

# ---- Fresh work lib -------------------------------------------------------
set RTLWORK [file normalize [file join $SIMDIR rtl_work]]
if {[file exists $RTLWORK]} { catch { vdel -lib $RTLWORK -all } }
vlib $RTLWORK
vmap work $RTLWORK

# ---- Helper ---------------------------------------------------------------
proc vcom_ok {path} {
    if {![file exists $path]} {
        puts "## WARN: Missing: $path"
        return
    }
    puts "## vcom: $path"
    vcom -93 -quiet -work work $path
}

# ---- Compile (in dependency order) ---------------------------------------
vcom_ok $PKG_NET
vcom_ok $PKG_IPCKSUM
vcom_ok $FIFO_VHD
vcom_ok $CSR_VHD
vcom_ok $PARSER_VHD
vcom_ok $TFTP_SERVER
vcom_ok $TFTP_CLIENT
vcom_ok $BYTE_ROM
vcom_ok $MON_VHD
vcom_ok $ACCEL_TOP
vcom_ok $TOP_VHD
vcom_ok $TB_VHD

# ---- Load simulation: top-level TB ----------------------------------------
if {[catch { vsim -t 1ps work.na_unified_top_tb } emsg]} {
    puts "## ERROR: vsim failed: $emsg"
    return
}

# ---- Waves ---------------------------------------------------------------
view wave
catch { wave clear }
catch { wave delete * }

set TB   sim:/na_unified_top_tb
set DUT  $TB/dut
set CLI  $DUT/u_client
set SRV  $DUT/u_server
set ACC  $DUT/u_accel
set PAR  $ACC/u_parser

# ---------------------------------------------------------------------------
# Waves for Test Protocol Cases 1–9
# ---------------------------------------------------------------------------

add wave -divider {Clock / Reset}
add wave -label clk_50   $TB/clk_50
add wave -label rst_n    $TB/rst_n

add wave -divider {Mode Switch}
add wave -label sw0      $TB/sw0

# ---------------------------------------------------------------------------
# Client <-> Server stream (c_* bus)
# ---------------------------------------------------------------------------
add wave -divider {Client TX to Server (c_*)}
catch { add wave -label c_tx_valid $DUT/c_tx_valid }
catch { add wave -label c_tx_ready $DUT/c_tx_ready }
catch { add wave -label c_tx_last  $DUT/c_tx_last }
catch { add wave -label c_tx_valid_dbg $DUT/c_tx_valid_dbg }
catch { add wave -radix hex -label c_tx_data  $DUT/c_tx_data }

# ---------------------------------------------------------------------------
# Server TX to Accelerator (s_* bus)
# ---------------------------------------------------------------------------
add wave -divider {Server TX to Accelerator (s_*)}
catch { add wave -label s_tx_valid $DUT/s_tx_valid }
catch { add wave -label s_tx_ready $DUT/s_tx_ready }
catch { add wave -label s_tx_last  $DUT/s_tx_last }
catch { add wave -radix hex -label s_tx_data  $DUT/s_tx_data }

# ---------------------------------------------------------------------------
# Client FSM (RRQ / ACK generator)
# ---------------------------------------------------------------------------
add wave -divider {Client FSM}
catch { add wave -radix hex      -label dbg_tx_state    $CLI/dbg_tx_state }
catch { add wave -radix unsigned -label stat_rrq_sent   $CLI/stat_rrq_sent }
catch { add wave -radix unsigned -label stat_data_seen  $CLI/stat_data_seen }
catch { add wave -radix unsigned -label stat_ack_sent   $CLI/stat_ack_sent }
catch { add wave                  -label stat_active    $CLI/stat_active }

# ---------------------------------------------------------------------------
# Accelerator Parser (what the accelerator "sees")
# ---------------------------------------------------------------------------
add wave -divider {Accelerator Parser}
catch { add wave -radix symbolic -label parser_st    $PAR/st }
catch { add wave -radix hex      -label tftp_opcode  $PAR/tftp_opcode }
catch { add wave -radix unsigned -label tftp_block   $PAR/tftp_block }
catch { add wave -radix unsigned -label bcnt         $PAR/bcnt }

# ---------------------------------------------------------------------------
# Client RX debug (inside tftp_client_fsm)
# ---------------------------------------------------------------------------
add wave -divider {Client RX debug}
add wave -label r_in_frm      $CLI/r_in_frm
add wave -label r_bcnt        -radix unsigned $CLI/r_bcnt
add wave -label r_ethertype   -radix hex      $CLI/r_ethertype
add wave -label r_ip_proto    -radix hex      $CLI/r_ip_proto
add wave -label r_udp_src     -radix hex      $CLI/r_udp_src_port
add wave -label r_udp_dst     -radix hex      $CLI/r_udp_dst_port
add wave -label r_tftp_opcode -radix hex      $CLI/r_tftp_opcode
add wave -label r_tftp_block  -radix unsigned $CLI/r_tftp_block
add wave -label blocks_seen   -radix unsigned $CLI/blocks_seen
add wave -label s_data_rx     -radix unsigned $CLI/s_data
add wave -label ack_pending   $CLI/ack_pending

# ---------------------------------------------------------------------------
# Client TX debug (ACK generator)
# ---------------------------------------------------------------------------
add wave -divider {Client TX debug}
add wave -label tx_st         -radix symbolic $CLI/tx_st
add wave -label tx_bcnt       -radix unsigned $CLI/tx_bcnt
add wave -label tx_valid_r    $CLI/tx_valid_r
add wave -label tx_last_r     $CLI/tx_last_r
add wave -label ack_go        $CLI/ack_go
add wave -label s_ack         -radix unsigned $CLI/s_ack

# ---------------------------------------------------------------------------
# LED path (top-level PASS/FAIL logic)
# ---------------------------------------------------------------------------
add wave -divider {LED path}
add wave -label stat_rrq_c    -radix unsigned $DUT/stat_rrq_c
add wave -label stat_data_c   -radix unsigned $DUT/stat_data_c
add wave -label stat_ack_c    -radix unsigned $DUT/stat_ack_c
add wave -label led_pass      $DUT/led_pass
add wave -label led_fail      $DUT/led_fail
add wave -label mode_accel    $DUT/mode_accel_only
add wave -label ack_seen      $DUT/ack_seen
add wave -label wd_cnt        -radix unsigned $DUT/wd_cnt
add wave -label pass_l        $DUT/pass_l
add wave -label fail_l        $DUT/fail_l
add wave -label rst_n_sync    $DUT/rst_n_sync

# ---------------------------------------------------------------------------
# Top-Level Sanity (client counters mirrored in top)
# ---------------------------------------------------------------------------
add wave -divider {Client counters}
catch { add wave -radix unsigned -label stat_rrq_c    $DUT/stat_rrq_c }
catch { add wave -radix unsigned -label stat_data_c   $DUT/stat_data_c }
catch { add wave -radix unsigned -label stat_ack_c    $DUT/stat_ack_c }
catch { add wave                  -label stat_active_c $DUT/stat_active_c }

# ---------------------------------------------------------------------------
# Board LEDs (validation verdict)
# ---------------------------------------------------------------------------
add wave -divider {Board LEDs}
catch { add wave -label led_pass $DUT/led_pass }
catch { add wave -label led_fail $DUT/led_fail }

update
radix -hex
WaveRestoreZoom {0 ps} {300 us}

# ---- Run ------------------------------------------------------------------
run 300 us

# ---- Run ------------------------------------------------------------------
# run 300 us
