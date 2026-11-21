// SPDX-License-Identifier: GPL-2.0-or-later
// Single-MID player for the SC-55 MiSTer core (Phase 1).
//
// Two modules:
//   mid_loader  - stores a downloaded MID file (CONF_STR "F2,MID", Main
//                 ioctl_index 2) in the free wave_ddr DDR3 window and
//                 validates its SMF header when the download finishes.
//   midi_player - SMF format 0/1 parser + scheduler. Reads the stored file
//                 back through wave_ddr's player read port and emits bytes
//                 through midi_inject at MIDI baud spacing (one byte per
//                 16000 clocks at 50 MHz), only while the core is running
//                 and the MCU receive ring buffer is not full.
//
// Design notes (see also docs/plan.md M5):
// * Downloads are routed by ioctl_index[5:0] (Main menu.cpp: an F entry's
//   explicit digit, else its position among the F entries; the extension
//   index rides in bits [7:6]). F1/ROM = 1, F2/MID = 2. The ROM loader is
//   gated to index 1 and the MID loader to index 2, so a MID download can
//   never clobber a ROM pack and vice versa.
// * A MID download never resets the core: rom_loader's `loading` (which
//   holds the core in reset through the sequencer) is driven by the ROM
//   download only. The player aborts if a MID download starts while it is
//   playing, so the loader owns the DDR3 window exclusively - but the
//   DDR3 port itself is shared with the PCM/player reads, so the loader
//   buffers bytes in a small FIFO and holds each write until wave_ddr
//   accepts it. Main sends blind and never polls ioctl_wait, so a
//   1-clock write pulse would be dropped whenever the port was busy
//   with a read; that is what lost bytes on a second MID load (the file
//   landed corrupt in DDR3 while the header check still passed).
// * MID storage is the free wave_ddr window 0x440000..0x7FFFFF (waves end
//   at 0x400000, ROM2's upper half at 0x400000..0x43FFFF). A MID longer
//   than the window, a bad header, format 2, more than 32 tracks or a
//   SMPTE division is refused (load_ok = 0).
// * The FPGA never receives the filename (ioctl carries only the
//   extension), so there is no song-title display; the status strip shows
//   a playing box instead.
// * Injection is aligned to core steps: a byte is only sent in the clock
//   immediately following a step_done pulse. The pulse marks the end of a
//   step, so the byte lands in the MCU receive buffer at the start of the
//   next step - before that step's Q_TIMERS UART update - and the bench
//   can feed the same byte to the reference model at the same step
//   boundary, so the render stays bit-exact. Waiting for the next pulse
//   delays a byte by at most one step (600 ns, inaudible). A byte is also
//   deferred while the UART receiver has a byte in the same clock, so
//   live UART MIDI is never dropped by the player.
// * Timekeeping is in microseconds with 20 fractional bits. A scan pass
//   at load time builds a tempo map (meta 0x51) in tick order; tick_fp =
//   (usec-per-quarter-note << 20) / ppq is recomputed by a sequential
//   divider when the tempo changes, and each event adds
//   delta_ticks * tick_fp to a 64-bit schedule accumulator, so there is no
//   per-event rounding drift.
// * The player never waits on a tempo entry that does not exist: the map
//   always starts with a default 500000 usec/qn entry at tick 0.
// * Only one file is held: loading a new MID replaces the old one.
//   Loading a MID while one is playing stops playback first.
// * A scan that aborts (a track header that is not "MTrk", or a track
//   length past the end of the file) latches scan_failed, shown as SCAN
//   FAILED on the strip. The header check passed for such a file, so
//   this is the only visible sign the stored bytes were corrupt.

module mid_loader (
    input  logic        clk,
    input  logic        rst,
    input  logic        ioctl_download,
    input  logic        ioctl_wr,
    input  logic [15:0] ioctl_index,
    input  logic [7:0]  ioctl_dout,
    input  logic        rom_wave_pend,  // rom_loader's wave write pending
    input  logic        wave_busy,      // wave_ddr write busy
    output logic        ioctl_wait,
    output logic        wave_wr,
    output logic [22:0] wave_addr,
    output logic [7:0]  wave_data,
    output logic        mid_dl,         // a MID download is in progress
    output logic        load_done,      // 1-clock pulse: download finished
    output logic        load_ok,
    output logic        mid_failed,     // latched: last MID download ended without load_ok
    output logic [22:0] load_len,
    output logic [15:0] load_fmt,
    output logic [15:0] load_ntracks,
    output logic [15:0] load_ppq
);
    // Free wave_ddr window: waves 0x000000..0x3FFFFF, ROM2 upper half
    // 0x400000..0x43FFFF. 0x440000 + 0x3C0000 = 0x800000 (end of window).
    localparam [22:0] MID_BASE = 23'h440000;
    localparam [22:0] MID_MAX  = 23'h3C0000;  // ~3.75 MiB

    wire is_mid = ioctl_download && (ioctl_index[5:0] == 6'd2);
    assign mid_dl = is_mid;

    logic        dl_d;
    logic        over;
    logic        fifo_over;      // a byte arrived with the write FIFO full
    logic        end_pend;       // download ended, FIFO still draining
    logic        wr_active;      // a write is presented, awaiting accept
    logic [22:0] count;
    logic [7:0]  hdr [0:13];

    // 4-deep FIFO of pending DDR3 byte writes {addr[22:0], data[7:0]}.
    // Main sends blind: it never polls ioctl_wait (its fast block write
    // just strobes bytes out), so the loader must absorb bytes at SPI
    // rate even while the DDR3 port is busy serving the playing song's
    // reads. 3-bit pointers: empty iff wp == rp, full iff (wp - rp) == 4.
    logic [30:0] wfifo [0:3];
    logic [2:0]  wfifo_wp, wfifo_rp;
    wire         wfifo_empty = (wfifo_wp == wfifo_rp);
    wire         wfifo_full  = ((wfifo_wp - wfifo_rp) == 3'd4);

    // Held while a byte waits for the port, while a write is in flight,
    // or while the port is busy (Main ignores it, but it is the honest
    // backpressure signal for anything that does look).
    assign ioctl_wait = !wfifo_empty || wr_active || wave_busy || wave_wr;

    // The header check, evaluated when a download ends (also drives
    // mid_failed, latched until the next download starts or succeeds).
    wire load_ok_now = (count >= 23'd14) && !over && !fifo_over &&
                       (hdr[0] == 8'h4D) && (hdr[1] == 8'h54) &&  // "MThd"
                       (hdr[2] == 8'h68) && (hdr[3] == 8'h64) &&
                       (hdr[4] == 8'd0) && (hdr[5] == 8'd0) &&
                       (hdr[6] == 8'd0) && (hdr[7] == 8'd6) &&   // header len 6
                       ({hdr[8], hdr[9]} <= 16'd1) &&             // format 0/1
                       ({hdr[10], hdr[11]} >= 16'd1) &&
                       ({hdr[10], hdr[11]} <= 16'd32) &&
                       (hdr[12][7] == 1'b0) &&                    // ticks/qn, not SMPTE
                       ({hdr[12][6:0], hdr[13]} != 15'd0);

    always_ff @(posedge clk) begin
        wave_wr   <= 1'b0;
        load_done <= 1'b0;
        dl_d      <= is_mid;
        if (rst) begin
            wfifo_wp   <= 3'd0;
            wfifo_rp   <= 3'd0;
            wr_active  <= 1'b0;
            end_pend   <= 1'b0;
            count      <= 23'd0;
            over       <= 1'b0;
            fifo_over  <= 1'b0;
            load_ok    <= 1'b0;
            mid_failed <= 1'b0;
            load_len   <= 23'd0;
            load_fmt   <= 16'd0;
            load_ntracks <= 16'd0;
            load_ppq   <= 16'd0;
        end else begin
            if (is_mid && !dl_d) begin
                wfifo_wp  <= 3'd0;
                wfifo_rp  <= 3'd0;
                wr_active <= 1'b0;
                end_pend  <= 1'b0;
                count     <= 23'd0;
                over      <= 1'b0;
                fifo_over <= 1'b0;
                load_ok   <= 1'b0;
                mid_failed <= 1'b0;
            end
            // Drain: present the FIFO head to the DDR3 port and HOLD it
            // until wave_ddr accepts it (wave_busy rises). The old code
            // pulsed wave_wr for one clock and dropped the byte if the
            // port was busy that clock serving the playing song's reads -
            // that lost bytes on every second MID load, corrupting the
            // file in DDR3 while the header check (from the SPI bytes)
            // still passed, so the scan aborted silently.
            if (wr_active) begin
                if (wave_busy) begin
                    wr_active <= 1'b0;
                    wfifo_rp  <= wfifo_rp + 3'd1;
                end else begin
                    wave_wr <= 1'b1;
                end
            end else if (!wfifo_empty && !wave_busy && !rom_wave_pend) begin
                wave_wr   <= 1'b1;
                wave_addr <= wfifo[wfifo_rp[1:0]][30:8];
                wave_data <= wfifo[wfifo_rp[1:0]][7:0];
                wr_active <= 1'b1;
            end
            if (is_mid && ioctl_wr) begin
                if (count < 23'd14) hdr[count[3:0]] <= ioctl_dout;
                if (!over && count < MID_MAX) begin
                    if (!wfifo_full) begin
                        wfifo[wfifo_wp[1:0]] <= {MID_BASE + count, ioctl_dout};
                        wfifo_wp <= wfifo_wp + 3'd1;
                    end else begin
                        fifo_over <= 1'b1;
                    end
                    count <= count + 23'd1;
                end else begin
                    over  <= 1'b1;
                    count <= count + 23'd1;
                end
            end
            if (!is_mid && dl_d) begin
                load_len     <= count;
                load_fmt     <= {hdr[8], hdr[9]};
                load_ntracks <= {hdr[10], hdr[11]};
                load_ppq     <= {1'b0, hdr[12][6:0], hdr[13]};
                load_ok      <= load_ok_now;
                mid_failed   <= !load_ok_now;
                end_pend     <= 1'b1;
            end
            // The scan must not start until every byte is in DDR3.
            if (end_pend && wfifo_empty && !wr_active) begin
                load_done <= 1'b1;
                end_pend  <= 1'b0;
            end
        end
    end
endmodule

// The two small memories use the shared hdl/common/sdp_ram.sv (one write
// port, one registered read port; a read of the address written in the
// same clock returns the old data - the player never does that).

// ---------------------------------------------------------------------------
// midi_player
// ---------------------------------------------------------------------------
// SMF format 0/1 parser + scheduler.
//
// Operation:
//   * load_done (a MID finished loading and passed the header check):
//     scan the file once, building the tempo map in tick order, then
//     start playing automatically when the scan finishes (a valid load
//     auto-plays; a failed load plays nothing and shows MID LOAD FAILED).
//   * Play (T[4], momentary): manual (re)start after Stop, or while a
//     scan is running (queued until the scan finishes). Re-walks the
//     track headers, then repeatedly fills each track's next event,
//     picks the earliest event across tracks, waits until its tick in
//     real time, and emits its bytes.
//   * Stop (T[5], momentary) or end of song: send the full panic
//     (CC120 All Sound Off + CC121 Reset All Controllers + CC123 All
//     Notes Off) on all 16 channels, then idle (or restart, if Loop O[6]).
//   * Loading a new MID while one is playing runs the same panic first:
//     the old song is silenced before the new file's scan starts. If the
//     new file finishes loading while the panic is still running, its
//     scan is deferred until the panic ends (scan_pending).
//
// Track state (one 192-bit word per track, in trk_ram):
//   [191]    ended, [190] has_ev, [189:182] run, [181:174] ev_status,
//   [173:158] ev_len, [157:134] ev_tempo, [133] ev_is_tempo,
//   [132:110] ev_data_off, [109:78] base_tick, [77:46] abs_tick,
//   [45:23] end_off, [22:0] off.
//   off/end_off/ev_data_off are file offsets (0..mid_len); the DDR3 byte
//   address is MID_BASE + offset. base_tick is the tick of the track's
//   last emitted event; abs_tick = base_tick + delta of its pending event.
//
// Tempo map (one 56-bit word per entry, in tmp_ram):
//   [55:24] tick, [23:0] usec-per-quarter-note (clamped to 10k..10M).
//   Entry 0 is the default 500000 at tick 0. At most 32 entries; further
//   tempo events are dropped (the previous tempo stays in force).
module midi_player (
    input  logic        clk,
    input  logic        rst,            // hard reset (power-on / core reset)
    input  logic        running,        // core sequencer running (sc55_board)
    input  logic        step_done,      // core step pulse (sc55_board)
    // MID file (from mid_loader)
    input  logic        mid_ok,
    input  logic [22:0] mid_len,
    input  logic [15:0] mid_ntracks,
    input  logic [15:0] mid_ppq,
    input  logic        load_done,      // 1-clock pulse: a new MID loaded
    input  logic        mid_dl,         // a MID download is in progress
    // controls (Main status bits; T triggers are momentary pulses)
    input  logic        play_trig,      // T[4]
    input  logic        stop_trig,      // T[5]
    input  logic        loop_en,        // O[6]
    // DDR3 player read port (wave_ddr, shared with PCM reads)
    output logic        preq,
    output logic [22:0] paddr,
    input  logic        pready,
    input  logic        prvalid,
    input  logic [7:0]  prdata,
    // MIDI injection into the core (sc55_board midi_inject)
    output logic        midi_inject,
    output logic [7:0]  midi_inject_data,
    // UART receiver state (sc55_board / sc55_mcu)
    input  logic        midi_full,      // MCU RX ring buffer (almost) full
    input  logic        uart_valid,     // UART receiver has a byte this clock
    // status
    output logic        playing,
    output logic        scan_failed  // latched: last scan aborted (bad track data)
);
    localparam [22:0] MID_BASE  = 23'h440000;
    localparam [13:0] BAUD_DIV  = 14'd16000;  // 320 us at 50 MHz = MIDI baud

    typedef enum logic [5:0] {
        P_IDLE,
        P_SCAN_INIT, P_PLAY_INIT,
        I_LOOP, I_GOT, I_DONE,
        F_LOOP, F_RD, F_DELTA_GOT, F_PEEK_GOT,
        F_META_TYPE_GOT, F_META_LEN_GOT,
        F_TEMPO0_GOT, F_TEMPO1_GOT, F_TEMPO2_GOT,
        F_SYSEX_LEN_GOT,
        F_DONE, F_NEXT,
        PICK_LOOP, PICK_RD, PICK_DONE,
        SCAN_MAP,
        DIV_INIT, DIV_RUN, MUL_INIT, MUL_RUN,
        T_SET, T_GET,
        P_PLAY_WAIT,
        E_START, E_RD_GOT, E_NEXT, E_CLR, E_CLR_RD,
        F_WAIT,
        S_SEND,
        P_PANIC, P_PANIC_LOAD, P_PANIC_ADV,
        RD_ISSUE, RD_WAIT
    } pstate_t;

    pstate_t state;
    logic        mode_scan;     // 1 = current operation is a tempo-map scan
    logic        play_pending;  // Play pressed while a scan was running
    logic        scan_pending;  // a good load arrived mid-panic: scan when it ends
    logic        map_valid;     // tempo map is complete and usable
    logic        mid_ok_r;      // latched load_ok
    logic [22:0] file_len;
    logic [15:0] ntracks;
    logic [15:0] ppq;

    // track iterators (6 bits: 0..32, so 32 tracks compare cleanly)
    logic [5:0]  fill_idx, pick_idx, init_idx;
    logic [22:0] chunk_off;     // file offset of the next track chunk
    logic [3:0]  hdr_pos;       // 0..7 header byte position
    logic [31:0] len_acc;

    // fill working set (mirrors one trk_ram word)
    logic [22:0] w_off, w_end, w_data_off;
    logic [31:0] w_abs, w_base;
    logic [15:0] w_len;
    logic [23:0] w_tempo;
    logic [7:0]  w_status, w_run;
    logic        w_is_tempo, w_has, w_ended;

    // VLQ / meta temporaries
    logic [27:0] vlq_acc;
    logic [1:0]  vlq_cnt;
    logic [7:0]  meta_type;
    logic [23:0] tempo_acc;

    // DDR3 byte reader
    logic [22:0] rd_ptr;        // file offset of the byte to read next
    pstate_t     rd_ret;        // state to enter when the byte arrives
    logic [7:0]  rd_byte;

    // pick (best = earliest pending event across tracks)
    logic [4:0]  best_idx;
    logic [31:0] best_tick;
    logic [15:0] best_len;
    logic [22:0] best_data_off;
    logic [23:0] best_tempo;
    logic [7:0]  best_status;
    logic        best_found, best_is_tempo;

    // tempo map
    logic [5:0]  map_count;     // entries written (0..32)
    logic [31:0] map_last_tick; // tick of the last map entry
    logic [23:0] map0_usec;     // usec of map entry 0 (tempo at tick 0)
    logic [5:0]  tempo_idx;     // index of the tempo entry in force
    logic [23:0] cur_usec;      // usec/qn in force
    logic        tempo_dirty;   // tick_fp must be recomputed

    // time (microseconds, 20 fractional bits)
    logic [43:0] tick_fp;       // (usec/qn << 20) / ppq
    logic [63:0] sched_us_fp;   // absolute time the next event is due
    logic [63:0] now_us_fp;     // free-running time while playing
    logic [19:0] wait_ticks;     // delta ticks to the next event (clamped)
    wire  [31:0] tick_delta = best_tick - g_tick;  // (part-select on an
    // expression is a Verilator syntax error; select from this wire instead)
    logic [31:0] g_tick;        // global tick of the last picked event
    logic [5:0]  us_div;        // 50 MHz / 50 = 1 us

    // sequential divider / multiplier (one bit per clock)
    logic [43:0] div_dvd, div_quot, div_rem;
    logic [14:0] div_divisor;
    logic [5:0]  div_cnt;
    logic [63:0] mul_a, mul_acc;
    logic [19:0] mul_b;
    logic [5:0]  mul_cnt;

    // emit
    logic [15:0] emit_idx;      // bytes of the current event already sent
    logic [7:0]  send_byte;
    pstate_t     send_ret;      // state after the byte is sent

    // panic (All Sound Off / Reset All Controllers / All Notes Off)
    logic [3:0]  panic_ch;
    logic [3:0]  panic_ph;
    pstate_t     panic_ret;     // P_IDLE, or P_PLAY_INIT to loop
    // Panic in flight: the panic FSM, including the shared S_SEND state
    // while it is sending a panic byte.
    wire panic_busy = (state == P_PANIC) || (state == P_PANIC_LOAD) ||
                      (state == P_PANIC_ADV) ||
                      (state == S_SEND && send_ret == P_PANIC_ADV);

    // edge detectors / free-running flags
    logic        play_d, stop_d, mid_dl_d;
    logic [13:0] baud_cnt;      // clocks until the next byte may be sent

    // track RAM (192-bit words, 32 tracks)
    logic        trk_we;
    logic [4:0]  trk_wa;
    logic [191:0] trk_wd;
    logic [191:0] trk_rd;
    // tempo RAM (56-bit words, 32 entries)
    logic        tmp_we;
    logic [4:0]  tmp_wa;
    logic [55:0] tmp_wd;
    logic [55:0] tmp_rd;

    sdp_ram #(.DW(192), .AW(5)) trk_ram (
        .clk(clk), .we(trk_we), .wa(trk_wa), .wd(trk_wd),
        .ra(trk_ra), .rd(trk_rd));
    sdp_ram #(.DW(56), .AW(5)) tmp_ram (
        .clk(clk), .we(tmp_we), .wa(tmp_wa), .wd(tmp_wd),
        .ra(tmp_ra), .rd(tmp_rd));

    // RAM read addresses are combinational from the state, so the data is
    // valid in the following state (one wait state is built into the FSM).
    // Exception: after E_CLR_RD writes a consumed word, the refill needs
    // TWO wait states (F_WAIT + F_LOOP) before F_RD, because the edge
    // that writes also samples rd from the pre-write memory.
    logic [4:0] trk_ra;
    logic [4:0] tmp_ra;
    assign trk_ra = (state == F_LOOP || state == F_RD || state == F_WAIT) ? fill_idx[4:0] :
                    (state == PICK_LOOP || state == PICK_RD) ? pick_idx[4:0] :
                    (state == E_CLR || state == E_CLR_RD) ? best_idx : 5'd0;
    assign tmp_ra = (state == T_SET || state == T_GET) ? (tempo_idx[4:0] + 5'd1) : 5'd0;

    // DDR3 byte-read port: one byte per request, address = MID_BASE + offset.
    assign preq  = (state == RD_ISSUE);
    assign paddr = MID_BASE + rd_ptr;

    // Data bytes after a status byte: C/D (program change, channel pressure)
    // take 1, other channel messages take 2, F1/F3 take 1, F2 takes 2,
    // F6 and real-time F8-FE take 0.
    function automatic [1:0] ev_ndata(input logic [7:0] s);
        case (s)
            8'hF1, 8'hF3: ev_ndata = 2'd1;
            8'hF2:        ev_ndata = 2'd2;
            8'hF6, 8'hF8, 8'hF9, 8'hFA, 8'hFB, 8'hFC, 8'hFD, 8'hFE:
                          ev_ndata = 2'd0;
            default: begin
                if (s[7:4] == 4'hC || s[7:4] == 4'hD) ev_ndata = 2'd1;
                else ev_ndata = 2'd2;
            end
        endcase
    endfunction

    // Tempo (usec per quarter note) is clamped so tick_fp always fits in
    // 44 bits: (10M << 20) / 1 < 2^44.
    function automatic [23:0] clamp_usec(input logic [23:0] u);
        if (u < 24'd10000)       clamp_usec = 24'd10000;
        else if (u > 24'd10000000) clamp_usec = 24'd10000000;
        else                     clamp_usec = u;
    endfunction

    always_ff @(posedge clk) begin
        // Locals used by one state each (blocking assignments, combinational
        // within this block).
        logic [32:0] t33;
        logic [23:0] t24;
        logic [27:0] len28;
        logic [43:0] rem_next, qn;
        logic [63:0] an;
        logic [5:0]  wi;

        // ---- defaults (a later assignment in this block wins) ----
        midi_inject <= 1'b0;
        trk_we      <= 1'b0;
        tmp_we      <= 1'b0;
        play_d      <= play_trig;
        stop_d      <= stop_trig;
        mid_dl_d    <= mid_dl;
        if (baud_cnt != 14'd0) baud_cnt <= baud_cnt - 14'd1;
        if (us_div == 6'd49) begin
            us_div <= 6'd0;
            if (playing) now_us_fp <= now_us_fp + 64'h100000;
        end else begin
            us_div <= us_div + 6'd1;
        end

        if (rst) begin
            state        <= P_IDLE;
            playing      <= 1'b0;
            play_pending <= 1'b0;
            scan_pending <= 1'b0;
            scan_failed  <= 1'b0;
            map_valid    <= 1'b0;
            mid_ok_r     <= 1'b0;
            baud_cnt     <= 14'd0;
            us_div       <= 6'd0;
            now_us_fp    <= 64'd0;
        end else if (load_done) begin
            // A new MID finished loading: abort anything in flight and
            // scan it. The scan re-validates the track headers, so a bad
            // file is caught here even if the header check passed.
            playing      <= 1'b0;
            play_pending <= 1'b0;
            scan_failed  <= 1'b0;
            map_valid    <= 1'b0;
            mid_ok_r     <= mid_ok;
            file_len     <= mid_len;
            ntracks      <= mid_ntracks;
            ppq          <= mid_ppq;
            if (mid_ok && panic_busy) begin
                // A panic is still silencing the old song: do not cut it
                // short (the scan would steal the FSM mid-panic). The
                // file info is latched above; the panic's completion
                // starts the scan instead.
                scan_pending <= 1'b1;
            end else if (mid_ok) begin
                mode_scan <= 1'b1;
                // Auto-play: start when the scan finishes (the scan's
                // PICK_DONE starts playback if play_pending is set).
                play_pending <= 1'b1;
                state     <= P_SCAN_INIT;
            end else if (!panic_busy) begin
                state <= P_IDLE;
            end
            // else: bad load during a panic - let the panic finish
            // (it ends in P_IDLE); MID LOAD FAILED is shown by the loader.
        end else if (mid_dl && !mid_dl_d) begin
            // A MID download just started: the new file replaces the one
            // playing, so silence the synth first (the same panic as
            // Stop). Just dropping `playing` would leave the old song's
            // notes sounding under the new one. The panic sends MIDI
            // bytes only, so it does not touch the DDR3 window the
            // loader is writing; the scan is deferred until the panic
            // ends (see load_done above).
            play_pending <= 1'b0;
            scan_pending <= 1'b0;
            if (playing) begin
                panic_ret <= P_IDLE;
                panic_ch  <= 4'd0;
                panic_ph  <= 4'd0;
                state     <= P_PANIC;
            end else begin
                playing <= 1'b0;
                state   <= P_IDLE;
            end
        end else if (stop_trig && !stop_d && (playing || (mode_scan && state != P_IDLE))) begin
            // Stop: silence the synth (panic), or cancel a running scan.
            play_pending <= 1'b0;
            if (playing) begin
                panic_ret <= P_IDLE;
                panic_ch  <= 4'd0;
                panic_ph  <= 4'd0;
                state     <= P_PANIC;
            end else begin
                map_valid <= 1'b0;
                state     <= P_IDLE;
            end
        end else if (playing && !running) begin
            // Core stopped (ROM load / reset / user reset): abort, no
            // panic - the core is being reset anyway.
            playing      <= 1'b0;
            play_pending <= 1'b0;
            state        <= P_IDLE;
        end else if (play_trig && !play_d && mid_ok_r && !playing && !mid_dl && running) begin
            if (mode_scan && state != P_IDLE) begin
                play_pending <= 1'b1;   // start when the scan finishes
            end else if (state == P_IDLE && map_valid) begin
                playing <= 1'b1;
                state   <= P_PLAY_INIT;
            end
        end else begin
            case (state)
            P_IDLE: begin
                // All control is handled above; nothing to do here.
            end

            // ---- scan / play initialisation (shared) ----
            P_SCAN_INIT: begin
                map_count     <= 6'd1;
                map_last_tick <= 32'd0;
                map0_usec     <= 24'd500000;
                // Default tempo entry 0: 500000 usec/qn at tick 0.
                tmp_we <= 1'b1;
                tmp_wa <= 5'd0;
                tmp_wd <= {32'd0, 24'd500000};
                chunk_off <= 23'd14;
                init_idx  <= 6'd0;
                state     <= I_LOOP;
            end
            P_PLAY_INIT: begin
                mode_scan   <= 1'b0;
                tempo_idx   <= 6'd0;
                cur_usec    <= map0_usec;
                tempo_dirty <= 1'b1;
                g_tick      <= 32'd0;
                sched_us_fp <= 64'd0;
                now_us_fp   <= 64'd0;   // overrides the prescaler this edge
                us_div      <= 6'd0;
                chunk_off   <= 23'd14;
                init_idx    <= 6'd0;
                state       <= I_LOOP;
            end

            // ---- track-header walk ("MTrk" + 32-bit BE length) ----
            I_LOOP: begin
                if ({10'd0, init_idx} >= ntracks) begin
                    fill_idx <= 6'd0;
                    state    <= F_LOOP;
                end else begin
                    hdr_pos <= 4'd0;
                    len_acc <= 32'd0;
                    rd_ptr  <= chunk_off;
                    rd_ret  <= I_GOT;
                    state   <= RD_ISSUE;
                end
            end
            I_GOT: begin
                if (hdr_pos < 4'd4) begin
                    if (rd_byte != (hdr_pos == 4'd0 ? 8'h4D :
                                    hdr_pos == 4'd1 ? 8'h54 :
                                    hdr_pos == 4'd2 ? 8'h72 : 8'h6B)) begin
                        mid_ok_r  <= 1'b0;
                        map_valid <= 1'b0;
                        scan_failed <= 1'b1;
                        state     <= P_IDLE;
                    end else begin
                        hdr_pos <= hdr_pos + 4'd1;
                        rd_ptr  <= rd_ptr + 23'd1;
                        state   <= RD_ISSUE;
                    end
                end else begin
                    len_acc <= {len_acc[23:0], rd_byte};
                    rd_ptr  <= rd_ptr + 23'd1;
                    if (hdr_pos == 4'd7) begin
                        state <= I_DONE;
                    end else begin
                        hdr_pos <= hdr_pos + 4'd1;
                        state   <= RD_ISSUE;
                    end
                end
            end
            I_DONE: begin
                // The track data must fit in the file.
                if ((chunk_off >= file_len) ||
                    ((file_len - chunk_off) < 23'd8) ||
                    (len_acc > {9'd0, (file_len - chunk_off - 23'd8)})) begin
                    mid_ok_r  <= 1'b0;
                    map_valid <= 1'b0;
                    scan_failed <= 1'b1;
                    state     <= P_IDLE;
                end else begin
                    trk_we <= 1'b1;
                    trk_wa <= init_idx[4:0];
                    // {ended, has, run, status, len, tempo, is_tempo,
                    //  data_off, base, abs, end, off}
                    trk_wd <= {1'b0, 1'b0, 8'd0, 8'd0, 16'd0, 24'd0, 1'b0,
                               chunk_off + 23'd8, 32'd0, 32'd0,
                               chunk_off + 23'd8 + len_acc[22:0],
                               chunk_off + 23'd8};
                    chunk_off <= chunk_off + 23'd8 + len_acc[22:0];
                    init_idx  <= init_idx + 6'd1;
                    state     <= I_LOOP;
                end
            end

            // ---- fill: read the next event of one track into its RAM word ----
            F_LOOP: begin
                if ({10'd0, fill_idx} >= ntracks) begin
                    pick_idx   <= 6'd0;
                    best_found <= 1'b0;
                    state      <= PICK_LOOP;
                end else begin
                    state <= F_RD;   // trk_rd is valid (ra was set in F_LOOP)
                end
            end
            F_RD: begin
                w_off      <= trk_rd[22:0];
                w_end      <= trk_rd[45:23];
                w_abs      <= trk_rd[77:46];
                w_base     <= trk_rd[109:78];
                w_data_off <= trk_rd[132:110];
                w_is_tempo <= trk_rd[133];
                w_tempo    <= trk_rd[157:134];
                w_len      <= trk_rd[173:158];
                w_status   <= trk_rd[181:174];
                w_run      <= trk_rd[189:182];
                w_has      <= trk_rd[190];
                w_ended    <= trk_rd[191];
                if (trk_rd[191] || trk_rd[190]) begin
                    state <= F_NEXT;   // ended, or an event is already pending
                end else if (trk_rd[22:0] >= trk_rd[45:23]) begin
                    w_ended <= 1'b1;   // exhausted without end-of-track
                    w_has   <= 1'b0;
                    state   <= F_DONE;
                end else begin
                    vlq_acc <= 28'd0;
                    vlq_cnt <= 2'd0;
                    rd_ptr  <= trk_rd[22:0];
                    rd_ret  <= F_DELTA_GOT;
                    state   <= RD_ISSUE;
                end
            end
            F_DELTA_GOT: begin
                vlq_acc <= {vlq_acc[20:0], rd_byte[6:0]};
                vlq_cnt <= vlq_cnt + 2'd1;
                if (rd_byte[7] && vlq_cnt < 2'd3) begin
                    rd_ptr <= rd_ptr + 23'd1;
                    rd_ret <= F_DELTA_GOT;
                    state  <= RD_ISSUE;
                end else begin
                    // delta complete: abs tick = base + delta (saturating)
                    t33 = {1'b0, w_base} +
                          {5'd0, {vlq_acc[20:0], rd_byte[6:0]}};
                    w_abs <= t33[32] ? 32'hFFFF_FFFF : t33[31:0];
                    w_off <= rd_ptr + 23'd1;
                    rd_ptr <= rd_ptr + 23'd1;
                    rd_ret <= F_PEEK_GOT;
                    state  <= RD_ISSUE;
                end
            end
            F_PEEK_GOT: begin
                if (rd_byte == 8'hFF) begin
                    // Meta event: read the type byte next.
                    w_off  <= rd_ptr + 23'd1;
                    rd_ptr <= rd_ptr + 23'd1;
                    rd_ret <= F_META_TYPE_GOT;
                    state  <= RD_ISSUE;
                end else if (rd_byte == 8'hF0 || rd_byte == 8'hF7) begin
                    // SysEx: running status is cancelled by SysEx.
                    w_status <= rd_byte;
                    w_run    <= 8'd0;
                    vlq_acc  <= 28'd0;
                    vlq_cnt  <= 2'd0;
                    w_off    <= rd_ptr + 23'd1;
                    rd_ptr   <= rd_ptr + 23'd1;
                    rd_ret   <= F_SYSEX_LEN_GOT;
                    state    <= RD_ISSUE;
                end else if (rd_byte < 8'h80) begin
                    // Running status: this byte is the first data byte.
                    if (w_run == 8'd0) begin
                        w_ended <= 1'b1;
                        w_has   <= 1'b0;
                        state   <= F_DONE;
                    end else begin
                        t24 = {1'b0, rd_ptr} + {22'd0, ev_ndata(w_run)};
                        if (t24 > {1'b0, w_end}) begin
                            w_ended <= 1'b1;
                            w_has   <= 1'b0;
                        end else begin
                            w_status   <= w_run;
                            w_data_off <= rd_ptr;
                            w_len      <= 16'd1 + {14'd0, ev_ndata(w_run)};
                            w_off      <= t24[22:0];
                            w_has      <= 1'b1;
                        end
                        state <= F_DONE;
                    end
                end else begin
                    // New status byte (channel message or system common).
                    t24 = {1'b0, rd_ptr} + 24'd1 + {22'd0, ev_ndata(rd_byte)};
                    if (t24 > {1'b0, w_end}) begin
                        w_ended <= 1'b1;
                        w_has   <= 1'b0;
                    end else begin
                        w_status   <= rd_byte;
                        w_data_off <= rd_ptr + 23'd1;
                        w_len      <= 16'd1 + {14'd0, ev_ndata(rd_byte)};
                        w_off      <= t24[22:0];
                        w_has      <= 1'b1;
                        if (rd_byte >= 8'h80 && rd_byte <= 8'hEF)
                            w_run <= rd_byte;
                        else if (rd_byte == 8'hF1 || rd_byte == 8'hF2 ||
                                 rd_byte == 8'hF3 || rd_byte == 8'hF6)
                            w_run <= 8'd0;   // system common cancels it
                        // Real-time F8-FE leaves running status alone.
                    end
                    state <= F_DONE;
                end
            end
            F_META_TYPE_GOT: begin
                meta_type <= rd_byte;
                vlq_acc   <= 28'd0;
                vlq_cnt   <= 2'd0;
                w_off     <= rd_ptr + 23'd1;
                rd_ptr    <= rd_ptr + 23'd1;
                rd_ret    <= F_META_LEN_GOT;
                state     <= RD_ISSUE;
            end
            F_META_LEN_GOT: begin
                len28 = {vlq_acc[20:0], rd_byte[6:0]};
                vlq_acc <= len28;
                vlq_cnt <= vlq_cnt + 2'd1;
                if (rd_byte[7] && vlq_cnt < 2'd3) begin
                    rd_ptr <= rd_ptr + 23'd1;
                    rd_ret <= F_META_LEN_GOT;
                    state  <= RD_ISSUE;
                end else if (meta_type == 8'h2F) begin
                    w_ended <= 1'b1;   // end of track
                    w_has   <= 1'b0;
                    state   <= F_DONE;
                end else if (meta_type == 8'h51 && len28 == 28'd3) begin
                    // Tempo: the 3 value bytes must fit in the track.
                    t24 = {1'b0, rd_ptr} + 24'd4;
                    if (t24 > {1'b0, w_end}) begin
                        w_ended <= 1'b1;
                        w_has   <= 1'b0;
                        state   <= F_DONE;
                    end else begin
                        tempo_acc <= 24'd0;
                        w_off     <= rd_ptr + 23'd1;
                        rd_ptr    <= rd_ptr + 23'd1;
                        rd_ret    <= F_TEMPO0_GOT;
                        state     <= RD_ISSUE;
                    end
                end else if (len28[27:23] != 5'd0) begin
                    w_ended <= 1'b1;   // meta data longer than the window
                    w_has   <= 1'b0;
                    state   <= F_DONE;
                end else begin
                    t24 = {1'b0, rd_ptr} + 24'd1 + {1'b0, len28[22:0]};
                    if (t24 > {1'b0, w_end}) begin
                        w_ended <= 1'b1;
                        w_has   <= 1'b0;
                    end else begin
                        w_off      <= t24[22:0];
                        w_len      <= 16'd0;   // meta: no bytes to emit
                        w_is_tempo <= 1'b0;
                        w_has      <= 1'b1;
                    end
                    state <= F_DONE;
                end
            end
            F_TEMPO0_GOT: begin
                tempo_acc <= {rd_byte, 16'd0};
                w_off     <= rd_ptr + 23'd1;
                rd_ptr    <= rd_ptr + 23'd1;
                rd_ret    <= F_TEMPO1_GOT;
                state     <= RD_ISSUE;
            end
            F_TEMPO1_GOT: begin
                tempo_acc <= {tempo_acc[23:16], rd_byte, 8'd0};
                w_off     <= rd_ptr + 23'd1;
                rd_ptr    <= rd_ptr + 23'd1;
                rd_ret    <= F_TEMPO2_GOT;
                state     <= RD_ISSUE;
            end
            F_TEMPO2_GOT: begin
                w_tempo    <= {tempo_acc[23:8], rd_byte};
                w_is_tempo <= 1'b1;
                w_len      <= 16'd0;
                w_has      <= 1'b1;
                w_off      <= rd_ptr + 23'd1;
                state      <= F_DONE;
            end
            F_SYSEX_LEN_GOT: begin
                len28 = {vlq_acc[20:0], rd_byte[6:0]};
                vlq_acc <= len28;
                vlq_cnt <= vlq_cnt + 2'd1;
                if (rd_byte[7] && vlq_cnt < 2'd3) begin
                    rd_ptr <= rd_ptr + 23'd1;
                    rd_ret <= F_SYSEX_LEN_GOT;
                    state  <= RD_ISSUE;
                end else if (len28[27:23] != 5'd0) begin
                    w_ended <= 1'b1;   // SysEx longer than the window
                    w_has   <= 1'b0;
                    state   <= F_DONE;
                end else begin
                    t24 = {1'b0, rd_ptr} + 24'd1 + {1'b0, len28[22:0]};
                    if (t24 > {1'b0, w_end}) begin
                        w_ended <= 1'b1;
                        w_has   <= 1'b0;
                    end else begin
                        w_off      <= t24[22:0];
                        w_data_off <= rd_ptr + 23'd1;
                        w_is_tempo <= 1'b0;
                        // Events longer than 16 KiB are skipped, not
                        // emitted (they would overflow the emit counter).
                        if (len28 > 28'd16383) w_len <= 16'd0;
                        else w_len <= 16'd1 + len28[15:0];
                        w_has <= 1'b1;
                    end
                    state <= F_DONE;
                end
            end
            F_DONE: begin
                trk_we <= 1'b1;
                trk_wa <= fill_idx[4:0];
                trk_wd <= {w_ended, w_has, w_run, w_status, w_len, w_tempo,
                           w_is_tempo, w_data_off, w_base, w_abs, w_end, w_off};
                state  <= F_NEXT;
            end
            F_NEXT: begin
                fill_idx <= fill_idx + 6'd1;
                state    <= F_LOOP;
            end
            // ---- pick: earliest pending event across all tracks ----
            PICK_LOOP: begin
                if ({10'd0, pick_idx} >= ntracks) begin
                    state <= PICK_DONE;
                end else begin
                    state <= PICK_RD;   // trk_rd is valid (ra was set in PICK_LOOP)
                end
            end
            PICK_RD: begin
                if (trk_rd[190] && (!best_found || trk_rd[77:46] < best_tick)) begin
                    best_found   <= 1'b1;
                    best_idx     <= pick_idx[4:0];
                    best_tick    <= trk_rd[77:46];
                    best_len     <= trk_rd[173:158];
                    best_status  <= trk_rd[181:174];
                    best_data_off <= trk_rd[132:110];
                    best_is_tempo <= trk_rd[133];
                    best_tempo   <= trk_rd[157:134];
                end
                pick_idx <= pick_idx + 6'd1;
                state    <= PICK_LOOP;
            end
            PICK_DONE: begin
                if (!best_found) begin
                    if (mode_scan) begin
                        map_valid <= 1'b1;
                        mode_scan <= 1'b0;
                        if (play_pending) begin
                            play_pending <= 1'b0;
                            playing      <= 1'b1;
                            state        <= P_PLAY_INIT;
                        end else begin
                            state <= P_IDLE;
                        end
                    end else begin
                        // Song over: silence the synth, then idle or loop.
                        panic_ret <= loop_en ? P_PLAY_INIT : P_IDLE;
                        panic_ch  <= 4'd0;
                        panic_ph  <= 4'd0;
                        state     <= P_PANIC;
                    end
                end else begin
                    // Advance the global tick to this event (never backwards).
                    if (best_tick > g_tick) begin
                        wait_ticks <= (tick_delta >= 32'd1048576) ?
                                    20'hFFFFF : tick_delta[19:0];
                        g_tick <= best_tick;
                    end else begin
                        wait_ticks <= 20'd0;
                    end
                    if (mode_scan) begin
                        if (best_is_tempo) state <= SCAN_MAP;
                        else state <= E_CLR;
                    end else begin
                        if (tempo_dirty) state <= DIV_INIT;
                        else state <= MUL_INIT;
                    end
                end
            end

            // ---- scan: record a tempo event into the map (tick order) ----
            SCAN_MAP: begin
                if (map_count > 6'd0 && map_last_tick == best_tick) begin
                    // Same tick as the last entry: overwrite it (a later
                    // tempo at the same tick wins, as in the SMF merge).
                    wi = map_count - 6'd1;
                    tmp_we <= 1'b1;
                    tmp_wa <= wi[4:0];
                    tmp_wd <= {best_tick, clamp_usec(best_tempo)};
                    map_last_tick <= best_tick;
                    if (wi == 6'd0) map0_usec <= clamp_usec(best_tempo);
                end else if (map_count < 6'd32) begin
                    wi = map_count;
                    tmp_we <= 1'b1;
                    tmp_wa <= wi[4:0];
                    tmp_wd <= {best_tick, clamp_usec(best_tempo)};
                    map_count <= map_count + 6'd1;
                    map_last_tick <= best_tick;
                    if (wi == 6'd0) map0_usec <= clamp_usec(best_tempo);
                end
                // else the map is full (32 entries): drop this tempo event;
                // the previous tempo stays in force. map_last_tick is not
                // updated, so a later event at this tick can still
                // overwrite the last entry.
                state <= E_CLR;
            end

            // ---- time math: tick_fp = (cur_usec << 20) / ppq ----
            DIV_INIT: begin
                div_dvd     <= {20'd0, cur_usec} << 20;
                div_divisor <= ppq[14:0];
                div_rem     <= 44'd0;
                div_quot    <= 44'd0;
                div_cnt     <= 6'd44;
                state       <= DIV_RUN;
            end
            DIV_RUN: begin
                rem_next = {div_rem[42:0], div_dvd[43]};
                if (rem_next >= {29'd0, div_divisor}) begin
                    div_rem <= rem_next - {29'd0, div_divisor};
                    qn = {div_quot[42:0], 1'b1};
                end else begin
                    div_rem <= rem_next;
                    qn = {div_quot[42:0], 1'b0};
                end
                div_quot <= qn;
                div_dvd  <= {div_dvd[42:0], 1'b0};
                if (div_cnt == 6'd1) begin
                    tick_fp     <= qn;
                    tempo_dirty <= 1'b0;
                    state       <= MUL_INIT;
                end else begin
                    div_cnt <= div_cnt - 6'd1;
                end
            end
            MUL_INIT: begin
                mul_a   <= {20'd0, tick_fp};
                mul_b   <= wait_ticks;
                mul_acc <= 64'd0;
                mul_cnt <= 6'd20;
                state   <= MUL_RUN;
            end
            MUL_RUN: begin
                if (mul_b[0]) an = mul_acc + mul_a;
                else           an = mul_acc;
                mul_acc <= an;
                mul_a   <= {mul_a[62:0], 1'b0};
                mul_b   <= {1'b0, mul_b[19:1]};
                if (mul_cnt == 6'd1) begin
                    sched_us_fp <= sched_us_fp + an;
                    state       <= T_SET;
                end else begin
                    mul_cnt <= mul_cnt - 6'd1;
                end
            end

            // ---- tempo pointer: advance while the next entry is due ----
            T_SET: begin
                if ({1'b0, tempo_idx} + 7'd1 >= {1'b0, map_count}) begin
                    state <= P_PLAY_WAIT;
                end else begin
                    state <= T_GET;   // tmp_rd is valid (ra was set in T_SET)
                end
            end
            T_GET: begin
                if (tmp_rd[55:24] <= g_tick) begin
                    tempo_idx   <= tempo_idx + 6'd1;
                    cur_usec    <= tmp_rd[23:0];
                    tempo_dirty <= 1'b1;
                    state       <= T_SET;   // check the following entry too
                end else begin
                    state <= P_PLAY_WAIT;
                end
            end

            // ---- wait until the event is due in real time ----
            P_PLAY_WAIT: begin
                if (now_us_fp >= sched_us_fp) begin
                    if (best_len == 16'd0) state <= E_CLR;   // meta: nothing to emit
                    else state <= E_START;
                end
            end

            // ---- emit the event's bytes ----
            E_START: begin
                emit_idx  <= 16'd0;
                send_byte <= best_status;
                send_ret  <= E_NEXT;
                state     <= S_SEND;
            end
            E_RD_GOT: begin
                send_byte <= rd_byte;
                send_ret  <= E_NEXT;
                state     <= S_SEND;
            end
            E_NEXT: begin
                emit_idx <= emit_idx + 16'd1;
                if (emit_idx + 16'd1 >= best_len) begin
                    state <= E_CLR;
                end else begin
                    // Next byte index is emit_idx+1; its file offset is
                    // best_data_off + emit_idx (byte 1 is at data_off).
                    rd_ptr <= best_data_off + {7'd0, emit_idx};
                    rd_ret <= E_RD_GOT;
                    state  <= RD_ISSUE;
                end
            end
            E_CLR: begin
                state <= E_CLR_RD;   // trk_rd is valid (ra was set in E_CLR)
            end
            E_CLR_RD: begin
                // Consume the event: clear has_ev, and move base_tick to
                // the tick it was emitted at (its next delta is relative
                // to that tick).
                trk_we <= 1'b1;
                trk_wa <= best_idx;
                trk_wd <= {trk_rd[191], 1'b0, trk_rd[189:110], best_tick,
                           trk_rd[77:0]};
                fill_idx <= 6'd0;
                state    <= F_WAIT;
            end
            F_WAIT: begin
                // One wait state after consuming an event before the
                // fill re-reads that track. The track RAM read is
                // registered (rd = mem[ra] of the previous clock), and
                // the edge that writes the consumed word also samples
                // rd from the pre-write memory - so in F_LOOP the read
                // still shows the stale word (has_ev = 1) and F_RD
                // takes the "event already pending" shortcut, skipping
                // the fill. The track then never advances and the pick
                // finds nothing, ending the song after its first event.
                // With several tracks the other tracks' fills give the
                // read time to catch up, which is why format-1 files
                // played and single-track (format-0) files died after
                // one event. F_WAIT presents the fill address for one
                // more clock, so F_RD sees the consumed word.
                state <= F_LOOP;
            end

            // ---- send one byte, gated on baud spacing / step / buffers ----
            // step_done is the 1-clock pulse at the end of a core step, so
            // the byte is queued at the start of the next step (before its
            // Q_TIMERS UART update) - the same step boundary the bench
            // feeds the byte to the reference model at.
            S_SEND: begin
                if (baud_cnt == 14'd0 && step_done && !midi_full &&
                    !uart_valid && running) begin
                    midi_inject      <= 1'b1;
                    midi_inject_data <= send_byte;
                    baud_cnt <= BAUD_DIV - 14'd1;
                    state    <= send_ret;
                end
            end

            // ---- panic: All Sound Off (CC120) + Reset All Controllers
            // (CC121) + All Notes Off (CC123) on all 16 channels, 9 bytes
            // per channel. CC123 alone was not enough at a loop wrap:
            // it leaves release-phase voices sounding and controller
            // state (pitch bend, expression, hold) behind, so pass 2
            // started with hung notes and stale controllers. CC120
            // silences everything immediately and CC121 restores the
            // controller defaults; the song's own tick-0 events then
            // re-establish volume, pan and patch (CC121 does not reset
            // volume, pan or patch). 144 bytes at MIDI baud is ~46 ms.
            P_PANIC: begin
                panic_ch <= 4'd0;
                panic_ph <= 4'd0;
                state    <= P_PANIC_LOAD;
            end
            P_PANIC_LOAD: begin
                case (panic_ph)
                    4'd0, 4'd3, 4'd6: send_byte <= {4'hB, panic_ch};
                    4'd1: send_byte <= 8'h78;   // CC120 All Sound Off
                    4'd4: send_byte <= 8'h79;   // CC121 Reset All Controllers
                    4'd7: send_byte <= 8'h7B;   // CC123 All Notes Off
                    default: send_byte <= 8'h00; // value 0 for each CC
                endcase
                send_ret <= P_PANIC_ADV;
                state    <= S_SEND;
            end
            P_PANIC_ADV: begin
                if (panic_ph == 4'd8) begin
                    panic_ph <= 4'd0;
                    if (panic_ch == 4'd15) begin
                        if (panic_ret == P_PLAY_INIT) begin
                            playing <= 1'b1;
                            state   <= P_PLAY_INIT;
                        end else if (scan_pending) begin
                            // A good MID finished loading while this panic
                            // was silencing the old song: scan it now
                            // (auto-play when the scan finishes).
                            scan_pending <= 1'b0;
                            mode_scan    <= 1'b1;
                            play_pending <= 1'b1;
                            state        <= P_SCAN_INIT;
                        end else begin
                            playing <= 1'b0;
                            state   <= P_IDLE;
                        end
                    end else begin
                        panic_ch <= panic_ch + 4'd1;
                        state    <= P_PANIC_LOAD;
                    end
                end else begin
                    panic_ph <= panic_ph + 4'd1;
                    state    <= P_PANIC_LOAD;
                end
            end

            // ---- DDR3 byte reader ----
            RD_ISSUE: begin
                if (pready) state <= RD_WAIT;
            end
            RD_WAIT: begin
                if (prvalid) begin
                    rd_byte <= prdata;
                    state   <= rd_ret;
                end
            end

            default: begin
                state <= P_IDLE;
            end
            endcase
        end
    end
endmodule
