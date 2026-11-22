// SC-55 core with its board services, independent of MiSTer's framework:
// ROM pack download, wave ROMs in DDR3, real-time pacing, MIDI in from a
// UART, audio to 16 bit at the output rate. mister/SC55.sv adds hps_io, the
// PLL, video; sim/board/board_lockstep.cpp drives it against the reference.
//
// The core is held in reset until a pack with a valid header has loaded,
// and again during any later download; then the PCM clears its eram, the
// MCU takes its reset vector, and steps run while the pacer allows (and
// step_gate, for the bench).
module sc55_board #(
    parameter longint CLK_HZ = 50_000_000,
    parameter logic [31:0] WAVE_BASE = 32'h3000_0000
) (
    input  logic        clk,
    input  logic        clk_vid,          // 40 MHz: 800 x 600 video
    input  logic        rst,              // power-on / hard reset
    input  logic        user_reset,       // OSD reset: re-run the MCU reset

    input  logic        ioctl_download,
    input  logic        ioctl_wr,
    input  logic [26:0] ioctl_addr,
    input  logic [7:0]  ioctl_dout,
    input  logic [15:0] ioctl_index,     // F-button index (low 6 bits)
    output logic        ioctl_wait,

    input  logic        uart_rxd,
    input  logic        midi_inject,      // bench: a MIDI byte straight into the receive buffer
    input  logic [7:0]  midi_inject_data,
    input  logic [31:0] buttons,

    input  logic        pace_enable,
    input  logic [1:0]  gain,             // output gain: 0, +6, +12, +18 dB (after the PCM, saturating)
    input  logic        play_trig,        // T[4]: start MID playback (momentary)
    input  logic        stop_trig,        // T[5]: stop MID playback (momentary)
    input  logic        loop_en,          // O[6]: loop the MID
    input  logic [10:0] ps2_key,          // PS/2 key event (browser nav)
    input  logic        extract_start,    // pulse: extract table entry extract_idx
    input  logic [8:0]  extract_idx,      // ZIP entry-table index to extract
    input  logic [8:0]  tbl_ra,           // entry-table read address (browser)
    output logic [191:0] tbl_rd,          // entry-table read data (browser)
    input  logic        step_gate,        // 1 on hardware; the bench opens it one step at a time

    output logic signed [15:0] audio_l,
    output logic signed [15:0] audio_r,
    output logic        sample_valid,     // the PCM's raw output (bench)
    output logic signed [31:0] sample_l,
    output logic signed [31:0] sample_r,

    // video (clk_vid)
    output logic        vid_hs,
    output logic        vid_vs,
    output logic        vid_hblank,
    output logic        vid_vblank,
    output logic [7:0]  vid_r,
    output logic [7:0]  vid_g,
    output logic [7:0]  vid_b,
    output logic        lcd_pix,          // bench: an LCD pixel at (lcd_x, lcd_y) is on vid_r/g/b
    output logic [9:0]  lcd_x,
    output logic [8:0]  lcd_y,

    // status
    output logic        rom_ok,
    output logic        running,
    output logic        browser_active, // an archive is loaded: the file
                                        // browser owns the arrow keys
    output logic        step_done,
    output logic        midi_seen,        // a MIDI byte arrived (LED)
    output logic        midi_alt,         // receiving at 9600 baud
    output logic [31:0] lag_us,
    output logic [15:0] underruns,
    output logic [15:0] overruns,
    output logic [15:0] dbg_pc,
    output logic [7:0]  dbg_cp,

    input  logic        DDRAM_BUSY,
    output logic [7:0]  DDRAM_BURSTCNT,
    output logic [28:0] DDRAM_ADDR,
    input  logic [63:0] DDRAM_DOUT,
    input  logic        DDRAM_DOUT_READY,
    output logic        DDRAM_RD,
    output logic [63:0] DDRAM_DIN,
    output logic [7:0]  DDRAM_BE,
    output logic        DDRAM_WE
);
    // ---------------------------------------------------------- ROM load
    // Downloads are routed by ioctl_index[5:0]: boot.rom auto-load = 0,
    // F1/ROM = 1, F2/MID = 2, F3/ZIP = 3. A MID download never holds the
    // core in reset (rom_loader's `loading` is the ROM download only) and
    // can never clobber the wave ROMs or ROM2's upper half (mid_loader
    // writes only 0x440000..0x7FFFFF). The ZIP loader writes the archive
    // to 0x800000..0x2FFFFFF and extracts entries into the MID window.
    logic        ld_we, ld_wave_wr, ld_wave_pend, rom_wait, loading, ok;
    logic        rom_failed, mid_failed;
    logic        mid_wave_wr, mid_wait, mid_dl, mid_ok, mid_load_done;
    logic [2:0]  ld_sel;
    logic [18:0] ld_addr;
    logic [1:0]  model;                   // from the pack: 0 mk1, 1 SC-155, 2 mk2, 3 SC-155mk2
    wire         mk2 = model[1];
    wire         sc155 = model[0];
    logic [7:0]  ld_data, ld_wave_data, mid_wave_data, zip_wave_data;
    logic [22:0] ld_wave_addr, mid_wave_addr, mid_len;
    logic [25:0] zip_wave_addr;
    logic [15:0] mid_fmt, mid_ntracks, mid_ppq;
    logic        wave_wr, wave_busy;
    logic [25:0] wave_addr;
    logic [7:0]  wave_data;
    // ZIP archive loader (F3): download + central-directory parse +
    // entry extract into the MID window.
    logic        zip_wave_wr, zip_wait, zip_dl, zip_load_done, zip_mid_ok;
    logic        zip_mid_dl, zip_extract_failed, zip_extract_done;
    logic        zip_is_m3u, zip_idle;
    logic [2:0]  zip_error;
    logic [22:0] zip_mid_len;
    logic [15:0] zip_mid_ntracks, zip_mid_ppq;
    logic        zip_arch_ok, zip_arch_failed, zip_dl_failed, zip_parse_failed;
    logic        zip_dl_seen;   // latched: a ZIP download was seen (diagnostic)
    logic        zip_dl_done;   // latched: a ZIP download ended with >=22 bytes
    // Last download seen (any index) while the core is running (ok=1, so the
    // boot ROM load at index 0 does not count). Diagnostic: shows which
    // ioctl_index Main actually sent for the F3 transfer.
    logic        last_dl_seen;
    logic [5:0]  last_dl_idx;
    logic        ioctl_download_d;
    logic [25:0] zip_arch_len;
    logic [9:0]  zip_entry_count;
    logic        zip_areq, zip_aready, zip_arvalid;
    logic [25:0] zip_aaddr;
    logic [7:0]  zip_ardata;
    // .m3u parser + playlist sequencer (see below).
    logic        m3u_start, m3u_done, m3u_busy, m3u_ok, m3u_failed;
    logic [6:0]  m3u_count;
    logic [8:0]  m3u_tbl_ra;
    logic        m3u_areq, m3u_aready, m3u_arvalid;
    logic [25:0] m3u_aaddr;
    logic [7:0]  m3u_ardata;
    logic [8:0]  m3u_entry_idx_r;   // table index of the .m3u being parsed
    logic        seq_extract_start;
    logic [8:0]  seq_extract_idx;
    logic        player_song_done;
    logic [6:0]  seq_pl_ra;
    logic [8:0]  seq_pl_rd;
    logic [6:0]  br_pl_ra;
    logic [8:0]  br_pl_rd;
    logic        pl_active;         // a playlist is loaded and playing
    logic [6:0]  pl_current;        // index of the song now playing (0..99)
    logic [6:0]  pl_count_r;        // songs in the current playlist
    logic [8:0]  zip_extract_idx_r; // extract index, latched when the extract starts
    // File browser (keyboard + on-screen two-column list).
    logic        br_nav_up, br_nav_down, br_nav_left, br_nav_right, br_nav_enter;
    logic        br_start_scan, br_scan_done, br_busy, br_visible;
    logic        br_extract_start;
    logic [8:0]  br_extract_idx;
    logic [8:0]  br_tbl_ra;
    logic        br_areq, br_aready, br_arvalid;
    logic [25:0] br_aaddr;
    logic [7:0]  br_ardata;
    logic        br_pl_jump;
    logic [6:0]  br_pl_jump_idx;
    logic        br_char_we;
    logic [9:0]  br_char_wa;
    logic [7:0]  br_char_wd;
    logic [9:0]  vid_char_ra;
    logic [7:0]  vid_char_rd;
    logic        zip_arch_ok_d;     // delayed zip_arch_ok (rising-edge detect)
    logic        br_playlist_ready; // pulse: a playlist was just parsed

    rom_loader loader (
        .clk, .rst,
        .ioctl_download, .ioctl_wr, .ioctl_addr, .ioctl_dout, .ioctl_index,
        .ioctl_wait(rom_wait),
        .ld_we, .ld_sel, .ld_addr, .ld_data,
        .wave_wr(ld_wave_wr), .wave_addr(ld_wave_addr), .wave_data(ld_wave_data),
        .wave_busy(acc_rom), .wave_pend(ld_wave_pend), .model,
        .loading, .ok, .rom_failed
    );
    mid_loader midload (
        .clk, .rst,
        .ioctl_download, .ioctl_wr, .ioctl_index, .ioctl_dout,
        .rom_wave_pend(ld_wave_pend), .wave_busy(acc_mid),
        .ioctl_wait(mid_wait),
        .wave_wr(mid_wave_wr), .wave_addr(mid_wave_addr), .wave_data(mid_wave_data),
        .mid_dl, .load_done(mid_load_done), .load_ok(mid_ok), .mid_failed,
        .load_len(mid_len), .load_fmt(mid_fmt), .load_ntracks(mid_ntracks), .load_ppq(mid_ppq)
    );
    zip_load zipload (
        .clk, .rst,
        .ioctl_download, .ioctl_wr, .ioctl_index, .ioctl_dout,
        .mid_dl, .wave_busy(acc_zip),
        .ioctl_wait(zip_wait),
        .wave_wr(zip_wave_wr), .wave_addr(zip_wave_addr), .wave_data(zip_wave_data),
        .areq(zip_areq), .aaddr(zip_aaddr), .aready(zip_aready),
        .arvalid(zip_arvalid), .ardata(zip_ardata),
        .extract_start(zip_extract_start), .extract_idx(zip_extract_idx),
        .x_load_done(zip_load_done), .x_mid_ok(zip_mid_ok),
        .x_mid_len(zip_mid_len), .x_mid_fmt(),
        .x_mid_ntracks(zip_mid_ntracks), .x_mid_ppq(zip_mid_ppq),
        .x_mid_dl(zip_mid_dl), .x_extract_failed(zip_extract_failed),
        .x_extract_done(zip_extract_done), .x_is_m3u(zip_is_m3u), .x_error(zip_error),
        .arch_ok(zip_arch_ok), .arch_len(zip_arch_len),
        .entry_count(zip_entry_count), .arch_failed(zip_arch_failed),
        .dl_failed(zip_dl_failed), .parse_failed(zip_parse_failed),
        .dl_seen(zip_dl_seen), .dl_done(zip_dl_done),
        .zip_dl, .zip_idle, .tbl_ra(zip_tbl_ra), .tbl_rd(zip_tbl_rd)
    );
    // The extract trigger is shared by the browser (user action), the
    // board's extract_start input, and the playlist sequencer (which
    // advances on song_done). A direct user action wins a tie; the three
    // never fire together in practice. The table port is shared by
    // m3u_parse, the browser, and the board input: m3u_parse owns it
    // while it is parsing, then the browser, then the board input.
    wire zip_extract_start = extract_start | br_extract_start | seq_extract_start;
    wire [8:0] zip_extract_idx = extract_start ? extract_idx :
                                 br_extract_start ? br_extract_idx : seq_extract_idx;
    wire [8:0] zip_tbl_ra = m3u_busy ? m3u_tbl_ra : br_busy ? br_tbl_ra : tbl_ra;
    logic [191:0] zip_tbl_rd;
    assign tbl_rd = zip_tbl_rd;
    // .m3u playlist parser. It runs only while zip_load is idle, so it
    // shares zip_load's archive byte-read port and table port (muxed
    // below). It reads the .m3u text that zip_load extracted into the MID
    // window and resolves each song line to an entry-table index. The
    // playlist RAM has two read ports: the sequencer's and the browser's.
    m3u_parse m3uparse (
        .clk, .rst,
        .start(m3u_start), .m3u_len(zip_mid_len[22:0]), .m3u_entry_idx(m3u_entry_idx_r),
        .entry_count(zip_entry_count),
        .done(m3u_done), .busy(m3u_busy), .parse_ok(m3u_ok), .parse_failed(m3u_failed),
        .playlist_count(m3u_count),
        .tbl_ra(m3u_tbl_ra), .tbl_rd(zip_tbl_rd),
        .rd_req(m3u_areq), .rd_addr(m3u_aaddr), .rd_ready(m3u_aready),
        .rd_valid(m3u_arvalid), .rd_data(m3u_ardata),
        .pl_ra(seq_pl_ra), .pl_rd(seq_pl_rd),
        .pl_ra_b(br_pl_ra), .pl_rd_b(br_pl_rd)
    );
    // Playlist sequencer. When a .m3u finishes parsing with at least one
    // song, the playlist becomes active and the first song is extracted
    // (which auto-plays). When the player pulses song_done, the sequencer
    // extracts the next song; at the end it wraps to the start if Loop
    // (O[6]) is on, otherwise the playlist stops. Stop (T[5]), a direct
    // MID load (F2), or a manual browser extract ends the playlist.
    // The playlist RAM read is registered, so advancing takes two cycles:
    // present the index, latch the entry, then pulse the extract.
    typedef enum logic [1:0] { S_IDLE, S_RD, S_TRIG } seq_t;
    seq_t seq_state;
    logic [8:0] seq_entry_r;
    always_ff @(posedge clk) begin
        m3u_start <= 1'b0;
        seq_extract_start <= 1'b0;
        if (rst) begin
            seq_state <= S_IDLE;
            pl_active <= 1'b0;
            pl_current <= 7'd0;
            pl_count_r <= 7'd0;
            m3u_entry_idx_r <= 9'd0;
            zip_extract_idx_r <= 9'd0;
            seq_extract_idx <= 9'd0;
            seq_pl_ra <= 7'd0;
            seq_entry_r <= 9'd0;
        end else begin
            // Latch the extract index when the extract starts. The muxed
            // zip_extract_idx is only valid during the start pulse (the
            // browser's extract_start is a 1-cycle pulse); by the time the
            // extract completes, the mux has switched to the sequencer's
            // (stale) seq_extract_idx, so sampling it then is wrong.
            if (zip_extract_start) zip_extract_idx_r <= zip_extract_idx;
            // A .m3u extract finished: parse it. (The .m3u text is in the
            // MID window; its length is zip_mid_len, its table index is
            // the one the extract used, latched above.)
            if (zip_extract_done && zip_is_m3u && !zip_extract_failed) begin
                pl_active <= 1'b0;   // old playlist stops; the new one starts below
                m3u_entry_idx_r <= zip_extract_idx_r;
                m3u_start <= 1'b1;
            end
            // A manual browser extract or a direct MID load ends the playlist.
            if (extract_start || br_extract_start || mid_load_done) pl_active <= 1'b0;
            // Stop ends the playlist.
            if (stop_trig) pl_active <= 1'b0;
            case (seq_state)
                S_IDLE: begin
                    if (m3u_done && m3u_ok) begin
                        // New playlist: start at song 0.
                        pl_active <= 1'b1;
                        pl_current <= 7'd0;
                        pl_count_r <= m3u_count;
                        seq_pl_ra <= 7'd0;
                        seq_state <= S_RD;
                    end else if (br_pl_jump && pl_active) begin
                        // User picked a song in the right column.
                        pl_current <= br_pl_jump_idx;
                        seq_pl_ra <= br_pl_jump_idx;
                        seq_state <= S_RD;
                    end else if (player_song_done && pl_active) begin
                        if (pl_current + 7'd1 < pl_count_r) begin
                            pl_current <= pl_current + 7'd1;
                            seq_pl_ra <= pl_current + 7'd1;
                            seq_state <= S_RD;
                        end else if (loop_en) begin
                            pl_current <= 7'd0;
                            seq_pl_ra <= 7'd0;
                            seq_state <= S_RD;
                        end else begin
                            pl_active <= 1'b0;
                        end
                    end
                end
                S_RD: begin
                    // The playlist RAM read is registered: pl_rd is valid
                    // the cycle after pl_ra is presented.
                    seq_state <= S_TRIG;
                end
                S_TRIG: begin
                    seq_entry_r <= seq_pl_rd;
                    seq_extract_idx <= seq_pl_rd;
                    seq_extract_start <= 1'b1;
                    seq_state <= S_IDLE;
                end
                default: seq_state <= S_IDLE;
            endcase
        end
    end
    // File browser: keyboard decoder + scan/navigation + character
    // buffer. The browser owns the table/archive ports while it scans
    // (br_busy); it only scans when zip_load and m3u_parse are idle.
    // start_scan pulses when the archive first parses (zip_arch_ok
    // rises). playlist_ready pulses when a playlist finishes parsing.
    browser_kbd brkbd (
        .clk, .rst, .ps2_key,
        .nav_up(br_nav_up), .nav_down(br_nav_down),
        .nav_left(br_nav_left), .nav_right(br_nav_right),
        .nav_enter(br_nav_enter)
    );
    browser brwsr (
        .clk, .rst,
        .nav_up(br_nav_up), .nav_down(br_nav_down),
        .nav_left(br_nav_left), .nav_right(br_nav_right),
        .nav_enter(br_nav_enter),
        .arch_ok(zip_arch_ok), .entry_count(zip_entry_count),
        .zip_idle, .m3u_idle(!m3u_busy),
        .start_scan(br_start_scan),
        .tbl_ra(br_tbl_ra), .tbl_rd(zip_tbl_rd),
        .rd_req(br_areq), .rd_addr(br_aaddr), .rd_ready(br_aready),
        .rd_valid(br_arvalid), .rd_data(br_ardata),
        .busy(br_busy),
        .extract_start(br_extract_start), .extract_idx(br_extract_idx),
        .playlist_count(m3u_count), .playlist_active(pl_active),
        .pl_current, .pl_rd(br_pl_rd), .pl_ra(br_pl_ra),
        .playing(player_playing),
        .pl_jump(br_pl_jump), .pl_jump_idx(br_pl_jump_idx),
        .char_we(br_char_we), .char_wa(br_char_wa), .char_wd(br_char_wd),
        .playlist_ready(br_playlist_ready),
        .visible(br_visible), .scan_done(br_scan_done),
        .left_count_o(), .left_cursor_o(), .active_right()
    );
    dc_ram #(.AW(10), .DW(8)) char_ram (
        .wr_clk(clk), .wr_en(br_char_we), .wr_addr(br_char_wa), .wr_data(br_char_wd),
        .rd_clk(clk_vid), .rd_addr(vid_char_ra), .rd_data(vid_char_rd));
    // Rising-edge detects: archive loaded, and a playlist parsed.
    // Also latches the index of the last download seen while the core is
    // running (diagnostic: which ioctl_index Main sent for the F3 transfer).
    always_ff @(posedge clk) begin
        zip_arch_ok_d <= zip_arch_ok;
        ioctl_download_d <= ioctl_download;
        br_start_scan <= 1'b0;
        br_playlist_ready <= 1'b0;
        if (rst) begin
            zip_arch_ok_d <= 1'b0;
            ioctl_download_d <= 1'b0;
            last_dl_seen <= 1'b0;
            last_dl_idx <= 6'd0;
        end else begin
            if (zip_arch_ok && !zip_arch_ok_d) br_start_scan <= 1'b1;
            if (m3u_done && m3u_ok) br_playlist_ready <= 1'b1;
            if (ioctl_download && !ioctl_download_d && ok) begin
                last_dl_seen <= 1'b1;
                last_dl_idx <= ioctl_index[5:0];
            end
        end
    end
    // The DDR3 write port is shared by three loaders. Priority ROM > ZIP >
    // MID (ROM's write is a 1-clock pulse, so it must win when it fires).
    // acc_sel latches which loader's byte wave_ddr accepted, so each loader
    // is told only about its own accept: a loader never pops its FIFO for
    // another loader's write (the old code used the global wave_busy, which
    // rises for any accepted write). wave_ddr accepts a write only when no
    // command is in flight (DDRAM_RD/WE low) and wbusy is low.
    logic [1:0] acc_sel;
    wire [1:0] wr_sel = ld_wave_wr ? 2'd0 : zip_wave_wr ? 2'd1 : mid_wave_wr ? 2'd2 : 2'd0;
    wire acc_rom = wave_busy && (acc_sel == 2'd0);
    wire acc_zip = wave_busy && (acc_sel == 2'd1);
    wire acc_mid = wave_busy && (acc_sel == 2'd2);
    wire wave_accept = wave_wr && !wave_busy && !DDRAM_RD && !DDRAM_WE;
    always_ff @(posedge clk) begin
        if (rst) acc_sel <= 2'd0;
        else if (wave_accept) acc_sel <= wr_sel;
    end
    assign wave_wr   = ld_wave_wr | zip_wave_wr | mid_wave_wr;
    assign wave_addr = (wr_sel == 2'd0) ? {3'b0, ld_wave_addr} :
                       (wr_sel == 2'd1) ? zip_wave_addr : {3'b0, mid_wave_addr};
    assign wave_data = (wr_sel == 2'd0) ? ld_wave_data :
                       (wr_sel == 2'd1) ? zip_wave_data : mid_wave_data;
    assign ioctl_wait = rom_wait | mid_wait | zip_wait;
    assign rom_ok = ok;
    assign browser_active = zip_arch_ok;
    // The player takes its MID from either a direct download or an extract:
    // whichever load_done pulsed selects that source's file info (mid_ok is
    // only sampled on load_done, and the two pulses never coincide).
    wire player_load_done = mid_load_done | zip_load_done;
    wire player_mid_ok = zip_load_done ? zip_mid_ok : mid_ok;
    wire [22:0] player_mid_len = zip_load_done ? zip_mid_len : mid_len;
    wire [15:0] player_mid_ntracks = zip_load_done ? zip_mid_ntracks : mid_ntracks;
    wire [15:0] player_mid_ppq = zip_load_done ? zip_mid_ppq : mid_ppq;
    wire player_mid_dl = mid_dl | zip_mid_dl;

    // ---------------------------------------------------------- sequencing
    typedef enum logic [2:0] { B_HOLD, B_CLEAR, B_RESET, B_RUN } bst_t;
    bst_t bst;
    logic core_rst, reset_start, reset_done, core_ready, run_step;

    always_ff @(posedge clk) begin
        reset_start <= 1'b0;
        if (rst || loading || !ok || user_reset) begin
            bst <= B_HOLD;
            core_rst <= 1'b1;
        end else begin
            case (bst)
                B_HOLD:  begin core_rst <= 1'b0; bst <= B_CLEAR; end
                B_CLEAR: if (core_ready) begin reset_start <= 1'b1; bst <= B_RESET; end
                B_RESET: if (reset_done) bst <= B_RUN;
                default: ;
            endcase
        end
    end
    assign running = (bst == B_RUN);

    // ---------------------------------------------------------- pacing
    logic allow;
    step_pacer #(.CLK_HZ(CLK_HZ), .STEP_NS(600), .STEP_NS_FAST(500)) pacer (
        .clk, .rst(core_rst), .enable(pace_enable), .fast(mk2), .step_done, .allow, .lag_us
    );
    assign run_step = running && allow && step_gate;

    // ---------------------------------------------------------- MIDI
    logic       rx_valid;
    logic [7:0] rx_data;
    midi_rx #(.CLK_HZ(CLK_HZ)) midi (
        .clk, .rst, .rxd(uart_rxd), .byte_valid(rx_valid), .byte_data(rx_data), .frame_err(), .alt(midi_alt)
    );
    // The built-in MID player injects through the same receive buffer as
    // the UART; the bench's midi_inject is the other source. The player
    // wins if both fire in the same clock (they never do in practice).
    logic        player_inject, player_playing, player_scan_failed, core_midi_full;
    logic [7:0]  player_inject_data;
    logic        player_preq, player_pready, player_prvalid;
    logic [22:0] player_paddr;
    logic [7:0]  player_prdata;
    midi_player player (
        .clk, .rst, .running, .step_done,
        .mid_ok(player_mid_ok), .mid_len(player_mid_len),
        .mid_ntracks(player_mid_ntracks), .mid_ppq(player_mid_ppq),
        .load_done(player_load_done), .mid_dl(player_mid_dl),
        .play_trig, .stop_trig, .loop_en,
        .preq(player_preq), .paddr(player_paddr), .pready(player_pready),
        .prvalid(player_prvalid), .prdata(player_prdata),
        .midi_inject(player_inject), .midi_inject_data(player_inject_data),
        .midi_full(core_midi_full), .uart_valid(rx_valid),
        .playing(player_playing), .scan_failed(player_scan_failed),
        .song_done(player_song_done)
    );
    wire       m_inject = midi_inject || player_inject;
    wire [7:0] m_inject_data = player_inject ? player_inject_data : midi_inject_data;
    wire       m_valid = rx_valid || m_inject;
    wire [7:0] m_data  = m_inject ? m_inject_data : rx_data;
    always_ff @(posedge clk) midi_seen <= m_valid;

    // ---------------------------------------------------------- core
    logic        rom_req, rom_ready, rom_rvalid;
    logic [21:0] rom_addr;
    logic [7:0]  rom_data;

    sc55_core core (
        .clk, .rst(core_rst),
        .run_step, .step_done, .reset_start, .reset_done, .ready(core_ready),
        .ld_we, .ld_sel, .ld_addr, .ld_data, .mk2, .sc155,
        .rom2x_req, .rom2x_addr, .rom2x_ack, .rom2x_data,
        .midi_valid(m_valid && running), .midi_data(m_data), .midi_full(core_midi_full), .buttons,
        .rom_req, .rom_addr, .rom_ready, .rom_data, .rom_rvalid,
        .enable_oversampling(1'b1),
        .sample_valid, .sample_l, .sample_r,
        .lcd_we(l_we), .lcd_rs(l_rs), .lcd_data(l_data), .lcd_enable(l_en),
        .cycles(), .dbg_pc, .dbg_cp, .dbg_sr()
    );

    // ---------------------------------------------------------- LCD
    logic       l_we, l_rs, l_en, lcd_on;
    logic [7:0] l_data;
    logic [7:0] dd [80];
    logic [4:0] cg [64];
    lcd_ctrl lcd (
        .clk, .rst(core_rst), .we(l_we), .rs(l_rs), .data(l_data), .enable_in(l_en),
        .dd, .cg, .enable(lcd_on), .dd_ram(), .c_flag()
    );

    // status for the strip under the LCD (shown only): MIDI and underrun
    // flashes; underrun since the core started (sticky); real-time lag that
    // grew after the first second of running (sticky; the reset and memory
    // clear before running always count some)
    logic [23:0] midi_stretch, ur_stretch;
    logic [15:0] last_ur;
    logic [25:0] run_time;
    logic [31:0] lag_base;
    logic        lag_seen;
    always_ff @(posedge clk) begin
        midi_stretch <= midi_seen ? 24'hffffff : (midi_stretch != 0 ? midi_stretch - 24'd1 : 24'd0);
        last_ur <= underruns;
        ur_stretch <= (underruns != last_ur) ? 24'hffffff : (ur_stretch != 0 ? ur_stretch - 24'd1 : 24'd0);
        if (!running) begin
            run_time <= '0;
            lag_seen <= 1'b0;
        end else if (run_time != 26'(CLK_HZ)) begin
            run_time <= run_time + 26'd1;
            lag_base <= lag_us;
        end else if (lag_us != lag_base) lag_seen <= 1'b1;
    end
    // Status text for the line to the right of the strip (lcd_video):
    // highest priority first. Errors are latched in the loaders until the
    // next download of that kind starts or succeeds. ZIP failures get
    // their own messages so a bad archive load names the stage that
    // failed (download / parse / extract) instead of a generic MID error.
    // Diagnostic messages (11, 12-15, 16, 18, 19) are 5-bit; they name
    // the stage a silent .mzp load reached, so one hardware test shows
    // exactly where the F3 transfer stops. They only fire while no
    // archive is parsed (or while a ZIP download is live), so they never
    // mask PLAYING/RUNNING once an archive is up.
    //   0 none, 1 ROM LOAD FAILED, 2 MID LOAD FAILED, 3 LOADING ROM,
    //   4 NO ROM PACK, 5 PLAYING, 6 RUNNING, 7 SCAN FAILED,
    //   8 ZIP DL FAILED, 9 ZIP PARSE FAIL, 10 ZIP EXT FAILED,
    //   11 ZIP DL SEEN, 12 DL IDX 0, 13 DL IDX 1, 14 DL IDX 2, 15 DL IDX ?,
    //   16 ZIP DL DONE, 18 ARCH OK, 19 LOADING ZIP
    logic [4:0] status_msg;
    always_comb begin
        if (rom_failed)        status_msg = 5'd1;
        else if (loading)      status_msg = 5'd3;
        else if (zip_dl_failed) status_msg = 5'd8;
        else if (zip_parse_failed) status_msg = 5'd9;
        else if (zip_extract_failed) status_msg = 5'd10;
        else if (mid_failed)   status_msg = 5'd2;
        else if (player_scan_failed) status_msg = 5'd7;
        else if (!ok)          status_msg = 5'd4;
        else if (zip_dl)       status_msg = 5'd19;
        else if (zip_dl_done && !zip_arch_ok) status_msg = 5'd16;
        else if (zip_dl_seen && !zip_arch_ok) status_msg = 5'd11;
        else if (last_dl_seen && !player_playing && !zip_arch_ok && (last_dl_idx != 6'd3)) begin
            if (last_dl_idx == 6'd0) status_msg = 5'd12;
            else if (last_dl_idx == 6'd1) status_msg = 5'd13;
            else if (last_dl_idx == 6'd2) status_msg = 5'd14;
            else status_msg = 5'd15;
        end
        else if (zip_arch_ok)  status_msg = 5'd18;
        else if (player_playing) status_msg = 5'd5;
        else if (running)      status_msg = 5'd6;
        else                   status_msg = 5'd0;
    end
    lcd_video video (
        .clk(clk_vid), .dd, .cg, .enable(lcd_on),
        .rom_ok, .loading, .running, .playing(player_playing),
        .midi_led(midi_stretch != 0), .underrun_led(ur_stretch != 0),
        .underrun_seen(underruns != 16'd0), .lag_seen,
        .msg(status_msg),
        .browser_visible(br_visible), .char_rd(vid_char_rd), .char_ra(vid_char_ra),
        .hs(vid_hs), .vs(vid_vs), .hblank(vid_hblank), .vblank(vid_vblank), .r(vid_r), .g(vid_g), .b(vid_b),
        .lcd_pix, .lcd_x, .lcd_y
    );

    // ROM2's upper half (mk2): DDR3 through a small cache
    logic        rom2x_req, rom2x_ack, ln_req, ln_valid;
    logic [17:0] rom2x_addr;
    logic [7:0]  rom2x_data;
    logic [14:0] ln_addr;
    logic [63:0] ln_data;
    rom2x_cache cache (
        .clk, .flush(rst || loading), .req(rom2x_req), .addr(rom2x_addr), .ack(rom2x_ack), .data(rom2x_data),
        .lreq(ln_req), .laddr(ln_addr), .lvalid(ln_valid), .ldata(ln_data)
    );

    // The archive byte-read port is shared by zip_load (parse/extract),
    // m3u_parse (playlist resolution), and the browser (scan + song-name
    // load). Priority m3u_parse > browser > zip_load. They never run at
    // the same time: m3u_parse only starts after an extract finishes, the
    // browser only scans while zip_load and m3u_parse are idle, and no
    // new extract starts while the browser or m3u_parse is busy. The
    // owner sees the ready/valid handshake; the others see it deasserted.
    // All three see the data byte; only the owner acts on it.
    wire arch_areq = m3u_busy ? m3u_areq : br_busy ? br_areq : zip_areq;
    wire [25:0] arch_aaddr = m3u_busy ? m3u_aaddr : br_busy ? br_aaddr : zip_aaddr;
    wire arch_aready, arch_arvalid;
    wire [7:0] arch_ardata;
    assign zip_aready = !m3u_busy && !br_busy && arch_aready;
    assign m3u_aready = m3u_busy && arch_aready;
    assign br_aready = !m3u_busy && br_busy && arch_aready;
    assign zip_arvalid = !m3u_busy && !br_busy && arch_arvalid;
    assign m3u_arvalid = m3u_busy && arch_arvalid;
    assign br_arvalid = !m3u_busy && br_busy && arch_arvalid;
    assign zip_ardata = arch_ardata;
    assign m3u_ardata = arch_ardata;
    assign br_ardata = arch_ardata;

    wave_ddr #(.BASE(WAVE_BASE)) wave (
        .clk, .rst,
        .wr(wave_wr), .waddr(wave_addr), .wdata(wave_data), .wbusy(wave_busy),
        .req(rom_req), .addr(rom_addr), .ready(rom_ready), .rvalid(rom_rvalid), .rdata(rom_data),
        .preq(player_preq), .paddr(player_paddr), .pready(player_pready),
        .prvalid(player_prvalid), .prdata(player_prdata),
        .areq(arch_areq), .aaddr(arch_aaddr), .aready(arch_aready),
        .arvalid(arch_arvalid), .ardata(arch_ardata),
        .lreq(ln_req), .laddr(ln_addr), .lvalid(ln_valid), .ldata(ln_data),
        .DDRAM_BUSY, .DDRAM_BURSTCNT, .DDRAM_ADDR, .DDRAM_DOUT, .DDRAM_DOUT_READY,
        .DDRAM_RD, .DDRAM_DIN, .DDRAM_BE, .DDRAM_WE
    );

    // ---------------------------------------------------------- audio
    // the renderer's s16 conversion (Normalize: clamp(x >> 15)), 64 -> 48 kHz
    // (resample.sv), then a FIFO that plays at 48 kHz in real time
    // with `gain` in 6 dB steps (like the reference front ends' --gain): clamp(x >> (15 - gain))
    function automatic logic signed [15:0] to16(input logic signed [31:0] v, input logic [1:0] g);
        logic signed [31:0] s;
        s = v >>> (5'd15 - {3'd0, g});
        if (s > 32'sd32767) return 16'sh7fff;
        if (s < -32'sd32768) return 16'sh8000;
        return s[15:0];
    endfunction

    logic        rs_valid;
    logic signed [15:0] rs_l, rs_r;
    resample rs (
        .clk, .rst(core_rst), .mk2,
        .in_valid(sample_valid), .in_l(to16(sample_l, gain)), .in_r(to16(sample_r, gain)),
        .out_valid(rs_valid), .out_l(rs_l), .out_r(rs_r)
    );

    audio_fifo #(.CLK_HZ(CLK_HZ), .RATE(48_000), .DEPTH_BITS(8)) fifo (
        .clk, .rst(core_rst),
        .in_valid(rs_valid), .in_l(rs_l), .in_r(rs_r),
        .out_l(audio_l), .out_r(audio_r), .out_strobe(),
        .underruns, .overruns
    );
endmodule
