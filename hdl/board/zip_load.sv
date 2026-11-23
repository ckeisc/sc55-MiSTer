// SPDX-License-Identifier: GPL-2.0-or-later
// zip_load.sv — ZIP archive loader + entry extractor for the SC-55 MIDI player.
//
// A ZIP archive (CONF_STR "F3,ZIP", Main ioctl_index 3) is downloaded into
// the free DDR3 window at 0x800000 (up to 40 MB), then its central
// directory is parsed (zip_dir) into a 512-entry on-chip table. An entry
// is extracted on demand (extract_start + extract_idx): its local header
// is read, the data is inflated (method 8, inflate.sv) or copied
// (method 0) into the MID window at 0x440000, the CRC-32 and length are
// verified, and the result is handed to midi_player exactly like a direct
// MID download (x_load_done + x_mid_*). Methods other than 0 and 8 are
// refused (Deflate64 (9) by policy). The entry table is readable by the
// playlist/browser through tbl_ra/tbl_rd.
//
// Main sends blind and never polls ioctl_wait, so the download and the
// extract both buffer bytes in a small FIFO and hold each DDR3 write until
// wave_ddr accepts it (the same held-write pattern as mid_loader). The
// archive is read back through wave_ddr's archive byte-read port, shared
// between the parser and the extractor (they never run at the same time).
// The ZIP download and the extract both write DDR3; the board arbitrates
// the shared write port with per-loader accept signals (see sc55_board),
// so a byte is only popped from a loader's FIFO when that loader's write
// was the one accepted.

module zip_load (
    input  logic        clk,
    input  logic        rst,
    // ZIP download (ioctl index 3)
    input  logic        ioctl_download,
    input  logic        ioctl_wr,
    input  logic [15:0] ioctl_index,
    input  logic [7:0]  ioctl_dout,
    input  logic        mid_dl,         // a MID download is in progress (abort extract)
    input  logic        wave_busy,      // wave_ddr write accept for THIS loader
    output logic        ioctl_wait,
    output logic        wave_wr,        // DDR3 write (ZIP download or extract)
    output logic [25:0] wave_addr,      // 26-bit byte address
    output logic [7:0]  wave_data,
    // Archive byte-read port (wave_ddr archive port)
    output logic        areq,
    output logic [25:0] aaddr,
    input  logic        aready,
    input  logic        arvalid,
    input  logic [7:0]  ardata,
    // Extract trigger (playlist/browser; TB for now)
    input  logic        extract_start,  // pulse
    input  logic [8:0]  extract_idx,    // entry index
    // Extract -> player (muxed in board with mid_loader's outputs)
    output logic        x_load_done,    // pulse: extracted MID ready
    output logic        x_mid_ok,
    output logic [22:0] x_mid_len,
    output logic [15:0] x_mid_fmt,
    output logic [15:0] x_mid_ntracks,
    output logic [15:0] x_mid_ppq,
    output logic        x_mid_dl,       // extract in progress
    output logic        x_extract_failed, // latched
    output logic        x_extract_done,   // pulse: extract finished
    output logic        x_is_m3u,         // the extracted entry is a .m3u
                                          // (valid with x_extract_done)
    output logic [2:0]  x_error,          // 0 none, 1 bad entry, 2 bad local hdr,
                                          // 3 inflate error, 4 CRC/len mismatch,
                                          // 5 header check fail, 6 aborted
    // Archive status
    output logic        arch_ok,
    output logic [25:0] arch_len,
    output logic [9:0]  entry_count,
    output logic        arch_failed,    // latched (any archive failure)
    output logic        dl_failed,      // latched: download failed (too small /
                                        // over 40 MB / FIFO overflow)
    output logic        parse_failed,   // latched: central-directory parse
                                        // failed (no EOCD / bad CD / >512 entries)
    output logic        dl_seen,        // latched: a ZIP download (index 3)
                                        // was seen at least once since reset.
                                        // Diagnostic: distinguishes "download
                                        // never reached the core" from
                                        // "download reached core but failed"
    output logic        dl_done,        // latched: a ZIP download ended with
                                        // >=22 bytes and no overflow (i.e. the
                                        // bytes all arrived; the parse may
                                        // still be running or have failed).
                                        // Diagnostic: distinguishes "download
                                        // started but never finished" from
                                        // "download finished, parse is next"
    output logic        zip_dl,         // ZIP download in progress
    output logic        zip_idle,       // not downloading, parsing, or extracting
    // Parse-stage diagnostics (latched; cleared on reset and on a new
    // download start): how far the central-directory parse got. The
    // ladder is started -> rd accepted -> data returned -> EOCD found
    // -> CD walk started -> arch_ok. A stuck parse shows the highest
    // stage reached, so one hardware test names the stall point.
    output logic        parse_started,  // zip_dir left S_IDLE
    output logic        parse_rd,       // an archive read was accepted
    output logic        parse_data,     // an archive read returned data
    output logic        parse_eocd,     // EOCD signature found
    output logic        parse_cd,       // central-directory walk began
    // Entry-table read port (playlist/browser)
    input  logic [8:0]  tbl_ra,
    output logic [191:0] tbl_rd
);
    // DDR3 layout (byte offsets in the core's DDR3 view): waves
    // 0x000000..0x3FFFFF, ROM2 upper half 0x400000..0x43FFFF, MID window
    // 0x440000..0x7FFFFF, ZIP archive 0x800000..0x2FFFFFF (40 MB max).
    localparam [25:0] ARCH_BASE = 26'h800000;
    localparam [25:0] ARCH_MAX  = 26'h2800000;  // 40 MB
    localparam [22:0] MID_BASE  = 23'h440000;
    localparam [22:0] MID_MAX   = 23'h3C0000;   // ~3.75 MiB (same as mid_loader)

    wire is_zip = ioctl_download && (ioctl_index[5:0] == 6'd3);
    assign zip_dl = is_zip;
    // Idle when no download, no central-directory parse, and no extract
    // is running (the browser only scans while this is high).
    assign zip_idle = !is_zip && (pstate == P_IDLE) && (xstate == X_IDLE);

    // ------------------------------------------------------------ CRC-32
    // IEEE reflected (poly 0xEDB88320), same as inflate.sv. crc starts at
    // 32'hFFFFFFFF; the final CRC is ~crc.
    function automatic logic [31:0] crc32_byte(input logic [31:0] crc, input logic [7:0] b);
        logic [31:0] c;
        integer i;
        begin
            c = crc ^ {24'd0, b};
            for (i = 0; i < 8; i = i + 1)
                c = c[0] ? (c >> 1) ^ 32'hEDB88320 : (c >> 1);
            crc32_byte = c;
        end
    endfunction

    // ------------------------------------------------------------ write FIFO
    // 4-deep FIFO of pending DDR3 byte writes {addr[25:0], data[7:0]}.
    // Shared by the ZIP download (archive region) and the extract (MID
    // window); the two never run at the same time. Held until accepted:
    // Main sends blind, so a 1-clock pulse would be dropped whenever the
    // port was busy serving reads (the second-MID-load bug). The FIFO is
    // 32 deep so a multi-MB archive burst from Main cannot overflow it
    // (a 4-deep FIFO was enough for small MIDs but not for a 3 MB ZIP).
    logic [33:0] wfifo [0:31];
    logic [5:0]  wfifo_wp, wfifo_rp;
    wire         wfifo_empty = (wfifo_wp == wfifo_rp);
    wire         wfifo_full  = ((wfifo_wp - wfifo_rp) == 6'd32);
    logic        wr_active;
    logic        dl_d;
    logic        over;           // archive longer than ARCH_MAX
    logic        fifo_over;      // a download byte arrived with the FIFO full
    logic        end_pend;       // download ended, FIFO still draining
    logic [25:0] count;          // archive bytes received
    logic [25:0] arch_len_r;

    // Push sources (mutually exclusive by phase/method): the download, the
    // inflate output, and the stored-copy byte. The FIFO has one write
    // port, so they are OR'd into a single push.
    logic        x_extracting;   // the extract FSM is not idle
    logic [15:0] method_r;       // entry method under extraction
    logic        is_m3u_r;       // entry under extraction is a .m3u
    assign x_is_m3u = is_m3u_r;
    wire         push_dl   = is_zip && ioctl_wr && !over && (count < ARCH_MAX) && !wfifo_full;
    wire         push_inf  = inf_out_valid && inf_out_ready;
    wire         push_copy = (xstate == X_COPY_PUSH) && !wfifo_full;
    wire         push_any  = push_dl || push_inf || push_copy;
    wire [25:0]  push_addr = push_dl ? (ARCH_BASE + count) : ({3'b0, MID_BASE} + {3'b0, out_count[22:0]});
    wire [7:0]   push_data = push_dl ? ioctl_dout : push_inf ? inf_out_data : copy_byte;
    logic [22:0] out_count;      // MID bytes written (extract)
    logic [31:0] copy_crc;       // running CRC for the stored-copy path
    logic [7:0]  copy_byte;
    logic [7:0]  hdr [0:13];     // first 14 bytes of the extracted MID

    assign ioctl_wait = !wfifo_empty || wr_active || wave_busy || wave_wr;

    // ------------------------------------------------------------ parse
    typedef enum logic [1:0] { P_IDLE, P_RUN } pstate_t;
    pstate_t     pstate;
    logic        zip_start;      // pulse to (re)start zip_dir
    logic [9:0]  tbl_count;      // entries stored (0..512)
    logic        tbl_over;       // more than 512 entries

    // zip_dir byte-read port (forwarded to the archive DDR3 port while parsing)
    logic        zip_rd_req;
    logic [25:0] zip_rd_addr;
    logic [7:0]  zip_rd_data;
    logic        zip_rd_valid;
    logic        zip_entry_valid;
    logic [25:0] zip_entry_name_ptr;
    logic [15:0] zip_entry_name_len;
    logic [15:0] zip_entry_method;
    logic [31:0] zip_entry_crc;
    logic [31:0] zip_entry_comp_size;
    logic [31:0] zip_entry_uncomp_size;
    logic [31:0] zip_entry_local_hdr_off;
    logic        zip_entry_is_dir;
    logic        zip_entry_is_m3u;
    logic [15:0] zip_entry_count;
    logic        zip_done;
    logic [2:0]  zip_error;
    logic        zip_started, zip_eocd, zip_cd;

    zip_dir parser (
        .clk, .rst,
        .start(zip_start),
        .arch_len(arch_len_r),
        .rd_req(zip_rd_req), .rd_addr(zip_rd_addr),
        .rd_data(zip_rd_data), .rd_valid(zip_rd_valid),
        .entry_valid(zip_entry_valid),
        .entry_name_ptr(zip_entry_name_ptr),
        .entry_name_len(zip_entry_name_len),
        .entry_method(zip_entry_method),
        .entry_crc(zip_entry_crc),
        .entry_comp_size(zip_entry_comp_size),
        .entry_uncomp_size(zip_entry_uncomp_size),
        .entry_local_hdr_off(zip_entry_local_hdr_off),
        .entry_is_dir(zip_entry_is_dir),
        .entry_is_m3u(zip_entry_is_m3u),
        .entry_count(zip_entry_count),
        .done(zip_done), .error(zip_error),
        .started(zip_started), .eocd_found(zip_eocd), .cd_started(zip_cd)
    );
    // Parse-stage flags are just the parser's latched stage outputs plus
    // the read handshake (accepted = areq && aready while parsing;
    // returned = arvalid while a parse read is in flight).
    assign parse_started = zip_started;
    assign parse_eocd = zip_eocd;
    assign parse_cd = zip_cd;

    // ------------------------------------------------------------ entry table
    // One 192-bit word per entry (512 entries, sdp_ram block RAM):
    //   [25:0]   name_ptr, [41:26] name_len, [57:42] method,
    //   [89:58]  crc, [121:90] comp_size, [153:122] uncomp_size,
    //   [185:154] local_hdr_off, [186] is_dir, [187] is_m3u, [191:188] 0.
    logic        tbl_we;
    logic [8:0]  tbl_wa;
    logic [191:0] tbl_wd;
    logic [8:0]  tbl_ra_mux;
    assign tbl_we = (pstate == P_RUN) && zip_entry_valid && (tbl_count < 10'd512);
    assign tbl_wa = tbl_count[8:0];
    assign tbl_wd = {4'b0, zip_entry_is_m3u, zip_entry_is_dir,
                     zip_entry_local_hdr_off, zip_entry_uncomp_size,
                     zip_entry_comp_size, zip_entry_crc, zip_entry_method,
                     zip_entry_name_len, zip_entry_name_ptr};
    // The extract owns the table read port while it is reading an entry;
    // otherwise the playlist/browser drives it.
    assign tbl_ra_mux = (xstate == X_TBL_RD || xstate == X_TBL_GOT) ? extract_idx_r : tbl_ra;
    sdp_ram #(.AW(9), .DW(192)) tbl_ram (
        .clk, .we(tbl_we), .wa(tbl_wa), .wd(tbl_wd),
        .ra(tbl_ra_mux), .rd(tbl_rd));

    // ------------------------------------------------------------ extract
    typedef enum logic [4:0] {
        X_IDLE,
        X_TBL_RD, X_TBL_GOT,
        X_LH_ISSUE, X_LH_WAIT,
        X_FEED_ISSUE, X_FEED_WAIT, X_FEED_HOLD,
        X_COPY_ISSUE, X_COPY_WAIT, X_COPY_PUSH,
        X_DRAIN, X_DRAIN_COPY, X_VERIFY, X_ERR
    } xstate_t;
    xstate_t     xstate;
    logic [8:0]  extract_idx_r;
    logic [31:0] crc_r, comp_r, uncomp_r, lh_off_r;
    logic [25:0] data_off;       // archive offset of the entry's data
    logic [5:0]  lh_idx;         // local-header byte index (0..29)
    logic [31:0] lh_sig;         // local-header signature accumulator
    logic [15:0] lh_name_len, lh_extra_len;
    logic [31:0] fed_count;      // compressed bytes fed to inflate
    logic [31:0] copied_count;   // stored bytes copied
    logic        inf_start;      // pulse to start inflate
    logic [7:0]  feed_byte;
    logic        inf_valid_r;    // a feed byte is presented to inflate
    logic [2:0]  x_err_r;

    assign x_extracting = (xstate != X_IDLE);
    assign x_mid_dl = x_extracting;
    assign arch_len = arch_len_r;
    // entry_count is tbl_count directly: the last entry's entry_valid and
    // zip_done can land in the same cycle, and a registered copy taken in
    // the zip_done branch would read the pre-increment tbl_count and lose it.
    assign entry_count = tbl_count;

    // Inflate (method 8). start pulses with comp_len/uncomp_len; the
    // output is pushed to the MID window through the write FIFO.
    // o_ready = inflate can take an input byte; i_ready = the FIFO can
    // take an output byte (driven by !wfifo_full so a byte is never lost).
    logic        inf_in_ready;
    logic        inf_out_ready;
    logic [7:0]  inf_out_data;
    logic        inf_out_valid;
    logic        inf_done;
    logic [2:0]  inf_error;
    logic [31:0] inf_crc;
    logic [31:0] inf_out_count;
    assign inf_out_ready = x_feeding && (method_r == 16'd8) && !wfifo_full;
    inflate inf (
        .clk, .rst,
        .start(inf_start), .comp_len(comp_r), .uncomp_len(uncomp_r),
        .i_data(feed_byte), .i_valid(inf_valid_r), .o_ready(inf_in_ready),
        .o_data(inf_out_data), .o_valid(inf_out_valid), .i_ready(inf_out_ready),
        .done(inf_done), .error(inf_error), .o_crc(inf_crc), .o_out_count(inf_out_count)
    );

    // ------------------------------------------------------------ archive read port
    // One read at a time, shared by the parser and the extractor (they
    // never run together). rd_inflight is set when a read is accepted and
    // cleared when its byte arrives; the consumer that issued it knows.
    logic        rd_inflight;
    wire         parse_active = (pstate == P_RUN);
    wire         extract_reading = ((xstate == X_LH_ISSUE) && (lh_idx < 6'd30)) ||
                                   ((xstate == X_FEED_ISSUE) && (fed_count < comp_r)) ||
                                   ((xstate == X_COPY_ISSUE) && (copied_count < comp_r));
    wire [25:0]  ext_rd_addr = (xstate == X_LH_ISSUE) ?
                                 (ARCH_BASE + lh_off_r[25:0] + {20'd0, lh_idx}) :
                               (xstate == X_FEED_ISSUE) ?
                                 (ARCH_BASE + data_off + fed_count[25:0]) :
                                 (ARCH_BASE + data_off + copied_count[25:0]);
    // areq is asserted only when the current ISSUE state actually wants a
    // read this cycle, and only while no read is already in flight. Without
    // the state guard, the lh_idx==30 cycle (which transitions to the feed
    // and wants no read) could still issue a spurious read of offset 30;
    // that extra byte desyncs the single-bit rd_inflight tracking and drops
    // a real stream byte (offset 46) downstream.
    //
    // Parse reads are latched, not pulsed: zip_dir's rd_req is a 1-cycle
    // pulse (it defaults to 0 every cycle), but wave_ddr's aready can be 0
    // in that one cycle (PCM/player requesting, or the queues momentarily
    // full). If the pulse is missed, zip_dir's rd_pending never clears and
    // the parse hangs forever waiting for data that was never requested —
    // exactly the hardware symptom (ZIP DL DONE, parse stuck, works in sim
    // where aready is 1 when the pulse fires). parse_rd_want holds the
    // request until the read is accepted. Extract reads don't need this:
    // extract_reading is already a level, so areq stays high until accepted.
    logic parse_rd_want;
    always_ff @(posedge clk) begin
        if (rst) parse_rd_want <= 1'b0;
        else if (is_zip && !dl_d) parse_rd_want <= 1'b0;      // new download
        else if (!parse_active) parse_rd_want <= 1'b0;         // parse ended
        else if (parse_active && areq && aready) parse_rd_want <= 1'b0;  // accepted
        else if (parse_active && zip_rd_req) parse_rd_want <= 1'b1;      // wanted
    end
    assign areq = parse_active ? ((zip_rd_req || parse_rd_want) && !rd_inflight) :
                  extract_reading ? !rd_inflight : 1'b0;
    assign aaddr = parse_active ? (ARCH_BASE + zip_rd_addr) : ext_rd_addr;
    assign zip_rd_valid = parse_active && rd_inflight && arvalid;
    assign zip_rd_data = ardata;

    always_ff @(posedge clk) begin
        if (rst) rd_inflight <= 1'b0;
        else if (arvalid) rd_inflight <= 1'b0;
        else if (areq && aready) rd_inflight <= 1'b1;
    end

    // Parse-stage read flags (latched; cleared on reset and on a new
    // download start, alongside dl_seen/dl_done).
    always_ff @(posedge clk) begin
        if (rst) begin
            parse_rd <= 1'b0; parse_data <= 1'b0;
        end else if (is_zip && !dl_d) begin
            parse_rd <= 1'b0; parse_data <= 1'b0;
        end else begin
            if (parse_active && areq && aready) parse_rd <= 1'b1;
            if (parse_active && rd_inflight && arvalid) parse_data <= 1'b1;
        end
    end

    // The header check for the extracted MID (same as mid_loader's).
    wire x_load_ok_now = (out_count >= 23'd14) &&
                         (hdr[0] == 8'h4D) && (hdr[1] == 8'h54) &&  // "MThd"
                         (hdr[2] == 8'h68) && (hdr[3] == 8'h64) &&
                         (hdr[4] == 8'd0) && (hdr[5] == 8'd0) &&
                         (hdr[6] == 8'd0) && (hdr[7] == 8'd6) &&   // header len 6
                         ({hdr[8], hdr[9]} <= 16'd1) &&             // format 0/1
                         ({hdr[10], hdr[11]} >= 16'd1) &&
                         ({hdr[10], hdr[11]} <= 16'd32) &&
                         (hdr[12][7] == 1'b0) &&                    // ticks/qn, not SMPTE
                         ({hdr[12][6:0], hdr[13]} != 15'd0);

    // ------------------------------------------------------------ control
    // The inflate output is only accepted while the extract is feeding or
    // draining; an abort (X_ERR) stops accepting it, so a killed extract
    // cannot write more bytes into the MID window.
    wire x_feeding = (xstate == X_FEED_ISSUE) || (xstate == X_FEED_WAIT) ||
                     (xstate == X_FEED_HOLD) || (xstate == X_DRAIN);

    always_ff @(posedge clk) begin
        // defaults (1-clock pulses)
        wave_wr <= 1'b0;
        x_load_done <= 1'b0;
        x_extract_done <= 1'b0;
        zip_start <= 1'b0;
        inf_start <= 1'b0;
        dl_d <= is_zip;
        if (rst) begin
            wfifo_wp <= 6'd0; wfifo_rp <= 6'd0; wr_active <= 1'b0;
            end_pend <= 1'b0; count <= 26'd0; over <= 1'b0; fifo_over <= 1'b0;
            arch_len_r <= 26'd0; arch_ok <= 1'b0; arch_failed <= 1'b0;
            dl_failed <= 1'b0; parse_failed <= 1'b0; dl_seen <= 1'b0; dl_done <= 1'b0;
            pstate <= P_IDLE; tbl_count <= 10'd0; tbl_over <= 1'b0;
            xstate <= X_IDLE; x_mid_ok <= 1'b0; x_mid_len <= 23'd0;
            x_mid_fmt <= 16'd0; x_mid_ntracks <= 16'd0; x_mid_ppq <= 16'd0;
            is_m3u_r <= 1'b0;
            x_extract_failed <= 1'b0; x_error <= 3'd0; x_err_r <= 3'd0;
            out_count <= 23'd0; copy_crc <= 32'hFFFFFFFF;
            inf_valid_r <= 1'b0;
        end else begin
            // ---- write FIFO drain (shared by download and extract) ----
            if (wr_active) begin
                if (wave_busy) begin
                    wr_active <= 1'b0;
                    wfifo_rp <= wfifo_rp + 6'd1;
                end else begin
                    wave_wr <= 1'b1;
                end
            end else if (!wfifo_empty && !wave_busy) begin
                wave_wr <= 1'b1;
                wave_addr <= wfifo[wfifo_rp[4:0]][33:8];
                wave_data <= wfifo[wfifo_rp[4:0]][7:0];
                wr_active <= 1'b1;
            end
            // ---- FIFO pushes (download / inflate output / copy byte) ----
            if (push_any) begin
                wfifo[wfifo_wp[4:0]] <= {push_addr, push_data};
                wfifo_wp <= wfifo_wp + 6'd1;
                if (push_inf || push_copy) begin
                    if (out_count < 23'd14) hdr[out_count[3:0]] <= push_data;
                    out_count <= out_count + 23'd1;
                    if (push_copy) copy_crc <= crc32_byte(copy_crc, push_data);
                end
            end
            // ---- ZIP download ----
            if (is_zip && !dl_d) begin
                wfifo_wp <= 6'd0; wfifo_rp <= 6'd0; wr_active <= 1'b0;
                end_pend <= 1'b0; count <= 26'd0; over <= 1'b0; fifo_over <= 1'b0;
                arch_ok <= 1'b0; arch_failed <= 1'b0;
                dl_failed <= 1'b0; parse_failed <= 1'b0; dl_seen <= 1'b1; dl_done <= 1'b0;
                pstate <= P_IDLE;
            end
            if (is_zip && ioctl_wr) begin
                if (!over && count < ARCH_MAX) begin
                    if (!wfifo_full) begin
                        count <= count + 26'd1;
                    end else begin
                        fifo_over <= 1'b1;
                        count <= count + 26'd1;
                    end
                end else begin
                    over <= 1'b1;
                    count <= count + 26'd1;
                end
            end
            if (!is_zip && dl_d) begin
                arch_len_r <= count;
                end_pend <= 1'b1;
            end
            // The parse must not start until every byte is in DDR3.
            if (end_pend && wfifo_empty && !wr_active) begin
                end_pend <= 1'b0;
                if ((count >= 26'd22) && !over && !fifo_over) begin
                    zip_start <= 1'b1;
                    dl_done <= 1'b1;
                    pstate <= P_RUN;
                    tbl_count <= 10'd0;
                    tbl_over <= 1'b0;
                end else begin
                    arch_failed <= 1'b1;
                    dl_failed <= 1'b1;
                    arch_ok <= 1'b0;
                end
            end
            // ---- parse ----
            if (pstate == P_RUN) begin
                if (zip_entry_valid) begin
                    if (tbl_count < 10'd512) tbl_count <= tbl_count + 10'd1;
                    else tbl_over <= 1'b1;
                end
                if (zip_done) begin
                    if (tbl_over) begin
                        arch_failed <= 1'b1; parse_failed <= 1'b1; arch_ok <= 1'b0;
                    end else begin
                        arch_ok <= 1'b1; arch_failed <= 1'b0;
                    end
                    pstate <= P_IDLE;
                end else if (zip_error != 3'd0) begin
                    arch_failed <= 1'b1; parse_failed <= 1'b1; arch_ok <= 1'b0;
                    pstate <= P_IDLE;
                end
            end
            // ---- extract ----
            if (x_extracting && (mid_dl || is_zip)) begin
                // A MID or ZIP download started mid-extract: abort (the
                // MID window / archive is being rewritten under us).
                x_err_r <= 3'd6;
                xstate <= X_ERR;
                inf_valid_r <= 1'b0;
            end else begin
                case (xstate)
                X_IDLE: begin
                    if (extract_start && arch_ok && (pstate == P_IDLE) && !mid_dl && !is_zip) begin
                        extract_idx_r <= extract_idx;
                        x_extract_failed <= 1'b0;
                        x_error <= 3'd0;
                        x_err_r <= 3'd0;
                        xstate <= X_TBL_RD;
                    end
                end
                X_TBL_RD: begin
                    xstate <= X_TBL_GOT;
                end
                X_TBL_GOT: begin
                    // tbl_rd is valid (registered read of extract_idx_r).
                    method_r <= tbl_rd[57:42];
                    crc_r <= tbl_rd[89:58];
                    comp_r <= tbl_rd[121:90];
                    uncomp_r <= tbl_rd[153:122];
                    lh_off_r <= tbl_rd[185:154];
                    is_m3u_r <= tbl_rd[187];
                    if (tbl_rd[186]) begin
                        x_err_r <= 3'd1; xstate <= X_ERR;   // a directory
                    end else if (tbl_rd[57:42] != 16'd0 && tbl_rd[57:42] != 16'd8) begin
                        x_err_r <= 3'd1; xstate <= X_ERR;   // unsupported method (incl. 9)
                    end else if (tbl_rd[153:122] > {9'd0, MID_MAX}) begin
                        x_err_r <= 3'd1; xstate <= X_ERR;   // too big for the MID window
                    end else begin
                        lh_idx <= 6'd0;
                        lh_sig <= 32'd0;
                        lh_name_len <= 16'd0;
                        lh_extra_len <= 16'd0;
                        xstate <= X_LH_ISSUE;
                    end
                end
                X_LH_ISSUE: begin
                    if (lh_idx == 6'd30) begin
                        if (lh_sig != 32'h504B0304) begin
                            x_err_r <= 3'd2; xstate <= X_ERR;   // bad local header
                        end else begin
                            data_off <= lh_off_r[25:0] + 26'd30 + {10'd0, lh_name_len} + {10'd0, lh_extra_len};
                            out_count <= 23'd0;
                            copy_crc <= 32'hFFFFFFFF;
                            if (method_r == 16'd8) begin
                                fed_count <= 32'd0;
                                inf_valid_r <= 1'b0;
                                inf_start <= 1'b1;
                                xstate <= X_FEED_ISSUE;
                            end else begin
                                copied_count <= 32'd0;
                                xstate <= X_COPY_ISSUE;
                            end
                        end
                    end else if (aready) begin
                        // areq is asserted by the port mux in this state;
                        // aready means the read was accepted.
                        xstate <= X_LH_WAIT;
                    end
                end
                X_LH_WAIT: begin
                    if (arvalid && rd_inflight) begin
                        if (lh_idx < 6'd4) lh_sig <= {lh_sig[23:0], ardata};
                        if (lh_idx == 6'd26) lh_name_len[7:0] <= ardata;
                        if (lh_idx == 6'd27) lh_name_len[15:8] <= ardata;
                        if (lh_idx == 6'd28) lh_extra_len[7:0] <= ardata;
                        if (lh_idx == 6'd29) lh_extra_len[15:8] <= ardata;
                        lh_idx <= lh_idx + 6'd1;
                        xstate <= X_LH_ISSUE;
                    end
                end
                X_FEED_ISSUE: begin
                    if (fed_count >= comp_r) begin
                        xstate <= X_DRAIN;
                    end else if (aready) begin
                        xstate <= X_FEED_WAIT;
                    end
                end
                X_FEED_WAIT: begin
                    if (arvalid && rd_inflight) begin
                        feed_byte <= ardata;
                        inf_valid_r <= 1'b1;
                        xstate <= X_FEED_HOLD;
                    end
                end
                X_FEED_HOLD: begin
                    if (inf_in_ready && inf_valid_r) begin
                        fed_count <= fed_count + 32'd1;
                        inf_valid_r <= 1'b0;
                        xstate <= X_FEED_ISSUE;
                    end else if (inf_error != 3'd0) begin
                        // Inflate died (bad table/symbol/stream) and is back
                        // in IDLE, so o_ready will never rise again; fail
                        // loudly instead of hanging here.
                        x_err_r <= 3'd3;
                        inf_valid_r <= 1'b0;
                        xstate <= X_ERR;
                    end
                end
                X_COPY_ISSUE: begin
                    if (copied_count >= comp_r) begin
                        xstate <= X_DRAIN_COPY;
                    end else if (aready) begin
                        xstate <= X_COPY_WAIT;
                    end
                end
                X_COPY_WAIT: begin
                    if (arvalid && rd_inflight) begin
                        copy_byte <= ardata;
                        xstate <= X_COPY_PUSH;
                    end
                end
                X_COPY_PUSH: begin
                    if (!wfifo_full) begin
                        copied_count <= copied_count + 32'd1;
                        xstate <= X_COPY_ISSUE;
                    end
                end
                X_DRAIN: begin
                    // Wait for inflate to finish AND for its output bytes
                    // to land in DDR3 (the player's scan reads DDR3).
                    if (inf_done && wfifo_empty && !wr_active) begin
                        xstate <= X_VERIFY;
                    end else if (inf_error != 3'd0) begin
                        // Inflate died without asserting done; fail loudly.
                        x_err_r <= 3'd3;
                        xstate <= X_ERR;
                    end
                end
                X_DRAIN_COPY: begin
                    if (wfifo_empty && !wr_active) begin
                        xstate <= X_VERIFY;
                    end
                end
                X_VERIFY: begin
                    if (is_m3u_r) begin
                        // A .m3u is text, not a MID: skip the SMF header
                        // check. Success is CRC + length only. x_mid_ok
                        // stays 0 (the player must not scan it); the board
                        // sees x_is_m3u and hands the text to m3u_parse.
                        // x_load_done is NOT pulsed for a .m3u.
                        if ((method_r == 16'd8 && inf_error == 3'd0 && inf_out_count == uncomp_r && inf_crc == crc_r) ||
                            (method_r == 16'd0 && out_count == uncomp_r[22:0] && (~copy_crc) == crc_r)) begin
                            x_mid_ok <= 1'b0;
                            x_mid_len <= out_count;
                            x_extract_done <= 1'b1;
                        end else begin
                            x_mid_ok <= 1'b0;
                            x_extract_failed <= 1'b1;
                            x_error <= (method_r == 16'd8 && inf_error != 3'd0) ? 3'd3 : 3'd4;
                            x_extract_done <= 1'b1;
                        end
                    end else if (method_r == 16'd8) begin
                        if ((inf_error == 3'd0) && (inf_out_count == uncomp_r) &&
                            (inf_crc == crc_r) && x_load_ok_now) begin
                            x_mid_ok <= 1'b1;
                            x_mid_len <= out_count;
                            x_mid_fmt <= {hdr[8], hdr[9]};
                            x_mid_ntracks <= {hdr[10], hdr[11]};
                            x_mid_ppq <= {1'b0, hdr[12][6:0], hdr[13]};
                            x_load_done <= 1'b1;
                            x_extract_done <= 1'b1;
                        end else begin
                            x_mid_ok <= 1'b0;
                            x_extract_failed <= 1'b1;
                            x_error <= (inf_error != 3'd0) ? 3'd3 :
                                       (inf_out_count != uncomp_r || inf_crc != crc_r) ? 3'd4 : 3'd5;
                            x_extract_done <= 1'b1;
                        end
                    end else begin
                        if ((out_count == uncomp_r[22:0]) && ((~copy_crc) == crc_r) && x_load_ok_now) begin
                            x_mid_ok <= 1'b1;
                            x_mid_len <= out_count;
                            x_mid_fmt <= {hdr[8], hdr[9]};
                            x_mid_ntracks <= {hdr[10], hdr[11]};
                            x_mid_ppq <= {1'b0, hdr[12][6:0], hdr[13]};
                            x_load_done <= 1'b1;
                            x_extract_done <= 1'b1;
                        end else begin
                            x_mid_ok <= 1'b0;
                            x_extract_failed <= 1'b1;
                            x_error <= (out_count != uncomp_r[22:0] || (~copy_crc) != crc_r) ? 3'd4 : 3'd5;
                            x_extract_done <= 1'b1;
                        end
                    end
                    xstate <= X_IDLE;
                end
                X_ERR: begin
                    x_mid_ok <= 1'b0;
                    x_extract_failed <= 1'b1;
                    x_error <= x_err_r;
                    x_extract_done <= 1'b1;
                    xstate <= X_IDLE;
                end
                default: xstate <= X_IDLE;
                endcase
            end
        end
    end
endmodule
