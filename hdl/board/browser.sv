// SPDX-License-Identifier: GPL-2.0-or-later
// Two-column archive browser for the SC-55 core (core logic: scan,
// navigation, activation). The display writer and the playlist column
// are separate increments; the bench verifies the left list through the
// browser's left_names / left_meta RAMs.
//
// Left column: folders and .m3u files in the current archive path
// (".." at the top to go up). Keyboard only: Up/Down move the cursor,
// Left/Right switch columns, Enter activates (folder -> enter it,
// ".." -> up, .m3u -> extract + parse + play immediately, song ->
// jump to it).
//
// The browser owns the entry-table port and the archive byte-read port
// while it is scanning (busy=1); it only scans when zip_load is idle
// and m3u_parse is idle, so the three users never collide.
//
// Table record layout (zip_load): [25:0] name_ptr, [41:26] name_len,
// [57:42] method, [89:58] crc, [121:90] comp_size, [153:122] uncomp_size,
// [185:154] local_hdr_off, [186] is_dir, [187] is_m3u.
module browser (
    input  logic        clk,
    input  logic        rst,
    // keyboard nav pulses (from browser_kbd)
    input  logic        nav_up,
    input  logic        nav_down,
    input  logic        nav_left,
    input  logic        nav_right,
    input  logic        nav_enter,
    // archive state
    input  logic        arch_ok,          // archive loaded + parsed
    input  logic [9:0]  entry_count,
    input  logic        zip_idle,         // zip_load is idle (no parse/extract)
    input  logic        m3u_idle,         // m3u_parse is idle
    // start a scan (pulse from the board: archive loaded / path changed)
    input  logic        start_scan,
    // table port (to zip_load's tbl_ram, via the board mux)
    output logic [8:0]  tbl_ra,
    input  logic [191:0] tbl_rd,
    // archive byte-read port (to wave_ddr, via the board mux)
    output logic        rd_req,
    output logic [25:0] rd_addr,
    input  logic        rd_ready,
    input  logic        rd_valid,
    input  logic [7:0]  rd_data,
    output logic        busy,             // owns the table/archive ports
    // extract trigger (to zip_load, via the board mux)
    output logic        extract_start,
    output logic [8:0]  extract_idx,
    // playlist (from m3u_parse; the browser has its own read port)
    input  logic [6:0]  playlist_count,
    input  logic        playlist_active,
    input  logic [6:0]  pl_current,
    input  logic [8:0]  pl_rd,            // playlist RAM read data (browser port)
    output logic [6:0]  pl_ra,            // playlist RAM read address
    // player state (for the progress line)
    input  logic        playing,
    // sequencer jump (set pl_current when the user picks a song)
    output logic        pl_jump,          // pulse: jump to pl_jump_idx
    output logic [6:0]  pl_jump_idx,
    // character buffer write port (to the dual-clock char RAM)
    output logic        char_we,
    output logic [9:0]  char_wa,
    output logic [9:0]  char_wd,          // {attr[1:0], ascii[7:0]}
    // a playlist was just parsed (pulse from the board)
    input  logic        playlist_ready,
    // status
    output logic        visible,          // archive loaded: show the browser
    output logic        scan_done,        // pulse: a scan finished
    output logic [5:0]  left_count_o,     // items in the left list
    output logic [5:0]  left_cursor_o,    // selected item
    output logic        active_right      // cursor is in the right column
);
    wire [25:0] tbl_name_ptr = tbl_rd[25:0];
    wire [15:0] tbl_name_len = tbl_rd[41:26];
    wire        tbl_is_m3u   = tbl_rd[187];

    localparam logic [25:0] ARCH_BASE = 26'h800000;
    localparam logic [8:0]  DOTDOT_IDX = 9'h1FF;   // marker: the ".." item

    // ---- current path (directory prefix, e.g. "GM/") ----
    logic [7:0] path_buf [0:63];
    logic [6:0] path_len;                  // 0 = archive root

    // ---- left list: dirs + .m3u files in the current path ----
    // meta: {is_dir(1), entry_idx(9), name_len(6)}; names packed 32/item,
    // NUL-terminated (name_len bytes + one 0x00).
    logic        left_we;
    logic [5:0]  left_wa;
    logic [15:0] left_wd;
    logic [5:0]  left_ra;
    logic [15:0] left_rd;
    sdp_ram #(.AW(6), .DW(16)) left_meta (
        .clk(clk), .we(left_we), .wa(left_wa), .wd(left_wd),
        .ra(left_ra), .rd(left_rd));
    logic        lname_we;
    logic [10:0] lname_wa;
    logic [7:0]  lname_wd;
    logic [10:0] lname_ra;
    logic [7:0]  lname_rd;
    sdp_ram #(.AW(11), .DW(8)) left_names (
        .clk(clk), .we(lname_we), .wa(lname_wa), .wd(lname_wd),
        .ra(lname_ra), .rd(lname_rd));
    logic [5:0]  left_count;
    logic [5:0]  left_cursor;
    // shadow copies (updated when an item is added; the meta RAM read
    // would cost a cycle and the nav logic needs them combinationally).
    logic        left_is_dir [0:63];
    logic [8:0]  left_entry [0:63];
    logic [5:0]  left_namelen [0:63];

    // ---- display writer (character buffer, clk domain) ----
    // Buffer: 64 cols x 16 rows, address = {row[3:0], col[5:0]}.
    // Row 0 = header (path on the left). Rows 1..14 = left-list items
    // (14 visible). Col 0 = cursor marker (">" or " "), cols 1..21 =
    // the name. Col 22 = divider. Cols 26..45 = right column (the
    // right pane starts 3 cols right of the divider, leaving a gap).
    // Right column: row 0 = header ("N/M SSs"), row 1 = blank,
    // rows 2..14 = songs (13 visible).
    // Character cell: {attr[1:0], ascii[7:0]}; attr 00 normal (white),
    // 01 loaded .m3u (green), 10 playing song (amber).
    localparam logic [1:0] ATTR_NORM = 2'b00;
    localparam logic [1:0] ATTR_LOAD = 2'b01;
    localparam logic [1:0] ATTR_PLAY = 2'b10;
    localparam int DISP_ROWS = 14;             // visible item rows (browser is 15 rows tall now)
    localparam int R_MARK = 26;                // right-column marker col
    localparam int R_NAME = 27;                // right-column name first col
    localparam int R_LAST = 45;                // right-column last col
    typedef enum logic [4:0] {
        D_IDLE, D_HDR, D_ITEM_SET, D_ITEM_MARK, D_ITEM_WAIT, D_ITEM_CH,
        D_ITEM_PAD, D_ITEM_BLANK, D_DONE,
        D_R_HDR, D_R_BLANK, D_R_ITEM_SET, D_R_ITEM_MARK, D_R_ITEM_WAIT, D_R_ITEM_CH,
        D_R_ITEM_PAD, D_R_ITEM_BLANK, D_R_DONE
    } dstate_t;
    dstate_t dstate;
    logic        d_dirty;
    logic [3:0]  d_row;                       // 0 = header, 1..10 = items
    logic [5:0]  d_col;
    logic [5:0]  d_item;                      // left-list item being drawn
    logic [5:0]  d_char;                      // char index within the name
    logic [5:0]  left_top;                    // first visible item (scroll)
    logic [6:0]  d_song;                      // playlist song being drawn
    logic [5:0]  d_rchar;                     // char index within the song name
    // ---- elapsed-time progress (right column) ----
    // Counts seconds while the player is playing; reset on a song change.
    logic [25:0] sec_div;                  // 50M cycles = 1 s at 50 MHz
    logic [15:0] play_sec;                 // elapsed seconds in this song
    logic [6:0]  prev_pl_current;
    logic        sec_tick;                 // 1-cycle pulse when play_sec increments
    // Right-pane cursor (playlist index) and the loaded .m3u's archive
    // entry index (for the color highlight). right_cursor is the song
    // the user picks with Up/Down when the focus is on the right pane;
    // Enter on it jumps the sequencer to that song.
    logic [6:0]  right_cursor;
    logic [8:0]  loaded_entry;            // archive entry idx of the loaded .m3u
    logic        loaded_valid;            // a playlist is loaded
    // Decimal conversion: a 10-bit value (0..999) to 3 ASCII chars
    // (hundreds, tens, ones), leading positions blank for < 100 / < 10.
    function automatic logic [23:0] dec3(input logic [9:0] n);
        logic [9:0] h, t, o;
        h = n / 10'd100;
        t = (n % 10'd100) / 10'd10;
        o = n % 10'd10;
        return {(h == 10'd0) ? 8'h20 : (8'h30 + h[7:0]),
                (h == 10'd0 && t == 10'd0) ? 8'h20 : (8'h30 + t[7:0]),
                (8'h30 + o[7:0])};
    endfunction
    // Header digits, precomputed once (not three copies of the divider).
    wire [23:0] hdr_cur = dec3({3'b0, pl_current} + 10'd1);
    wire [23:0] hdr_tot = dec3({3'b0, playlist_count});
    wire [23:0] hdr_sec = dec3(play_sec[9:0]);

    // Row color for the display writer: the loaded .m3u's row (left)
    // is green, the playing song's row (right) is amber, everything
    // else is normal white. Evaluated from the row currently being
    // drawn (d_item / d_song); the writer prepends it to every char.
    wire [1:0] wr_attr =
        ((dstate == D_ITEM_MARK || dstate == D_ITEM_CH || dstate == D_ITEM_PAD) &&
         loaded_valid && !left_is_dir[d_item] && (left_entry[d_item] == loaded_entry)) ? ATTR_LOAD :
        ((dstate == D_R_ITEM_MARK || dstate == D_R_ITEM_CH || dstate == D_R_ITEM_PAD) &&
         playlist_active && (d_song == pl_current)) ? ATTR_PLAY :
        ATTR_NORM;

    // Elapsed-time counter: increments once per second while playing;
    // resets when the song changes or playback stops/starts a new song.
    // sec_tick pulses for one cycle when play_sec increments; the main
    // FSM uses it to redraw the header every second (not only on a key).
    always_ff @(posedge clk) begin
        if (rst) begin
            sec_div <= 26'd0;
            play_sec <= 16'd0;
            prev_pl_current <= 7'd0;
            sec_tick <= 1'b0;
        end else begin
            sec_tick <= 1'b0;
            if (pl_current != prev_pl_current) begin
                play_sec <= 16'd0;
                sec_div <= 26'd0;
                prev_pl_current <= pl_current;
            end else if (playing) begin
                if (sec_div >= 26'd49_999_999) begin
                    sec_div <= 26'd0;
                    play_sec <= play_sec + 16'd1;
                    sec_tick <= 1'b1;
                end else begin
                    sec_div <= sec_div + 26'd1;
                end
            end else begin
                sec_div <= 26'd0;
            end
        end
    end

    // ---- song names (right column): basename per playlist song ----
    // song_names: 100 songs x 32 bytes, NUL-terminated.
    logic        sname_we;
    logic [11:0] sname_wa;
    logic [7:0]  sname_wd;
    logic [11:0] sname_ra;
    logic [7:0]  sname_rd;
    sdp_ram #(.AW(12), .DW(8)) song_names (
        .clk(clk), .we(sname_we), .wa(sname_wa), .wd(sname_wd),
        .ra(sname_ra), .rd(sname_rd));
    logic [5:0]  song_namelen [0:99];         // basename length per song
    logic        songs_ready;                 // song names loaded
    logic [6:0]  r_top;                       // first visible song (scroll)

    // (char_we/char_wa/char_wd are module outputs, driven below.)
    typedef enum logic [4:0] {
        B_IDLE,
        B_SCAN_INIT,        // set up the list (".." first, if not root)
        B_SCAN_TBL,         // present tbl_ra
        B_SCAN_TBL_WAIT,    // registered RAM read lands
        B_SCAN_TBL_GOT,     // latch the record
        B_SCAN_NAME,        // issue a name-byte read (rd_req)
        B_SCAN_NAME_WAIT,   // wait for rd_valid, process the byte
        B_SCAN_DECIDE,      // does this entry add a left-list item?
        B_DEDUP_SET,        // present a left_names read for dedup
        B_DEDUP_WAIT,       // registered RAM read lands
        B_DEDUP_GET,        // compare the byte
        B_ADD,              // append the item (write meta + name bytes)
        B_PATH_APPEND_SET,  // read a dir-name byte (entering a folder)
        B_PATH_APPEND_WAIT, // registered RAM read lands
        B_PATH_APPEND_GET,  // append it to path_buf
        B_PATH_UP,          // strip the last path component
        B_SCAN_NEXT,        // advance cand_idx
        B_SCAN_DONE,
        // ---- song-name loader (right column) ----
        B_SONG_PL,          // present pl_ra for this song
        B_SONG_PL_WAIT,     // registered playlist-RAM read lands
        B_SONG_PL_GOT,      // latch the entry index (+ present tbl_ra)
        B_SONG_TBL_WAIT,    // registered table-RAM read lands
        B_SONG_TBL_GOT,     // latch name_ptr/name_len
        B_SONG_NAME,        // issue a name-byte read (rd_req)
        B_SONG_NAME_WAIT,   // wait for rd_valid, process the byte
        B_SONG_STORE,       // copy the basename into song_names
        B_SONG_NEXT,        // advance to the next song
        B_SONG_DONE
    } bstate_t;
    bstate_t bstate;
    logic [8:0]  cand_idx;
    logic [25:0] cand_name_ptr;
    logic [15:0] cand_name_len;
    logic        cand_is_m3u;
    logic [7:0]  name_idx;
    logic        prefix_ok;
    logic [7:0]  child_buf [0:31];
    logic [5:0]  child_len;
    logic        child_is_dir;
    // song loader registers
    logic [6:0]  song_idx;                  // playlist index being loaded
    logic [8:0]  song_entry;                // entry index of that song
    logic [25:0] song_name_ptr;
    logic [15:0] song_name_len;
    logic [7:0]  song_byte_idx;             // byte index within the name
    logic [7:0]  song_base_buf [0:31];      // basename being captured
    logic [5:0]  song_base_len;
    logic [5:0]  song_store_idx;
    logic        child_done;
    logic [5:0]  dedup_idx;
    logic [5:0]  dedup_byte;
    logic        add_is_dir;
    logic [8:0]  add_entry;
    logic [5:0]  add_idx;
    logic [1:0]  init_phase;              // B_SCAN_INIT micro-step
    logic [5:0]  path_app_idx;
    logic [5:0]  path_scan_idx;   // backward-scan index for B_PATH_UP

    // Path-up is a sequential backward scan (not a combinational loop
    // over the whole 64-byte buffer): B_PATH_UP walks path_scan_idx down
    // from path_len-2 until it finds a '/', then path_len becomes that
    // index + 1 (or 0 if none). A 63-iteration combinational priority
    // chain costs far more ALMs than one indexed read per cycle.

    // Archive read handshake. rd_req is asserted only in a name-read
    // state, only while a read is actually wanted and none is in flight,
    // so a state-exit cycle never issues a spurious read (the zip_load /
    // m3u_parse lesson). The scan and the song loader share the port;
    // they never run at the same time.
    logic rd_inflight;
    wire scan_want = (bstate == B_SCAN_NAME) && (name_idx < cand_name_len[7:0]) && !child_done;
    wire song_want = (bstate == B_SONG_NAME) && (song_byte_idx < song_name_len[7:0]);
    wire want_read = scan_want || song_want;
    assign rd_req  = want_read && !rd_inflight;
    assign rd_addr = (bstate == B_SONG_NAME) ? (ARCH_BASE + song_name_ptr + {18'd0, song_byte_idx})
                                             : (ARCH_BASE + cand_name_ptr + {18'd0, name_idx});
    always_ff @(posedge clk) begin
        if (rst) rd_inflight <= 1'b0;
        else if (rd_valid) rd_inflight <= 1'b0;
        else if (rd_req && rd_ready) rd_inflight <= 1'b1;
    end

    assign busy = (bstate != B_IDLE);
    assign visible = arch_ok;
    assign left_count_o = left_count;
    assign left_cursor_o = left_cursor;

    function automatic logic lower_eq(input logic [7:0] a, input logic [7:0] b);
        logic [7:0] la, lb;
        la = (a >= 8'h41 && a <= 8'h5A) ? (a + 8'd32) : a;
        lb = (b >= 8'h41 && b <= 8'h5A) ? (b + 8'd32) : b;
        return la == lb;
    endfunction

    always_ff @(posedge clk) begin
        extract_start <= 1'b0;
        scan_done <= 1'b0;
        pl_jump <= 1'b0;
        left_we <= 1'b0;
        lname_we <= 1'b0;
        sname_we <= 1'b0;
        char_we <= 1'b0;
        if (rst) begin
            bstate <= B_IDLE;
            path_len <= 7'd0;
            left_count <= 6'd0;
            left_cursor <= 6'd0;
            left_top <= 6'd0;
            cand_idx <= 9'd0;
            active_right <= 1'b0;
            tbl_ra <= 9'd0;
            left_ra <= 6'd0;
            lname_ra <= 11'd0;
            extract_idx <= 9'd0;
            pl_ra <= 7'd0;
            pl_jump_idx <= 7'd0;
            init_phase <= 2'd0;
            path_app_idx <= 6'd0;
            dstate <= D_IDLE;
            d_dirty <= 1'b0;
            d_row <= 4'd0;
            d_col <= 6'd0;
            d_item <= 6'd0;
            d_char <= 6'd0;
            d_song <= 7'd0;
            d_rchar <= 6'd0;
            song_idx <= 7'd0;
            song_entry <= 9'd0;
            song_name_ptr <= 26'd0;
            song_name_len <= 16'd0;
            song_byte_idx <= 8'd0;
            song_base_len <= 6'd0;
            song_store_idx <= 6'd0;
            songs_ready <= 1'b0;
            r_top <= 7'd0;
            right_cursor <= 7'd0;
            loaded_entry <= 9'd0;
            loaded_valid <= 1'b0;
            sname_ra <= 12'd0;
        end else begin
            case (bstate)
                // ------------------------------------------------ idle / nav
                B_IDLE: begin
                    if (start_scan && arch_ok && zip_idle && m3u_idle) begin
                        init_phase <= 2'd0;
                        bstate <= B_SCAN_INIT;
                    end else if (playlist_ready && playlist_active && m3u_idle) begin
                        // A playlist was just parsed: load its song names.
                        song_idx <= 7'd0;
                        songs_ready <= 1'b0;
                        r_top <= 7'd0;
                        right_cursor <= 7'd0;
                        bstate <= B_SONG_PL;
                    end else if (arch_ok) begin
                        // Up/Down move the cursor in the FOCUSED pane:
                        // left list (dirs + .m3u) or right playlist.
                        // Left/Right move the focus between panes (Right
                        // only when a playlist is loaded). Enter activates
                        // the focused item: folder/.m3u on the left, jump
                        // to the song on the right.
                        if (!active_right && nav_up && left_count != 0) begin
                            left_cursor <= (left_cursor == 0) ? (left_count - 6'd1) : (left_cursor - 6'd1);
                            // Keep the cursor in the visible window.
                            if (left_cursor == 0) left_top <= (left_count > 6'(DISP_ROWS)) ? (left_count - 6'(DISP_ROWS)) : 6'd0;
                            else if ((left_cursor - 6'd1) < left_top) left_top <= left_cursor - 6'd1;
                            d_dirty <= 1'b1;
                        end else if (!active_right && nav_down && left_count != 0) begin
                            left_cursor <= (left_cursor + 6'd1 >= left_count) ? 6'd0 : (left_cursor + 6'd1);
                            if (left_cursor + 6'd1 >= left_count) left_top <= 6'd0;
                            else if ((left_cursor + 6'd1) >= (left_top + 6'(DISP_ROWS))) left_top <= left_cursor + 6'd1 - (6'(DISP_ROWS) - 6'd1);
                            d_dirty <= 1'b1;
                        end else if (active_right && nav_up && playlist_count != 0) begin
                            right_cursor <= (right_cursor == 7'd0) ? (playlist_count - 7'd1) : (right_cursor - 7'd1);
                            // Right pane shows DISP_ROWS-1 songs (row 1 is
                            // the blank line under the header).
                            if (right_cursor == 7'd0) r_top <= (playlist_count > 7'(DISP_ROWS - 1)) ? (playlist_count - 7'(DISP_ROWS - 1)) : 7'd0;
                            else if ((right_cursor - 7'd1) < r_top) r_top <= right_cursor - 7'd1;
                            d_dirty <= 1'b1;
                        end else if (active_right && nav_down && playlist_count != 0) begin
                            right_cursor <= (right_cursor + 7'd1 >= playlist_count) ? 7'd0 : (right_cursor + 7'd1);
                            if (right_cursor + 7'd1 >= playlist_count) r_top <= 7'd0;
                            else if ((right_cursor + 7'd1) >= (r_top + 7'(DISP_ROWS - 1))) r_top <= right_cursor + 7'd1 - (7'(DISP_ROWS - 1) - 7'd1);
                            d_dirty <= 1'b1;
                        end else if (nav_left) begin
                            active_right <= 1'b0;
                            d_dirty <= 1'b1;
                        end else if (nav_right && playlist_active) begin
                            active_right <= 1'b1;
                            d_dirty <= 1'b1;
                        end else if (nav_enter && !active_right && left_count != 0) begin
                            if (left_entry[left_cursor] == DOTDOT_IDX) begin
                                path_scan_idx <= (path_len >= 7'd2) ? (path_len[5:0] - 6'd2) : 6'd0;
                                bstate <= B_PATH_UP;
                            end else if (left_is_dir[left_cursor]) begin
                                // Enter the directory (only if the longer
                                // path still fits in the 64-byte buffer).
                                if ((path_len + {1'b0, left_namelen[left_cursor]} + 7'd1) <= 7'd63) begin
                                    path_app_idx <= 6'd0;
                                    bstate <= B_PATH_APPEND_SET;
                                end
                            end else begin
                                // .m3u: extract it; the sequencer parses
                                // it and starts the playlist. Remember
                                // which entry it was, for the color
                                // highlight on the left pane.
                                extract_idx <= left_entry[left_cursor];
                                extract_start <= 1'b1;
                                loaded_entry <= left_entry[left_cursor];
                                loaded_valid <= 1'b1;
                                right_cursor <= 7'd0;
                            end
                        end else if (nav_enter && active_right && playlist_active && playlist_count != 0) begin
                            // Jump the sequencer to the focused song.
                            pl_jump_idx <= right_cursor;
                            pl_jump <= 1'b1;
                            d_dirty <= 1'b1;
                        end
                        // Header refresh: redraw once per second while a
                        // playlist is playing, so the elapsed-time header
                        // updates even with no key pressed.
                        if (sec_tick && playlist_active) d_dirty <= 1'b1;
                    end
                end

                // ------------------------------------------------ scan
                B_SCAN_INIT: begin
                    if (path_len == 0) begin
                        left_count <= 6'd0;
                        cand_idx <= 9'd0;
                        left_cursor <= 6'd0;
                        bstate <= B_SCAN_TBL;
                    end else if (init_phase == 2'd0) begin
                        // "..": first '.'.
                        lname_we <= 1'b1; lname_wa <= 11'd0; lname_wd <= 8'h2E;
                        init_phase <= 2'd1;
                    end else if (init_phase == 2'd1) begin
                        lname_we <= 1'b1; lname_wa <= 11'd1; lname_wd <= 8'h2E;
                        init_phase <= 2'd2;
                    end else begin
                        lname_we <= 1'b1; lname_wa <= 11'd2; lname_wd <= 8'd0;
                        left_we <= 1'b1; left_wa <= 6'd0;
                        left_wd <= {1'b1, DOTDOT_IDX, 6'd2};
                        left_is_dir[0] <= 1'b1;
                        left_entry[0] <= DOTDOT_IDX;
                        left_namelen[0] <= 6'd2;
                        left_count <= 6'd1;
                        cand_idx <= 9'd0;
                        left_cursor <= 6'd0;
                        init_phase <= 2'd0;
                        bstate <= B_SCAN_TBL;
                    end
                end

                B_SCAN_TBL: begin
                    tbl_ra <= cand_idx;
                    bstate <= B_SCAN_TBL_WAIT;
                end

                B_SCAN_TBL_WAIT: begin
                    bstate <= B_SCAN_TBL_GOT;
                end

                B_SCAN_TBL_GOT: begin
                    cand_name_ptr <= tbl_name_ptr;
                    cand_name_len <= tbl_name_len;
                    cand_is_m3u <= tbl_is_m3u;
                    name_idx <= 8'd0;
                    prefix_ok <= 1'b1;
                    child_len <= 6'd0;
                    child_is_dir <= 1'b0;
                    child_done <= 1'b0;
                    bstate <= B_SCAN_NAME;
                end

                B_SCAN_NAME: begin
                    if (child_done || name_idx >= cand_name_len[7:0]) begin
                        bstate <= B_SCAN_DECIDE;
                    end else if (rd_req && rd_ready) begin
                        bstate <= B_SCAN_NAME_WAIT;
                    end
                end

                B_SCAN_NAME_WAIT: begin
                    if (rd_valid && rd_inflight) begin
                        if (name_idx < {1'b0, path_len}) begin
                            if (!lower_eq(rd_data, path_buf[name_idx[5:0]]))
                                prefix_ok <= 1'b0;
                            name_idx <= name_idx + 8'd1;
                            bstate <= B_SCAN_NAME;
                        end else if (!prefix_ok) begin
                            bstate <= B_SCAN_DECIDE;
                        end else if (rd_data == 8'h2F) begin
                            child_is_dir <= 1'b1;
                            child_done <= 1'b1;
                            bstate <= B_SCAN_DECIDE;
                        end else begin
                            if (child_len < 6'd32) begin
                                child_buf[child_len[4:0]] <= rd_data;
                                child_len <= child_len + 6'd1;
                            end
                            name_idx <= name_idx + 8'd1;
                            bstate <= B_SCAN_NAME;
                        end
                    end
                end

                B_SCAN_DECIDE: begin
                    if (!prefix_ok || child_len == 0) begin
                        bstate <= B_SCAN_NEXT;
                    end else if (child_is_dir) begin
                        dedup_idx <= (path_len != 0) ? 6'd1 : 6'd0;
                        dedup_byte <= 6'd0;
                        bstate <= B_DEDUP_SET;
                    end else if (cand_is_m3u) begin
                        add_is_dir <= 1'b0;
                        add_entry <= cand_idx;
                        add_idx <= 6'd0;
                        bstate <= B_ADD;
                    end else begin
                        bstate <= B_SCAN_NEXT;
                    end
                end

                B_DEDUP_SET: begin
                    if (dedup_idx >= left_count) begin
                        add_is_dir <= 1'b1;
                        add_entry <= cand_idx;
                        add_idx <= 6'd0;
                        bstate <= B_ADD;
                    end else if (!left_is_dir[dedup_idx]) begin
                        dedup_idx <= dedup_idx + 6'd1;
                        dedup_byte <= 6'd0;
                        bstate <= B_DEDUP_SET;
                    end else begin
                        lname_ra <= {dedup_idx, dedup_byte[4:0]};
                        bstate <= B_DEDUP_WAIT;
                    end
                end

                B_DEDUP_WAIT: begin
                    bstate <= B_DEDUP_GET;
                end

                B_DEDUP_GET: begin
                    if (dedup_byte < child_len) begin
                        if (lname_rd != child_buf[dedup_byte[4:0]]) begin
                            dedup_idx <= dedup_idx + 6'd1;
                            dedup_byte <= 6'd0;
                            bstate <= B_DEDUP_SET;
                        end else begin
                            dedup_byte <= dedup_byte + 6'd1;
                            bstate <= B_DEDUP_SET;
                        end
                    end else begin
                        if (lname_rd == 8'd0) begin
                            bstate <= B_SCAN_NEXT;   // duplicate dir
                        end else begin
                            dedup_idx <= dedup_idx + 6'd1;
                            dedup_byte <= 6'd0;
                            bstate <= B_DEDUP_SET;
                        end
                    end
                end

                B_ADD: begin
                    if (add_idx < child_len) begin
                        lname_we <= 1'b1;
                        lname_wa <= {left_count, add_idx[4:0]};
                        lname_wd <= child_buf[add_idx[4:0]];
                        add_idx <= add_idx + 6'd1;
                    end else if (add_idx == child_len) begin
                        lname_we <= 1'b1;
                        lname_wa <= {left_count, add_idx[4:0]};
                        lname_wd <= 8'd0;
                        add_idx <= add_idx + 6'd1;
                    end else begin
                        left_we <= 1'b1;
                        left_wa <= left_count;
                        left_wd <= {add_is_dir, add_entry, child_len};
                        left_is_dir[left_count] <= add_is_dir;
                        left_entry[left_count] <= add_entry;
                        left_namelen[left_count] <= child_len;
                        left_count <= left_count + 6'd1;
                        bstate <= B_SCAN_NEXT;
                    end
                end

                B_SCAN_NEXT: begin
                    if (cand_idx + 9'd1 >= entry_count) begin
                        bstate <= B_SCAN_DONE;
                    end else begin
                        cand_idx <= cand_idx + 9'd1;
                        bstate <= B_SCAN_TBL;
                    end
                end

                B_SCAN_DONE: begin
                    scan_done <= 1'b1;
                    left_top <= 6'd0;
                    d_dirty <= 1'b1;
                    bstate <= B_IDLE;
                end

                // ------------------------------------------------ path change
                B_PATH_APPEND_SET: begin
                    lname_ra <= {left_cursor, path_app_idx[4:0]};
                    bstate <= B_PATH_APPEND_WAIT;
                end

                B_PATH_APPEND_WAIT: begin
                    bstate <= B_PATH_APPEND_GET;
                end

                B_PATH_APPEND_GET: begin
                    if (path_app_idx < left_namelen[left_cursor]) begin
                        path_buf[path_len[5:0] + path_app_idx[5:0]] <= lname_rd;
                        path_app_idx <= path_app_idx + 6'd1;
                        bstate <= B_PATH_APPEND_SET;
                    end else begin
                        path_buf[path_len[5:0] + left_namelen[left_cursor][5:0]] <= 8'h2F;
                        path_len <= path_len + {1'b0, left_namelen[left_cursor]} + 7'd1;
                        bstate <= B_SCAN_INIT;
                    end
                end

                B_PATH_UP: begin
                    if (path_buf[path_scan_idx] == 8'h2F) begin
                        path_len <= {1'b0, path_scan_idx} + 7'd1;
                        bstate <= B_SCAN_INIT;
                    end else if (path_scan_idx == 6'd0) begin
                        path_len <= 7'd0;
                        bstate <= B_SCAN_INIT;
                    end else begin
                        path_scan_idx <= path_scan_idx - 6'd1;
                    end
                end

                // ------------------------------------------------ song loader
                B_SONG_PL: begin
                    pl_ra <= song_idx;
                    bstate <= B_SONG_PL_WAIT;
                end

                B_SONG_PL_WAIT: begin
                    bstate <= B_SONG_PL_GOT;
                end

                B_SONG_PL_GOT: begin
                    song_entry <= pl_rd[8:0];
                    tbl_ra <= pl_rd[8:0];
                    bstate <= B_SONG_TBL_WAIT;
                end

                B_SONG_TBL_WAIT: begin
                    bstate <= B_SONG_TBL_GOT;
                end

                B_SONG_TBL_GOT: begin
                    song_name_ptr <= tbl_name_ptr;
                    song_name_len <= tbl_name_len;
                    song_byte_idx <= 8'd0;
                    song_base_len <= 6'd0;
                    bstate <= B_SONG_NAME;
                end

                B_SONG_NAME: begin
                    if (song_byte_idx >= song_name_len[7:0]) begin
                        song_store_idx <= 6'd0;
                        bstate <= B_SONG_STORE;
                    end else if (rd_req && rd_ready) begin
                        bstate <= B_SONG_NAME_WAIT;
                    end
                end

                B_SONG_NAME_WAIT: begin
                    if (rd_valid && rd_inflight) begin
                        if (rd_data == 8'h2F) begin
                            // '/': start a new basename.
                            song_base_len <= 6'd0;
                        end else if (song_base_len < 6'd32) begin
                            song_base_buf[song_base_len[4:0]] <= rd_data;
                            song_base_len <= song_base_len + 6'd1;
                        end
                        song_byte_idx <= song_byte_idx + 8'd1;
                        bstate <= B_SONG_NAME;
                    end
                end

                B_SONG_STORE: begin
                    if (song_store_idx < song_base_len) begin
                        sname_we <= 1'b1;
                        sname_wa <= {song_idx, song_store_idx[4:0]};
                        sname_wd <= song_base_buf[song_store_idx[4:0]];
                        song_store_idx <= song_store_idx + 6'd1;
                    end else if (song_store_idx == song_base_len) begin
                        sname_we <= 1'b1;
                        sname_wa <= {song_idx, song_store_idx[4:0]};
                        sname_wd <= 8'd0;
                        song_store_idx <= song_store_idx + 6'd1;
                    end else begin
                        song_namelen[song_idx] <= song_base_len;
                        bstate <= B_SONG_NEXT;
                    end
                end

                B_SONG_NEXT: begin
                    if (song_idx + 7'd1 >= playlist_count) begin
                        bstate <= B_SONG_DONE;
                    end else begin
                        song_idx <= song_idx + 7'd1;
                        bstate <= B_SONG_PL;
                    end
                end

                B_SONG_DONE: begin
                    songs_ready <= 1'b1;
                    d_dirty <= 1'b1;
                    bstate <= B_IDLE;
                end

                default: bstate <= B_IDLE;
            endcase

            // ---- display writer (runs only when the scan is idle) ----
            if (bstate == B_IDLE) begin
                case (dstate)
                    D_IDLE: begin
                        if (d_dirty) begin
                            d_row <= 4'd0;
                            d_col <= 6'd0;
                            dstate <= D_HDR;
                        end
                    end

                    D_HDR: begin
                        // Row 0: the path (left), divider at col 22.
                        // One char per cycle, cols 0..45.
                        char_we <= 1'b1;
                        char_wa <= {d_row, d_col};
                        if (d_col == 6'd22) begin
                            char_wd <= {wr_attr, 8'h7C}; // '|'
                        end else if (d_col < path_len[5:0]) begin
                            char_wd <= {wr_attr, path_buf[d_col[5:0]]};
                        end else if (d_col == 6'd0) begin
                            char_wd <= {wr_attr, 8'h2F}; // '/' at root
                        end else begin
                            char_wd <= {wr_attr, 8'h20}; // space
                        end
                        if (d_col == 6'd45) begin
                            d_row <= 4'd1;
                            d_col <= 6'd0;
                            d_item <= left_top;
                            d_char <= 6'd0;
                            dstate <= D_ITEM_SET;
                        end else begin
                            d_col <= d_col + 6'd1;
                        end
                    end

                    D_ITEM_SET: begin
                        if (d_row > 4'(DISP_ROWS)) begin
                            dstate <= D_DONE;
                        end else if ((left_top + ({2'b0, d_row} - 6'd1)) >= left_count) begin
                            d_col <= 6'd0;
                            dstate <= D_ITEM_BLANK;
                        end else begin
                            d_item <= left_top + ({2'b0, d_row} - 6'd1);
                            d_col <= 6'd0;
                            d_char <= 6'd0;
                            dstate <= D_ITEM_MARK;
                        end
                    end

                    D_ITEM_MARK: begin
                        // Col 0: cursor marker (">" on the left cursor,
                        // only while the left pane has focus), "v" on the
                        // last visible row when more entries sit below it,
                        // else blank. Set up the first name-char read.
                        char_we <= 1'b1;
                        char_wa <= {d_row, 6'd0};
                        char_wd <= {wr_attr, (!active_right && (d_item == left_cursor)) ? 8'h3E :
                                              ((d_row == 4'(DISP_ROWS)) && ((left_top + 6'(DISP_ROWS)) < left_count)) ? 8'h76 : 8'h20};
                        lname_ra <= {d_item, 5'd0};
                        d_col <= 6'd1;
                        d_char <= 6'd0;
                        dstate <= D_ITEM_WAIT;
                    end

                    D_ITEM_WAIT: begin
                        dstate <= D_ITEM_CH;
                    end

                    D_ITEM_CH: begin
                        // lname_rd is the name char at d_char.
                        if (d_col <= 6'd21 && d_char < left_namelen[d_item]) begin
                            char_we <= 1'b1;
                            char_wa <= {d_row, d_col};
                            char_wd <= {wr_attr, lname_rd};
                            d_char <= d_char + 6'd1;
                            d_col <= d_col + 6'd1;
                            if ((d_char + 6'd1) < left_namelen[d_item] && (d_col + 6'd1) <= 6'd21) begin
                                lname_ra <= {d_item, d_char[4:0] + 5'd1};
                                dstate <= D_ITEM_WAIT;
                            end else begin
                                dstate <= D_ITEM_PAD;
                            end
                        end else begin
                            dstate <= D_ITEM_PAD;
                        end
                    end

                    D_ITEM_PAD: begin
                        // Fill cols d_col..21 with spaces, then the
                        // divider at 22, then the gap cols 23..25 with
                        // spaces (the right pane starts at col 26).
                        if (d_col <= 6'd21) begin
                            char_we <= 1'b1;
                            char_wa <= {d_row, d_col};
                            char_wd <= {wr_attr, 8'h20};
                            d_col <= d_col + 6'd1;
                        end else if (d_col == 6'd22) begin
                            char_we <= 1'b1;
                            char_wa <= {d_row, 6'd22};
                            char_wd <= {wr_attr, 8'h7C};
                            d_col <= d_col + 6'd1;
                        end else if (d_col <= 6'd25) begin
                            char_we <= 1'b1;
                            char_wa <= {d_row, d_col};
                            char_wd <= {wr_attr, 8'h20};
                            d_col <= d_col + 6'd1;
                        end else begin
                            d_row <= d_row + 4'd1;
                            d_col <= 6'd0;
                            d_char <= 6'd0;
                            dstate <= D_ITEM_SET;
                        end
                    end

                    D_ITEM_BLANK: begin
                        if (d_col <= 6'd21) begin
                            char_we <= 1'b1;
                            char_wa <= {d_row, d_col};
                            char_wd <= {wr_attr, 8'h20};
                            d_col <= d_col + 6'd1;
                        end else if (d_col == 6'd22) begin
                            char_we <= 1'b1;
                            char_wa <= {d_row, 6'd22};
                            char_wd <= {wr_attr, 8'h7C};
                            d_col <= d_col + 6'd1;
                        end else if (d_col <= 6'd25) begin
                            char_we <= 1'b1;
                            char_wa <= {d_row, d_col};
                            char_wd <= {wr_attr, 8'h20};
                            d_col <= d_col + 6'd1;
                        end else begin
                            d_row <= d_row + 4'd1;
                            d_col <= 6'd0;
                            dstate <= D_ITEM_SET;
                        end
                    end

                    D_DONE: begin
                        if (songs_ready && playlist_active) begin
                            d_row <= 4'd0;
                            d_col <= 6'(R_MARK);
                            d_song <= r_top;
                            d_rchar <= 6'd0;
                            dstate <= D_R_HDR;
                        end else begin
                            d_dirty <= 1'b0;
                            dstate <= D_IDLE;
                        end
                    end

                    // ---- right column (playlist) ----
                    D_R_HDR: begin
                        // Row 0, cols 26..45: "N/M  SSs" (current song /
                        // total, elapsed seconds). One char per cycle.
                        char_we <= 1'b1;
                        char_wa <= {d_row, d_col};
                        case (d_col)
                            6'(R_MARK + 0): char_wd <= {wr_attr, hdr_cur[23:16]};
                            6'(R_MARK + 1): char_wd <= {wr_attr, hdr_cur[15:8]};
                            6'(R_MARK + 2): char_wd <= {wr_attr, hdr_cur[7:0]};
                            6'(R_MARK + 3): char_wd <= {wr_attr, 8'h2F}; // '/'
                            6'(R_MARK + 4): char_wd <= {wr_attr, hdr_tot[23:16]};
                            6'(R_MARK + 5): char_wd <= {wr_attr, hdr_tot[15:8]};
                            6'(R_MARK + 6): char_wd <= {wr_attr, hdr_tot[7:0]};
                            6'(R_MARK + 7): char_wd <= {wr_attr, 8'h20};
                            6'(R_MARK + 8): char_wd <= {wr_attr, 8'h20};
                            6'(R_MARK + 9): char_wd <= {wr_attr, hdr_sec[23:16]};
                            6'(R_MARK + 10): char_wd <= {wr_attr, hdr_sec[15:8]};
                            6'(R_MARK + 11): char_wd <= {wr_attr, hdr_sec[7:0]};
                            6'(R_MARK + 12): char_wd <= {wr_attr, 8'h73}; // 's'
                            default: char_wd <= {wr_attr, 8'h20};
                        endcase
                        if (d_col == 6'(R_LAST)) begin
                            d_row <= 4'd1;
                            d_col <= 6'(R_MARK);
                            dstate <= D_R_BLANK;
                        end else begin
                            d_col <= d_col + 6'd1;
                        end
                    end

                    D_R_BLANK: begin
                        // Row 1 is an empty line between the header and
                        // the song list.
                        if (d_col <= 6'(R_LAST)) begin
                            char_we <= 1'b1;
                            char_wa <= {d_row, d_col};
                            char_wd <= {wr_attr, 8'h20};
                            d_col <= d_col + 6'd1;
                        end else begin
                            d_row <= 4'd2;
                            d_col <= 6'(R_MARK);
                            d_song <= r_top;
                            d_rchar <= 6'd0;
                            dstate <= D_R_ITEM_SET;
                        end
                    end

                    D_R_ITEM_SET: begin
                        // Songs start at row 2 (row 1 is the blank line),
                        // so song index = r_top + (d_row - 2); 13 visible.
                        if (d_row > 4'(DISP_ROWS)) begin
                            dstate <= D_R_DONE;
                        end else if ((r_top + ({2'b0, d_row} - 6'd2)) >= playlist_count) begin
                            d_col <= 6'(R_MARK);
                            dstate <= D_R_ITEM_BLANK;
                        end else begin
                            d_song <= r_top + ({2'b0, d_row} - 6'd2);
                            d_col <= 6'(R_MARK);
                            d_rchar <= 6'd0;
                            dstate <= D_R_ITEM_MARK;
                        end
                    end

                    D_R_ITEM_MARK: begin
                        // Marker col: ">" on the right cursor (only while
                        // the right pane has focus), "v" on the last
                        // visible row when more songs sit below it, else
                        // blank. The playing song is colored, not marked.
                        char_we <= 1'b1;
                        char_wa <= {d_row, 6'(R_MARK)};
                        char_wd <= {wr_attr, (active_right && (d_song == right_cursor)) ? 8'h3E :
                                              ((d_row == 4'(DISP_ROWS)) && ((r_top + 7'(DISP_ROWS - 1)) < playlist_count)) ? 8'h76 : 8'h20};
                        sname_ra <= {d_song, 5'd0};
                        d_col <= 6'(R_NAME);
                        d_rchar <= 6'd0;
                        dstate <= D_R_ITEM_WAIT;
                    end

                    D_R_ITEM_WAIT: begin
                        dstate <= D_R_ITEM_CH;
                    end

                    D_R_ITEM_CH: begin
                        // sname_rd is the name char at d_rchar.
                        if (d_col <= 6'(R_LAST) && d_rchar < song_namelen[d_song]) begin
                            char_we <= 1'b1;
                            char_wa <= {d_row, d_col};
                            char_wd <= {wr_attr, sname_rd};
                            d_rchar <= d_rchar + 6'd1;
                            d_col <= d_col + 6'd1;
                            if ((d_rchar + 6'd1) < song_namelen[d_song] && (d_col + 6'd1) <= 6'(R_LAST)) begin
                                sname_ra <= {d_song, d_rchar[4:0] + 5'd1};
                                dstate <= D_R_ITEM_WAIT;
                            end else begin
                                dstate <= D_R_ITEM_PAD;
                            end
                        end else begin
                            dstate <= D_R_ITEM_PAD;
                        end
                    end

                    D_R_ITEM_PAD: begin
                        if (d_col <= 6'(R_LAST)) begin
                            char_we <= 1'b1;
                            char_wa <= {d_row, d_col};
                            char_wd <= {wr_attr, 8'h20};
                            d_col <= d_col + 6'd1;
                        end else begin
                            d_row <= d_row + 4'd1;
                            d_col <= 6'(R_MARK);
                            d_rchar <= 6'd0;
                            dstate <= D_R_ITEM_SET;
                        end
                    end

                    D_R_ITEM_BLANK: begin
                        if (d_col <= 6'(R_LAST)) begin
                            char_we <= 1'b1;
                            char_wa <= {d_row, d_col};
                            char_wd <= {wr_attr, 8'h20};
                            d_col <= d_col + 6'd1;
                        end else begin
                            d_row <= d_row + 4'd1;
                            d_col <= 6'(R_MARK);
                            dstate <= D_R_ITEM_SET;
                        end
                    end

                    D_R_DONE: begin
                        d_dirty <= 1'b0;
                        dstate <= D_IDLE;
                    end

                    default: dstate <= D_IDLE;
                endcase
            end
        end
    end
endmodule
