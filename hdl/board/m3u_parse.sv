// SPDX-License-Identifier: GPL-2.0-or-later
// m3u_parse: parse a .m3u playlist that zip_load extracted into the MID
// window (DDR3 at 0x440000) and resolve each song line to an entry-table
// index, building a playlist of up to 100 songs.
//
// Resolution rules (from Joe's MIDI.zip):
//   * lines are relative to the .m3u's own directory (e.g. GM/TEST.M3U
//     listing AKUMA68K\11_BELMO.MID means GM/AKUMA68K/11_BELMO.MID)
//   * backslashes are normalised to slashes
//   * matching is case-insensitive over the full path (35 of 327 lines in
//     MIDI.zip only match case-insensitively)
//   * blank lines and '#' comment lines are skipped
//   * paths up to 5 directory levels are supported (name buffer 160 bytes)
//   * unresolvable lines are skipped, not fatal
//
// The parser reads the .m3u text and the entry names through the shared
// archive byte-read port (rd_req/rd_ready/rd_valid/rd_data, the same
// ready/valid protocol as wave_ddr's archive port) and reads entry records
// through the table port (tbl_ra/tbl_rd). It runs only while zip_load is
// idle, so the board muxes the archive port to it; it never runs at the
// same time as a parse or extract.
//
// The playlist is stored in a 128x9 sdp_ram (100 songs max, 9-bit entry
// indices 0..511) with a read port (pl_ra/pl_rd) for the browser and the
// sequencer. `done` pulses when parsing finishes; `playlist_count` is the
// number of songs resolved (0..100). `parse_failed` is latched when no
// song resolved.
module m3u_parse (
    input  logic        clk,
    input  logic        rst,
    // control
    input  logic        start,          // pulse: parse the .m3u in the MID window
    input  logic [22:0] m3u_len,        // length of .m3u text in the MID window
    input  logic [8:0]  m3u_entry_idx,  // table index of the .m3u entry (for its dir)
    input  logic [9:0]  entry_count,    // number of entries in the table
    output logic        done,           // pulse: parse finished
    output logic        busy,           // high while parsing (owns the archive port)
    output logic        parse_ok,       // 1 if playlist_count > 0
    output logic [6:0]  playlist_count, // songs resolved (0..100)
    output logic        parse_failed,   // latched: no songs resolved
    // entry-table read port (to zip_load's tbl_ram)
    output logic [8:0]  tbl_ra,
    input  logic [191:0] tbl_rd,
    // archive byte-read port (shared with zip_load; board muxes it)
    output logic        rd_req,
    output logic [25:0] rd_addr,
    input  logic        rd_ready,
    input  logic        rd_valid,
    input  logic [7:0]  rd_data,
    // playlist RAM read ports (sequencer + browser). The playlist is
    // written once by the parser; two RAMs with the same writes give
    // each reader its own port, so the browser's reads can never steal
    // the sequencer's 2-cycle advance.
    input  logic [6:0]  pl_ra,
    output logic [8:0]  pl_rd,
    input  logic [6:0]  pl_ra_b,
    output logic [8:0]  pl_rd_b
);
    localparam [25:0] MID_BASE  = 26'h440000;  // .m3u text lives in the MID window
    localparam [25:0] ARCH_BASE = 26'h800000;  // entry names live in the archive
    localparam int PL_MAX = 100;

    // Table record layout (zip_load): [25:0] name_ptr, [41:26] name_len,
    // [57:42] method, [89:58] crc, [121:90] comp_size, [153:122] uncomp_size,
    // [185:154] local_hdr_off, [186] is_dir, [187] is_m3u.
    wire [25:0] tbl_name_ptr = tbl_rd[25:0];
    wire [15:0] tbl_name_len = tbl_rd[41:26];

    assign busy = (mstate != M_IDLE);

    typedef enum logic [3:0] {
        M_IDLE,
        M_TBL_M3U,      // read the .m3u entry's table record
        M_TBL_M3U_GOT,  // latch name_ptr/name_len (registered table read)
        M_DIR_RD,       // read the .m3u name, keep the dir prefix
        M_DIR_WAIT,
        M_LINE_RD,      // read .m3u text bytes, build one line
        M_LINE_WAIT,
        M_LINE_DONE,    // line complete: normalise + resolve (or skip)
        M_SCAN_TBL,     // read candidate entry's table record
        M_SCAN_TBL_GOT, // latch candidate name_ptr/name_len
        M_NAME_RD,      // read candidate name bytes, compare with path
        M_NAME_WAIT,
        M_NEXT_CAND,    // advance to the next candidate entry
        M_NEXT_LINE,    // advance to the next .m3u line
        M_DONE
    } mstate_t;
    mstate_t mstate;

    // ---- buffers ----
    // dir_buf: the .m3u's directory prefix, e.g. "GM/" (with trailing slash).
    logic [7:0]  dir_buf [0:63];
    logic [6:0]  dir_len;               // 0..64
    // path_len: full resolved path length (dir + line). The path bytes are
    // read through path_byte() (dir first, then the line); no separate
    // path buffer is needed.
    logic [7:0]  path_len;              // 0..160
    // line_buf: raw line bytes (normalised) before the dir is prepended.
    logic [7:0]  line_buf [0:127];
    logic [7:0]  line_len;              // 0..128

    // ---- read-port handshake ----
    // rd_req is asserted only in a *_RD state that actually wants a byte
    // this cycle, and only while no read is in flight. (This is the same
    // discipline as zip_load's fixed areq gating: asserting the request in
    // a cycle that transitions away without wanting a read issues a
    // spurious read and desyncs the single-bit in-flight flag. Here the
    // spurious read was the byte past the .m3u name, 'P', latched as the
    // first .m3u byte and shifting the whole line by one.)
    logic rd_inflight;
    wire  want_read = ((mstate == M_DIR_RD)  && (rd_idx < m3u_name_len[6:0])) ||
                      ((mstate == M_LINE_RD) && (m3u_pos < m3u_len)) ||
                      ((mstate == M_NAME_RD) && name_match &&
                       (name_idx < cand_name_len[7:0]) && (name_idx < path_len));
    assign rd_req  = want_read && !rd_inflight;
    assign rd_addr = (mstate == M_DIR_RD)  ? (ARCH_BASE + m3u_name_ptr + {19'd0, rd_idx}) :
                     (mstate == M_LINE_RD) ? (MID_BASE + {3'd0, m3u_pos}) :
                                             (ARCH_BASE + cand_name_ptr + {18'd0, name_idx});
    always_ff @(posedge clk) begin
        if (rst) rd_inflight <= 1'b0;
        else if (rd_valid) rd_inflight <= 1'b0;
        else if (rd_req && rd_ready) rd_inflight <= 1'b1;
    end

    // ---- state / counters ----
    logic [25:0] m3u_name_ptr;
    logic [15:0] m3u_name_len;
    logic [6:0]  rd_idx;                // byte index within the name being read
    logic [22:0] m3u_pos;               // byte offset within the .m3u text
    logic [8:0]  cand_idx;              // candidate entry being scanned
    logic [25:0] cand_name_ptr;
    logic [15:0] cand_name_len;
    logic [7:0]  name_idx;              // byte index within the candidate name
    logic        name_match;            // still matching so far
    logic [6:0]  pl_count;              // songs resolved
    logic [6:0]  pl_wr_idx;             // playlist write index

    // playlist RAM (128x9; 100 songs max) - two copies, same writes,
    // one read port each (sequencer + browser).
    logic        pl_we;
    logic [6:0]  pl_wa;
    logic [8:0]  pl_wd;
    sdp_ram #(.AW(7), .DW(9)) pl_ram (
        .clk(clk), .we(pl_we), .wa(pl_wa), .wd(pl_wd),
        .ra(pl_ra), .rd(pl_rd));
    sdp_ram #(.AW(7), .DW(9)) pl_ram_b (
        .clk(clk), .we(pl_we), .wa(pl_wa), .wd(pl_wd),
        .ra(pl_ra_b), .rd(pl_rd_b));

    function automatic logic [7:0] lower(input logic [7:0] c);
        lower = (c >= 8'h41 && c <= 8'h5A) ? (c + 8'd32) : c;
    endfunction

    always_ff @(posedge clk) begin
        // defaults (1-clock pulses)
        done <= 1'b0;
        pl_we <= 1'b0;
        tbl_ra <= tbl_ra;   // hold unless assigned below
        if (rst) begin
            mstate <= M_IDLE;
            parse_ok <= 1'b0;
            playlist_count <= 7'd0;
            parse_failed <= 1'b0;
            dir_len <= 7'd0;
            path_len <= 8'd0;
            line_len <= 8'd0;
            pl_count <= 7'd0;
        end else begin
            case (mstate)
            M_IDLE: begin
                if (start) begin
                    parse_ok <= 1'b0;
                    parse_failed <= 1'b0;
                    pl_count <= 7'd0;
                    pl_wr_idx <= 7'd0;
                    m3u_pos <= 23'd0;
                    line_len <= 8'd0;
                    dir_len <= 7'd0;
                    tbl_ra <= m3u_entry_idx;
                    mstate <= M_TBL_M3U;
                end
            end
            // The table read is registered: the record is valid the cycle
            // after tbl_ra is presented.
            M_TBL_M3U: begin
                mstate <= M_TBL_M3U_GOT;
            end
            M_TBL_M3U_GOT: begin
                m3u_name_ptr <= tbl_name_ptr;
                m3u_name_len <= tbl_name_len;
                rd_idx <= 7'd0;
                mstate <= M_DIR_RD;
            end
            // Read the .m3u entry's name; keep everything up to and
            // including the last '/' as the directory prefix.
            M_DIR_RD: begin
                if (rd_idx >= m3u_name_len[6:0]) begin
                    mstate <= M_LINE_RD;
                end else if (rd_ready) begin
                    mstate <= M_DIR_WAIT;
                end
            end
            M_DIR_WAIT: begin
                if (rd_valid && rd_inflight) begin
                    if (rd_data == 8'h2F) dir_len <= rd_idx + 7'd1;  // '/'
                    if (rd_idx < 7'd64) dir_buf[rd_idx[5:0]] <= rd_data;
                    rd_idx <= rd_idx + 7'd1;
                    mstate <= M_DIR_RD;
                end
            end
            // Read .m3u text bytes into line_buf until newline / end.
            M_LINE_RD: begin
                if (m3u_pos >= m3u_len) begin
                    // End of file: finish the pending line, then done.
                    mstate <= M_LINE_DONE;
                end else if (rd_ready) begin
                    mstate <= M_LINE_WAIT;
                end
            end
            M_LINE_WAIT: begin
                if (rd_valid && rd_inflight) begin
                    m3u_pos <= m3u_pos + 23'd1;
                    if (rd_data == 8'h0A) begin
                        // newline: line complete
                        mstate <= M_LINE_DONE;
                    end else if (rd_data == 8'h0D) begin
                        // CR: ignore (CRLF line endings)
                        mstate <= M_LINE_RD;
                    end else if (line_len < 8'd128) begin
                        line_buf[line_len[6:0]] <= (rd_data == 8'h5C) ? 8'h2F : rd_data;
                        line_len <= line_len + 8'd1;
                        mstate <= M_LINE_RD;
                    end else begin
                        // line too long: keep consuming until newline
                        mstate <= M_LINE_RD;
                    end
                end
            end
            // A line is complete (or EOF). Skip blanks and '#' comments;
            // otherwise build the full path (dir + line) and resolve it.
            M_LINE_DONE: begin
                if (line_len == 8'd0 || line_buf[0] == 8'h23) begin
                    // blank or comment: next line
                    line_len <= 8'd0;
                    mstate <= M_NEXT_LINE;
                end else if (pl_count >= 7'(PL_MAX)) begin
                    // playlist full: stop
                    mstate <= M_DONE;
                end else begin
                    // Full path = dir + line (read via path_byte() during
                    // the resolve scan); start scanning candidate entries.
                    path_len <= dir_len + line_len[7:0];
                    cand_idx <= 9'd0;
                    tbl_ra <= 9'd0;
                    mstate <= M_SCAN_TBL;
                end
            end
            // Read candidate entry's table record.
            M_SCAN_TBL: begin
                mstate <= M_SCAN_TBL_GOT;
            end
            M_SCAN_TBL_GOT: begin
                cand_name_ptr <= tbl_name_ptr;
                cand_name_len <= tbl_name_len;
                name_idx <= 8'd0;
                name_match <= 1'b1;
                mstate <= M_NAME_RD;
            end
            // Read candidate name bytes, compare with path_buf (dir+line).
            // path byte i = (i < dir_len) ? dir_buf[i] : line_buf[i-dir_len].
            M_NAME_RD: begin
                if (!name_match) begin
                    // already mismatched: skip the rest of this name
                    mstate <= M_NEXT_CAND;
                end else if (name_idx >= cand_name_len[7:0]) begin
                    // candidate name exhausted: match iff path exhausted too
                    if (name_idx == path_len) begin
                        // full match: record this entry in the playlist
                        pl_we <= 1'b1;
                        pl_wa <= pl_wr_idx;
                        pl_wd <= cand_idx;
                        pl_wr_idx <= pl_wr_idx + 7'd1;
                        pl_count <= pl_count + 7'd1;
                        line_len <= 8'd0;
                        mstate <= M_NEXT_LINE;
                    end else begin
                        mstate <= M_NEXT_CAND;
                    end
                end else if (name_idx >= path_len) begin
                    // path exhausted but name longer: no match
                    name_match <= 1'b0;
                    mstate <= M_NEXT_CAND;
                end else if (rd_ready) begin
                    mstate <= M_NAME_WAIT;
                end
            end
            M_NAME_WAIT: begin
                if (rd_valid && rd_inflight) begin
                    if (lower(rd_data) != lower(path_byte(name_idx))) begin
                        name_match <= 1'b0;
                    end
                    name_idx <= name_idx + 8'd1;
                    mstate <= M_NAME_RD;
                end
            end
            M_NEXT_CAND: begin
                // Advance to the next candidate entry (or give up on this line).
                if (cand_idx + 9'd1 >= entry_count) begin
                    // no match for this line: skip it
                    line_len <= 8'd0;
                    mstate <= M_NEXT_LINE;
                end else begin
                    cand_idx <= cand_idx + 9'd1;
                    tbl_ra <= cand_idx + 9'd1;
                    mstate <= M_SCAN_TBL;
                end
            end
            M_NEXT_LINE: begin
                if (m3u_pos >= m3u_len && line_len == 8'd0) begin
                    mstate <= M_DONE;
                end else begin
                    mstate <= M_LINE_RD;
                end
            end
            M_DONE: begin
                playlist_count <= pl_count;
                parse_ok <= (pl_count != 7'd0);
                parse_failed <= (pl_count == 7'd0);
                done <= 1'b1;
                mstate <= M_IDLE;
            end
            default: mstate <= M_IDLE;
            endcase
        end
    end

    // path byte i: dir first, then the line.
    function automatic logic [7:0] path_byte(input logic [7:0] i);
        logic [7:0] li;
        li = i - {1'b0, dir_len};
        if (i < {1'b0, dir_len}) path_byte = dir_buf[i[5:0]];
        else path_byte = line_buf[li[6:0]];
    endfunction
endmodule
