// SPDX-License-Identifier: GPL-2.0-or-later
// zip_dir.sv — ZIP central-directory parser for the SC-55 MIDI player archive browser.
//
// The ZIP archive is already stored in DDR3 (byte-addressed, base added by
// the caller). This module finds the End Of Central Directory record by
// scanning backwards from the end (up to 64 KiB + 22 B, covering an EOCD
// comment), validates it, then walks the central directory and emits one
// record per entry: name pointer/length, compression method, CRC-32, sizes,
// local-header offset, is_dir / is_m3u. Entry names are NOT copied —
// name_ptr/name_len point at the name bytes inside the central directory
// and the caller reads them from DDR3 when displaying them.
//
// Byte-read port contract: pulse rd_req with rd_addr; the DDR3 side answers
// with rd_valid + rd_data after an arbitrary latency (one byte per req).
// Methods other than 0 (stored) and 8 (deflate) are reported, not rejected;
// the caller refuses them (Deflate64 (9) is refused by policy).
// ZIP64 is not supported: archives are < 40 MB, so the 32-bit fields hold.

module zip_dir (
    input  wire        clk,
    input  wire        rst,
    input  wire        start,          // pulse to (re)start a parse
    input  wire [25:0] arch_len,       // archive length in bytes
    // DDR3 byte-read port (byte address within the archive)
    output reg         rd_req,
    output reg  [25:0] rd_addr,
    input  wire [7:0]  rd_data,
    input  wire        rd_valid,
    // One record per entry (pulsed with entry_valid)
    output reg         entry_valid,
    output reg  [25:0] entry_name_ptr,
    output reg  [15:0] entry_name_len,
    output reg  [15:0] entry_method,
    output reg  [31:0] entry_crc,
    output reg  [31:0] entry_comp_size,
    output reg  [31:0] entry_uncomp_size,
    output reg  [31:0] entry_local_hdr_off,
    output reg         entry_is_dir,
    output reg         entry_is_m3u,
    output reg  [15:0] entry_count,    // entries emitted so far
    output reg         done,
    output reg  [2:0]  error,          // 0 none, 1 no EOCD, 2 bad CD sig,
                                        // 3 CD overrun, 4 archive too small
    // Parse-stage diagnostics (latched; cleared on reset and on start):
    // started = left S_IDLE, eocd_found = EOCD signature found,
    // cd_started = central-directory walk began. Together with the
    // read-accepted / read-returned flags in zip_load they show exactly
    // where a stuck parse stops on hardware.
    output reg         started,
    output reg         eocd_found,
    output reg         cd_started
);

    localparam [3:0] S_IDLE    = 4'd0,
                     S_SCAN    = 4'd1,
                     S_EOCD    = 4'd2,
                     S_CD_HDR  = 4'd3,
                     S_CD_TAIL = 4'd4,
                     S_CD_EMIT = 4'd5,
                     S_DONE    = 4'd6,
                     S_ERROR   = 4'd7;

    reg [3:0]  state;
    reg [25:0] len_r;
    reg [25:0] scan_pos;       // next scan read position (descending)
    reg [25:0] scan_end;       // lowest scan position
    reg [31:0] win;            // 4-byte sliding window for EOCD signature
    reg        rd_pending;
    reg [25:0] eocd_pos;
    reg [4:0]  eocd_idx;       // byte index within EOCD (0..21)
    reg [31:0] acc;            // little-endian field assembler
    reg [31:0] cd_size;
    reg [25:0] cd_offset;
    reg [15:0] cd_entries;
    reg [15:0] comment_len;
    reg [25:0] cd_pos;         // current central-directory entry position
    reg [15:0] count;
    reg [5:0]  hdr_idx;        // byte index within 46-byte CD header
    reg [15:0] method_r;
    reg [31:0] crc_r;
    reg [31:0] comp_size_r;
    reg [31:0] uncomp_size_r;
    reg [15:0] name_len_r;
    reg [15:0] extra_len_r;
    reg [15:0] comment_len_r;
    reg [31:0] local_hdr_off_r;
    reg [25:0] name_ptr_r;
    reg [2:0]  tail_idx;       // bytes read of the name tail (0..3)
    reg [2:0]  tail_n;         // bytes to read (min(name_len,4))
    reg [31:0] tail;           // name-tail accumulator
    reg        is_dir_r;
    reg        is_m3u_r;

    // EOCD signature "PK\005\006" (file order 50 4B 05 06); read backwards
    // the shift register ends up as 50 4B 05 06.
    localparam [31:0] EOCD_SIG = 32'h504B0506;
    // Central-directory header signature "PK\001\002" (file order 50 4B 01 02).
    localparam [31:0] CD_SIG   = 32'h02014B50;

    // Field-start byte indexes inside the 46-byte central header.
    wire hdr_field_start = (hdr_idx == 6'd10) || (hdr_idx == 6'd16) ||
                           (hdr_idx == 6'd20) || (hdr_idx == 6'd24) ||
                           (hdr_idx == 6'd28) || (hdr_idx == 6'd30) ||
                           (hdr_idx == 6'd32) || (hdr_idx == 6'd42);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= S_IDLE;
            rd_req <= 1'b0;
            rd_addr <= 26'd0;
            entry_valid <= 1'b0;
            entry_count <= 16'd0;
            done <= 1'b0;
            error <= 3'd0;
            rd_pending <= 1'b0;
            win <= 32'd0;
            acc <= 32'd0;
            count <= 16'd0;
            started <= 1'b0; eocd_found <= 1'b0; cd_started <= 1'b0;
        end else begin
            rd_req <= 1'b0;
            entry_valid <= 1'b0;

            case (state)
            S_IDLE: begin
                if (start) begin
                    len_r <= arch_len;
                    entry_count <= 16'd0;
                    count <= 16'd0;
                    done <= 1'b0;
                    error <= 3'd0;
                    win <= 32'd0;
                    rd_pending <= 1'b0;
                    started <= 1'b1; eocd_found <= 1'b0; cd_started <= 1'b0;
                    if (arch_len < 26'd22) begin
                        error <= 3'd4;
                        state <= S_ERROR;
                    end else begin
                        scan_pos <= arch_len - 26'd1;
                        scan_end <= (arch_len > 26'd65557) ?
                                    (arch_len - 26'd65557) : 26'd0;
                        state <= S_SCAN;
                    end
                end
            end

            // ---- EOCD backwards scan -------------------------------------
            S_SCAN: begin
                if (!rd_pending) begin
                    rd_req <= 1'b1;
                    rd_addr <= scan_pos;
                    rd_pending <= 1'b1;
                end else if (rd_valid) begin
                    rd_pending <= 1'b0;
                    win <= {rd_data, win[31:8]};
                    if ({rd_data, win[31:8]} == EOCD_SIG) begin
                        eocd_pos <= scan_pos;
                        eocd_idx <= 5'd4;
                        acc <= 32'd0;
                        eocd_found <= 1'b1;
                        state <= S_EOCD;
                    end else if (scan_pos == scan_end) begin
                        error <= 3'd1;
                        state <= S_ERROR;
                    end else begin
                        scan_pos <= scan_pos - 26'd1;
                    end
                end
            end

            // ---- Read EOCD fields (bytes 4..21) and validate -------------
            S_EOCD: begin
                if (!rd_pending) begin
                    rd_req <= 1'b1;
                    rd_addr <= eocd_pos + {21'd0, eocd_idx};
                    rd_pending <= 1'b1;
                end else if (rd_valid) begin
                    rd_pending <= 1'b0;
                    // Assemble fields with the LE shift trick.
                    if (eocd_idx == 5'd10 || eocd_idx == 5'd12 ||
                        eocd_idx == 5'd16 || eocd_idx == 5'd20)
                        acc <= {rd_data, 24'd0};
                    else
                        acc <= {rd_data, acc[31:8]};
                    // Latch each field on its last byte. The latch reads
                    // the OLD acc: a 16-bit field is {new byte, acc top
                    // byte}, a 32-bit field is {new byte, acc top 3 bytes}.
                    if (eocd_idx == 5'd11)
                        cd_entries <= {rd_data, acc[31:24]};
                    if (eocd_idx == 5'd15)
                        cd_size <= {rd_data, acc[31:8]};
                    if (eocd_idx == 5'd19)
                        // 32-bit field, stored truncated to 26 bits
                        // (valid: cd_offset < arch_len <= 2^26).
                        cd_offset <= {rd_data[1:0], acc[31:8]};
                    if (eocd_idx == 5'd21) begin
                        comment_len <= {rd_data, acc[31:24]};
                        // Validate with the freshly latched comment length:
                        // the record must end exactly at arch_len and the
                        // central directory must end at the EOCD.
                        if ((eocd_pos + 26'd22 + {10'd0, rd_data, acc[31:24]} == len_r) &&
                            (cd_offset + cd_size[25:0] == eocd_pos)) begin
                            cd_pos <= cd_offset;
                            count <= 16'd0;
                            if (cd_entries == 16'd0) begin
                                done <= 1'b1;
                                entry_count <= 16'd0;
                                state <= S_DONE;
                            end else begin
                                hdr_idx <= 6'd0;
                                acc <= 32'd0;
                                cd_started <= 1'b1;
                                state <= S_CD_HDR;
                            end
                        end else begin
                            // False positive (signature bytes inside file
                            // data): resume the scan below this candidate.
                            win <= 32'd0;
                            scan_pos <= eocd_pos - 26'd1;
                            state <= S_SCAN;
                        end
                    end else begin
                        eocd_idx <= eocd_idx + 5'd1;
                    end
                end
            end

            // ---- Read one 46-byte central-directory header ---------------
            S_CD_HDR: begin
                if (!rd_pending) begin
                    rd_req <= 1'b1;
                    rd_addr <= cd_pos + {20'd0, hdr_idx};
                    rd_pending <= 1'b1;
                end else if (rd_valid) begin
                    rd_pending <= 1'b0;
                    if (hdr_field_start)
                        acc <= {rd_data, 24'd0};
                    else
                        acc <= {rd_data, acc[31:8]};
                    if (hdr_idx == 6'd3 && {rd_data, acc[31:8]} != CD_SIG) begin
                        error <= 3'd2;
                        state <= S_ERROR;
                    end else begin
                        if (hdr_idx == 6'd11)
                            method_r <= {rd_data, acc[31:24]};
                        if (hdr_idx == 6'd19)
                            crc_r <= {rd_data, acc[31:8]};
                        if (hdr_idx == 6'd23)
                            comp_size_r <= {rd_data, acc[31:8]};
                        if (hdr_idx == 6'd27)
                            uncomp_size_r <= {rd_data, acc[31:8]};
                        if (hdr_idx == 6'd29)
                            name_len_r <= {rd_data, acc[31:24]};
                        if (hdr_idx == 6'd31)
                            extra_len_r <= {rd_data, acc[31:24]};
                        if (hdr_idx == 6'd33)
                            comment_len_r <= {rd_data, acc[31:24]};
                        if (hdr_idx == 6'd45)
                            local_hdr_off_r <= {rd_data, acc[31:8]};
                        if (hdr_idx == 6'd45) begin
                            name_ptr_r <= cd_pos + 26'd46;
                            tail_n <= (name_len_r < 16'd4) ?
                                      name_len_r[2:0] : 3'd4;
                            tail_idx <= 3'd0;
                            tail <= 32'd0;
                            state <= S_CD_TAIL;
                        end else begin
                            hdr_idx <= hdr_idx + 6'd1;
                        end
                    end
                end
            end

            // ---- Read the last (up to) 4 name bytes ----------------------
            // tail ends up {last, 2nd-last, 3rd-last, 4th-last}; is_dir
            // looks at the last byte, is_m3u at all four (case-insensitive).
            S_CD_TAIL: begin
                if (tail_idx == tail_n) begin
                    is_dir_r <= (tail_n != 3'd0) && (tail[31:24] == 8'h2F);
                    is_m3u_r <= (tail_n == 3'd4) &&
                                (tail[7:0] == 8'h2E) &&
                                ((tail[15:8] == 8'h6D) || (tail[15:8] == 8'h4D)) &&
                                (tail[23:16] == 8'h33) &&
                                ((tail[31:24] == 8'h75) || (tail[31:24] == 8'h55));
                    state <= S_CD_EMIT;
                end else if (!rd_pending) begin
                    rd_req <= 1'b1;
                    rd_addr <= name_ptr_r + {10'd0, name_len_r} -
                               {23'd0, tail_n} + {23'd0, tail_idx};
                    rd_pending <= 1'b1;
                end else if (rd_valid) begin
                    rd_pending <= 1'b0;
                    tail <= {rd_data, tail[31:8]};
                    tail_idx <= tail_idx + 3'd1;
                end
            end

            // ---- Emit the entry record ------------------------------------
            S_CD_EMIT: begin
                entry_valid <= 1'b1;
                entry_name_ptr <= name_ptr_r;
                entry_name_len <= name_len_r;
                entry_method <= method_r;
                entry_crc <= crc_r;
                entry_comp_size <= comp_size_r;
                entry_uncomp_size <= uncomp_size_r;
                entry_local_hdr_off <= local_hdr_off_r;
                entry_is_dir <= is_dir_r;
                entry_is_m3u <= is_m3u_r;
                entry_count <= count + 16'd1;
                count <= count + 16'd1;
                cd_pos <= cd_pos + 26'd46 + {10'd0, name_len_r} +
                          {10'd0, extra_len_r} + {10'd0, comment_len_r};
                if (count + 16'd1 == cd_entries) begin
                    done <= 1'b1;
                    state <= S_DONE;
                end else if (cd_pos + 26'd46 + {10'd0, name_len_r} +
                             {10'd0, extra_len_r} + {10'd0, comment_len_r} >
                             cd_offset + cd_size[25:0]) begin
                    error <= 3'd3;
                    state <= S_ERROR;
                end else begin
                    hdr_idx <= 6'd0;
                    acc <= 32'd0;
                    state <= S_CD_HDR;
                end
            end

            S_DONE: begin
                if (start) begin
                    // Restart (same handling as S_IDLE).
                    len_r <= arch_len;
                    entry_count <= 16'd0;
                    count <= 16'd0;
                    done <= 1'b0;
                    error <= 3'd0;
                    win <= 32'd0;
                    rd_pending <= 1'b0;
                    if (arch_len < 26'd22) begin
                        error <= 3'd4;
                        state <= S_ERROR;
                    end else begin
                        scan_pos <= arch_len - 26'd1;
                        scan_end <= (arch_len > 26'd65557) ?
                                    (arch_len - 26'd65557) : 26'd0;
                        state <= S_SCAN;
                    end
                end
            end

            S_ERROR: begin
                if (start) begin
                    len_r <= arch_len;
                    entry_count <= 16'd0;
                    count <= 16'd0;
                    done <= 1'b0;
                    error <= 3'd0;
                    win <= 32'd0;
                    rd_pending <= 1'b0;
                    if (arch_len < 26'd22) begin
                        error <= 3'd4;
                        state <= S_ERROR;
                    end else begin
                        scan_pos <= arch_len - 26'd1;
                        scan_end <= (arch_len > 26'd65557) ?
                                    (arch_len - 26'd65557) : 26'd0;
                        state <= S_SCAN;
                    end
                end
            end

            default: state <= S_IDLE;
            endcase
        end
    end

endmodule
