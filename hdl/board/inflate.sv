// SPDX-License-Identifier: GPL-2.0-or-later
// inflate.sv — serial DEFLATE decoder (RFC 1951) for the SC-55 MIDI player
// archive browser.
//
// Hand-written, streaming: compressed bytes in (ready/valid), decompressed
// bytes out (valid/ready, held until accepted). Supports stored, fixed and
// dynamic Huffman blocks, a 32 KiB sliding window in block RAM, and a CRC-32
// of the output for the caller to compare against the ZIP central directory.
//
// The decoder consumes at most comp_len input bytes and produces exactly
// uncomp_len output bytes on success; any mismatch, bad table, bad symbol
// or truncated input raises error and asserts done. Deflate64 and ZIP64 are
// not supported (the caller refuses those methods before starting a decode).
//
// Throughput is modest by design (a few clocks per output byte) — a 200 KiB
// song inflates in tens of milliseconds at 50 MHz, far faster than the
// DDR3 write path that follows it.

module inflate (
    input  wire        clk,
    input  wire        rst,
    input  wire        start,          // pulse to start a decode
    input  wire [31:0] comp_len,       // compressed input length (bytes)
    input  wire [31:0] uncomp_len,     // expected output length (bytes)
    // Compressed input byte stream (ready/valid; consumed on o_ready&&i_valid)
    input  wire [7:0]  i_data,
    input  wire        i_valid,
    output wire        o_ready,
    // Decompressed output byte stream (valid/ready; held until accepted)
    output reg  [7:0]  o_data,
    output reg         o_valid,
    input  wire        i_ready,
    output reg         done,
    output reg  [2:0]  error,          // 0 none, 1 bad block type,
                                        // 2 bad Huffman table, 3 bad symbol
                                        // or distance, 4 output length
                                        // mismatch, 5 truncated input
    output reg  [31:0] o_crc,          // CRC-32 of the output (at done)
    output reg  [31:0] o_out_count     // bytes produced (at done)
);

    // ---------------- states ----------------
    localparam [5:0]
        S_IDLE        = 6'd0,
        S_HDR_F       = 6'd1,   // BFINAL (1 bit)
        S_HDR_T       = 6'd2,   // BTYPE (2 bits)
        S_ST_ALIGN    = 6'd3,   // stored: align to byte boundary
        S_ST_LEN1     = 6'd4,   // stored: LEN
        S_ST_LEN2     = 6'd5,   // stored: NLEN
        S_ST_COPY     = 6'd6,   // stored: copy LEN bytes
        S_ST_COPY_RET = 6'd7,
        S_FIXED_LIT   = 6'd8,   // build fixed literal table
        S_FIXED_DIST  = 6'd9,   // build fixed distance table
        S_DYN_HDR1    = 6'd10,  // HLIT
        S_DYN_HDR2    = 6'd11,  // HDIST
        S_DYN_HDR3    = 6'd12,  // HCLEN
        S_DYN_CL      = 6'd13,  // read code-length code lengths
        S_DYN_CL_BLD  = 6'd14,  // build code-length table
        S_DYN_LENS    = 6'd15,  // decode literal/distance lengths
        S_DYN_LENS_GOT= 6'd16,
        S_DYN_R16     = 6'd17,  // repeat previous length
        S_DYN_R17     = 6'd18,  // repeat zero (3-10)
        S_DYN_R18     = 6'd19,  // repeat zero (11-138)
        S_DYN_FILL    = 6'd20,  // fill repeated lengths
        S_DYN_BLD_LIT = 6'd21,  // build literal table
        S_DYN_BLD_DST = 6'd22,  // build distance table
        S_DATA        = 6'd23,  // decode literal/length symbol
        S_DATA_GOT    = 6'd24,
        S_LEXT        = 6'd25,  // length extra bits
        S_DSYM        = 6'd26,  // decode distance symbol
        S_DSYM_GOT    = 6'd27,
        S_DEXT        = 6'd28,  // distance extra bits
        S_COPY_READ   = 6'd29,  // LZ77 copy: issue window read
        S_COPY_WAIT   = 6'd30,  // LZ77 copy: wait for window data
        S_COPY_EMIT   = 6'd31,  // LZ77 copy: emit window byte
        S_COPY_NEXT   = 6'd32,
        S_BLK_END     = 6'd33,
        S_FINISH      = 6'd34,
        S_ERR         = 6'd35,
        S_B_INIT      = 6'd36,  // table build: zero counts
        S_B_COUNT     = 6'd37,  // table build: count code lengths
        S_B_FIRST     = 6'd38,  // table build: first codes + offsets
        S_B_SYM       = 6'd39,  // table build: assign symbols
        S_B_COPY      = 6'd40,  // table build: copy working set to table
        S_H_INIT      = 6'd41,  // Huffman decode: init
        S_H_BIT       = 6'd42,  // Huffman decode: take one bit
        S_H_SYM       = 6'd43,  // Huffman decode: symbol index
        S_H_SYM_RD    = 6'd44,  // Huffman decode: read symbol
        S_E_PUT       = 6'd45;  // emit one output byte

    // Table selectors for the build / Huffman sub-FSMs.
    localparam [1:0] T_LIT = 2'd0, T_DIST = 2'd1, T_CL = 2'd2;

    reg [5:0] state, ret_state;

    // Stream parameters and progress.
    reg [31:0] comp_len_r, uncomp_len_r;
    reg        bfinal_r;
    reg [31:0] out_count_r;
    reg [31:0] crc_r;
    reg [14:0] wptr;

    // ---------------- bit reader ----------------
    // LSB-first bit buffer. Refills from the input stream whenever there is
    // room (bitcnt <= 24) and input remains; takes are combinational when
    // enough bits are buffered.
    reg [31:0] bitbuf;
    reg [5:0]  bitcnt;
    reg [31:0] in_left;

    assign o_ready = (state != S_IDLE) && (bitcnt <= 6'd24) && (in_left != 32'd0);

    // Take interface (combinational; the main FSM asserts take_req in the
    // states that need bits and latches take_data when take_ack is high).
    reg        take_req_c;
    reg [4:0]  take_n_c;
    reg [15:0] stored_len;
    reg [8:0]  hlit_r, hdist_r;
    reg [4:0]  hclen_r;
    reg [4:0]  dyn_i;
    reg [8:0]  dyn_j;
    reg [8:0]  len_base_r;
    reg [3:0]  len_ext_n;
    reg [8:0]  len_r;
    reg [15:0] dist_base_r;
    reg [3:0]  dist_ext_n;
    reg [15:0] dist_r;
    reg [8:0]  copy_idx;

    wire take_req = take_req_c;
    wire [4:0] take_n = take_n_c;
    wire take_ack = (bitcnt >= {1'b0, take_n});
    wire [15:0] take_data = bitbuf[15:0];
    wire take_stuck = take_req && !take_ack && (in_left == 32'd0);

    always @* begin
        take_req_c = 1'b0;
        take_n_c = 5'd0;
        case (state)
            S_HDR_F: begin take_req_c = 1'b1; take_n_c = 5'd1; end
            S_HDR_T: begin take_req_c = 1'b1; take_n_c = 5'd2; end
            S_ST_ALIGN: if (bitcnt[2:0] != 3'd0) begin
                             take_req_c = 1'b1;
                             take_n_c = {2'd0, bitcnt[2:0]};
                         end
            S_ST_LEN1, S_ST_LEN2: begin take_req_c = 1'b1; take_n_c = 5'd16; end
            S_ST_COPY: begin take_req_c = 1'b1; take_n_c = 5'd8; end
            S_DYN_HDR1, S_DYN_HDR2: begin take_req_c = 1'b1; take_n_c = 5'd5; end
            S_DYN_HDR3: begin take_req_c = 1'b1; take_n_c = 5'd4; end
            S_DYN_CL: begin take_req_c = 1'b1; take_n_c = 5'd3; end
            S_DYN_R16: begin take_req_c = 1'b1; take_n_c = 5'd2; end
            S_DYN_R17: begin take_req_c = 1'b1; take_n_c = 5'd3; end
            S_DYN_R18: begin take_req_c = 1'b1; take_n_c = 5'd7; end
            S_H_BIT: begin take_req_c = 1'b1; take_n_c = 5'd1; end
            S_LEXT: if (len_ext_n != 4'd0) begin
                        take_req_c = 1'b1;
                        take_n_c = {1'b0, len_ext_n};
                    end
            S_DEXT: if (dist_ext_n != 4'd0) begin
                        take_req_c = 1'b1;
                        take_n_c = {1'b0, dist_ext_n};
                    end
            default: begin end
        endcase
    end

    // Bit-reader state: refill and take may coincide in one clock; the new
    // byte is shifted in at the old bitcnt and the take shifts the result.
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            bitbuf <= 32'd0;
            bitcnt <= 6'd0;
            in_left <= 32'd0;
        end else if ((state == S_IDLE) && start) begin
            bitbuf <= 32'd0;
            bitcnt <= 6'd0;
            in_left <= comp_len;
        end else begin
            if ((o_ready && i_valid) && (take_req && take_ack)) begin
                bitbuf <= ((bitbuf | ({24'd0, i_data} << bitcnt)) >> take_n);
                bitcnt <= bitcnt + 6'd8 - {1'b0, take_n};
                in_left <= in_left - 32'd1;
            end else if (o_ready && i_valid) begin
                bitbuf <= bitbuf | ({24'd0, i_data} << bitcnt);
                bitcnt <= bitcnt + 6'd8;
                in_left <= in_left - 32'd1;
            end else if (take_req && take_ack) begin
                bitbuf <= bitbuf >> take_n;
                bitcnt <= bitcnt - {1'b0, take_n};
            end
        end
    end

    // ---------------- CRC-32 (IEEE, reflected) ----------------
    function [31:0] crc32_byte(input [31:0] crc, input [7:0] b);
        integer i;
        reg [31:0] c;
        begin
            c = crc;
            for (i = 0; i < 8; i = i + 1) begin
                if (c[0] ^ b[i]) c = (c >> 1) ^ 32'hEDB88320;
                else c = c >> 1;
            end
            crc32_byte = c;
        end
    endfunction

    // ---------------- length / distance tables (RFC 1951) ----------------
    function [8:0] len_base(input [8:0] sym);
        case (sym)
            9'd257: len_base = 9'd3;   9'd258: len_base = 9'd4;
            9'd259: len_base = 9'd5;   9'd260: len_base = 9'd6;
            9'd261: len_base = 9'd7;   9'd262: len_base = 9'd8;
            9'd263: len_base = 9'd9;   9'd264: len_base = 9'd10;
            9'd265: len_base = 9'd11;  9'd266: len_base = 9'd13;
            9'd267: len_base = 9'd15;  9'd268: len_base = 9'd17;
            9'd269: len_base = 9'd19;  9'd270: len_base = 9'd23;
            9'd271: len_base = 9'd27;  9'd272: len_base = 9'd31;
            9'd273: len_base = 9'd35;  9'd274: len_base = 9'd43;
            9'd275: len_base = 9'd51;  9'd276: len_base = 9'd59;
            9'd277: len_base = 9'd67;  9'd278: len_base = 9'd83;
            9'd279: len_base = 9'd99;  9'd280: len_base = 9'd115;
            9'd281: len_base = 9'd131; 9'd282: len_base = 9'd163;
            9'd283: len_base = 9'd195; 9'd284: len_base = 9'd227;
            9'd285: len_base = 9'd258;
            default: len_base = 9'd0;
        endcase
    endfunction

    function [3:0] len_extra(input [8:0] sym);
        case (sym)
            9'd257, 9'd258, 9'd259, 9'd260,
            9'd261, 9'd262, 9'd263, 9'd264: len_extra = 4'd0;
            9'd265, 9'd266, 9'd267, 9'd268: len_extra = 4'd1;
            9'd269, 9'd270, 9'd271, 9'd272: len_extra = 4'd2;
            9'd273, 9'd274, 9'd275, 9'd276: len_extra = 4'd3;
            9'd277, 9'd278, 9'd279, 9'd280: len_extra = 4'd4;
            9'd281, 9'd282, 9'd283, 9'd284: len_extra = 4'd5;
            default: len_extra = 4'd0;
        endcase
    endfunction

    function [15:0] dist_base(input [4:0] sym);
        case (sym)
            5'd0: dist_base = 16'd1;    5'd1: dist_base = 16'd2;
            5'd2: dist_base = 16'd3;    5'd3: dist_base = 16'd4;
            5'd4: dist_base = 16'd5;    5'd5: dist_base = 16'd7;
            5'd6: dist_base = 16'd9;    5'd7: dist_base = 16'd13;
            5'd8: dist_base = 16'd17;   5'd9: dist_base = 16'd25;
            5'd10: dist_base = 16'd33;  5'd11: dist_base = 16'd49;
            5'd12: dist_base = 16'd65;  5'd13: dist_base = 16'd97;
            5'd14: dist_base = 16'd129; 5'd15: dist_base = 16'd193;
            5'd16: dist_base = 16'd257; 5'd17: dist_base = 16'd385;
            5'd18: dist_base = 16'd513; 5'd19: dist_base = 16'd769;
            5'd20: dist_base = 16'd1025; 5'd21: dist_base = 16'd1537;
            5'd22: dist_base = 16'd2049; 5'd23: dist_base = 16'd3073;
            5'd24: dist_base = 16'd4097; 5'd25: dist_base = 16'd6145;
            5'd26: dist_base = 16'd8193; 5'd27: dist_base = 16'd12289;
            5'd28: dist_base = 16'd16385; 5'd29: dist_base = 16'd24577;
            default: dist_base = 16'd0;
        endcase
    endfunction

    function [3:0] dist_extra(input [4:0] sym);
        case (sym)
            5'd0, 5'd1, 5'd2, 5'd3: dist_extra = 4'd0;
            5'd4, 5'd5: dist_extra = 4'd1;
            5'd6, 5'd7: dist_extra = 4'd2;
            5'd8, 5'd9: dist_extra = 4'd3;
            5'd10, 5'd11: dist_extra = 4'd4;
            5'd12, 5'd13: dist_extra = 4'd5;
            5'd14, 5'd15: dist_extra = 4'd6;
            5'd16, 5'd17: dist_extra = 4'd7;
            5'd18, 5'd19: dist_extra = 4'd8;
            5'd20, 5'd21: dist_extra = 4'd9;
            5'd22, 5'd23: dist_extra = 4'd10;
            5'd24, 5'd25: dist_extra = 4'd11;
            5'd26, 5'd27: dist_extra = 4'd12;
            5'd28, 5'd29: dist_extra = 4'd13;
            default: dist_extra = 4'd0;
        endcase
    endfunction

    function [3:0] fixed_lit_len(input [8:0] i);
        if (i < 9'd144) fixed_lit_len = 4'd8;
        else if (i < 9'd256) fixed_lit_len = 4'd9;
        else if (i < 9'd280) fixed_lit_len = 4'd7;
        else fixed_lit_len = 4'd8;
    endfunction

    // Code-length code order (RFC 1951 section 3.2.7).
    function [4:0] cl_order(input [4:0] i);
        case (i)
            5'd0: cl_order = 5'd16; 5'd1: cl_order = 5'd17;
            5'd2: cl_order = 5'd18; 5'd3: cl_order = 5'd0;
            5'd4: cl_order = 5'd8;  5'd5: cl_order = 5'd7;
            5'd6: cl_order = 5'd9;  5'd7: cl_order = 5'd6;
            5'd8: cl_order = 5'd10; 5'd9: cl_order = 5'd5;
            5'd10: cl_order = 5'd11; 5'd11: cl_order = 5'd4;
            5'd12: cl_order = 5'd12; 5'd13: cl_order = 5'd3;
            5'd14: cl_order = 5'd13; 5'd15: cl_order = 5'd2;
            5'd16: cl_order = 5'd14; 5'd17: cl_order = 5'd1;
            5'd18: cl_order = 5'd15;
            default: cl_order = 5'd0;
        endcase
    endfunction

    // ---------------- Huffman tables ----------------
    // Persistent tables (lit + dist live through the data phase; cl lives
    // only while decoding dynamic lengths).
    reg [8:0]  lit_cnt  [1:15];
    reg [14:0] lit_first[1:15];
    reg [8:0]  lit_offs [1:15];
    reg [8:0]  lit_sym  [0:287];
    reg [8:0]  dist_cnt  [1:15];
    reg [14:0] dist_first[1:15];
    reg [8:0]  dist_offs [1:15];
    reg [4:0]  dist_sym  [0:31];
    reg [8:0]  cl_cnt  [1:15];
    reg [14:0] cl_first[1:15];
    reg [8:0]  cl_offs [1:15];
    reg [4:0]  cl_sym  [0:18];

    // Decoded code lengths (dynamic blocks).
    reg [3:0] lens   [0:319];   // HLIT literal + HDIST distance lengths
    reg [3:0] cl_lens[0:18];    // code-length code lengths

    // Table-build working set (shared; copied to the target table at the end).
    reg [1:0]  b_target;
    reg        b_fixed;
    reg [8:0]  b_n;              // number of lengths to build from
    reg [8:0]  b_base;           // lens[] base (HLIT for the distance table)
    reg [8:0]  b_i;
    reg [8:0]  b_total;          // symbols with a nonzero length
    reg [8:0]  w_cnt [1:15];
    reg [14:0] w_first[1:15];
    reg [8:0]  w_offs [1:15];
    reg [8:0]  w_ptr  [1:15];
    reg [3:0]  b_l;
    reg [16:0] b_left;
    reg [16:0] b_code;
    reg [8:0]  b_offs_acc;
    reg [4:0]  b_k;              // copy index (1..15)

    // Length source for the table build.
    function [3:0] b_len(input [8:0] i);
        if (b_fixed) begin
            if (b_target == T_LIT) b_len = fixed_lit_len(i);
            else b_len = 4'd5;              // fixed distance: all 5 bits
        end else if (b_target == T_CL) begin
            b_len = cl_lens[i[4:0]];
        end else begin
            b_len = lens[b_base + i];
        end
    endfunction

    // Huffman decode state.
    reg [1:0]  h_table;
    reg [14:0] h_code;
    reg [4:0]  h_len;
    reg [8:0]  h_sym;
    reg [8:0]  h_sym_idx;
    reg [7:0]  emit_byte;
    reg [7:0]  fill_cnt;
    reg [3:0]  fill_val;

    // Huffman table lookups for the code length being checked (chk) and the
    // length just completed (cur). Indexed by chk_len / h_len.
    reg [4:0] chk_len;
    reg [8:0] chk_cnt;
    reg [14:0] chk_first;
    reg [8:0] chk_offs;
    reg [8:0] cur_cnt;
    reg [14:0] cur_first;
    reg [8:0] cur_offs;

    always @* begin
        chk_len = h_len + 5'd1;
        case (h_table)
            T_LIT: begin
                chk_cnt = lit_cnt[chk_len]; chk_first = lit_first[chk_len];
                chk_offs = lit_offs[chk_len];
                cur_cnt = lit_cnt[h_len]; cur_first = lit_first[h_len];
                cur_offs = lit_offs[h_len];
            end
            T_DIST: begin
                chk_cnt = dist_cnt[chk_len]; chk_first = dist_first[chk_len];
                chk_offs = dist_offs[chk_len];
                cur_cnt = dist_cnt[h_len]; cur_first = dist_first[h_len];
                cur_offs = dist_offs[h_len];
            end
            default: begin
                chk_cnt = cl_cnt[chk_len]; chk_first = cl_first[chk_len];
                chk_offs = cl_offs[chk_len];
                cur_cnt = cl_cnt[h_len]; cur_first = cl_first[h_len];
                cur_offs = cl_offs[h_len];
            end
        endcase
    end

    // ---------------- 32 KiB sliding window ----------------
    reg        win_we;
    reg [14:0] win_wa;
    reg [7:0]  win_wd;
    reg [14:0] win_ra;
    wire [7:0] win_rd;

    sdp_ram #(.AW(15), .DW(8)) win_ram (
        .clk(clk), .we(win_we), .wa(win_wa), .wd(win_wd),
        .ra(win_ra), .rd(win_rd)
    );

    // Extra-bit field values (low take_n bits of the bit buffer, masked).
    wire [4:0]  len_ext_val  = take_data[4:0] &
                               ((5'd1 << len_ext_n) - 5'd1);
    wire [12:0] dist_ext_val = take_data[12:0] &
                               ((13'd1 << dist_ext_n) - 13'd1);
    // Next Huffman code / length after taking one more bit.
    wire [14:0] h_ncode = {h_code[13:0], take_data[0]};
    wire [4:0]  h_nlen  = h_len + 5'd1;

    // ---------------- main FSM ----------------
    integer k;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= S_IDLE;
            ret_state <= S_IDLE;
            o_data <= 8'd0;
            o_valid <= 1'b0;
            done <= 1'b0;
            error <= 3'd0;
            o_crc <= 32'd0;
            o_out_count <= 32'd0;
            win_we <= 1'b0;
            win_wa <= 15'd0;
            win_wd <= 8'd0;
            win_ra <= 15'd0;
        end else begin
            win_we <= 1'b0;
            if (take_stuck) begin
                // The stream needs more bits than remain in the input.
                error <= 3'd5;
                done <= 1'b1;
                o_crc <= ~crc_r;
                o_out_count <= out_count_r;
                state <= S_ERR;
            end else begin
            case (state)
            S_IDLE: begin
                if (start) begin
                    comp_len_r <= comp_len;
                    uncomp_len_r <= uncomp_len;
                    out_count_r <= 32'd0;
                    crc_r <= 32'hFFFFFFFF;
                    wptr <= 15'd0;
                    done <= 1'b0;
                    error <= 3'd0;
                    o_valid <= 1'b0;
                    state <= S_HDR_F;
                end
            end

            S_HDR_F: if (take_ack) begin
                bfinal_r <= take_data[0];
                state <= S_HDR_T;
            end

            S_HDR_T: if (take_ack) begin
                if (take_data[1:0] == 2'd0) begin
                    state <= S_ST_ALIGN;
                end else if (take_data[1:0] == 2'd1) begin
                    state <= S_FIXED_LIT;
                end else if (take_data[1:0] == 2'd2) begin
                    state <= S_DYN_HDR1;
                end else begin
                    error <= 3'd1;
                    done <= 1'b1;
                    o_crc <= ~crc_r;
                    o_out_count <= out_count_r;
                    state <= S_ERR;
                end
            end

            S_ST_ALIGN: begin
                if (bitcnt[2:0] == 3'd0) state <= S_ST_LEN1;
                else if (take_ack) state <= S_ST_LEN1;
            end

            S_ST_LEN1: if (take_ack) begin
                stored_len <= take_data[15:0];
                state <= S_ST_LEN2;
            end

            S_ST_LEN2: if (take_ack) begin
                if (take_data[15:0] != ~stored_len) begin
                    error <= 3'd5;
                    done <= 1'b1;
                    o_crc <= ~crc_r;
                    o_out_count <= out_count_r;
                    state <= S_ERR;
                end else if (stored_len == 16'd0) begin
                    state <= S_BLK_END;
                end else begin
                    state <= S_ST_COPY;
                end
            end

            S_ST_COPY: if (take_ack) begin
                emit_byte <= take_data[7:0];
                stored_len <= stored_len - 16'd1;
                ret_state <= S_ST_COPY_RET;
                state <= S_E_PUT;
            end

            S_ST_COPY_RET: begin
                if (stored_len == 16'd0) state <= S_BLK_END;
                else state <= S_ST_COPY;
            end

            S_FIXED_LIT: begin
                b_target <= T_LIT; b_fixed <= 1'b1;
                b_n <= 9'd288; b_base <= 9'd0;
                ret_state <= S_FIXED_DIST;
                state <= S_B_INIT;
            end

            S_FIXED_DIST: begin
                b_target <= T_DIST; b_fixed <= 1'b1;
                b_n <= 9'd32; b_base <= 9'd0;
                ret_state <= S_DATA;
                state <= S_B_INIT;
            end

            S_DYN_HDR1: if (take_ack) begin
                hlit_r <= {4'd0, take_data[4:0]} + 9'd257;
                state <= S_DYN_HDR2;
            end

            S_DYN_HDR2: if (take_ack) begin
                hdist_r <= {4'd0, take_data[4:0]} + 9'd1;
                state <= S_DYN_HDR3;
            end

            S_DYN_HDR3: if (take_ack) begin
                hclen_r <= take_data[3:0] + 5'd4;
                for (k = 0; k < 19; k = k + 1) cl_lens[k] <= 4'd0;
                dyn_i <= 5'd0;
                state <= S_DYN_CL;
            end

            S_DYN_CL: if (take_ack) begin
                cl_lens[cl_order(dyn_i)] <= {1'b0, take_data[2:0]};
                if (dyn_i == hclen_r - 5'd1) begin
                    dyn_j <= 9'd0;
                    state <= S_DYN_CL_BLD;
                end else begin
                    dyn_i <= dyn_i + 5'd1;
                end
            end

            S_DYN_CL_BLD: begin
                b_target <= T_CL; b_fixed <= 1'b0;
                b_n <= 9'd19; b_base <= 9'd0;
                ret_state <= S_DYN_LENS;
                state <= S_B_INIT;
            end

            S_DYN_LENS: begin
                h_table <= T_CL;
                ret_state <= S_DYN_LENS_GOT;
                state <= S_H_INIT;
            end

            S_DYN_LENS_GOT: begin
                if (h_sym < 9'd16) begin
                    lens[dyn_j] <= h_sym[3:0];
                    if (dyn_j == hlit_r + hdist_r - 9'd1) begin
                        state <= S_DYN_BLD_LIT;
                    end else begin
                        dyn_j <= dyn_j + 9'd1;
                        state <= S_DYN_LENS;
                    end
                end else if (h_sym == 9'd16) begin
                    if (dyn_j == 9'd0) begin
                        error <= 3'd2;
                        done <= 1'b1;
                        o_crc <= ~crc_r;
                        o_out_count <= out_count_r;
                        state <= S_ERR;
                    end else begin
                        fill_val <= lens[dyn_j - 9'd1];
                        state <= S_DYN_R16;
                    end
                end else if (h_sym == 9'd17) begin
                    fill_val <= 4'd0;
                    state <= S_DYN_R17;
                end else if (h_sym == 9'd18) begin
                    fill_val <= 4'd0;
                    state <= S_DYN_R18;
                end else begin
                    error <= 3'd2;
                    done <= 1'b1;
                    o_crc <= ~crc_r;
                    o_out_count <= out_count_r;
                    state <= S_ERR;
                end
            end

            S_DYN_R16: if (take_ack) begin
                fill_cnt <= {6'd0, take_data[1:0]} + 8'd3;
                if (dyn_j + {1'b0, ({6'd0, take_data[1:0]} + 8'd3)} >
                    hlit_r + hdist_r) begin
                    error <= 3'd2;
                    done <= 1'b1;
                    o_crc <= ~crc_r;
                    o_out_count <= out_count_r;
                    state <= S_ERR;
                end else begin
                    state <= S_DYN_FILL;
                end
            end

            S_DYN_R17: if (take_ack) begin
                fill_cnt <= {5'd0, take_data[2:0]} + 8'd3;
                if (dyn_j + {1'b0, ({5'd0, take_data[2:0]} + 8'd3)} >
                    hlit_r + hdist_r) begin
                    error <= 3'd2;
                    done <= 1'b1;
                    o_crc <= ~crc_r;
                    o_out_count <= out_count_r;
                    state <= S_ERR;
                end else begin
                    state <= S_DYN_FILL;
                end
            end

            S_DYN_R18: if (take_ack) begin
                fill_cnt <= {1'd0, take_data[6:0]} + 8'd11;
                if (dyn_j + {1'b0, ({1'd0, take_data[6:0]} + 8'd11)} >
                    hlit_r + hdist_r) begin
                    error <= 3'd2;
                    done <= 1'b1;
                    o_crc <= ~crc_r;
                    o_out_count <= out_count_r;
                    state <= S_ERR;
                end else begin
                    state <= S_DYN_FILL;
                end
            end

            S_DYN_FILL: begin
                lens[dyn_j] <= fill_val;
                dyn_j <= dyn_j + 9'd1;
                if (fill_cnt == 8'd1) begin
                    if (dyn_j + 9'd1 == hlit_r + hdist_r) begin
                        state <= S_DYN_BLD_LIT;
                    end else begin
                        state <= S_DYN_LENS;
                    end
                end else begin
                    fill_cnt <= fill_cnt - 8'd1;
                end
            end

            S_DYN_BLD_LIT: begin
                b_target <= T_LIT; b_fixed <= 1'b0;
                b_n <= hlit_r; b_base <= 9'd0;
                ret_state <= S_DYN_BLD_DST;
                state <= S_B_INIT;
            end

            S_DYN_BLD_DST: begin
                b_target <= T_DIST; b_fixed <= 1'b0;
                b_n <= hdist_r; b_base <= hlit_r;
                ret_state <= S_DATA;
                state <= S_B_INIT;
            end

            S_DATA: begin
                h_table <= T_LIT;
                ret_state <= S_DATA_GOT;
                state <= S_H_INIT;
            end

            S_DATA_GOT: begin
                if (h_sym < 9'd256) begin
                    emit_byte <= h_sym[7:0];
                    ret_state <= S_DATA;
                    state <= S_E_PUT;
                end else if (h_sym == 9'd256) begin
                    state <= S_BLK_END;
                end else if (h_sym <= 9'd285) begin
                    len_base_r <= len_base(h_sym);
                    len_ext_n <= len_extra(h_sym);
                    state <= S_LEXT;
                end else begin
                    error <= 3'd3;
                    done <= 1'b1;
                    o_crc <= ~crc_r;
                    o_out_count <= out_count_r;
                    state <= S_ERR;
                end
            end

            S_LEXT: begin
                if (len_ext_n == 4'd0) begin
                    len_r <= len_base_r;
                    state <= S_DSYM;
                end else if (take_ack) begin
                    len_r <= len_base_r + {4'd0, len_ext_val};
                    state <= S_DSYM;
                end
            end

            S_DSYM: begin
                h_table <= T_DIST;
                ret_state <= S_DSYM_GOT;
                state <= S_H_INIT;
            end

            S_DSYM_GOT: begin
                if (h_sym > 9'd29) begin
                    error <= 3'd3;
                    done <= 1'b1;
                    o_crc <= ~crc_r;
                    o_out_count <= out_count_r;
                    state <= S_ERR;
                end else begin
                    dist_base_r <= dist_base(h_sym[4:0]);
                    dist_ext_n <= dist_extra(h_sym[4:0]);
                    state <= S_DEXT;
                end
            end

            S_DEXT: begin
                if (dist_ext_n == 4'd0) begin
                    dist_r <= dist_base_r;
                    copy_idx <= 9'd0;
                    state <= S_COPY_READ;
                end else if (take_ack) begin
                    dist_r <= dist_base_r + {3'd0, dist_ext_val};
                    copy_idx <= 9'd0;
                    state <= S_COPY_READ;
                end
            end

            S_COPY_READ: begin
                if ((dist_r == 16'd0) ||
                    ({16'd0, dist_r} > out_count_r)) begin
                    error <= 3'd3;
                    done <= 1'b1;
                    o_crc <= ~crc_r;
                    o_out_count <= out_count_r;
                    state <= S_ERR;
                end else begin
                    // copy_idx is NOT reset here: it counts the bytes
                    // emitted so far in this match (reset once in S_DEXT).
                    win_ra <= wptr - dist_r[14:0];
                    state <= S_COPY_WAIT;
                end
            end

            S_COPY_WAIT: state <= S_COPY_EMIT;

            S_COPY_EMIT: begin
                emit_byte <= win_rd;
                copy_idx <= copy_idx + 9'd1;
                ret_state <= S_COPY_NEXT;
                state <= S_E_PUT;
            end

            S_COPY_NEXT: begin
                // The match is one symbol inside the block: decode the
                // next symbol (literal, match, or end-of-block).
                if (copy_idx == len_r) state <= S_DATA;
                else state <= S_COPY_READ;
            end

            S_BLK_END: begin
                if (bfinal_r) state <= S_FINISH;
                else state <= S_HDR_F;
            end

            S_FINISH: begin
                if (out_count_r == uncomp_len_r) begin
                    error <= 3'd0;
                end else begin
                    error <= 3'd4;
                end
                done <= 1'b1;
                o_crc <= ~crc_r;
                o_out_count <= out_count_r;
                state <= S_IDLE;
            end

            S_ERR: state <= S_IDLE;

            // ---- table build sub-FSM ----
            S_B_INIT: begin
                if (b_n == 9'd0) begin
                    error <= 3'd2;
                    done <= 1'b1;
                    o_crc <= ~crc_r;
                    o_out_count <= out_count_r;
                    state <= S_ERR;
                end else begin
                    for (k = 1; k < 16; k = k + 1) w_cnt[k] <= 9'd0;
                    b_i <= 9'd0;
                    b_total <= 9'd0;
                    state <= S_B_COUNT;
                end
            end

            S_B_COUNT: begin
                if (b_len(b_i) != 4'd0) begin
                    w_cnt[b_len(b_i)] <= w_cnt[b_len(b_i)] + 9'd1;
                    b_total <= b_total + 9'd1;
                end
                if (b_i == b_n - 9'd1) begin
                    b_i <= 9'd1;
                    b_left <= 17'd1;
                    b_code <= 17'd0;
                    b_offs_acc <= 9'd0;
                    state <= S_B_FIRST;
                end else begin
                    b_i <= b_i + 9'd1;
                end
            end

            S_B_FIRST: begin
                w_first[b_i[3:0]] <= b_code[14:0];
                w_offs[b_i[3:0]] <= b_offs_acc;
                w_ptr[b_i[3:0]] <= b_offs_acc;
                if ((b_left << 1) < {8'd0, w_cnt[b_i[3:0]]}) begin
                    // Oversubscribed: more codes than this length allows.
                    error <= 3'd2;
                    done <= 1'b1;
                    o_crc <= ~crc_r;
                    o_out_count <= out_count_r;
                    state <= S_ERR;
                end else if (b_i == 9'd15) begin
                    // Incomplete is an error, except for distance and
                    // code-length tables, and single-symbol tables.
                    if ((((b_left << 1) - {8'd0, w_cnt[4'd15]}) != 17'd0) &&
                        (b_target != T_DIST) && (b_target != T_CL) &&
                        (b_total != 9'd1)) begin
                        error <= 3'd2;
                        done <= 1'b1;
                        o_crc <= ~crc_r;
                        o_out_count <= out_count_r;
                        state <= S_ERR;
                    end else begin
                        b_i <= 9'd0;
                        state <= S_B_SYM;
                    end
                end else begin
                    b_left <= (b_left << 1) - {8'd0, w_cnt[b_i[3:0]]};
                    b_code <= (b_code + {8'd0, w_cnt[b_i[3:0]]}) << 1;
                    b_offs_acc <= b_offs_acc + w_cnt[b_i[3:0]];
                    b_i <= b_i + 9'd1;
                end
            end

            S_B_SYM: begin
                if (b_len(b_i) != 4'd0) begin
                    case (b_target)
                        T_LIT: lit_sym[w_ptr[b_len(b_i)]] <= b_i[8:0];
                        T_DIST: dist_sym[w_ptr[b_len(b_i)][4:0]] <= b_i[4:0];
                        default: cl_sym[w_ptr[b_len(b_i)][4:0]] <= b_i[4:0];
                    endcase
                    w_ptr[b_len(b_i)] <= w_ptr[b_len(b_i)] + 9'd1;
                end
                if (b_i == b_n - 9'd1) begin
                    state <= S_B_COPY;
                end else begin
                    b_i <= b_i + 9'd1;
                end
            end

            S_B_COPY: begin
                for (k = 1; k < 16; k = k + 1) begin
                    case (b_target)
                        T_LIT: begin
                            lit_cnt[k] <= w_cnt[k];
                            lit_first[k] <= w_first[k];
                            lit_offs[k] <= w_offs[k];
                        end
                        T_DIST: begin
                            dist_cnt[k] <= w_cnt[k];
                            dist_first[k] <= w_first[k];
                            dist_offs[k] <= w_offs[k];
                        end
                        default: begin
                            cl_cnt[k] <= w_cnt[k];
                            cl_first[k] <= w_first[k];
                            cl_offs[k] <= w_offs[k];
                        end
                    endcase
                end
                state <= ret_state;
            end

            // ---- Huffman decode sub-FSM ----
            S_H_INIT: begin
                h_code <= 15'd0;
                h_len <= 5'd0;
                state <= S_H_BIT;
            end

            S_H_BIT: begin
                if (take_ack) begin
                    if ((({1'b0, h_ncode} - {1'b0, chk_first}) <
                         {7'd0, chk_cnt})) begin
                        h_code <= h_ncode;
                        h_len <= h_nlen;
                        state <= S_H_SYM;
                    end else if (h_len == 5'd14) begin
                        error <= 3'd3;
                        done <= 1'b1;
                        o_crc <= ~crc_r;
                        o_out_count <= out_count_r;
                        state <= S_ERR;
                    end else begin
                        h_code <= h_ncode;
                        h_len <= h_nlen;
                        state <= S_H_BIT;
                    end
                end
            end

            S_H_SYM: begin
                // Index is offs + (code - first); the true sum is < 288
                // (offs + count <= symbol total), so 9-bit arithmetic is exact.
                h_sym_idx <= cur_offs + (h_code[8:0] - cur_first[8:0]);
                state <= S_H_SYM_RD;
            end

            S_H_SYM_RD: begin
                case (h_table)
                    T_LIT: h_sym <= lit_sym[h_sym_idx];
                    T_DIST: h_sym <= {4'd0, dist_sym[h_sym_idx[4:0]]};
                    default: h_sym <= {4'd0, cl_sym[h_sym_idx[4:0]]};
                endcase
                state <= ret_state;
            end

            // ---- emit one output byte ----
            S_E_PUT: begin
                o_valid <= 1'b1;
                o_data <= emit_byte;
                if (o_valid && i_ready) begin
                    o_valid <= 1'b0;
                    win_we <= 1'b1;
                    win_wa <= wptr;
                    win_wd <= emit_byte;
                    wptr <= wptr + 15'd1;
                    out_count_r <= out_count_r + 32'd1;
                    crc_r <= crc32_byte(crc_r, emit_byte);
                    state <= ret_state;
                end
            end

            default: state <= S_IDLE;
            endcase
            end
        end
    end

endmodule
