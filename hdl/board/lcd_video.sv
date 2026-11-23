// 800 x 600 at 60 Hz (40 MHz pixel clock): the SC-55's LCD drawn 1:1 as the
// reference's LCD_Render draws it for the mk1 (lcd.cpp: 741 x 268, the
// background bitmap, the eight text fields of 5 x 7 dots of 5 x 5 pixels on
// a 6-pixel pitch, the L/R marks and the level bars), placed at (29, 40);
// below it a status strip. Reads the LCD controller's state from the core
// clock without synchronisation (display only).
module lcd_video (
    input  logic       clk,              // pixel clock
    input  logic [7:0] dd [80],
    input  logic [4:0] cg [64],
    input  logic       enable,
    // status strip
    input  logic       rom_ok,
    input  logic       loading,
    input  logic       running,
    input  logic       playing,          // the built-in MID player is playing
    input  logic       midi_led,
    input  logic       underrun_led,
    input  logic       underrun_seen,
    input  logic       lag_seen,
    input  logic [4:0] msg,              // status text id (sc55_board):
                                         // 0 none, 1 ROM LOAD FAILED,
                                         // 2 MID LOAD FAILED, 3 LOADING ROM,
                                         // 4 NO ROM PACK, 5 PLAYING, 6 RUNNING
    // browser text region (character buffer, dual-clock RAM)
    input  logic       browser_visible,  // show the file browser
    input  logic [9:0] char_rd,          // char buffer read data {attr[1:0], ascii[7:0]}
    output logic [9:0] char_ra,          // char buffer read address
    output logic       hs,
    output logic       vs,
    output logic       hblank,
    output logic       vblank,
    output logic [7:0] r,
    output logic [7:0] g,
    output logic [7:0] b,
    // the LCD pixel at the output (bench): valid, position, colour
    output logic       lcd_pix,
    output logic [9:0] lcd_x,
    output logic [8:0] lcd_y
);
    localparam int X0 = 29, Y0 = 29, LW = 741, LH = 268;

    // background bitmap: pixel p in bit p % 8 of byte p / 8
`ifndef LCD_BACK_HEX
    `define LCD_BACK_HEX "../hdl/board/lcd_back.hex"     // Quartus: relative to mister/
`endif
    logic [14:0] back_a;
    logic [7:0]  back_q;
    sdp_ram #(.AW(15), .DW(8), .INIT(`LCD_BACK_HEX)) u_back (
        .clk, .we(1'b0), .wa(15'd0), .wd(8'd0), .ra(back_a), .rd(back_q));
    assign back_a = 15'(p1 >> 3);

    function automatic logic [4:0] font_row(input logic [7:0] fc, input logic [3:0] fr);
        `include "lcd_font.svh"
    endfunction

    // Status text (one line to the right of the strip, 15 cells of 18 px at 3 px/dot).
    // Character at cell c of message m; anything unlisted is a space.
    //   0 none, 1 ROM LOAD FAILED, 2 MID LOAD FAILED, 3 LOADING ROM,
    //   4 NO ROM PACK, 5 PLAYING, 6 RUNNING, 7 SCAN FAILED,
    //   8 ZIP DL FAILED, 9 ZIP PARSE FAIL, 10 ZIP EXT FAILED,
    //   11 ZIP DL SEEN (diagnostic: a ZIP download reached the core
    //   but the archive is not parsed yet),
    //   12-15 DL IDX 0/1/2/? (diagnostic: the ioctl_index of the last
    //   download seen while the core was running),
    //   16 ZIP DL DONE (diagnostic: a ZIP download ended with >=22 bytes),
    //   18 ARCH OK (diagnostic: the archive parsed), 19 LOADING ZIP (live),
    //   20-24 parse-stage ladder (diagnostic: how far the central-directory
    //   parse got — START, RD accepted, DATA returned, EOCD FOUND, CD WALK)
    function automatic logic [7:0] msg_char(input logic [4:0] m, input logic [3:0] c);
        case (m)
             5'd1: begin // ROM LOAD FAILED
                case (c)
                    4'd0: return "R"; 4'd1: return "O"; 4'd2: return "M";
                    4'd3: return " ";
                    4'd4: return "L"; 4'd5: return "O"; 4'd6: return "A"; 4'd7: return "D";
                    4'd8: return " ";
                    4'd9: return "F"; 4'd10: return "A"; 4'd11: return "I";
                    4'd12: return "L"; 4'd13: return "E"; 4'd14: return "D";
                    default: return " ";
                endcase
            end
             5'd2: begin // MID LOAD FAILED
                case (c)
                    4'd0: return "M"; 4'd1: return "I"; 4'd2: return "D";
                    4'd3: return " ";
                    4'd4: return "L"; 4'd5: return "O"; 4'd6: return "A"; 4'd7: return "D";
                    4'd8: return " ";
                    4'd9: return "F"; 4'd10: return "A"; 4'd11: return "I";
                    4'd12: return "L"; 4'd13: return "E"; 4'd14: return "D";
                    default: return " ";
                endcase
            end
             5'd3: begin // LOADING ROM
                case (c)
                    4'd0: return "L"; 4'd1: return "O"; 4'd2: return "A"; 4'd3: return "D";
                    4'd4: return "I"; 4'd5: return "N"; 4'd6: return "G";
                    4'd7: return " ";
                    4'd8: return "R"; 4'd9: return "O"; 4'd10: return "M";
                    default: return " ";
                endcase
            end
             5'd4: begin // NO ROM PACK
                case (c)
                    4'd0: return "N"; 4'd1: return "O";
                    4'd2: return " ";
                    4'd3: return "R"; 4'd4: return "O"; 4'd5: return "M";
                    4'd6: return " ";
                    4'd7: return "P"; 4'd8: return "A"; 4'd9: return "C"; 4'd10: return "K";
                    default: return " ";
                endcase
            end
             5'd5: begin // PLAYING
                case (c)
                    4'd0: return "P"; 4'd1: return "L"; 4'd2: return "A"; 4'd3: return "Y";
                    4'd4: return "I"; 4'd5: return "N"; 4'd6: return "G";
                    default: return " ";
                endcase
            end
             5'd6: begin // RUNNING
                case (c)
                    4'd0: return "R"; 4'd1: return "U"; 4'd2: return "N"; 4'd3: return "N";
                    4'd4: return "I"; 4'd5: return "N"; 4'd6: return "G";
                    default: return " ";
                endcase
            end
             5'd7: begin // SCAN FAILED
                case (c)
                    4'd0: return "S"; 4'd1: return "C"; 4'd2: return "A"; 4'd3: return "N";
                    4'd4: return " ";
                    4'd5: return "F"; 4'd6: return "A"; 4'd7: return "I";
                    4'd8: return "L"; 4'd9: return "E"; 4'd10: return "D";
                    default: return " ";
                endcase
            end
             5'd8: begin // ZIP DL FAILED
                case (c)
                    4'd0: return "Z"; 4'd1: return "I"; 4'd2: return "P";
                    4'd3: return " ";
                    4'd4: return "D"; 4'd5: return "L";
                    4'd6: return " ";
                    4'd7: return "F"; 4'd8: return "A"; 4'd9: return "I";
                    4'd10: return "L"; 4'd11: return "E"; 4'd12: return "D";
                    default: return " ";
                endcase
            end
             5'd9: begin // ZIP PARSE FAIL
                case (c)
                    4'd0: return "Z"; 4'd1: return "I"; 4'd2: return "P";
                    4'd3: return " ";
                    4'd4: return "P"; 4'd5: return "A"; 4'd6: return "R";
                    4'd7: return "S"; 4'd8: return "E";
                    4'd9: return " ";
                    4'd10: return "F"; 4'd11: return "A"; 4'd12: return "I";
                    4'd13: return "L";
                    default: return " ";
                endcase
            end
             5'd10: begin // ZIP EXT FAILED
                case (c)
                    4'd0: return "Z"; 4'd1: return "I"; 4'd2: return "P";
                    4'd3: return " ";
                    4'd4: return "E"; 4'd5: return "X"; 4'd6: return "T";
                    4'd7: return " ";
                    4'd8: return "F"; 4'd9: return "A"; 4'd10: return "I";
                    4'd11: return "L"; 4'd12: return "E"; 4'd13: return "D";
                    default: return " ";
                endcase
            end
             5'd11: begin // ZIP DL SEEN
                case (c)
                    4'd0: return "Z"; 4'd1: return "I"; 4'd2: return "P";
                    4'd3: return " ";
                    4'd4: return "D"; 4'd5: return "L";
                    4'd6: return " ";
                    4'd7: return "S"; 4'd8: return "E"; 4'd9: return "E";
                    4'd10: return "N";
                    default: return " ";
                endcase
            end
            5'd12: begin // DL IDX 0
                case (c)
                    4'd0: return "D"; 4'd1: return "L";
                    4'd2: return " ";
                    4'd3: return "I"; 4'd4: return "D"; 4'd5: return "X";
                    4'd6: return " ";
                    4'd7: return "0";
                    default: return " ";
                endcase
            end
            5'd13: begin // DL IDX 1
                case (c)
                    4'd0: return "D"; 4'd1: return "L";
                    4'd2: return " ";
                    4'd3: return "I"; 4'd4: return "D"; 4'd5: return "X";
                    4'd6: return " ";
                    4'd7: return "1";
                    default: return " ";
                endcase
            end
            5'd14: begin // DL IDX 2
                case (c)
                    4'd0: return "D"; 4'd1: return "L";
                    4'd2: return " ";
                    4'd3: return "I"; 4'd4: return "D"; 4'd5: return "X";
                    4'd6: return " ";
                    4'd7: return "2";
                    default: return " ";
                endcase
            end
            5'd15: begin // DL IDX ?
                case (c)
                    4'd0: return "D"; 4'd1: return "L";
                    4'd2: return " ";
                    4'd3: return "I"; 4'd4: return "D"; 4'd5: return "X";
                    4'd6: return " ";
                    4'd7: return "?";
                    default: return " ";
                endcase
            end
            5'd16: begin // ZIP DL DONE
                case (c)
                    4'd0: return "Z"; 4'd1: return "I"; 4'd2: return "P";
                    4'd3: return " ";
                    4'd4: return "D"; 4'd5: return "L";
                    4'd6: return " ";
                    4'd7: return "D"; 4'd8: return "O"; 4'd9: return "N";
                    4'd10: return "E";
                    default: return " ";
                endcase
            end
            5'd18: begin // MZP LOADED (was "ARCH OK": the archive parsed
                // and the browser is up; "MZP LOADED" names what happened.
                // The core never receives the archive filename from Main,
                // so the generic name is used, not "<file>.MZP LOADED".)
                case (c)
                    4'd0: return "M"; 4'd1: return "Z"; 4'd2: return "P";
                    4'd3: return " ";
                    4'd4: return "L"; 4'd5: return "O"; 4'd6: return "A"; 4'd7: return "D";
                    4'd8: return "E"; 4'd9: return "D";
                    default: return " ";
                endcase
            end
            5'd19: begin // LOADING ZIP
                case (c)
                    4'd0: return "L"; 4'd1: return "O"; 4'd2: return "A"; 4'd3: return "D";
                    4'd4: return "I"; 4'd5: return "N"; 4'd6: return "G";
                    4'd7: return " ";
                    4'd8: return "Z"; 4'd9: return "I"; 4'd10: return "P";
                    default: return " ";
                endcase
            end
            5'd20: begin // PARSE START
                case (c)
                    4'd0: return "P"; 4'd1: return "A"; 4'd2: return "R"; 4'd3: return "S";
                    4'd4: return "E";
                    4'd5: return " ";
                    4'd6: return "S"; 4'd7: return "T"; 4'd8: return "A"; 4'd9: return "R";
                    4'd10: return "T";
                    default: return " ";
                endcase
            end
            5'd21: begin // PARSE RD
                case (c)
                    4'd0: return "P"; 4'd1: return "A"; 4'd2: return "R"; 4'd3: return "S";
                    4'd4: return "E";
                    4'd5: return " ";
                    4'd6: return "R"; 4'd7: return "D";
                    default: return " ";
                endcase
            end
            5'd22: begin // PARSE DATA
                case (c)
                    4'd0: return "P"; 4'd1: return "A"; 4'd2: return "R"; 4'd3: return "S";
                    4'd4: return "E";
                    4'd5: return " ";
                    4'd6: return "D"; 4'd7: return "A"; 4'd8: return "T"; 4'd9: return "A";
                    default: return " ";
                endcase
            end
            5'd23: begin // EOCD FOUND
                case (c)
                    4'd0: return "E"; 4'd1: return "O"; 4'd2: return "C"; 4'd3: return "D";
                    4'd4: return " ";
                    4'd5: return "F"; 4'd6: return "O"; 4'd7: return "U"; 4'd8: return "N";
                    4'd9: return "D";
                    default: return " ";
                endcase
            end
            5'd24: begin // CD WALK
                case (c)
                    4'd0: return "C"; 4'd1: return "D";
                    4'd2: return " ";
                    4'd3: return "W"; 4'd4: return "A"; 4'd5: return "L"; 4'd6: return "K";
                    default: return " ";
                endcase
            end
            default: return " ";
        endcase
    endfunction

    function automatic logic [23:0] msg_color(input logic [4:0] m);
        case (m)
            5'd1, 5'd2, 5'd4, 5'd7, 5'd8, 5'd9, 5'd10: return 24'hff8080;  // errors: light red
            5'd3, 5'd11, 5'd12, 5'd13, 5'd14, 5'd15, 5'd16, 5'd19, 5'd20, 5'd21, 5'd22, 5'd23, 5'd24: return 24'hffcf40;  // loading / diagnostics: amber
            5'd18: return 24'h40e080;              // archive parsed: green
            5'd5: return 24'h40e080;              // playing: green
            5'd6: return 24'hd0d0d0;              // running: light grey
            default: return 24'hffffff;
        endcase
    endfunction

    // LR marks (lcd.cpp LR), 12 rows of 11 dots
    function automatic logic lr_dot(input logic letter, input logic [3:0] i, input logic [3:0] j);
        logic [10:0] row;
        if (!letter) row = (i >= 4'd10) ? 11'b11111111111 : 11'b11000000000;
        else case (i)
            4'd0: row = 11'b11111111100;
            4'd1: row = 11'b11111111110;
            4'd2, 4'd3, 4'd4: row = 11'b11000000110;
            4'd5: row = 11'b11111111110;
            4'd6: row = 11'b11111111100;
            4'd7: row = 11'b11000001100;
            4'd8, 4'd9: row = 11'b11000000110;
            default: row = 11'b11000000011;
        endcase
        return row[4'd10 - j];
    endfunction

    // ------------------------------------------------------------ timing
    logic [10:0] x;
    logic [9:0]  y;
    always_ff @(posedge clk) begin
        if (x == 11'd1055) begin
            x <= 11'd0;
            y <= (y == 10'd627) ? 10'd0 : y + 10'd1;
        end else x <= x + 11'd1;
    end

    // ------------------------------------------------------------ stage 0: per-axis cells
    // x and y separately: position in the LCD, the text/bar cell and the offset
    // in it (one constant division each); stage 1 combines them
    logic [10:0] x0;
    logic [9:0]  y0;
    logic        in_area0;
    logic [9:0]  lx0, ly0;
    logic [1:0]  tx_in3, tx_in16;       // per side: inside 3 (side 1 also 16) cells
    logic [3:0]  tx_q [2];              // cell number (35-pixel pitch)
    logic [5:0]  tx_cm [2];             // offset in the cell
    logic        bx_in;
    logic [1:0]  bx_q;                  // bar column (130-pixel pitch)
    logic [7:0]  bx_cm;
    logic [3:0]  ty_in;                 // per text field: inside its 42 rows
    logic [5:0]  ty_d [4];
    logic [1:0]  by_in;                 // per bar row: inside its 86 rows
    logic [6:0]  by_d [2];
    // status text line in the strip row: LCD-relative lx 0..270 (15 cells
    // of 18 px), ly 300..321 (7 rows of 3 px). It sits on the left of the
    // strip, replacing the old state color band; the text itself names
    // the state, so the band is redundant. Positioned relative to the LCD
    // origin so it moves with the LCD and the strip.
    logic        txt_in0;
    logic [3:0]  txt_q0;                // cell number (18-pixel pitch)
    logic [4:0]  txt_cm0;               // offset in the cell
    logic [2:0]  txt_row0;              // font row (3 px per row)
    // browser text region: absolute x 29..770 (46 cols of 16 px),
    // absolute y 360..584 (14 rows of 16 px); the 5x7 font is drawn
    // 2x (10x14) in the top-left of each 16x16 cell. It starts one
    // full line below the status strip (which ends at absolute y 350),
    // so the first browser row never overlaps the status text or the
    // four LEDs; it ends at 584, leaving a 16 px bottom margin.
    logic        brw_in0;
    logic [3:0]  brw_xoff0;             // offset in the cell (0..15)
    logic [3:0]  brw_yoff0;

    function automatic logic [9:0] div35(input logic [9:0] v); return v / 10'd35; endfunction
    function automatic logic [9:0] mod35(input logic [9:0] v); return v % 10'd35; endfunction
    function automatic logic [9:0] div130(input logic [9:0] v); return v / 10'd130; endfunction
    function automatic logic [9:0] mod130(input logic [9:0] v); return v % 10'd130; endfunction

    always_ff @(posedge clk) begin : stage0
        logic [10:0] lxs, lys;          // x - X0, y - Y0 (two's complement, 11 bits)
        logic [9:0]  lx, ly, c;
        lxs = x - 11'(X0);
        lys = {1'b0, y} - 11'(Y0);
        lx = lxs[9:0];
        ly = lys[9:0];
        x0 <= x;
        y0 <= y;
        in_area0 <= !lxs[10] && !lys[10] && lxs < 11'(LW) && lys < 11'(LH);
        lx0 <= lx;
        ly0 <= ly;
        // text columns 34 (3 cells) and 153 (3 cells, or 16 in field 0)
        for (int sd = 0; sd < 2; sd++) begin
            c = lx - ((sd != 0) ? 10'd153 : 10'd34);
            tx_in3[sd]  <= lx >= ((sd != 0) ? 10'd153 : 10'd34) && c < 10'd105;
            tx_in16[sd] <= lx >= ((sd != 0) ? 10'd153 : 10'd34) && c < 10'd560;
            tx_q[sd]  <= 4'(div35(c));
            tx_cm[sd] <= 6'(mod35(c));
        end
        c = lx - 10'd293;
        bx_in <= lx >= 10'd293 && c < 10'd520;
        bx_q  <= 2'(div130(c));
        bx_cm <= 8'(mod130(c));
        for (int f = 0; f < 4; f++) begin
            ty_in[f] <= ly >= 10'(11 + 64 * f) && ly < 10'(11 + 64 * f + 42);
            ty_d[f]  <= 6'(ly - 10'(11 + 64 * f));
        end
        for (int i = 0; i < 2; i++) begin
            by_in[i] <= ly >= 10'(71 + 88 * i) && ly < 10'(71 + 88 * i + 86);
            by_d[i]  <= 7'(ly - 10'(71 + 88 * i));
        end
        if ((lx < 10'd270) && (ly >= 10'd300) && (ly < 10'd321)) begin
            txt_in0  <= 1'b1;
            txt_q0   <= 4'(lx / 10'd18);
            txt_cm0  <= 5'(lx % 10'd18);
            txt_row0 <= 3'((ly - 10'd300) / 10'd3);
        end else begin
            txt_in0  <= 1'b0;
            txt_q0   <= 4'd0;
            txt_cm0  <= 5'd0;
            txt_row0 <= 3'd0;
        end
        // Browser region: x 29..770, y 360..584. Present the char
        // buffer address here so the registered read lands in stage 2.
        if (browser_visible && (x >= 11'd29) && (x < 11'd770) && (y >= 10'd360) && (y < 10'd584)) begin
            brw_in0   <= 1'b1;
            brw_xoff0 <= 4'((x - 11'd29) % 11'd16);
            brw_yoff0 <= 4'((y - 10'd360) % 10'd16);
            char_ra   <= {4'((y - 10'd360) / 10'd16), 6'((x - 11'd29) / 11'd16)};
        end else begin
            brw_in0   <= 1'b0;
            brw_xoff0 <= 4'd0;
            brw_yoff0 <= 4'd0;
            char_ra   <= 10'd0;
        end
    end

    // ------------------------------------------------------------ stage 1: what is at this pixel
    // kind: 0 background, 1 text dot, 2 level dot, 3 LR mark
    logic [1:0]  k1;
    logic [6:0]  ch_idx1;          // index into dd (LCD) or message cell (text line)
    logic [3:0]  frow1;            // font row
    logic [2:0]  fbit1;            // dot column (bit 4 - j)
    logic        txt1;             // this pixel is in the status text line
    logic        brw1;             // this pixel is in the browser text region
    logic        in_lcd1, vis1, hs1, vs1, hb1, vb1;
    logic [17:0] p1;
    logic [9:0]  lx1;
    logic [8:0]  ly1;

    always_ff @(posedge clk) begin : stage1
        logic [5:0] cm;
        logic [7:0] bm;
        logic       inn;
        hb1 <= x0 >= 11'd800;
        vb1 <= y0 >= 10'd600;
        hs1 <= (x0 >= 11'd840 && x0 < 11'd968);
        vs1 <= (y0 >= 10'd601 && y0 < 10'd605);
        in_lcd1 <= in_area0;
        lx1 <= lx0;
        ly1 <= ly0[8:0];
        p1 <= 18'(ly0) * 18'(LW) + 18'(lx0);
        k1 <= 2'd0;
        ch_idx1 <= 7'd0;
        frow1 <= 4'd0;
        fbit1 <= 3'd0;
        // text fields: rows 11, 75, 139, 203; dots 5 x 5 on a 6-pixel pitch, cells of 35
        for (int f = 0; f < 4; f++) begin
            if (in_area0 && ty_in[f] && (ty_d[f] % 6'd6) < 6'd5) begin
                for (int sd = 0; sd < 2; sd++) begin : b_lcd_video_1
                    logic [6:0] base;
                    case (f * 2 + sd)
                        0: base = 7'd0;  1: base = 7'd3;  2: base = 7'd40; 3: base = 7'd43;
                        4: base = 7'd49; 5: base = 7'd46; 6: base = 7'd52; default: base = 7'd55;
                    endcase
                    inn = (sd != 0 && f == 0) ? tx_in16[sd] : tx_in3[sd];
                    cm = tx_cm[sd];
                    if (inn && cm < 6'd30 && (cm % 6'd6) < 6'd5) begin
                        k1 <= 2'd1;
                        ch_idx1 <= base + 7'(tx_q[sd]);
                        frow1 <= 4'(ty_d[f] / 6'd6);
                        fbit1 <= 3'(6'd4 - cm / 6'd6);
                    end
                end
            end
        end
        // level bars: rows 71 + 88 i, 8 dots of 9 rows on 11; columns 293 + 130 j, dots 24 wide on 26
        for (int i = 0; i < 2; i++) begin
            if (in_area0 && by_in[i] && (by_d[i] % 7'd11) < 7'd9 && bx_in) begin
                bm = bx_cm;
                if ((bm % 8'd26) < 8'd24 && (bm / 8'd26) < ((bx_q == 2'd3) ? 8'd1 : 8'd5)) begin
                    k1 <= 2'd2;
                    ch_idx1 <= 7'd20 + 7'(bx_q) + 7'(40 * i);
                    frow1 <= 4'(by_d[i] / 7'd11);
                    fbit1 <= 3'(8'd4 - bm / 8'd26);
                end
            end
        end
        // LR marks at rows 70 and 232, columns 264..274
        for (int l = 0; l < 2; l++) begin
            if (in_area0 && ly0 >= 10'((l != 0) ? 232 : 70) && ly0 < 10'((l != 0) ? 244 : 82) && lx0 >= 10'd264 && lx0 < 10'd275) begin
                if (lr_dot(1'(l), 4'(ly0 - 10'((l != 0) ? 232 : 70)), 4'(lx0 - 10'd264))) begin
                    k1 <= 2'd3;
                    ch_idx1 <= 7'd58;
                    frow1 <= 4'd0;
                    fbit1 <= 3'd0;
                end
            end
        end
        // Status text line below the strip: same font pipeline as the LCD
        // text, but the character comes from the message lookup (stage 2).
        // The LCD area (absolute y 40..308) and this line (y 380..401)
        // never overlap, so they cannot both claim the pixel.
        txt1 <= 1'b0;
        if (txt_in0 && txt_cm0 < 5'd15) begin
            k1 <= 2'd1;
            ch_idx1 <= {3'd0, txt_q0};
            frow1 <= {1'b0, txt_row0};
            fbit1 <= 3'd4 - 3'(txt_cm0 / 5'd3);
            txt1 <= 1'b1;
        end
        // Browser text region: the character comes from the char buffer
        // (stage 2, registered read). The 5x7 font is drawn 2x, so a
        // dot is on when the font row has the bit for this dot column.
        // Pixels outside the 10x14 glyph area are background.
        brw1 <= 1'b0;
        if (brw_in0 && brw_xoff0 < 4'd10 && brw_yoff0 < 4'd14) begin
            k1 <= 2'd1;
            ch_idx1 <= 7'd0;
            frow1 <= {1'b0, brw_yoff0[3:1]};
            fbit1 <= 3'd4 - 3'(brw_xoff0[3:1]);
            brw1 <= 1'b1;
        end
    end

    // ------------------------------------------------------------ stage 2: character code, background bit
    logic [1:0]  k2;
    logic [7:0]  ch2;
    logic [1:0]  attr2;            // browser cell color attr (00 white, 01 green, 10 amber)
    logic [3:0]  frow2;
    logic [2:0]  fbit2;
    logic        txt2;             // this pixel is in the status text line
    logic        brw2;             // this pixel is in the browser text region
    logic        in_lcd2, hs2, vs2, hb2, vb2;
    logic [2:0]  back_bit2;
    logic [9:0]  lx2;
    logic [8:0]  ly2;
    always_ff @(posedge clk) begin
        k2 <= k1; frow2 <= frow1; fbit2 <= fbit1;
        ch2 <= brw1 ? char_rd[7:0] : (txt1 ? msg_char(msg, ch_idx1[3:0]) : dd[ch_idx1]);
        attr2 <= brw1 ? char_rd[9:8] : 2'b00;
        txt2 <= txt1;
        brw2 <= brw1;
        back_bit2 <= p1[2:0];
        in_lcd2 <= in_lcd1; hs2 <= hs1; vs2 <= vs1; hb2 <= hb1; vb2 <= vb1;
        lx2 <= lx1; ly2 <= ly1;
    end

    // ------------------------------------------------------------ stage 3: font row
    logic [1:0]  k3;
    logic [4:0]  row3;
    logic [1:0]  attr3;
    logic [2:0]  fbit3;
    logic        txt3;             // this pixel is in the status text line
    logic        brw3;             // this pixel is in the browser text region
    logic        back3, in_lcd3, hs3, vs3, hb3, vb3;
    logic [9:0]  lx3;
    logic [8:0]  ly3;
    always_ff @(posedge clk) begin
        k3 <= k2; fbit3 <= fbit2;
        attr3 <= attr2;
        txt3 <= txt2;
        brw3 <= brw2;
        row3 <= (ch2 >= 8'd16) ? font_row(ch2 - 8'd16, frow2) : cg[{ch2[2:0], frow2[2:0]}];
        back3 <= back_q[back_bit2];
        in_lcd3 <= in_lcd2; hs3 <= hs2; vs3 <= vs2; hb3 <= hb2; vb3 <= vb2;
        lx3 <= lx2; ly3 <= ly2;
    end

    // ------------------------------------------------------------ stage 4: colour
    // color1 (dot on) 000000, color2 (dot off) 0050c8, background 0f6fff / 000000
    logic [7:0] st;
    logic [4:0] msg4;             // status text id, delayed to this stage
    always_ff @(posedge clk) begin : b_lcd_video_2
        logic on;
        st <= {playing, lag_seen, underrun_seen, underrun_led, midi_led, running, loading, rom_ok};
        msg4 <= msg;
        hs <= !hs3; vs <= !vs3; hblank <= hb3; vblank <= vb3;
        lcd_pix <= in_lcd3; lcd_x <= lx3; lcd_y <= ly3;
        on = (k3 == 2'd3) ? row3[0] : row3[fbit3];
        if (in_lcd3) begin
            if (!enable) {r, g, b} <= 24'h000000;
            else if (k3 == 2'd0) {r, g, b} <= back3 ? 24'h000000 : 24'hff8c00;
            else {r, g, b} <= on ? 24'h000000 : 24'hff8c00;
        end else begin
            {r, g, b} <= 24'h101828;
            // browser text region: colored dots on the dark background.
            // attr 00 white (normal), 01 green (loaded .m3u), 10 amber
            // (playing song, matching the LCD orange).
            if (brw3 && on)
                {r, g, b} <= (attr3 == 2'b01) ? 24'h40e080 :
                             (attr3 == 2'b10) ? 24'hff8c00 : 24'hffffff;
            // status strip under the LCD: the state band is gone (the
            // status text now names the state). Four square LEDs, 21x21,
            // same height as the text line, right-aligned to the LCD's
            // right edge so the text has the full left side.
            if (ly3 >= 9'd300 && ly3 < 9'd321 && lx3 >= 10'd645 && lx3 < 10'd666)
                {r, g, b} <= st[3] ? 24'hffffff : 24'h303848;
            if (ly3 >= 9'd300 && ly3 < 9'd321 && lx3 >= 10'd670 && lx3 < 10'd691)
                {r, g, b} <= st[4] ? 24'hff4040 : st[5] ? 24'h902020 : 24'h303848;
            if (ly3 >= 9'd300 && ly3 < 9'd321 && lx3 >= 10'd695 && lx3 < 10'd716)
                {r, g, b} <= st[6] ? 24'he0a020 : 24'h303848;
            // playing box: green while the built-in MID player is playing
            if (ly3 >= 9'd300 && ly3 < 9'd321 && lx3 >= 10'd720 && lx3 < 10'd741)
                {r, g, b} <= st[7] ? 24'h20c040 : 24'h303848;
            // status text line in the strip row: one line of 5x7 text at
            // 3 px/dot (LCD-relative lx 0..270, ly 300..321); blank when
            // msg is 0, since msg_char returns spaces for it
            if (txt3 && on)
                {r, g, b} <= msg_color(msg4);
        end
    end
endmodule
