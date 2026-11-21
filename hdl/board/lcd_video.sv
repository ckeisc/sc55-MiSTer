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
    input  logic [2:0] msg,              // status text id (sc55_board):
                                         // 0 none, 1 ROM LOAD FAILED,
                                         // 2 MID LOAD FAILED, 3 LOADING ROM,
                                         // 4 NO ROM PACK, 5 PLAYING, 6 RUNNING
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
    localparam int X0 = 29, Y0 = 40, LW = 741, LH = 268;

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

    // Status text (one line below the strip, 15 cells of 18 px at 3 px/dot).
    // Character at cell c of message m; anything unlisted is a space.
    //   0 none, 1 ROM LOAD FAILED, 2 MID LOAD FAILED, 3 LOADING ROM,
    //   4 NO ROM PACK, 5 PLAYING, 6 RUNNING, 7 SCAN FAILED
    function automatic logic [7:0] msg_char(input logic [2:0] m, input logic [3:0] c);
        case (m)
            3'd1: begin // ROM LOAD FAILED
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
            3'd2: begin // MID LOAD FAILED
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
            3'd3: begin // LOADING ROM
                case (c)
                    4'd0: return "L"; 4'd1: return "O"; 4'd2: return "A"; 4'd3: return "D";
                    4'd4: return "I"; 4'd5: return "N"; 4'd6: return "G";
                    4'd7: return " ";
                    4'd8: return "R"; 4'd9: return "O"; 4'd10: return "M";
                    default: return " ";
                endcase
            end
            3'd4: begin // NO ROM PACK
                case (c)
                    4'd0: return "N"; 4'd1: return "O";
                    4'd2: return " ";
                    4'd3: return "R"; 4'd4: return "O"; 4'd5: return "M";
                    4'd6: return " ";
                    4'd7: return "P"; 4'd8: return "A"; 4'd9: return "C"; 4'd10: return "K";
                    default: return " ";
                endcase
            end
            3'd5: begin // PLAYING
                case (c)
                    4'd0: return "P"; 4'd1: return "L"; 4'd2: return "A"; 4'd3: return "Y";
                    4'd4: return "I"; 4'd5: return "N"; 4'd6: return "G";
                    default: return " ";
                endcase
            end
            3'd6: begin // RUNNING
                case (c)
                    4'd0: return "R"; 4'd1: return "U"; 4'd2: return "N"; 4'd3: return "N";
                    4'd4: return "I"; 4'd5: return "N"; 4'd6: return "G";
                    default: return " ";
                endcase
            end
            3'd7: begin // SCAN FAILED
                case (c)
                    4'd0: return "S"; 4'd1: return "C"; 4'd2: return "A"; 4'd3: return "N";
                    4'd4: return " ";
                    4'd5: return "F"; 4'd6: return "A"; 4'd7: return "I";
                    4'd8: return "L"; 4'd9: return "E"; 4'd10: return "D";
                    default: return " ";
                endcase
            end
            default: return " ";
        endcase
    endfunction

    function automatic logic [23:0] msg_color(input logic [2:0] m);
        case (m)
            3'd1, 3'd2, 3'd4, 3'd7: return 24'hff8080;  // errors: light red
            3'd3: return 24'hffcf40;              // loading: amber
            3'd5: return 24'h40e080;              // playing: green
            3'd6: return 24'hd0d0d0;              // running: light grey
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
    // status text line below the strip: absolute x 29..299 (15 cells of
    // 18 px), absolute y 380..401 (7 rows of 3 px)
    logic        txt_in0;
    logic [3:0]  txt_q0;                // cell number (18-pixel pitch)
    logic [4:0]  txt_cm0;               // offset in the cell
    logic [2:0]  txt_row0;              // font row (3 px per row)

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
        if ((x >= 11'd29) && (x < 11'd299) && (y >= 10'd380) && (y < 10'd401)) begin
            txt_in0  <= 1'b1;
            txt_q0   <= 4'((x - 11'd29) / 11'd18);
            txt_cm0  <= 5'((x - 11'd29) % 11'd18);
            txt_row0 <= 3'((y - 10'd380) / 10'd3);
        end else begin
            txt_in0  <= 1'b0;
            txt_q0   <= 4'd0;
            txt_cm0  <= 5'd0;
            txt_row0 <= 3'd0;
        end
    end

    // ------------------------------------------------------------ stage 1: what is at this pixel
    // kind: 0 background, 1 text dot, 2 level dot, 3 LR mark
    logic [1:0]  k1;
    logic [6:0]  ch_idx1;          // index into dd (LCD) or message cell (text line)
    logic [3:0]  frow1;            // font row
    logic [2:0]  fbit1;            // dot column (bit 4 - j)
    logic        txt1;             // this pixel is in the status text line
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
    end

    // ------------------------------------------------------------ stage 2: character code, background bit
    logic [1:0]  k2;
    logic [7:0]  ch2;
    logic [3:0]  frow2;
    logic [2:0]  fbit2;
    logic        txt2;             // this pixel is in the status text line
    logic        in_lcd2, hs2, vs2, hb2, vb2;
    logic [2:0]  back_bit2;
    logic [9:0]  lx2;
    logic [8:0]  ly2;
    always_ff @(posedge clk) begin
        k2 <= k1; frow2 <= frow1; fbit2 <= fbit1;
        ch2 <= txt1 ? msg_char(msg, ch_idx1[3:0]) : dd[ch_idx1];
        txt2 <= txt1;
        back_bit2 <= p1[2:0];
        in_lcd2 <= in_lcd1; hs2 <= hs1; vs2 <= vs1; hb2 <= hb1; vb2 <= vb1;
        lx2 <= lx1; ly2 <= ly1;
    end

    // ------------------------------------------------------------ stage 3: font row
    logic [1:0]  k3;
    logic [4:0]  row3;
    logic [2:0]  fbit3;
    logic        txt3;             // this pixel is in the status text line
    logic        back3, in_lcd3, hs3, vs3, hb3, vb3;
    logic [9:0]  lx3;
    logic [8:0]  ly3;
    always_ff @(posedge clk) begin
        k3 <= k2; fbit3 <= fbit2;
        txt3 <= txt2;
        row3 <= (ch2 >= 8'd16) ? font_row(ch2 - 8'd16, frow2) : cg[{ch2[2:0], frow2[2:0]}];
        back3 <= back_q[back_bit2];
        in_lcd3 <= in_lcd2; hs3 <= hs2; vs3 <= vs2; hb3 <= hb2; vb3 <= vb2;
        lx3 <= lx2; ly3 <= ly2;
    end

    // ------------------------------------------------------------ stage 4: colour
    // color1 (dot on) 000000, color2 (dot off) 0050c8, background 0f6fff / 000000
    logic [7:0] st;
    logic [2:0] msg4;             // status text id, delayed to this stage
    always_ff @(posedge clk) begin : b_lcd_video_2
        logic on;
        st <= {playing, lag_seen, underrun_seen, underrun_led, midi_led, running, loading, rom_ok};
        msg4 <= msg;
        hs <= !hs3; vs <= !vs3; hblank <= hb3; vblank <= vb3;
        lcd_pix <= in_lcd3; lcd_x <= lx3; lcd_y <= ly3;
        on = (k3 == 2'd3) ? row3[0] : row3[fbit3];
        if (in_lcd3) begin
            if (!enable) {r, g, b} <= 24'h000000;
            else if (k3 == 2'd0) {r, g, b} <= back3 ? 24'h000000 : 24'h0f6fff;
            else {r, g, b} <= on ? 24'h000000 : 24'h0050c8;
        end else begin
            {r, g, b} <= 24'h101828;
            // status strip under the LCD: state band, MIDI and underrun boxes
            if (ly3 >= 9'd300 && ly3 < 9'd320 && lx3 < 10'd200)
                {r, g, b} <= st[2] ? 24'h20c040 : st[1] ? 24'he0a020 : st[0] ? 24'h2060c0 : 24'hc02020;
            if (ly3 >= 9'd300 && ly3 < 9'd320 && lx3 >= 10'd220 && lx3 < 10'd260)
                {r, g, b} <= st[3] ? 24'hffffff : 24'h303848;
            if (ly3 >= 9'd300 && ly3 < 9'd320 && lx3 >= 10'd280 && lx3 < 10'd320)
                {r, g, b} <= st[4] ? 24'hff4040 : st[5] ? 24'h902020 : 24'h303848;
            if (ly3 >= 9'd300 && ly3 < 9'd320 && lx3 >= 10'd340 && lx3 < 10'd380)
                {r, g, b} <= st[6] ? 24'he0a020 : 24'h303848;
            // playing box: green while the built-in MID player is playing
            if (ly3 >= 9'd300 && ly3 < 9'd320 && lx3 >= 10'd400 && lx3 < 10'd440)
                {r, g, b} <= st[7] ? 24'h20c040 : 24'h303848;
            // status text line below the strip: one line of 5x7 text at
            // 3 px/dot (absolute y 380..401); blank when msg is 0, since
            // msg_char returns spaces for it
            if (txt3 && on)
                {r, g, b} <= msg_color(msg4);
        end
    end
endmodule
