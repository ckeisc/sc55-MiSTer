// SPDX-License-Identifier: GPL-2.0-or-later
// PS/2 keyboard decoder for the archive browser. Watches hps_io's ps2_key
// ([10] toggles per event, [9] pressed, [8] extended, [7:0] scan code,
// Set 2) and emits a 1-cycle pulse for each browser key on PRESS only
// (releases are ignored, so holding a key does not repeat).
//
//   Up    = extended 0x75     Down  = extended 0x72
//   Left  = extended 0x6B     Right = extended 0x74
//   Enter = 0x5A (not extended)
module browser_kbd (
    input  logic clk,
    input  logic rst,
    input  logic [10:0] ps2_key,
    output logic nav_up,
    output logic nav_down,
    output logic nav_left,
    output logic nav_right,
    output logic nav_enter
);
    logic toggle_d;
    always_ff @(posedge clk) begin
        nav_up    <= 1'b0;
        nav_down  <= 1'b0;
        nav_left  <= 1'b0;
        nav_right <= 1'b0;
        nav_enter <= 1'b0;
        toggle_d  <= ps2_key[10];
        if (rst) begin
            toggle_d <= 1'b0;
        end else if (ps2_key[10] != toggle_d) begin
            // A new key event. Only presses act (bit 9 = 1); releases are
            // ignored so a held key does not repeat.
            if (ps2_key[9]) begin
                if (ps2_key[8]) begin
                    case (ps2_key[7:0])
                        8'h75: nav_up    <= 1'b1;
                        8'h72: nav_down  <= 1'b1;
                        8'h6b: nav_left  <= 1'b1;
                        8'h74: nav_right <= 1'b1;
                        default: ;
                    endcase
                end else begin
                    case (ps2_key[7:0])
                        8'h5a: nav_enter <= 1'b1;
                        default: ;
                    endcase
                end
            end
        end
    end
endmodule
