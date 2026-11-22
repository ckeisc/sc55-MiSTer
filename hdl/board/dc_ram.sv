// SPDX-License-Identifier: GPL-2.0-or-later
// Dual-clock simple dual-port RAM: written on wr_clk, read on rd_clk.
// Maps to block RAM (M20K) with independent clocks. The read is
// registered (rd_data updates one rd_clk after rd_addr changes).
module dc_ram #(
    parameter int AW = 10,
    parameter int DW = 8
) (
    input  logic            wr_clk,
    input  logic            wr_en,
    input  logic [AW-1:0]  wr_addr,
    input  logic [DW-1:0]  wr_data,
    input  logic            rd_clk,
    input  logic [AW-1:0]  rd_addr,
    output logic [DW-1:0]  rd_data
);
    logic [DW-1:0] mem [2**AW];
    always_ff @(posedge wr_clk) begin
        if (wr_en) mem[wr_addr] <= wr_data;
    end
    always_ff @(posedge rd_clk) begin
        rd_data <= mem[rd_addr];
    end
endmodule
