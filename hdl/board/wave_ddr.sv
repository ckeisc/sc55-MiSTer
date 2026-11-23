// Wave ROM in MiSTer's DDR3 (DDRAM_*, Avalon-MM, 64-bit words, burst 1).
//
// Read stream for the PCM chip (sc55_pcm rom_*): byte requests taken with
// req && ready, bytes returned in request order on rvalid. A request in the
// same 8-byte word as the request just before it shares that word's DDR3
// read (the four sample bytes of a voice are usually consecutive), so a
// voice costs about two DDR3 reads; reads are pipelined (commands do not
// wait for data). A second byte-read port (preq/paddr/pready/prvalid/prdata)
// serves the built-in MID player (midi_player): the two byte ports share
// the request/command/data queues, with a per-entry stream tag routing the
// byte back and a per-stream last-word merge state, so a player request can
// never steal a PCM byte (and vice versa). A third byte-read port
// (areq/aaddr/aready/arvalid/ardata) serves the ZIP archive loader
// (zip_load): the central-directory parser and the entry extractor. At most
// one byte request is accepted per clock; the PCM port (audio) wins, then
// the player port, then the archive port. Write port for loading: one byte
// per write, held until accepted (wbusy). Line port: one 64-bit word at
// BASE + 0x400000 + 8 * laddr (ROM2's upper 256 KB on the mk2, for
// rom2x_cache), ahead of queued PCM reads; returned data is routed by a
// tag kept per issued read.
//
// The wave ROMs live at BASE (bytes; DDR3 from 0x30000000 is free for cores).
// The byte ports cover the whole 48 MB window (waves 0..0x3FFFFF, ROM2 upper
// half 0x400000..0x43FFFF, MID files 0x440000..0x7FFFFF, ZIP archive
// 0x800000..0x2FFFFFF). The PCM port only addresses the waves (22-bit),
// the player port the MID window (23-bit); only the archive port and the
// write port use the full 26-bit range, so the command queue holds 23-bit
// word offsets.
module wave_ddr #(
    parameter logic [31:0] BASE = 32'h3000_0000,
    parameter int QBITS = 4
) (
    input  logic        clk,
    input  logic        rst,

    input  logic        wr,
    input  logic [25:0] waddr,
    input  logic [7:0]  wdata,
    output logic        wbusy,

    input  logic        req,
    input  logic [21:0] addr,
    output logic        ready,
    output logic        rvalid,
    output logic [7:0]  rdata,

    input  logic        preq,             // player (midi_player) byte read
    input  logic [22:0] paddr,
    output logic        pready,
    output logic        prvalid,
    output logic [7:0]  prdata,

    input  logic        areq,             // archive (zip_load) byte read
    input  logic [25:0] aaddr,
    output logic        aready,
    output logic        arvalid,
    output logic [7:0]  ardata,

    input  logic        lreq,             // held until lvalid
    input  logic [14:0] laddr,
    output logic        lvalid,
    output logic [63:0] ldata,

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
    localparam int Q = 1 << QBITS;
    localparam logic [28:0] BASE_W = BASE[31:3];

    // request entries {stream, new word read, byte in word}, in order;
    // stream 0 = PCM byte port, 1 = player byte port, 2 = archive byte port
    logic [5:0]        rq [Q];
    logic [QBITS:0]    rq_wp, rq_rp;
    // DDR3 commands (word offsets) still to issue
    logic [22:0]       cq [Q];
    logic [QBITS:0]    cq_wp, cq_rp;
    // returned words
    logic [63:0]       dq [Q];
    logic [QBITS:0]    dq_wp, dq_rp;
    logic [QBITS:0]    inflight;        // commands issued, data not yet back
    logic              tq [Q];          // per issued read: 1 = line port
    logic [QBITS:0]    tq_wp, tq_rp;
    logic              l_busy;          // the line read is issued or waiting
    logic [22:0]       last_word, p_last_word, a_last_word;
    logic              have_last, p_have_last, a_have_last;
    logic [63:0]       cur, p_cur, a_cur;

    wire [22:0] word   = {4'b0, addr[21:3]};   // PCM port: 22-bit byte address
    wire        same   = have_last && word == last_word;
    wire [22:0] p_word = {3'b0, paddr[22:3]};  // player port: 23-bit byte address
    wire        p_same = p_have_last && p_word == p_last_word;
    wire [22:0] a_word = aaddr[25:3];          // archive port: 26-bit byte address
    wire        a_same = a_have_last && a_word == a_last_word;
    wire        rq_full = (rq_wp - rq_rp) == (QBITS+1)'(Q);
    wire        cq_full = (cq_wp - cq_rp) == (QBITS+1)'(Q);
    // a new word needs room for its data too (commands issued + queued + words held < Q)
    wire        d_room = ((cq_wp - cq_rp) + inflight + (dq_wp - dq_rp)) < (QBITS+1)'(Q - 1);
    wire        pcm_acc = req && ready;
    assign ready = !rq_full && !cq_full && d_room && !wbusy;
    // At most one byte request is accepted per clock; the PCM port (audio)
    // wins, then the player port, then the archive port.
    wire        player_acc = preq && pready;
    assign pready = !rq_full && !cq_full && d_room && !wbusy && !pcm_acc;
    assign aready = !rq_full && !cq_full && d_room && !wbusy && !pcm_acc && !player_acc;

    assign DDRAM_BURSTCNT = 8'd1;

    always_ff @(posedge clk) begin
        rvalid <= 1'b0;
        prvalid <= 1'b0;
        arvalid <= 1'b0;
        lvalid <= 1'b0;
        if (rst) begin
            tq_wp <= '0; tq_rp <= '0; l_busy <= 1'b0;
            rq_wp <= '0; rq_rp <= '0; cq_wp <= '0; cq_rp <= '0; dq_wp <= '0; dq_rp <= '0;
            inflight <= '0;
            have_last <= 1'b0;
            p_have_last <= 1'b0;
            a_have_last <= 1'b0;
            DDRAM_RD <= 1'b0;
            DDRAM_WE <= 1'b0;
            wbusy <= 1'b0;
        end else begin : b_wave_ddr_1
            logic issued, returned;
            issued = 1'b0;
            returned = 1'b0;

            // accept a byte request (at most one per clock; PCM wins,
            // then player, then archive)
            if (pcm_acc) begin
                rq[rq_wp[QBITS-1:0]] <= {2'd0, !same, addr[2:0]};
                rq_wp <= rq_wp + 1'b1;
                if (!same) begin
                    cq[cq_wp[QBITS-1:0]] <= word;
                    cq_wp <= cq_wp + 1'b1;
                end
                last_word <= word;
                have_last <= 1'b1;
            end else if (player_acc) begin
                rq[rq_wp[QBITS-1:0]] <= {2'd1, !p_same, paddr[2:0]};
                rq_wp <= rq_wp + 1'b1;
                if (!p_same) begin
                    cq[cq_wp[QBITS-1:0]] <= p_word;
                    cq_wp <= cq_wp + 1'b1;
                end
                p_last_word <= p_word;
                p_have_last <= 1'b1;
            end else if (areq && aready) begin
                rq[rq_wp[QBITS-1:0]] <= {2'd2, !a_same, aaddr[2:0]};
                rq_wp <= rq_wp + 1'b1;
                if (!a_same) begin
                    cq[cq_wp[QBITS-1:0]] <= a_word;
                    cq_wp <= cq_wp + 1'b1;
                end
                a_last_word <= a_word;
                a_have_last <= 1'b1;
            end

            // DDR3 command: a held command goes when the controller is not busy
            if (DDRAM_RD || DDRAM_WE) begin
                if (!DDRAM_BUSY) begin
                    if (DDRAM_RD) issued = 1'b1;
                    DDRAM_RD <= 1'b0;
                    DDRAM_WE <= 1'b0;
                    if (DDRAM_WE) wbusy <= 1'b0;
                end
            end else if (wr && !wbusy) begin
                DDRAM_WE <= 1'b1;
                DDRAM_ADDR <= BASE_W + {6'h0, waddr[25:3]};
                DDRAM_DIN <= {8{wdata}};
                DDRAM_BE <= 8'd1 << waddr[2:0];
                wbusy <= 1'b1;
                have_last <= 1'b0;        // contents change: no merging across a load
                p_have_last <= 1'b0;
                a_have_last <= 1'b0;
            end else if (lreq && !l_busy && !lvalid && inflight < (QBITS+1)'(Q - 1)) begin
                DDRAM_RD <= 1'b1;
                DDRAM_ADDR <= BASE_W + 29'h80000 + {14'h0, laddr};
                tq[tq_wp[QBITS-1:0]] <= 1'b1;
                tq_wp <= tq_wp + 1'b1;
                l_busy <= 1'b1;
            end else if (cq_wp != cq_rp) begin
                DDRAM_RD <= 1'b1;
                DDRAM_ADDR <= BASE_W + {6'h0, cq[cq_rp[QBITS-1:0]]};
                cq_rp <= cq_rp + 1'b1;
                tq[tq_wp[QBITS-1:0]] <= 1'b0;
                tq_wp <= tq_wp + 1'b1;
            end

            // data back, in order
            if (DDRAM_DOUT_READY) begin
                if (tq[tq_rp[QBITS-1:0]]) begin
                    ldata <= DDRAM_DOUT;
                    lvalid <= 1'b1;
                    l_busy <= 1'b0;
                end else begin
                    dq[dq_wp[QBITS-1:0]] <= DDRAM_DOUT;
                    dq_wp <= dq_wp + 1'b1;
                end
                tq_rp <= tq_rp + 1'b1;
                returned = 1'b1;
            end
            inflight <= inflight + (QBITS+1)'(issued) - (QBITS+1)'(returned);

            // bytes out, in request order, routed to the requesting port
            if (rq_wp != rq_rp) begin : b_wave_ddr_2
                logic [5:0] e;
                logic [63:0] w;
                e = rq[rq_rp[QBITS-1:0]];
                if (!e[3] || dq_wp != dq_rp) begin
                    w = (e[5:4] == 2'd2) ? a_cur :
                        (e[5:4] == 2'd1) ? p_cur : cur;
                    if (e[3]) begin
                        w = dq[dq_rp[QBITS-1:0]];
                        dq_rp <= dq_rp + 1'b1;
                        if (e[5:4] == 2'd2) a_cur <= w;
                        else if (e[5:4] == 2'd1) p_cur <= w;
                        else cur <= w;
                    end
                    if (e[5:4] == 2'd2) begin
                        ardata <= w[8 * e[2:0] +: 8];
                        arvalid <= 1'b1;
                    end else if (e[5:4] == 2'd1) begin
                        prdata <= w[8 * e[2:0] +: 8];
                        prvalid <= 1'b1;
                    end else begin
                        rdata <= w[8 * e[2:0] +: 8];   // little-endian: byte 0 in bits 7:0
                        rvalid <= 1'b1;
                    end
                    rq_rp <= rq_rp + 1'b1;
                end
            end
        end
    end
endmodule
