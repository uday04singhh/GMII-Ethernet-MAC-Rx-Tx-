`timescale 1ns / 1ps
//  GMII_MAC_Top.sv
//  Top-level wrapper that wires GMII_Rx_MAC 

module GMII_MAC_Top #(
    parameter FIFO_DEPTH = 2048          
)(
    input  wire        rx_clk,
    input  wire        tx_clk,
    input  wire        rst,             

    //GMII RX (from PHY)
    input  wire [7:0]  gmii_rxd,
    input  wire        gmii_rx_dv,
    input  wire        gmii_rx_er,

    //GMII TX (to PHY)
    output wire [7:0]  gmii_txd,
    output wire        gmii_tx_en,
    output wire        gmii_tx_er,

    input  wire [7:0]  s_axis_tdata,
    input  wire        s_axis_tvalid,
    input  wire        s_axis_tlast,
    output wire        s_axis_tready,

    input  wire        loopback_en,      

    output wire        rx_frame_done,
    output wire        rx_crc_ok,
    output wire        tx_frame_done,
    output wire        fifo_overflow,

    output wire [$clog2(FIFO_DEPTH):0] fifo_level
);


wire [7:0]  rx_m_tdata;
wire        rx_m_tvalid;
wire        rx_m_tlast;
wire        rx_m_tuser;   // set when frame is truncated/errored


GMII_Rx_MAC u_rx_mac (
    .rx_clk        (rx_clk),
    .rst           (rst),
    .gmii_rxd      (gmii_rxd),
    .gmii_rx_dv    (gmii_rx_dv),
    .gmii_rx_er    (gmii_rx_er),
    .m_axis_tdata  (rx_m_tdata),
    .m_axis_tvalid (rx_m_tvalid),
    .m_axis_tlast  (rx_m_tlast),
    .m_axis_tuser  (rx_m_tuser),
    .crc_ok        (rx_crc_ok),
    .frame_done    (rx_frame_done)
);


wire        fifo_wr_en   = rx_m_tvalid;
wire [8:0]  fifo_din     = {rx_m_tlast, rx_m_tdata};  
wire        fifo_full;

wire        fifo_rd_en;
wire [8:0]  fifo_dout;
wire        fifo_empty;

reg         fifo_overflow_r;
assign fifo_overflow = fifo_overflow_r;

always @(posedge rx_clk) begin
    if (rst)
        fifo_overflow_r <= 1'b0;
    else if (fifo_wr_en && fifo_full)
        fifo_overflow_r <= 1'b1;   
end

// ──────────────────────────────────────────────────────────────────────────
//  Async FIFO — behavioural model (replace with vendor prim in real design)
//  Uses Gray-code pointers for CDC safety.
// ──────────────────────────────────────────────────────────────────────────
localparam ADDR_W = $clog2(FIFO_DEPTH);

// Storage
reg [8:0] mem [0:FIFO_DEPTH-1];

// Binary & Gray write pointer (rx_clk)
reg [ADDR_W:0] wr_bin, wr_gray;
// Binary & Gray read  pointer (tx_clk)
reg [ADDR_W:0] rd_bin, rd_gray;

// Synchronised copies (2FF)
reg [ADDR_W:0] wr_gray_s1_txclk, wr_gray_sync_txclk;
reg [ADDR_W:0] rd_gray_s1_rxclk, rd_gray_sync_rxclk;

// Write side
always @(posedge rx_clk) begin
    if (rst) begin
        wr_bin  <= '0;
        wr_gray <= '0;
    end else if (fifo_wr_en && !fifo_full) begin
        mem[wr_bin[ADDR_W-1:0]] <= fifo_din;
        wr_bin                  <= wr_bin + 1'b1;
        wr_gray                 <= (wr_bin + 1'b1) ^ ((wr_bin + 1'b1) >> 1);
    end
end

// Read side
always @(posedge tx_clk) begin
    if (rst) begin
        rd_bin  <= '0;
        rd_gray <= '0;
    end else if (fifo_rd_en && !fifo_empty) begin
        rd_bin  <= rd_bin + 1'b1;
        rd_gray <= (rd_bin + 1'b1) ^ ((rd_bin + 1'b1) >> 1);
    end
end

assign fifo_dout = mem[rd_bin[ADDR_W-1:0]];

// 2-FF synchronisers
always @(posedge tx_clk) begin
    wr_gray_s1_txclk   <= wr_gray;
    wr_gray_sync_txclk <= wr_gray_s1_txclk;
end
always @(posedge rx_clk) begin
    rd_gray_s1_rxclk   <= rd_gray;
    rd_gray_sync_rxclk <= rd_gray_s1_rxclk;
end

// Full / empty flags
assign fifo_empty = (wr_gray_sync_txclk == rd_gray);
assign fifo_full  = (wr_gray == {~rd_gray_sync_rxclk[ADDR_W:ADDR_W-1],
                                   rd_gray_sync_rxclk[ADDR_W-2:0]});

// Occupancy (tx_clk domain, approximate due to CDC)
wire [ADDR_W:0] wr_bin_tx;
genvar gi;
generate
    for (gi = ADDR_W; gi >= 0; gi = gi - 1) begin : gray2bin_wr
        if (gi == ADDR_W)
            assign wr_bin_tx[gi] = wr_gray_sync_txclk[gi];
        else
            assign wr_bin_tx[gi] = wr_bin_tx[gi+1] ^ wr_gray_sync_txclk[gi];
    end
endgenerate
assign fifo_level = wr_bin_tx - rd_bin;

// ============================================================================
//  Handshake bridge: FIFO read → Tx AXI-Stream
//
//  The FIFO data port has no tready on the source side (Rx MAC).
//  We add a skid buffer here so the Tx MAC can assert tready low
//  without dropping bytes.
// ============================================================================

// Skid buffer registers (tx_clk domain)
reg [8:0]  skid_data;
reg        skid_valid;

// FIFO-side signals driven by the skid logic
wire        bridge_tvalid;
wire [7:0]  bridge_tdata;
wire        bridge_tlast;
wire        bridge_tready;   // from mux/Tx MAC

// Pop from FIFO into skid buffer when skid is empty or being consumed
assign fifo_rd_en = !fifo_empty && (!skid_valid || bridge_tready);

always @(posedge tx_clk) begin
    if (rst) begin
        skid_valid <= 1'b0;
        skid_data  <= 9'h0;
    end else begin
        if (fifo_rd_en) begin
            skid_data  <= fifo_dout;
            skid_valid <= 1'b1;
        end else if (bridge_tready) begin
            skid_valid <= 1'b0;
        end
    end
end

assign bridge_tvalid = skid_valid;
assign bridge_tdata  = skid_data[7:0];
assign bridge_tlast  = skid_data[8];

// ============================================================================
//  AXI-Stream mux: loopback vs. external source
// ============================================================================
wire [7:0]  tx_s_tdata;
wire        tx_s_tvalid;
wire        tx_s_tlast;
wire        tx_s_tready;

assign tx_s_tdata  = loopback_en ? bridge_tdata  : s_axis_tdata;
assign tx_s_tvalid = loopback_en ? bridge_tvalid : s_axis_tvalid;
assign tx_s_tlast  = loopback_en ? bridge_tlast  : s_axis_tlast;

// tready back-routing
assign bridge_tready = loopback_en & tx_s_tready;
assign s_axis_tready = (~loopback_en) & tx_s_tready;

// ============================================================================
//  Tx MAC instantiation
// ============================================================================
GMII_Tx_MAC u_tx_mac (
    .tx_clk        (tx_clk),
    .rst           (rst),
    .s_axis_tdata  (tx_s_tdata),
    .s_axis_tvalid (tx_s_tvalid),
    .s_axis_tlast  (tx_s_tlast),
    .s_axis_tready (tx_s_tready),
    .gmii_txd      (gmii_txd),
    .gmii_tx_en    (gmii_tx_en),
    .gmii_tx_er    (gmii_tx_er),
    .frame_done    (tx_frame_done)
);

endmodule
