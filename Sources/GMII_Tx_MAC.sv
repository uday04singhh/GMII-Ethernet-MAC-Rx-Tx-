`timescale 1ns / 1ps

module GMII_Tx_MAC (
    input  wire        tx_clk,
    input  wire        rst,

    // AXI-Stream slave — one ITCH payload per frame
    input  wire [7:0]  s_axis_tdata,
    input  wire        s_axis_tvalid,
    input  wire        s_axis_tlast,
    output reg         s_axis_tready,

    // GMII outputs
    output reg  [7:0]  gmii_txd,
    output reg         gmii_tx_en,
    output reg         gmii_tx_er,

    output reg         frame_done    // pulses 1 cycle when a frame finishes
);

// ---------------------------------------------------------------------------
// CRC-32 — identical polynomial / bit-order to the Rx side
// ---------------------------------------------------------------------------
function [31:0] crc32_byte;
    input [31:0] crc_in;
    input [7:0]  data;
    integer i;
    reg [31:0] crc;
    begin
        crc = crc_in ^ {24'h0, data};
        for (i = 0; i < 8; i = i + 1)
            crc = (crc[0]) ? (crc >> 1) ^ 32'hEDB88320 : (crc >> 1);
        crc32_byte = crc;
    end
endfunction

// ---------------------------------------------------------------------------
// State machine — same naming style as Rx
// ---------------------------------------------------------------------------
typedef enum logic [2:0] {
    IDLE     = 3'd0,
    PREAMBLE = 3'd1,
    HEADER   = 3'd2,
    IPV4     = 3'd3,
    UDP      = 3'd4,
    ITCH     = 3'd5,
    CRC      = 3'd6
} state_t;

state_t state;

// ---------------------------------------------------------------------------
// Same localparams as Rx — must stay in sync
// ---------------------------------------------------------------------------
localparam [47:0] FPGA_MAC  = 48'hAABBCCDDEEFF;
localparam [47:0] DST_MAC   = 48'hFFFFFFFFFFFF;   // broadcast; change to unicast as needed
localparam [31:0] FPGA_IP   = 32'hC0A8010A;
localparam [31:0] DST_IP    = 32'hC0A801FF;        // change to target
localparam [15:0] FPGA_PORT = 16'h1388;            // src UDP port  (same as Rx FPGA_PORT)
localparam [15:0] DST_PORT  = 16'h1388;            // dst UDP port

// IEEE 802.3 minimum inter-frame gap = 12 bytes
localparam [3:0] IFG_BYTES  = 4'd12;

// ---------------------------------------------------------------------------
// Internal signals
// ---------------------------------------------------------------------------
logic [31:0] crc_reg;
logic [7:0]  byte_cnt;          // counts byte currently being driven — mirrors Rx byte_cnt

// 4-byte staging buffer — same depth / ptr style as Rx rx_buf
logic [7:0]  tx_buf  [0:3];
logic [1:0]  buf_wr_ptr;
logic [1:0]  buf_rd_ptr;
logic        buf_full;          // sticky, set once all 4 slots written for first time

// Payload byte counter
logic [15:0] payload_len;       // total AXI bytes accepted for this frame (latched)
logic [15:0] payload_byte_cnt;  // bytes forwarded from ITCH state

// IFG drain counter
logic [3:0]  ifg_cnt;

// Latched length-dependent header fields (computed after payload_len known)
logic [15:0] udp_len;           // 8  + payload_len
logic [15:0] ip_len;            // 20 + udp_len
logic [15:0] ip_checksum;       // one's complement over fixed IPv4 header fields

// CRC byte selector (0-3 during CRC state)
logic [1:0]  crc_byte_sel;

// ---------------------------------------------------------------------------
// IP header checksum over the 20-byte IPv4 header (checksum field = 0x0000)
// Fields that are constant: ver/IHL=0x4500, ID=0x0000, flags/frag=0x4000,
//                           TTL/proto=0x4011, src IP, dst IP.
// Only the total-length field varies per frame.
// ---------------------------------------------------------------------------
function automatic [15:0] ip_hdr_checksum;
    input [15:0] ip_total_len;
    reg [31:0] s;
    begin
        s  = 32'h4500 + {16'd0, ip_total_len};  // ver/IHL/DSCP + total length
        s  = s + 32'h0000;                       // identification
        s  = s + 32'h4000;                       // flags=DF, frag offset=0
        s  = s + 32'h4011;                       // TTL=64, protocol=UDP
        // checksum field itself is 0x0000 (omitted from sum)
        s  = s + {16'd0, FPGA_IP[31:16]};
        s  = s + {16'd0, FPGA_IP[15:0]};
        s  = s + {16'd0, DST_IP[31:16]};
        s  = s + {16'd0, DST_IP[15:0]};
        // fold 32-bit sum into 16 bits (two passes covers all carry)
        s  = (s >> 16) + (s & 32'hFFFF);
        s  = (s >> 16) + (s & 32'hFFFF);
        ip_hdr_checksum = ~s[15:0];
    end
endfunction

// ---------------------------------------------------------------------------
// Header byte ROM — returns the correct byte for byte_cnt 0..41
// Mirrors the byte indices the Rx MAC parses:
//   0-5   DST MAC
//   6-11  SRC MAC
//  12-13  EtherType 0x0800
//  14     IP ver/IHL 0x45
//  15     DSCP/ECN   0x00
//  16-17  IP total length (dynamic)
//  18-19  ID         0x0000
//  20-21  Flags+Frag 0x4000
//  22     TTL        0x40
//  23     Protocol   0x11
//  24-25  IP checksum (dynamic)
//  26-29  Src IP
//  30-33  Dst IP
//  34-35  Src UDP port
//  36-37  Dst UDP port
//  38-39  UDP length (dynamic)
//  40-41  UDP checksum 0x0000
// ---------------------------------------------------------------------------
function automatic [7:0] header_byte;
    input [7:0]  idx;
    input [15:0] ip_l;
    input [15:0] udp_l;
    input [15:0] ip_cks;
    begin
        case (idx)
            8'd0:  header_byte = DST_MAC[47:40];
            8'd1:  header_byte = DST_MAC[39:32];
            8'd2:  header_byte = DST_MAC[31:24];
            8'd3:  header_byte = DST_MAC[23:16];
            8'd4:  header_byte = DST_MAC[15:8];
            8'd5:  header_byte = DST_MAC[7:0];
            8'd6:  header_byte = FPGA_MAC[47:40];
            8'd7:  header_byte = FPGA_MAC[39:32];
            8'd8:  header_byte = FPGA_MAC[31:24];
            8'd9:  header_byte = FPGA_MAC[23:16];
            8'd10: header_byte = FPGA_MAC[15:8];
            8'd11: header_byte = FPGA_MAC[7:0];
            8'd12: header_byte = 8'h08;
            8'd13: header_byte = 8'h00;
            8'd14: header_byte = 8'h45;
            8'd15: header_byte = 8'h00;
            8'd16: header_byte = ip_l[15:8];
            8'd17: header_byte = ip_l[7:0];
            8'd18: header_byte = 8'h00;
            8'd19: header_byte = 8'h00;
            8'd20: header_byte = 8'h40;
            8'd21: header_byte = 8'h00;
            8'd22: header_byte = 8'h40;
            8'd23: header_byte = 8'h11;
            8'd24: header_byte = ip_cks[15:8];
            8'd25: header_byte = ip_cks[7:0];
            8'd26: header_byte = FPGA_IP[31:24];
            8'd27: header_byte = FPGA_IP[23:16];
            8'd28: header_byte = FPGA_IP[15:8];
            8'd29: header_byte = FPGA_IP[7:0];
            8'd30: header_byte = DST_IP[31:24];
            8'd31: header_byte = DST_IP[23:16];
            8'd32: header_byte = DST_IP[15:8];
            8'd33: header_byte = DST_IP[7:0];
            8'd34: header_byte = FPGA_PORT[15:8];
            8'd35: header_byte = FPGA_PORT[7:0];
            8'd36: header_byte = DST_PORT[15:8];
            8'd37: header_byte = DST_PORT[7:0];
            8'd38: header_byte = udp_l[15:8];
            8'd39: header_byte = udp_l[7:0];
            8'd40: header_byte = 8'h00;    // UDP checksum disabled
            8'd41: header_byte = 8'h00;
            default: header_byte = 8'hxx;
        endcase
    end
endfunction

// ---------------------------------------------------------------------------
// Main FSM
// ---------------------------------------------------------------------------
always @(posedge tx_clk) begin
    if (rst) begin
        state             <= IDLE;
        gmii_tx_en        <= 1'b0;
        gmii_tx_er        <= 1'b0;
        gmii_txd          <= 8'h00;
        s_axis_tready     <= 1'b0;
        frame_done        <= 1'b0;
        crc_reg           <= 32'hFFFFFFFF;
        byte_cnt          <= 8'd0;
        buf_wr_ptr        <= 2'd0;
        buf_rd_ptr        <= 2'd0;
        buf_full          <= 1'b0;
        payload_len       <= 16'd0;
        payload_byte_cnt  <= 16'd0;
        ifg_cnt           <= 4'd0;
        udp_len           <= 16'd0;
        ip_len            <= 16'd0;
        ip_checksum       <= 16'd0;
        crc_byte_sel      <= 2'd0;
    end
    else begin
        // Default de-asserts — same pattern as Rx clearing tvalid/tlast every cycle
        gmii_tx_en    <= 1'b0;
        gmii_tx_er    <= 1'b0;
        gmii_txd      <= 8'h00;
        s_axis_tready <= 1'b0;
        frame_done    <= 1'b0;

        case (state)

        // -------------------------------------------------------------------
        // IDLE: drain IFG, then wait for AXI data.
        // Pre-fetch first AXI byte into staging buffer while still in IDLE
        // so PREAMBLE can start immediately without a stall.
        // Mirrors Rx IDLE which resets all flags and waits for SFD.
        // -------------------------------------------------------------------
        IDLE: begin
            crc_reg          <= 32'hFFFFFFFF;
            byte_cnt         <= 8'd0;
            buf_wr_ptr       <= 2'd0;
            buf_rd_ptr       <= 2'd0;
            buf_full         <= 1'b0;
            payload_len      <= 16'd0;
            payload_byte_cnt <= 16'd0;
            crc_byte_sel     <= 2'd0;

            if (ifg_cnt != 4'd0) begin
                ifg_cnt <= ifg_cnt - 4'd1;
            end
            else if (s_axis_tvalid) begin
                // Accept first byte into slot 0 of staging buffer
                s_axis_tready      <= 1'b1;
                tx_buf[2'd0]       <= s_axis_tdata;
                buf_wr_ptr         <= 2'd1;
                payload_len        <= 16'd1;
                state              <= PREAMBLE;
            end
        end

        // -------------------------------------------------------------------
        // PREAMBLE: emit 7x 0x55 then SFD 0xD5.
        // byte_cnt 0-6 = preamble bytes, 7 = SFD.
        // Continue accepting AXI bytes into staging buffer during preamble
        // so HEADER has data ready — mirrors Rx buf pre-fill concept.
        // CRC does NOT cover preamble/SFD (same as Rx which starts CRC
        // after the SFD in HEADER state).
        // -------------------------------------------------------------------
        PREAMBLE: begin
            gmii_tx_en <= 1'b1;

            // Keep draining AXI into staging buffer while slots are free
            if (s_axis_tvalid && !buf_full) begin
                s_axis_tready          <= 1'b1;
                tx_buf[buf_wr_ptr]     <= s_axis_tdata;
                buf_wr_ptr             <= buf_wr_ptr + 2'd1;
                payload_len            <= payload_len + 16'd1;
                if ((buf_wr_ptr + 2'd1) == buf_rd_ptr)
                    buf_full <= 1'b1;
            end

            if (byte_cnt < 8'd7) begin
                gmii_txd <= 8'h55;
                byte_cnt <= byte_cnt + 8'd1;
            end
            else begin
                // SFD
                gmii_txd <= 8'hD5;

                // Latch all length-dependent fields now that payload_len
                // is stable — same moment Rx exits preamble and enters HEADER
                udp_len     <= payload_len + 16'd8;
                ip_len      <= payload_len + 16'd28;
                ip_checksum <= ip_hdr_checksum(payload_len + 16'd28);

                byte_cnt <= 8'd0;
                state    <= HEADER;
            end
        end

        // -------------------------------------------------------------------
        // HEADER: emit bytes 0-41 (ETH + IPv4 + UDP headers).
        // byte_cnt counts the header byte currently on the wire — mirrors
        // exactly how Rx byte_cnt counts the byte currently on gmii_rxd.
        // CRC accumulates from byte 0, same as Rx starting CRC in HEADER.
        // Continue pre-fetching AXI payload into staging buffer.
        // -------------------------------------------------------------------
        HEADER: begin
            gmii_tx_en <= 1'b1;

            // Continue accepting AXI payload into staging buffer
            if (s_axis_tvalid && !buf_full) begin
                s_axis_tready          <= 1'b1;
                tx_buf[buf_wr_ptr]     <= s_axis_tdata;
                buf_wr_ptr             <= buf_wr_ptr + 2'd1;
                payload_len            <= payload_len + 16'd1;
                if ((buf_wr_ptr + 2'd1) == buf_rd_ptr)
                    buf_full <= 1'b1;
            end

            begin : drive_hdr
                reg [7:0] hbyte;
                hbyte    = header_byte(byte_cnt, ip_len, udp_len, ip_checksum);
                gmii_txd <= hbyte;
                crc_reg  <= crc32_byte(crc_reg, hbyte);
            end

            if (byte_cnt == 8'd41) begin
                byte_cnt <= 8'd0;
                state    <= ITCH;
            end
            else
                byte_cnt <= byte_cnt + 8'd1;
        end

        // -------------------------------------------------------------------
        // ITCH: stream payload from staging buffer; refill from AXI.
        //
        // Read pointer chases write pointer with 0-gap (no lookahead needed
        // on Tx — we are producing bytes, not stripping a CRC tail).
        // payload_byte_cnt tracks how many bytes have been driven; when it
        // reaches payload_len the last byte is on the wire and we move to CRC.
        //
        // Mirrors Rx ITCH structure: buf_full gate before first output,
        // buf_ptr (here split into rd/wr) advancing each cycle.
        // -------------------------------------------------------------------
        ITCH: begin
            gmii_tx_en <= 1'b1;

            // Refill staging buffer from AXI when space is available
            if (s_axis_tvalid && ((buf_wr_ptr + 2'd1) != buf_rd_ptr)) begin
                s_axis_tready      <= 1'b1;
                tx_buf[buf_wr_ptr] <= s_axis_tdata;
                buf_wr_ptr         <= buf_wr_ptr + 2'd1;
            end

            // Drive oldest byte from staging buffer whenever data is present
            if (buf_wr_ptr != buf_rd_ptr) begin
                reg [7:0] pbyte;
                pbyte            = tx_buf[buf_rd_ptr];
                gmii_txd         <= pbyte;
                crc_reg          <= crc32_byte(crc_reg, pbyte);
                buf_rd_ptr       <= buf_rd_ptr + 2'd1;
                payload_byte_cnt <= payload_byte_cnt + 16'd1;

                if (payload_byte_cnt == (payload_len - 16'd1)) begin
                    crc_byte_sel <= 2'd0;
                    state        <= CRC;
                end
            end
        end

        // -------------------------------------------------------------------
        // CRC: append 4 FCS bytes, little-endian, bit-inverted.
        // Ethernet FCS = ~crc_reg emitted LSB-first (byte 0 = bits 7:0).
        // Mirrors Rx CHECK which absorbs exactly 4 bytes then checks residue.
        // -------------------------------------------------------------------
        CRC: begin
            gmii_tx_en <= 1'b1;

            case (crc_byte_sel)
                2'd0: gmii_txd <= ~crc_reg[7:0];
                2'd1: gmii_txd <= ~crc_reg[15:8];
                2'd2: gmii_txd <= ~crc_reg[23:16];
                2'd3: gmii_txd <= ~crc_reg[31:24];
            endcase

            if (crc_byte_sel == 2'd3) begin
                gmii_tx_en <= 1'b0;         // de-assert with last byte
                frame_done <= 1'b1;
                ifg_cnt    <= IFG_BYTES - 4'd1;
                state      <= IDLE;
            end
            else
                crc_byte_sel <= crc_byte_sel + 2'd1;
        end

        endcase
    end
end

endmodule
