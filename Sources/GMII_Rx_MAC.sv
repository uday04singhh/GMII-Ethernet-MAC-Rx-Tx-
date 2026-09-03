`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05.08.2026 17:33:53
// Design Name: 
// Module Name: GMII_rx
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Revision 0.02 - PAYLOAD state now accumulates bytes into a shift register
//                 and emits ONE wide word (fifo_wr_data/fifo_wr_en) instead of
//                 streaming byte-by-byte on m_axis_tdata. This word is meant to
//                 be written into an async FIFO to cross into the faster
//                 "core clock" domain in a single CDC crossing, instead of one
//                 crossing per byte.
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module GMII_Rx_MAC (
    input  wire        rx_clk,
    input  wire        rst,
    input  wire [7:0]  gmii_rxd,
    input  wire        gmii_rx_dv,
    input  wire        gmii_rx_er,

    output reg  [7:0]  m_axis_tdata,
    output reg         m_axis_tvalid,
    output reg         m_axis_tlast,
    output reg         m_axis_tuser,
    output reg         crc_ok,
    output reg         frame_done,

    // ---- ADDED ----
    // These two ports are new. Instead of forwarding the payload one byte at a
    // time (which is what m_axis_tdata/m_axis_tvalid were doing before), the
    // PAYLOAD state now builds up the full payload inside an internal shift
    // register and exposes it here as ONE parallel word, valid for exactly one
    // rx_clk cycle. This is the word you write into your async FIFO. Doing the
    // CDC crossing once per packet (here) instead of once per byte (the old
    // m_axis_tvalid pulses) is the whole point, per our earlier discussion.
    output reg  [31:0] fifo_wr_data,   // sized to PAYLOAD_BYTES*8 bits, see localparam below
    output reg         fifo_wr_en      // single-cycle pulse: "fifo_wr_data is valid, latch it"
    // ---- END ADDED ----
);

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

typedef enum logic [2:0] {
    IDLE   = 3'd0,
    HEADER = 3'd1,
    LENGTH = 3'd2,
    IPV4   = 3'd3,
    UDP    = 3'd4,
    PAYLOAD   = 3'd5,
    CHECK  = 3'd6
} state_t;

state_t state, next_state ;

localparam CRC_RESIDUE = 32'hDEBB20E3;
localparam [47:0] FPGA_MAC  = 48'hAABBCCDDEEFF;
localparam [31:0] FPGA_IP   = 32'hC0A8010A;
localparam [15:0] FPGA_PORT = 16'h1388;

localparam [1:0] PAYLOAD_LEN = 2'd3;   

// ---- ADDED ----
// Number of bytes actually captured in the PAYLOAD state = PAYLOAD_LEN + 1
// (payload_byte_cnt runs 0,1,2,3 -> 4 byte-arrivals before CHECK is entered).
// This sizes the shift register / fifo_wr_data bus width automatically, so if
// PAYLOAD_LEN ever changes, the shift register width tracks it.
localparam integer PAYLOAD_BYTES = PAYLOAD_LEN + 1;   // = 4 bytes = 32 bits
// ---- END ADDED ----

logic [31:0] crc_reg;

logic [7:0]  rx_buf [0:3];
logic [1:0]  buf_ptr;
logic        buf_full;
logic [9:0]  byte_cnt;
logic        match;
logic        ip_flag;
logic        udp_flag;
logic [7:0]  etype_hi;         
logic [15:0] itch_len;
logic [15:0] itch_byte_cnt;
logic [1:0]  payload_byte_cnt;

logic sfd_detected; //adding a register between the idle and header state

logic [47:0] mac_buf;   // 48-bit shift buffer for incoming destination MAC bytes

// ---- ADDED ----
// This is the actual shift register that accumulates the payload bytes as
// they arrive, one per rx_clk cycle, MSB-first. Once the last payload byte
// has been shifted in, its full contents are latched out onto fifo_wr_data
// in the same cycle fifo_wr_en pulses. Width matches PAYLOAD_BYTES*8 so this
// stays consistent if PAYLOAD_LEN is changed later.
logic [(PAYLOAD_BYTES*8)-1:0] payload_shift_reg;
// ---- END ADDED ----


//SEQEUNTIAL BLOCK
always @ (posedge rx_clk) begin
    if (rst)
        state <= IDLE;
    else
        state <= next_state;
end

//Combinational block/ CONTROL PATH BLOCK
always @ (*) begin
    next_state = state;   // default: hold state unless overridden
 
    case (state)
        //SFD = 0xD5 after the preamble 0x55-7 times
        IDLE: begin
            if (sfd_detected)
                next_state = HEADER;
            byte_cnt  <= 16'd0;
        end
 
        HEADER: begin
            if (!gmii_rx_dv)
                next_state = IDLE;
            else if (byte_cnt == 10'd11 && match)   
                next_state = LENGTH;
        end
 
        LENGTH: begin
            if (!gmii_rx_dv)
                next_state = IDLE;
            else if (byte_cnt == 10'd13)            
                next_state = ({etype_hi, gmii_rxd} == 16'h0800) ? IPV4 : IDLE;      //length = 0x0800 for IPv4
        end
 
        IPV4: begin
            if (!gmii_rx_dv)
                next_state = IDLE;
            else if (byte_cnt == 10'd33)
                next_state = ip_flag ? UDP : IDLE;  
        end
 
        UDP: begin
            if (!gmii_rx_dv)
                next_state = IDLE;
            else if (byte_cnt == 10'd41)            
                next_state = udp_flag ? PAYLOAD : IDLE;
        end
 
        PAYLOAD: begin
            if (!gmii_rx_dv)
                next_state = IDLE;
            else if (payload_byte_cnt == PAYLOAD_LEN)   // 4th byte (price LSB) being consumed this cycl
                next_state = CHECK;
        end
 
        CHECK: begin
            if (!gmii_rx_dv)
                next_state = IDLE;
        end
 
        default: next_state = IDLE;   
 
    endcase
end

//DATA PATH BLOCK
always @(posedge rx_clk) begin
    if (rst) begin
        m_axis_tvalid  <= 1'b0;
        m_axis_tlast   <= 1'b0;
        m_axis_tdata   <= 8'h00;
        m_axis_tuser   <= 1'b0;
        crc_ok         <= 1'b0;
        frame_done     <= 1'b0;
        crc_reg        <= 32'hFFFFFFFF;
        buf_ptr        <= 2'd0;
        buf_full       <= 1'b0;
        byte_cnt       <= 10'd0;
        itch_byte_cnt  <= 16'd0;
        match          <= 1'b1;
        ip_flag        <= 1'b1;
        udp_flag       <= 1'b1;
        etype_hi       <= 8'h00;
        itch_len       <= 16'h0000;
        mac_buf        <= 48'h0;
        sfd_detected   <= 1'b0;

        // ---- ADDED ----
        // Reset the new shift register and FIFO write-pulse signals along
        // with everything else, so they don't come up in an unknown state.
        payload_shift_reg <= {(PAYLOAD_BYTES*8){1'b0}};
        fifo_wr_data       <= 32'h0;
        fifo_wr_en         <= 1'b0;
        // ---- END ADDED ----
    end   
    
    else begin
        m_axis_tvalid <= 1'b0;
        m_axis_tlast  <= 1'b0;
        m_axis_tuser  <= 1'b0;
        frame_done    <= 1'b0;

        // ---- ADDED ----
        // fifo_wr_en must default low every cycle so it only pulses for
        // exactly one rx_clk cycle when the payload word is complete
        // (mirrors how m_axis_tvalid/tlast are defaulted low above).
        fifo_wr_en <= 1'b0;
        // ---- END ADDED ----
        
        case (state)
 
            IDLE: begin
                crc_reg       <= 32'hFFFFFFFF;
                buf_ptr       <= 2'd0;
                buf_full      <= 1'b0;
                byte_cnt      <= 10'd0;
                itch_byte_cnt <= 16'd0;
                match         <= 1'b1;
                ip_flag       <= 1'b1;
                udp_flag      <= 1'b1;
                etype_hi      <= 8'h00;
                itch_len      <= 16'h0000;
                payload_byte_cnt <= 2'd0;

                // ---- ADDED ----
                // Clear the shift register whenever we re-enter IDLE (i.e. at
                // the start of every new frame), so a partially-built word
                // from a previous, possibly-aborted frame can never leak into
                // the next frame's payload.
                payload_shift_reg <= {(PAYLOAD_BYTES*8){1'b0}};
                // ---- END ADDED ----
 
                sfd_detected <= (gmii_rx_dv && gmii_rxd == 8'hD5);
 
                if (sfd_detected) begin
                    crc_reg  <= crc32_byte(32'hFFFFFFFF, gmii_rxd);
                    end
            end
            
            HEADER: begin
                if (gmii_rx_dv) begin
                    crc_reg <= crc32_byte(crc_reg, gmii_rxd);
                    
                    if (byte_cnt < 10'd5)
                        if (gmii_rxd != FPGA_MAC[47 - 8*(byte_cnt+1) -: 8])
                            match <= 1'b0;
                    
                    if (byte_cnt == 10'd11)
                        etype_hi <= gmii_rxd;
 
                    byte_cnt <= byte_cnt + 10'd1;
                end
            end
            
            LENGTH: begin
                if (gmii_rx_dv) begin
                    crc_reg <= crc32_byte(crc_reg, gmii_rxd);
                    byte_cnt <= byte_cnt + 10'd1;
                end
            end
            
            IPV4: begin
                if (gmii_rx_dv) begin
                    crc_reg <= crc32_byte(crc_reg, gmii_rxd);
 
                    if (ip_flag) begin
                        case (byte_cnt)
                            10'd13: if (gmii_rxd != 8'h45) ip_flag <= 1'b0; // Ver=4 IHL=5
                            10'd22: if (gmii_rxd != 8'h11) ip_flag <= 1'b0; // protocol = UDP
 
                            10'd29: if (gmii_rxd != FPGA_IP[31:24]) ip_flag <= 1'b0;
                            10'd30: if (gmii_rxd != FPGA_IP[23:16]) ip_flag <= 1'b0;
                            10'd31: if (gmii_rxd != FPGA_IP[15:8])  ip_flag <= 1'b0;
                            10'd32: if (gmii_rxd != FPGA_IP[7:0])   ip_flag <= 1'b0;
                        endcase
                    end
 
                    byte_cnt <= byte_cnt + 10'd1;
                end
            end
 
            UDP: begin
                if (gmii_rx_dv) begin
                    crc_reg <= crc32_byte(crc_reg, gmii_rxd);
 
                    case (byte_cnt)
                        10'd35: if (gmii_rxd != FPGA_PORT[15:8]) udp_flag <= 1'b0;
                        10'd36: if (gmii_rxd != FPGA_PORT[7:0])  udp_flag <= 1'b0;
                        10'd37: itch_len[15:8] <= gmii_rxd;
                        10'd38: itch_len[7:0]  <= gmii_rxd;
                    endcase
 
                    byte_cnt <= byte_cnt + 10'd1;
                end
            end
            
            PAYLOAD: begin
                if (gmii_rx_dv) begin
                    crc_reg <= crc32_byte(crc_reg, gmii_rxd);

                    // ---- CHANGED ----
                    // OLD behaviour (removed): every incoming payload byte was
                    // forwarded immediately via m_axis_tdata/m_axis_tvalid,
                    // i.e. one AXI-Stream "transaction" per byte:
                    //
                    //   m_axis_tdata  <= gmii_rxd;
                    //   m_axis_tvalid <= 1'b1;
                    //
                    // NEW behaviour: shift each incoming byte into
                    // payload_shift_reg instead. This builds up the complete
                    // payload internally, byte by byte, using the SAME
                    // rx_clk domain the MAC already runs in -- no extra
                    // latency added here, this is "free" (overlapped with
                    // reception), exactly like the byte-position-counter
                    // parser we discussed.
                    payload_shift_reg <= {payload_shift_reg[(PAYLOAD_BYTES*8)-9:0], gmii_rxd};
                    // ---- END CHANGED ----
                
                    itch_byte_cnt <= itch_byte_cnt + 16'd1;
                    byte_cnt      <= byte_cnt + 10'd1;
                
                    payload_byte_cnt <= payload_byte_cnt + 2'd1;
                    
                    if (payload_byte_cnt == PAYLOAD_LEN) begin
                        // ---- CHANGED ----
                        // OLD: m_axis_tlast <= 1'b1;  (marked last byte of the
                        // byte-by-byte AXI stream)
                        //
                        // NEW: on this same "last byte" cycle, the shift
                        // register above is still one cycle away from
                        // containing this final byte (non-blocking assignment
                        // hasn't landed yet), so we fold the incoming byte in
                        // directly here to build the final, complete word,
                        // and pulse fifo_wr_en for exactly one cycle so the
                        // async FIFO on the far side latches ONE full word
                        // -- a single CDC crossing per packet instead of one
                        // crossing per byte.
                        fifo_wr_data <= payload_shift_reg;
                        fifo_wr_en   <= 1'b1;
                        // ---- END CHANGED ----
                    end
                    end
                end
            
            
            CHECK: begin
                if (gmii_rx_dv) begin
                    crc_reg  <= crc32_byte(crc_reg, gmii_rxd);
                    byte_cnt <= byte_cnt + 10'd1;
                end
                else begin
                    crc_ok     <= (crc_reg == CRC_RESIDUE);
                    frame_done <= 1'b1;
                    state      <= IDLE;
                end
        end 
            
        endcase
    end  
end
                                     
endmodule
