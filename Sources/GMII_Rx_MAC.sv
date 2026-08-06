`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Create Date: 05.08.2026 17:33:53
// Module Name: GMII_rx
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
    output reg         frame_done
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
    end   
    
    else begin
        m_axis_tvalid <= 1'b0;
        m_axis_tlast  <= 1'b0;
        m_axis_tuser  <= 1'b0;
        frame_done    <= 1'b0;
        
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
                
                    // forward the payload byte straight to the AXI-Stream side
                    m_axis_tdata  <= gmii_rxd;
                    m_axis_tvalid <= 1'b1;
                
                    itch_byte_cnt <= itch_byte_cnt + 16'd1;
                    byte_cnt      <= byte_cnt + 10'd1;
                
                    payload_byte_cnt <= payload_byte_cnt + 2'd1;
                    
                    if (payload_byte_cnt == PAYLOAD_LEN)
                        m_axis_tlast <= 1'b1;
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








