`timescale 1ns / 1ps

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

state_t state;

localparam CRC_RESIDUE = 32'hC704DD7B;
localparam [47:0] FPGA_MAC  = 48'hAABBCCDDEEFF;
localparam [31:0] FPGA_IP   = 32'hC0A8010A;
localparam [15:0] FPGA_PORT = 16'h1388;

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

always @(posedge rx_clk) begin
    if (rst) begin
        state          <= IDLE;
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

            if (gmii_rx_dv && gmii_rxd == 8'hD5)
                state <= HEADER;
        end

        HEADER: begin
            if (!gmii_rx_dv) begin
                state <= IDLE;
            end
            else begin
                crc_reg  <= crc32_byte(crc_reg, gmii_rxd);

                if (byte_cnt < 10'd6) begin
                    if (gmii_rxd != FPGA_MAC[47 - 8*byte_cnt[2:0] -: 8])
                        match <= 1'b0;
                end

                if (byte_cnt == 10'd11) begin
                    if (match)
                        state    <= LENGTH;
                end

                byte_cnt <= byte_cnt + 10'd1;
            end
        end
        
        LENGTH: begin
            if (!gmii_rx_dv) begin
                state <= IDLE;
            end
            else begin
                crc_reg  <= crc32_byte(crc_reg, gmii_rxd);

                if (byte_cnt == 10'd12)
                    etype_hi <= gmii_rxd;           // latch high byte

                if (byte_cnt == 10'd13) begin
                    if ({etype_hi, gmii_rxd} == 16'h0800)
                        state <= IPV4;
                    else
                        state <= IDLE;
                end

                byte_cnt <= byte_cnt + 10'd1;
            end
        end

       
        IPV4: begin
            if (!gmii_rx_dv) begin
                state <= IDLE;
            end
            else begin
                crc_reg  <= crc32_byte(crc_reg, gmii_rxd);

                if (ip_flag) begin
                    case (byte_cnt)
                        10'd14: if (gmii_rxd != 8'h45)  ip_flag <= 1'b0; // Ver = 4 IHL = 5
                        10'd23: if (gmii_rxd != 8'h11)  ip_flag <= 1'b0; // protocol = UDP

                        // FIX 2: subtract base index 30 to get octet offset 0-3
                        10'd30: if (gmii_rxd != FPGA_IP[31:24]) ip_flag <= 1'b0;
                        10'd31: if (gmii_rxd != FPGA_IP[23:16]) ip_flag <= 1'b0;
                        10'd32: if (gmii_rxd != FPGA_IP[15:8])  ip_flag <= 1'b0;
                        10'd33: if (gmii_rxd != FPGA_IP[7:0])   ip_flag <= 1'b0;
                    endcase
                end

                if (byte_cnt == 10'd33) begin
                    if (ip_flag)
                        state <= UDP;
                    else
                        state <= IDLE;
                end

                byte_cnt <= byte_cnt + 10'd1;
            end
        end

        UDP: begin
            if (!gmii_rx_dv) begin
                state <= IDLE;
            end
            else begin
                crc_reg  <= crc32_byte(crc_reg, gmii_rxd);

                case (byte_cnt)
                    10'd36: if (gmii_rxd != FPGA_PORT[15:8]) udp_flag <= 1'b0;
                    10'd37: if (gmii_rxd != FPGA_PORT[7:0])  udp_flag <= 1'b0;
                    10'd38: itch_len[15:8] <= gmii_rxd;
                    10'd39: itch_len[7:0]  <= gmii_rxd;
                endcase

                if (byte_cnt == 10'd41) begin
                    if (udp_flag)
                        state <= PAYLOAD;
                    else
                        state <= IDLE;
                end

                byte_cnt <= byte_cnt + 10'd1;
            end
        end

        // -------------------------------------------------------------------
        // ITCH payload with 4-byte CRC lookahead
        //
        // FIX 4: buf_full is a sticky flag set once buf_ptr wraps to 0
        // after the first 4 writes (i.e. after writing index 3).
        // Output slot is always (buf_ptr + 2'd1) mod 4, which is the
        // oldest unread entry - correct because buf_ptr is incremented
        // BEFORE the read in the same cycle.
        //
        // The end condition compares itch_byte_cnt against the payload
        // length minus the 8-byte UDP header minus 4 CRC bytes minus 1
        // (because itch_byte_cnt is post-incremented).
        // -------------------------------------------------------------------
        PAYLOAD: begin
            if (!gmii_rx_dv) begin
                // Truncated frame
                m_axis_tlast  <= 1'b1;
                m_axis_tvalid <= buf_full;
                m_axis_tuser  <= 1'b1;
                state         <= IDLE;
            end
            else begin
                crc_reg <= crc32_byte(crc_reg, gmii_rxd);

                // Write incoming byte into the circular buffer
                rx_buf[buf_ptr] <= gmii_rxd;
                buf_ptr         <= buf_ptr + 2'd1;

                // Set buf_full once we've written all 4 slots (buf_ptr
                // just wrapped from 3 to 0 on the next cycle, but we
                // detect it here while buf_ptr is still 3).
                if (!buf_full && buf_ptr == 2'd3)
                    buf_full <= 1'b1;

                // Once the buffer is primed, the oldest byte is always
                // at (buf_ptr + 1) - the slot we are about to overwrite
                // next cycle, i.e. the furthest behind the write pointer.
                if (buf_full) begin
                    m_axis_tdata  <= rx_buf[buf_ptr + 2'd1]; // oldest slot
                    m_axis_tvalid <= 1'b1;

                    if (itch_byte_cnt == (itch_len - 16'd8 - 16'd4 - 16'd1)) begin
                        m_axis_tlast <= 1'b1;
                        state        <= CHECK;
                    end

                    itch_byte_cnt <= itch_byte_cnt + 16'd1;
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