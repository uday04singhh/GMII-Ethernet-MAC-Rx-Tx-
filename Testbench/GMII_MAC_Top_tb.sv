`timescale 1ns / 1ps

module tb_GMII_Rx_MAC();

    reg rx_clk = 0;
    reg rst;
    reg [7:0] gmii_rxd;
    reg gmii_rx_dv;
    reg gmii_rx_er;

    wire [7:0] m_axis_tdata;
    wire m_axis_tvalid;
    wire m_axis_tlast;
    wire m_axis_tuser;
    wire crc_ok;
    wire frame_done;

    always #4 rx_clk = ~rx_clk;

    // Instantiate Unit Under Test (UUT)
    GMII_Rx_MAC uut (
        .rx_clk(rx_clk),
        .rst(rst),
        .gmii_rxd(gmii_rxd),
        .gmii_rx_dv(gmii_rx_dv),
        .gmii_rx_er(gmii_rx_er),
        .m_axis_tdata(m_axis_tdata),
        .m_axis_tvalid(m_axis_tvalid),
        .m_axis_tlast(m_axis_tlast),
        .m_axis_tuser(m_axis_tuser),
        .crc_ok(crc_ok),
        .frame_done(frame_done)
    );

    // Task to send a single byte
    task send_byte(input [7:0] data);
        begin
            @(posedge rx_clk);
            gmii_rxd = data;
            gmii_rx_dv = 1;
        end
    endtask

    // Main Simulation Process
    initial begin
        // Initialize
        rst = 1;
        gmii_rxd = 8'h00;
        gmii_rx_dv = 0;
        gmii_rx_er = 0;

        // Hold reset for 100ns
        #100;
        @(posedge rx_clk);
        rst = 0;
        #20;

        // --- TEST CASE 1: Valid ITCH Packet ---
        $display("Starting Test Case 1: Valid ITCH Packet");
        
        // 1. Preamble (7 bytes of 0x55)
        repeat(7) send_byte(8'h55);
        
        // 2. SFD (Start Frame Delimiter)
        send_byte(8'hD5);

        // 3. Destination MAC (AABBCCDDEEFF)
        send_byte(8'hAA); send_byte(8'hBB); send_byte(8'hCC); 
        send_byte(8'hDD); send_byte(8'hEE); send_byte(8'hFF);

        // 4. Source MAC (Just random bytes)
        repeat(6) send_byte(8'h12);

        // 5. EtherType (IPv4 = 0x0800)
        send_byte(8'h08); send_byte(8'h00);

        // 6. IPv4 Header (Minimum 20 bytes)
        send_byte(8'h45); // Version/IHL
        send_byte(8'h00); // DSCP
        send_byte(8'h00); send_byte(8'h2E); // Total Length (46 bytes)
        repeat(5) send_byte(8'h00); // ID, Flags, TTL
        send_byte(8'h11); // Protocol (UDP)
        send_byte(8'h00); send_byte(8'h00); // Checksum
        repeat(4) send_byte(8'h00); // Source IP
        send_byte(8'hC0); send_byte(8'hA8); send_byte(8'h01); send_byte(8'h0A); // Dest IP (192.168.1.10)

        // 7. UDP Header (8 bytes)
        send_byte(8'h00); send_byte(8'h00); // Source Port
        send_byte(8'h13); send_byte(8'h88); // Dest Port (5000)
        send_byte(8'h00); send_byte(8'h12); // Length (18 bytes = 8 hdr + 10 payload)
        send_byte(8'h00); send_byte(8'h00); // Checksum

        // 8. ITCH Payload (10 bytes)
        // Let's send a simulated 'A' (Add Order) message type
        send_byte(8'h41); // 'A'
        repeat(9) send_byte(8'hEE); // Mock data

        // 9. Frame Check Sequence (FCS / CRC-32)
        // NOTE: In a real test, you'd calculate this. 
        // For basic FSM testing, we send dummy bytes to trigger the transition.
        repeat(4) send_byte(8'hAA); 

        // End of Packet
        @(posedge rx_clk);
        gmii_rx_dv = 0;
        gmii_rxd = 8'h00;

        // Wait to see results
        repeat(10) @(posedge rx_clk);
        
        if (frame_done)
            $display("Packet processing complete. CRC_OK: %b", crc_ok);

        #100;
        $finish;
    end

endmodule
