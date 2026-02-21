// ============================================================
// Gaussian Blur (3x3) for DE1-SoC FPGA synthesis
// - frame_buffer loaded via .mif at synthesis time
// - blur_buffer output streamed to VGA controller
// ============================================================
`timescale 1ns/1ps

module gaussian_blur_rgb565_320x240 (
    input  wire        clk,
    input  wire        rst,
    input  wire        start,
    output reg         done,

    // VGA read port: driven by vga_controller
    input  wire [16:0] vga_addr,
    output wire [15:0] vga_pixel
);
    localparam integer W = 320;
    localparam integer H = 240;
    localparam integer N = W * H;  // 76800

    // -----------------------------------------------------------
    // Input frame buffer — initialized from output.mif at synthesis
    // -----------------------------------------------------------
    reg [15:0] frame_buffer [0:N-1];
    initial begin
        $readmemh("output.hex", frame_buffer);  // Quartus uses .mif; keep for sim fallback
    end

    // -----------------------------------------------------------
    // Output blur buffer — VGA controller reads from here
    // -----------------------------------------------------------
    reg [15:0] blur_buffer [0:N-1];

    // VGA reads blur_buffer asynchronously
    assign vga_pixel = blur_buffer[vga_addr];

    // -----------------------------------------------------------
    // Line buffers, window registers, counters — UNCHANGED
    // -----------------------------------------------------------
    reg [15:0] line1 [0:W-1];
    reg [15:0] line2 [0:W-1];

    reg [8:0]  x;
    reg [7:0]  y;
    reg [16:0] addr;
    reg        running;

    reg [8:0]  x_d;
    reg [7:0]  y_d;
    reg [16:0] addr_d;

    reg [15:0] p_in, p_mid, p_top, p_in_d;

    reg [15:0] r0_0, r0_1, r0_2;
    reg [15:0] r1_0, r1_1, r1_2;
    reg [15:0] r2_0, r2_1, r2_2;

    function automatic [4:0] R5(input [15:0] p); begin R5 = p[15:11]; end endfunction
    function automatic [5:0] G6(input [15:0] p); begin G6 = p[10:5];  end endfunction
    function automatic [4:0] B5(input [15:0] p); begin B5 = p[4:0];   end endfunction

    wire [9:0] sumR =
        R5(r0_0) + (R5(r0_1)<<1) + R5(r0_2) +
        (R5(r1_0)<<1) + (R5(r1_1)<<2) + (R5(r1_2)<<1) +
        R5(r2_0) + (R5(r2_1)<<1) + R5(r2_2);

    wire [9:0] sumG =
        G6(r0_0) + (G6(r0_1)<<1) + G6(r0_2) +
        (G6(r1_0)<<1) + (G6(r1_1)<<2) + (G6(r1_2)<<1) +
        G6(r2_0) + (G6(r2_1)<<1) + G6(r2_2);

    wire [9:0] sumB =
        B5(r0_0) + (B5(r0_1)<<1) + B5(r0_2) +
        (B5(r1_0)<<1) + (B5(r1_1)<<2) + (B5(r1_2)<<1) +
        B5(r2_0) + (B5(r2_1)<<1) + B5(r2_2);

    wire [4:0]  blurR = sumR[9:4];
    wire [5:0]  blurG = sumG[9:4];
    wire [4:0]  blurB = sumB[9:4];
    wire [15:0] blurred_pixel = {blurR, blurG, blurB};

    wire [8:0]  out_x    = x_d - 1;
    wire [7:0]  out_y    = y_d - 1;
    wire [16:0] out_addr = out_y * W + out_x;

    integer i;

    always @(posedge clk) begin
        if (rst) begin
            x<=0; y<=0; addr<=0;
            x_d<=0; y_d<=0; addr_d<=0;
            running<=0; done<=0;
            p_in<=0; p_mid<=0; p_top<=0; p_in_d<=0;
            r0_0<=0; r0_1<=0; r0_2<=0;
            r1_0<=0; r1_1<=0; r1_2<=0;
            r2_0<=0; r2_1<=0; r2_2<=0;
            for (i=0; i<W; i=i+1) begin
                line1[i]<=0; line2[i]<=0;
            end
        end else begin
            done <= 0;
            if (start && !running) begin
                running<=1; x<=0; y<=0; addr<=0;
                x_d<=0; y_d<=0; addr_d<=0;
            end
            if (running) begin
                p_in   <= frame_buffer[addr];
                p_mid  <= line1[x];
                p_top  <= line2[x];
                x_d    <= x;   y_d    <= y;   addr_d <= addr;
                p_in_d <= p_in;
                r0_0<=r0_1; r0_1<=r0_2; r0_2<=p_top;
                r1_0<=r1_1; r1_1<=r1_2; r1_2<=p_mid;
                r2_0<=r2_1; r2_1<=r2_2; r2_2<=p_in;
                line2[x] <= line1[x];
                line1[x] <= p_in;
                blur_buffer[addr_d] <= p_in_d;
                if (x_d >= 2 && y_d >= 2)
                    blur_buffer[out_addr] <= blurred_pixel;
                if (addr == N-1) begin
                    running<=0; done<=1;
                end else begin
                    addr <= addr + 1;
                    if (x == W-1) begin x<=0; y<=y+1; end
                    else x <= x+1;
                end
            end
        end
    end

endmodule
