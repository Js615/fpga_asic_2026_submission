module top (
    input  wire       CLOCK_50,    // DE1-SoC 50MHz clock
    input  wire [1:0] KEY,         // KEY[0]=reset, KEY[1]=start
    output wire       VGA_HS,
    output wire       VGA_VS,
    output wire [3:0] VGA_R,
    output wire [3:0] VGA_G,
    output wire [3:0] VGA_B
);
    wire clk_25mhz;
    wire done;
    wire [16:0] pixel_addr;
    wire [15:0] pixel_data;

    // PLL: divide 50MHz → 25MHz for VGA
    // Generate this in Quartus IP Catalog → ALTPLL
    pll vga_pll (
        .inclk0(CLOCK_50),
        .c0(clk_25mhz)
    );

    wire rst   = ~KEY[0];
    wire start = ~KEY[1];

    gaussian_blur_rgb565_320x240 blur (
        .clk(CLOCK_50),
        .rst(rst),
        .start(start),
        .done(done),
        .vga_addr(pixel_addr),
        .vga_pixel(pixel_data)
    );

    vga_controller vga (
        .clk_25mhz(clk_25mhz),
        .rst(rst),
        .hsync(VGA_HS),
        .vsync(VGA_VS),
        .vga_r(VGA_R[3:0]),   // DE1-SoC VGA is 4-bit per channel
        .vga_g(VGA_G[3:0]),
        .vga_b(VGA_B[3:0]),
        .pixel_addr(pixel_addr),
        .pixel_data(pixel_data)
    );

endmodule
