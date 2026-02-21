// VGA 640x480 @ 60Hz controller for DE1-SoC
// Displays 320x240 image pixel-doubled (each pixel = 2x2 on screen)
module vga_controller (
    input  wire        clk_25mhz,   // 25 MHz pixel clock (use Quartus PLL)
    input  wire        rst,
    output reg         hsync,
    output reg         vsync,
    output wire [4:0]  vga_r,
    output wire [5:0]  vga_g,
    output wire [4:0]  vga_b,
    // Connect to blur_buffer in gs.v
    output wire [16:0] pixel_addr,
    input  wire [15:0] pixel_data
);
    // VGA 640x480 timing constants
    localparam H_VISIBLE = 640, H_FRONT = 16, H_SYNC = 96, H_BACK = 48;
    localparam V_VISIBLE = 480, V_FRONT = 10, V_SYNC = 2,  V_BACK = 33;
    localparam H_TOTAL = H_VISIBLE + H_FRONT + H_SYNC + H_BACK; // 800
    localparam V_TOTAL = V_VISIBLE + V_FRONT + V_SYNC + V_BACK; // 525

    reg [9:0] hcount = 0;
    reg [9:0] vcount = 0;

    always @(posedge clk_25mhz) begin
        if (rst) begin
            hcount <= 0; vcount <= 0;
        end else begin
            if (hcount == H_TOTAL - 1) begin
                hcount <= 0;
                if (vcount == V_TOTAL - 1) vcount <= 0;
                else vcount <= vcount + 1;
            end else hcount <= hcount + 1;
        end
    end

    wire active = (hcount < H_VISIBLE) && (vcount < V_VISIBLE);

    always @(posedge clk_25mhz) begin
        hsync <= ~(hcount >= H_VISIBLE + H_FRONT &&
                   hcount <  H_VISIBLE + H_FRONT + H_SYNC);
        vsync <= ~(vcount >= V_VISIBLE + V_FRONT &&
                   vcount <  V_VISIBLE + V_FRONT + V_SYNC);
    end

    // Pixel-double: map 640x480 → 320x240 by dividing coords by 2
    wire [8:0] img_x = hcount[9:1];  // hcount / 2
    wire [7:0] img_y = vcount[9:1];  // vcount / 2
    assign pixel_addr = img_y * 320 + img_x;

    assign vga_r = active ? pixel_data[15:11] : 5'b0;
    assign vga_g = active ? pixel_data[10:5]  : 6'b0;
    assign vga_b = active ? pixel_data[4:0]   : 5'b0;

endmodule
