// this module handles the hold and modify mode (HAM)
// the module has its own colour pallete bank, this is to let
// the sprites run simultaneously with a HAM playfield


module denise_hamgenerator
(
  input  wire           clk,              // 28MHz clock
  input  wire           clk7_en,          // 7MHz clock enable
  input  wire [  9-1:1] reg_address_in,   // register adress inputs
  input  wire [ 12-1:0] data_in,          // bus data in
  input  wire [  6-1:0] bpldata,          // bitplane data input
  output reg  [ 12-1:0] rgb               // RGB output
);

// register names and adresses
parameter COLORBASE = 9'h180;         // colour table base address

// local signals
reg   [ 12-1:0] colortable [0:16-1];  // colour table
reg   [ 12-1:0] selcolor;             // selected colour register output
reg   [ 12-1:0] rgb_prev;             // previous rgb value

// writing of HAM colour table from bus (implemented using dual port ram)
always @ (posedge clk) begin
  if (clk7_en) begin
    if (reg_address_in[8:5] == COLORBASE[8:5])
      colortable[reg_address_in[4:1]] <= #1 data_in[11:0];
  end
end

// reading of colour table
always @ (posedge clk) begin
  selcolor <= #1 colortable[bpldata[3:0]];
end

// register previous rgb value
always @ (posedge clk) begin
  rgb_prev <= #1 rgb;
end

// HAM instruction decoder/processor
always @ (*) begin
  case (bpldata[5:4])
    2'b00: // load rgb output with colour from table
      rgb = selcolor;
    2'b01: // hold green and red, modify blue
      rgb = {rgb_prev[11:4],bpldata[3:0]};
    2'b10: // hold green and blue, modify red
      rgb = {bpldata[3:0],rgb_prev[7:0]};
    2'b11: // hold blue and red, modify green
      rgb = {rgb_prev[11:8],bpldata[3:0],rgb_prev[3:0]};
    default:
      rgb = selcolor;
  endcase
end


endmodule

