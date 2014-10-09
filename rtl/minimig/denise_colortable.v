// this is the 32 colour colour table
// because this module also supports EHB (extra half brite) mode,
// it actually has a 6bit colour select input
// the 6th bit selects EHB colour while the lower 5 bit select the actual colour register

module denise_colortable
(
  input  wire           clk,              // 28MHz clock
  input  wire           clk7_en,          // 7MHz clock enable
  input  wire [  9-1:1] reg_address_in,   // register adress inputs
  input  wire [ 12-1:0] data_in,          // bus data in
  input  wire [  6-1:0] select,           // colour select input
  input  wire           a1k,              // EHB control
  output reg  [ 12-1:0] rgb               // RGB output
);

// register names and adresses
parameter COLORBASE = 9'h180;         // colour table base address

// local signals
reg   [ 12-1:0] colortable [0:32-1];  // colour table
reg   [ 12-1:0] selcolor;             // selected colour register output

// writing of colour table from bus (implemented using dual port ram)
always @ (posedge clk) begin
  if (clk7_en) begin
    if (reg_address_in[8:6] == COLORBASE[8:6])
      colortable[reg_address_in[5:1]] <= #1 data_in[11:0];
  end
end

// reading of colour table
always @ (posedge clk) begin
  selcolor <= #1 colortable[select[4:0]];
end

// extra half brite mode shifter
always @ (*) begin
  if (select[5] && !a1k) // half bright, shift every component 1 position to the right
    rgb <= {1'b0,selcolor[11:9],1'b0,selcolor[7:5],1'b0,selcolor[3:1]};
  else // normal colour select
    rgb <= selcolor;
end


endmodule

