module user_io( 

	input        clk_sys,
	input        SPI_CLK,
	input        SPI_SS_IO,
	output       reg SPI_MISO,
	input        SPI_MOSI,
	input  [7:0] CORE_TYPE,

	output [15:0] JOY0,
	output [15:0] JOY1,

	output [2:0] MOUSE_BUTTONS,
	output       KBD_MOUSE_STROBE,
	output       KMS_LEVEL,
	output [1:0] KBD_MOUSE_TYPE,
	output [7:0] KBD_MOUSE_DATA,

	output [1:0] BUTTONS,
	output [1:0] SWITCHES,
	output [3:0] CONF
);

reg [6:0]     sbuf;
reg [7:0]     cmd;
reg [2:0] 	  bit_cnt;    // counts bits 0-7 0-7 ...
reg [9:0]     byte_cnt;   // counts bytes
reg [7:0] 	  but_sw;

reg           kbd_mouse_strobe;
reg           kbd_mouse_strobe_level;
reg [1:0]     kbd_mouse_type;
reg [7:0]     kbd_mouse_data;
reg [2:0]     mouse_buttons;
reg [31:0]    joystick_0;
reg [31:0]    joystick_1;
reg [31:0]    joystick_2;
reg [31:0]    joystick_3;
reg [31:0]    joystick_4;

assign JOY0 = joystick_0[15:0];
assign JOY1 = joystick_1[15:0];

assign KBD_MOUSE_DATA = kbd_mouse_data; // 8 bit movement data
assign KBD_MOUSE_TYPE = kbd_mouse_type; // 0=mouse x,1=mouse y, 2=keycode, 3=OSD kbd
assign KBD_MOUSE_STROBE = kbd_mouse_strobe; // strobe, data valid on rising edge
assign KMS_LEVEL = kbd_mouse_strobe_level; // level change of kbd_mouse_strobe
assign MOUSE_BUTTONS = mouse_buttons; // state of the two mouse buttons

assign BUTTONS  = but_sw[1:0];
assign SWITCHES = but_sw[3:2];
assign CONF     = but_sw[7:4];

// SPI bit and byte counters
always@(posedge SPI_CLK or posedge SPI_SS_IO) begin
	if(SPI_SS_IO == 1) begin
		bit_cnt <= 0;
		byte_cnt <= 0;
	end else begin
		if((bit_cnt == 7)&&(~&byte_cnt)) 
			byte_cnt <= byte_cnt + 8'd1;

		bit_cnt <= bit_cnt + 1'd1;
	end
end

always@(negedge SPI_CLK or posedge SPI_SS_IO) begin
	if(SPI_SS_IO == 1) begin
	   SPI_MISO <= 1'bZ;
	end else begin
		SPI_MISO <= CORE_TYPE[~bit_cnt];
	end
end

// SPI receiver IO -> FPGA

reg       spi_receiver_strobe_r = 0;
reg       spi_transfer_end_r = 1;
reg [7:0] spi_byte_in;

// Read at spi_sck clock domain, assemble bytes for transferring to clk_sys
always@(posedge SPI_CLK or posedge SPI_SS_IO) begin

	if(SPI_SS_IO == 1) begin
		spi_transfer_end_r <= 1;
	end else begin
		spi_transfer_end_r <= 0;

		if(bit_cnt != 7)
			sbuf[6:0] <= { sbuf[5:0], SPI_MOSI };

		// finished reading a byte, prepare to transfer to clk_sys
		if(bit_cnt == 7) begin
			spi_byte_in <= { sbuf, SPI_MOSI};
			spi_receiver_strobe_r <= ~spi_receiver_strobe_r;
		end
	end
end

// Process bytes from SPI at the clk_sys domain
always @(posedge clk_sys) begin

	reg       spi_receiver_strobe;
	reg       spi_transfer_end;
	reg       spi_receiver_strobeD;
	reg       spi_transfer_endD;
	reg [7:0] acmd;
	reg [7:0] abyte_cnt;   // counts bytes

	//synchronize between SPI and sys clock domains
	spi_receiver_strobeD <= spi_receiver_strobe_r;
	spi_receiver_strobe <= spi_receiver_strobeD;
	spi_transfer_endD	<= spi_transfer_end_r;
	spi_transfer_end	<= spi_transfer_endD;

	// strobe is set whenever a valid byte has been received
	kbd_mouse_strobe <= 0;

	if (~spi_transfer_endD & spi_transfer_end) begin
		abyte_cnt <= 8'd0;
	end else if (spi_receiver_strobeD ^ spi_receiver_strobe) begin

		if(~&abyte_cnt) 
			abyte_cnt <= abyte_cnt + 8'd1;

		if(abyte_cnt == 0) begin
			acmd <= spi_byte_in;
			case (spi_byte_in)
				8'h04: kbd_mouse_type <= 2'b00;  // first mouse axis
				8'h05: kbd_mouse_type <= 2'b10;  // keyboard
		 		8'h06: kbd_mouse_type <= 2'b11;  // OSD keyboard
			endcase
		end else begin
			case(acmd)
				// buttons and switches
				8'h01: but_sw <= spi_byte_in;
				8'h60: if (abyte_cnt < 5) joystick_0[(abyte_cnt-1)<<3 +:8] <= spi_byte_in;
				8'h61: if (abyte_cnt < 5) joystick_1[(abyte_cnt-1)<<3 +:8] <= spi_byte_in;
				8'h62: if (abyte_cnt < 5) joystick_2[(abyte_cnt-1)<<3 +:8] <= spi_byte_in;
				8'h63: if (abyte_cnt < 5) joystick_3[(abyte_cnt-1)<<3 +:8] <= spi_byte_in;
				8'h64: if (abyte_cnt < 5) joystick_4[(abyte_cnt-1)<<3 +:8] <= spi_byte_in;
				8'h04:
				begin
					if (abyte_cnt == 8'd1) begin
						kbd_mouse_data <= spi_byte_in;
						kbd_mouse_strobe <= 1;
						kbd_mouse_strobe_level <= ~kbd_mouse_strobe_level;
					end else if (abyte_cnt == 8'd2) begin
						// second byte contains y movement data
						kbd_mouse_data <= spi_byte_in;
						kbd_mouse_strobe <= 1;
						kbd_mouse_strobe_level <= ~kbd_mouse_strobe_level;
						kbd_mouse_type <= 2'b01;
					end else begin
						// third byte contains the buttons
						mouse_buttons[2:0] <= spi_byte_in[2:0];
					end
				end
				8'h05, 8'h06:
				begin
					kbd_mouse_data <= spi_byte_in;
					kbd_mouse_strobe <= 1;
					kbd_mouse_strobe_level <= ~kbd_mouse_strobe_level;
				end
			endcase
		end
	end
end

endmodule

