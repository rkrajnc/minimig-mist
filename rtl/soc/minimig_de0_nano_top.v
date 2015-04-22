/********************************************/
/* minimig_de0_nan0_top.v                   */
/* Altera DE0 Nano FPGA Top File            */
/*                                          */
/* 2012, rok.krajnc@gmail.com               */
/* 2015, Stefan Kristiansson                */
/********************************************/


`define MINIMIG_DE0_NANO
//`define SOC_SIM

`include "minimig_defines.vh"


module minimig_de0_nano_top (
  // clock inputs
  input                 CLOCK_50,     // 50 MHz
  // USB JTAG Link
/*
  input                 TDI,          // CPLD -> FPGA (data in)
  input                 TCK,          // CPLD -> FPGA (clk)
  input                 TCS,          // CPLD -> FPGA (CS)
  output                TDO,          // FPGA -> CPLD (data out)
*/
  // push button inputs
  input       [    1:0] KEY,          // Pushbutton[1:0]
  // switch inputs
  input       [    3:0] SW,           // Toggle Switch[3:0]
  // LED outputs
  output      [    7:0] LEDG,         // LED Green[7:0]
  // UART
  output                UART_TXD,     // UART Transmitter
  input                 UART_RXD,     // UART Receiver
  // I2C
/*
  inout                 I2C_SDAT,     // I2C Data
  output                I2C_SCLK,     // I2C Clock
*/
  // PS2
  inout                 PS2_DAT,      // PS2 Keyboard Data
  inout                 PS2_CLK,      // PS2 Keyboard Clock
  inout                 PS2_MDAT,     // PS2 Mouse Data
  inout                 PS2_MCLK,     // PS2 Mouse Clock
  // VGA
  output                VGA_RST,      // VGA Reset (active low.)
  output                VGA_PCLK,     // VGA Pixel Clock
  output                VGA_DEN,      // VGA DATA EN
  output                VGA_HS,       // VGA H_SYNC
  output                VGA_VS,       // VGA V_SYNC
  output      [  7:0]   VGA_R,        // VGA Red[7:0]
  output      [  7:0]   VGA_G,        // VGA Green[7:0]
  output      [  7:0]   VGA_B,        // VGA Blue[7:0]
  // Audio CODEC
/*
  inout                 AUD_ADCLRCK,  // Audio CODEC ADC LR Clock
  input                 AUD_ADCDAT,   // Audio CODEC ADC Data
  inout                 AUD_DACLRCK,  // Audio CODEC DAC LR Clock
  output                AUD_DACDAT,   // Audio CODEC DAC Data
  inout                 AUD_BCLK,     // Audio CODEC Bit-Stream Clock
  output                AUD_XCK,      // Audio CODEC Chip Clock
 */
  // SD Card
  input                 SD_DAT,       // SD Card Data            - spi MISO
  output                SD_DAT3,      // SD Card Data 3          - spi CS
  output                SD_CMD,       // SD Card Command Signal  - spi MOSI
  output                SD_CLK,       // SD Card Clock           - spi CLK
  // SDRAM
  inout       [ 15:0]   DRAM_DQ,      // SDRAM Data bus 16 Bits
  output      [ 12:0]   DRAM_ADDR,    // SDRAM Address bus 12 Bits
  output                DRAM_LDQM,    // SDRAM Low-byte Data Mask
  output                DRAM_UDQM,    // SDRAM High-byte Data Mask
  output                DRAM_WE_N,    // SDRAM Write Enable
  output                DRAM_CAS_N,   // SDRAM Column Address Strobe
  output                DRAM_RAS_N,   // SDRAM Row Address Strobe
  output                DRAM_CS_N,    // SDRAM Chip Select
  output                DRAM_BA_0,    // SDRAM Bank Address 0
  output                DRAM_BA_1,    // SDRAM Bank Address 1
  output                DRAM_CLK,     // SDRAM Clock
  output                DRAM_CKE,     // SDRAM Clock Enable
/*
  // FLASH
  inout       [  8-1:0] FL_DQ,        // FLASH Data bus 8 Bits
  output      [ 22-1:0] FL_ADDR,      // FLASH Address bus 22 Bits
  output                FL_WE_N,      // FLASH Write Enable
  output                FL_RST_N,     // FLASH Reset
  output                FL_OE_N,      // FLASH Output Enable
  output                FL_CE_N,      // FLASH Chip Enable
 */
  // MINIMIG specific
  input       [  6-1:0] Joya,         // joystick port A
  input       [  6-1:0] Joyb,         // joystick port B
  output                AUDIOLEFT,    // sigma-delta DAC output left
  output                AUDIORIGHT    // sigma-delta DAC output right
);



////////////////////////////////////////
// internal signals                   //
////////////////////////////////////////

// clock
wire           pll_in_clk;
wire           clk_114;
wire           clk_28;
wire           clk_sdram;
wire           pll_locked;
wire           clk_7;
wire           clk7_en;
wire           c1;
wire           c3;
wire           cck;
wire [ 10-1:0] eclk;
wire           clk_50;

// reset
wire           pll_rst;
wire           sdctl_rst;
wire           rst_50;
wire           rst_minimig;
wire           rst_cpu;

// ctrl
wire           rom_status;
wire           ram_status;
wire           reg_status;
wire           dram_status;
wire           ctrl_txd;
wire           ctrl_rxd;
wire [ 23-1:0] dram_adr;
wire           dram_cs;
wire           dram_we;
wire [  4-1:0] dram_sel;
wire [ 32-1:0] dram_dat_w;
wire [ 32-1:0] dram_dat_r;
wire           dram_ack;
wire           dram_err;

// bridge
wire [ 23-1:0] bridge_adr;
wire           bridge_cs;
wire           bridge_we;
wire [  4-1:0] bridge_sel;
wire [ 32-1:0] bridge_dat_w;
wire [ 32-1:0] bridge_dat_r;
wire           bridge_ack;
wire           bridge_err;

// tg68
wire           tg68_rst;
wire [ 16-1:0] tg68_dat_in;
wire [ 16-1:0] tg68_dat_out;
wire [ 32-1:0] tg68_adr;
wire [  3-1:0] tg68_IPL;
wire           tg68_dtack;
wire           tg68_as;
wire           tg68_uds;
wire           tg68_lds;
wire           tg68_rw;
wire           tg68_ena7RD;
wire           tg68_ena7WR;
wire           tg68_enaWR;
wire [ 16-1:0] tg68_cout;
wire           tg68_cpuena;
wire [  4-1:0] cpu_config;
wire [  6-1:0] memcfg;
wire           vsync;
wire [ 32-1:0] tg68_cad;
wire [  6-1:0] tg68_cpustate;
wire           tg68_cdma;
wire           tg68_clds;
wire           tg68_cuds;
wire [ 32-1:0] tg68_VBR_out;

// minimig
wire           minimig_rst_out;
wire [ 16-1:0] ram_data;      // sram data bus
wire [ 16-1:0] ramdata_in;    // sram data bus in
wire [ 22-1:1] ram_address;   // sram address bus
wire           _ram_bhe;      // sram upper byte select
wire           _ram_ble;      // sram lower byte select
wire           _ram_we;       // sram write enable
wire           _ram_oe;       // sram output enable
wire           _15khz;        // scandoubler disable
wire           sdo;           // SPI data output
wire [ 15-1:0] ldata;         // left DAC data
wire [ 15-1:0] rdata;         // right DAC data
wire           audio_left;
wire           audio_right;
wire           floppy_fwr;
wire           floppy_frd;
wire           hd_fwr;
wire           hd_frd;
wire           minimig_txd;
wire           minimig_rxd;

// host bus
wire           host_cs;
wire [ 24-1:0] host_adr;
wire           host_we;
wire [  2-1:0] host_bs;
wire [ 16-1:0] host_wdat;
wire [ 16-1:0] host_rdat;
wire           host_ack;

// sdram
wire           reset_out;
wire [  4-1:0] sdram_cs;
wire [  2-1:0] sdram_dqm;
wire [  2-1:0] sdram_ba;

// audio
wire           audio_lr_switch;
wire           audio_lr_mix;

// ctrl
wire [  8-1:0] FL_DAT_W;
wire [  8-1:0] FL_DAT_R;
wire [  4-1:0] SPI_CS_N;
wire           SPI_DI;
wire           rst_ext;
wire [  4-1:0] ctrl_cfg;
wire [  4-1:0] ctrl_status;
wire [  4-1:0] sys_status;

// indicators
wire [  8-1:0] track;

// uart
wire           uart_sel;



////////////////////////////////////////
// input synchronizers                //
////////////////////////////////////////

wire   sw_3, sw_2, sw_1, sw_0;
wire   key_3, key_2, key_1, key_0;

i_sync #(.DW(4)) i_sync_sw_28 (
  .clk  (clk_28),
  .i    ({SW[3], SW[2], SW[1], SW[0]}),
  .o    ({sw_3, sw_2,  sw_1,  sw_0})
);

i_sync #(.DW(2)) i_sync_key_28 (
  .clk  (clk_28),
  .i    ({KEY[1], KEY[0]}),
  .o    ({key_1, key_0})
);

i_sync #(.DW(4)) i_sync_ctrl_50 (
  .clk  (clk_50),
  .i    ({vsync, ~tg68_rst, minimig_rst_out, ~reset_out}),
  .o    (sys_status)
);

// temp
/*
wire [3-1:0] cctrl;
i_sync #(.DW(3)) i_sync_key_28_tmp (
  .clk  (clk_28),
  .i    ({SW[3], SW[2], SW[1]}),
  .o    (cctrl)
);
*/
// temp


////////////////////////////////////////
// toplevel assignments               //
////////////////////////////////////////

// assign unused outputs
assign TDO              = 1'b1;

// UART
// uart_sel = 0 => minimig, uart_sel = 1 => or1k
assign uart_sel         = sw_0;
assign UART_TXD         = uart_sel ? ctrl_txd : minimig_txd;
assign ctrl_rxd         = uart_sel ? UART_RXD : 1'b1;
assign minimig_rxd      = uart_sel ? 1'b1     : UART_RXD;

// SD card
assign SD_DAT3          = SPI_CS_N[0];

// SDRAM
assign DRAM_CKE         = 1'b1;
assign DRAM_CLK         = clk_sdram;
assign DRAM_CS_N        = sdram_cs[0];
assign DRAM_LDQM        = sdram_dqm[0];
assign DRAM_UDQM        = sdram_dqm[1];
assign DRAM_BA_0        = sdram_ba[0];
assign DRAM_BA_1        = sdram_ba[1];

// FLASH
assign FL_DQ            = FL_OE_N   ? FL_DAT_W   : 8'bzzzzzzzz;
assign FL_DAT_R         = FL_DQ;

// AUDIO
`ifdef MINIMIG_SERIAL_AUDIO
assign AUDIOLEFT        = audio_lr_switch ? audio_right : audio_left;
assign AUDIORIGHT       = audio_lr_switch ? audio_left : audio_right;
`else
assign AUDIOLEFT        = 1'b0;
assign AUDIORIGHT       = 1'b0;
`endif

// ctrl
assign SPI_DI           = !SPI_CS_N[0] ? SD_DAT : sdo;
assign rst_ext          = !KEY[0];
assign ctrl_cfg         = {1'b0, 1'b0, 1'b0, 1'b0};

// clock
assign pll_in_clk       = CLOCK_50;

// reset
assign pll_rst          = !KEY[0];
assign sdctl_rst        = pll_locked & KEY[0];

// audio
assign audio_lr_switch  = sw_2;
assign audio_lr_mix     = sw_3;

// minimig (VGA scandoubler)
assign _15khz           = sw_1;



////////////////////////////////////////
// modules                            //
////////////////////////////////////////

//// control block ////
ctrl_top ctrl_top (
  // system
  .clk_in       (CLOCK_50         ),  // input 50MHz clock
  .rst_ext      (rst_ext          ),  // external reset input
  .clk_out      (clk_50           ),  // output 50MHz clock from internal PLL
  .rst_out      (rst_50           ),  // reset output from internal reset generator
  .rst_minimig  (rst_minimig      ),  // minimig reset output
  .rst_cpu      (rst_cpu          ),  // TG68K reset output
  // config
  .boot_sel     (1'b0             ),  // select FLASH boot location
  .ctrl_cfg     (ctrl_cfg         ),  // config for ctrl module
  // status
  .rom_status   (rom_status       ),  // ROM slave activity
  .reg_status   (reg_status       ),  // REG slave activity
  .dram_status  (dram_status      ),  // DRAM slave activity
  .ctrl_status  (ctrl_status      ),  // CTRL LEDs
  .sys_status   (sys_status       ),  // SYS status input
  // FLASH interface
  .fl_adr       (FL_ADDR          ),  // FLASH address output
  .fl_ce_n      (FL_CE_N          ),  // FLASH chip enable output
  .fl_we_n      (FL_WE_N          ),  // FLASH write enable output
  .fl_oe_n      (FL_OE_N          ),  // FLASH output enable
  .fl_rst_n     (FL_RST_N         ),  // FLASH reset
  .fl_dat_w     (FL_DAT_W         ),  // FLASH write data
  .fl_dat_r     (FL_DAT_R         ),  // FLASH read data
  // DRAM interface
  .dram_adr     (dram_adr         ),
  .dram_cs      (dram_cs          ),
  .dram_we      (dram_we          ),
  .dram_sel     (dram_sel         ),
  .dram_dat_w   (dram_dat_w       ),
  .dram_dat_r   (dram_dat_r       ),
  .dram_ack     (dram_ack         ),
  .dram_err     (dram_err         ),
  // UART
  .uart_txd     (ctrl_txd         ),  // UART transmit output
  .uart_rxd     (ctrl_rxd         ),  // UART receive input
  // SPI
  .spi_cs_n     (SPI_CS_N         ),  // SPI chip select output
  .spi_clk      (SD_CLK           ),  // SPI clock
  .spi_do       (SD_CMD           ),  // SPI data input
  .spi_di       (SPI_DI           )   // SPI data output
);


//// qmem async 32-to-32 bridge ////
`define CTRL_SDRAM_BRIDGE
`ifdef CTRL_SDRAM_BRIDGE
qmem_bridge #(
  .MAW (23),
  .MSW (4 ),
  .MDW (32),
  .SAW (23),
  .SSW (4 ),
  .SDW (32)
) qmem_bridge (
  // master
  .m_clk        (clk_50           ),
  .m_adr        (dram_adr         ),
  .m_cs         (dram_cs          ),
  .m_we         (dram_we          ),
  .m_sel        (dram_sel         ),
  .m_dat_w      (dram_dat_w       ),
  .m_dat_r      (dram_dat_r       ),
  .m_ack        (dram_ack         ),
  .m_err        (dram_err         ),
  // slave
  .s_clk        (clk_114          ),
  .s_adr        (bridge_adr       ),
  .s_cs         (bridge_cs        ),
  .s_we         (bridge_we        ),
  .s_sel        (bridge_sel       ),
  .s_dat_w      (bridge_dat_w     ),
  .s_dat_r      (bridge_dat_r     ),
  .s_ack        (bridge_ack       ),
  .s_err        (bridge_err       )
);
`else
assign dram_ack = 1'b1;
assign dram_err = 1'b0;
assign bridge_adr = 23'hxxxxxx;
assign bridge_cs = 1'b0;
assign bridge_we = 1'bx;
assign bridge_sel = 4'bxx;
assign bridge_dat_w = 32'hxxxx;
`endif // CTRL_SDRAM_BRIDGE


//// indicators ////
wire fifo_full;

indicators indicators(
  .clk          (clk_7            ),
  .rst          (~pll_locked      ),
  .track        (track            ),
  .f_wr         (floppy_fwr       ),
  .f_rd         (floppy_frd       ),
  .h_wr         (hd_fwr           ),
  .h_rd         (hd_frd           ),
  .status       ({rom_status, ram_status, reg_status, dram_status}),
  .ctrl_status  (ctrl_status      ),
  .sys_status   (sys_status       ),
  .fifo_full    (fifo_full),
  .led_g        (LEDG             )
);


//// amiga clocks ////
amiga_clk amiga_clk (
  .rst          (pll_rst          ), // async reset input
  .clk_in       (pll_in_clk       ), // input clock     ( 50.000000MHz)
  .clk_114      (clk_114          ), // output clock c0 (114.285714MHz)
  .clk_sdram    (clk_sdram        ), // output clock c2 (114.285714MHz, -146.25 deg)
  .clk_28       (clk_28           ), // output clock c1 ( 28.571428MHz)
  .clk_7        (clk_7            ), // output clock 7  (  7.142857MHz)
  .clk7_en      (clk7_en          ), // output clock 7 enable (on 28MHz clock domain)
  .c1           (c1               ), // clk28m clock domain signal synchronous with clk signal
  .c3           (c3               ), // clk28m clock domain signal synchronous with clk signal delayed by 90 degrees
  .cck          (cck              ), // colour clock output (3.54 MHz)
  .eclk         (eclk             ), // 0.709379 MHz clock enable output (clk domain pulse)
  .locked       (pll_locked       )  // pll locked output
);


//// audio ////
audio_top audio_top (
  .clk          (clk_28           ),  // 28MHz input clock
  .rst_n        (reset_out        ),  // active low reset (from sdram controller)
  // config
  .exchan       (audio_lr_switch  ),  // switch audio left / right channel
  .mix          (audio_lr_mix     ),  // normal / centered mix (play some left channel on the right channel and vise-versa)
  // audio shifter
  .rdata        (rdata            ),  // right channel sample data
  .ldata        (ldata            ),  // left channel sample data
  .aud_bclk     (AUD_BCLK         ),  // CODEC data clock
  .aud_daclrck  (AUD_DACLRCK      ),  // CODEC data clock
  .aud_dacdat   (AUD_DACDAT       ),  // CODEC data
  .aud_xck      (AUD_XCK          ),  // CODEC data clock
  // I2C audio config
  .i2c_sclk     (I2C_SCLK         ),  // CODEC config clock
  .i2c_sdat     (I2C_SDAT         )   // CODEC config data
);


//// TG68K main CPU ////
TG68K tg68k (
  .clk          (clk_114          ),
  .reset        (tg68_rst         ),
  .clkena_in    (1'b1             ),
  .IPL          (tg68_IPL         ),
  .dtack        (tg68_dtack       ),
  .vpa          (1'b1             ),
  .ein          (1'b1             ),
  .addr         (tg68_adr         ),
  .data_read    (tg68_dat_in      ),
  .data_write   (tg68_dat_out     ),
  .as           (tg68_as          ),
  .uds          (tg68_uds         ),
  .lds          (tg68_lds         ),
  .rw           (tg68_rw          ),
  .e            (                 ),
  .vma          (                 ),
  .wrd          (                 ),
  .ena7RDreg    (tg68_ena7RD      ),
  .ena7WRreg    (tg68_ena7WR      ),
  .enaWRreg     (tg68_enaWR       ),
  .fromram      (tg68_cout        ),
  .ramready     (tg68_cpuena      ),
  .cpu          (cpu_config[1:0]  ),
//  .fastkick     (cctrl[1]/*cpu_config[3]*/    ),
  .memcfg       (memcfg           ),
  .ramaddr      (tg68_cad         ),
  .cpustate     (tg68_cpustate    ),
  .nResetOut    (                 ),
  .skipFetch    (                 ),
  .cpuDMA       (tg68_cdma        ),
  .ramlds       (tg68_clds        ),
  .ramuds       (tg68_cuds        ),
  .VBR_out      (tg68_VBR_out     )
);

/*
//// TG68 main CPU ////
TG68 tg68 (
  .clk          (clk_114          ),
  .reset        (tg68_rst         ),
  .clkena_in    (1'b1             ),
  .data_in      (tg68_dat_in      ),
  .data_out     (tg68_dat_out     ),
  .IPL          (tg68_IPL         ),
  .dtack        (tg68_dtack       ),
  .addr         (tg68_adr         ),
  .as           (tg68_as          ),
  .uds          (tg68_uds         ),
  .lds          (tg68_lds         ),
  .rw           (tg68_rw          ),
  .drive_data   (                 ),
  .enaRDreg     (tg68_ena7RD      ),
  .enaWRreg     (tg68_ena7WR      )
);
*/

//// sdram ////
sdram_ctrl sdram (
  // sys
  .sysclk       (clk_114          ),
  .c_7m         (clk_7            ),
  .reset_in     (sdctl_rst        ),
  .cache_rst    (tg68_rst         ),
  .reset_out    (reset_out        ),
  .cache_ena    (1/*cctrl[0]*/    ),
  // sdram
  .sdaddr       (DRAM_ADDR        ),
  .sd_cs        (sdram_cs         ),
  .ba           (sdram_ba         ),
  .sd_we        (DRAM_WE_N        ),
  .sd_ras       (DRAM_RAS_N       ),
  .sd_cas       (DRAM_CAS_N       ),
  .dqm          (sdram_dqm        ),
  .sdata        (DRAM_DQ          ),
  // host
  .host_cs      (bridge_cs        ),
  .host_adr     ({~bridge_adr[22], 2'b00, bridge_adr[21:0]}),
  .host_we      (bridge_we        ),
  .host_bs      (bridge_sel       ),
  .host_wdat    (bridge_dat_w     ),
  .host_rdat    (bridge_dat_r     ),
  .host_ack     (bridge_ack       ),
  // chip
  .chipAddr     ({2'b00, ram_address[21:1]}),
  .chipL        (_ram_ble         ),
  .chipU        (_ram_bhe         ),
  .chipRW       (_ram_we          ),
  .chip_dma     (_ram_oe          ),
  .chipWR       (ram_data         ),
  .chipRD       (ramdata_in       ),
  // cpu
  .cpuAddr      (tg68_cad[24:1]   ),
  .cpustate     (tg68_cpustate    ),
  .cpuL         (tg68_clds        ),
  .cpuU         (tg68_cuds        ),
  .cpu_dma      (tg68_cdma        ),
  .cpuWR        (tg68_dat_out     ),
  .cpuRD        (tg68_cout        ),
  .enaWRreg     (tg68_enaWR       ),
  .ena7RDreg    (tg68_ena7RD      ),
  .ena7WRreg    (tg68_ena7WR      ),
  .cpuena       (tg68_cpuena      )
);

assign VGA_PCLK = clk_28;
assign VGA_RST = 1; // Active low
assign VGA_DEN = VGA_HS & VGA_VS;

//// minimig top ////
Minimig1 minimig (
  //m68k pins
  .cpu_address  (tg68_adr[23:1]   ), // M68K address bus
  .cpu_data     (tg68_dat_in      ), // M68K data bus
  .cpudata_in   (tg68_dat_out     ), // M68K data in
  ._cpu_ipl     (tg68_IPL         ), // M68K interrupt request
  ._cpu_as      (tg68_as          ), // M68K address strobe
  ._cpu_uds     (tg68_uds         ), // M68K upper data strobe
  ._cpu_lds     (tg68_lds         ), // M68K lower data strobe
  .cpu_r_w      (tg68_rw          ), // M68K read / write
  ._cpu_dtack   (tg68_dtack       ), // M68K data acknowledge
  ._cpu_reset   (tg68_rst         ), // M68K reset
  .cpu_clk      (clk_7            ), // M68K clock
  .cpu_vbr      (tg68_VBR_out     ), // M68K VBR
  //sram pins
  .ram_data     (ram_data         ), // SRAM data bus
  .ramdata_in   (ramdata_in       ), // SRAM data bus in
  .ram_address  (ram_address[21:1]), // SRAM address bus
  ._ram_bhe     (_ram_bhe         ), // SRAM upper byte select
  ._ram_ble     (_ram_ble         ), // SRAM lower byte select
  ._ram_we      (_ram_we          ), // SRAM write enable
  ._ram_oe      (_ram_oe          ), // SRAM output enable
  //system  pins
  .rst_ext      (rst_minimig      ), // reset from ctrl block
  .rst_out      (minimig_rst_out  ), // minimig reset status
  .clk28m       (clk_28           ), // output clock c1 ( 28.687500MHz)
  .clk          (clk_7            ), // output clock 7  (  7.171875MHz)
  .clk7_en      (clk7_en          ), // 7MHz clock enable
  .c1           (c1               ), // clk28m clock domain signal synchronous with clk signal
  .c3           (c3               ), // clk28m clock domain signal synchronous with clk signal delayed by 90 degrees
  .cck          (cck              ), // colour clock output (3.54 MHz)
  .eclk         (eclk             ), // 0.709379 MHz clock enable output (clk domain pulse)
  //rs232 pins
  .rxd          (minimig_rxd      ), // RS232 receive
  .txd          (minimig_txd      ), // RS232 send
  .cts          (1'b0             ), // RS232 clear to send
  .rts          (                 ), // RS232 request to send
  //I/O
  ._joy1        (Joya             ), // joystick 1 [fire2,fire,up,down,left,right] (default mouse port)
  ._joy2        (Joyb             ), // joystick 2 [fire2,fire,up,down,left,right] (default joystick port)
  .mouse_btn1   (1'b1             ), // mouse button 1
  .mouse_btn2   (1'b1             ), // mouse button 2
  ._15khz       (_15khz           ), // scandoubler disable
  .msdat        (PS2_MDAT         ), // PS2 mouse data
  .msclk        (PS2_MCLK         ), // PS2 mouse clk
  .kbddat       (PS2_DAT          ), // PS2 keyboard data
  .kbdclk       (PS2_CLK          ), // PS2 keyboard clk
  //host controller interface (SPI)
  ._scs         (SPI_CS_N[3:1]    ), // SPI chip select
  .direct_sdi   (SD_DAT           ), // SD Card direct in
  .sdi          (SD_CMD           ), // SPI data input
  .sdo          (sdo              ), // SPI data output
  .sck          (SD_CLK           ), // SPI clock
  //video
  ._hsync       (VGA_HS           ), // horizontal sync
  ._vsync       (VGA_VS           ), // vertical sync
  .red          (VGA_R[7:4]       ), // red
  .green        (VGA_G[7:4]       ), // green
  .blue         (VGA_B[7:4]       ), // blue
  //audio
  .left         (audio_left       ), // audio bitstream left
  .right        (audio_right      ), // audio bitstream right
  .ldata        (ldata            ), // left DAC data
  .rdata        (rdata            ), // right DAC data
  //user i/o
  .cpu_config   (cpu_config       ), // CPU config
  .memcfg       (memcfg           ), // memory config
  .init_b       (vsync            ), // vertical sync for MCU (sync OSD update)
  .fifo_full    (fifo_full        ),
  // fifo / track display
  .trackdisp    (track            ), // floppy track number
  .secdisp      (                 ), // sector
  .floppy_fwr   (floppy_fwr       ), // floppy fifo writing
  .floppy_frd   (floppy_frd       ), // floppy fifo reading
  .hd_fwr       (hd_fwr           ), // hd fifo writing
  .hd_frd       (hd_frd           )  // hd fifo  ading
);


endmodule
