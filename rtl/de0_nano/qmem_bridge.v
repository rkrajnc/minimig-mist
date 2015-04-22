/********************************************/
/* qmem_bridge.v                            */
/* QMEM 32-to-32 bit async bridge           */
/*                                          */
/* 2013, rok.krajnc@gmail.com               */
/* 2015, Stefan Kristiansson                */
/********************************************/


module qmem_bridge #(
  parameter MAW = 22,
  parameter MSW = 4,
  parameter MDW = 32,
  parameter SAW = 22,
  parameter SSW = 2,
  parameter SDW = 16
)(
  // master
  input  wire           m_clk,
  input  wire [MAW-1:0] m_adr,
  input  wire           m_cs,
  input  wire           m_we,
  input  wire [MSW-1:0] m_sel,
  input  wire [MDW-1:0] m_dat_w,
  output reg  [MDW-1:0] m_dat_r,
  output reg            m_ack = 1'b0,
  output wire           m_err,
  // slave
  input  wire           s_clk,
  output reg  [SAW-1:0] s_adr,
  output reg            s_cs,
  output reg            s_we,
  output reg  [SSW-1:0] s_sel,
  output reg  [SDW-1:0] s_dat_w,
  input  wire [SDW-1:0] s_dat_r,
  input  wire           s_ack,
  input  wire           s_err
);


// sync master cs
reg [  3-1:0] cs_sync = 3'b000;
always @ (posedge s_clk) cs_sync <= {cs_sync[1:0], m_cs};

// detect master cs posedge
wire cs_posedge;
assign cs_posedge = cs_sync[1] && !cs_sync[2];

// latch master data
reg  [MAW-1:0] adr_d = {MAW{1'b0}};
reg            we_d = 1'b0;
reg  [MSW-1:0] sel_d = {MSW{1'b0}};
reg  [MDW-1:0] dat_w_d = {MDW{1'b0}};
always @ (posedge s_clk) begin
  if (cs_sync[1]) begin
    adr_d   <= m_adr;
    we_d    <= m_we;
    sel_d   <= m_sel;
    dat_w_d <= m_dat_w;
  end
end

// output state machine
reg  [  3-1:0] state = 3'b000;
localparam ST_IDLE     = 3'b000;
localparam ST_SETUP  = 3'b010;
localparam ST_WAIT   = 3'b011;
localparam ST_A_WAIT   = 3'b111;
reg  [  2-1:0] s_ack_sync = 2'b00;
reg done = 1'b0;
always @ (posedge s_clk) begin
  case (state)
    ST_IDLE : begin
      if (cs_sync[2] & !s_ack_sync[1]) begin
        state <= ST_SETUP;
      end
    end
    ST_SETUP : begin
      s_cs    <= 1'b1;
      s_adr   <= {adr_d[SAW-1:2], 1'b0, 1'b0};
      s_sel   <= sel_d;
      s_we    <= we_d;
      s_dat_w <= dat_w_d;
      state   <= ST_WAIT;
    end
    ST_WAIT : begin
      if (s_ack) begin
        s_cs    <= 1'b0;
        m_dat_r <= s_dat_r;
        done    <= 1'b1;
        state   <= ST_A_WAIT;
      end
    end
    ST_A_WAIT : begin
      if (s_ack_sync[1]) begin
        done  <= 1'b0;
        state <= ST_IDLE;
      end
    end
  endcase
end

// master ack
reg  [  3-1:0] m_ack_sync = 3'b000;
always @ (posedge m_clk) begin
  m_ack_sync <= {m_ack_sync[1:0], done};
end
wire m_ack_posedge;
assign m_ack_posedge = m_ack_sync[1] && !m_ack_sync[2];
always @ (posedge m_clk) begin
  if (m_ack_posedge) m_ack <= 1'b1;
  else if (m_ack) m_ack <= 1'b0;
end
always @ (posedge s_clk) begin
  s_ack_sync <= {s_ack_sync[0], m_ack_sync[2]};
end

// master err
assign m_err = 1'b0;


endmodule
