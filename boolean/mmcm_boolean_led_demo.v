// SPDX-License-Identifier: MIT
// Simple LED-based demo to visualize MMCME2_ADV clocks on Boolean board.

`timescale 1ns / 1ps

module mmcm_boolean_led_demo (
    input  wire clk_in1,
    input  wire rst,
    output wire [15:0] led
);
  wire clk_out1;
  wire clk_out2;
  wire clk_out3;
  wire clk_out4;
  wire clk_out5;
  wire clk_out6;
  wire clk_out7;
  wire locked;

  clk_wiz_0_clk_wiz clk_wiz_i (
    .clk_in1 (clk_in1),
    .reset   (rst),
    .clk_out1(clk_out1), // 100 MHz
    .clk_out2(clk_out2), // 100 MHz
    .clk_out3(clk_out3), // 100 MHz
    .clk_out4(clk_out4), // 100 MHz
    .clk_out5(clk_out5), // 50 MHz
    .clk_out6(clk_out6), // 33.33 MHz
    .clk_out7(clk_out7), // 25 MHz
    .locked  (locked)
  );

  localparam integer COUNTER_WIDTH = 25;

  reg [COUNTER_WIDTH-1:0] cnt_clk1 = {COUNTER_WIDTH{1'b0}};
  reg [COUNTER_WIDTH-1:0] cnt_clk5 = {COUNTER_WIDTH{1'b0}};
  reg [COUNTER_WIDTH-1:0] cnt_clk6 = {COUNTER_WIDTH{1'b0}};
  reg [COUNTER_WIDTH-1:0] cnt_clk7 = {COUNTER_WIDTH{1'b0}};

  always @(posedge clk_out1) begin
    if (rst) cnt_clk1 <= {COUNTER_WIDTH{1'b0}};
    else cnt_clk1 <= cnt_clk1 + 1'b1;
  end

  always @(posedge clk_out5) begin
    if (rst) cnt_clk5 <= {COUNTER_WIDTH{1'b0}};
    else cnt_clk5 <= cnt_clk5 + 1'b1;
  end

  always @(posedge clk_out6) begin
    if (rst) cnt_clk6 <= {COUNTER_WIDTH{1'b0}};
    else cnt_clk6 <= cnt_clk6 + 1'b1;
  end

  always @(posedge clk_out7) begin
    if (rst) cnt_clk7 <= {COUNTER_WIDTH{1'b0}};
    else cnt_clk7 <= cnt_clk7 + 1'b1;
  end

  // frequency test
  assign led[0]  = cnt_clk1[COUNTER_WIDTH-1]; // fastest
  assign led[1]  = cnt_clk5[COUNTER_WIDTH-1]; // 1/2 slower
  assign led[2]  = cnt_clk6[COUNTER_WIDTH-1]; // 1/3 slower
  assign led[3]  = cnt_clk7[COUNTER_WIDTH-1]; // 1/4 slower
  // phase and duty cycle test
  assign led[4]  = clk_out1 & clk_out2; // faint light
  assign led[5]  = clk_out2; // 50%
  assign led[6]  = clk_out3; // faint light
  assign led[7]  = clk_out4; // 75% 
  // aux signals
  assign led[8]  = locked; // on during normal operation
  assign led[9]  = ~rst; // on during normal operation
  assign led[15:10] = 6'b0;

endmodule

