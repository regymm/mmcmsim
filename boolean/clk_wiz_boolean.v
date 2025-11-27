// SPDX-License-Identifier: MIT
// MMCME2_ADV direct instantiation example

`timescale 1ns / 1ps

module clk_wiz_0_clk_wiz (
    input  wire clk_in1,
    input  wire reset,
    output wire clk_out1,
    output wire clk_out2,
    output wire clk_out3,
    output wire clk_out4,
    output wire clk_out5,
    output wire clk_out6,
    output wire clk_out7,
    output wire locked
);
  wire clk_in1_clk_wiz_0;

  IBUF clkin1_ibufg (
    .O (clk_in1_clk_wiz_0),
    .I (clk_in1)
  );

  wire clkfbout_clk_wiz_0;
  wire clkfbout_buf_clk_wiz_0;

  wire clk_out1_clk_wiz_0;
  wire clk_out2_clk_wiz_0;
  wire clk_out3_clk_wiz_0;
  wire clk_out4_clk_wiz_0;
  wire clk_out5_clk_wiz_0;
  wire clk_out6_clk_wiz_0;
  wire clk_out7_clk_wiz_0;

  wire locked_int;
  wire reset_high = reset;

  MMCME2_ADV #(
    .BANDWIDTH            ("OPTIMIZED"),
    .CLKOUT4_CASCADE      ("FALSE"),
    .COMPENSATION         ("ZHOLD"),
    .STARTUP_WAIT         ("FALSE"),
    .DIVCLK_DIVIDE        (1),
    .CLKFBOUT_MULT_F      (10.000),
    .CLKFBOUT_PHASE       (0.000),
    .CLKFBOUT_USE_FINE_PS ("FALSE"),
    .CLKOUT0_DIVIDE_F     (10.000),
    .CLKOUT0_PHASE        (0.000),
    .CLKOUT0_DUTY_CYCLE   (0.500),
    .CLKOUT0_USE_FINE_PS  ("FALSE"),
    .CLKOUT1_DIVIDE       (10),
    .CLKOUT1_PHASE        (90.000),
    .CLKOUT1_DUTY_CYCLE   (0.500),
    .CLKOUT1_USE_FINE_PS  ("FALSE"),
    .CLKOUT2_DIVIDE       (10),
    .CLKOUT2_PHASE        (180.000),
    .CLKOUT2_DUTY_CYCLE   (0.25),
    .CLKOUT2_USE_FINE_PS  ("FALSE"),
    .CLKOUT3_DIVIDE       (10),
    .CLKOUT3_PHASE        (270.000),
    .CLKOUT3_DUTY_CYCLE   (0.75),
    .CLKOUT3_USE_FINE_PS  ("FALSE"),
    .CLKOUT4_DIVIDE       (20),
    .CLKOUT4_PHASE        (0.000),
    .CLKOUT4_DUTY_CYCLE   (0.500),
    .CLKOUT4_USE_FINE_PS  ("FALSE"),
    .CLKOUT5_DIVIDE       (30),
    .CLKOUT5_PHASE        (0.000),
    .CLKOUT5_DUTY_CYCLE   (0.500),
    .CLKOUT5_USE_FINE_PS  ("FALSE"),
    .CLKOUT6_DIVIDE       (40),
    .CLKOUT6_PHASE        (0.000),
    .CLKOUT6_DUTY_CYCLE   (0.500),
    .CLKOUT6_USE_FINE_PS  ("FALSE"),
    .CLKIN1_PERIOD        (10.000),
    .CLKIN2_PERIOD        (8.000)
  ) mmcm_adv_inst (
    .CLKFBOUT  (clkfbout_clk_wiz_0),
    .CLKFBOUTB (),
    .CLKOUT0   (clk_out1_clk_wiz_0),
    .CLKOUT0B  (),
    .CLKOUT1   (clk_out2_clk_wiz_0),
    .CLKOUT1B  (),
    .CLKOUT2   (clk_out3_clk_wiz_0),
    .CLKOUT2B  (),
    .CLKOUT3   (clk_out4_clk_wiz_0),
    .CLKOUT3B  (),
    .CLKOUT4   (clk_out5_clk_wiz_0),
    .CLKOUT5   (clk_out6_clk_wiz_0),
    .CLKOUT6   (clk_out7_clk_wiz_0),
    .CLKFBIN   (clkfbout_buf_clk_wiz_0),
    .CLKIN1    (clk_in1_clk_wiz_0),
    .CLKIN2    (1'b0),
    .CLKINSEL  (1'b1),
    .DADDR     (7'h0),
    .DCLK      (1'b0),
    .DEN       (1'b0),
    .DI        (16'h0),
    .DO        (),
    .DRDY      (),
    .DWE       (1'b0),
    .PSCLK     (1'b0),
    .PSEN      (1'b0),
    .PSINCDEC  (1'b0),
    .PSDONE    (),
    .LOCKED    (locked_int),
    .CLKINSTOPPED (),
    .CLKFBSTOPPED (),
    .PWRDWN    (1'b0),
    .RST       (reset_high)
  );

  assign locked = locked_int;

  BUFG clkf_buf (
    .O (clkfbout_buf_clk_wiz_0),
    .I (clkfbout_clk_wiz_0)
  );

  BUFG clkout1_buf (.O(clk_out1), .I(clk_out1_clk_wiz_0));
  BUFG clkout2_buf (.O(clk_out2), .I(clk_out2_clk_wiz_0));
  BUFG clkout3_buf (.O(clk_out3), .I(clk_out3_clk_wiz_0));
  BUFG clkout4_buf (.O(clk_out4), .I(clk_out4_clk_wiz_0));
  BUFG clkout5_buf (.O(clk_out5), .I(clk_out5_clk_wiz_0));
  BUFG clkout6_buf (.O(clk_out6), .I(clk_out6_clk_wiz_0));
  BUFG clkout7_buf (.O(clk_out7), .I(clk_out7_clk_wiz_0));

endmodule

