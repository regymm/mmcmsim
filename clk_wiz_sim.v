// SPDX-License-Identifier: MIT
// Author: regymm
`timescale 1ns / 1ps

module print_freq_after_lock #(
    parameter expected_freq = -1,
    parameter expected_duty = -1,
    parameter allowed_err = 0.1
) (
    input [8*9:1] name,
    input clk,
    input locked,
    output reg pass = 0
);
  function real abs;
    input real in;
    begin
        abs = in > 0 ? in : -in;
    end
  endfunction
  real rising_edge;
  real falling_edge;
  real freq = -1;
  real duty = -1;
  reg freq_pass = 0;
  reg duty_pass = 0;
  reg printed = 0;
  integer i;
  integer cycles_to_measure = 10;
  always @ (posedge clk) begin
    if (locked & !printed) begin
        @(posedge clk);
        @(posedge clk);
        rising_edge = $realtime;
        @(negedge clk) falling_edge = $realtime;
        @(posedge clk) begin
            duty = 100*(1-($realtime-falling_edge)/($realtime-rising_edge));
            for (i = 0; i < cycles_to_measure - 1; i = i + 1) begin
                @(posedge clk);
            end
            freq = 1.0e3 * cycles_to_measure / ($realtime - rising_edge);
            $display("%s: frequency %0.3f MHz, duty cycle %0.3f%%.", 
            name, freq, duty);
            if (expected_freq > 0 && abs(expected_freq - freq) > allowed_err) $display("WARNING: %s frequency deviated from expectation, %0.3f instead of %0.3f!", 
                name, freq, expected_freq);
            else freq_pass = 1;
            if (expected_duty > 0 && abs(expected_duty - duty) > allowed_err) $display("WARNING: %s duty cycle deviated from expectation, %0.3f instead of %0.3f!", 
                name, duty, expected_duty);
            else duty_pass = 1;
            pass = freq_pass & duty_pass;
            printed = 1;
        end
    end
  end
  always @ (*) begin
    if (!locked) printed = 0;
  end
endmodule

module tb_clk_wiz_0;

  localparam real CLK_IN1_PERIOD = 10.0;   // 100 MHz
  localparam real CLK_IN2_PERIOD = 8.0;    // 125 MHz

  reg  clk_in1   = 0;
  reg  clk_in2   = 0;
  reg  clk_in_sel = 0;
  reg  reset      = 0;
  wire locked;

  wire clk_out1;
  wire clk_out2;
  wire clk_out3;
  wire clk_out4;
  wire clk_out5;
  wire clk_out6;
  wire clk_out7;

  // Check that clk_in_sel doesn't change when reset is low
  reg clk_in_sel_prev = 0;
  initial clk_in_sel_prev = clk_in_sel;
  always @(clk_in_sel) begin
    if (!reset && clk_in_sel !== clk_in_sel_prev) begin
      $display("[%0t] ERROR: clk_in_sel changed from %b to %b while reset is low! MMCM must be reset before changing clock selection.", 
             $time, clk_in_sel_prev, clk_in_sel);
      $finish;
    end
    clk_in_sel_prev = clk_in_sel;
  end

  clk_wiz_0_clk_wiz dut (
    .clk_in1   (clk_in1),
    .clk_in2   (clk_in2),
    .clk_in_sel(clk_in_sel),
    .reset     (reset),
    .locked    (locked),
    .clk_out1  (clk_out1),
    .clk_out2  (clk_out2),
    .clk_out3  (clk_out3),
    .clk_out4  (clk_out4),
    .clk_out5  (clk_out5),
    .clk_out6  (clk_out6),
    .clk_out7  (clk_out7)
  );

  // Clock generation
  always #(CLK_IN1_PERIOD/2.0) clk_in1 = ~clk_in1;
  always #(CLK_IN2_PERIOD/2.0) clk_in2 = ~clk_in2;
  


  // input frequency tests
  print_freq_after_lock #(.expected_freq(100.0), .expected_duty(50.0)) print_freq_after_lockin1(
    .name("clk_in1"),
    .clk(clk_in1),
    .locked(locked),
    .pass(pass_in1)
  );
  print_freq_after_lock #(.expected_freq(125.0), .expected_duty(50.0)) print_freq_after_lockin2(
    .name("clk_in2"),
    .clk(clk_in2),
    .locked(locked),
    .pass(pass_in2)
  );
  // output frequency tests 0: 100 MHz in
  print_freq_after_lock #(.expected_freq(1.25*100.0), .expected_duty(50.0)) print_freq_after_lockout1_0(
    .name("clk_out1"),
    .clk(clk_out1),
    .locked(!clk_in_sel & locked),
    .pass(pass_out1_0)
  );
  print_freq_after_lock #(.expected_freq(1.25*100.0), .expected_duty(50.0)) print_freq_after_lockout2_0(
    .name("clk_out2"),
    .clk(clk_out2),
    .locked(!clk_in_sel & locked),
    .pass(pass_out2_0)
  );
  print_freq_after_lock #(.expected_freq(1.25*100.0), .expected_duty(27.8)) print_freq_after_lockout3_0(
    .name("clk_out3"),
    .clk(clk_out3),
    .locked(!clk_in_sel & locked),
    .pass(pass_out3_0)
  );
  print_freq_after_lock #(.expected_freq(1.25*100.0), .expected_duty(77.8)) print_freq_after_lockout4_0(
    .name("clk_out4"),
    .clk(clk_out4),
    .locked(!clk_in_sel & locked),
    .pass(pass_out4_0)
  );
  print_freq_after_lock #(.expected_freq(1.25*33.333), .expected_duty(50.0)) print_freq_after_lockout501(
    .name("clk_out5"),
    .clk(clk_out5),
    .locked(!clk_in_sel & locked),
    .pass(pass_out5_0)
  );
  print_freq_after_lock #(.expected_freq(1.25*150.0), .expected_duty(50.0)) print_freq_after_lockout6_0(
    .name("clk_out6"),
    .clk(clk_out6),
    .locked(!clk_in_sel & locked),
    .pass(pass_out6_0)
  );
  print_freq_after_lock #(.expected_freq(1.25*225.0), .expected_duty(50.0)) print_freq_after_lockout7_0(
    .name("clk_out7"),
    .clk(clk_out7),
    .locked(!clk_in_sel & locked),
    .pass(pass_out7_0)
  );
  // output frequency tests 1: 125 MHz in
  print_freq_after_lock #(.expected_freq(100.0), .expected_duty(50.0)) print_freq_after_lockout1_1(
    .name("clk_out1"),
    .clk(clk_out1),
    .locked(clk_in_sel & locked),
    .pass(pass_out1_1)
  );
  print_freq_after_lock #(.expected_freq(100.0), .expected_duty(50.0)) print_freq_after_lockout2_1(
    .name("clk_out2"),
    .clk(clk_out2),
    .locked(clk_in_sel & locked),
    .pass(pass_out2_1)
  );
  print_freq_after_lock #(.expected_freq(100.0), .expected_duty(27.8)) print_freq_after_lockout3_1(
    .name("clk_out3"),
    .clk(clk_out3),
    .locked(clk_in_sel & locked),
    .pass(pass_out3_1)
  );
  print_freq_after_lock #(.expected_freq(100.0), .expected_duty(77.8)) print_freq_after_lockout4_1(
    .name("clk_out4"),
    .clk(clk_out4),
    .locked(clk_in_sel & locked),
    .pass(pass_out4_1)
  );
  print_freq_after_lock #(.expected_freq(33.333), .expected_duty(50.0)) print_freq_after_lockout5_1(
    .name("clk_out5"),
    .clk(clk_out5),
    .locked(clk_in_sel & locked),
    .pass(pass_out5_1)
  );
  print_freq_after_lock #(.expected_freq(150.0), .expected_duty(50.0)) print_freq_after_lockout6_1(
    .name("clk_out6"),
    .clk(clk_out6),
    .locked(clk_in_sel & locked),
    .pass(pass_out6_1)
  );
  print_freq_after_lock #(.expected_freq(225.0), .expected_duty(50.0)) print_freq_after_lockout7_1(
    .name("clk_out7"),
    .clk(clk_out7),
    .locked(clk_in_sel & locked),
    .pass(pass_out7_1)
  );
  //==========================================================================
  // Test sequence
  //==========================================================================
  initial begin
    $dumpfile("sim.vcd");
    $dumpvars(99,tb_clk_wiz_0);
  end
  initial begin
    $timeformat(-9,2," ns",10);
    $display("[%0t] START simulation",$time);

    reset = 0;
    clk_in_sel = 0; // use primary (100 MHz)
    #1000;
    // reset before changing selection
    $display("[%0t] Apply reset",$time);
    reset = 1;
    #50;
    $display("[%0t] Change clk_in_sel DURING reset",$time);
    clk_in_sel = 1; // switch to 125 MHz
    #50;
    $display("[%0t] Deassert reset",$time);
    reset = 0;

    wait(locked==1);
    $display("[%0t] MMCM locked",$time);
    #1000;
    $display("[%0t] END simulation",$time);
    if (pass_in1 && 
    pass_out1_0 && pass_out2_0 && pass_out3_0 && pass_out4_0 && pass_out5_0 && pass_out6_0 && pass_out7_0 &&
    pass_in2 && pass_out1_1 && pass_out2_1 && pass_out3_1 && pass_out4_1 && pass_out5_1 && pass_out6_1 && pass_out7_1) begin
      $display("[%0t] ALL TESTS PASSED",$time);
      $finish;
    end else begin
      $display("[%0t] SOME TESTS FAILED",$time);
      $fatal;
    end
  end

  // simple monitor
  initial
    $monitor("[%0t] reset=%0b clk_in_sel=%0b locked=%0b",$time,reset,clk_in_sel,locked);

endmodule

module clk_wiz_0_clk_wiz 

 (// Clock in ports
  input         clk_in2,
  input         clk_in_sel,
  // Clock out ports
  output        clk_out1,
  output        clk_out2,
  output        clk_out3,
  output        clk_out4,
  output        clk_out5,
  output        clk_out6,
  output        clk_out7,
  // Status and control signals
  input         reset,
  output        locked,
  input         clk_in1
 );
  // Input buffering
  //------------------------------------
wire clk_in1_clk_wiz_0;
wire clk_in2_clk_wiz_0;
  IBUF clkin1_ibufg
   (.O (clk_in1_clk_wiz_0),
    .I (clk_in1));

  IBUF clkin2_ibufg
   (.O (clk_in2_clk_wiz_0),
    .I (clk_in2));

  wire        clk_out1_clk_wiz_0;
  wire        clk_out2_clk_wiz_0;
  wire        clk_out3_clk_wiz_0;
  wire        clk_out4_clk_wiz_0;
  wire        clk_out5_clk_wiz_0;
  wire        clk_out6_clk_wiz_0;
  wire        clk_out7_clk_wiz_0;

  wire        locked_int;
  wire        clkfbout_clk_wiz_0;
  wire        clkfbout_buf_clk_wiz_0;
  wire        reset_high;

  MMCME2_ADV
  #(.BANDWIDTH            ("OPTIMIZED"),
    .CLKOUT4_CASCADE      ("FALSE"),
    .COMPENSATION         ("ZHOLD"),
    .STARTUP_WAIT         ("FALSE"),
    .DIVCLK_DIVIDE        (1),
    .CLKFBOUT_MULT_F      (9.000),
    .CLKFBOUT_PHASE       (0.000),
    .CLKFBOUT_USE_FINE_PS ("FALSE"),
    .CLKOUT0_DIVIDE_F     (9.000),
    .CLKOUT0_PHASE        (0.000),
    .CLKOUT0_DUTY_CYCLE   (0.500),
    .CLKOUT0_USE_FINE_PS  ("FALSE"),
    .CLKOUT1_DIVIDE       (9),
    .CLKOUT1_PHASE        (90.000),
    .CLKOUT1_DUTY_CYCLE   (0.500),
    .CLKOUT1_USE_FINE_PS  ("FALSE"),
    .CLKOUT2_DIVIDE       (9),
    .CLKOUT2_PHASE        (180.000),
    .CLKOUT2_DUTY_CYCLE   (0.278),
    .CLKOUT2_USE_FINE_PS  ("FALSE"),
    .CLKOUT3_DIVIDE       (9),
    .CLKOUT3_PHASE        (270.000),
    .CLKOUT3_DUTY_CYCLE   (0.778),
    .CLKOUT3_USE_FINE_PS  ("FALSE"),
    .CLKOUT4_DIVIDE       (27),
    .CLKOUT4_PHASE        (0.000),
    .CLKOUT4_DUTY_CYCLE   (0.500),
    .CLKOUT4_USE_FINE_PS  ("FALSE"),
    .CLKOUT5_DIVIDE       (6),
    .CLKOUT5_PHASE        (0.000),
    .CLKOUT5_DUTY_CYCLE   (0.500),
    .CLKOUT5_USE_FINE_PS  ("FALSE"),
    .CLKOUT6_DIVIDE       (4),
    .CLKOUT6_PHASE        (0.000),
    .CLKOUT6_DUTY_CYCLE   (0.500),
    .CLKOUT6_USE_FINE_PS  ("FALSE"),
    .CLKIN1_PERIOD        (10.000),
    .CLKIN2_PERIOD        (8.000))
  mmcm_adv_inst
    // Output clocks
   (
    .CLKFBOUT            (clkfbout_clk_wiz_0),
    .CLKFBOUTB           (),
    .CLKOUT0             (clk_out1_clk_wiz_0),
    .CLKOUT0B            (),
    .CLKOUT1             (clk_out2_clk_wiz_0),
    .CLKOUT1B            (),
    .CLKOUT2             (clk_out3_clk_wiz_0),
    .CLKOUT2B            (),
    .CLKOUT3             (clk_out4_clk_wiz_0),
    .CLKOUT3B            (),
    .CLKOUT4             (clk_out5_clk_wiz_0),
    .CLKOUT5             (clk_out6_clk_wiz_0),
    .CLKOUT6             (clk_out7_clk_wiz_0),
     // Input clock control
    .CLKFBIN             (clkfbout_buf_clk_wiz_0),
    .CLKIN1              (clk_in1_clk_wiz_0),
    .CLKIN2              (clk_in2_clk_wiz_0),
    .CLKINSEL            (clk_in_sel),
    // Ports for dynamic reconfiguration
    .DADDR               (7'h0),
    .DCLK                (1'b0),
    .DEN                 (1'b0),
    .DI                  (16'h0),
    .DO                  (),
    .DRDY                (),
    .DWE                 (1'b0),
    // Ports for dynamic phase shift
    .PSCLK               (1'b0),
    .PSEN                (1'b0),
    .PSINCDEC            (1'b0),
    .PSDONE              (),
    // Other control and status signals
    .LOCKED              (locked_int),
    .CLKINSTOPPED        (),
    .CLKFBSTOPPED        (),
    .PWRDWN              (1'b0),
    .RST                 (reset_high));

  assign reset_high = reset; 

  assign locked = locked_int;
// Clock Monitor clock assigning
//--------------------------------------
 // Output buffering
  //-----------------------------------

  BUFG clkf_buf
   (.O (clkfbout_buf_clk_wiz_0),
    .I (clkfbout_clk_wiz_0));






  BUFG clkout1_buf
   (.O   (clk_out1),
    .I   (clk_out1_clk_wiz_0));


  BUFG clkout2_buf
   (.O   (clk_out2),
    .I   (clk_out2_clk_wiz_0));

  BUFG clkout3_buf
   (.O   (clk_out3),
    .I   (clk_out3_clk_wiz_0));

  BUFG clkout4_buf
   (.O   (clk_out4),
    .I   (clk_out4_clk_wiz_0));

  BUFG clkout5_buf
   (.O   (clk_out5),
    .I   (clk_out5_clk_wiz_0));

  BUFG clkout6_buf
   (.O   (clk_out6),
    .I   (clk_out6_clk_wiz_0));

  BUFG clkout7_buf
   (.O   (clk_out7),
    .I   (clk_out7_clk_wiz_0));

  // Simulation check: verify feedback clock buffer output matches input
  always @(clkfbout_buf_clk_wiz_0 or clkfbout_clk_wiz_0) begin
    if (clkfbout_buf_clk_wiz_0 !== clkfbout_clk_wiz_0) begin
      $error("CLKFBIN and CLKFBOUT are not connected! Output clock will misbehave on hardware!");
      $finish;
    end
  end


endmodule
