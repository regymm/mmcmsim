// SPDX-License-Identifier: MIT
`timescale 1ns/1ps

// Behavioral simulation model approximating Xilinx MMCME2_ADV / PLLE2_ADV
// - Matches MMCME2_ADV parameters and ports. Unsupported ports are present but ignored.
// - Computes VCO frequency and output clocks using Xilinx formulas.
// - Parameter limits checking (7-series typical). Emits $error on violations.
// - Models reset/lock: lock asserts after a stabilization time when input clock is present and params valid.
// - Generates real-time clocks with phase and duty via delays
// Difference from hardware:
// - Real HW: MMCM emits a single-cycle on all clocks on initialize or after reset
// - Real HW: phase shifts and duty cycles are not 100% accurate, but with roundings, the round amount depends on frequency
// - Real HW: when feedback disconnects, output still presents. 
// - Real HW: has dynamic phase shift and dynamic reconfiguration


module mmcmpll_sim_core #(
    // Common generics (subset sufficient for most clocking wizard IP)
    parameter BANDWIDTH = "OPTIMIZED",           // ignored in sim
    parameter CLKOUT4_CASCADE = "FALSE",         // ignored in sim
    parameter COMPENSATION = "ZHOLD",            // ignored in sim
    parameter STARTUP_WAIT = "FALSE",            // ignored in sim

    parameter integer DIVCLK_DIVIDE = 1,                 // 1..106 (MMCME2)
    parameter real    CLKFBOUT_MULT_F = 5.0,             // 2.0..64.0 (MMCME2)
    parameter real    CLKFBOUT_PHASE = 0.0,
    parameter CLKFBOUT_USE_FINE_PS = "FALSE",   // ignored

    parameter real    CLKOUT0_DIVIDE_F = 1.0,            // 1.0..128.0
    parameter real    CLKOUT0_PHASE = 0.0,               // degrees
    parameter real    CLKOUT0_DUTY_CYCLE = 0.5,          // 0..1
    parameter CLKOUT0_USE_FINE_PS = "FALSE",

    parameter integer CLKOUT1_DIVIDE = 1,                // 1..128
    parameter real    CLKOUT1_PHASE = 0.0,
    parameter real    CLKOUT1_DUTY_CYCLE = 0.5,
    parameter CLKOUT1_USE_FINE_PS = "FALSE",

    parameter integer CLKOUT2_DIVIDE = 1,
    parameter real    CLKOUT2_PHASE = 0.0,
    parameter real    CLKOUT2_DUTY_CYCLE = 0.5,
    parameter CLKOUT2_USE_FINE_PS = "FALSE",

    parameter integer CLKOUT3_DIVIDE = 1,
    parameter real    CLKOUT3_PHASE = 0.0,
    parameter real    CLKOUT3_DUTY_CYCLE = 0.5,
    parameter CLKOUT3_USE_FINE_PS = "FALSE",

    parameter integer CLKOUT4_DIVIDE = 1,
    parameter real    CLKOUT4_PHASE = 0.0,
    parameter real    CLKOUT4_DUTY_CYCLE = 0.5,
    parameter CLKOUT4_USE_FINE_PS = "FALSE",

    parameter integer CLKOUT5_DIVIDE = 1,
    parameter real    CLKOUT5_PHASE = 0.0,
    parameter real    CLKOUT5_DUTY_CYCLE = 0.5,
    parameter CLKOUT5_USE_FINE_PS = "FALSE",

    parameter integer CLKOUT6_DIVIDE = 1,
    parameter real    CLKOUT6_PHASE = 0.0,
    parameter real    CLKOUT6_DUTY_CYCLE = 0.5,
    parameter CLKOUT6_USE_FINE_PS = "FALSE",

    parameter real    CLKIN1_PERIOD = 0.0,               // ns, 0 means unknown
    parameter real    CLKIN2_PERIOD = 0.0                // ns
) (
    // Outputs
    output reg CLKFBOUT,
    output      CLKFBOUTB,
    output reg CLKOUT0,
    output      CLKOUT0B,
    output reg CLKOUT1,
    output      CLKOUT1B,
    output reg CLKOUT2,
    output      CLKOUT2B,
    output reg CLKOUT3,
    output      CLKOUT3B,
    output reg CLKOUT4,
    output reg CLKOUT5,
    output reg CLKOUT6,

    // Inputs
    input  CLKFBIN,
    input  CLKIN1,
    input  CLKIN2,
    input  CLKINSEL,
    // DRP / PS (ignored in behavioral model)
    input  [6:0] DADDR,
    input  DCLK,
    input  DEN,
    input  [15:0] DI,
    output [15:0] DO,
    output DRDY,
    input  DWE,
    input  PSCLK,
    input  PSEN,
    input  PSINCDEC,
    output PSDONE,
    // Status
    output reg LOCKED,
    output CLKINSTOPPED,
    output CLKFBSTOPPED,
    input  PWRDWN,
    input  RST
);

    // Unused outputs driven to default
    assign CLKFBOUTB = 1'b0;
    assign CLKOUT0B  = 1'b0;
    assign CLKOUT1B  = 1'b0;
    assign CLKOUT2B  = 1'b0;
    assign CLKOUT3B  = 1'b0;
    assign DO = 16'h0000;
    assign DRDY = 1'b0;
    assign PSDONE = 1'b0;
    assign CLKINSTOPPED = 1'b0;  // simplified: not modeled
    assign CLKFBSTOPPED = 1'b0;  // simplified: not modeled

    // Internal real-time params
    real input_period_ns;
    real input_freq_mhz;
    real vco_freq_mhz;
    real vco_period_ns;

    // Derived output periods (use unpacked arrays with indices handled manually for plain Verilog)
    real out_period_ns [0:6];
    real out_high_ns   [0:6];
    real out_phase_ns  [0:6];

    // Parameter limits (7-series typical)
    // MMCME2 VCO range: 600..1200 MHz (per UG472). Some devices allow up to 1440/1600; use 600..1200 conservative.
    real VCO_MIN_MHZ = 600.0;
    real VCO_MAX_MHZ = 1200.0;
    // Input clock 10 MHz..800 MHz typical (we allow 5..800 for flexibility if wizard sets).
    real CIN_MIN_MHZ  = 10.0;
    real CIN_MAX_MHZ  = 800.0;

    // Lock modeling
    time lock_delay_ps = 0;  // computed based on input period and a small constant
    time last_reset_deassert_ps = 0;

    // Helpers
    function real maxr(input real a, input real b); maxr = (a>b)?a:b; endfunction
    function real minr(input real a, input real b); minr = (a<b)?a:b; endfunction

    // Compute frequency and generate clocks when locked
    integer i;
    reg feedback_ok;
    reg rst_sync;
    reg in_sel_sync;

    // Simple clock presence detect (edges)
    reg clkin1_d = 0, clkin2_d = 0;
    always @(posedge CLKIN1) clkin1_d <= ~clkin1_d;
    always @(posedge CLKIN2) clkin2_d <= ~clkin2_d;

    // Validate feedback connection (must loop back in sim)
    always @* begin
        feedback_ok = (CLKFBIN === CLKFBOUT);
    end

    // Parameter checking against common constraints
    task automatic check_params;
        real m, d;
        begin
            m = CLKFBOUT_MULT_F;
            d = (DIVCLK_DIVIDE < 1) ? 1.0 : DIVCLK_DIVIDE;
            if (CLKINSEL !== 1'bx) begin
                if (input_freq_mhz>0.0 && (input_freq_mhz < CIN_MIN_MHZ || input_freq_mhz > CIN_MAX_MHZ)) begin
                    $error("MMCME2_ADV sim: Input frequency %0.3f MHz out of supported range [%0.3f,%0.3f] MHz", input_freq_mhz, CIN_MIN_MHZ, CIN_MAX_MHZ);
                end
            end
            if (m < 2.0 || m > 64.0) begin
                $error("MMCME2_ADV sim: CLKFBOUT_MULT_F=%0.3f out of range [2.0,64.0]", m);
            end
            if (d < 1.0 || d > 106.0) begin
                $error("MMCME2_ADV sim: DIVCLK_DIVIDE=%0.3f out of range [1,106]", d);
            end
            if (vco_freq_mhz>0.0 && (vco_freq_mhz < VCO_MIN_MHZ || vco_freq_mhz > VCO_MAX_MHZ)) begin
                $error("MMCME2_ADV sim: VCO frequency %0.3f MHz out of range [%0.3f,%0.3f] MHz", vco_freq_mhz, VCO_MIN_MHZ, VCO_MAX_MHZ);
            end
        end
    endtask

    // Recompute on param-like changes: on edges of inputs or control
    task automatic recompute_params;
        real m, d, p0, duty0;
        real divs [0:6];
        real phases [0:6];
        real dutys  [0:6];
        integer k;
        begin
            // Select input period
            if (CLKINSEL === 1'b1) begin
                input_period_ns = (CLKIN1_PERIOD > 0.0) ? CLKIN1_PERIOD : 0.0;
            end else begin
                input_period_ns = (CLKIN2_PERIOD > 0.0) ? CLKIN2_PERIOD : 0.0;
            end
            input_freq_mhz = 1000.0 / input_period_ns;

            // Compute VCO
            d = (DIVCLK_DIVIDE < 1) ? 1.0 : DIVCLK_DIVIDE;
            m = CLKFBOUT_MULT_F;
            if (input_freq_mhz > 0.0 && d > 0.0)
                vco_freq_mhz = input_freq_mhz * m / d;
            else
                vco_freq_mhz = 0.0;
            vco_period_ns = (vco_freq_mhz>0.0) ? (1000.0 / vco_freq_mhz) : 0.0;

            // Collect output dividers
            divs[0] = (CLKOUT0_DIVIDE_F < 1.0) ? 1.0 : CLKOUT0_DIVIDE_F;
            divs[1] = (CLKOUT1_DIVIDE   < 1)   ? 1.0 : CLKOUT1_DIVIDE;
            divs[2] = (CLKOUT2_DIVIDE   < 1)   ? 1.0 : CLKOUT2_DIVIDE;
            divs[3] = (CLKOUT3_DIVIDE   < 1)   ? 1.0 : CLKOUT3_DIVIDE;
            divs[4] = (CLKOUT4_DIVIDE   < 1)   ? 1.0 : CLKOUT4_DIVIDE;
            divs[5] = (CLKOUT5_DIVIDE   < 1)   ? 1.0 : CLKOUT5_DIVIDE;
            divs[6] = (CLKOUT6_DIVIDE   < 1)   ? 1.0 : CLKOUT6_DIVIDE;

            phases[0] = CLKOUT0_PHASE; dutys[0] = CLKOUT0_DUTY_CYCLE;
            phases[1] = CLKOUT1_PHASE; dutys[1] = CLKOUT1_DUTY_CYCLE;
            phases[2] = CLKOUT2_PHASE; dutys[2] = CLKOUT2_DUTY_CYCLE;
            phases[3] = CLKOUT3_PHASE; dutys[3] = CLKOUT3_DUTY_CYCLE;
            phases[4] = CLKOUT4_PHASE; dutys[4] = CLKOUT4_DUTY_CYCLE;
            phases[5] = CLKOUT5_PHASE; dutys[5] = CLKOUT5_DUTY_CYCLE;
            phases[6] = CLKOUT6_PHASE; dutys[6] = CLKOUT6_DUTY_CYCLE;

            for (k=0;k<7;k=k+1) begin
                if (vco_period_ns>0.0) begin
                    out_period_ns[k] = vco_period_ns * divs[k];
                    out_high_ns[k]   = maxr(1.0, out_period_ns[k] * minr(maxr(dutys[k],0.0),1.0));
                    out_phase_ns[k]  = (phases[k] % 360.0) * out_period_ns[k] / 360.0;
                end else begin
                    out_period_ns[k] = 0.0;
                    out_high_ns[k]   = 0.0;
                    out_phase_ns[k]  = 0.0;
                end
            end
            check_params();
            $display("VCO     CALCULATED period_ns = %0.3f ( MHz = %0.3f )", vco_period_ns, 1000.0/vco_period_ns);
            $display("CLKOUT0 CALCULATED period_ns = %0.3f ( MHz = %0.3f ), out_high_ns = %0.3f, out_phase_ns = %0.3f", out_period_ns[0], 1000.0/out_period_ns[0], out_high_ns[0], out_phase_ns[0]);
            $display("CLKOUT1 CALCULATED period_ns = %0.3f ( MHz = %0.3f ), out_high_ns = %0.3f, out_phase_ns = %0.3f", out_period_ns[1], 1000.0/out_period_ns[1], out_high_ns[1], out_phase_ns[1]);
            $display("CLKOUT2 CALCULATED period_ns = %0.3f ( MHz = %0.3f ), out_high_ns = %0.3f, out_phase_ns = %0.3f", out_period_ns[2], 1000.0/out_period_ns[2], out_high_ns[2], out_phase_ns[2]);
            $display("CLKOUT3 CALCULATED period_ns = %0.3f ( MHz = %0.3f ), out_high_ns = %0.3f, out_phase_ns = %0.3f", out_period_ns[3], 1000.0/out_period_ns[3], out_high_ns[3], out_phase_ns[3]);
            $display("CLKOUT4 CALCULATED period_ns = %0.3f ( MHz = %0.3f ), out_high_ns = %0.3f, out_phase_ns = %0.3f", out_period_ns[4], 1000.0/out_period_ns[4], out_high_ns[4], out_phase_ns[4]);
            $display("CLKOUT5 CALCULATED period_ns = %0.3f ( MHz = %0.3f ), out_high_ns = %0.3f, out_phase_ns = %0.3f", out_period_ns[5], 1000.0/out_period_ns[5], out_high_ns[5], out_phase_ns[5]);
            $display("CLKOUT6 CALCULATED period_ns = %0.3f ( MHz = %0.3f ), out_high_ns = %0.3f, out_phase_ns = %0.3f", out_period_ns[6], 1000.0/out_period_ns[6], out_high_ns[6], out_phase_ns[6]);

            lock_delay_ps = 436_100; 
        end
    endtask

    // Compute parameters initially
    initial begin
        CLKFBOUT = 1'b0;
        CLKOUT0 = 1'b0;
        CLKOUT1 = 1'b0;
        CLKOUT2 = 1'b0;
        CLKOUT3 = 1'b0;
        CLKOUT4 = 1'b0;
        CLKOUT5 = 1'b0;
        CLKOUT6 = 1'b0;
        LOCKED  = 1'b0;
        recompute_params();
    end
    always @(CLKINSEL) begin
        recompute_params();
    end

    // Track deassert of reset
    always @(negedge RST) begin
        last_reset_deassert_ps = $time;
    end

    // Simple lock state machine (plain Verilog encoding)
    localparam S_WAIT_FB   = 1;
    localparam S_WAIT_LOCK = 2;
    localparam S_LOCKED    = 3;
    integer st;
    initial st = S_WAIT_FB;

    task automatic clk_calc(input real period_ns, input real high_ns, input real phase_ns, 
                           output time t_high_out, output time t_low_out, output time t_phase_out);
        begin
            t_high_out = (high_ns>0.0)? high_ns : 0;
            t_low_out  = (period_ns>0.0)? (period_ns - t_high_out) : 0;
            t_phase_out= (phase_ns>0.0)? phase_ns : 0;
        end
    endtask

    // Main lock/clock control
    always @(*) begin
        if (RST) begin
            #1; LOCKED = 1'b0;
            st = S_WAIT_FB;
        end else begin
            case (st)
                S_WAIT_FB: begin
                    if (feedback_ok) begin
                        st = S_WAIT_LOCK;
                        // schedule lock assertion after delay
                        #(lock_delay_ps/1000);
                        if (!RST && feedback_ok && input_freq_mhz>0.0) begin
                            LOCKED = 1'b1;
                            st = S_LOCKED;
                            fork
                            // CLKOUT0 generator
                            begin
                                time t_h, t_l, t_p;
                                clk_calc(out_period_ns[0], out_high_ns[0], out_phase_ns[0], t_h, t_l, t_p);
                                if (t_p>0) #(t_p);
                                while (LOCKED) begin
                                    CLKOUT0 = 1'b1; #(t_h);
                                    CLKOUT0 = 1'b0; #(t_l);
                                end
                                CLKOUT0 = 1'b0;
                            end
                            // CLKOUT1 generator
                            begin
                                time t_h, t_l, t_p;
                                clk_calc(out_period_ns[1], out_high_ns[1], out_phase_ns[1], t_h, t_l, t_p);
                                if (t_p>0) #(t_p);
                                while (LOCKED) begin
                                    CLKOUT1 = 1'b1; #(t_h);
                                    CLKOUT1 = 1'b0; #(t_l);
                                end
                                CLKOUT1 = 1'b0;
                            end
                            // CLKOUT2 generator
                            begin
                                time t_h, t_l, t_p;
                                clk_calc(out_period_ns[2], out_high_ns[2], out_phase_ns[2], t_h, t_l, t_p);
                                if (t_p>0) #(t_p);
                                while (LOCKED) begin
                                    CLKOUT2 = 1'b1; #(t_h);
                                    CLKOUT2 = 1'b0; #(t_l);
                                end
                                CLKOUT2 = 1'b0;
                            end
                            // CLKOUT3 generator
                            begin
                                time t_h, t_l, t_p;
                                clk_calc(out_period_ns[3], out_high_ns[3], out_phase_ns[3], t_h, t_l, t_p);
                                if (t_p>0) #(t_p);
                                while (LOCKED) begin
                                    CLKOUT3 = 1'b1; #(t_h);
                                    CLKOUT3 = 1'b0; #(t_l);
                                end
                                CLKOUT3 = 1'b0;
                            end
                            // CLKOUT4 generator
                            begin
                                time t_h, t_l, t_p;
                                clk_calc(out_period_ns[4], out_high_ns[4], out_phase_ns[4], t_h, t_l, t_p);
                                if (t_p>0) #(t_p);
                                while (LOCKED) begin
                                    CLKOUT4 = 1'b1; #(t_h);
                                    CLKOUT4 = 1'b0; #(t_l);
                                end
                                CLKOUT4 = 1'b0;
                            end
                            // CLKOUT5 generator
                            begin
                                time t_h, t_l, t_p;
                                clk_calc(out_period_ns[5], out_high_ns[5], out_phase_ns[5], t_h, t_l, t_p);
                                if (t_p>0) #(t_p);
                                while (LOCKED) begin
                                    CLKOUT5 = 1'b1; #(t_h);
                                    CLKOUT5 = 1'b0; #(t_l);
                                end
                                CLKOUT5 = 1'b0;
                            end
                            // CLKOUT6 generator
                            begin
                                time t_h, t_l, t_p;
                                clk_calc(out_period_ns[6], out_high_ns[6], out_phase_ns[6], t_h, t_l, t_p);
                                if (t_p>0) #(t_p);
                                while (LOCKED) begin
                                    CLKOUT6 = 1'b1; #(t_h);
                                    CLKOUT6 = 1'b0; #(t_l);
                                end
                                CLKOUT6 = 1'b0;
                            end
                            // CLKFBOUT generator (feedback approximate with selected input clock)
                            begin
                                time t_h, t_l, t_p;
                                clk_calc(input_period_ns, input_period_ns/2, 0.0, t_h, t_l, t_p);
                                if (t_p>0) #(t_p);
                                while (LOCKED) begin
                                    CLKFBOUT = 1'b1; #(t_h);
                                    CLKFBOUT = 1'b0; #(t_l);
                                end
                                CLKFBOUT = 1'b0;
                            end
                            join_none
                        end
                    end
                end
                S_WAIT_LOCK: begin
                    // no-op; wait till RST
                end
            endcase
        end
    end

endmodule

// MMCME2_ADV is most commonly used 
module MMCME2_ADV #(
    parameter string BANDWIDTH = "OPTIMIZED",
    parameter string CLKOUT4_CASCADE = "FALSE",
    parameter string COMPENSATION = "ZHOLD",
    parameter string STARTUP_WAIT = "FALSE",
    parameter integer DIVCLK_DIVIDE = 1,
    parameter real    CLKFBOUT_MULT_F = 5.0,
    parameter real    CLKFBOUT_PHASE = 0.0,
    parameter string  CLKFBOUT_USE_FINE_PS = "FALSE",
    parameter real    CLKOUT0_DIVIDE_F = 1.0,
    parameter real    CLKOUT0_PHASE = 0.0,
    parameter real    CLKOUT0_DUTY_CYCLE = 0.5,
    parameter string  CLKOUT0_USE_FINE_PS = "FALSE",
    parameter integer CLKOUT1_DIVIDE = 1,
    parameter real    CLKOUT1_PHASE = 0.0,
    parameter real    CLKOUT1_DUTY_CYCLE = 0.5,
    parameter string  CLKOUT1_USE_FINE_PS = "FALSE",
    parameter integer CLKOUT2_DIVIDE = 1,
    parameter real    CLKOUT2_PHASE = 0.0,
    parameter real    CLKOUT2_DUTY_CYCLE = 0.5,
    parameter string  CLKOUT2_USE_FINE_PS = "FALSE",
    parameter integer CLKOUT3_DIVIDE = 1,
    parameter real    CLKOUT3_PHASE = 0.0,
    parameter real    CLKOUT3_DUTY_CYCLE = 0.5,
    parameter string  CLKOUT3_USE_FINE_PS = "FALSE",
    parameter integer CLKOUT4_DIVIDE = 1,
    parameter real    CLKOUT4_PHASE = 0.0,
    parameter real    CLKOUT4_DUTY_CYCLE = 0.5,
    parameter string  CLKOUT4_USE_FINE_PS = "FALSE",
    parameter integer CLKOUT5_DIVIDE = 1,
    parameter real    CLKOUT5_PHASE = 0.0,
    parameter real    CLKOUT5_DUTY_CYCLE = 0.5,
    parameter string  CLKOUT5_USE_FINE_PS = "FALSE",
    parameter integer CLKOUT6_DIVIDE = 1,
    parameter real    CLKOUT6_PHASE = 0.0,
    parameter real    CLKOUT6_DUTY_CYCLE = 0.5,
    parameter string  CLKOUT6_USE_FINE_PS = "FALSE",
    parameter real    CLKIN1_PERIOD = 0.0,
    parameter real    CLKIN2_PERIOD = 0.0
) (
    output CLKFBOUT, output CLKFBOUTB,
    output CLKOUT0,  output CLKOUT0B,
    output CLKOUT1,  output CLKOUT1B,
    output CLKOUT2,  output CLKOUT2B,
    output CLKOUT3,  output CLKOUT3B,
    output CLKOUT4,  output CLKOUT5, output CLKOUT6,
    input  CLKFBIN,
    input  CLKIN1, input CLKIN2, input CLKINSEL,
    input  [6:0] DADDR, input DCLK, input DEN, input [15:0] DI, output [15:0] DO, output DRDY, input DWE,
    input  PSCLK, input PSEN, input PSINCDEC, output PSDONE,
    output LOCKED, output CLKINSTOPPED, output CLKFBSTOPPED,
    input  PWRDWN, input RST
);
    mmcmpll_sim_core #(
        .BANDWIDTH(BANDWIDTH), .CLKOUT4_CASCADE(CLKOUT4_CASCADE), .COMPENSATION(COMPENSATION), .STARTUP_WAIT(STARTUP_WAIT),
        .DIVCLK_DIVIDE(DIVCLK_DIVIDE), .CLKFBOUT_MULT_F(CLKFBOUT_MULT_F), .CLKFBOUT_PHASE(CLKFBOUT_PHASE), .CLKFBOUT_USE_FINE_PS(CLKFBOUT_USE_FINE_PS),
        .CLKOUT0_DIVIDE_F(CLKOUT0_DIVIDE_F), .CLKOUT0_PHASE(CLKOUT0_PHASE), .CLKOUT0_DUTY_CYCLE(CLKOUT0_DUTY_CYCLE), .CLKOUT0_USE_FINE_PS(CLKOUT0_USE_FINE_PS),
        .CLKOUT1_DIVIDE(CLKOUT1_DIVIDE), .CLKOUT1_PHASE(CLKOUT1_PHASE), .CLKOUT1_DUTY_CYCLE(CLKOUT1_DUTY_CYCLE), .CLKOUT1_USE_FINE_PS(CLKOUT1_USE_FINE_PS),
        .CLKOUT2_DIVIDE(CLKOUT2_DIVIDE), .CLKOUT2_PHASE(CLKOUT2_PHASE), .CLKOUT2_DUTY_CYCLE(CLKOUT2_DUTY_CYCLE), .CLKOUT2_USE_FINE_PS(CLKOUT2_USE_FINE_PS),
        .CLKOUT3_DIVIDE(CLKOUT3_DIVIDE), .CLKOUT3_PHASE(CLKOUT3_PHASE), .CLKOUT3_DUTY_CYCLE(CLKOUT3_DUTY_CYCLE), .CLKOUT3_USE_FINE_PS(CLKOUT3_USE_FINE_PS),
        .CLKOUT4_DIVIDE(CLKOUT4_DIVIDE), .CLKOUT4_PHASE(CLKOUT4_PHASE), .CLKOUT4_DUTY_CYCLE(CLKOUT4_DUTY_CYCLE), .CLKOUT4_USE_FINE_PS(CLKOUT4_USE_FINE_PS),
        .CLKOUT5_DIVIDE(CLKOUT5_DIVIDE), .CLKOUT5_PHASE(CLKOUT5_PHASE), .CLKOUT5_DUTY_CYCLE(CLKOUT5_DUTY_CYCLE), .CLKOUT5_USE_FINE_PS(CLKOUT5_USE_FINE_PS),
        .CLKOUT6_DIVIDE(CLKOUT6_DIVIDE), .CLKOUT6_PHASE(CLKOUT6_PHASE), .CLKOUT6_DUTY_CYCLE(CLKOUT6_DUTY_CYCLE), .CLKOUT6_USE_FINE_PS(CLKOUT6_USE_FINE_PS),
        .CLKIN1_PERIOD(CLKIN1_PERIOD), .CLKIN2_PERIOD(CLKIN2_PERIOD)
    ) core (
        .CLKFBOUT(CLKFBOUT), .CLKFBOUTB(CLKFBOUTB),
        .CLKOUT0(CLKOUT0), .CLKOUT0B(CLKOUT0B),
        .CLKOUT1(CLKOUT1), .CLKOUT1B(CLKOUT1B),
        .CLKOUT2(CLKOUT2), .CLKOUT2B(CLKOUT2B),
        .CLKOUT3(CLKOUT3), .CLKOUT3B(CLKOUT3B),
        .CLKOUT4(CLKOUT4), .CLKOUT5(CLKOUT5), .CLKOUT6(CLKOUT6),
        .CLKFBIN(CLKFBIN), .CLKIN1(CLKIN1), .CLKIN2(CLKIN2), .CLKINSEL(CLKINSEL),
        .DADDR(DADDR), .DCLK(DCLK), .DEN(DEN), .DI(DI), .DO(DO), .DRDY(DRDY), .DWE(DWE),
        .PSCLK(PSCLK), .PSEN(PSEN), .PSINCDEC(PSINCDEC), .PSDONE(PSDONE),
        .LOCKED(LOCKED), .CLKINSTOPPED(CLKINSTOPPED), .CLKFBSTOPPED(CLKFBSTOPPED),
        .PWRDWN(PWRDWN), .RST(RST)
    );
endmodule

module IBUF (
    output O,
    input  I
);
    assign O = I;
endmodule

module BUFG (
    output O,
    input  I
);
    assign O = I;
endmodule