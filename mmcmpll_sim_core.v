// SPDX-License-Identifier: MIT
// Author: regymm
`timescale 1ps/1ps

// Behavioral simulation model approximating Xilinx MMCME2_ADV / PLLE2_ADV
// - Matches MMCME2_ADV parameters and ports. Unsupported ports are present but ignored.
// - Computes VCO frequency and output clocks using Xilinx formulas.
// - Basic parameter limits checking (7-series typical). Only allows clksel to change when reset is high.
// - Models reset/lock: lock asserts after a stabilization time when input clock is present and params valid.
// - Generates ps-accurate clocks with phase and duty via delays
// Difference from hardware:
// - Real HW: MMCM emits a single-cycle on all clocks on initialize or after reset
// - Real HW: phase shifts and duty cycles are not 100% accurate, but with roundings, the round amount depends on frequency
// - Real HW: when feedback disconnects, output still presents. 
// - Real HW: has dynamic phase shift and dynamic reconfiguration

// Phase and duty cycle accuracy rules observed from clocking wizard:
// - min_phase/360 * DIV = 1/8
// - min_duty_cycle% * DIV = 1
// - min_duty_cycle_increase% * DIV = 1/2
// E.g. 400 MHz output, VCO=800MHz, DIV=2
// - min_phase = 1/8 * 1/2 * 360 = 22.5, 22.5*N phase is available
// - min_duty_cycle = 1 * 1/2 * 100 = 50, less than 50% is not available
// - min_duty_cycle_increase = 1/2 * 1/2 * 100 = 25, so 50, 75 is available
// E.g. 100 MHz in and out, want maximum duty cycle
// - VCO optimized to be 1100 MHz, DIV=11
// - min_duty_cycle_increase = 1/2 * 1/11 * 100 = 4.545%, so maximum duty cycle is 1 - 4.545% = 95.4545%
// - If there's no duty cycle requirement, VCO will be optimized to be 1000 MHz in clkwiz. 


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
    real vco_period_ps_r;   // ps
    real vco_sub_ps_r;      // ps (VCO/8)
    time vco_sub_ps_floor;  // integer ps base delay
    real vco_sub_ps_frac;   // fractional ps part [0,1)
    real vco_sub_ps_acc;    // fractional error accumulator

    // Derived output periods (use unpacked arrays with indices handled manually for plain Verilog)
    real out_period_ns [0:6];
    real out_high_ns   [0:6];
    real out_phase_ns  [0:6];

    // Quantized output parameters in 1/8 VCO substeps
    integer out_period_x8 [0:6];   // = round(div*8)
    integer out_high_x8   [0:6];   // multiple of 4, clamped to [min, max]
    integer out_phase_x8  [0:6];   // 0..period-1
    integer out_pos_x8    [0:6];   // running phase accumulator 0..period-1

    // Parameter limits (7-series typical)
    // MMCME2 VCO range: 600..1200 MHz (per UG472). Some devices allow up to 1440/1600
    real VCO_MIN_MHZ = 600.0;
    real VCO_MAX_MHZ = 1200.0;
    // Input clock 10 MHz..800 MHz typical (start from 5.0?)
    real CIN_MIN_MHZ  = 10.0;
    real CIN_MAX_MHZ  = 800.0;

    // Lock modeling
    time lock_delay_ps = 436_100;  // Similar to modelsim lock delay

    // Helpers
    function real maxr(input real a, input real b); maxr = (a>b)?a:b; endfunction
    function real minr(input real a, input real b); minr = (a<b)?a:b; endfunction
    function integer round_real_to_int(input real r);
        begin
            if (r >= 0.0) round_real_to_int = $rtoi(r + 0.5);
            else          round_real_to_int = -$rtoi(-r + 0.5);
        end
    endfunction

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
        integer min_high8, step8, req_high8;
        integer req_phase8;
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
            vco_period_ps_r = (vco_freq_mhz>0.0) ? (1.0e6 / vco_freq_mhz) : 0.0;
            vco_sub_ps_r    = vco_period_ps_r / 8.0; // 1/8 VCO sub-tick
            vco_sub_ps_floor = vco_sub_ps_r; // trunc to integer ps
            vco_sub_ps_frac  = vco_sub_ps_r - vco_sub_ps_floor;
            vco_sub_ps_acc   = 0.0;

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
                    // Bound inputs
                    dutys[k]  = minr(maxr(dutys[k],0.0),1.0);
                    phases[k] = phases[k] - 360.0 * $floor(phases[k]/360.0);
                    // Quantize to 1/8-VCO substeps per constraints (see header lines 16-27)
                    // - Period in substeps: DIV*8, fractional DIV allowed only for CLKOUT0 on MMCME2_ADV
                    // - Usually, DIVs are always integers
                    if (k==0) out_period_x8[k] = (divs[k] < 1.0) ? 8 : round_real_to_int(divs[k]*8.0);
                    else      out_period_x8[k] = round_real_to_int($floor(divs[k]+0.5)*8.0);
                    if (out_period_x8[k] < 8) out_period_x8[k] = 8; // minimum

                    // Duty cycle quantization
                    // - Minimum high and low each 1 VCO cycle (8 substeps) when period>=16; otherwise force 50%
                    if (out_period_x8[k] >= 16) begin
                        min_high8 = 8;    // 1 VCO cycle
                        step8     = 4;    // 1/2 VCO cycle increments
                        req_high8 = round_real_to_int(dutys[k]*out_period_x8[k]);
                        // quantize to nearest multiple of step8
                        req_high8 = (req_high8/step8)*step8 + (((req_high8%step8) >= (step8/2)) ? step8 : 0);
                        if (req_high8 < min_high8) req_high8 = min_high8;
                        if (req_high8 > out_period_x8[k]-min_high8) req_high8 = out_period_x8[k]-min_high8;
                    end else begin
                        // For very small divisors, only 50% duty is realizable
                        min_high8 = out_period_x8[k]/2;
                        req_high8 = min_high8;
                    end
                    out_high_x8[k] = req_high8;

                    // Phase quantization: step = 1 substep (VCO/8)
                    req_phase8 = round_real_to_int((phases[k] * out_period_x8[k]) / 360.0);
                    // wrap into range [0, period-1]
                    req_phase8 = req_phase8 % out_period_x8[k];
                    if (req_phase8 < 0) req_phase8 = req_phase8 + out_period_x8[k];
                    out_phase_x8[k] = req_phase8;
                    // For reporting only
                    out_high_ns[k]   = (out_high_x8[k]   * (vco_period_ns/8.0));
                    out_phase_ns[k]  = (out_phase_x8[k]  * (vco_period_ns/8.0));
                end else begin
                    out_period_ns[k] = 0.0;
                    out_high_ns[k]   = 0.0;
                    out_phase_ns[k]  = 0.0;
                    out_period_x8[k] = 8;
                    out_high_x8[k]   = 4;
                    out_phase_x8[k]  = 0;
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
        end
    endtask

    // Compute parameters initially, and recompute when input clk freq changes when CLKINSEL changes
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

    // Sub-tick (VCO/8) generator using ps-resolution with fractional error cancellation
    // Generate forever; outputs consume only when LOCKED
    reg vco_subtick = 1'b0;
    time dly_ps;
    real timejump_ps;
    initial begin
        forever begin
            if (vco_sub_ps_floor == 0) begin
                // If not configured, wait a little to avoid busy loop
                #1000;
            end else begin
                timejump_ps = (vco_sub_ps_acc + vco_sub_ps_frac) >= 0.5 ? 1 :((vco_sub_ps_acc + vco_sub_ps_frac) <= -0.5 ? -1 : 0);
                dly_ps = vco_sub_ps_floor + timejump_ps;
                vco_sub_ps_acc = (vco_sub_ps_acc + vco_sub_ps_frac) - timejump_ps;
                #(dly_ps);
                vco_subtick <= 1'b1;
                // zero-time fall to create a posedge
                #0 vco_subtick <= 1'b0;
            end
        end
    end

    // lock control
    always @(*) begin
        if (RST) begin
            #1; LOCKED = 1'b0;
        end else begin
            // schedule lock assertion after delay (ps timescale)
            #(lock_delay_ps);
            if (!RST && feedback_ok && input_freq_mhz>0.0) begin
                LOCKED = 1'b1;
            end
        end
    end

    // Initialize positions and outputs on lock changes
    integer kk;
    always @(posedge LOCKED or negedge LOCKED) begin
        if (!LOCKED) begin
            CLKOUT0 = 1'b0; CLKOUT1 = 1'b0; CLKOUT2 = 1'b0;
            CLKOUT3 = 1'b0; CLKOUT4 = 1'b0; CLKOUT5 = 1'b0; CLKOUT6 = 1'b0;
            CLKFBOUT = 1'b0;
            for (kk=0; kk<7; kk=kk+1) begin
                out_pos_x8[kk] = 0;
            end
        end else begin
            for (kk=0; kk<7; kk=kk+1) begin
                // Start at quantized phase
                out_pos_x8[kk] = out_period_x8[kk] - (out_phase_x8[kk] % ((out_period_x8[kk] > 0)? out_period_x8[kk] : 1));
                // $display("out_phase_x8[%d] = %d", kk, out_period_x8[kk]);
            end
        end
    end

    // CLKOUTs
    always @(posedge vco_subtick) begin
        if (LOCKED) begin
            integer j;
            for (j=0; j<7; j=j+1) begin
                if (out_period_x8[j] > 0) begin
                    out_pos_x8[j] = (out_pos_x8[j] + 1 >= out_period_x8[j]) ? 0 : (out_pos_x8[j] + 1);
                    case (j)
                        0: CLKOUT0 = out_pos_x8[0] < out_high_x8[0];
                        1: CLKOUT1 = out_pos_x8[1] < out_high_x8[1];
                        2: CLKOUT2 = out_pos_x8[2] < out_high_x8[2];
                        3: CLKOUT3 = out_pos_x8[3] < out_high_x8[3];
                        4: CLKOUT4 = out_pos_x8[4] < out_high_x8[4];
                        5: CLKOUT5 = out_pos_x8[5] < out_high_x8[5];
                        6: CLKOUT6 = out_pos_x8[6] < out_high_x8[6];
                    endcase
                end
            end
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