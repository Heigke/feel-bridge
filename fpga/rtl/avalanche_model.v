// ============================================================================
// avalanche_model.v — NS-RAM Avalanche Physics Engine (Pipeline-Friendly)
// ============================================================================
//
// Faithfully implements Lanza's NS-RAM behavioral SPICE model in synthesizable
// Verilog RTL using Q16.16 fixed-point arithmetic throughout.
//
// PHYSICS MODEL (Lanza et al.):
//
//   BVpar(Vg)  = 3.5 - 1.5 * Vg
//   BVpar(T)   = BVpar0 * (1 - 21.3e-6 * (T - 300))
//   Vt         = 0.05 * (T / 300)
//   uramp(x)   = max(x, 0)
//   I_aval     = I0 * min(exp(uramp(Vcb - BVpar) / Vt), 200)
//   I_bulk     = I0_bulk * min(exp(uramp(Vcb - BVpar) / Vt), 200) * 0.1
//
// PIPELINE: 8 stages at 10 MHz. Each stage has at most ONE DSP multiply
// or simple logic. The division (uramp/Vt) is eliminated by computing
// 1/Vt = 6000/T via a reciprocal LUT, then multiplying: arg = uramp * (1/Vt).
//
//   Stage 1: BVpar0 = 3.5 - 1.5*Vg
//   Stage 2: Temp correction part 1 (correction_factor)
//   Stage 3: Temp correction part 2 (BVpar), uramp, recip_vt LUT fetch
//   Stage 4: exp_arg = uramp * recip_vt (one multiply, no division!)
//   Stage 5: LUT address computation
//   Stage 6: LUT fetch (BRAM read)
//   Stage 7: Linear interpolation
//   Stage 8: Output (kill_switch, I_bulk)
//
// Pipeline-friendly: valid_in/valid_out handshake, neuron_id pass-through.
// Shared exp_lut and recip_vt_lut BRAMs (one copy for all neurons).
//
// ============================================================================

module avalanche_model (
    input  wire        clk_phy,      // 10 MHz physics clock
    input  wire        rst,          // active-high synchronous reset
    input  wire        kill_switch,  // 1 = force I_aval=0, I_bulk=0
    input  wire [31:0] vg,           // gate voltage, Q16.16
    input  wire [31:0] vcb,          // collector-base voltage, Q16.16
    input  wire [31:0] temp_k,       // temperature in Kelvin, Q16.16
    // Pipeline handshake
    input  wire        valid_in,     // input data valid
    input  wire [6:0]  neuron_id_in, // neuron ID pass-through
    output reg         valid_out,    // output data valid
    output reg  [6:0]  neuron_id_out,// neuron ID pass-through
    // Outputs
    output reg  [31:0] i_aval,       // avalanche current, Q16.16
    output reg  [31:0] i_bulk,       // bulk heating current, Q16.16
    output reg  [31:0] bvpar         // computed BVpar for telemetry, Q16.16
);

    // ========================================================================
    // Q16.16 Constants
    // ========================================================================
    localparam [31:0] CONST_3V5       = 32'h0003_8000;  // 3.5
    localparam [31:0] CONST_1V5       = 32'h0001_8000;  // 1.5
    localparam [31:0] TEMP_COEFF_Q824 = 32'd357;        // 21.3e-6 * 2^24
    localparam [31:0] CONST_300K      = 32'h012C_0000;  // 300K in Q16.16
    localparam [31:0] CONST_ONE       = 32'h0001_0000;  // 1.0 in Q16.16
    localparam [31:0] CLAMP_200       = 32'h00C8_0000;  // 200.0 in Q16.16
    localparam [31:0] CONST_0V1       = 32'h0000_199A;  // 0.1 in Q16.16
    localparam [31:0] LUT_SCALE       = 32'd193;        // 1023/5.3
    localparam [31:0] MAX_EXP_ARG     = 32'h0005_4CCC;  // 5.3 in Q16.16

    // ========================================================================
    // Exp LUT — 1024 entries, Q16.16, stored in BRAM (shared)
    // ========================================================================
    reg [31:0] exp_lut [0:1023];
    initial begin
        $readmemh("exp_lut.hex", exp_lut);
    end

    // ========================================================================
    // Reciprocal-of-Vt LUT — 256 entries, Q16.16 (shared)
    // ========================================================================
    reg [31:0] recip_vt_lut [0:255];
    initial begin
        $readmemh("recip_vt_lut.hex", recip_vt_lut);
    end

    // ========================================================================
    // Pipeline registers — valid and neuron_id pass-through at each stage
    // ========================================================================

    // Stage 1 outputs
    reg [31:0] s1_bvpar0;
    reg [31:0] s1_vcb;
    reg [31:0] s1_temp_k;
    reg        s1_kill;
    reg        s1_valid;
    reg [6:0]  s1_nid;

    // Stage 2 outputs
    reg [31:0] s2_bvpar0;
    reg [31:0] s2_correction;
    reg        s2_delta_negative;
    reg [31:0] s2_vcb;
    reg [31:0] s2_temp_k;
    reg        s2_kill;
    reg        s2_valid;
    reg [6:0]  s2_nid;

    // Stage 3 outputs
    reg [31:0] s3_uramp;
    reg [31:0] s3_recip_vt;
    reg [31:0] s3_bvpar;
    reg        s3_kill;
    reg        s3_valid;
    reg [6:0]  s3_nid;

    // Stage 4 outputs
    reg [31:0] s4_exp_arg;
    reg [31:0] s4_bvpar;
    reg        s4_kill;
    reg        s4_valid;
    reg [6:0]  s4_nid;

    // Stage 5 outputs
    reg [9:0]  s5_lut_addr;
    reg [9:0]  s5_lut_addr_next;
    reg [15:0] s5_frac;
    reg [31:0] s5_bvpar;
    reg        s5_kill;
    reg        s5_saturated;
    reg        s5_valid;
    reg [6:0]  s5_nid;

    // Stage 6 outputs
    reg [31:0] s6_lut_lo;
    reg [31:0] s6_lut_hi;
    reg [15:0] s6_frac;
    reg [31:0] s6_bvpar;
    reg        s6_kill;
    reg        s6_saturated;
    reg        s6_valid;
    reg [6:0]  s6_nid;

    // Stage 7 outputs
    reg [31:0] s7_exp_val;
    reg [31:0] s7_bvpar;
    reg        s7_kill;
    reg        s7_valid;
    reg [6:0]  s7_nid;

    // ========================================================================
    // Stage 1: BVpar0 = 3.5 - 1.5 * Vg
    // ========================================================================
    wire [63:0] vg_times_1v5 = CONST_1V5 * vg;
    wire [31:0] vg_term      = vg_times_1v5[47:16];
    wire [31:0] bvpar0_comb  = (CONST_3V5 > vg_term) ? (CONST_3V5 - vg_term) : 32'd0;

    always @(posedge clk_phy) begin
        if (rst) begin
            s1_bvpar0 <= 32'd0;
            s1_vcb    <= 32'd0;
            s1_temp_k <= CONST_300K;
            s1_kill   <= 1'b1;
            s1_valid  <= 1'b0;
            s1_nid    <= 7'd0;
        end else begin
            s1_bvpar0 <= bvpar0_comb;
            s1_vcb    <= vcb;
            s1_temp_k <= temp_k;
            s1_kill   <= kill_switch;
            s1_valid  <= valid_in;
            s1_nid    <= neuron_id_in;
        end
    end

    // ========================================================================
    // Stage 2: Temperature correction part 1
    // ========================================================================
    wire signed [31:0] delta_t     = $signed(s1_temp_k) - $signed(CONST_300K);
    wire               delta_neg   = delta_t[31];
    wire [31:0]        abs_delta_t = delta_neg ? (~delta_t + 1) : delta_t;
    wire [63:0]        temp_product = abs_delta_t * TEMP_COEFF_Q824;
    wire [31:0]        corr_factor = temp_product[55:24];

    always @(posedge clk_phy) begin
        if (rst) begin
            s2_bvpar0         <= 32'd0;
            s2_correction     <= 32'd0;
            s2_delta_negative <= 1'b0;
            s2_vcb            <= 32'd0;
            s2_temp_k         <= CONST_300K;
            s2_kill           <= 1'b1;
            s2_valid          <= 1'b0;
            s2_nid            <= 7'd0;
        end else begin
            s2_bvpar0         <= s1_bvpar0;
            s2_correction     <= corr_factor;
            s2_delta_negative <= delta_neg;
            s2_vcb            <= s1_vcb;
            s2_temp_k         <= s1_temp_k;
            s2_kill           <= s1_kill;
            s2_valid          <= s1_valid;
            s2_nid            <= s1_nid;
        end
    end

    // ========================================================================
    // Stage 3: BVpar complete + uramp + reciprocal Vt LUT fetch
    // ========================================================================
    wire [63:0] bvpar_corr_prod = s2_bvpar0 * s2_correction;
    wire [31:0] bvpar_corr      = bvpar_corr_prod[47:16];

    wire [31:0] bvpar_temp_comb = s2_delta_negative ?
        (s2_bvpar0 + bvpar_corr) :
        ((s2_bvpar0 > bvpar_corr) ? (s2_bvpar0 - bvpar_corr) : 32'd0);

    wire        vcb_gt_bvpar    = (s2_vcb > bvpar_temp_comb);
    wire [31:0] uramp_comb      = vcb_gt_bvpar ? (s2_vcb - bvpar_temp_comb) : 32'd0;

    wire [15:0] temp_int = s2_temp_k[31:16];
    wire [7:0]  recip_idx = (temp_int < 16'd256) ? 8'd0 :
                            (temp_int > 16'd511) ? 8'd255 :
                            temp_int[7:0];

    always @(posedge clk_phy) begin
        if (rst) begin
            s3_uramp    <= 32'd0;
            s3_recip_vt <= 32'h0014_0000;
            s3_bvpar    <= 32'd0;
            s3_kill     <= 1'b1;
            s3_valid    <= 1'b0;
            s3_nid      <= 7'd0;
        end else begin
            s3_uramp    <= uramp_comb;
            s3_recip_vt <= recip_vt_lut[recip_idx];
            s3_bvpar    <= bvpar_temp_comb;
            s3_kill     <= s2_kill;
            s3_valid    <= s2_valid;
            s3_nid      <= s2_nid;
        end
    end

    // ========================================================================
    // Stage 4: exp_arg = uramp * (1/Vt)
    // ========================================================================
    wire [63:0] arg_product    = s3_uramp * s3_recip_vt;
    wire [31:0] exp_arg_raw    = arg_product[47:16];
    wire        arg_saturated  = (exp_arg_raw > MAX_EXP_ARG);
    wire [31:0] exp_arg_clamped = arg_saturated ? MAX_EXP_ARG : exp_arg_raw;

    always @(posedge clk_phy) begin
        if (rst) begin
            s4_exp_arg <= 32'd0;
            s4_bvpar   <= 32'd0;
            s4_kill    <= 1'b1;
            s4_valid   <= 1'b0;
            s4_nid     <= 7'd0;
        end else begin
            s4_exp_arg <= exp_arg_clamped;
            s4_bvpar   <= s3_bvpar;
            s4_kill    <= s3_kill;
            s4_valid   <= s3_valid;
            s4_nid     <= s3_nid;
        end
    end

    // ========================================================================
    // Stage 5: LUT address computation
    // ========================================================================
    wire [47:0] index_product = s4_exp_arg * LUT_SCALE;
    wire [9:0]  lut_addr      = (index_product[31:16] > 10'd1022) ? 10'd1022 : index_product[25:16];
    wire [9:0]  lut_addr_next = lut_addr + 10'd1;
    wire [15:0] lut_frac      = index_product[15:0];

    always @(posedge clk_phy) begin
        if (rst) begin
            s5_lut_addr      <= 10'd0;
            s5_lut_addr_next <= 10'd1;
            s5_frac          <= 16'd0;
            s5_bvpar         <= 32'd0;
            s5_kill          <= 1'b1;
            s5_saturated     <= 1'b0;
            s5_valid         <= 1'b0;
            s5_nid           <= 7'd0;
        end else begin
            s5_lut_addr      <= lut_addr;
            s5_lut_addr_next <= lut_addr_next;
            s5_frac          <= lut_frac;
            s5_bvpar         <= s4_bvpar;
            s5_kill          <= s4_kill;
            s5_saturated     <= (s4_exp_arg >= MAX_EXP_ARG);
            s5_valid         <= s4_valid;
            s5_nid           <= s4_nid;
        end
    end

    // ========================================================================
    // Stage 6: LUT fetch (BRAM read)
    // ========================================================================
    always @(posedge clk_phy) begin
        if (rst) begin
            s6_lut_lo    <= CONST_ONE;
            s6_lut_hi    <= CONST_ONE;
            s6_frac      <= 16'd0;
            s6_bvpar     <= 32'd0;
            s6_kill      <= 1'b1;
            s6_saturated <= 1'b0;
            s6_valid     <= 1'b0;
            s6_nid       <= 7'd0;
        end else begin
            s6_lut_lo    <= exp_lut[s5_lut_addr];
            s6_lut_hi    <= exp_lut[s5_lut_addr_next];
            s6_frac      <= s5_frac;
            s6_bvpar     <= s5_bvpar;
            s6_kill      <= s5_kill;
            s6_saturated <= s5_saturated;
            s6_valid     <= s5_valid;
            s6_nid       <= s5_nid;
        end
    end

    // ========================================================================
    // Stage 7: Linear interpolation
    // ========================================================================
    wire signed [32:0] lut_diff       = $signed({1'b0, s6_lut_hi}) - $signed({1'b0, s6_lut_lo});
    wire signed [48:0] interp_product = lut_diff * $signed({1'b0, s6_frac});
    wire [31:0]        interp_offset  = interp_product[47:16];

    wire [31:0] exp_interpolated = s6_saturated ? CLAMP_200 : (s6_lut_lo + interp_offset);
    wire [31:0] exp_clamped      = (exp_interpolated > CLAMP_200) ? CLAMP_200 : exp_interpolated;

    always @(posedge clk_phy) begin
        if (rst) begin
            s7_exp_val <= CONST_ONE;
            s7_bvpar   <= 32'd0;
            s7_kill    <= 1'b1;
            s7_valid   <= 1'b0;
            s7_nid     <= 7'd0;
        end else begin
            s7_exp_val <= exp_clamped;
            s7_bvpar   <= s6_bvpar;
            s7_kill    <= s6_kill;
            s7_valid   <= s6_valid;
            s7_nid     <= s6_nid;
        end
    end

    // ========================================================================
    // Stage 8: Output — kill_switch, I_bulk = I_aval * 0.1
    // ========================================================================
    wire [63:0] bulk_product = s7_exp_val * CONST_0V1;
    wire [31:0] i_bulk_val   = bulk_product[47:16];

    always @(posedge clk_phy) begin
        if (rst) begin
            i_aval        <= 32'd0;
            i_bulk        <= 32'd0;
            bvpar         <= 32'd0;
            valid_out     <= 1'b0;
            neuron_id_out <= 7'd0;
        end else begin
            bvpar         <= s7_bvpar;
            valid_out     <= s7_valid;
            neuron_id_out <= s7_nid;
            if (s7_kill) begin
                i_aval <= 32'd0;
                i_bulk <= 32'd0;
            end else begin
                i_aval <= s7_exp_val;
                i_bulk <= i_bulk_val;
            end
        end
    end

endmodule
