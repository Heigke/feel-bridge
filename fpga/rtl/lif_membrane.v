// =============================================================================
// lif_membrane.v — Leaky Integrate-and-Fire membrane (Pipeline-Friendly)
// =============================================================================
// Physics basis: Pazos LIF SPICE model
//   Cint = 102 fF, Euler integration at 10 MHz (dt = 100 ns)
//   dVmem = (I_exc + I_syn + I_bias - I_leak) * dt / Cint
//
// All values Q16.16 fixed-point unless noted.
// Signed arithmetic for current summation (leak can exceed excitation).
// Vmem clamped to [0, MAX] — no negative membrane voltage.
//
// Kill-shot causal chain: i_aval=0 => I_exc=0 => no spikes possible.
//
// Pipeline-friendly version:
//   - vmem/refractory state read from external BRAM (vmem_in, refract_in)
//   - Results written back to BRAM (vmem_out, refract_out, spike_cnt_out)
//   - valid_in/valid_out handshake with neuron_id pass-through
//   - 2-stage pipeline (stage 1: current compute, stage 2: integration)
// =============================================================================

module lif_membrane (
    input  wire        clk_phy,         // 10 MHz physics clock
    input  wire        rst,             // active-high synchronous reset
    input  wire [31:0] i_aval,          // from avalanche_model (Q16.16)
    input  wire [31:0] mac_signal,      // MAC signal from GPU  (Q16.16, 0..1.0)
    input  wire [3:0]  pre_spikes,      // pre-synaptic spike inputs (1-bit each)
    input  wire [31:0] syn_w_packed,    // packed {w3[7:0], w2[7:0], w1[7:0], w0[7:0]}
    input  wire [31:0] leak_cond_in,    // runtime-configurable leak conductance (Q16.16)
    // Runtime-configurable parameters (from host via Ethernet)
    input  wire [31:0] cfg_threshold,   // spike threshold (Q16.16)
    input  wire [31:0] cfg_base_exc,    // avalanche excitation gain (Q16.16)
    input  wire [31:0] cfg_bias_gain,   // MAC current injection gain (Q16.16)
    input  wire [31:0] cfg_dt_over_c,   // integration step size (Q16.16)
    input  wire [15:0] cfg_refract_cyc, // refractory period in cycles
    // External state input (from BRAM)
    input  wire [31:0] vmem_in,         // current membrane voltage from BRAM
    input  wire [15:0] refract_in,      // current refractory counter from BRAM
    input  wire [15:0] spike_cnt_in,    // current spike count from BRAM
    // Pipeline handshake
    input  wire        valid_in,
    input  wire [6:0]  neuron_id_in,
    output reg         valid_out,
    output reg  [6:0]  neuron_id_out,
    // State output (to BRAM write-back)
    output reg         spike_out,       // 1-cycle pulse on threshold crossing
    output reg  [31:0] vmem_out,        // updated membrane voltage
    output reg  [15:0] refract_out,     // updated refractory counter
    output reg  [15:0] spike_cnt_out    // updated spike count
);

// -------------------------------------------------------------------------
// Parameters (compile-time defaults, overridden by cfg_* inputs at runtime)
// -------------------------------------------------------------------------
parameter [31:0] THRESHOLD      = 32'h0000_8000;  // 0.50V (lower for partial reset regime)
parameter [31:0] LEAK_COND      = 32'h0000_0004;  // leak conductance (slow cortical: τ≈210ms)
parameter [31:0] BASE_EXC       = 32'h0000_0333;  // 0.0125 — moderate avalanche gain
parameter [31:0] BIAS_BASE      = 32'h0000_0000;  // 0.0 — ZERO bias
parameter [31:0] BIAS_GAIN      = 32'h0000_0800;  // 0.03125 — direct MAC current injection
parameter [31:0] DT_OVER_C      = 32'h0000_0200;  // 0.0078 — slow integration
parameter [15:0] REFRACT_CYCLES = 16'd50;          // 5 us @ 10 MHz

// Runtime-active values (from cfg_* inputs)
wire [31:0] act_threshold  = cfg_threshold;
wire [31:0] act_base_exc   = cfg_base_exc;
wire [31:0] act_bias_gain  = cfg_bias_gain;
wire [31:0] act_dt_over_c  = cfg_dt_over_c;
wire [15:0] act_refract    = cfg_refract_cyc;

// Maximum representable Q16.16 positive value
localparam [31:0] VMEM_MAX = 32'h7FFF_FFFF;

// Unpack synapse weights from packed 4×8-bit to Q16.16
// Each 8-bit value maps to Q16.16: {16'd0, w[7:0], 8'd0} for 0..255/256 range
wire [31:0] syn_w0 = {16'd0, syn_w_packed[7:0],   8'd0};
wire [31:0] syn_w1 = {16'd0, syn_w_packed[15:8],  8'd0};
wire [31:0] syn_w2 = {16'd0, syn_w_packed[23:16], 8'd0};
wire [31:0] syn_w3 = {16'd0, syn_w_packed[31:24], 8'd0};

// -------------------------------------------------------------------------
// Synaptic current computation — combinational
// -------------------------------------------------------------------------
wire               refractory_in = (refract_in != 16'd0);
wire               syn_enable = ~refractory_in;
wire signed [31:0] syn_contrib0 = (pre_spikes[0] & syn_enable) ? $signed(syn_w0) : 32'sd0;
wire signed [31:0] syn_contrib1 = (pre_spikes[1] & syn_enable) ? $signed(syn_w1) : 32'sd0;
wire signed [31:0] syn_contrib2 = (pre_spikes[2] & syn_enable) ? $signed(syn_w2) : 32'sd0;
wire signed [31:0] syn_contrib3 = (pre_spikes[3] & syn_enable) ? $signed(syn_w3) : 32'sd0;

wire signed [33:0] syn_sum_wide = {syn_contrib0[31], syn_contrib0[31], syn_contrib0}
                                + {syn_contrib1[31], syn_contrib1[31], syn_contrib1}
                                + {syn_contrib2[31], syn_contrib2[31], syn_contrib2}
                                + {syn_contrib3[31], syn_contrib3[31], syn_contrib3};

wire signed [31:0] syn_sum_sat = (syn_sum_wide > $signed(34'sh1_7FFF_FFFF)) ? 32'h7FFF_FFFF :
                                 (syn_sum_wide < $signed(34'sh2_8000_0000)) ? 32'h8000_0000 :
                                  syn_sum_wide[31:0];

// -------------------------------------------------------------------------
// Pipeline stage 1: Compute all four current components
// -------------------------------------------------------------------------
reg signed [63:0] i_exc_full;
reg signed [63:0] i_bias_full;
reg signed [63:0] i_leak_full;

reg signed [31:0] p1_i_exc;
reg signed [31:0] p1_i_syn;
reg signed [31:0] p1_i_bias;
reg signed [31:0] p1_i_leak;
reg        [31:0] p1_vmem;
reg               p1_refractory;
reg        [15:0] p1_refract_ctr;
reg        [15:0] p1_spike_cnt;
reg               p1_valid;
reg        [6:0]  p1_nid;

always @(posedge clk_phy) begin
    if (rst) begin
        p1_i_exc       <= 32'sd0;
        p1_i_syn       <= 32'sd0;
        p1_i_bias      <= 32'sd0;
        p1_i_leak      <= 32'sd0;
        p1_vmem        <= 32'd0;
        p1_refractory  <= 1'b0;
        p1_refract_ctr <= 16'd0;
        p1_spike_cnt   <= 16'd0;
        p1_valid       <= 1'b0;
        p1_nid         <= 7'd0;
    end else begin
        p1_valid <= valid_in;
        p1_nid   <= neuron_id_in;

        // I_exc = act_base_exc * i_aval
        i_exc_full = $signed({1'b0, act_base_exc}) * $signed({1'b0, i_aval});
        p1_i_exc <= i_exc_full[47:16];

        // I_syn
        p1_i_syn <= syn_sum_sat;

        // I_bias = BIAS_BASE + act_bias_gain * mac_signal
        i_bias_full = $signed({1'b0, act_bias_gain}) * $signed({1'b0, mac_signal});
        p1_i_bias <= $signed({1'b0, BIAS_BASE}) + $signed(i_bias_full[47:16]);

        // I_leak = leak_cond * Vmem (runtime-configurable)
        i_leak_full = $signed({1'b0, leak_cond_in}) * $signed({1'b0, vmem_in});
        p1_i_leak <= i_leak_full[47:16];

        // Forward state from BRAM
        p1_vmem        <= vmem_in;
        p1_refractory  <= refractory_in;
        p1_refract_ctr <= refract_in;
        p1_spike_cnt   <= spike_cnt_in;
    end
end

// -------------------------------------------------------------------------
// Pipeline stage 2: Integration, threshold, refractory
// -------------------------------------------------------------------------
reg signed [31:0] i_net;
reg signed [63:0] dv_full;
reg signed [31:0] dv;
reg signed [32:0] vmem_next_raw;
reg        [31:0] vmem_next;

always @(posedge clk_phy) begin
    if (rst) begin
        vmem_out      <= 32'd0;
        spike_out     <= 1'b0;
        refract_out   <= 16'd0;
        spike_cnt_out <= 16'd0;
        valid_out     <= 1'b0;
        neuron_id_out <= 7'd0;
    end else begin
        valid_out     <= p1_valid;
        neuron_id_out <= p1_nid;
        spike_out     <= 1'b0;

        if (!p1_valid) begin
            // No valid data — pass through zeros
            vmem_out      <= 32'd0;
            refract_out   <= 16'd0;
            spike_cnt_out <= 16'd0;
        end else if (p1_refractory) begin
            // Refractory period: hold Vmem (preserve residual), count down
            vmem_out <= p1_vmem;
            spike_cnt_out <= p1_spike_cnt;
            if (p1_refract_ctr <= 16'd1) begin
                refract_out <= 16'd0;
            end else begin
                refract_out <= p1_refract_ctr - 16'd1;
            end
        end else begin
            // Active integration
            i_net = p1_i_exc + p1_i_syn + p1_i_bias - p1_i_leak;

            dv_full = $signed(i_net) * $signed({1'b0, act_dt_over_c});
            dv = dv_full[47:16];

            vmem_next_raw = {1'b0, p1_vmem} + {{1{dv[31]}}, dv};

            // Clamp to [0, VMEM_MAX]
            if (vmem_next_raw[32]) begin
                vmem_next = 32'd0;
            end else if (vmem_next_raw[31]) begin
                vmem_next = VMEM_MAX;
            end else begin
                vmem_next = vmem_next_raw[31:0];
            end

            // Threshold comparison
            if (vmem_next > act_threshold) begin
                spike_out     <= 1'b1;
                vmem_out      <= vmem_next - act_threshold;  // partial reset: preserve residual
                refract_out   <= act_refract;
                // Increment spike count with saturation
                spike_cnt_out <= (p1_spike_cnt == 16'hFFFF) ? 16'hFFFF : p1_spike_cnt + 16'd1;
            end else begin
                vmem_out      <= vmem_next;
                refract_out   <= 16'd0;
                spike_cnt_out <= p1_spike_cnt;
            end
        end
    end
end

endmodule
