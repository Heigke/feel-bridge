// =============================================================================
// neuron_scheduler.v — State Storage + Round-Robin Scheduler for 128 Neurons
// =============================================================================
// Manages per-neuron state and sequences neuron IDs through a shared
// avalanche+LIF pipeline. Round-robin counter feeds neuron_id into the pipeline
// each clock cycle. State is read at pipeline entry and written back when the
// pipeline result emerges.
//
// All arrays use distributed RAM (LUT RAM) since they have multiple read ports
// and complex write patterns (bit-slice writes, conditional writes).
// 128 × 32 bits = 512 bytes per array. Total ~2048 LUTs for storage.
// =============================================================================

module neuron_scheduler #(
    parameter NUM_NEURONS    = 128,
    parameter NEURON_ADDR_W  = 7,
    parameter PIPELINE_DEPTH = 11
) (
    input  wire        clk_phy,
    input  wire        rst,

    // --- Read port: state for current neuron entering pipeline ---
    output reg  [NEURON_ADDR_W-1:0] rd_neuron_id,
    output reg                      rd_valid,
    output reg  [31:0]              rd_vg,
    output reg  [31:0]              rd_vmem,
    output reg  [15:0]              rd_spike_cnt,
    output reg  [15:0]              rd_refract_ctr,
    output reg  [31:0]              rd_syn_w_packed,

    // --- Write-back port: results from pipeline exit ---
    input  wire [NEURON_ADDR_W-1:0] wb_neuron_id,
    input  wire                     wb_valid,
    input  wire [31:0]              wb_vmem,
    input  wire [15:0]              wb_spike_cnt,
    input  wire [15:0]              wb_refract_ctr,

    // --- Host configuration port (clk_phy domain, after CDC) ---
    input  wire [NEURON_ADDR_W-1:0] cfg_neuron_id,
    input  wire [31:0]              cfg_vg,
    input  wire                     cfg_vg_wen,
    input  wire [31:0]              cfg_syn_w_packed,
    input  wire                     cfg_syn_wen,

    // --- Telemetry readout port ---
    input  wire [NEURON_ADDR_W-1:0] telem_neuron_id,
    input  wire                     telem_req,
    output reg  [31:0]              telem_vg,
    output reg  [31:0]              telem_vmem,
    output reg  [15:0]              telem_spike_cnt,
    output reg  [15:0]              telem_refract_ctr,
    output reg                      telem_valid,

    // --- Spike counter reset (from telemetry latch) ---
    input  wire                     spike_cnt_reset,

    // --- Regime Vg: default when no override ---
    input  wire [31:0]              regime_vg
);

    // =====================================================================
    // State storage — plain registers (not RAM arrays)
    // =====================================================================
    // With multiple read ports, partial bit-slice access, and multiple
    // conditional writers, Vivado cannot infer these as BRAM. Using flat
    // registers. At 128 neurons this is ~16K FFs — fits within xc7a100t
    // (126,800 FFs available, ~12.6%).
    // =====================================================================
    reg [31:0] mem_vg   [0:NUM_NEURONS-1];
    reg [31:0] mem_vmem [0:NUM_NEURONS-1];
    reg [15:0] mem_scnt [0:NUM_NEURONS-1];
    reg [15:0] mem_refr [0:NUM_NEURONS-1];
    reg [31:0] mem_syn  [0:NUM_NEURONS-1];
    reg        vg_override [0:NUM_NEURONS-1];

    // Initialize to zero
    integer k;
    initial begin
        for (k = 0; k < NUM_NEURONS; k = k + 1) begin
            mem_vg[k]      = 32'd0;
            mem_vmem[k]    = 32'd0;
            mem_scnt[k]    = 16'd0;
            mem_refr[k]    = 16'd0;
            mem_syn[k]     = 32'h1010_2020; // lateral: N±1=0.125, N±2=0.0625
            vg_override[k] = 1'b0;
        end
    end

    // =====================================================================
    // Round-robin counter
    // =====================================================================
    reg [NEURON_ADDR_W-1:0] rr_counter;

    always @(posedge clk_phy) begin
        if (rst)
            rr_counter <= {NEURON_ADDR_W{1'b0}};
        else if (rr_counter == NUM_NEURONS - 1)
            rr_counter <= {NEURON_ADDR_W{1'b0}};
        else
            rr_counter <= rr_counter + 1;
    end

    // =====================================================================
    // Read port — 1-cycle latency
    // =====================================================================
    always @(posedge clk_phy) begin
        if (rst) begin
            rd_neuron_id    <= {NEURON_ADDR_W{1'b0}};
            rd_valid        <= 1'b0;
            rd_vg           <= 32'd0;
            rd_vmem         <= 32'd0;
            rd_spike_cnt    <= 16'd0;
            rd_refract_ctr  <= 16'd0;
            rd_syn_w_packed <= 32'd0;
        end else begin
            rd_neuron_id    <= rr_counter;
            rd_valid        <= 1'b1;
            rd_vg           <= vg_override[rr_counter] ? mem_vg[rr_counter] : regime_vg;
            rd_vmem         <= mem_vmem[rr_counter];
            rd_spike_cnt    <= mem_scnt[rr_counter];
            rd_refract_ctr  <= mem_refr[rr_counter];
            rd_syn_w_packed <= mem_syn[rr_counter];
        end
    end

    // =====================================================================
    // Vmem write — single writer: pipeline write-back
    // =====================================================================
    always @(posedge clk_phy) begin
        if (wb_valid && !rst)
            mem_vmem[wb_neuron_id] <= wb_vmem;
    end

    // =====================================================================
    // Refract write — single writer: pipeline write-back
    // =====================================================================
    always @(posedge clk_phy) begin
        if (wb_valid && !rst)
            mem_refr[wb_neuron_id] <= wb_refract_ctr;
    end

    // =====================================================================
    // Spike count write — merged single writer:
    //   Priority: reset scan > pipeline write-back
    // =====================================================================
    reg                     resetting;
    reg [NEURON_ADDR_W-1:0] reset_idx;

    always @(posedge clk_phy) begin
        if (rst) begin
            resetting <= 1'b0;
            reset_idx <= {NEURON_ADDR_W{1'b0}};
        end else if (resetting) begin
            mem_scnt[reset_idx] <= 16'd0;
            if (reset_idx == NUM_NEURONS - 1) begin
                resetting <= 1'b0;
            end else begin
                reset_idx <= reset_idx + 1;
            end
        end else if (spike_cnt_reset) begin
            resetting <= 1'b1;
            reset_idx <= {NEURON_ADDR_W{1'b0}};
        end else if (wb_valid) begin
            mem_scnt[wb_neuron_id] <= wb_spike_cnt;
        end
    end

    // =====================================================================
    // Vg write — single writer: host config
    // =====================================================================
    always @(posedge clk_phy) begin
        if (rst) begin
            // vg_override initialized by initial block
        end else if (cfg_vg_wen) begin
            mem_vg[cfg_neuron_id]     <= cfg_vg;
            vg_override[cfg_neuron_id] <= 1'b1;
        end
    end

    // =====================================================================
    // Synapse weight write — single writer: host config
    // =====================================================================
    always @(posedge clk_phy) begin
        if (!rst && cfg_syn_wen)
            mem_syn[cfg_neuron_id] <= cfg_syn_w_packed;
    end

    // =====================================================================
    // Telemetry readout — 1-cycle latency after telem_req
    // =====================================================================
    reg telem_req_d;
    always @(posedge clk_phy) begin
        if (rst) begin
            telem_vg          <= 32'd0;
            telem_vmem        <= 32'd0;
            telem_spike_cnt   <= 16'd0;
            telem_refract_ctr <= 16'd0;
            telem_valid       <= 1'b0;
            telem_req_d       <= 1'b0;
        end else begin
            telem_req_d <= telem_req;
            telem_valid <= telem_req_d;
            if (telem_req) begin
                telem_vg          <= vg_override[telem_neuron_id] ?
                                      mem_vg[telem_neuron_id] : regime_vg;
                telem_vmem        <= mem_vmem[telem_neuron_id];
                telem_spike_cnt   <= mem_scnt[telem_neuron_id];
                telem_refract_ctr <= mem_refr[telem_neuron_id];
            end
        end
    end

endmodule
