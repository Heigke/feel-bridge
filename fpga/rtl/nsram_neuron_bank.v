// =============================================================================
// nsram_neuron_bank.v — Time-Multiplexed Bank of 128 NS-RAM Neurons
// =============================================================================
// Single shared avalanche+LIF pipeline processes all 128 neurons in round-robin.
// Per-neuron state (vg, vmem, spike_cnt, refract, syn_w) stored in BRAM via
// neuron_scheduler module.
//
// Shared: Vcb pulse generator, bulk capacitor integrator, pre-synaptic spike
// generators. Pipeline depth = 10 (8 avalanche + 2 LIF).
//
// All analog values Q16.16 fixed-point. Physics at 10 MHz, control at 100 MHz.
//
// Vcb pulse train: period 10 us (1000 cycles @ 10 MHz), high 1 us (100 cycles)
// Amplitude 3.15V = 0x0003_2666. Vcb = V_pulse - V_bulk (leaky integrator).
//
// Pre-synaptic spike generators: 4 independent pulse trains with configurable
// period/phase, matching SPICE Vpx1-Vpx4. Defaults: 2.0, 3.0, 1.0, 4.0 us.
// =============================================================================

module nsram_neuron_bank #(
    parameter NUM_NEURONS   = 128,
    parameter NEURON_ADDR_W = 7
) (
    input  wire        clk_sys,          // 100 MHz system clock
    input  wire        clk_phy,          // 10 MHz physics clock
    input  wire        rst,
    input  wire        kill_switch,      // global kill-shot
    input  wire [31:0] temp_k,           // shared temperature Q16.16
    input  wire [31:0] mac_signal,       // shared MAC signal Q16.16
    input  wire [31:0] leak_cond,       // runtime-configurable leak conductance Q16.16
    // Runtime-configurable LIF parameters
    input  wire [31:0] cfg_threshold,   // spike threshold (Q16.16)
    input  wire [31:0] cfg_base_exc,    // avalanche excitation gain (Q16.16)
    input  wire [31:0] cfg_bias_gain,   // MAC current injection gain (Q16.16)
    input  wire [31:0] cfg_dt_over_c,   // integration step size (Q16.16)
    input  wire [15:0] cfg_refract_cyc, // refractory period in cycles
    input  wire [1:0]  regime,           // 0=cold, 1=hot, 2=coupled
    // Per-neuron config (active neuron selected by neuron_sel)
    input  wire [NEURON_ADDR_W-1:0] neuron_sel,
    input  wire [31:0] vg_write,
    input  wire        vg_wen,
    input  wire [31:0] syn_w_write,      // packed 4×8-bit synapse weights
    input  wire        syn_wen,
    // Outputs
    output wire [NUM_NEURONS-1:0] spike_out,  // per-neuron spike pulses
    output wire [15:0] spike_count_total,      // aggregate count since last read
    // Telemetry output for selected neuron
    output reg  [31:0] vmem_sel,
    output reg  [31:0] bvpar_sel,
    output reg  [15:0] spike_count_sel,
    output reg  [15:0] refract_sel,
    input  wire        count_reset,      // reset spike counters (on telemetry read)
    // Debug probe outputs
    output reg  [31:0] dbg_i_aval_sel,
    output reg  [31:0] dbg_vcb,
    output reg  [31:0] dbg_v_bulk,
    output reg         dbg_kill_phy
);

// =========================================================================
// Parameters
// =========================================================================
localparam PIPELINE_DEPTH = 10; // 8 avalanche + 2 LIF

// Vcb pulse generator
localparam [9:0]  VCB_PERIOD    = 10'd1000;
localparam [9:0]  VCB_HIGH_TIME = 10'd100;
localparam [31:0] VCB_AMPLITUDE = 32'h0003_2666; // 3.15V Q16.16

// Bulk capacitor leaky integrator
localparam [31:0] BULK_LEAK_ALPHA = 32'h0000_0100;
localparam [31:0] BULK_CHARGE_K   = 32'h0000_0004;

// Pre-synaptic pulse generator periods (in 10 MHz cycles)
localparam [15:0] PSX_PERIOD_0 = 16'd20;
localparam [15:0] PSX_PERIOD_1 = 16'd30;
localparam [15:0] PSX_PERIOD_2 = 16'd10;
localparam [15:0] PSX_PERIOD_3 = 16'd40;

localparam [15:0] PSX_PHASE_0 = 16'd0;
localparam [15:0] PSX_PHASE_1 = 16'd5;
localparam [15:0] PSX_PHASE_2 = 16'd2;
localparam [15:0] PSX_PHASE_3 = 16'd10;

// Regime gate voltage defaults (Q16.16)
localparam [31:0] VG_COLD    = 32'h0000_4CCD; // 0.30V
localparam [31:0] VG_HOT     = 32'h0000_7333; // 0.45V
localparam [31:0] VG_BASE_C  = 32'h0000_599A; // 0.35V
localparam [31:0] VG_GAIN_C  = 32'h0000_199A; // 0.10

// =========================================================================
// Vcb Pulse Generator (shared across all neurons)
// =========================================================================
reg [9:0] vcb_counter;
reg [31:0] v_pulse;

always @(posedge clk_phy) begin
    if (rst) begin
        vcb_counter <= 10'd0;
        v_pulse     <= 32'd0;
    end else begin
        if (vcb_counter >= VCB_PERIOD - 1)
            vcb_counter <= 10'd0;
        else
            vcb_counter <= vcb_counter + 10'd1;
        v_pulse <= (vcb_counter < VCB_HIGH_TIME) ? VCB_AMPLITUDE : 32'd0;
    end
end

// =========================================================================
// Bulk Capacitor Model (leaky integrator, shared)
// =========================================================================
reg [31:0] v_bulk;
reg signed [63:0] bulk_leak_prod;
reg signed [63:0] bulk_charge_prod;

always @(posedge clk_phy) begin
    if (rst) begin
        v_bulk <= 32'd0;
    end else begin
        bulk_leak_prod = $signed({1'b0, v_bulk}) * $signed({1'b0, BULK_LEAK_ALPHA});
        bulk_charge_prod = (v_pulse != 32'd0) ?
            ($signed({1'b0, BULK_CHARGE_K}) * $signed({1'b0, VCB_AMPLITUDE})) : 64'sd0;
        if (v_bulk < bulk_leak_prod[47:16])
            v_bulk <= 32'd0;
        else
            v_bulk <= v_bulk - bulk_leak_prod[47:16] + bulk_charge_prod[47:16];
    end
end

wire [31:0] vcb_signal = (v_pulse > v_bulk) ? (v_pulse - v_bulk) : 32'd0;

// =========================================================================
// Pre-Synaptic Spike Generators (4 independent pulse trains)
// =========================================================================
reg [15:0] psx_counter [0:3];
wire [3:0] pre_spikes;

always @(posedge clk_phy) begin
    if (rst) psx_counter[0] <= PSX_PHASE_0;
    else if (psx_counter[0] >= PSX_PERIOD_0 - 1) psx_counter[0] <= 16'd0;
    else psx_counter[0] <= psx_counter[0] + 16'd1;
end
assign pre_spikes[0] = (psx_counter[0] == 16'd0);

always @(posedge clk_phy) begin
    if (rst) psx_counter[1] <= PSX_PHASE_1;
    else if (psx_counter[1] >= PSX_PERIOD_1 - 1) psx_counter[1] <= 16'd0;
    else psx_counter[1] <= psx_counter[1] + 16'd1;
end
assign pre_spikes[1] = (psx_counter[1] == 16'd0);

always @(posedge clk_phy) begin
    if (rst) psx_counter[2] <= PSX_PHASE_2;
    else if (psx_counter[2] >= PSX_PERIOD_2 - 1) psx_counter[2] <= 16'd0;
    else psx_counter[2] <= psx_counter[2] + 16'd1;
end
assign pre_spikes[2] = (psx_counter[2] == 16'd0);

always @(posedge clk_phy) begin
    if (rst) psx_counter[3] <= PSX_PHASE_3;
    else if (psx_counter[3] >= PSX_PERIOD_3 - 1) psx_counter[3] <= 16'd0;
    else psx_counter[3] <= psx_counter[3] + 16'd1;
end
assign pre_spikes[3] = (psx_counter[3] == 16'd0);

// =========================================================================
// Regime-based Gate Voltage Computation
// =========================================================================
reg [31:0] vg_regime;
reg signed [63:0] vg_coupled_prod;

always @(posedge clk_phy) begin
    if (rst) begin
        vg_regime <= VG_COLD;
    end else begin
        case (regime)
            2'd0: vg_regime <= VG_COLD;
            2'd1: vg_regime <= VG_HOT;
            2'd2: begin
                vg_coupled_prod = $signed({1'b0, VG_GAIN_C}) * $signed({1'b0, mac_signal});
                vg_regime <= VG_BASE_C + vg_coupled_prod[47:16];
            end
            default: vg_regime <= VG_COLD;
        endcase
    end
end

// =========================================================================
// Kill Switch — 2-FF synchronizer from clk_sys to clk_phy
// =========================================================================
reg kill_sync_0, kill_sync_1;
always @(posedge clk_phy) begin
    if (rst) begin
        kill_sync_0 <= 1'b0;
        kill_sync_1 <= 1'b0;
    end else begin
        kill_sync_0 <= kill_switch;
        kill_sync_1 <= kill_sync_0;
    end
end
wire kill_phy = kill_sync_1;

wire [31:0] vcb_killed = kill_phy ? 32'd0 : vcb_signal;

// =========================================================================
// CDC: neuron_sel, vg_write, vg_wen, syn_w_write, syn_wen → clk_phy
// =========================================================================
// Use toggle handshake for write enable signals, 2-FF for data (stable by
// the time toggle arrives).

// Vg write CDC
reg        vg_wen_toggle_sys;
reg        vg_wen_prev;
reg [NEURON_ADDR_W-1:0] vg_sel_sys;
reg [31:0] vg_data_sys;

always @(posedge clk_sys) begin
    if (rst) begin
        vg_wen_toggle_sys <= 1'b0;
        vg_wen_prev       <= 1'b0;
        vg_sel_sys        <= {NEURON_ADDR_W{1'b0}};
        vg_data_sys       <= 32'd0;
    end else begin
        vg_wen_prev <= vg_wen;
        if (vg_wen && !vg_wen_prev) begin
            vg_wen_toggle_sys <= ~vg_wen_toggle_sys;
            vg_sel_sys        <= neuron_sel;
            vg_data_sys       <= vg_write;
        end
    end
end

reg vg_toggle_phy0, vg_toggle_phy1, vg_toggle_phy2;
reg [NEURON_ADDR_W-1:0] vg_sel_phy;
reg [31:0] vg_data_phy;
wire cfg_vg_wen_phy = (vg_toggle_phy1 != vg_toggle_phy2);

always @(posedge clk_phy) begin
    if (rst) begin
        vg_toggle_phy0 <= 1'b0;
        vg_toggle_phy1 <= 1'b0;
        vg_toggle_phy2 <= 1'b0;
        vg_sel_phy     <= {NEURON_ADDR_W{1'b0}};
        vg_data_phy    <= 32'd0;
    end else begin
        vg_toggle_phy0 <= vg_wen_toggle_sys;
        vg_toggle_phy1 <= vg_toggle_phy0;
        vg_toggle_phy2 <= vg_toggle_phy1;
        vg_sel_phy     <= vg_sel_sys;
        vg_data_phy    <= vg_data_sys;
    end
end

// Synapse write CDC
reg        syn_wen_toggle_sys;
reg        syn_wen_prev;
reg [NEURON_ADDR_W-1:0] syn_sel_sys;
reg [31:0] syn_data_sys;

always @(posedge clk_sys) begin
    if (rst) begin
        syn_wen_toggle_sys <= 1'b0;
        syn_wen_prev       <= 1'b0;
        syn_sel_sys        <= {NEURON_ADDR_W{1'b0}};
        syn_data_sys       <= 32'd0;
    end else begin
        syn_wen_prev <= syn_wen;
        if (syn_wen && !syn_wen_prev) begin
            syn_wen_toggle_sys <= ~syn_wen_toggle_sys;
            syn_sel_sys        <= neuron_sel;
            syn_data_sys       <= syn_w_write;
        end
    end
end

reg syn_toggle_phy0, syn_toggle_phy1, syn_toggle_phy2;
reg [NEURON_ADDR_W-1:0] syn_sel_phy;
reg [31:0] syn_data_phy;
wire cfg_syn_wen_phy = (syn_toggle_phy1 != syn_toggle_phy2);

always @(posedge clk_phy) begin
    if (rst) begin
        syn_toggle_phy0 <= 1'b0;
        syn_toggle_phy1 <= 1'b0;
        syn_toggle_phy2 <= 1'b0;
        syn_sel_phy     <= {NEURON_ADDR_W{1'b0}};
        syn_data_phy    <= 32'd0;
    end else begin
        syn_toggle_phy0 <= syn_wen_toggle_sys;
        syn_toggle_phy1 <= syn_toggle_phy0;
        syn_toggle_phy2 <= syn_toggle_phy1;
        syn_sel_phy     <= syn_sel_sys;
        syn_data_phy    <= syn_data_sys;
    end
end

// count_reset CDC
reg cr_toggle_sys, cr_prev;
always @(posedge clk_sys) begin
    if (rst) begin
        cr_toggle_sys <= 1'b0;
        cr_prev       <= 1'b0;
    end else begin
        cr_prev <= count_reset;
        if (count_reset && !cr_prev)
            cr_toggle_sys <= ~cr_toggle_sys;
    end
end

reg cr_phy0, cr_phy1, cr_phy2;
wire spike_cnt_reset_phy = (cr_phy1 != cr_phy2);
always @(posedge clk_phy) begin
    if (rst) begin
        cr_phy0 <= 1'b0;
        cr_phy1 <= 1'b0;
        cr_phy2 <= 1'b0;
    end else begin
        cr_phy0 <= cr_toggle_sys;
        cr_phy1 <= cr_phy0;
        cr_phy2 <= cr_phy1;
    end
end

// =========================================================================
// Neuron Scheduler — BRAM state + round-robin
// =========================================================================
wire [NEURON_ADDR_W-1:0] sched_rd_neuron_id;
wire                     sched_rd_valid;
wire [31:0]              sched_rd_vg;
wire [31:0]              sched_rd_vmem;
wire [15:0]              sched_rd_spike_cnt;
wire [15:0]              sched_rd_refract_ctr;
wire [31:0]              sched_rd_syn_w;

// Telemetry readout (in clk_phy domain)
reg  [NEURON_ADDR_W-1:0] telem_nid_phy;
reg                      telem_req_phy;
wire [31:0]              telem_vg_phy;
wire [31:0]              telem_vmem_phy;
wire [15:0]              telem_scnt_phy;
wire [15:0]              telem_refr_phy;
wire                     telem_valid_phy;

// Forward declarations for LIF outputs (used in scheduler write-back)
wire        lif_valid_out;
wire [6:0]  lif_neuron_id_out;
wire        lif_spike;
wire [31:0] lif_vmem_out;
wire [15:0] lif_refract_out;
wire [15:0] lif_spike_cnt_out;

neuron_scheduler #(
    .NUM_NEURONS    (NUM_NEURONS),
    .NEURON_ADDR_W  (NEURON_ADDR_W),
    .PIPELINE_DEPTH (PIPELINE_DEPTH)
) u_scheduler (
    .clk_phy          (clk_phy),
    .rst              (rst),
    // Read port
    .rd_neuron_id     (sched_rd_neuron_id),
    .rd_valid         (sched_rd_valid),
    .rd_vg            (sched_rd_vg),
    .rd_vmem          (sched_rd_vmem),
    .rd_spike_cnt     (sched_rd_spike_cnt),
    .rd_refract_ctr   (sched_rd_refract_ctr),
    .rd_syn_w_packed  (sched_rd_syn_w),
    // Write-back port
    .wb_neuron_id     (lif_neuron_id_out),
    .wb_valid         (lif_valid_out),
    .wb_vmem          (lif_vmem_out),
    .wb_spike_cnt     (lif_spike_cnt_out),
    .wb_refract_ctr   (lif_refract_out),
    // Configuration
    .cfg_neuron_id    (cfg_vg_wen_phy ? vg_sel_phy : syn_sel_phy),
    .cfg_vg           (vg_data_phy),
    .cfg_vg_wen       (cfg_vg_wen_phy),
    .cfg_syn_w_packed (syn_data_phy),
    .cfg_syn_wen      (cfg_syn_wen_phy),
    // Telemetry
    .telem_neuron_id  (telem_nid_phy),
    .telem_req        (telem_req_phy),
    .telem_vg         (telem_vg_phy),
    .telem_vmem       (telem_vmem_phy),
    .telem_spike_cnt  (telem_scnt_phy),
    .telem_refract_ctr(telem_refr_phy),
    .telem_valid      (telem_valid_phy),
    // Spike counter reset
    .spike_cnt_reset  (spike_cnt_reset_phy),
    // Regime Vg
    .regime_vg        (vg_regime)
);

// =========================================================================
// Shared Avalanche Pipeline
// =========================================================================
wire [31:0] aval_i_aval;
wire [31:0] aval_i_bulk;
wire [31:0] aval_bvpar;
wire        aval_valid_out;
wire [6:0]  aval_nid_out;

avalanche_model u_aval (
    .clk_phy      (clk_phy),
    .rst          (rst),
    .kill_switch  (kill_phy),
    .vcb          (vcb_killed),
    .vg           (sched_rd_vg),
    .temp_k       (temp_k),
    .valid_in     (sched_rd_valid),
    .neuron_id_in (sched_rd_neuron_id),
    .valid_out    (aval_valid_out),
    .neuron_id_out(aval_nid_out),
    .i_aval       (aval_i_aval),
    .i_bulk       (aval_i_bulk),
    .bvpar        (aval_bvpar)
);

// =========================================================================
// Shared LIF Membrane Pipeline
// =========================================================================
// LIF needs vmem/refract/spike_cnt from BRAM for the neuron that exits
// the avalanche pipeline. We use a delay line to look up the state
// from the scheduler at the right time.
//
// The scheduler read port feeds neuron state at pipeline entry. But
// LIF needs the state AFTER avalanche processing (8 cycles later).
// Since state doesn't change during pipeline flight (only one copy
// in-flight), we delay the scheduler read outputs to match.

// Delay vmem/refract/spike_cnt by 8 cycles (avalanche pipeline depth)
// to align with avalanche output
localparam AVAL_DEPTH = 8; // avalanche pipeline stages

reg [31:0] vmem_delay   [0:AVAL_DEPTH-1];
reg [15:0] refract_delay[0:AVAL_DEPTH-1];
reg [15:0] scnt_delay   [0:AVAL_DEPTH-1];
reg [31:0] synw_delay   [0:AVAL_DEPTH-1];

integer di;
always @(posedge clk_phy) begin
    if (rst) begin
        for (di = 0; di < AVAL_DEPTH; di = di + 1) begin
            vmem_delay[di]    <= 32'd0;
            refract_delay[di] <= 16'd0;
            scnt_delay[di]    <= 16'd0;
            synw_delay[di]    <= 32'd0;
        end
    end else begin
        vmem_delay[0]    <= sched_rd_vmem;
        refract_delay[0] <= sched_rd_refract_ctr;
        scnt_delay[0]    <= sched_rd_spike_cnt;
        synw_delay[0]    <= sched_rd_syn_w;
        for (di = 1; di < AVAL_DEPTH; di = di + 1) begin
            vmem_delay[di]    <= vmem_delay[di-1];
            refract_delay[di] <= refract_delay[di-1];
            scnt_delay[di]    <= scnt_delay[di-1];
            synw_delay[di]    <= synw_delay[di-1];
        end
    end
end

wire [31:0] lif_vmem_in    = vmem_delay[AVAL_DEPTH-1];
wire [15:0] lif_refract_in = refract_delay[AVAL_DEPTH-1];
wire [15:0] lif_scnt_in    = scnt_delay[AVAL_DEPTH-1];
wire [31:0] lif_synw_in    = synw_delay[AVAL_DEPTH-1];

lif_membrane u_lif (
    .clk_phy       (clk_phy),
    .rst           (rst),
    .i_aval        (aval_i_aval),
    .mac_signal    (mac_signal),
    .pre_spikes    (lateral_spikes),
    .syn_w_packed  (lif_synw_in),
    .leak_cond_in  (leak_cond),
    .cfg_threshold (cfg_threshold),
    .cfg_base_exc  (cfg_base_exc),
    .cfg_bias_gain (cfg_bias_gain),
    .cfg_dt_over_c (cfg_dt_over_c),
    .cfg_refract_cyc(cfg_refract_cyc),
    .vmem_in       (lif_vmem_in),
    .refract_in    (lif_refract_in),
    .spike_cnt_in  (lif_scnt_in),
    .valid_in      (aval_valid_out),
    .neuron_id_in  (aval_nid_out),
    .valid_out     (lif_valid_out),
    .neuron_id_out (lif_neuron_id_out),
    .spike_out     (lif_spike),
    .vmem_out      (lif_vmem_out),
    .refract_out   (lif_refract_out),
    .spike_cnt_out (lif_spike_cnt_out)
);

// =========================================================================
// Spike Output — per-neuron spike pulses (clk_phy domain)
// =========================================================================
// Demux lif_spike to per-neuron spike lines
reg [NUM_NEURONS-1:0] spike_pulse_phy;
integer si;
always @(posedge clk_phy) begin
    if (rst) begin
        spike_pulse_phy <= {NUM_NEURONS{1'b0}};
    end else begin
        // Clear all first, then set the one that fired
        spike_pulse_phy <= {NUM_NEURONS{1'b0}};
        if (lif_valid_out && lif_spike)
            spike_pulse_phy[lif_neuron_id_out] <= 1'b1;
    end
end

assign spike_out = spike_pulse_phy;

// =========================================================================
// Lateral Spike History — nearest-neighbor recurrent ring topology
// =========================================================================
// Each neuron connects to its 4 nearest neighbors on a ring (N±1, N±2).
// spike_history[N] records whether neuron N spiked in the last pipeline pass.
// This creates natural cross-neuron nonlinear mixing for reservoir computing.
// =========================================================================
reg lateral_spike [0:NUM_NEURONS-1];
integer li;
always @(posedge clk_phy) begin
    if (rst) begin
        for (li = 0; li < NUM_NEURONS; li = li + 1)
            lateral_spike[li] <= 1'b0;
    end else if (lif_valid_out) begin
        lateral_spike[lif_neuron_id_out] <= lif_spike;
    end
end

// Neighbor lookup based on neuron entering LIF pipeline
wire [NEURON_ADDR_W-1:0] lateral_center = aval_nid_out;
wire [3:0] lateral_spikes;
assign lateral_spikes[0] = lateral_spike[(lateral_center - 7'd1) & 7'h7F]; // N-1
assign lateral_spikes[1] = lateral_spike[(lateral_center + 7'd1) & 7'h7F]; // N+1
assign lateral_spikes[2] = lateral_spike[(lateral_center - 7'd2) & 7'h7F]; // N-2
assign lateral_spikes[3] = lateral_spike[(lateral_center + 7'd2) & 7'h7F]; // N+2

// =========================================================================
// Telemetry — CDC from clk_phy readout to clk_sys
// =========================================================================
// The bridge top drives neuron_sel and count_reset in clk_sys domain.
// We need to:
// 1. Pass neuron_sel → clk_phy (telem_nid_phy)
// 2. Read from scheduler telemetry port
// 3. Sync results back to clk_sys

// neuron_sel CDC to clk_phy (just 2-FF sync the index)
reg [NEURON_ADDR_W-1:0] nsel_phy0, nsel_phy1;
always @(posedge clk_phy) begin
    if (rst) begin
        nsel_phy0 <= {NEURON_ADDR_W{1'b0}};
        nsel_phy1 <= {NEURON_ADDR_W{1'b0}};
    end else begin
        nsel_phy0 <= neuron_sel;
        nsel_phy1 <= nsel_phy0;
    end
end

// Telemetry request: continuously read the selected neuron
// (one read per cycle is fine since scheduler supports it)
always @(posedge clk_phy) begin
    if (rst) begin
        telem_nid_phy <= {NEURON_ADDR_W{1'b0}};
        telem_req_phy <= 1'b0;
    end else begin
        telem_nid_phy <= nsel_phy1;
        telem_req_phy <= 1'b1; // always requesting
    end
end

// CDC: telemetry results back to clk_sys (2-FF)
reg [31:0] tm_vmem_sync0, tm_vmem_sync1;
reg [31:0] tm_bvpar_sync0, tm_bvpar_sync1;
reg [15:0] tm_scnt_sync0, tm_scnt_sync1;
reg [15:0] tm_refr_sync0, tm_refr_sync1;
reg [31:0] tm_vcb_sync0, tm_vcb_sync1;
reg [31:0] tm_vbulk_sync0, tm_vbulk_sync1;
reg        tm_kill_sync0, tm_kill_sync1;

// Track last i_aval from pipeline for debug
reg [31:0] last_i_aval_phy;
always @(posedge clk_phy) begin
    if (rst)
        last_i_aval_phy <= 32'd0;
    else if (lif_valid_out && lif_neuron_id_out == nsel_phy1)
        last_i_aval_phy <= aval_i_aval; // Note: aval output is 2 cycles ahead of lif
end

// Track last bvpar from pipeline for selected neuron
reg [31:0] last_bvpar_phy;
always @(posedge clk_phy) begin
    if (rst)
        last_bvpar_phy <= 32'd0;
    else if (aval_valid_out && aval_nid_out == nsel_phy1)
        last_bvpar_phy <= aval_bvpar;
end

always @(posedge clk_sys) begin
    if (rst) begin
        tm_vmem_sync0  <= 32'd0; tm_vmem_sync1  <= 32'd0;
        tm_bvpar_sync0 <= 32'd0; tm_bvpar_sync1 <= 32'd0;
        tm_scnt_sync0  <= 16'd0; tm_scnt_sync1  <= 16'd0;
        tm_refr_sync0  <= 16'd0; tm_refr_sync1  <= 16'd0;
        tm_vcb_sync0   <= 32'd0; tm_vcb_sync1   <= 32'd0;
        tm_vbulk_sync0 <= 32'd0; tm_vbulk_sync1 <= 32'd0;
        tm_kill_sync0  <= 1'b0;  tm_kill_sync1  <= 1'b0;
    end else begin
        tm_vmem_sync0  <= telem_vmem_phy;
        tm_vmem_sync1  <= tm_vmem_sync0;
        tm_bvpar_sync0 <= last_bvpar_phy;
        tm_bvpar_sync1 <= tm_bvpar_sync0;
        tm_scnt_sync0  <= telem_scnt_phy;
        tm_scnt_sync1  <= tm_scnt_sync0;
        tm_refr_sync0  <= telem_refr_phy;
        tm_refr_sync1  <= tm_refr_sync0;
        tm_vcb_sync0   <= vcb_killed;
        tm_vcb_sync1   <= tm_vcb_sync0;
        tm_vbulk_sync0 <= v_bulk;
        tm_vbulk_sync1 <= tm_vbulk_sync0;
        tm_kill_sync0  <= kill_phy;
        tm_kill_sync1  <= tm_kill_sync0;
    end
end

// Output mux
reg [31:0] iaval_sync0, iaval_sync1;
always @(posedge clk_sys) begin
    if (rst) begin
        iaval_sync0 <= 32'd0; iaval_sync1 <= 32'd0;
    end else begin
        iaval_sync0 <= last_i_aval_phy;
        iaval_sync1 <= iaval_sync0;
    end
end

always @(*) begin
    vmem_sel        = tm_vmem_sync1;
    bvpar_sel       = tm_bvpar_sync1;
    spike_count_sel = tm_scnt_sync1;
    refract_sel     = tm_refr_sync1;
    dbg_i_aval_sel  = iaval_sync1;
    dbg_vcb         = tm_vcb_sync1;
    dbg_v_bulk      = tm_vbulk_sync1;
    dbg_kill_phy    = tm_kill_sync1;
end

// =========================================================================
// Aggregate Spike Count (unused in 128-neuron mode, keep for compat)
// =========================================================================
assign spike_count_total = 16'd0; // aggregate not meaningful for 128 neurons

endmodule
