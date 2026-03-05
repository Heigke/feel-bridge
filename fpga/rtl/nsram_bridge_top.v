// =============================================================================
// nsram_bridge_top.v — Top-level NS-RAM FPGA Bridge for Arty A7-100T
// =============================================================================
// Connects UART host interface to NS-RAM neuron bank (128 time-multiplexed neurons).
// Clock domains: clk_sys (100 MHz), clk_phy (10 MHz).
// UART: 921600 baud 8N1.
//
// Telemetry packet (on READ_TELEMETRY command):
//   [0x55] [0x02] [LEN_HI] [LEN_LO] [per-neuron: spike_count(16) + vmem(16) + bvpar(16)] * 128 [CRC8]
//   Payload = 128 * 6 = 768 bytes. Total: 4 header + 768 payload + 1 CRC = 773 bytes
//
// Kill switch: UART CMD_KILL OR sw[0] (active high).
// Regime: sw[2:1].
// LED[0]: spike activity blink, LED[1]: kill active, LED[2]: heartbeat, LED[3]: RX ever.
// =============================================================================

module nsram_bridge_top #(
    parameter NUM_NEURONS   = 128,
    parameter NEURON_ADDR_W = 7
) (
    input  wire        CLK100MHZ,       // 100 MHz oscillator
    input  wire [3:0]  sw,              // switches
    input  wire [3:0]  btn,             // buttons (btn[0]=reset)
    input  wire        uart_txd_in,     // FTDI UART RX (host TX -> FPGA RX)
    output wire        uart_rxd_out,    // FTDI UART TX (FPGA TX -> host RX)
    output wire [3:0]  led,             // status LEDs
    output wire [11:0] led_rgb          // RGB LEDs for regime indication
);

// =========================================================================
// Reset synchronizer
// =========================================================================
wire rst_raw = btn[0];
reg [3:0] rst_pipe;
always @(posedge CLK100MHZ) begin
    rst_pipe <= {rst_pipe[2:0], rst_raw};
end
wire rst_btn = rst_pipe[3];

// =========================================================================
// Clock Divider: 100 MHz -> 10 MHz via MMCME2_BASE
// =========================================================================
wire clk_sys = CLK100MHZ;
wire clk_phy_mmcm;
wire mmcm_locked;
wire mmcm_fb;

MMCME2_BASE #(
    .BANDWIDTH          ("OPTIMIZED"),
    .CLKFBOUT_MULT_F    (10.0),
    .CLKFBOUT_PHASE     (0.0),
    .CLKIN1_PERIOD       (10.0),
    .CLKOUT0_DIVIDE_F   (100.0),
    .CLKOUT0_DUTY_CYCLE (0.5),
    .CLKOUT0_PHASE      (0.0),
    .DIVCLK_DIVIDE      (1),
    .REF_JITTER1        (0.010),
    .STARTUP_WAIT        ("FALSE")
) u_mmcm (
    .CLKFBOUT (mmcm_fb),
    .CLKOUT0  (clk_phy_mmcm),
    .CLKOUT1  (), .CLKOUT2 (), .CLKOUT3 (),
    .CLKOUT4  (), .CLKOUT5 (), .CLKOUT6 (),
    .CLKFBIN  (mmcm_fb),
    .CLKIN1   (CLK100MHZ),
    .LOCKED   (mmcm_locked),
    .PWRDWN   (1'b0),
    .RST      (rst_raw)
);

wire clk_phy;
BUFG u_bufg_phy (.I(clk_phy_mmcm), .O(clk_phy));

wire rst = rst_btn | ~mmcm_locked;

// =========================================================================
// UART RX
// =========================================================================
wire [7:0] rx_data;
wire       rx_valid;

uart_rx u_uart_rx (
    .clk   (clk_sys),
    .rst   (rst),
    .rx    (uart_txd_in),
    .data  (rx_data),
    .valid (rx_valid)
);

// =========================================================================
// UART TX
// =========================================================================
reg  [7:0] tx_data;
reg        tx_start;
wire       tx_busy;

uart_tx u_uart_tx (
    .clk   (clk_sys),
    .rst   (rst),
    .data  (tx_data),
    .start (tx_start),
    .tx    (uart_rxd_out),
    .busy  (tx_busy)
);

// =========================================================================
// Command Decoder
// =========================================================================
// Commands:
//   0x01 = SET_VG:           [neuron_id(8)] [vg(32 BE)] — 5 bytes payload
//   0x02 = READ_TELEMETRY:   no payload
//   0x03 = SET_KILL:         [enable(8)] — 1 byte payload
//   0x04 = SET_SYNAPSE:      [neuron_id(8)] [syn_w_packed(32 BE)] — 5 bytes
//   0x05 = SET_TEMP:         [temp_k(32 BE)] — 4 bytes
//   0x06 = SET_MAC:          [mac_signal(32 BE)] — 4 bytes
//   0x07 = READ_DEBUG:       no payload
//   0x08 = SET_VG_BATCH:     [start_id(8)] [count(8)] [vg0(32)..vgN(32)]

localparam CMD_SET_VG        = 8'h01;
localparam CMD_READ_TELEM    = 8'h02;
localparam CMD_SET_KILL      = 8'h03;
localparam CMD_SET_SYNAPSE   = 8'h04;
localparam CMD_SET_TEMP      = 8'h05;
localparam CMD_SET_MAC       = 8'h06;
localparam CMD_READ_DEBUG    = 8'h07;
localparam CMD_SET_VG_BATCH  = 8'h08;

// 5-bit state machine
reg [4:0] cd_state;
localparam CD5_IDLE      = 5'd0;
localparam CD5_CMD       = 5'd1;
localparam CD5_VG_SEL    = 5'd2;
localparam CD5_VG_D0     = 5'd3;
localparam CD5_VG_D1     = 5'd4;
localparam CD5_VG_D2     = 5'd5;
localparam CD5_VG_D3     = 5'd6;
localparam CD5_KILL      = 5'd7;
localparam CD5_SYN_SEL   = 5'd8;
localparam CD5_SYN_D0    = 5'd9;
localparam CD5_SYN_D1    = 5'd10;
localparam CD5_SYN_D2    = 5'd11;
localparam CD5_SYN_D3    = 5'd12;
localparam CD5_TEMP_D0   = 5'd13;
localparam CD5_TEMP_D1   = 5'd14;
localparam CD5_TEMP_D2   = 5'd15;
localparam CD5_TEMP_D3   = 5'd16;
localparam CD5_MAC_D0    = 5'd17;
localparam CD5_MAC_D1    = 5'd18;
localparam CD5_MAC_D2    = 5'd19;
localparam CD5_MAC_D3    = 5'd20;
localparam CD5_TELEM_REQ = 5'd21;
localparam CD5_BATCH_START = 5'd22;
localparam CD5_BATCH_CNT   = 5'd23;
localparam CD5_BATCH_D0    = 5'd24;
localparam CD5_BATCH_D1    = 5'd25;
localparam CD5_BATCH_D2    = 5'd26;
localparam CD5_BATCH_D3    = 5'd27;

reg [7:0]  cd_cmd;
reg [NEURON_ADDR_W-1:0] cd_neuron_sel;
reg [31:0] cd_data32;

// Batch state
reg [NEURON_ADDR_W-1:0] batch_start_id;
reg [7:0]               batch_count;
reg [7:0]               batch_idx;

// Neuron bank control registers
reg [31:0] reg_temp_k;
reg [31:0] reg_mac_signal;
reg        reg_kill_uart;
reg [NEURON_ADDR_W-1:0] reg_neuron_sel;
reg [31:0] reg_vg_write;
reg        reg_vg_wen;
reg [31:0] reg_syn_w_write;
reg        reg_syn_wen;
reg        telem_request;
reg        debug_request;

// RX activity tracking
reg        rx_ever_received;
reg [7:0]  rx_first_byte;

reg        loopback_active;

always @(posedge clk_sys) begin
    if (rst) begin
        rx_ever_received <= 1'b0;
        rx_first_byte    <= 8'd0;
    end else begin
        if (rx_valid && !rx_ever_received) begin
            rx_ever_received <= 1'b1;
            rx_first_byte    <= rx_data;
        end
    end
end

reg        crc_error_ever;

always @(posedge clk_sys) begin
    if (rst) begin
        cd_state       <= CD5_IDLE;
        cd_cmd         <= 8'd0;
        cd_neuron_sel  <= {NEURON_ADDR_W{1'b0}};
        cd_data32      <= 32'd0;
        reg_temp_k     <= 32'h012C_0000;
        reg_mac_signal <= 32'h0000_8000;
        reg_kill_uart  <= 1'b0;
        reg_vg_wen     <= 1'b0;
        reg_syn_wen    <= 1'b0;
        telem_request  <= 1'b0;
        debug_request  <= 1'b0;
        crc_error_ever <= 1'b0;
        batch_start_id <= {NEURON_ADDR_W{1'b0}};
        batch_count    <= 8'd0;
        batch_idx      <= 8'd0;
    end else begin
        reg_vg_wen    <= 1'b0;
        reg_syn_wen   <= 1'b0;
        telem_request <= 1'b0;
        debug_request <= 1'b0;

        if (rx_valid && !loopback_active) begin
            case (cd_state)
                CD5_IDLE: begin
                    if (rx_data == 8'h55)
                        cd_state <= CD5_CMD;
                end

                CD5_CMD: begin
                    cd_cmd <= rx_data;
                    case (rx_data)
                        CMD_READ_TELEM: begin
                            telem_request <= 1'b1;
                            cd_state      <= CD5_IDLE;
                        end
                        CMD_READ_DEBUG: begin
                            debug_request <= 1'b1;
                            cd_state      <= CD5_IDLE;
                        end
                        CMD_SET_VG:       cd_state <= CD5_VG_SEL;
                        CMD_SET_KILL:     cd_state <= CD5_KILL;
                        CMD_SET_SYNAPSE:  cd_state <= CD5_SYN_SEL;
                        CMD_SET_TEMP:     cd_state <= CD5_TEMP_D0;
                        CMD_SET_MAC:      cd_state <= CD5_MAC_D0;
                        CMD_SET_VG_BATCH: cd_state <= CD5_BATCH_START;
                        default:          cd_state <= CD5_IDLE;
                    endcase
                end

                // --- SET_VG: [neuron_id(8)] [D3] [D2] [D1] [D0] ---
                CD5_VG_SEL: begin
                    cd_neuron_sel <= rx_data[NEURON_ADDR_W-1:0];
                    cd_state      <= CD5_VG_D0;
                end
                CD5_VG_D0: begin cd_data32[31:24] <= rx_data; cd_state <= CD5_VG_D1; end
                CD5_VG_D1: begin cd_data32[23:16] <= rx_data; cd_state <= CD5_VG_D2; end
                CD5_VG_D2: begin cd_data32[15:8]  <= rx_data; cd_state <= CD5_VG_D3; end
                CD5_VG_D3: begin
                    cd_data32[7:0] <= rx_data;
                    reg_neuron_sel <= cd_neuron_sel;
                    reg_vg_write   <= {cd_data32[31:8], rx_data};
                    reg_vg_wen     <= 1'b1;
                    cd_state       <= CD5_IDLE;
                end

                // --- SET_KILL: [enable] ---
                CD5_KILL: begin
                    reg_kill_uart <= rx_data[0];
                    cd_state      <= CD5_IDLE;
                end

                // --- SET_SYNAPSE: [neuron_id(8)] [D3] [D2] [D1] [D0] ---
                // Now sends packed 4×8-bit weights as 32-bit value
                CD5_SYN_SEL: begin
                    cd_neuron_sel <= rx_data[NEURON_ADDR_W-1:0];
                    cd_state      <= CD5_SYN_D0;
                end
                CD5_SYN_D0: begin cd_data32[31:24] <= rx_data; cd_state <= CD5_SYN_D1; end
                CD5_SYN_D1: begin cd_data32[23:16] <= rx_data; cd_state <= CD5_SYN_D2; end
                CD5_SYN_D2: begin cd_data32[15:8]  <= rx_data; cd_state <= CD5_SYN_D3; end
                CD5_SYN_D3: begin
                    cd_data32[7:0]  <= rx_data;
                    reg_neuron_sel  <= cd_neuron_sel;
                    reg_syn_w_write <= {cd_data32[31:8], rx_data};
                    reg_syn_wen     <= 1'b1;
                    cd_state        <= CD5_IDLE;
                end

                // --- SET_TEMP: [D3] [D2] [D1] [D0] ---
                CD5_TEMP_D0: begin cd_data32[31:24] <= rx_data; cd_state <= CD5_TEMP_D1; end
                CD5_TEMP_D1: begin cd_data32[23:16] <= rx_data; cd_state <= CD5_TEMP_D2; end
                CD5_TEMP_D2: begin cd_data32[15:8]  <= rx_data; cd_state <= CD5_TEMP_D3; end
                CD5_TEMP_D3: begin
                    reg_temp_k <= {cd_data32[31:8], rx_data};
                    cd_state   <= CD5_IDLE;
                end

                // --- SET_MAC: [D3] [D2] [D1] [D0] ---
                CD5_MAC_D0: begin cd_data32[31:24] <= rx_data; cd_state <= CD5_MAC_D1; end
                CD5_MAC_D1: begin cd_data32[23:16] <= rx_data; cd_state <= CD5_MAC_D2; end
                CD5_MAC_D2: begin cd_data32[15:8]  <= rx_data; cd_state <= CD5_MAC_D3; end
                CD5_MAC_D3: begin
                    reg_mac_signal <= {cd_data32[31:8], rx_data};
                    cd_state       <= CD5_IDLE;
                end

                // --- SET_VG_BATCH: [start_id] [count] [vg0_D3..D0] ... ---
                CD5_BATCH_START: begin
                    batch_start_id <= rx_data[NEURON_ADDR_W-1:0];
                    cd_state       <= CD5_BATCH_CNT;
                end
                CD5_BATCH_CNT: begin
                    batch_count <= rx_data;
                    batch_idx   <= 8'd0;
                    cd_state    <= CD5_BATCH_D0;
                end
                CD5_BATCH_D0: begin cd_data32[31:24] <= rx_data; cd_state <= CD5_BATCH_D1; end
                CD5_BATCH_D1: begin cd_data32[23:16] <= rx_data; cd_state <= CD5_BATCH_D2; end
                CD5_BATCH_D2: begin cd_data32[15:8]  <= rx_data; cd_state <= CD5_BATCH_D3; end
                CD5_BATCH_D3: begin
                    cd_data32[7:0] <= rx_data;
                    reg_neuron_sel <= batch_start_id + batch_idx[NEURON_ADDR_W-1:0];
                    reg_vg_write   <= {cd_data32[31:8], rx_data};
                    reg_vg_wen     <= 1'b1;
                    batch_idx      <= batch_idx + 8'd1;
                    if (batch_idx + 8'd1 >= batch_count)
                        cd_state <= CD5_IDLE;
                    else
                        cd_state <= CD5_BATCH_D0;
                end

                default: cd_state <= CD5_IDLE;
            endcase
        end
    end
end

// =========================================================================
// Kill Switch: OR of UART command and sw[0]
// =========================================================================
wire kill_switch = reg_kill_uart | sw[0];

// =========================================================================
// Regime from switches: sw[2:1]
// =========================================================================
wire [1:0] regime = sw[2:1];

// =========================================================================
// Neuron Bank
// =========================================================================
wire [NUM_NEURONS-1:0] nb_spike_out;
wire [15:0] nb_spike_count_total;
wire [31:0] nb_vmem_sel;
wire [31:0] nb_bvpar_sel;
wire [15:0] nb_spike_count_sel;
wire [31:0] nb_dbg_i_aval;
wire [31:0] nb_dbg_vcb;
wire [31:0] nb_dbg_v_bulk;
wire        nb_dbg_kill_phy;
reg         nb_count_reset;

wire [NEURON_ADDR_W-1:0] bank_neuron_sel;

nsram_neuron_bank #(
    .NUM_NEURONS   (NUM_NEURONS),
    .NEURON_ADDR_W (NEURON_ADDR_W)
) u_neuron_bank (
    .clk_sys          (clk_sys),
    .clk_phy          (clk_phy),
    .rst              (rst),
    .kill_switch      (kill_switch),
    .temp_k           (reg_temp_k),
    .mac_signal       (reg_mac_signal),
    .regime           (regime),
    .neuron_sel       (bank_neuron_sel),
    .vg_write         (reg_vg_write),
    .vg_wen           (reg_vg_wen),
    .syn_w_write      (reg_syn_w_write),
    .syn_wen          (reg_syn_wen),
    .spike_out        (nb_spike_out),
    .spike_count_total(nb_spike_count_total),
    .vmem_sel         (nb_vmem_sel),
    .bvpar_sel        (nb_bvpar_sel),
    .spike_count_sel  (nb_spike_count_sel),
    .count_reset      (nb_count_reset),
    .dbg_i_aval_sel   (nb_dbg_i_aval),
    .dbg_vcb          (nb_dbg_vcb),
    .dbg_v_bulk       (nb_dbg_v_bulk),
    .dbg_kill_phy     (nb_dbg_kill_phy)
);

// =========================================================================
// Telemetry Transmitter State Machine
// =========================================================================
// On telem_request: latch spike counts, then serialize 773-byte packet.
//
// Packet: [0x55] [0x02] [LEN_HI] [LEN_LO] [neuron0..127 data] [CRC8]
// Per-neuron: spike_count[15:8], spike_count[7:0],
//             vmem[15:8], vmem[7:0],
//             bvpar[31:24], bvpar[23:16]

localparam TM_IDLE       = 4'd0;
localparam TM_LATCH      = 4'd1;
localparam TM_WAIT       = 4'd2;
localparam TM_HDR0       = 4'd3;
localparam TM_HDR1       = 4'd4;
localparam TM_HDR2       = 4'd5;
localparam TM_HDR3       = 4'd6;
localparam TM_NEURON     = 4'd7;
localparam TM_CRC        = 4'd8;
localparam TM_TX_WAIT    = 4'd9;
localparam TM_BEACON     = 4'd10;
localparam TM_BEACON_W   = 4'd11;
localparam TM_LOOPBACK   = 4'd12;
localparam TM_LOOPBACK_W = 4'd13;
localparam TM_DEBUG      = 4'd14;
localparam TM_SCAN       = 4'd15;

reg [3:0]  tm_state;
reg [3:0]  tm_next_state;
reg [NEURON_ADDR_W-1:0] tm_neuron_idx;
reg [2:0]  tm_byte_idx;
reg [7:0]  tm_crc;
reg [7:0]  tm_wait_cnt;
reg [1:0]  beacon_idx;

// Snapshot storage for telemetry scan
// Use BRAM for 128 neurons of telemetry data
reg [15:0] tm_spike_cnt [0:NUM_NEURONS-1];
reg [31:0] tm_vmem      [0:NUM_NEURONS-1];
reg [31:0] tm_bvpar     [0:NUM_NEURONS-1];
reg [NEURON_ADDR_W-1:0] tm_scan_idx;
reg        tm_scanning;

// Debug probe snapshot
reg [31:0] dbg_snap_iaval;
reg [31:0] dbg_snap_vcb;
reg [31:0] dbg_snap_vbulk;
reg [7:0]  dbg_snap_flags;

// Debug state — separate counter for debug FSM
reg [4:0]  dbg_byte_cnt;

// CRC-8 (polynomial x^8 + x^2 + x + 1, 0x07, init 0x00)
function [7:0] crc8_byte;
    input [7:0] crc_in;
    input [7:0] data_in;
    reg [7:0] crc_tmp;
    integer b;
    begin
        crc_tmp = crc_in ^ data_in;
        for (b = 0; b < 8; b = b + 1) begin
            if (crc_tmp[7])
                crc_tmp = {crc_tmp[6:0], 1'b0} ^ 8'h07;
            else
                crc_tmp = {crc_tmp[6:0], 1'b0};
        end
        crc8_byte = crc_tmp;
    end
endfunction

// Payload length: 128 * 6 = 768 bytes
localparam [15:0] PAYLOAD_LEN = 16'd768;

always @(posedge clk_sys) begin
    if (rst) begin
        tm_state       <= TM_BEACON;
        tm_next_state  <= TM_IDLE;
        tm_neuron_idx  <= {NEURON_ADDR_W{1'b0}};
        tm_byte_idx    <= 3'd0;
        tm_crc         <= 8'd0;
        tm_wait_cnt    <= 8'd0;
        tm_scanning    <= 1'b0;
        tm_scan_idx    <= {NEURON_ADDR_W{1'b0}};
        beacon_idx     <= 2'd0;
        loopback_active <= 1'b0;
        nb_count_reset <= 1'b0;
        tx_start       <= 1'b0;
        tx_data        <= 8'd0;
        dbg_byte_cnt   <= 5'd0;
    end else begin
        tx_start       <= 1'b0;
        nb_count_reset <= 1'b0;

        case (tm_state)
            TM_BEACON: begin
                if (!tx_busy && !tx_start) begin
                    case (beacon_idx)
                        2'd0: tx_data <= 8'h42;  // 'B'
                        2'd1: tx_data <= 8'h4F;  // 'O'
                        2'd2: tx_data <= 8'h4B;  // 'K'
                        2'd3: tx_data <= 8'h0A;  // '\n'
                    endcase
                    tx_start <= 1'b1;
                    tm_state <= TM_BEACON_W;
                end
            end

            TM_BEACON_W: begin
                if (!tx_busy && !tx_start) begin
                    if (beacon_idx == 2'd3)
                        tm_state <= TM_IDLE;
                    else begin
                        beacon_idx <= beacon_idx + 2'd1;
                        tm_state   <= TM_BEACON;
                    end
                end
            end

            TM_LOOPBACK: begin
                if (rx_valid) begin
                    tx_data  <= rx_data ^ 8'h80;
                    tx_start <= 1'b1;
                    tm_state <= TM_LOOPBACK_W;
                end
                if (sw[3]) begin
                    tm_state        <= TM_IDLE;
                    loopback_active <= 1'b0;
                end
            end

            TM_LOOPBACK_W: begin
                if (!tx_busy && !tx_start)
                    tm_state <= TM_LOOPBACK;
            end

            TM_IDLE: begin
                if (telem_request) begin
                    tm_state <= TM_LATCH;
                end else if (debug_request) begin
                    dbg_snap_iaval <= nb_dbg_i_aval;
                    dbg_snap_vcb   <= nb_dbg_vcb;
                    dbg_snap_vbulk <= nb_dbg_v_bulk;
                    dbg_snap_flags <= {7'd0, nb_dbg_kill_phy};
                    tm_crc         <= 8'd0;
                    dbg_byte_cnt   <= 5'd0;
                    tm_state       <= TM_DEBUG;
                end
            end

            TM_LATCH: begin
                // Start scan — reset happens AFTER all counts are captured
                tm_scanning    <= 1'b1;
                tm_scan_idx    <= {NEURON_ADDR_W{1'b0}};
                tm_wait_cnt    <= 8'd0;
                tm_state       <= TM_SCAN;
            end

            // Scan all 128 neurons: cycle neuron_sel, capture vmem/bvpar/spike_cnt
            // CDC round-trip: neuron_sel → 2FF@clk_phy → scheduler → 2FF@clk_sys
            // Needs ~42 clk_sys cycles minimum. Using 50 for margin.
            TM_SCAN: begin
                tm_wait_cnt    <= tm_wait_cnt + 8'd1;

                if (tm_wait_cnt == 8'd49) begin
                    // CDC settled: capture telemetry snapshot
                    tm_vmem[tm_scan_idx]      <= nb_vmem_sel;
                    tm_bvpar[tm_scan_idx]     <= nb_bvpar_sel;
                    tm_spike_cnt[tm_scan_idx] <= nb_spike_count_sel;
                    tm_wait_cnt               <= 8'd0;

                    if (tm_scan_idx == NUM_NEURONS - 1) begin
                        // All counts captured — NOW reset spike counters
                        nb_count_reset <= 1'b1;
                        tm_scanning    <= 1'b0;
                        tm_state       <= TM_HDR0;
                        tm_crc         <= 8'd0;
                        tm_neuron_idx  <= {NEURON_ADDR_W{1'b0}};
                        tm_byte_idx    <= 3'd0;
                    end else begin
                        tm_scan_idx <= tm_scan_idx + 1;
                    end
                end
            end

            TM_DEBUG: begin
                if (!tx_busy && !tx_start) begin
                    case (dbg_byte_cnt)
                        5'd0:  tx_data <= 8'h55;
                        5'd1:  tx_data <= 8'h07;
                        5'd2:  tx_data <= 8'h0D;
                        5'd3:  tx_data <= dbg_snap_iaval[31:24];
                        5'd4:  tx_data <= dbg_snap_iaval[23:16];
                        5'd5:  tx_data <= dbg_snap_iaval[15:8];
                        5'd6:  tx_data <= dbg_snap_iaval[7:0];
                        5'd7:  tx_data <= dbg_snap_vcb[31:24];
                        5'd8:  tx_data <= dbg_snap_vcb[23:16];
                        5'd9:  tx_data <= dbg_snap_vcb[15:8];
                        5'd10: tx_data <= dbg_snap_vcb[7:0];
                        5'd11: tx_data <= dbg_snap_vbulk[31:24];
                        5'd12: tx_data <= dbg_snap_vbulk[23:16];
                        5'd13: tx_data <= dbg_snap_vbulk[15:8];
                        5'd14: tx_data <= dbg_snap_vbulk[7:0];
                        5'd15: tx_data <= dbg_snap_flags;
                        5'd16: tx_data <= tm_crc;
                        default: tx_data <= 8'd0;
                    endcase

                    if (dbg_byte_cnt < 5'd16) begin
                        case (dbg_byte_cnt)
                            5'd0:  tm_crc <= crc8_byte(tm_crc, 8'h55);
                            5'd1:  tm_crc <= crc8_byte(tm_crc, 8'h07);
                            5'd2:  tm_crc <= crc8_byte(tm_crc, 8'h0D);
                            5'd3:  tm_crc <= crc8_byte(tm_crc, dbg_snap_iaval[31:24]);
                            5'd4:  tm_crc <= crc8_byte(tm_crc, dbg_snap_iaval[23:16]);
                            5'd5:  tm_crc <= crc8_byte(tm_crc, dbg_snap_iaval[15:8]);
                            5'd6:  tm_crc <= crc8_byte(tm_crc, dbg_snap_iaval[7:0]);
                            5'd7:  tm_crc <= crc8_byte(tm_crc, dbg_snap_vcb[31:24]);
                            5'd8:  tm_crc <= crc8_byte(tm_crc, dbg_snap_vcb[23:16]);
                            5'd9:  tm_crc <= crc8_byte(tm_crc, dbg_snap_vcb[15:8]);
                            5'd10: tm_crc <= crc8_byte(tm_crc, dbg_snap_vcb[7:0]);
                            5'd11: tm_crc <= crc8_byte(tm_crc, dbg_snap_vbulk[31:24]);
                            5'd12: tm_crc <= crc8_byte(tm_crc, dbg_snap_vbulk[23:16]);
                            5'd13: tm_crc <= crc8_byte(tm_crc, dbg_snap_vbulk[15:8]);
                            5'd14: tm_crc <= crc8_byte(tm_crc, dbg_snap_vbulk[7:0]);
                            5'd15: tm_crc <= crc8_byte(tm_crc, dbg_snap_flags);
                            default: ;
                        endcase
                    end

                    tx_start <= 1'b1;

                    if (dbg_byte_cnt == 5'd16) begin
                        tm_state      <= TM_TX_WAIT;
                        tm_next_state <= TM_IDLE;
                        dbg_byte_cnt  <= 5'd0;
                    end else begin
                        dbg_byte_cnt  <= dbg_byte_cnt + 5'd1;
                        tm_state      <= TM_TX_WAIT;
                        tm_next_state <= TM_DEBUG;
                    end
                end
            end

            TM_HDR0: begin
                if (!tx_busy) begin
                    tx_data       <= 8'h55;
                    tx_start      <= 1'b1;
                    tm_crc        <= crc8_byte(tm_crc, 8'h55);
                    tm_state      <= TM_TX_WAIT;
                    tm_next_state <= TM_HDR1;
                end
            end

            TM_HDR1: begin
                if (!tx_busy) begin
                    tx_data       <= 8'h02;
                    tx_start      <= 1'b1;
                    tm_crc        <= crc8_byte(tm_crc, 8'h02);
                    tm_state      <= TM_TX_WAIT;
                    tm_next_state <= TM_HDR2;
                end
            end

            TM_HDR2: begin
                if (!tx_busy) begin
                    tx_data       <= PAYLOAD_LEN[15:8]; // 0x03
                    tx_start      <= 1'b1;
                    tm_crc        <= crc8_byte(tm_crc, PAYLOAD_LEN[15:8]);
                    tm_state      <= TM_TX_WAIT;
                    tm_next_state <= TM_HDR3;
                end
            end

            TM_HDR3: begin
                if (!tx_busy) begin
                    tx_data       <= PAYLOAD_LEN[7:0]; // 0x00
                    tx_start      <= 1'b1;
                    tm_crc        <= crc8_byte(tm_crc, PAYLOAD_LEN[7:0]);
                    tm_state      <= TM_TX_WAIT;
                    tm_next_state <= TM_NEURON;
                end
            end

            TM_NEURON: begin
                if (!tx_busy) begin
                    case (tm_byte_idx)
                        3'd0: tx_data <= tm_spike_cnt[tm_neuron_idx][15:8];
                        3'd1: tx_data <= tm_spike_cnt[tm_neuron_idx][7:0];
                        3'd2: tx_data <= tm_vmem[tm_neuron_idx][15:8];
                        3'd3: tx_data <= tm_vmem[tm_neuron_idx][7:0];
                        3'd4: tx_data <= tm_bvpar[tm_neuron_idx][31:24];
                        3'd5: tx_data <= tm_bvpar[tm_neuron_idx][23:16];
                        default: tx_data <= 8'd0;
                    endcase

                    tx_start <= 1'b1;

                    case (tm_byte_idx)
                        3'd0: tm_crc <= crc8_byte(tm_crc, tm_spike_cnt[tm_neuron_idx][15:8]);
                        3'd1: tm_crc <= crc8_byte(tm_crc, tm_spike_cnt[tm_neuron_idx][7:0]);
                        3'd2: tm_crc <= crc8_byte(tm_crc, tm_vmem[tm_neuron_idx][15:8]);
                        3'd3: tm_crc <= crc8_byte(tm_crc, tm_vmem[tm_neuron_idx][7:0]);
                        3'd4: tm_crc <= crc8_byte(tm_crc, tm_bvpar[tm_neuron_idx][31:24]);
                        3'd5: tm_crc <= crc8_byte(tm_crc, tm_bvpar[tm_neuron_idx][23:16]);
                        default: tm_crc <= crc8_byte(tm_crc, 8'd0);
                    endcase

                    if (tm_byte_idx == 3'd5) begin
                        tm_byte_idx <= 3'd0;
                        if (tm_neuron_idx == NUM_NEURONS - 1) begin
                            tm_state      <= TM_TX_WAIT;
                            tm_next_state <= TM_CRC;
                        end else begin
                            tm_neuron_idx <= tm_neuron_idx + 1;
                            tm_state      <= TM_TX_WAIT;
                            tm_next_state <= TM_NEURON;
                        end
                    end else begin
                        tm_byte_idx   <= tm_byte_idx + 3'd1;
                        tm_state      <= TM_TX_WAIT;
                        tm_next_state <= TM_NEURON;
                    end
                end
            end

            TM_CRC: begin
                if (!tx_busy) begin
                    tx_data       <= tm_crc;
                    tx_start      <= 1'b1;
                    tm_state      <= TM_TX_WAIT;
                    tm_next_state <= TM_IDLE;
                end
            end

            TM_TX_WAIT: begin
                if (tx_busy || tx_start) begin
                    // wait
                end else begin
                    tm_state <= tm_next_state;
                end
            end

            default: tm_state <= TM_IDLE;
        endcase
    end
end

// =========================================================================
// Neuron Bank neuron_sel mux (override for telemetry scan)
// =========================================================================
assign bank_neuron_sel = tm_scanning ? tm_scan_idx : reg_neuron_sel;

// =========================================================================
// LED Drivers
// =========================================================================

// LED[0]: Spike activity — blink on any spike (OR of all neurons)
reg [21:0] spike_blink_cnt;
reg        spike_blink;

always @(posedge clk_sys) begin
    if (rst) begin
        spike_blink_cnt <= 22'd0;
        spike_blink     <= 1'b0;
    end else begin
        if (|nb_spike_out) begin
            spike_blink_cnt <= 22'd3_000_000;
            spike_blink     <= 1'b1;
        end else if (spike_blink_cnt != 22'd0) begin
            spike_blink_cnt <= spike_blink_cnt - 22'd1;
        end else begin
            spike_blink <= 1'b0;
        end
    end
end

// LED[1]: Kill switch active
wire led_kill = kill_switch;

// LED[2]: Heartbeat — 1 Hz toggle
reg [26:0] hb_counter;
reg        hb_led;

always @(posedge clk_sys) begin
    if (rst) begin
        hb_counter <= 27'd0;
        hb_led     <= 1'b0;
    end else begin
        if (hb_counter == 27'd49_999_999) begin
            hb_counter <= 27'd0;
            hb_led     <= ~hb_led;
        end else begin
            hb_counter <= hb_counter + 27'd1;
        end
    end
end

// LED[3]: RX ever received
assign led[0] = spike_blink;
assign led[1] = led_kill;
assign led[2] = hb_led;
assign led[3] = rx_ever_received;

// =========================================================================
// RGB LEDs — Regime Indication
// =========================================================================
wire [2:0] regime_color = (regime == 2'd0) ? 3'b001 :
                          (regime == 2'd1) ? 3'b100 :
                          (regime == 2'd2) ? 3'b101 :
                                             3'b010;

assign led_rgb = {regime_color, regime_color, regime_color, regime_color};

endmodule
