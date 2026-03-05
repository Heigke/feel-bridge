// =============================================================================
// nsram_eth_top.v — NS-RAM FPGA Bridge with Ethernet + UART
// =============================================================================
// Adds UDP/IP Ethernet interface alongside existing UART for high-speed
// neuron telemetry and command interface.
//
// Ethernet: 100BASE-T MII via on-board DP83848 PHY
// UDP port 7700: commands + telemetry (same protocol as UART)
// UDP port 7701: auto-telemetry (FPGA pushes telemetry at configurable rate)
//
// UART is kept for backward compatibility and debugging.
// =============================================================================

`resetall
`timescale 1ns / 1ps
`default_nettype none

module nsram_eth_top #(
    parameter NUM_NEURONS   = 128,
    parameter NEURON_ADDR_W = 7
) (
    input  wire        CLK100MHZ,
    input  wire [3:0]  sw,
    input  wire [3:0]  btn,
    // UART
    input  wire        uart_txd_in,
    output wire        uart_rxd_out,
    // Ethernet MII PHY
    output wire        phy_ref_clk,
    input  wire        phy_rx_clk,
    input  wire [3:0]  phy_rxd,
    input  wire        phy_rx_dv,
    input  wire        phy_rx_er,
    input  wire        phy_tx_clk,
    output wire [3:0]  phy_txd,
    output wire        phy_tx_en,
    input  wire        phy_col,
    input  wire        phy_crs,
    output wire        phy_reset_n,
    // LEDs
    output wire [3:0]  led,
    output wire [11:0] led_rgb
);

// =========================================================================
// Reset synchronizer
// =========================================================================
wire rst_raw = btn[0];
reg [3:0] rst_pipe;
always @(posedge CLK100MHZ)
    rst_pipe <= {rst_pipe[2:0], rst_raw};
wire rst_btn = rst_pipe[3];

// =========================================================================
// Clock generation
// =========================================================================
// MMCM: 100 MHz → 125 MHz (Ethernet logic) + 25 MHz (PHY ref) + 10 MHz (physics)
wire clk_sys = CLK100MHZ;
wire clk_125mhz_mmcm, clk_25mhz_mmcm, clk_10mhz_mmcm;
wire mmcm_locked, mmcm_fb;

MMCME2_BASE #(
    .BANDWIDTH          ("OPTIMIZED"),
    .CLKFBOUT_MULT_F    (10.0),      // VCO = 100 * 10 = 1000 MHz
    .CLKFBOUT_PHASE     (0.0),
    .CLKIN1_PERIOD       (10.0),     // 100 MHz
    .CLKOUT0_DIVIDE_F   (8.0),      // 1000/8 = 125 MHz (Ethernet)
    .CLKOUT0_DUTY_CYCLE (0.5),
    .CLKOUT0_PHASE      (0.0),
    .CLKOUT1_DIVIDE     (40),       // 1000/40 = 25 MHz (PHY ref)
    .CLKOUT1_DUTY_CYCLE (0.5),
    .CLKOUT1_PHASE      (0.0),
    .CLKOUT2_DIVIDE     (100),      // 1000/100 = 10 MHz (physics)
    .CLKOUT2_DUTY_CYCLE (0.5),
    .CLKOUT2_PHASE      (0.0),
    .DIVCLK_DIVIDE      (1),
    .REF_JITTER1        (0.010),
    .STARTUP_WAIT        ("FALSE")
) u_mmcm (
    .CLKFBOUT (mmcm_fb),
    .CLKOUT0  (clk_125mhz_mmcm),
    .CLKOUT1  (clk_25mhz_mmcm),
    .CLKOUT2  (clk_10mhz_mmcm),
    .CLKOUT3  (), .CLKOUT4 (), .CLKOUT5 (), .CLKOUT6 (),
    .CLKFBIN  (mmcm_fb),
    .CLKIN1   (CLK100MHZ),
    .LOCKED   (mmcm_locked),
    .PWRDWN   (1'b0),
    .RST      (rst_raw)
);

wire clk_125mhz, clk_25mhz, clk_phy;
BUFG u_bufg_125 (.I(clk_125mhz_mmcm), .O(clk_125mhz));
BUFG u_bufg_25  (.I(clk_25mhz_mmcm),  .O(clk_25mhz));
BUFG u_bufg_phy (.I(clk_10mhz_mmcm),  .O(clk_phy));

wire rst = rst_btn | ~mmcm_locked;

// PHY reference clock = 25 MHz
assign phy_ref_clk = clk_25mhz;

// =========================================================================
// Synchronized reset for 125 MHz domain
// =========================================================================
reg [3:0] rst_125_pipe;
always @(posedge clk_125mhz)
    rst_125_pipe <= {rst_125_pipe[2:0], rst};
wire rst_125 = rst_125_pipe[3];

// =========================================================================
// UART RX/TX (100 MHz domain, directly on CLK100MHZ)
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
// Ethernet MAC + UDP/IP stack (125 MHz domain)
// =========================================================================
// AXI between MAC and Ethernet modules
wire [7:0] mac_rx_axis_tdata;
wire mac_rx_axis_tvalid, mac_rx_axis_tready, mac_rx_axis_tlast, mac_rx_axis_tuser;
wire [7:0] mac_tx_axis_tdata;
wire mac_tx_axis_tvalid, mac_tx_axis_tready, mac_tx_axis_tlast, mac_tx_axis_tuser;

// Ethernet frame wires
wire rx_eth_hdr_ready, rx_eth_hdr_valid;
wire [47:0] rx_eth_dest_mac, rx_eth_src_mac;
wire [15:0] rx_eth_type;
wire [7:0] rx_eth_payload_axis_tdata;
wire rx_eth_payload_axis_tvalid, rx_eth_payload_axis_tready;
wire rx_eth_payload_axis_tlast, rx_eth_payload_axis_tuser;

wire tx_eth_hdr_ready, tx_eth_hdr_valid;
wire [47:0] tx_eth_dest_mac, tx_eth_src_mac;
wire [15:0] tx_eth_type;
wire [7:0] tx_eth_payload_axis_tdata;
wire tx_eth_payload_axis_tvalid, tx_eth_payload_axis_tready;
wire tx_eth_payload_axis_tlast, tx_eth_payload_axis_tuser;

// IP frame wires
wire rx_ip_hdr_valid, rx_ip_hdr_ready;
wire [47:0] rx_ip_eth_dest_mac, rx_ip_eth_src_mac;
wire [15:0] rx_ip_eth_type;
wire [3:0] rx_ip_version, rx_ip_ihl;
wire [5:0] rx_ip_dscp;
wire [1:0] rx_ip_ecn;
wire [15:0] rx_ip_length, rx_ip_identification;
wire [2:0] rx_ip_flags;
wire [12:0] rx_ip_fragment_offset;
wire [7:0] rx_ip_ttl, rx_ip_protocol;
wire [15:0] rx_ip_header_checksum;
wire [31:0] rx_ip_source_ip, rx_ip_dest_ip;
wire [7:0] rx_ip_payload_axis_tdata;
wire rx_ip_payload_axis_tvalid, rx_ip_payload_axis_tready;
wire rx_ip_payload_axis_tlast, rx_ip_payload_axis_tuser;

// UDP frame wires
wire rx_udp_hdr_valid, rx_udp_hdr_ready;
wire [47:0] rx_udp_eth_dest_mac, rx_udp_eth_src_mac;
wire [15:0] rx_udp_eth_type;
wire [3:0] rx_udp_ip_version, rx_udp_ip_ihl;
wire [5:0] rx_udp_ip_dscp;
wire [1:0] rx_udp_ip_ecn;
wire [15:0] rx_udp_ip_length, rx_udp_ip_identification;
wire [2:0] rx_udp_ip_flags;
wire [12:0] rx_udp_ip_fragment_offset;
wire [7:0] rx_udp_ip_ttl, rx_udp_ip_protocol;
wire [15:0] rx_udp_ip_header_checksum;
wire [31:0] rx_udp_ip_source_ip, rx_udp_ip_dest_ip;
wire [15:0] rx_udp_source_port, rx_udp_dest_port;
wire [15:0] rx_udp_length, rx_udp_checksum;
wire [7:0] rx_udp_payload_axis_tdata;
wire rx_udp_payload_axis_tvalid, rx_udp_payload_axis_tready;
wire rx_udp_payload_axis_tlast, rx_udp_payload_axis_tuser;

wire tx_udp_hdr_valid, tx_udp_hdr_ready;
wire [7:0] tx_udp_payload_axis_tdata;
wire tx_udp_payload_axis_tvalid, tx_udp_payload_axis_tready;
wire tx_udp_payload_axis_tlast, tx_udp_payload_axis_tuser;

// Configuration
wire [47:0] local_mac   = 48'h02_00_00_00_00_01;    // locally administered
wire [31:0] local_ip    = {8'd192, 8'd168, 8'd0, 8'd50};  // 192.168.0.50
wire [31:0] gateway_ip  = {8'd192, 8'd168, 8'd0, 8'd1};
wire [31:0] subnet_mask = {8'd255, 8'd255, 8'd255, 8'd0};

// IP ports not used (raw IP)
assign rx_ip_hdr_ready = 1;
assign rx_ip_payload_axis_tready = 1;

// PHY reset
assign phy_reset_n = !rst_125;

// ---- Ethernet MAC (MII, 100 Mbps) ----
eth_mac_mii_fifo #(
    .TARGET("XILINX"),
    .CLOCK_INPUT_STYLE("BUFR"),
    .ENABLE_PADDING(1),
    .MIN_FRAME_LENGTH(64),
    .TX_FIFO_DEPTH(4096),
    .TX_FRAME_FIFO(1),
    .RX_FIFO_DEPTH(4096),
    .RX_FRAME_FIFO(1)
) u_eth_mac (
    .rst(rst_125),
    .logic_clk(clk_125mhz),
    .logic_rst(rst_125),
    .tx_axis_tdata(mac_tx_axis_tdata),
    .tx_axis_tvalid(mac_tx_axis_tvalid),
    .tx_axis_tready(mac_tx_axis_tready),
    .tx_axis_tlast(mac_tx_axis_tlast),
    .tx_axis_tuser(mac_tx_axis_tuser),
    .rx_axis_tdata(mac_rx_axis_tdata),
    .rx_axis_tvalid(mac_rx_axis_tvalid),
    .rx_axis_tready(mac_rx_axis_tready),
    .rx_axis_tlast(mac_rx_axis_tlast),
    .rx_axis_tuser(mac_rx_axis_tuser),
    .mii_rx_clk(phy_rx_clk),
    .mii_rxd(phy_rxd),
    .mii_rx_dv(phy_rx_dv),
    .mii_rx_er(phy_rx_er),
    .mii_tx_clk(phy_tx_clk),
    .mii_txd(phy_txd),
    .mii_tx_en(phy_tx_en),
    .mii_tx_er(),
    .tx_fifo_overflow(),
    .tx_fifo_bad_frame(),
    .tx_fifo_good_frame(),
    .rx_error_bad_frame(),
    .rx_error_bad_fcs(),
    .rx_fifo_overflow(),
    .rx_fifo_bad_frame(),
    .rx_fifo_good_frame(),
    .cfg_ifg(8'd12),
    .cfg_tx_enable(1'b1),
    .cfg_rx_enable(1'b1)
);

// ---- Ethernet frame RX ----
eth_axis_rx u_eth_rx (
    .clk(clk_125mhz), .rst(rst_125),
    .s_axis_tdata(mac_rx_axis_tdata),
    .s_axis_tvalid(mac_rx_axis_tvalid),
    .s_axis_tready(mac_rx_axis_tready),
    .s_axis_tlast(mac_rx_axis_tlast),
    .s_axis_tuser(mac_rx_axis_tuser),
    .m_eth_hdr_valid(rx_eth_hdr_valid),
    .m_eth_hdr_ready(rx_eth_hdr_ready),
    .m_eth_dest_mac(rx_eth_dest_mac),
    .m_eth_src_mac(rx_eth_src_mac),
    .m_eth_type(rx_eth_type),
    .m_eth_payload_axis_tdata(rx_eth_payload_axis_tdata),
    .m_eth_payload_axis_tvalid(rx_eth_payload_axis_tvalid),
    .m_eth_payload_axis_tready(rx_eth_payload_axis_tready),
    .m_eth_payload_axis_tlast(rx_eth_payload_axis_tlast),
    .m_eth_payload_axis_tuser(rx_eth_payload_axis_tuser),
    .busy(), .error_header_early_termination()
);

// ---- Ethernet frame TX ----
eth_axis_tx u_eth_tx (
    .clk(clk_125mhz), .rst(rst_125),
    .s_eth_hdr_valid(tx_eth_hdr_valid),
    .s_eth_hdr_ready(tx_eth_hdr_ready),
    .s_eth_dest_mac(tx_eth_dest_mac),
    .s_eth_src_mac(tx_eth_src_mac),
    .s_eth_type(tx_eth_type),
    .s_eth_payload_axis_tdata(tx_eth_payload_axis_tdata),
    .s_eth_payload_axis_tvalid(tx_eth_payload_axis_tvalid),
    .s_eth_payload_axis_tready(tx_eth_payload_axis_tready),
    .s_eth_payload_axis_tlast(tx_eth_payload_axis_tlast),
    .s_eth_payload_axis_tuser(tx_eth_payload_axis_tuser),
    .m_axis_tdata(mac_tx_axis_tdata),
    .m_axis_tvalid(mac_tx_axis_tvalid),
    .m_axis_tready(mac_tx_axis_tready),
    .m_axis_tlast(mac_tx_axis_tlast),
    .m_axis_tuser(mac_tx_axis_tuser),
    .busy()
);

// ---- UDP/IP complete stack ----
udp_complete u_udp (
    .clk(clk_125mhz), .rst(rst_125),
    // Ethernet frame input
    .s_eth_hdr_valid(rx_eth_hdr_valid),
    .s_eth_hdr_ready(rx_eth_hdr_ready),
    .s_eth_dest_mac(rx_eth_dest_mac),
    .s_eth_src_mac(rx_eth_src_mac),
    .s_eth_type(rx_eth_type),
    .s_eth_payload_axis_tdata(rx_eth_payload_axis_tdata),
    .s_eth_payload_axis_tvalid(rx_eth_payload_axis_tvalid),
    .s_eth_payload_axis_tready(rx_eth_payload_axis_tready),
    .s_eth_payload_axis_tlast(rx_eth_payload_axis_tlast),
    .s_eth_payload_axis_tuser(rx_eth_payload_axis_tuser),
    // Ethernet frame output
    .m_eth_hdr_valid(tx_eth_hdr_valid),
    .m_eth_hdr_ready(tx_eth_hdr_ready),
    .m_eth_dest_mac(tx_eth_dest_mac),
    .m_eth_src_mac(tx_eth_src_mac),
    .m_eth_type(tx_eth_type),
    .m_eth_payload_axis_tdata(tx_eth_payload_axis_tdata),
    .m_eth_payload_axis_tvalid(tx_eth_payload_axis_tvalid),
    .m_eth_payload_axis_tready(tx_eth_payload_axis_tready),
    .m_eth_payload_axis_tlast(tx_eth_payload_axis_tlast),
    .m_eth_payload_axis_tuser(tx_eth_payload_axis_tuser),
    // IP frame input (unused)
    .s_ip_hdr_valid(1'b0),
    .s_ip_hdr_ready(),
    .s_ip_dscp(6'd0), .s_ip_ecn(2'd0),
    .s_ip_length(16'd0), .s_ip_ttl(8'd0), .s_ip_protocol(8'd0),
    .s_ip_source_ip(32'd0), .s_ip_dest_ip(32'd0),
    .s_ip_payload_axis_tdata(8'd0),
    .s_ip_payload_axis_tvalid(1'b0),
    .s_ip_payload_axis_tready(),
    .s_ip_payload_axis_tlast(1'b0),
    .s_ip_payload_axis_tuser(1'b0),
    // IP frame output (unused)
    .m_ip_hdr_valid(rx_ip_hdr_valid),
    .m_ip_hdr_ready(rx_ip_hdr_ready),
    .m_ip_eth_dest_mac(), .m_ip_eth_src_mac(), .m_ip_eth_type(),
    .m_ip_version(), .m_ip_ihl(), .m_ip_dscp(), .m_ip_ecn(),
    .m_ip_length(), .m_ip_identification(), .m_ip_flags(),
    .m_ip_fragment_offset(), .m_ip_ttl(), .m_ip_protocol(),
    .m_ip_header_checksum(), .m_ip_source_ip(), .m_ip_dest_ip(),
    .m_ip_payload_axis_tdata(rx_ip_payload_axis_tdata),
    .m_ip_payload_axis_tvalid(rx_ip_payload_axis_tvalid),
    .m_ip_payload_axis_tready(rx_ip_payload_axis_tready),
    .m_ip_payload_axis_tlast(),
    .m_ip_payload_axis_tuser(),
    // UDP frame input (TX to network)
    .s_udp_hdr_valid(tx_udp_hdr_valid),
    .s_udp_hdr_ready(tx_udp_hdr_ready),
    .s_udp_ip_dscp(6'd0), .s_udp_ip_ecn(2'd0),
    .s_udp_ip_ttl(8'd64),
    .s_udp_ip_source_ip(local_ip),
    .s_udp_ip_dest_ip(reply_dest_ip),
    .s_udp_source_port(16'd7700),
    .s_udp_dest_port(reply_dest_port),
    .s_udp_length(tx_udp_length),
    .s_udp_checksum(16'd0),
    .s_udp_payload_axis_tdata(tx_udp_payload_axis_tdata),
    .s_udp_payload_axis_tvalid(tx_udp_payload_axis_tvalid),
    .s_udp_payload_axis_tready(tx_udp_payload_axis_tready),
    .s_udp_payload_axis_tlast(tx_udp_payload_axis_tlast),
    .s_udp_payload_axis_tuser(tx_udp_payload_axis_tuser),
    // UDP frame output (RX from network)
    .m_udp_hdr_valid(rx_udp_hdr_valid),
    .m_udp_hdr_ready(rx_udp_hdr_ready),
    .m_udp_eth_dest_mac(), .m_udp_eth_src_mac(), .m_udp_eth_type(),
    .m_udp_ip_version(), .m_udp_ip_ihl(), .m_udp_ip_dscp(), .m_udp_ip_ecn(),
    .m_udp_ip_length(), .m_udp_ip_identification(), .m_udp_ip_flags(),
    .m_udp_ip_fragment_offset(), .m_udp_ip_ttl(), .m_udp_ip_protocol(),
    .m_udp_ip_header_checksum(),
    .m_udp_ip_source_ip(rx_udp_ip_source_ip),
    .m_udp_ip_dest_ip(),
    .m_udp_source_port(rx_udp_source_port),
    .m_udp_dest_port(rx_udp_dest_port),
    .m_udp_length(rx_udp_length),
    .m_udp_checksum(),
    .m_udp_payload_axis_tdata(rx_udp_payload_axis_tdata),
    .m_udp_payload_axis_tvalid(rx_udp_payload_axis_tvalid),
    .m_udp_payload_axis_tready(rx_udp_payload_axis_tready),
    .m_udp_payload_axis_tlast(rx_udp_payload_axis_tlast),
    .m_udp_payload_axis_tuser(rx_udp_payload_axis_tuser),
    // Config
    .local_mac(local_mac),
    .local_ip(local_ip),
    .gateway_ip(gateway_ip),
    .subnet_mask(subnet_mask),
    .clear_arp_cache(1'b0)
);

// =========================================================================
// UDP Command/Telemetry Controller (125 MHz domain)
// =========================================================================
// Protocol: same as UART but over UDP packets on port 7700
//   RX: [0x55][CMD][payload...] — exactly like UART
//   TX: [0x55][0x02][LEN_HI][LEN_LO][768B telemetry][CRC8]
//
// Auto-telemetry: When enabled, FPGA pushes telemetry at ~1 kHz to
// the last host that sent any command (reply_dest_ip / reply_dest_port).

// Latch sender info for replies
reg [31:0] reply_dest_ip;
reg [15:0] reply_dest_port;
reg        has_host;

// Match on port 7700
wire port_match = (rx_udp_dest_port == 16'd7700);
assign rx_udp_hdr_ready = 1'b1;
assign rx_udp_payload_axis_tready = 1'b1;

// ---- RX: UDP → command registers (125 MHz domain) ----
// Simple byte-at-a-time state machine, same as UART command decoder
reg [5:0] udp_cd_state;
localparam UCD_IDLE   = 6'd0;
localparam UCD_CMD    = 6'd1;
localparam UCD_VG_SEL = 6'd2;
localparam UCD_VG_D0  = 6'd3;
localparam UCD_VG_D1  = 6'd4;
localparam UCD_VG_D2  = 6'd5;
localparam UCD_VG_D3  = 6'd6;
localparam UCD_KILL   = 6'd7;
localparam UCD_BATCH_START = 6'd8;
localparam UCD_BATCH_CNT   = 6'd9;
localparam UCD_BATCH_D0    = 6'd10;
localparam UCD_BATCH_D1    = 6'd11;
localparam UCD_BATCH_D2    = 6'd12;
localparam UCD_BATCH_D3    = 6'd13;
localparam UCD_TEMP_D0     = 6'd14;
localparam UCD_TEMP_D1     = 6'd15;
localparam UCD_TEMP_D2     = 6'd16;
localparam UCD_TEMP_D3     = 6'd17;
localparam UCD_MAC_D0      = 6'd18;
localparam UCD_MAC_D1      = 6'd19;
localparam UCD_MAC_D2      = 6'd20;
localparam UCD_MAC_D3      = 6'd21;
// CMD_SET_RATE: [rate_div(16 BE)] — set auto-telemetry rate divider
localparam UCD_RATE_D0     = 6'd22;
localparam UCD_RATE_D1     = 6'd23;
localparam UCD_LEAK_D0     = 6'd24;
localparam UCD_LEAK_D1     = 6'd25;
localparam UCD_LEAK_D2     = 6'd26;
localparam UCD_LEAK_D3     = 6'd27;
// Generic 32-bit param: reuses udp_data32, latches to target on D3
localparam UCD_PARAM_D0    = 6'd28;
localparam UCD_PARAM_D1    = 6'd29;
localparam UCD_PARAM_D2    = 6'd30;
localparam UCD_PARAM_D3    = 6'd31;
// 16-bit refract param
localparam UCD_REFR_D0     = 6'd32;
localparam UCD_REFR_D1     = 6'd33;

localparam CMD_SET_VG       = 8'h01;
localparam CMD_READ_TELEM   = 8'h02;
localparam CMD_SET_KILL     = 8'h03;
localparam CMD_SET_TEMP     = 8'h05;
localparam CMD_SET_MAC      = 8'h06;
localparam CMD_SET_VG_BATCH = 8'h08;
localparam CMD_SET_RATE     = 8'h09;  // new: auto-telemetry rate
localparam CMD_SET_LEAK     = 8'h0A;  // runtime leak conductance
localparam CMD_SET_THRESH   = 8'h0B;  // runtime threshold (Q16.16)
localparam CMD_SET_BASE_EXC = 8'h0C;  // runtime base excitation (Q16.16)
localparam CMD_SET_BIAS_GAIN= 8'h0D;  // runtime bias gain (Q16.16)
localparam CMD_SET_DT_C     = 8'h0E;  // runtime dt/C (Q16.16)
localparam CMD_SET_REFRACT  = 8'h0F;  // runtime refractory cycles (16-bit)

reg [7:0]  udp_cmd;
reg [NEURON_ADDR_W-1:0] udp_neuron_sel;
reg [31:0] udp_data32;

// Neuron bank control (from UDP, directly in 125 MHz domain)
// These will be CDC'd to sys/phy domains
reg [31:0] eth_reg_temp_k;
reg [31:0] eth_reg_mac_signal;
reg        eth_kill;
reg [NEURON_ADDR_W-1:0] eth_neuron_sel;
reg [31:0] eth_vg_write;
reg        eth_vg_wen;
reg        eth_telem_request;

// Batch
reg [NEURON_ADDR_W-1:0] eth_batch_start;
reg [7:0]  eth_batch_count;
reg [7:0]  eth_batch_idx;

// Runtime leak conductance (Q16.16), default matches lif_membrane parameter
reg [31:0] eth_reg_leak_cond;

// Runtime LIF parameters (default matches lif_membrane compile-time values)
reg [31:0] eth_reg_threshold;
reg [31:0] eth_reg_base_exc;
reg [31:0] eth_reg_bias_gain;
reg [31:0] eth_reg_dt_over_c;
reg [15:0] eth_reg_refract_cyc;
reg [2:0]  eth_param_sel;  // which param the generic D0-D3 states target

// Auto-telemetry rate: default ~1 kHz (125_000 cycles at 125 MHz)
reg [19:0] auto_telem_div;
reg [19:0] auto_telem_cnt;
reg        auto_telem_en;

initial begin
    eth_reg_temp_k    = 32'h012C_0000;
    eth_reg_mac_signal = 32'h0000_8000;
    eth_reg_leak_cond  = 32'h0000_0004;  // default τ≈210ms
    eth_reg_threshold  = 32'h0000_8000;  // 0.50V
    eth_reg_base_exc   = 32'h0000_0333;  // 0.0125
    eth_reg_bias_gain  = 32'h0000_0800;  // 0.03125
    eth_reg_dt_over_c  = 32'h0000_0200;  // 0.0078
    eth_reg_refract_cyc = 16'd50;        // 5us @ 10MHz
    eth_param_sel      = 3'd0;
    eth_kill          = 1'b0;
    eth_vg_wen        = 1'b0;
    eth_telem_request = 1'b0;
    auto_telem_div    = 20'd125_000;  // 1 kHz
    auto_telem_en     = 1'b0;
    has_host          = 1'b0;
end

always @(posedge clk_125mhz) begin
    if (rst_125) begin
        udp_cd_state      <= UCD_IDLE;
        eth_vg_wen        <= 1'b0;
        eth_telem_request <= 1'b0;
        eth_kill          <= 1'b0;
        has_host          <= 1'b0;
        auto_telem_en     <= 1'b0;
        auto_telem_div    <= 20'd125_000;
        auto_telem_cnt    <= 20'd0;
    end else begin
        eth_vg_wen        <= 1'b0;
        eth_telem_request <= 1'b0;

        // Auto-telemetry timer
        if (auto_telem_en && has_host) begin
            if (auto_telem_cnt >= auto_telem_div) begin
                auto_telem_cnt    <= 20'd0;
                eth_telem_request <= 1'b1;
            end else begin
                auto_telem_cnt <= auto_telem_cnt + 20'd1;
            end
        end

        // Latch sender on valid UDP header to port 7700
        if (rx_udp_hdr_valid && port_match) begin
            reply_dest_ip   <= rx_udp_ip_source_ip;
            reply_dest_port <= rx_udp_source_port;
            has_host        <= 1'b1;
        end

        // Process UDP payload bytes
        if (rx_udp_payload_axis_tvalid && port_match) begin
            case (udp_cd_state)
                UCD_IDLE: begin
                    if (rx_udp_payload_axis_tdata == 8'h55)
                        udp_cd_state <= UCD_CMD;
                end

                UCD_CMD: begin
                    udp_cmd <= rx_udp_payload_axis_tdata;
                    case (rx_udp_payload_axis_tdata)
                        CMD_READ_TELEM: begin
                            eth_telem_request <= 1'b1;
                            udp_cd_state      <= UCD_IDLE;
                        end
                        CMD_SET_VG:       udp_cd_state <= UCD_VG_SEL;
                        CMD_SET_KILL:     udp_cd_state <= UCD_KILL;
                        CMD_SET_TEMP:     udp_cd_state <= UCD_TEMP_D0;
                        CMD_SET_MAC:      udp_cd_state <= UCD_MAC_D0;
                        CMD_SET_VG_BATCH: udp_cd_state <= UCD_BATCH_START;
                        CMD_SET_RATE:     udp_cd_state <= UCD_RATE_D0;
                        CMD_SET_LEAK:     udp_cd_state <= UCD_LEAK_D0;
                        CMD_SET_THRESH:   begin eth_param_sel <= 3'd0; udp_cd_state <= UCD_PARAM_D0; end
                        CMD_SET_BASE_EXC: begin eth_param_sel <= 3'd1; udp_cd_state <= UCD_PARAM_D0; end
                        CMD_SET_BIAS_GAIN:begin eth_param_sel <= 3'd2; udp_cd_state <= UCD_PARAM_D0; end
                        CMD_SET_DT_C:     begin eth_param_sel <= 3'd3; udp_cd_state <= UCD_PARAM_D0; end
                        CMD_SET_REFRACT:  udp_cd_state <= UCD_REFR_D0;
                        default:          udp_cd_state <= UCD_IDLE;
                    endcase
                end

                // SET_VG: [neuron_id] [D3][D2][D1][D0]
                UCD_VG_SEL: begin
                    udp_neuron_sel <= rx_udp_payload_axis_tdata[NEURON_ADDR_W-1:0];
                    udp_cd_state   <= UCD_VG_D0;
                end
                UCD_VG_D0: begin udp_data32[31:24] <= rx_udp_payload_axis_tdata; udp_cd_state <= UCD_VG_D1; end
                UCD_VG_D1: begin udp_data32[23:16] <= rx_udp_payload_axis_tdata; udp_cd_state <= UCD_VG_D2; end
                UCD_VG_D2: begin udp_data32[15:8]  <= rx_udp_payload_axis_tdata; udp_cd_state <= UCD_VG_D3; end
                UCD_VG_D3: begin
                    udp_data32[7:0] <= rx_udp_payload_axis_tdata;
                    eth_neuron_sel  <= udp_neuron_sel;
                    eth_vg_write    <= {udp_data32[31:8], rx_udp_payload_axis_tdata};
                    eth_vg_wen      <= 1'b1;
                    udp_cd_state    <= UCD_IDLE;
                end

                UCD_KILL: begin
                    eth_kill     <= rx_udp_payload_axis_tdata[0];
                    udp_cd_state <= UCD_IDLE;
                end

                UCD_TEMP_D0: begin udp_data32[31:24] <= rx_udp_payload_axis_tdata; udp_cd_state <= UCD_TEMP_D1; end
                UCD_TEMP_D1: begin udp_data32[23:16] <= rx_udp_payload_axis_tdata; udp_cd_state <= UCD_TEMP_D2; end
                UCD_TEMP_D2: begin udp_data32[15:8]  <= rx_udp_payload_axis_tdata; udp_cd_state <= UCD_TEMP_D3; end
                UCD_TEMP_D3: begin
                    eth_reg_temp_k <= {udp_data32[31:8], rx_udp_payload_axis_tdata};
                    udp_cd_state   <= UCD_IDLE;
                end

                UCD_MAC_D0: begin udp_data32[31:24] <= rx_udp_payload_axis_tdata; udp_cd_state <= UCD_MAC_D1; end
                UCD_MAC_D1: begin udp_data32[23:16] <= rx_udp_payload_axis_tdata; udp_cd_state <= UCD_MAC_D2; end
                UCD_MAC_D2: begin udp_data32[15:8]  <= rx_udp_payload_axis_tdata; udp_cd_state <= UCD_MAC_D3; end
                UCD_MAC_D3: begin
                    eth_reg_mac_signal <= {udp_data32[31:8], rx_udp_payload_axis_tdata};
                    udp_cd_state       <= UCD_IDLE;
                end

                // SET_VG_BATCH: [start_id][count][vg0_D3..D0]...
                UCD_BATCH_START: begin
                    eth_batch_start <= rx_udp_payload_axis_tdata[NEURON_ADDR_W-1:0];
                    udp_cd_state    <= UCD_BATCH_CNT;
                end
                UCD_BATCH_CNT: begin
                    eth_batch_count <= rx_udp_payload_axis_tdata;
                    eth_batch_idx   <= 8'd0;
                    udp_cd_state    <= UCD_BATCH_D0;
                end
                UCD_BATCH_D0: begin udp_data32[31:24] <= rx_udp_payload_axis_tdata; udp_cd_state <= UCD_BATCH_D1; end
                UCD_BATCH_D1: begin udp_data32[23:16] <= rx_udp_payload_axis_tdata; udp_cd_state <= UCD_BATCH_D2; end
                UCD_BATCH_D2: begin udp_data32[15:8]  <= rx_udp_payload_axis_tdata; udp_cd_state <= UCD_BATCH_D3; end
                UCD_BATCH_D3: begin
                    udp_data32[7:0] <= rx_udp_payload_axis_tdata;
                    eth_neuron_sel  <= eth_batch_start + eth_batch_idx[NEURON_ADDR_W-1:0];
                    eth_vg_write    <= {udp_data32[31:8], rx_udp_payload_axis_tdata};
                    eth_vg_wen      <= 1'b1;
                    eth_batch_idx   <= eth_batch_idx + 8'd1;
                    if (eth_batch_idx + 8'd1 >= eth_batch_count)
                        udp_cd_state <= UCD_IDLE;
                    else
                        udp_cd_state <= UCD_BATCH_D0;
                end

                // SET_LEAK: [D3][D2][D1][D0] — leak conductance Q16.16
                UCD_LEAK_D0: begin udp_data32[31:24] <= rx_udp_payload_axis_tdata; udp_cd_state <= UCD_LEAK_D1; end
                UCD_LEAK_D1: begin udp_data32[23:16] <= rx_udp_payload_axis_tdata; udp_cd_state <= UCD_LEAK_D2; end
                UCD_LEAK_D2: begin udp_data32[15:8]  <= rx_udp_payload_axis_tdata; udp_cd_state <= UCD_LEAK_D3; end
                UCD_LEAK_D3: begin
                    eth_reg_leak_cond <= {udp_data32[31:8], rx_udp_payload_axis_tdata};
                    udp_cd_state      <= UCD_IDLE;
                end

                // SET_RATE: [div_hi][div_lo] — auto-telemetry divider
                UCD_RATE_D0: begin
                    udp_data32[15:8] <= rx_udp_payload_axis_tdata;
                    udp_cd_state     <= UCD_RATE_D1;
                end
                UCD_RATE_D1: begin
                    // Rate divider in units of 1 kHz (so 1 = 1 kHz, 10 = 100 Hz)
                    // div = value * 125_000 (but capped at 20 bits)
                    if ({udp_data32[15:8], rx_udp_payload_axis_tdata} == 16'd0) begin
                        auto_telem_en  <= 1'b0;
                    end else begin
                        auto_telem_en  <= 1'b1;
                        // Simple: store raw as cycle count. Host sends actual cycle count.
                        auto_telem_div <= {4'd0, udp_data32[15:8], rx_udp_payload_axis_tdata};
                    end
                    udp_cd_state <= UCD_IDLE;
                end

                // Generic 32-bit parameter: [D3][D2][D1][D0], target selected by eth_param_sel
                UCD_PARAM_D0: begin udp_data32[31:24] <= rx_udp_payload_axis_tdata; udp_cd_state <= UCD_PARAM_D1; end
                UCD_PARAM_D1: begin udp_data32[23:16] <= rx_udp_payload_axis_tdata; udp_cd_state <= UCD_PARAM_D2; end
                UCD_PARAM_D2: begin udp_data32[15:8]  <= rx_udp_payload_axis_tdata; udp_cd_state <= UCD_PARAM_D3; end
                UCD_PARAM_D3: begin
                    case (eth_param_sel)
                        3'd0: eth_reg_threshold <= {udp_data32[31:8], rx_udp_payload_axis_tdata};
                        3'd1: eth_reg_base_exc  <= {udp_data32[31:8], rx_udp_payload_axis_tdata};
                        3'd2: eth_reg_bias_gain <= {udp_data32[31:8], rx_udp_payload_axis_tdata};
                        3'd3: eth_reg_dt_over_c <= {udp_data32[31:8], rx_udp_payload_axis_tdata};
                        default: ;
                    endcase
                    udp_cd_state <= UCD_IDLE;
                end

                // SET_REFRACT: [D1][D0] — 16-bit refractory cycles
                UCD_REFR_D0: begin udp_data32[15:8] <= rx_udp_payload_axis_tdata; udp_cd_state <= UCD_REFR_D1; end
                UCD_REFR_D1: begin
                    eth_reg_refract_cyc <= {udp_data32[15:8], rx_udp_payload_axis_tdata};
                    udp_cd_state <= UCD_IDLE;
                end

                default: udp_cd_state <= UCD_IDLE;
            endcase

            // Reset state on packet boundary
            if (rx_udp_payload_axis_tlast)
                udp_cd_state <= UCD_IDLE;
        end
    end
end

// =========================================================================
// Neuron Bank (clk_sys=100MHz, clk_phy=10MHz)
// =========================================================================
// Mux between UART commands and Ethernet commands
// Priority: Ethernet (faster) over UART

// UART command decoder — reuse inline (simplified from nsram_bridge_top)
localparam UCMD_SET_VG   = 8'h01;
localparam UCMD_TELEM    = 8'h02;
localparam UCMD_KILL     = 8'h03;
localparam UCMD_BATCH    = 8'h08;

reg [4:0] uart_cd_state;
reg [7:0] uart_cmd;
reg [NEURON_ADDR_W-1:0] uart_neuron_sel;
reg [31:0] uart_data32;
reg [31:0] uart_vg_write;
reg        uart_vg_wen;
reg        uart_kill;
reg        uart_telem_req;
reg [NEURON_ADDR_W-1:0] uart_batch_start;
reg [7:0] uart_batch_count, uart_batch_idx;

always @(posedge clk_sys) begin
    if (rst) begin
        uart_cd_state  <= 5'd0;
        uart_vg_wen    <= 1'b0;
        uart_kill      <= 1'b0;
        uart_telem_req <= 1'b0;
    end else begin
        uart_vg_wen    <= 1'b0;
        uart_telem_req <= 1'b0;
        if (rx_valid) begin
            case (uart_cd_state)
                5'd0: if (rx_data == 8'h55) uart_cd_state <= 5'd1;
                5'd1: begin
                    uart_cmd <= rx_data;
                    case (rx_data)
                        UCMD_TELEM: begin uart_telem_req <= 1'b1; uart_cd_state <= 5'd0; end
                        UCMD_SET_VG: uart_cd_state <= 5'd2;
                        UCMD_KILL:   uart_cd_state <= 5'd7;
                        UCMD_BATCH:  uart_cd_state <= 5'd22;
                        default:     uart_cd_state <= 5'd0;
                    endcase
                end
                // SET_VG
                5'd2: begin uart_neuron_sel <= rx_data[NEURON_ADDR_W-1:0]; uart_cd_state <= 5'd3; end
                5'd3: begin uart_data32[31:24] <= rx_data; uart_cd_state <= 5'd4; end
                5'd4: begin uart_data32[23:16] <= rx_data; uart_cd_state <= 5'd5; end
                5'd5: begin uart_data32[15:8]  <= rx_data; uart_cd_state <= 5'd6; end
                5'd6: begin
                    uart_vg_write <= {uart_data32[31:8], rx_data};
                    uart_neuron_sel <= uart_neuron_sel;
                    uart_vg_wen <= 1'b1;
                    uart_cd_state <= 5'd0;
                end
                // KILL
                5'd7: begin uart_kill <= rx_data[0]; uart_cd_state <= 5'd0; end
                // BATCH
                5'd22: begin uart_batch_start <= rx_data[NEURON_ADDR_W-1:0]; uart_cd_state <= 5'd23; end
                5'd23: begin uart_batch_count <= rx_data; uart_batch_idx <= 8'd0; uart_cd_state <= 5'd24; end
                5'd24: begin uart_data32[31:24] <= rx_data; uart_cd_state <= 5'd25; end
                5'd25: begin uart_data32[23:16] <= rx_data; uart_cd_state <= 5'd26; end
                5'd26: begin uart_data32[15:8]  <= rx_data; uart_cd_state <= 5'd27; end
                5'd27: begin
                    uart_vg_write <= {uart_data32[31:8], rx_data};
                    uart_neuron_sel <= uart_batch_start + uart_batch_idx[NEURON_ADDR_W-1:0];
                    uart_vg_wen <= 1'b1;
                    uart_batch_idx <= uart_batch_idx + 8'd1;
                    if (uart_batch_idx + 8'd1 >= uart_batch_count)
                        uart_cd_state <= 5'd0;
                    else
                        uart_cd_state <= 5'd24;
                end
                default: uart_cd_state <= 5'd0;
            endcase
        end
    end
end

// ---- CDC: Ethernet commands (125 MHz) → sys (100 MHz) ----
// Simple pulse CDC for vg_wen and telem_request
// For Vg write: use toggle handshake

reg eth_vg_toggle;
reg [2:0] eth_vg_toggle_sync;
reg eth_vg_toggle_prev;

always @(posedge clk_125mhz)
    if (rst_125) eth_vg_toggle <= 1'b0;
    else if (eth_vg_wen) eth_vg_toggle <= ~eth_vg_toggle;

always @(posedge clk_sys) begin
    eth_vg_toggle_sync <= {eth_vg_toggle_sync[1:0], eth_vg_toggle};
    eth_vg_toggle_prev <= eth_vg_toggle_sync[2];
end

wire eth_vg_wen_sys = (eth_vg_toggle_sync[2] != eth_vg_toggle_prev);

// CDC for telem request
reg eth_telem_toggle;
reg [2:0] eth_telem_toggle_sync;
reg eth_telem_toggle_prev;

always @(posedge clk_125mhz)
    if (rst_125) eth_telem_toggle <= 1'b0;
    else if (eth_telem_request) eth_telem_toggle <= ~eth_telem_toggle;

always @(posedge clk_sys) begin
    eth_telem_toggle_sync <= {eth_telem_toggle_sync[1:0], eth_telem_toggle};
    eth_telem_toggle_prev <= eth_telem_toggle_sync[2];
end

wire eth_telem_req_sys = (eth_telem_toggle_sync[2] != eth_telem_toggle_prev);

// ---- Mux: Ethernet vs UART ----
wire kill_switch = eth_kill | uart_kill | sw[0];
wire [1:0] regime = sw[2:1];

// Vg write: Ethernet takes priority
wire        nb_vg_wen      = eth_vg_wen_sys ? 1'b1 : uart_vg_wen;
wire [NEURON_ADDR_W-1:0] nb_neuron_sel_wr = eth_vg_wen_sys ? eth_neuron_sel : uart_neuron_sel;
wire [31:0] nb_vg_write    = eth_vg_wen_sys ? eth_vg_write : uart_vg_write;

wire        nb_telem_req   = eth_telem_req_sys | uart_telem_req;

// ---- Neuron Bank ----
wire [NUM_NEURONS-1:0] nb_spike_out;
wire [15:0] nb_spike_count_total;
wire [31:0] nb_vmem_sel, nb_bvpar_sel;
wire [15:0] nb_spike_count_sel;
wire [15:0] nb_refract_sel;
wire [31:0] nb_dbg_i_aval, nb_dbg_vcb, nb_dbg_v_bulk;
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
    .temp_k           (eth_reg_temp_k),
    .mac_signal       (eth_reg_mac_signal),
    .leak_cond        (eth_reg_leak_cond),
    .cfg_threshold    (eth_reg_threshold),
    .cfg_base_exc     (eth_reg_base_exc),
    .cfg_bias_gain    (eth_reg_bias_gain),
    .cfg_dt_over_c    (eth_reg_dt_over_c),
    .cfg_refract_cyc  (eth_reg_refract_cyc),
    .regime           (regime),
    .neuron_sel       (bank_neuron_sel),
    .vg_write         (nb_vg_write),
    .vg_wen           (nb_vg_wen),
    .syn_w_write      (32'd0),
    .syn_wen          (1'b0),
    .spike_out        (nb_spike_out),
    .spike_count_total(nb_spike_count_total),
    .vmem_sel         (nb_vmem_sel),
    .bvpar_sel        (nb_bvpar_sel),
    .spike_count_sel  (nb_spike_count_sel),
    .refract_sel      (nb_refract_sel),
    .count_reset      (nb_count_reset),
    .dbg_i_aval_sel   (nb_dbg_i_aval),
    .dbg_vcb          (nb_dbg_vcb),
    .dbg_v_bulk       (nb_dbg_v_bulk),
    .dbg_kill_phy     (nb_dbg_kill_phy)
);

// =========================================================================
// Telemetry Scan + Transmit (both UART and UDP)
// =========================================================================
// Reuse the existing scan approach: cycle through 128 neurons, snapshot,
// then serialize. For UDP: write entire 773-byte packet to a BRAM buffer,
// then stream it out as a single UDP payload.

localparam [15:0] PAYLOAD_LEN = 16'd1024;  // 128 * 8 bytes/neuron

// Telemetry snapshot storage
reg [15:0] tm_spike_cnt [0:NUM_NEURONS-1];
reg [31:0] tm_vmem      [0:NUM_NEURONS-1];
reg [15:0] tm_refract   [0:NUM_NEURONS-1];

// Scan state machine (100 MHz)
reg [3:0]  tm_state;
localparam TM_IDLE     = 4'd0;
localparam TM_SCAN     = 4'd1;
localparam TM_UART_TX  = 4'd2;
localparam TM_UDP_TX   = 4'd3;
localparam TM_BEACON   = 4'd4;

reg [NEURON_ADDR_W-1:0] tm_scan_idx;
reg [7:0]  tm_wait_cnt;
reg        tm_scanning;
reg        tm_scan_done;
reg        telem_via_uart;
reg        telem_via_udp;

assign bank_neuron_sel = tm_scanning ? tm_scan_idx : nb_neuron_sel_wr;

// CRC-8
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

// ---- UART telemetry TX ----
reg [NEURON_ADDR_W-1:0] uart_tm_neuron;
reg [2:0]  uart_tm_byte;
reg [7:0]  uart_tm_crc;
reg [3:0]  uart_tm_state;
localparam UTX_IDLE = 4'd0;
localparam UTX_HDR0 = 4'd1;
localparam UTX_HDR1 = 4'd2;
localparam UTX_HDR2 = 4'd3;
localparam UTX_HDR3 = 4'd4;
localparam UTX_DATA = 4'd5;
localparam UTX_CRC  = 4'd6;
localparam UTX_WAIT = 4'd7;
localparam UTX_BEACON = 4'd8;

reg [3:0] uart_tm_next;
reg [1:0] beacon_idx;

// ---- UDP telemetry TX (125 MHz domain) ----
// Buffer: store 773-byte telemetry packet, stream via AXI-S
reg [10:0] udp_tx_addr;   // up to 1029
reg [10:0] udp_tx_len;
(* ram_style = "block" *) reg [7:0]  udp_tx_buf [0:1055]; // 1029 + margin, BRAM for timing
reg        udp_tx_active;
reg        udp_tx_start;
reg [15:0] tx_udp_length;

// Combinational byte select for UDP telemetry build (outside always block for Verilog-2001)
reg [7:0] udp_nbyte;
always @(*) begin
    case (udp_tm_byte)
        3'd0: udp_nbyte = tm_spike_cnt[udp_tm_neuron][15:8];
        3'd1: udp_nbyte = tm_spike_cnt[udp_tm_neuron][7:0];
        3'd2: udp_nbyte = tm_vmem[udp_tm_neuron][31:24];
        3'd3: udp_nbyte = tm_vmem[udp_tm_neuron][23:16];
        3'd4: udp_nbyte = tm_vmem[udp_tm_neuron][15:8];
        3'd5: udp_nbyte = tm_vmem[udp_tm_neuron][7:0];
        3'd6: udp_nbyte = tm_refract[udp_tm_neuron][15:8];
        3'd7: udp_nbyte = tm_refract[udp_tm_neuron][7:0];
    endcase
end

// CDC: scan_done pulse (sys → 125 MHz)
reg tm_scan_done_toggle;
reg [2:0] tm_scan_done_sync;
reg tm_scan_done_prev;

always @(posedge clk_sys)
    if (rst) tm_scan_done_toggle <= 1'b0;
    else if (tm_scan_done) tm_scan_done_toggle <= ~tm_scan_done_toggle;

always @(posedge clk_125mhz) begin
    tm_scan_done_sync <= {tm_scan_done_sync[1:0], tm_scan_done_toggle};
    tm_scan_done_prev <= tm_scan_done_sync[2];
end
wire udp_scan_done = (tm_scan_done_sync[2] != tm_scan_done_prev);

// ---- Main scan FSM (100 MHz) ----
always @(posedge clk_sys) begin
    if (rst) begin
        tm_state      <= TM_BEACON;
        tm_scanning   <= 1'b0;
        tm_scan_done  <= 1'b0;
        tm_scan_idx   <= 0;
        tm_wait_cnt   <= 0;
        nb_count_reset <= 1'b0;
        telem_via_uart <= 1'b0;
        telem_via_udp  <= 1'b0;
        uart_tm_state <= UTX_BEACON;
        beacon_idx    <= 2'd0;
        tx_start      <= 1'b0;
        tx_data       <= 8'd0;
    end else begin
        nb_count_reset <= 1'b0;
        tx_start       <= 1'b0;
        tm_scan_done   <= 1'b0;

        case (tm_state)
            TM_BEACON: begin
                // Send "BOK\n" over UART on startup
                if (!tx_busy && !tx_start) begin
                    case (beacon_idx)
                        2'd0: tx_data <= 8'h42;
                        2'd1: tx_data <= 8'h4F;
                        2'd2: tx_data <= 8'h4B;
                        2'd3: tx_data <= 8'h0A;
                    endcase
                    tx_start <= 1'b1;
                    if (beacon_idx == 2'd3)
                        tm_state <= TM_IDLE;
                    else
                        beacon_idx <= beacon_idx + 2'd1;
                end
            end

            TM_IDLE: begin
                if (nb_telem_req) begin
                    tm_scanning <= 1'b1;
                    tm_scan_idx <= 0;
                    tm_wait_cnt <= 0;
                    // Determine source
                    telem_via_uart <= uart_telem_req;
                    telem_via_udp  <= eth_telem_req_sys;
                    tm_state       <= TM_SCAN;
                end
            end

            TM_SCAN: begin
                tm_wait_cnt <= tm_wait_cnt + 8'd1;
                if (tm_wait_cnt == 8'd49) begin
                    tm_vmem[tm_scan_idx]      <= nb_vmem_sel;
                    tm_refract[tm_scan_idx]   <= nb_refract_sel;
                    tm_spike_cnt[tm_scan_idx] <= nb_spike_count_sel;
                    tm_wait_cnt               <= 8'd0;

                    if (tm_scan_idx == NUM_NEURONS - 1) begin
                        nb_count_reset <= 1'b1;
                        tm_scanning    <= 1'b0;
                        tm_scan_done   <= 1'b1;
                        // Route to UART or UDP (or both)
                        if (telem_via_uart)
                            tm_state <= TM_UART_TX;
                        else
                            tm_state <= TM_IDLE;
                    end else begin
                        tm_scan_idx <= tm_scan_idx + 1;
                    end
                end
            end

            TM_UART_TX: begin
                // UART telemetry serialization
                case (uart_tm_state)
                    UTX_IDLE: begin
                        uart_tm_state <= UTX_HDR0;
                        uart_tm_crc   <= 8'd0;
                        uart_tm_neuron <= 0;
                        uart_tm_byte   <= 0;
                    end
                    UTX_HDR0: begin
                        if (!tx_busy && !tx_start) begin
                            tx_data <= 8'h55;
                            tx_start <= 1'b1;
                            uart_tm_crc <= crc8_byte(uart_tm_crc, 8'h55);
                            uart_tm_state <= UTX_WAIT;
                            uart_tm_next <= UTX_HDR1;
                        end
                    end
                    UTX_HDR1: begin
                        if (!tx_busy && !tx_start) begin
                            tx_data <= 8'h02;
                            tx_start <= 1'b1;
                            uart_tm_crc <= crc8_byte(uart_tm_crc, 8'h02);
                            uart_tm_state <= UTX_WAIT;
                            uart_tm_next <= UTX_HDR2;
                        end
                    end
                    UTX_HDR2: begin
                        if (!tx_busy && !tx_start) begin
                            tx_data <= PAYLOAD_LEN[15:8];
                            tx_start <= 1'b1;
                            uart_tm_crc <= crc8_byte(uart_tm_crc, PAYLOAD_LEN[15:8]);
                            uart_tm_state <= UTX_WAIT;
                            uart_tm_next <= UTX_HDR3;
                        end
                    end
                    UTX_HDR3: begin
                        if (!tx_busy && !tx_start) begin
                            tx_data <= PAYLOAD_LEN[7:0];
                            tx_start <= 1'b1;
                            uart_tm_crc <= crc8_byte(uart_tm_crc, PAYLOAD_LEN[7:0]);
                            uart_tm_state <= UTX_WAIT;
                            uart_tm_next <= UTX_DATA;
                        end
                    end
                    UTX_DATA: begin
                        if (!tx_busy && !tx_start) begin
                            case (uart_tm_byte)
                                3'd0: begin tx_data <= tm_spike_cnt[uart_tm_neuron][15:8]; uart_tm_crc <= crc8_byte(uart_tm_crc, tm_spike_cnt[uart_tm_neuron][15:8]); end
                                3'd1: begin tx_data <= tm_spike_cnt[uart_tm_neuron][7:0];  uart_tm_crc <= crc8_byte(uart_tm_crc, tm_spike_cnt[uart_tm_neuron][7:0]); end
                                3'd2: begin tx_data <= tm_vmem[uart_tm_neuron][31:24];     uart_tm_crc <= crc8_byte(uart_tm_crc, tm_vmem[uart_tm_neuron][31:24]); end
                                3'd3: begin tx_data <= tm_vmem[uart_tm_neuron][23:16];     uart_tm_crc <= crc8_byte(uart_tm_crc, tm_vmem[uart_tm_neuron][23:16]); end
                                3'd4: begin tx_data <= tm_vmem[uart_tm_neuron][15:8];      uart_tm_crc <= crc8_byte(uart_tm_crc, tm_vmem[uart_tm_neuron][15:8]); end
                                3'd5: begin tx_data <= tm_vmem[uart_tm_neuron][7:0];       uart_tm_crc <= crc8_byte(uart_tm_crc, tm_vmem[uart_tm_neuron][7:0]); end
                                3'd6: begin tx_data <= tm_refract[uart_tm_neuron][15:8];   uart_tm_crc <= crc8_byte(uart_tm_crc, tm_refract[uart_tm_neuron][15:8]); end
                                3'd7: begin tx_data <= tm_refract[uart_tm_neuron][7:0];    uart_tm_crc <= crc8_byte(uart_tm_crc, tm_refract[uart_tm_neuron][7:0]); end
                            endcase
                            tx_start <= 1'b1;
                            if (uart_tm_byte == 3'd7) begin
                                uart_tm_byte <= 3'd0;
                                if (uart_tm_neuron == NUM_NEURONS - 1) begin
                                    uart_tm_state <= UTX_WAIT;
                                    uart_tm_next  <= UTX_CRC;
                                end else begin
                                    uart_tm_neuron <= uart_tm_neuron + 1;
                                    uart_tm_state  <= UTX_WAIT;
                                    uart_tm_next   <= UTX_DATA;
                                end
                            end else begin
                                uart_tm_byte  <= uart_tm_byte + 3'd1;
                                uart_tm_state <= UTX_WAIT;
                                uart_tm_next  <= UTX_DATA;
                            end
                        end
                    end
                    UTX_CRC: begin
                        if (!tx_busy && !tx_start) begin
                            tx_data <= uart_tm_crc;
                            tx_start <= 1'b1;
                            uart_tm_state <= UTX_WAIT;
                            uart_tm_next  <= UTX_IDLE;
                            // Done — go back to IDLE
                        end
                    end
                    UTX_WAIT: begin
                        if (!tx_busy && !tx_start) begin
                            if (uart_tm_next == UTX_IDLE)
                                tm_state <= TM_IDLE;
                            uart_tm_state <= uart_tm_next;
                        end
                    end
                    UTX_BEACON: uart_tm_state <= UTX_IDLE;
                    default: uart_tm_state <= UTX_IDLE;
                endcase
            end

            default: tm_state <= TM_IDLE;
        endcase
    end
end

// ---- UDP telemetry TX FSM (125 MHz domain) ----
// On udp_scan_done: build 773-byte packet in buffer, then stream out

reg [3:0] udp_tm_state;
localparam UTMS_IDLE = 4'd0;
localparam UTMS_BUILD = 4'd1;
localparam UTMS_SEND_HDR = 4'd2;
localparam UTMS_PRELOAD = 4'd5;   // 1-cycle BRAM read latency
localparam UTMS_SEND_DATA = 4'd3;
localparam UTMS_DONE = 4'd4;

reg [NEURON_ADDR_W-1:0] udp_tm_neuron;
reg [2:0] udp_tm_byte;
reg [7:0] udp_tm_crc;
reg [10:0] udp_buf_wr_addr;

// AXI-Stream TX — registered BRAM read for timing closure
// Data valid toggles: addr presented → 1 cycle later data in buf_rd → assert tvalid
reg [7:0] udp_tx_buf_rd;
reg       udp_tx_rd_phase;   // 0 = reading BRAM, 1 = data ready for transfer
reg       udp_tx_last_flag;  // latched last-byte indicator
always @(posedge clk_125mhz) begin
    udp_tx_buf_rd <= udp_tx_buf[udp_tx_addr];
end

assign tx_udp_payload_axis_tdata  = udp_tx_buf_rd;
assign tx_udp_payload_axis_tvalid = udp_tx_active && udp_tx_rd_phase;
assign tx_udp_payload_axis_tlast  = udp_tx_active && udp_tx_rd_phase && udp_tx_last_flag;
assign tx_udp_payload_axis_tuser  = 1'b0;
assign tx_udp_hdr_valid           = udp_tx_start;

always @(posedge clk_125mhz) begin
    if (rst_125) begin
        udp_tm_state    <= UTMS_IDLE;
        udp_tx_active   <= 1'b0;
        udp_tx_start    <= 1'b0;
        udp_tx_addr     <= 11'd0;
        tx_udp_length   <= 16'd0;
        udp_tx_rd_phase <= 1'b0;
        udp_tx_last_flag <= 1'b0;
    end else begin
        udp_tx_start <= 1'b0;

        case (udp_tm_state)
            UTMS_IDLE: begin
                if (udp_scan_done && has_host) begin
                    // Build packet in buffer
                    udp_tm_neuron  <= 0;
                    udp_tm_byte    <= 0;
                    udp_tm_crc     <= 8'd0;
                    udp_buf_wr_addr <= 11'd0;
                    udp_tm_state   <= UTMS_BUILD;
                end
            end

            UTMS_BUILD: begin
                // Write header + data + CRC into buffer
                // Header: [0x55][0x02][0x04][0x00] = 4 bytes
                // Data: 128 * 8 = 1024 bytes
                // CRC: 1 byte
                // Total: 1029 bytes
                case (udp_buf_wr_addr)
                    11'd0: begin
                        udp_tx_buf[0] <= 8'h55;
                        udp_tm_crc <= crc8_byte(8'd0, 8'h55);
                        udp_buf_wr_addr <= 11'd1;
                    end
                    11'd1: begin
                        udp_tx_buf[1] <= 8'h02;
                        udp_tm_crc <= crc8_byte(udp_tm_crc, 8'h02);
                        udp_buf_wr_addr <= 11'd2;
                    end
                    11'd2: begin
                        udp_tx_buf[2] <= 8'h04;  // LEN_HI (1024 = 0x0400)
                        udp_tm_crc <= crc8_byte(udp_tm_crc, 8'h04);
                        udp_buf_wr_addr <= 11'd3;
                    end
                    11'd3: begin
                        udp_tx_buf[3] <= 8'h00;  // LEN_LO
                        udp_tm_crc <= crc8_byte(udp_tm_crc, 8'h00);
                        udp_buf_wr_addr <= 11'd4;
                        udp_tm_neuron   <= 0;
                        udp_tm_byte     <= 0;
                    end
                    default: begin
                        if (udp_buf_wr_addr < 11'd1028) begin
                            // Neuron data — use combinational udp_nbyte
                            udp_tx_buf[udp_buf_wr_addr] <= udp_nbyte;
                            udp_tm_crc <= crc8_byte(udp_tm_crc, udp_nbyte);
                            udp_buf_wr_addr <= udp_buf_wr_addr + 11'd1;
                            if (udp_tm_byte == 3'd7) begin
                                udp_tm_byte   <= 3'd0;
                                udp_tm_neuron <= udp_tm_neuron + 1;
                            end else begin
                                udp_tm_byte <= udp_tm_byte + 3'd1;
                            end
                        end else begin
                            // CRC byte
                            udp_tx_buf[1028] <= udp_tm_crc;
                            udp_tx_len       <= 11'd1029;
                            tx_udp_length    <= 16'd1029;
                            udp_tm_state     <= UTMS_SEND_HDR;
                        end
                    end
                endcase
            end

            UTMS_SEND_HDR: begin
                // Assert UDP header valid, wait for ready
                udp_tx_start <= 1'b1;
                udp_tx_addr  <= 11'd0;  // present addr 0 to BRAM
                if (tx_udp_hdr_ready) begin
                    udp_tx_active    <= 1'b1;
                    udp_tx_rd_phase  <= 1'b0;  // reading cycle
                    udp_tx_last_flag <= 1'b0;
                    udp_tm_state     <= UTMS_PRELOAD;
                end
            end

            UTMS_PRELOAD: begin
                // 1-cycle BRAM read latency — data for udp_tx_addr now in udp_tx_buf_rd
                udp_tx_rd_phase  <= 1'b1;  // data is ready, assert tvalid
                udp_tx_last_flag <= (udp_tx_addr == udp_tx_len - 1);
                udp_tm_state     <= UTMS_SEND_DATA;
            end

            UTMS_SEND_DATA: begin
                if (tx_udp_payload_axis_tready && udp_tx_rd_phase) begin
                    // Transfer occurred
                    if (udp_tx_addr == udp_tx_len - 1) begin
                        // Last byte transferred
                        udp_tx_active   <= 1'b0;
                        udp_tx_rd_phase <= 1'b0;
                        udp_tm_state    <= UTMS_IDLE;
                    end else begin
                        // Advance to next byte — need 1-cycle BRAM read
                        udp_tx_addr     <= udp_tx_addr + 11'd1;
                        udp_tx_rd_phase <= 1'b0;  // deassert tvalid during read
                        udp_tm_state    <= UTMS_PRELOAD;
                    end
                end
            end

            default: udp_tm_state <= UTMS_IDLE;
        endcase
    end
end

// =========================================================================
// LED Drivers
// =========================================================================
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
        end else if (spike_blink_cnt != 22'd0)
            spike_blink_cnt <= spike_blink_cnt - 22'd1;
        else
            spike_blink <= 1'b0;
    end
end

reg [26:0] hb_counter;
reg        hb_led;

always @(posedge clk_sys) begin
    if (rst) begin hb_counter <= 0; hb_led <= 0; end
    else if (hb_counter == 27'd49_999_999) begin hb_counter <= 0; hb_led <= ~hb_led; end
    else hb_counter <= hb_counter + 1;
end

assign led[0] = spike_blink;
assign led[1] = kill_switch;
assign led[2] = hb_led;
assign led[3] = has_host;  // Ethernet host connected

wire [2:0] regime_color = (regime == 2'd0) ? 3'b001 :
                          (regime == 2'd1) ? 3'b100 :
                          (regime == 2'd2) ? 3'b101 :
                                             3'b010;
assign led_rgb = {regime_color, regime_color, regime_color, regime_color};

endmodule

`resetall
