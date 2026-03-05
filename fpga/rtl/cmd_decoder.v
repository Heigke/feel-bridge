// =============================================================================
// cmd_decoder.v — Binary Packet Command Decoder for NS-RAM FPGA Bridge
// =============================================================================
//
// Host-to-FPGA packet format:
//   [0xAA] [CMD] [LEN] [PAYLOAD_0 .. PAYLOAD_(LEN-1)] [CRC8]
//
// Supported commands:
//   CMD 0x01: SET_TEMPERATURE   — 4 bytes: Q16.16 temp_kelvin
//   CMD 0x02: SET_GATE_VOLTAGE  — 5 bytes: neuron_id(8) + Q16.16 Vg
//   CMD 0x03: SET_MAC_SIGNAL    — 4 bytes: Q16.16 mac_value
//   CMD 0x04: KILL_SWITCH       — 1 byte:  0x00=off, 0x01=on
//   CMD 0x05: SET_REGIME        — 1 byte:  0x00=cold, 0x01=hot, 0x02=coupled
//   CMD 0x06: READ_TELEMETRY    — 0 bytes  (request bulk readback)
//   CMD 0x07: SET_SYNAPSE       — 6 bytes: neuron_id(8) + syn_id(8) + Q16.16 weight
//   CMD 0x08: RESET             — 0 bytes
//
// CRC-8 polynomial: x^8 + x^2 + x + 1 (0x07), init=0x00, no reflect/xor
//
// State machine: IDLE -> SYNC -> CMD -> LEN -> PAYLOAD -> CRC -> VALIDATE
//
// On any framing error (bad sync, unexpected length, CRC mismatch),
// the decoder returns to IDLE and does not assert cmd_valid.
// =============================================================================

module cmd_decoder (
    input  wire        clk,
    input  wire        rst,
    input  wire [7:0]  rx_data,       // byte from uart_rx
    input  wire        rx_valid,      // 1-cycle pulse from uart_rx

    // Decoded command outputs (active for 1 cycle when cmd_valid pulses)
    output reg         cmd_valid,     // pulse: valid command decoded
    output reg  [7:0]  cmd_id,        // command type (0x01-0x08)
    output reg  [7:0]  cmd_neuron,    // neuron ID (SET_GATE_VOLTAGE, SET_SYNAPSE)
    output reg  [7:0]  cmd_syn_id,    // synapse ID (SET_SYNAPSE only)
    output reg  [31:0] cmd_data,      // Q16.16 payload data
    output reg         cmd_kill,      // kill switch state (latched)
    output reg  [1:0]  cmd_regime,    // regime mode (latched)
    output reg         cmd_read_telem,// pulse: telemetry read requested
    output reg         cmd_reset      // pulse: global reset requested
);

    // -------------------------------------------------------------------------
    // Command IDs
    // -------------------------------------------------------------------------
    localparam CMD_SET_TEMP     = 8'h01;
    localparam CMD_SET_GATE     = 8'h02;
    localparam CMD_SET_MAC      = 8'h03;
    localparam CMD_KILL_SWITCH  = 8'h04;
    localparam CMD_SET_REGIME   = 8'h05;
    localparam CMD_READ_TELEM   = 8'h06;
    localparam CMD_SET_SYNAPSE  = 8'h07;
    localparam CMD_RESET        = 8'h08;

    // -------------------------------------------------------------------------
    // Expected payload lengths per command (used for validation)
    // -------------------------------------------------------------------------
    function [3:0] expected_len;
        input [7:0] cmd;
        case (cmd)
            CMD_SET_TEMP:    expected_len = 4'd4;  // Q16.16
            CMD_SET_GATE:    expected_len = 4'd5;  // neuron_id + Q16.16
            CMD_SET_MAC:     expected_len = 4'd4;  // Q16.16
            CMD_KILL_SWITCH: expected_len = 4'd1;  // on/off byte
            CMD_SET_REGIME:  expected_len = 4'd1;  // regime byte
            CMD_READ_TELEM:  expected_len = 4'd0;  // no payload
            CMD_SET_SYNAPSE: expected_len = 4'd6;  // neuron + syn + Q16.16
            CMD_RESET:       expected_len = 4'd0;  // no payload
            default:         expected_len = 4'hF;  // invalid sentinel
        endcase
    endfunction

    // -------------------------------------------------------------------------
    // State encoding
    // -------------------------------------------------------------------------
    localparam S_IDLE    = 3'd0; // waiting for sync byte 0xAA
    localparam S_CMD     = 3'd1; // waiting for command byte
    localparam S_LEN     = 3'd2; // waiting for length byte
    localparam S_PAYLOAD = 3'd3; // collecting payload bytes
    localparam S_CRC     = 3'd4; // waiting for CRC byte
    localparam S_VALID   = 3'd5; // CRC matched — dispatch command

    // -------------------------------------------------------------------------
    // Registers
    // -------------------------------------------------------------------------
    reg [2:0]  state;
    reg [7:0]  pkt_cmd;           // latched command byte
    reg [3:0]  pkt_len;           // latched payload length (max 6)
    reg [3:0]  pay_idx;           // payload byte index
    reg [47:0] pay_buf;           // payload buffer (max 6 bytes = 48 bits)
    reg [7:0]  crc_accum;         // running CRC-8 accumulator

    // -------------------------------------------------------------------------
    // CRC-8 single-byte calculation (polynomial 0x07)
    // Computes new CRC given current accumulator and input byte.
    // Bit-serial implementation: 8 iterations of shift-and-XOR.
    // -------------------------------------------------------------------------
    function [7:0] crc8_byte;
        input [7:0] crc_in;
        input [7:0] data_in;
        reg [7:0] crc;
        integer i;
        begin
            crc = crc_in ^ data_in;
            for (i = 0; i < 8; i = i + 1) begin
                if (crc[7])
                    crc = {crc[6:0], 1'b0} ^ 8'h07;
                else
                    crc = {crc[6:0], 1'b0};
            end
            crc8_byte = crc;
        end
    endfunction

    // -------------------------------------------------------------------------
    // Main state machine — advances on each rx_valid pulse
    // -------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            state       <= S_IDLE;
            cmd_valid   <= 1'b0;
            cmd_id      <= 8'd0;
            cmd_neuron  <= 8'd0;
            cmd_syn_id  <= 8'd0;
            cmd_data    <= 32'd0;
            cmd_kill    <= 1'b0;
            cmd_regime  <= 2'd0;
            cmd_read_telem <= 1'b0;
            cmd_reset   <= 1'b0;
            pkt_cmd     <= 8'd0;
            pkt_len     <= 4'd0;
            pay_idx     <= 4'd0;
            pay_buf     <= 48'd0;
            crc_accum   <= 8'd0;
        end else begin
            // Pulses default low each cycle
            cmd_valid      <= 1'b0;
            cmd_read_telem <= 1'b0;
            cmd_reset      <= 1'b0;

            if (rx_valid) begin
                case (state)
                    // ---------------------------------------------------------
                    // IDLE: scan for sync byte 0xAA
                    // Any other byte is silently discarded.
                    // ---------------------------------------------------------
                    S_IDLE: begin
                        if (rx_data == 8'hAA) begin
                            crc_accum <= 8'h00; // reset CRC at start of packet
                            state     <= S_CMD;
                        end
                        // else: stay in IDLE, ignore byte
                    end

                    // ---------------------------------------------------------
                    // CMD: receive command byte, validate it is known
                    // ---------------------------------------------------------
                    S_CMD: begin
                        if (rx_data >= CMD_SET_TEMP && rx_data <= CMD_RESET) begin
                            pkt_cmd   <= rx_data;
                            crc_accum <= crc8_byte(crc_accum, rx_data);
                            state     <= S_LEN;
                        end else begin
                            // Unknown command — framing error, return to IDLE
                            state <= S_IDLE;
                        end
                    end

                    // ---------------------------------------------------------
                    // LEN: receive payload length, validate against expected
                    // ---------------------------------------------------------
                    S_LEN: begin
                        crc_accum <= crc8_byte(crc_accum, rx_data);

                        // Validate length matches the command's expected length
                        if (rx_data[3:0] == expected_len(pkt_cmd) &&
                            rx_data[7:4] == 4'd0) begin
                            pkt_len <= rx_data[3:0];
                            pay_idx <= 4'd0;
                            pay_buf <= 48'd0;

                            if (rx_data[3:0] == 4'd0) begin
                                // No payload — go straight to CRC
                                state <= S_CRC;
                            end else begin
                                state <= S_PAYLOAD;
                            end
                        end else begin
                            // Length mismatch — framing error
                            state <= S_IDLE;
                        end
                    end

                    // ---------------------------------------------------------
                    // PAYLOAD: collect LEN bytes into pay_buf (MSB first)
                    // Bytes are stored big-endian: first byte in highest position.
                    // ---------------------------------------------------------
                    S_PAYLOAD: begin
                        crc_accum <= crc8_byte(crc_accum, rx_data);

                        // Shift buffer left by 8 and insert new byte at bottom
                        pay_buf <= {pay_buf[39:0], rx_data};
                        pay_idx <= pay_idx + 4'd1;

                        if (pay_idx + 4'd1 == pkt_len) begin
                            state <= S_CRC;
                        end
                    end

                    // ---------------------------------------------------------
                    // CRC: receive CRC byte, compare with accumulated CRC
                    // The CRC covers CMD + LEN + PAYLOAD (not the sync byte).
                    // ---------------------------------------------------------
                    S_CRC: begin
                        if (crc_accum == rx_data) begin
                            // CRC match — command is valid
                            state <= S_VALID;
                        end else begin
                            // CRC mismatch — discard packet
                            state <= S_IDLE;
                        end
                    end

                    // ---------------------------------------------------------
                    // VALID: should not receive bytes here; handled below
                    // ---------------------------------------------------------
                    S_VALID: begin
                        // Unexpected byte during dispatch — ignore, go to IDLE
                        state <= S_IDLE;
                    end

                    default: state <= S_IDLE;
                endcase
            end // rx_valid

            // -----------------------------------------------------------------
            // Command dispatch (non-rx_valid cycle after entering S_VALID)
            // This runs on the cycle AFTER the CRC check passes.
            // -----------------------------------------------------------------
            if (state == S_VALID) begin
                cmd_valid <= 1'b1;
                cmd_id    <= pkt_cmd;

                case (pkt_cmd)
                    // SET_TEMPERATURE: payload = Q16.16 (4 bytes)
                    CMD_SET_TEMP: begin
                        cmd_data <= pay_buf[31:0];
                    end

                    // SET_GATE_VOLTAGE: payload = neuron_id(8) + Q16.16 (5 bytes)
                    // pay_buf layout after 5 bytes: [neuron_id | Q16.16_MSB..LSB]
                    CMD_SET_GATE: begin
                        cmd_neuron <= pay_buf[39:32];
                        cmd_data   <= pay_buf[31:0];
                    end

                    // SET_MAC_SIGNAL: payload = Q16.16 (4 bytes)
                    CMD_SET_MAC: begin
                        cmd_data <= pay_buf[31:0];
                    end

                    // KILL_SWITCH: payload = 1 byte (0x00=off, 0x01=on)
                    CMD_KILL_SWITCH: begin
                        cmd_kill <= pay_buf[0];
                    end

                    // SET_REGIME: payload = 1 byte (0x00=cold, 0x01=hot, 0x02=coupled)
                    CMD_SET_REGIME: begin
                        cmd_regime <= pay_buf[1:0];
                    end

                    // READ_TELEMETRY: no payload — just pulse the request
                    CMD_READ_TELEM: begin
                        cmd_read_telem <= 1'b1;
                    end

                    // SET_SYNAPSE: payload = neuron_id(8) + syn_id(8) + Q16.16 (6 bytes)
                    // pay_buf layout after 6 bytes: [neuron | syn | Q16.16_MSB..LSB]
                    CMD_SET_SYNAPSE: begin
                        cmd_neuron <= pay_buf[47:40];
                        cmd_syn_id <= pay_buf[39:32];
                        cmd_data   <= pay_buf[31:0];
                    end

                    // RESET: no payload — just pulse the reset
                    CMD_RESET: begin
                        cmd_reset <= 1'b1;
                    end

                    default: begin
                        // Should not reach here (validated in S_CMD)
                        cmd_valid <= 1'b0;
                    end
                endcase

                state <= S_IDLE;
            end
        end
    end

endmodule
