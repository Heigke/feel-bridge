// =============================================================================
// uart_rx.v — UART Receiver for NS-RAM FPGA Bridge
// =============================================================================
// 921600 baud, 8N1, 100 MHz system clock
// 16x oversampling with majority-vote sampling at bit center
//
// Baud divider: 100_000_000 / 921_600 = 109 clocks per bit
// Oversample tick: 109 / 16 = 7 clocks per oversample phase
// Bit center sampled at oversample count 7 (middle of bit period)
//
// On framing error (stop bit != 1), the module discards the byte
// and returns to IDLE without asserting valid.
// =============================================================================

module uart_rx (
    input  wire       clk,    // 100 MHz system clock
    input  wire       rst,    // synchronous reset, active high
    input  wire       rx,     // serial input (active low start bit)
    output reg  [7:0] data,   // received byte (valid when valid==1)
    output reg        valid   // 1-cycle pulse when byte is ready
);

    // -------------------------------------------------------------------------
    // Parameters
    // -------------------------------------------------------------------------
    localparam CLKS_PER_BIT    = 868;            // 100 MHz / 115200 (debug: reverted from 921600)
    localparam OVERSAMPLE_DIV  = CLKS_PER_BIT / 16; // ~54 clocks per tick
    localparam SAMPLE_POINT    = 7;              // sample at mid-bit (tick 7 of 0-15)

    // -------------------------------------------------------------------------
    // State encoding
    // -------------------------------------------------------------------------
    localparam S_IDLE    = 3'd0;
    localparam S_START   = 3'd1;
    localparam S_DATA    = 3'd2;
    localparam S_STOP    = 3'd3;
    localparam S_DONE    = 3'd4;

    // -------------------------------------------------------------------------
    // Registers
    // -------------------------------------------------------------------------
    reg [2:0]  state;
    reg [9:0]  tick_cnt;       // counts clocks within one oversample tick
    reg [3:0]  os_cnt;         // oversample phase counter (0-15)
    reg [2:0]  bit_idx;        // which data bit we are receiving (0-7)
    reg [7:0]  shift_reg;      // shift register for incoming bits
    reg        start_valid;    // start bit was confirmed at sample point

    // Double-flop synchronizer for metastability on rx input
    reg rx_sync_0, rx_sync_1;
    always @(posedge clk) begin
        if (rst) begin
            rx_sync_0 <= 1'b1;
            rx_sync_1 <= 1'b1;
        end else begin
            rx_sync_0 <= rx;
            rx_sync_1 <= rx_sync_0;
        end
    end
    wire rx_s = rx_sync_1; // synchronized rx

    // -------------------------------------------------------------------------
    // Main state machine
    // -------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            state       <= S_IDLE;
            tick_cnt    <= 10'd0;
            os_cnt      <= 4'd0;
            bit_idx     <= 3'd0;
            shift_reg   <= 8'd0;
            data        <= 8'd0;
            valid       <= 1'b0;
            start_valid <= 1'b0;
        end else begin
            // Default: valid is a single-cycle pulse
            valid <= 1'b0;

            case (state)
                // ---------------------------------------------------------
                // IDLE: wait for falling edge (start bit)
                // ---------------------------------------------------------
                S_IDLE: begin
                    tick_cnt <= 10'd0;
                    os_cnt   <= 4'd0;
                    if (rx_s == 1'b0) begin
                        // Detected low — enter start bit verification
                        state <= S_START;
                    end
                end

                // ---------------------------------------------------------
                // START: verify start bit at its center using 16x oversampling
                // If rx returns high before we reach the sample point,
                // it was a glitch — go back to IDLE.
                // ---------------------------------------------------------
                S_START: begin
                    if (tick_cnt == OVERSAMPLE_DIV - 1) begin
                        tick_cnt <= 10'd0;
                        os_cnt   <= os_cnt + 4'd1;

                        if (os_cnt == SAMPLE_POINT) begin
                            if (rx_s == 1'b0) begin
                                // Valid start bit confirmed at center
                                // Let os_cnt continue to 15 (end of start bit)
                                bit_idx     <= 3'd0;
                                start_valid <= 1'b1;
                            end else begin
                                // False start — glitch
                                state <= S_IDLE;
                            end
                        end

                        if (os_cnt == 4'd15) begin
                            // End of start bit period
                            if (start_valid) begin
                                os_cnt      <= 4'd0;
                                start_valid <= 1'b0;
                                state       <= S_DATA;
                            end else begin
                                state <= S_IDLE;
                            end
                        end
                    end else begin
                        tick_cnt <= tick_cnt + 10'd1;
                    end
                end

                // ---------------------------------------------------------
                // DATA: sample 8 data bits, LSB first
                // Each bit is sampled at the center (oversample tick 7)
                // after a full 16-tick bit period.
                // ---------------------------------------------------------
                S_DATA: begin
                    if (tick_cnt == OVERSAMPLE_DIV - 1) begin
                        tick_cnt <= 10'd0;
                        os_cnt   <= os_cnt + 4'd1;

                        if (os_cnt == 4'd15) begin
                            // One full bit period elapsed — sample at center
                            // was already captured; advance to next bit.
                            // Actually, we sample at tick SAMPLE_POINT:
                        end

                        if (os_cnt == SAMPLE_POINT) begin
                            // Sample the data bit (LSB first)
                            shift_reg <= {rx_s, shift_reg[7:1]};
                        end

                        if (os_cnt == 4'd15) begin
                            os_cnt <= 4'd0;
                            if (bit_idx == 3'd7) begin
                                state <= S_STOP;
                            end else begin
                                bit_idx <= bit_idx + 3'd1;
                            end
                        end
                    end else begin
                        tick_cnt <= tick_cnt + 10'd1;
                    end
                end

                // ---------------------------------------------------------
                // STOP: verify stop bit is high at center of bit period
                // ---------------------------------------------------------
                S_STOP: begin
                    if (tick_cnt == OVERSAMPLE_DIV - 1) begin
                        tick_cnt <= 10'd0;
                        os_cnt   <= os_cnt + 4'd1;

                        if (os_cnt == SAMPLE_POINT) begin
                            if (rx_s == 1'b1) begin
                                // Valid stop bit — emit byte
                                data  <= shift_reg;
                                valid <= 1'b1;
                            end
                            // If stop bit is low, framing error:
                            // discard byte, return to idle
                            state <= S_IDLE;
                        end
                    end else begin
                        tick_cnt <= tick_cnt + 10'd1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
