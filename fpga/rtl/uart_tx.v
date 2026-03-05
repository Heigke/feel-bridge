// =============================================================================
// uart_tx.v — UART Transmitter for NS-RAM FPGA Bridge
// =============================================================================
// 921600 baud, 8N1, 100 MHz system clock
// Baud divider: 100_000_000 / 921_600 = 109 clocks per bit
//
// Protocol: [START=0] [D0..D7 LSB first] [STOP=1]
// Total: 10 bit periods per frame
//
// Usage: load data, assert start for 1 cycle. busy goes high immediately
// and stays high until the stop bit has been fully transmitted.
// Do not assert start while busy is high.
// =============================================================================

module uart_tx (
    input  wire       clk,    // 100 MHz system clock
    input  wire       rst,    // synchronous reset, active high
    input  wire [7:0] data,   // byte to transmit
    input  wire       start,  // 1-cycle pulse to begin transmission
    output reg        tx,     // serial output line (idles high)
    output reg        busy    // 1 while transmitting
);

    // -------------------------------------------------------------------------
    // Parameters
    // -------------------------------------------------------------------------
    localparam CLKS_PER_BIT = 868; // 100 MHz / 115200 baud (debug: reverted from 921600)

    // -------------------------------------------------------------------------
    // State encoding
    // -------------------------------------------------------------------------
    localparam S_IDLE  = 2'd0;
    localparam S_START = 2'd1;
    localparam S_DATA  = 2'd2;
    localparam S_STOP  = 2'd3;

    // -------------------------------------------------------------------------
    // Registers
    // -------------------------------------------------------------------------
    reg [1:0]  state;
    reg [9:0]  baud_cnt;   // counts clocks within one bit period
    reg [2:0]  bit_idx;    // which data bit we are sending (0-7)
    reg [7:0]  shift_reg;  // latched copy of data to transmit

    // -------------------------------------------------------------------------
    // Main state machine
    // -------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            state     <= S_IDLE;
            tx        <= 1'b1;  // line idles high
            busy      <= 1'b0;
            baud_cnt  <= 10'd0;
            bit_idx   <= 3'd0;
            shift_reg <= 8'd0;
        end else begin
            case (state)
                // ---------------------------------------------------------
                // IDLE: wait for start pulse, line held high
                // ---------------------------------------------------------
                S_IDLE: begin
                    tx   <= 1'b1;
                    busy <= 1'b0;
                    if (start) begin
                        shift_reg <= data;      // latch the byte
                        busy      <= 1'b1;
                        baud_cnt  <= 10'd0;
                        state     <= S_START;
                    end
                end

                // ---------------------------------------------------------
                // START: drive tx low for one bit period
                // ---------------------------------------------------------
                S_START: begin
                    tx <= 1'b0; // start bit
                    if (baud_cnt == CLKS_PER_BIT - 1) begin
                        baud_cnt <= 10'd0;
                        bit_idx  <= 3'd0;
                        state    <= S_DATA;
                    end else begin
                        baud_cnt <= baud_cnt + 10'd1;
                    end
                end

                // ---------------------------------------------------------
                // DATA: send 8 bits LSB first, one bit period each
                // ---------------------------------------------------------
                S_DATA: begin
                    tx <= shift_reg[0]; // current bit (LSB)
                    if (baud_cnt == CLKS_PER_BIT - 1) begin
                        baud_cnt  <= 10'd0;
                        shift_reg <= {1'b0, shift_reg[7:1]}; // shift right
                        if (bit_idx == 3'd7) begin
                            state <= S_STOP;
                        end else begin
                            bit_idx <= bit_idx + 3'd1;
                        end
                    end else begin
                        baud_cnt <= baud_cnt + 10'd1;
                    end
                end

                // ---------------------------------------------------------
                // STOP: drive tx high for one bit period, then return idle
                // ---------------------------------------------------------
                S_STOP: begin
                    tx <= 1'b1; // stop bit
                    if (baud_cnt == CLKS_PER_BIT - 1) begin
                        baud_cnt <= 10'd0;
                        busy     <= 1'b0;
                        state    <= S_IDLE;
                    end else begin
                        baud_cnt <= baud_cnt + 10'd1;
                    end
                end

                default: begin
                    state <= S_IDLE;
                    tx    <= 1'b1;
                end
            endcase
        end
    end

endmodule
