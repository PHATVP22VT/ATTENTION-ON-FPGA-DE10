// =============================================================================
// MODULE: uart_tx
// Chức năng: Gửi một byte dữ liệu ra đường UART nối tiếp
//
// Giao thức: 8N1 — 1 start bit + 8 data bits (LSB trước) + 1 stop bit
//
// Cách dùng:
//   1. Đặt i_tx_data = byte cần gửi
//   2. Bật i_tx_start = 1 đúng 1 clock
//   3. Chờ o_tx_busy = 0 → byte đã gửi xong, có thể gửi byte tiếp
//
// Sơ đồ thời gian:
//   i_tx_start  ‾|_______
//   o_tx_busy   _|‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾|_
//   o_tx        ‾|_start_|d0|d1|...|d7|‾stop‾|
// =============================================================================
module uart_tx #(
    parameter int CLK_FREQ  = 50_000_000,
    parameter int BAUD_RATE = 115_200
)(
    input  logic       i_clk,
    input  logic       i_rst_n,
    input  logic [7:0] i_tx_data,   // Byte cần gửi
    input  logic       i_tx_start,  // Pulse 1 clock để bắt đầu
    output logic       o_tx,        // Chân TX nối đến FT232R
    output logic       o_tx_busy    // =1 khi đang gửi, =0 khi rảnh
);

    localparam int CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;  // ≈ 434

    typedef enum logic [2:0] {
        IDLE,   // Đường TX giữ HIGH (idle)
        START,  // Gửi start bit (LOW)
        DATA,   // Gửi 8 bit dữ liệu
        STOP    // Gửi stop bit (HIGH)
    } state_t;
    state_t state;

    logic [15:0] clk_cnt;
    logic [2:0]  bit_idx;
    logic [7:0]  tx_shift;   // Thanh ghi giữ byte đang gửi

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            state    <= IDLE;
            o_tx     <= 1'b1;   // Idle = HIGH
            o_tx_busy <= 1'b0;
            clk_cnt  <= '0;
            bit_idx  <= '0;
            tx_shift <= '0;
        end else begin
            case (state)

                // ── Chờ lệnh gửi ──
                IDLE: begin
                    o_tx      <= 1'b1;
                    o_tx_busy <= 1'b0;
                    clk_cnt   <= '0;
                    bit_idx   <= '0;
                    if (i_tx_start) begin
                        tx_shift  <= i_tx_data;   // Chốt dữ liệu ngay khi nhận lệnh
                        o_tx_busy <= 1'b1;
                        state     <= START;
                    end
                end

                // ── Gửi start bit: kéo TX xuống LOW trong 1 bit-time ──
                START: begin
                    o_tx <= 1'b0;
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt <= '0;
                        state   <= DATA;
                    end else begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                end

                // ── Gửi 8 bit: LSB trước, mỗi bit kéo dài 1 bit-time ──
                DATA: begin
                    o_tx <= tx_shift[0];  // Luôn gửi bit thấp nhất
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt  <= '0;
                        tx_shift <= {1'b0, tx_shift[7:1]};  // Dịch phải
                        if (bit_idx == 3'd7) begin
                            bit_idx <= '0;
                            state   <= STOP;
                        end else begin
                            bit_idx <= bit_idx + 1'b1;
                        end
                    end else begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                end

                // ── Gửi stop bit: TX = HIGH trong 1 bit-time ──
                STOP: begin
                    o_tx <= 1'b1;
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt   <= '0;
                        o_tx_busy <= 1'b0;
                        state     <= IDLE;
                    end else begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                end

                default: begin
                    o_tx  <= 1'b1;
                    state <= IDLE;
                end
            endcase
        end
    end

endmodule