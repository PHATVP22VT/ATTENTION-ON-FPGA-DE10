// =============================================================================
// MODULE: uart_controller  (v2 - fixed)
//
// Các lỗi đã sửa so với v1:
//   Bug 1: rx_byte_cnt đếm word (0..191) nhưng so sánh với TOTAL_RX_BYTES=384
//          → Sửa: đếm word_cnt (0..191), so sánh với TOTAL_WORDS_IN-1=191
//   Bug 2: Trong WAIT_CMD check "if (state == DONE)" luôn false
//          → Sửa: dùng cờ result_ready riêng
//
// Giao thức UART:
//   PC → 0xAA  → gửi 384 byte X (big-endian, 16-bit/word, word0_hi, word0_lo, ...)
//   PC → 0x55  → yêu cầu đọc kết quả Y
//   FPGA → 18 byte Y (9 word × 2 byte, big-endian)
// =============================================================================
module uart_controller #(
    parameter int CLK_FREQ  = 50_000_000,
    parameter int BAUD_RATE = 115_200,
    parameter int SEQ_LEN   = 3,
    parameter int D_MODEL   = 64
)(
    input  logic        i_clk,
    input  logic        i_rst_n,

    // Kết nối uart_rx
    input  logic [7:0]  i_rx_data,
    input  logic        i_rx_valid,

    // Kết nối uart_tx
    output logic [7:0]  o_tx_data,
    output logic        o_tx_start,
    input  logic        i_tx_busy,

    // Cổng ghi RAM đệm X (word address 0..191)
    output logic        o_rx_ram_we,
    output logic [7:0]  o_rx_ram_addr,
    output logic [15:0] o_rx_ram_data,

    // Điều khiển attention_top
    output logic        o_start_linear,
    output logic [1:0]  o_token_sel,
    input  logic        i_linear_done,
    output logic        o_start_attn,
    input  logic        i_attn_done,
    output logic        o_start_softmax,
    input  logic        i_softmax_done,

    // Kết quả Y từ attention_top
    input  logic        i_y_we,
    input  logic [3:0]  i_y_addr,
    input  logic signed [15:0] i_y_data,

    // Debug
    output logic        o_result_ready,
    output logic [3:0]  o_state_debug   // 0..12
);

    // 192 word vào, 9 word ra
    localparam int TOTAL_WORDS_IN  = SEQ_LEN * D_MODEL;   // 192
    localparam int TOTAL_WORDS_OUT = SEQ_LEN * SEQ_LEN;   // 9

    localparam logic [7:0] CMD_START = 8'hAA;
    localparam logic [7:0] CMD_READ  = 8'h55;

    typedef enum logic [3:0] {
        WAIT_CMD,       // 0
        RECV_DATA,      // 1
        RUN_LINEAR,     // 2
        WAIT_LINEAR,    // 3
        RUN_ATTN,       // 4
        WAIT_ATTN,      // 5
        RUN_SOFTMAX,    // 6
        WAIT_SOFTMAX,   // 7
        // 8: (không dùng — DONE đã hợp nhất vào WAIT_CMD)
        SEND_RESULT  = 4'd9,   // 9
        WAIT_TX_LOAD = 4'd10,  // 10
        WAIT_TX_BUSY = 4'd11,  // 11
        WAIT_TX_DONE = 4'd12   // 12
    } state_t;
    state_t state;

    assign o_state_debug  = state[3:0];
    logic        result_ready;
    assign o_result_ready = result_ready;

    logic [7:0]  rx_word_cnt;   // Đếm word 0..191
    logic        rx_high_byte;  // =1 đang chờ byte cao
    logic [7:0]  rx_high_val;

    
    logic signed [15:0] y_result [0:8];

    logic [3:0]  tx_word_idx;
    logic        tx_send_high;
    logic [1:0]  token_cnt;

    // Chốt kết quả Y
    always_ff @(posedge i_clk) begin
        if (i_y_we)
            y_result[i_y_addr] <= i_y_data;
    end

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            state           <= WAIT_CMD;
            rx_word_cnt     <= '0;
            rx_high_byte    <= 1'b1;
            rx_high_val     <= '0;
            o_rx_ram_we     <= 1'b0;
            o_rx_ram_addr   <= '0;
            o_rx_ram_data   <= '0;
            o_start_linear  <= 1'b0;
            o_token_sel     <= 2'b00;
            o_start_attn    <= 1'b0;
            o_start_softmax <= 1'b0;
            o_tx_data       <= '0;
            o_tx_start      <= 1'b0;
            tx_word_idx     <= '0;
            tx_send_high    <= 1'b1;
            token_cnt       <= 2'b00;
            result_ready    <= 1'b0;
        end else begin
            o_rx_ram_we     <= 1'b0;
            o_start_linear  <= 1'b0;
            o_start_attn    <= 1'b0;
            o_start_softmax <= 1'b0;
            o_tx_start      <= 1'b0;

            case (state)

                WAIT_CMD: begin
                    if (i_rx_valid) begin
                        if (i_rx_data == CMD_START) begin
                            rx_word_cnt  <= '0;
                            rx_high_byte <= 1'b1;
                            result_ready <= 1'b0;
                            state        <= RECV_DATA;
                        end else if (i_rx_data == CMD_READ && result_ready) begin
                            // Bug 2 fix: check cờ result_ready
                            tx_word_idx  <= '0;
                            tx_send_high <= 1'b1;
                            state        <= SEND_RESULT;
                        end
                    end
                end

                RECV_DATA: begin
                    if (i_rx_valid) begin
                        if (rx_high_byte) begin
                            rx_high_val  <= i_rx_data;
                            rx_high_byte <= 1'b0;
                        end else begin
                            // Bug 1 fix: địa chỉ = rx_word_cnt (0..191), so sánh với 191
                            o_rx_ram_we   <= 1'b1;
                            o_rx_ram_addr <= rx_word_cnt;
                            o_rx_ram_data <= {rx_high_val, i_rx_data};
                            rx_high_byte  <= 1'b1;

                            if (rx_word_cnt == TOTAL_WORDS_IN - 1) begin
                                token_cnt <= 2'b00;
                                state     <= RUN_LINEAR;
                            end else begin
                                rx_word_cnt <= rx_word_cnt + 1'b1;
                            end
                        end
                    end
                end

                RUN_LINEAR: begin
                    o_start_linear <= 1'b1;
                    o_token_sel    <= token_cnt;
                    state          <= WAIT_LINEAR;
                end

                WAIT_LINEAR: begin
                    if (i_linear_done) begin
                        if (token_cnt == SEQ_LEN - 1)
                            state <= RUN_ATTN;
                        else begin
                            token_cnt <= token_cnt + 1'b1;
                            state     <= RUN_LINEAR;
                        end
                    end
                end

                RUN_ATTN: begin
                    o_start_attn <= 1'b1;
                    state        <= WAIT_ATTN;
                end

                WAIT_ATTN: begin
                    if (i_attn_done) state <= RUN_SOFTMAX;
                end

                RUN_SOFTMAX: begin
                    o_start_softmax <= 1'b1;
                    state           <= WAIT_SOFTMAX;
                end

                WAIT_SOFTMAX: begin
                    if (i_softmax_done) begin
                        result_ready <= 1'b1;
                        state        <= WAIT_CMD;  // Fix: về WAIT_CMD thay vì DONE
                    end
                end

                // State DONE đã được hợp nhất vào WAIT_CMD ở trên.
                // WAIT_CMD đã xử lý CMD_READ khi result_ready=1 và CMD_START để reset.

                SEND_RESULT: begin
                    // 1. Chuẩn bị dữ liệu (combinational để có ngay trong cycle này)
                    //    o_tx_data phải stable TRƯỚC khi o_tx_start lên 1
                    if (tx_send_high) o_tx_data <= y_result[tx_word_idx][15:8];
                    else              o_tx_data <= y_result[tx_word_idx][7:0];
                    state <= WAIT_TX_LOAD; // Thêm 1 cycle chờ data stable
                end

                WAIT_TX_LOAD: begin
                    // 1b. Data đã stable từ cycle trước, giờ mới kích hoạt TX
                    o_tx_start <= 1'b1;
                    state      <= WAIT_TX_BUSY;
                end

                WAIT_TX_BUSY: begin
                    // 2. Ép start giữ mức 1 cho đến khi TX báo busy
                    if (i_tx_busy) begin
                        state <= WAIT_TX_DONE;
                    end else begin
                        o_tx_start <= 1'b1; 
                    end
                end

                WAIT_TX_DONE: begin
                    // 3. Chờ TX gửi xong byte hiện tại rồi mới nhảy sang byte tiếp theo
                    if (!i_tx_busy) begin
                        if (tx_send_high) begin
                            tx_send_high <= 1'b0;
                            state        <= SEND_RESULT; // Quay lại gửi byte Low
                        end else begin
                            tx_send_high <= 1'b1;
                            if (tx_word_idx == TOTAL_WORDS_OUT - 1)
                                state <= WAIT_CMD;       // Đã gửi xong đủ 18 byte
                            else begin
                                tx_word_idx <= tx_word_idx + 1'b1;
                                state       <= SEND_RESULT; // Chuyển sang word tiếp theo
                            end
                        end
                    end
                end

                default: state <= WAIT_CMD;
            endcase
        end
    end

    

endmodule