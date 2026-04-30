// =============================================================================
// MODULE: attention_top_with_uart  (TOP-LEVEL cho DE10-Standard)
//
// Chức năng: Bọc toàn bộ hệ thống, kết nối UART với attention_top
//            và hiển thị kết quả Y lên 6 màn hình HEX 7-đoạn
//
// Sơ đồ luồng:
//   PC ──USB──► FT232R ──UART──► uart_rx ──► uart_controller
//                                                   │
//                                            ghi u_input_ram
//                                                   │ start_*
//                                                   ▼
//                                           attention_top
//                                                   │ Y results
//                                          ┌────────┴────────┐
//                                          ▼                 ▼
//                                     uart_tx          hex_display
//                                     (→ PC)      (→ HEX5..HEX0)
//
// Pin mapping DE10-Standard:
//   CLOCK_50  → PIN_AF14
//   KEY[0]    → PIN_AJ4   (active-low reset)
//   KEY[1]    → PIN_AH4   (active-low, chuyển hàng HEX)
//   UART_RXD  → PIN_W15
//   UART_TXD  → PIN_AA15
//   o_result_ready → PIN_AA24 (LED hoặc debug pin)
//
//   HEX0[6:0] → PIN_AJ17 AJ18 AH18 AH19 AF18 AF19 AE19
//   HEX1[6:0] → PIN_AE18 AD18 AC18 AC19 AB18 AB19 AA19
//   HEX2[6:0] → PIN_AA18 Y18  Y19  W19  W18  V18  V19
//   HEX3[6:0] → PIN_U19  U18  T18  T19  R18  R19  P19
//   HEX4[6:0] → PIN_P18  N19  N18  M18  L18  L19  K18
//   HEX5[6:0] → PIN_K19  J18  J19  H18  H19  G18  G19
// =============================================================================
module attention_top_with_uart #(
    parameter int CLK_FREQ   = 50_000_000,
    parameter int BAUD_RATE  = 115_200,
    parameter int SEQ_LEN    = 3,
    parameter int D_MODEL    = 64,
    parameter int DATA_WIDTH = 16
)(
    input  logic        CLOCK_50,
    input  logic [1:0]  KEY,        // KEY[0]=reset, KEY[1]=chuyển hàng HEX
    input  logic        UART_RXD,
    output logic        UART_TXD,
    output logic        o_result_ready,

    // 6 màn hình HEX 7-đoạn (active-low)
    output logic [6:0]  HEX0,
    output logic [6:0]  HEX1,
    output logic [6:0]  HEX2,
    output logic [6:0]  HEX3,
    output logic [6:0]  HEX4,
    output logic [6:0]  HEX5
);

    wire clk   = CLOCK_50;
    wire rst_n = KEY[0];

    // ─────────────────────────────────────────────────────────────────────
    // Tín hiệu nội bộ
    // ─────────────────────────────────────────────────────────────────────
    logic [7:0]  rx_data;
    logic        rx_valid;

    logic [7:0]  tx_data;
    logic        tx_start;
    logic        tx_busy;

    logic        ctrl_rx_ram_we;
    logic [7:0]  ctrl_rx_ram_addr;
    logic [15:0] ctrl_rx_ram_data;

    logic        ctrl_start_linear;
    logic [1:0]  ctrl_token_sel;
    logic        ctrl_linear_done;
    logic        ctrl_start_attn;
    logic        ctrl_attn_done;
    logic        ctrl_start_softmax;
    logic        ctrl_softmax_done;

    logic        y_we;
    logic [3:0]  y_addr;
    logic signed [DATA_WIDTH-1:0] y_data;

    logic        result_ready_int;
    assign o_result_ready = result_ready_int;

    // ─────────────────────────────────────────────────────────────────────
    // 1. UART Receiver
    // ─────────────────────────────────────────────────────────────────────
    uart_rx #(
        .CLK_FREQ(CLK_FREQ), .BAUD_RATE(BAUD_RATE)
    ) u_uart_rx (
        .i_clk     (clk),
        .i_rst_n   (rst_n),
        .i_rx      (UART_RXD),
        .o_rx_data (rx_data),
        .o_rx_valid(rx_valid)
    );

    // ─────────────────────────────────────────────────────────────────────
    // 2. UART Transmitter
    // ─────────────────────────────────────────────────────────────────────
    uart_tx #(
        .CLK_FREQ(CLK_FREQ), .BAUD_RATE(BAUD_RATE)
    ) u_uart_tx (
        .i_clk    (clk),
        .i_rst_n  (rst_n),
        .i_tx_data (tx_data),
        .i_tx_start(tx_start),
        .o_tx      (UART_TXD),
        .o_tx_busy (tx_busy)
    );

    // ─────────────────────────────────────────────────────────────────────
    // 3. UART Controller
    // ─────────────────────────────────────────────────────────────────────
    uart_controller #(
        .CLK_FREQ(CLK_FREQ), .BAUD_RATE(BAUD_RATE),
        .SEQ_LEN(SEQ_LEN),   .D_MODEL(D_MODEL)
    ) u_ctrl (
        .i_clk           (clk),
        .i_rst_n         (rst_n),
        .i_rx_data       (rx_data),
        .i_rx_valid      (rx_valid),
        .o_tx_data       (tx_data),
        .o_tx_start      (tx_start),
        .i_tx_busy       (tx_busy),
        .o_rx_ram_we     (ctrl_rx_ram_we),
        .o_rx_ram_addr   (ctrl_rx_ram_addr),
        .o_rx_ram_data   (ctrl_rx_ram_data),
        .o_start_linear  (ctrl_start_linear),
        .o_token_sel     (ctrl_token_sel),
        .i_linear_done   (ctrl_linear_done),
        .o_start_attn    (ctrl_start_attn),
        .i_attn_done     (ctrl_attn_done),
        .o_start_softmax (ctrl_start_softmax),
        .i_softmax_done  (ctrl_softmax_done),
        .i_y_we          (y_we),
        .i_y_addr        (y_addr),
        .i_y_data        (y_data),
        .o_result_ready  (result_ready_int),
        .o_state_debug   ()
    );

    // ─────────────────────────────────────────────────────────────────────
    // 4. Attention Top
    // ─────────────────────────────────────────────────────────────────────
    attention_top #(
        .DATA_WIDTH(DATA_WIDTH), .SEQ_LEN(SEQ_LEN)
    ) u_attn_top (
        .clk               (clk),
        .rst_n             (rst_n),
        .i_rx_ram_we       (ctrl_rx_ram_we),
        .i_rx_ram_addr     (ctrl_rx_ram_addr),
        .i_rx_ram_data     (ctrl_rx_ram_data),
        .i_start_linear    (ctrl_start_linear),
        .i_token_selection (ctrl_token_sel),
        .o_linear_done     (ctrl_linear_done),
        .i_start_attn      (ctrl_start_attn),
        .o_attn_done       (ctrl_attn_done),
        .i_start_softmax   (ctrl_start_softmax),
        .o_softmax_done    (ctrl_softmax_done),
        .o_softmax_sum     (),
        .o_z_ram_we        (y_we),
        .o_z_ram_wr_addr   (y_addr),
        .o_z_ram_data      (y_data)
    );

    // ─────────────────────────────────────────────────────────────────────
    // 5. HEX Display Controller
    // ─────────────────────────────────────────────────────────────────────
    hex_display_controller #(
        .CLK_FREQ(CLK_FREQ)
    ) u_hex_disp (
        .clk            (clk),
        .rst_n          (rst_n),
        .i_result_ready (result_ready_int),
        .i_y_we         (y_we),
        .i_y_addr       (y_addr),
        .i_y_data       (y_data),
        .i_key_next     (KEY[1]),
        .HEX0           (HEX0),
        .HEX1           (HEX1),
        .HEX2           (HEX2),
        .HEX3           (HEX3),
        .HEX4           (HEX4),
        .HEX5           (HEX5)
    );

endmodule